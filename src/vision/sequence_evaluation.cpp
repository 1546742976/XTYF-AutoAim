#include "autoaim/vision/evaluation.hpp"
#include <algorithm>

namespace autoaim::vision {
namespace {
void accumulate(SequenceTotals& total, const std::optional<FrameEvaluation>& frame,
                 EvaluationFrameState state) {
  ++total.frames;
  total.processed_frames += state.detector_ran;
  total.rejected_or_dropped_frames += !state.accepted_by_pipeline;
  total.expired_frames += state.expired;
  if (!frame) {
    ++total.unlabeled_frames;

    return;
  }

  total.negative_frames += frame->truths == 0;
  total.frames_with_misses += frame->matched < frame->truths;
  if (state.accepted_by_pipeline)
    total.usable_matches += frame->matched;

  for (auto field : {&FrameEvaluation::truths, &FrameEvaluation::predictions,
       &FrameEvaluation::matched, &FrameEvaluation::semantic_matched,
       &FrameEvaluation::category_errors, &FrameEvaluation::color_errors,
       &FrameEvaluation::size_errors, &FrameEvaluation::visible_corners,
       &FrameEvaluation::pose_pairs})
    total.detections.*field += (*frame).*field;
  for (auto field : {&FrameEvaluation::corner_error_sum_px,
       &FrameEvaluation::corner_error_squared_px,
       &FrameEvaluation::position_error_sum_m, &FrameEvaluation::rotation_error_sum_rad})
    total.detections.*field += (*frame).*field;
  total.detections.maximum_corner_error_px =
      std::max(total.detections.maximum_corner_error_px, frame->maximum_corner_error_px);
}
} // namespace

SequenceEvaluation::SequenceEvaluation(EvaluationOptions options) : options_(std::move(options)) {
  evaluate_frame({}, {}, options_);
}

void SequenceEvaluation::add(core::FrameId id,
    const std::optional<std::vector<ArmorAnnotation>>& truth,
    const std::vector<EvaluatedDetection>& predictions, EvaluationFrameState state,
    InterventionGroup intervention) {
  const auto group = static_cast<std::size_t>(intervention);
  if (id <= last_id_ || group >= groups_.size() ||
      (!state.detector_ran && (!predictions.empty() || state.accepted_by_pipeline)) ||
      (state.expired && state.accepted_by_pipeline))
    throw std::invalid_argument("Invalid evaluation frame order/state/group");

  std::optional<FrameEvaluation> frame;
  if (truth)
    frame = evaluate_frame(*truth, predictions, options_);
  last_id_ = id;
  accumulate(total_, frame, state);
  accumulate(groups_[group], frame, state);

  if (!truth) {
    empty_streak_ = 0;
    tracks_.clear();

    return;
  }

  empty_streak_ = frame->truths && frame->matched == 0 ? empty_streak_ + 1 : 0;
  maximum_empty_streak_ = std::max(maximum_empty_streak_, empty_streak_);
  std::map<std::string, std::size_t> current;
  for (std::size_t i = 0; i < truth->size(); ++i) {
    const auto& track = (*truth)[i].track_id;
    if (track.empty())
      continue;
    if (current.count(track))
      throw std::invalid_argument("Repeated evaluation track identity");
    const auto found = tracks_.find(track);
    const auto previous = found == tracks_.end() ? 0 : found->second;
    const auto missing = frame->truth_matched[i] ? 0 : previous + 1;
    current.emplace(track, missing);
    maximum_track_streak_ = std::max(maximum_track_streak_, missing);
  }
  tracks_ = std::move(current);
}

const SequenceTotals& SequenceEvaluation::totals() const noexcept {
  return total_;
}

const std::array<SequenceTotals, 3>& SequenceEvaluation::by_intervention() const noexcept {
  return groups_;
}

std::size_t SequenceEvaluation::maximum_empty_detection_streak() const noexcept {
  return maximum_empty_streak_;
}

std::size_t SequenceEvaluation::maximum_track_loss_streak() const noexcept {
  return maximum_track_streak_;
}
} // namespace autoaim::vision
