#include "autoaim/pipeline/frame_sync.hpp"
#include <algorithm>

namespace autoaim::pipeline {
FrameSync::FrameSync(std::size_t capacity, core::Seconds gap, core::Generation generation)
    : capacity_(capacity), maximum_gap_(gap), generation_(generation) {
  if (capacity < 2 || gap.value() <= 0)
    throw std::invalid_argument("Invalid pose history limits");
}

bool FrameSync::push(hal::GimbalFeedback feedback) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (feedback.generation != generation_)
    return false;

  if (!history_.empty() &&
      (feedback.sampled_at.domain() != history_.back().sampled_at.domain() ||
       core::elapsed(feedback.sampled_at, history_.back().sampled_at).value() <= 0))
    return false;

  history_.push_back(std::move(feedback));

  if (history_.size() > capacity_)
    history_.pop_front();

  return true;
}

std::optional<vision::AlignedPose> FrameSync::query(core::TimePoint time,
                                                    core::Generation generation) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (generation != generation_ || history_.empty() ||
      time.domain() != history_.front().sampled_at.domain())
    return std::nullopt;

  const auto after = std::lower_bound(history_.begin(), history_.end(), time,
                                      [](const auto& item, core::TimePoint stamp) {
                                        return item.sampled_at.nanoseconds() < stamp.nanoseconds();
                                      });

  if (after == history_.end())
    return std::nullopt;

  const auto before = core::same_time(after->sampled_at, time)
                          ? after
                          : (after == history_.begin() ? history_.end() : std::prev(after));

  if (before == history_.end() || !before->pose_valid || !after->pose_valid)
    return std::nullopt;

  const double gap = core::elapsed(after->sampled_at, before->sampled_at).value();

  if (gap > maximum_gap_.value())
    return std::nullopt;

  try {
    auto pose = [](const hal::GimbalFeedback& sample) {
      const auto& q = sample.quaternion_wxyz;

      return math::SE3(Eigen::Quaterniond(q[0], q[1], q[2], q[3]), Eigen::Vector3d::Zero());
    };

    const auto aligned =
        gap == 0 ? pose(*after)
                 : math::interpolate(pose(*before), pose(*after),
                                     core::elapsed(time, before->sampled_at).value() / gap);

    return vision::AlignedPose(time, before->sampled_at, after->sampled_at,
                               math::Transform<math::GimbalFrame, math::WorldFrame>(aligned), true);
  } catch (const std::invalid_argument&) {
    return std::nullopt;
  }
}

void FrameSync::reset(core::Generation generation) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (generation <= generation_)
    throw std::invalid_argument("Generation must advance");

  generation_ = generation;
  history_.clear();
}
} // namespace autoaim::pipeline
