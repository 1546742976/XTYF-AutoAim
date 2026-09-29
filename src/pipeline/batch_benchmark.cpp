#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/pipeline/pipeline.hpp"
#include "autoaim/pipeline/uart_writer.hpp"
#include "autoaim/hal/recording_transport.hpp"
#include "autoaim/pipeline/run_metadata.hpp"
#include "session_annotation.hpp"
#include "yaml_output.hpp"
#include "autoaim_build_information.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <streambuf>

namespace autoaim::pipeline {
namespace {
// 评测只使用完整写入计数，不消费命令文本。仍经 RecordingTransport 编码、
// 写出和计数，仅接收端不保留历史；普通文件记录端与失败锁存不变。
class DiscardBuffer final : public std::streambuf {
protected:
  std::streamsize xsputn(const char*, std::streamsize count) override {
    return count;
  }

  int_type overflow(int_type value) override {
    return traits_type::not_eof(value);
  }
};

YAML::Node ratio(double numerator, std::size_t denominator) {
  if (!denominator)
    return YAML::Node(YAML::NodeType::Null);

  return YAML::Node(numerator / denominator);
}

YAML::Node summarize(const vision::SequenceTotals& t) {
  YAML::Node node;
  node["frames"] = t.frames;
  node["unlabeled_frames"] = t.unlabeled_frames;
  node["negative_frames"] = t.negative_frames;
  node["processed_frames"] = t.processed_frames;
  node["rejected_or_dropped_frames"] = t.rejected_or_dropped_frames;
  node["expired_frames"] = t.expired_frames;
  node["frames_with_misses"] = t.frames_with_misses;
  const auto& d = t.detections;
  node["truths"] = d.truths;
  node["predictions"] = d.predictions;
  node["geometric_tp"] = d.matched;
  node["false_positives"] = d.predictions - d.matched;
  node["misses"] = d.truths - d.matched;
  node["precision"] = ratio(d.matched, d.predictions);
  node["recall"] = ratio(d.matched, d.truths);
  node["semantic_precision"] = ratio(d.semantic_matched, d.predictions);
  node["semantic_recall"] = ratio(d.semantic_matched, d.truths);
  node["usable_chain_recall"] = ratio(t.usable_matches, d.truths);
  node["category_errors"] = d.category_errors;
  node["color_errors"] = d.color_errors;
  node["size_errors"] = d.size_errors;
  node["visible_corners"] = d.visible_corners;
  node["mean_corner_error_px"] = ratio(d.corner_error_sum_px, d.visible_corners);
  node["maximum_corner_error_px"] = d.visible_corners ? YAML::Node(d.maximum_corner_error_px) :
                                                        YAML::Node(YAML::NodeType::Null);
  node["corner_rms_px"] = d.visible_corners ? YAML::Node(
      std::sqrt(d.corner_error_squared_px / d.visible_corners)) : YAML::Node(YAML::NodeType::Null);
  node["pose_pairs"] = d.pose_pairs;
  node["mean_position_error_m"] = ratio(d.position_error_sum_m, d.pose_pairs);
  node["mean_rotation_error_rad"] = ratio(d.rotation_error_sum_rad, d.pose_pairs);

  return node;
}

using AnnotationTruth = std::optional<std::vector<vision::ArmorAnnotation>>;

struct ImageFact {
  core::FrameId id;
  core::TimePoint exposure;
  core::TimePoint at;
  cv::Size size;
  AnnotationTruth truth;
};

// 只保存标注和时间/操作输入事实，不持有解码像素；构建完成后不再修改。
using DatasetEvent = std::variant<std::optional<hal::OperatorInput>, hal::ButtonSample, ImageFact>;
using DatasetFacts = std::vector<DatasetEvent>;

struct LabeledFrame {
  const AnnotationTruth* truth; // 借用本次比较的不可变事实，寿命覆盖各配置运行。
  vision::InterventionGroup intervention;
  bool expired_on_arrival;
  std::vector<vision::EvaluatedDetection> predictions;
  vision::EvaluationFrameState state{false, false, false};
};

DatasetFacts read_facts(const std::filesystem::path& manifest, vision::TeamColor enemy) {
  hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
  hal::FileReplay input(manifest, clock);
  DatasetFacts facts;

  for (;;) {
    auto event = input.next(1);
    if (!event)
      throw std::invalid_argument(event.error().message);
    if (std::holds_alternative<hal::ReplayEnd>(event.value()))
      break;
    if (const auto* feedback = std::get_if<hal::GimbalFeedback>(&event.value()))
      facts.emplace_back(feedback->operator_input);
    if (const auto* key = std::get_if<hal::ButtonSample>(&event.value()))
      facts.emplace_back(*key);

    const auto* image = std::get_if<std::shared_ptr<const core::CapturedFrame>>(&event.value());
    if (!image)
      continue;

    const auto& frame = **image;
    const auto node = input.annotations(frame.stamp.frame_id);
    AnnotationTruth truth;
    if (node.IsDefined() && !node.IsNull()) {
      truth = vision::read_annotations(node, {frame.image.width, frame.image.height});
      truth->erase(std::remove_if(truth->begin(), truth->end(), [&](const auto& item) {
        return item.label.color != enemy;
      }), truth->end());
    }
    facts.emplace_back(ImageFact{frame.stamp.frame_id, frame.stamp.exposure, clock.now(),
                                  {frame.image.width, frame.image.height}, std::move(truth)});
  }

  return facts;
}

std::map<core::FrameId, LabeledFrame> labels(const DatasetFacts& facts,
                                          const PipelineConfig& config) {
  std::optional<hal::OperatorInput> op;
  std::map<core::FrameId, LabeledFrame> frames;

  for (const auto& event : facts) {
    if (const auto* feedback = std::get_if<std::optional<hal::OperatorInput>>(&event);
        feedback && !config.independent_button_input)
      op = *feedback;
    if (const auto* key = std::get_if<hal::ButtonSample>(&event);
        key && config.independent_button_input)
      op = hal::OperatorInput{key->sampled_at, key->valid, key->intervening, 0,
                              core::Evidence::missing()};

    const auto* frame = std::get_if<ImageFact>(&event);
    if (!frame)
      continue;

    // 有效期和输入来源属于配置，不能缓存第一次运行算出的分层或过期状态。
    auto group = vision::InterventionGroup::unknown;
    if (op && op->valid &&
        core::fresh(op->sampled_at, frame->at, config.infantry.input_maximum_age))
      group = op->intervening ? vision::InterventionGroup::manual : vision::InterventionGroup::none;
    const bool expired = !core::fresh(frame->exposure, frame->at, config.queue.maximum_age);
    frames.emplace(frame->id, LabeledFrame{&frame->truth, group, expired, {}});
  }

  return frames;
}

} // namespace

int run_batch_benchmark(int argc, char** argv) {
  try {
    std::filesystem::path dataset, output;
    std::vector<std::filesystem::path> configurations;
    vision::EvaluationOptions options{0};
    bool have_iou = false;
    std::optional<std::string> reference;
    std::optional<double> position_limit, rotation_limit;

    for (int i = 1; i < argc; ++i) {
      const std::string key(argv[i]);
      if (++i == argc)
        throw std::invalid_argument("Missing batch benchmark argument");
      const std::string value(argv[i]);
      if (key == "--config")
        configurations.emplace_back(value);
      else if (key == "--dataset")
        dataset = value;
      else if (key == "--output")
        output = value;
      else if (key == "--iou") {
        std::size_t consumed = 0;
        options.minimum_iou = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(options.minimum_iou) ||
            options.minimum_iou <= 0 || options.minimum_iou > 1)
          throw std::invalid_argument("--iou must be a complete finite number in (0,1]");
        have_iou = true;
      }
      else if (key == "--pose-reference")
        reference = value;
      else if (key == "--pose-position-limit-m" || key == "--pose-rotation-limit-rad") {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(parsed) || parsed < 0)
          throw std::invalid_argument(key + " must be a complete finite nonnegative number");
        (key == "--pose-position-limit-m" ? position_limit : rotation_limit) = parsed;
      }
      else
        throw std::invalid_argument("Unknown batch argument: " + key);
    }

    if (!have_iou)
      throw std::invalid_argument("Missing required --iou in (0,1]");
    if (configurations.empty() || dataset.empty() || output.empty() ||
        std::filesystem::exists(output))
      throw std::invalid_argument("Configurations/dataset/new output directory required");
    if (reference || position_limit || rotation_limit) {
      if (!reference || !position_limit || !rotation_limit)
        throw std::invalid_argument("Pose reference and both uncertainty limits must be explicit");
      options.pose_truth = vision::PoseTruthLimits{*reference, *position_limit, *rotation_limit};
    }
    vision::SequenceEvaluation check(options);
    YAML::Node report, costs;
    report["report_schema_version"] = 1;
    report["command_line"] = std::vector<std::string>(argv, argv + argc);
    report["working_directory"] = std::filesystem::current_path().string();
    report["build"] = YAML::Load(autoaim_build_information);
    const auto provenance = annotation_detail::annotation_provenance(dataset);
    for (const auto& entry : provenance)
      report[entry.first.as<std::string>()] = YAML::Clone(entry.second);
    const auto dataset_content = annotation_detail::dataset_fingerprint(dataset);
    report["dataset_fingerprint"] = dataset_content;
    report["pose_metrics"] = reference ? "conditional_on_accepted_reference" : "not_produced";
    report["dataset"] = std::filesystem::absolute(dataset).string();
    report["minimum_iou"] = options.minimum_iou;
    report["matching"] = "axis-aligned enclosure IoU; descending IoU then truth/prediction index";
    report["truth_scope"] = "configured enemy color only; unlabeled frames excluded from P/R";
    report["pose_reference"] =
        reference ? YAML::Node(*reference) : YAML::Node(YAML::NodeType::Null);
    std::optional<vision::TeamColor> common_enemy;
    std::optional<DatasetFacts> facts;

    for (const auto& path : configurations) {
      auto loaded = load_pipeline_config(path);
      if (!loaded)
        throw std::invalid_argument(loaded.error().message);
      auto config = std::move(loaded).value();
      config.input_manifest = dataset;
      const auto enemy = std::visit([](const auto& detector) {
        return detector.enemy;
      }, config.detector);
      if (common_enemy && enemy != *common_enemy)
        throw std::invalid_argument("Compared configurations must select the same enemy color");
      common_enemy = enemy;
      if (!facts)
        facts = read_facts(config.input_manifest, enemy);
      auto frames = labels(*facts, config);
      if (frames.empty())
        throw std::invalid_argument("Benchmark dataset has no images");

      auto run = describe_run(path, config);
      run["configuration_path"] = std::filesystem::absolute(path).string();
      if (const auto* yolo = std::get_if<vision::YoloOptions>(&config.detector))
        run["model_path"] = yolo->model_path;

      const auto begin = std::chrono::steady_clock::now();
      auto clock =
          std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
      DiscardBuffer command_buffer;
      std::ostream command_stream(&command_buffer);
      hal::RecordingTransport transport(command_stream, *clock);
      Pipeline pipeline(std::move(config), std::move(clock),
          make_uart14_writer(transport, core::Seconds(0.1)),
          control::PublishSchedule::replay_events, {},
          [&](const vision::FramePacket& packet,
              const std::vector<vision::EvaluatedDetection>& predictions,
              vision::EvaluationFrameState state) {
            auto& frame = frames.at(packet.frame.stamp.frame_id);
            frame.predictions = predictions;
            frame.state = state;
          });
      const auto prepared = std::chrono::steady_clock::now();
      const auto result = pipeline.run_file();
      const auto end = std::chrono::steady_clock::now();
      if (!result)
        throw std::runtime_error(result.error().message);

      vision::SequenceEvaluation evaluation(options);
      for (const auto& entry : frames) {
        auto state = entry.second.state;
        state.expired = state.expired || entry.second.expired_on_arrival;
        evaluation.add(entry.first, *entry.second.truth, entry.second.predictions, state,
                        entry.second.intervention);
      }
      run["summary"] = summarize(evaluation.totals());
      constexpr const char* names[] = {"unknown", "manual", "none"};
      for (std::size_t i = 0; i < 3; ++i)
        run["by_intervention"][names[i]] = summarize(evaluation.by_intervention()[i]);
      run["maximum_empty_detection_streak"] = evaluation.maximum_empty_detection_streak();
      run["maximum_track_loss_streak"] = evaluation.maximum_track_loss_streak();
      const auto& metrics = result.value();
      run["queue_drops"] =
          std::vector<std::uint64_t>(metrics.queue_drops.begin(), metrics.queue_drops.end());
      run["result_drops"] =
          std::vector<std::uint64_t>(metrics.result_drops.begin(), metrics.result_drops.end());
      run["drop_reason_order"] = std::vector<std::string>{"invalid", "expired", "old_generation",
          "capacity", "no_buffer", "out_of_order", "closed"};
      run["command_writes"] = transport.completed_writes();
      run["command_format"] = "uart14-recording";
      run["maximum_observation_age_s"] = metrics.maximum_observation_age_s;
      const auto published = pipeline.publisher_status();
      run["last_first_complete_write_age_s"] = published.last_first_send_age ?
          YAML::Node(published.last_first_send_age->value()) : YAML::Node(YAML::NodeType::Null);
      report["runs"].push_back(run);

      auto measured = timing_report(metrics);
      measured["configuration_path"] = std::filesystem::absolute(path).string();
      measured["initialization_ms"] =
          std::chrono::duration<double, std::milli>(prepared - begin).count();
      measured["replay_wall_ms"] =
          std::chrono::duration<double, std::milli>(end - prepared).count();
      costs["runs"].push_back(measured);
    }

    if (!annotation_detail::same_yaml(dataset_content,
                                      annotation_detail::dataset_fingerprint(dataset)))
      throw std::runtime_error("Dataset changed during benchmark");
    if (!std::filesystem::create_directory(output))
      throw std::invalid_argument("Evaluation output directory already exists");
    yaml_detail::write_text(output / "report.yaml", report, "Cannot write evaluation report");
    yaml_detail::write_text(output / "timing.yaml", costs, "Cannot write evaluation report");
    std::cout << "Compared " << configurations.size() << " configurations; report="
              << (output / "report.yaml").string()
              << "\nLocal measurements only; not NUC acceptance.\n";

    return 0;
  } catch (const std::exception& error) {
    std::cerr << "batch benchmark: " << error.what() << '\n';

    return 1;
  }
}
} // namespace autoaim::pipeline
