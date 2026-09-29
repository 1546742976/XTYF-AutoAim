#include "autoaim/pipeline/offline_tools.hpp"
#include "session_annotation.hpp"
#include "autoaim/vision/annotation.hpp"
#include "autoaim/hal/session_writer.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>
#include <optional>

namespace autoaim::pipeline {
namespace {
namespace detail = annotation_detail;
namespace fs = std::filesystem;

// 仅本次导出借用源节点；排序指针而非 YAML::Node，避免其赋值的别名语义修改源节点。
class FeedbackIndex {
public:
  explicit FeedbackIndex(const detail::SessionSource& source) {
    std::vector<Entry> ordered;
    for (const auto& event : source.events) {
      if (event["kind"].as<std::string>() != "feedback")
        continue;
      const bool explicit_time = event["sampled_ns"].IsDefined();
      const auto time = event[explicit_time ? "sampled_ns" : "at_ns"].as<std::int64_t>();
      ordered.push_back({time, &event,
          explicit_time && event["pose_valid"].as<bool>() ? &event : nullptr});
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const Entry& a, const Entry& b) {
      return a.time < b.time;
    });
    for (const auto& entry : ordered) {
      if (entries_.empty() || entries_.back().time != entry.time)
        entries_.push_back(entry);
      else if (!entries_.back().exact)
        entries_.back().exact = entry.exact;
    }
  }

  YAML::Node reference(std::int64_t exposure) const {
    YAML::Node result;
    result["meaning"] = "raw feedback reference only; not synchronized pose or pose truth";
    result["before"] = YAML::Node(YAML::NodeType::Null);
    result["after"] = YAML::Node(YAML::NodeType::Null);
    result["exact_pose"] = YAML::Node(YAML::NodeType::Null);
    const auto next = std::lower_bound(entries_.begin(), entries_.end(), exposure,
        [](const Entry& entry, std::int64_t time) {
          return entry.time < time;
        });
    if (next != entries_.end())
      result["after"] = sample(*next->first, next->time);
    if (next != entries_.end() && next->time == exposure) {
      // 原导出器同一个反馈共用节点；保留别名以保持序列化字节，而不只保持值相等。
      result["before"] = result["after"];
      if (next->exact)
        result["exact_pose"] = next->exact == next->first ? result["after"] :
                                                         sample(*next->exact, next->time);
    } else if (next != entries_.begin()) {
      const auto& previous = *std::prev(next);
      result["before"] = sample(*previous.first, previous.time);
    }

    return result;
  }

private:
  struct Entry {
    std::int64_t time;
    const YAML::Node* first;
    const YAML::Node* exact;
  };

  static YAML::Node sample(const YAML::Node& event, std::int64_t time) {
    YAML::Node sample;
    sample["sampled_ns"] = time;
    sample["sample_time_source"] = event["sampled_ns"].IsDefined() ? "sampled_ns" :
                                                                 "replay_at_ns_fallback";
    sample["event"] = YAML::Clone(event);

    return sample;
  }

  std::vector<Entry> entries_;
};

void export_session(const detail::SessionSource& source, const fs::path& output) {
  detail::OutputDirectory pending(output);
  YAML::Node index, annotations;
  index["schema_version"] = 1;
  index["source_fingerprint"] = source.fingerprint;
  index["frames"] = YAML::Node(YAML::NodeType::Sequence);
  annotations["schema_version"] = 1;
  annotations["source_fingerprint"] = source.fingerprint;
  annotations["reviewed"] = false;
  annotations["frames"] = YAML::Node(YAML::NodeType::Sequence);
  const FeedbackIndex feedback(source);
  for (const auto& entry : source.images) {
    const auto& image = entry.second;
    YAML::Node frame, draft;
    frame["frame_id"] = entry.first;
    frame["path"] = image.relative_path.generic_string();
    frame["exported_path"] = (fs::path("frames") / image.relative_path).generic_string();
    frame["width"] = image.size.width;
    frame["height"] = image.size.height;
    for (const auto* key : {"generation", "exposure_ns", "received_ns", "time_origin",
                            "timing_sigma_s", "device_frame_id", "device_timestamp_ticks"})
      frame[key] = image.event[key].IsDefined() ? YAML::Clone(image.event[key]) :
                                                YAML::Node(YAML::NodeType::Null);
    frame["has_annotations"] =
        image.event["annotations"].IsDefined() && !image.event["annotations"].IsNull();
    frame["feedback_reference"] =
        feedback.reference(image.event["exposure_ns"].as<std::int64_t>());
    index["frames"].push_back(frame);
    draft["frame_id"] = entry.first;
    draft["annotations"] = image.event["annotations"].IsDefined() ?
                               YAML::Clone(image.event["annotations"]) :
                               YAML::Node(YAML::NodeType::Null);
    annotations["frames"].push_back(draft);
  }
  detail::copy_images(source, pending.path() / "frames");
  detail::write_yaml(pending.path() / "index.yaml", index);
  detail::write_yaml(pending.path() / "annotations.yaml", annotations);
  if (!detail::same_yaml(source.fingerprint, detail::dataset_fingerprint(source.manifest)))
    throw std::runtime_error("Source session changed during export");
  pending.publish();
}

void draw_review(const detail::SessionSource& source, core::FrameId id,
                  const YAML::Node& annotations, const fs::path& output) {
  const auto& entry = source.images.at(id);
  auto image = cv::imread((source.manifest.parent_path() / entry.relative_path).string());
  if (image.empty())
    throw std::runtime_error("Cannot decode review image");
  const auto labels = vision::read_annotations(annotations, entry.size);
  constexpr const char* names[] = {"TL", "TR", "BR", "BL"};
  for (std::size_t i = 0; i < labels.size(); ++i) {
    const auto& label = labels[i];
    std::vector<cv::Point> points;
    // 极远的不可见点也合法；只裁剪绘图坐标，避免浮点转 int 溢出，不改原始标注。
    for (const auto& point : label.corners)
      points.emplace_back(cvRound(std::clamp(double(point.x), -100000.0, 100000.0)),
                          cvRound(std::clamp(double(point.y), -100000.0, 100000.0)));
    cv::polylines(image, points, true, cv::Scalar(0, 255, 255), 1);
    for (std::size_t corner = 0; corner < 4; ++corner) {
      const auto& point = label.corners[corner];
      const std::string caption = std::to_string(i + 1) + ":" + names[corner] +
                                  (label.visible[corner] ? "" : " v=false");
      if (point.x >= 0 && point.y >= 0 && point.x < image.cols && point.y < image.rows) {
        cv::circle(image, points[corner], 3, cv::Scalar(0, 255, 0),
                    label.visible[corner] ? cv::FILLED : 1);
        int baseline = 0;
        const auto text_size = cv::getTextSize(caption, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1,
                                               &baseline);
        const int x_offset = corner == 0 || corner == 3 ? -text_size.width - 5 : 5;
        const int y_offset = corner < 2 ? -6 : text_size.height + 7;
        cv::Point location = points[corner] + cv::Point(x_offset, y_offset);
        location.x = std::clamp(location.x, 0, std::max(0, image.cols - text_size.width));
        location.y = std::clamp(location.y, std::min(text_size.height, image.rows - 1),
                                image.rows - 1);
        cv::putText(image, caption, location,
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
      } else {
        const auto text = caption + " outside (" + std::to_string(point.x) + "," +
                          std::to_string(point.y) + ")";
        cv::putText(image, text, {2, 12 + static_cast<int>((i * 4 + corner) * 13)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
      }
    }
  }
  if (!cv::imwrite(output.string(), image))
    throw std::runtime_error("Cannot write review image");
}

bool check_session(const detail::SessionSource& source, const fs::path& annotations,
                    const fs::path& output) {
  const auto checked = detail::check_annotations(source, annotations, source.fingerprint);
  detail::OutputDirectory pending(output);
  YAML::Node report;
  report["schema_version"] = 1;
  report["valid"] = checked.problems.size() == 0;
  report["source_matches"] = checked.source_matches;
  report["problems"] = checked.problems;
  report["valid_frames"] = checked.frames.size();
  detail::write_yaml(pending.path() / "check.yaml", report);
  if (checked.source_matches) {
    fs::create_directory(pending.path() / "frames");
    for (const auto& frame : checked.frames)
      draw_review(source, frame.first, frame.second,
                  pending.path() / "frames" / ("frame_" + std::to_string(frame.first) + ".png"));
  }
  if (!detail::same_yaml(source.fingerprint, detail::dataset_fingerprint(source.manifest)))
    throw std::runtime_error("Source session changed during review");
  pending.publish();
  for (const auto& issue : checked.problems)
    std::cerr << YAML::Dump(issue) << '\n';

  return checked.problems.size() == 0;
}

void flow_style(YAML::Node node) {
  if (node.IsMap()) {
    node.SetStyle(YAML::EmitterStyle::Flow);
    for (const auto& pair : node)
      flow_style(pair.second);
  } else if (node.IsSequence()) {
    node.SetStyle(YAML::EmitterStyle::Flow);
    for (const auto& value : node)
      flow_style(value);
  }
}

void apply_session(const detail::SessionSource& source, const fs::path& annotations,
                    const fs::path& output) {
  const auto submitted = detail::read_bytes(annotations);
  const auto checked = detail::check_annotations(source, annotations, source.fingerprint);
  if (checked.problems.size())
    throw std::invalid_argument(
        "Cannot apply invalid annotations: " + YAML::Dump(checked.problems));
  detail::OutputDirectory pending(output);
  detail::copy_images(source, pending.path());
  auto lines = source.lines;
  for (const auto& image : source.images) {
    auto event = YAML::Clone(image.second.event);
    event.remove("annotations");
    const auto found = checked.frames.find(image.first);
    if (found != checked.frames.end())
      event["annotations"] = YAML::Clone(found->second);
    // 保留原标量文本，不将其转换成 double 再输出；新人工 block 标注强制转为 flow。
    flow_style(event);
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter.SetFloatPrecision(9);
    emitter << YAML::Flow << event;
    if (!emitter.good())
      throw std::runtime_error("Cannot serialize annotated image event");
    const auto& original = source.lines.at(image.second.line);
    const auto prefix = original.substr(0, original.find_first_not_of(" \t"));
    const auto newline = original.size() >= 2 && original.substr(original.size() - 2) == "\r\n" ?
        "\r\n" : !original.empty() && original.back() == '\n' ? "\n" : "";
    lines[image.second.line] = prefix + "- " + emitter.c_str() + newline;
  }
  std::string events;
  for (const auto& line : lines)
    events += line;
  detail::write_bytes(pending.path() / "events.yaml", events);
  detail::write_bytes(pending.path() / "annotations.yaml", submitted);
  const auto copied = detail::read_session(pending.path());
  for (const auto& image : source.images) {
    auto original = YAML::Clone(image.second.event);
    auto updated = YAML::Clone(copied.images.at(image.first).event);
    original.remove("annotations");
    updated.remove("annotations");
    if (!detail::same_yaml(original, updated))
      throw std::runtime_error("Non-annotation image fields changed during apply");
  }
  YAML::Node provenance;
  provenance["schema_version"] = 1;
  provenance["reviewed"] = true;
  provenance["annotations_file"] = "annotations.yaml";
  provenance["source_fingerprint"] = source.fingerprint;
  provenance["output_fingerprint"] = copied.fingerprint;
  provenance["annotations_fingerprint"] =
      detail::file_fingerprint(pending.path() / "annotations.yaml", "annotations.yaml");
  detail::write_yaml(pending.path() / "annotation_provenance.yaml", provenance);
  detail::annotation_provenance(pending.path() / "events.yaml");
  if (!detail::same_yaml(source.fingerprint, detail::dataset_fingerprint(source.manifest)) ||
      submitted != detail::read_bytes(annotations))
    throw std::runtime_error("Input changed during apply");
  pending.publish();
}

void self_test() {
  detail::OutputDirectory sandbox(fs::temp_directory_path() / "autoaim-annotation-self-test");
  const auto root = sandbox.path();
  const core::TimePoint at(1, core::ClockDomain::replay);
  const auto pixels = std::make_shared<std::vector<std::uint8_t>>(16 * 16 * 3, 0);
  hal::SessionWriter writer(root / "source", {"synthetic-self-test", {}, {}});
  writer.append(at, std::make_shared<const core::CapturedFrame>(core::Stamp(1, 1, at),
      core::Image(16, 16, 48, pixels), at, core::TimeOrigin::synthetic, core::Seconds(0),
      core::Evidence::missing()), YAML::Load("[]"));
  writer.finish();
  const auto source = detail::read_session(root / "source");
  export_session(source, root / "export");
  auto labels = detail::read_document(root / "export/annotations.yaml");
  labels["reviewed"] = true;
  detail::write_yaml(root / "export/annotations.yaml", labels);
  if (!check_session(source, root / "export/annotations.yaml", root / "review"))
    throw std::runtime_error("Annotation self-test review failed");
  apply_session(source, root / "export/annotations.yaml", root / "applied");
  const auto binding = detail::annotation_provenance(root / "applied/events.yaml");
  if (!binding["annotations_reviewed"].as<bool>())
    throw std::runtime_error("Annotation self-test provenance failed");
  std::cout << "Annotation export/check/apply self-test passed (synthetic only)\n";
}
} // namespace

int run_annotate_session(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      self_test();

      return 0;
    }
    fs::path source_path, output, annotations;
    std::string action;
    for (int i = 1; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--help") {
        std::cout << "annotate_session --export NEW_DIR --session SOURCE\n"
                     "annotate_session --check --session SOURCE --annotations FILE "
                     "--output NEW_REVIEW_DIR\n"
                     "annotate_session --apply --session SOURCE --annotations FILE "
                     "--output NEW_SESSION_DIR\nannotate_session --self-test\n";

        return 0;
      }
      if (option == "--check" || option == "--apply") {
        if (!action.empty())
          throw std::invalid_argument("Choose exactly one annotation action");
        action = option == "--check" ? "check" : "apply";
        continue;
      }
      if (++i == argc)
        throw std::invalid_argument("Missing annotation argument");
      if (option == "--session")
        source_path = argv[i];
      else if (option == "--annotations")
        annotations = argv[i];
      else if (option == "--output" && output.empty())
        output = argv[i];
      else if (option == "--export" && action.empty() && output.empty()) {
        action = "export";
        output = argv[i];
      } else
        throw std::invalid_argument("Unknown/repeated annotation argument: " + option);
    }
    if (action.empty() || source_path.empty() || output.empty() ||
        (action != "export" && annotations.empty()) || (action == "export" && !annotations.empty()))
      throw std::invalid_argument(
          "Action/source/new output required; check/apply need annotations");
    const auto source = detail::read_session(source_path);
    if (detail::within(output, source.manifest.parent_path()))
      throw std::invalid_argument("Output must be outside the source session");
    if (action == "export")
      export_session(source, output);
    else if (action == "apply")
      apply_session(source, annotations, output);
    else {
      if (!check_session(source, annotations, output))
        return 1;
    }

    return 0;
  } catch (const std::exception& error) {
    std::cerr << "annotate_session: " << error.what() << '\n';

    return 1;
  }
}
} // namespace autoaim::pipeline
