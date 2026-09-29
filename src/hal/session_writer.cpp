#include "autoaim/hal/session_writer.hpp"
#include <opencv2/imgcodecs.hpp>

namespace autoaim::hal {
namespace {
void write_item(std::ostream& output, const YAML::Node& node) {
  YAML::Emitter emitter;
  // 回放必须保留原始浮点值，不能因文本默认精度改变后续解算输入。
  emitter.SetDoublePrecision(17);
  emitter.SetFloatPrecision(9);
  emitter << YAML::Flow << node;
  output << "  - " << emitter.c_str() << '\n';
  output.flush();

  if (!emitter.good() || !output)
    throw std::runtime_error("Session write failed");
}

void timestamp_before(core::TimePoint sampled, core::TimePoint at) {
  if (sampled.domain() != at.domain() || sampled.nanoseconds() > at.nanoseconds())
    throw std::invalid_argument("Session sample time differs from event order/domain");
}
} // namespace

SessionWriter::SessionWriter(const std::filesystem::path& directory,
                             const SessionMetadata& metadata) : directory_(directory) {
  if (!std::filesystem::create_directory(directory_))
    throw std::invalid_argument("Session directory already exists");

  events_.open(directory_ / "events.yaml");
  derived_.open(directory_ / "derived.yaml");

  if (!events_ || !derived_)
    throw std::runtime_error("Cannot create session files");

  YAML::Node session(YAML::NodeType::Map);

  if (metadata.device_id)
    session["device_id"] = *metadata.device_id;
  if (metadata.configuration_id)
    session["configuration_id"] = *metadata.configuration_id;
  if (metadata.model_id)
    session["model_id"] = *metadata.model_id;

  YAML::Emitter emitter;
  emitter << YAML::Flow << session;
  events_ << "schema_version: 2\ndomain: replay\nsession: " << emitter.c_str() << '\n';
  events_.flush();
  derived_ << "schema_version: 2\nresults:\n";
}

void SessionWriter::check_time(core::TimePoint at) const {
  if (finished_ || failed_)
    throw std::logic_error("Session writer is closed or failed");
  if (at.domain() != core::ClockDomain::replay || at.nanoseconds() < previous_ns_)
    throw std::invalid_argument("Session time must be monotonic in replay domain");
}

void SessionWriter::append(core::TimePoint at, const ReplayEvent& event,
                            const YAML::Node& annotations) {
  std::lock_guard<std::mutex> lock(mutex_);
  check_time(at);

  try {
    YAML::Node node;
    node["at_ns"] = at.nanoseconds();

    if (const auto* sample = std::get_if<GimbalFeedback>(&event)) {
      timestamp_before(sample->sampled_at, at);
      node["kind"] = "feedback";
      node["sampled_ns"] = sample->sampled_at.nanoseconds();
      node["quaternion_wxyz"] = std::vector<double>(sample->quaternion_wxyz.begin(),
                                                    sample->quaternion_wxyz.end());
      node["yaw_rad"] = sample->yaw_rad;
      node["pitch_rad"] = sample->pitch_rad;
      node["bullet_speed_mps"] = sample->bullet_speed_mps;
      node["pose_valid"] = sample->pose_valid;
      node["status_valid"] = sample->status_valid;

      if (sample->operator_input) {
        const auto& op = *sample->operator_input;
        timestamp_before(op.sampled_at, at);
        node["operator"]["sampled_ns"] = op.sampled_at.nanoseconds();
        node["operator"]["valid"] = op.valid;
        node["operator"]["intervening"] = op.intervening;
        node["operator"]["enable_event"] = op.enable_event;
      }
    } else if (const auto* uart = std::get_if<UartChunk>(&event)) {
      timestamp_before(uart->received_at, at);
      node["kind"] = "uart";
      node["received_ns"] = uart->received_at.nanoseconds();
      node["bytes"] = std::vector<unsigned>(uart->bytes.begin(), uart->bytes.end());
    } else if (const auto* button = std::get_if<ButtonSample>(&event)) {
      timestamp_before(button->sampled_at, at);
      node["kind"] = "button";
      node["sampled_ns"] = button->sampled_at.nanoseconds();
      node["valid"] = button->valid;
      node["intervening"] = button->intervening;
      node["pressed"] = button->pressed;
    } else if (const auto* image =
                   std::get_if<std::shared_ptr<const core::CapturedFrame>>(&event)) {
      if (!*image || (*image)->stamp.frame_id <= previous_frame_)
        throw std::invalid_argument("Missing image or repeated frame id");

      const auto& frame = **image;
      timestamp_before(frame.received_at, at);
      timestamp_before(frame.stamp.exposure, at);
      const auto filename = "frame_" + std::to_string(frame.stamp.frame_id) + ".png";
      const cv::Mat pixels(frame.image.height, frame.image.width, CV_8UC3,
                           const_cast<std::uint8_t*>(frame.image.pixels->data()),
                           frame.image.stride);

      if (!cv::imwrite((directory_ / filename).string(), pixels))
        throw std::runtime_error("Cannot write session image");

      node["kind"] = "image";
      node["path"] = filename;
      node["frame_id"] = frame.stamp.frame_id;
      node["generation"] = frame.stamp.generation;
      node["exposure_ns"] = frame.stamp.exposure.nanoseconds();
      node["received_ns"] = frame.received_at.nanoseconds();
      node["time_origin"] = frame.time_origin == core::TimeOrigin::synthetic ? "synthetic" :
                            frame.time_origin == core::TimeOrigin::hardware_mapped ?
                                "hardware_mapped" : "receipt_estimated";
      node["timing_sigma_s"] = frame.timing_uncertainty.value();

      if (frame.capture.device_frame_id)
        node["device_frame_id"] = *frame.capture.device_frame_id;
      if (frame.capture.device_timestamp_ticks)
        node["device_timestamp_ticks"] = *frame.capture.device_timestamp_ticks;
      if (frame.capture.device_dropped_frames)
        node["device_dropped_frames"] = *frame.capture.device_dropped_frames;
      if (frame.capture.pixel_format)
        node["pixel_format"] = *frame.capture.pixel_format;
      if (frame.capture.roi_offset)
        node["roi_offset"] = std::vector<int>{(*frame.capture.roi_offset)[0],
                                              (*frame.capture.roi_offset)[1]};
      if (frame.capture.device_frame_discontinuity)
        node["device_frame_discontinuity"] = *frame.capture.device_frame_discontinuity;
      if (annotations.IsDefined() && !annotations.IsNull())
        node["annotations"] = YAML::Clone(annotations);

      previous_frame_ = frame.stamp.frame_id;
    } else
      throw std::invalid_argument("ReplayEnd is not a record; call finish explicitly");

    if (!have_events_)
      events_ << "events:\n";
    write_item(events_, node);
    have_events_ = true;
    previous_ns_ = at.nanoseconds();
  } catch (...) {
    failed_ = true;
    throw;
  }
}

void SessionWriter::derived(core::TimePoint at, const std::string& kind,
                            const YAML::Node& values) {
  std::lock_guard<std::mutex> lock(mutex_);
  check_time(at);

  if (kind != "command" && kind != "drop" && kind != "timing" && kind != "run")
    throw std::invalid_argument("Unknown derived record kind");

  try {
    YAML::Node node;
    node["at_ns"] = at.nanoseconds();
    node["kind"] = kind;
    node["values"] = YAML::Clone(values);
    write_item(derived_, node);
  } catch (...) {
    failed_ = true;
    throw;
  }
}

void SessionWriter::finish() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (finished_ || failed_)
    throw std::logic_error("Session already closed or failed");

  derived_ << "complete: true\n";
  derived_.flush();

  if (!derived_)
    throw std::runtime_error("Derived results flush failed");
  if (!have_events_)
    events_ << "events: []\n";

  events_ << "complete: true\n";
  events_.flush();

  if (!events_)
    throw std::runtime_error("Session flush failed");
  finished_ = true;
}
} // namespace autoaim::hal
