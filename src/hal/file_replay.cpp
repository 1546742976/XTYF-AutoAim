#include "autoaim/hal/file_replay.hpp"
#include "autoaim/core/config.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cstring>
#include <unordered_map>

namespace autoaim::hal {
struct FileReplay::Impl {
  std::filesystem::path base;
  ReplayClock& clock;
  std::vector<YAML::Node> events;
  std::size_t cursor = 0;
  std::size_t annotation_cursor = 0;
  std::unordered_map<core::FrameId, std::size_t> annotation_index;
  core::FrameId previous_frame = 0;
  bool failed = false;
  SessionMetadata metadata;

  Impl(std::filesystem::path parent, ReplayClock& source) : base(std::move(parent)), clock(source) {
  }
};

FileReplay::FileReplay(const std::filesystem::path& manifest, ReplayClock& clock)
    : impl_(std::make_unique<Impl>(manifest.parent_path(), clock)) {
  auto config = core::Config::load(manifest);

  if (!config)
    throw std::invalid_argument(config.error().message);

  if (config.value().require<std::string>("domain") != "replay")
    throw std::invalid_argument("File replay requires replay domain");

  // 复用已校验的节点；第二次读文件既重复解析，也可能读到另一版清单。
  const auto field = [&](const char* key) {
    return config.value().contains(key) ? config.value().require<YAML::Node>(key)
                                        : YAML::Node(YAML::NodeType::Undefined);
  };

  if (const auto version = field("schema_version")) {
    const auto complete = field("complete");

    if (version.as<int>() != 2 || !complete || !complete.as<bool>())
      throw std::invalid_argument("Unsupported or incomplete session");
  }

  if (const auto session = field("session")) {
    if (session["device_id"])
      impl_->metadata.device_id = session["device_id"].as<std::string>();
    if (session["configuration_id"])
      impl_->metadata.configuration_id = session["configuration_id"].as<std::string>();
    if (session["model_id"])
      impl_->metadata.model_id = session["model_id"].as<std::string>();
  }

  impl_->events = config.value().require<std::vector<YAML::Node>>("events");
  auto previous = clock.now().nanoseconds();

  for (const auto& event : impl_->events) {
    const auto at = event["at_ns"].as<std::int64_t>();
    const auto kind = event["kind"].as<std::string>();

    if (at < previous || (kind != "image" && kind != "feedback" && kind != "uart" &&
                          kind != "button"))
      throw std::invalid_argument("Replay event order/type invalid");

    previous = at;
  }
}

FileReplay::~FileReplay() = default;

const SessionMetadata& FileReplay::metadata() const noexcept {
  return impl_->metadata;
}

YAML::Node FileReplay::annotations(core::FrameId frame_id) const {
  auto& self = *impl_;
  const auto copy = [&](std::size_t index) {
    const auto& event = self.events[index];

    return YAML::Clone(event["annotations"]);
  };

  if (const auto found = self.annotation_index.find(frame_id);
      found != self.annotation_index.end())
    return copy(found->second);

  // 独立于 next() 的消费游标。遇到非法帧号不越过它，保留原查询异常顺序；
  // emplace 保留首次匹配，重复帧是否可消费仍由 next() 判定。
  while (self.annotation_cursor < self.events.size()) {
    const auto index = self.annotation_cursor;
    const auto& event = self.events[index];

    if (event["kind"].as<std::string>() == "image") {
      const auto id = event["frame_id"].as<core::FrameId>();
      self.annotation_index.emplace(id, index);
      ++self.annotation_cursor;

      if (id == frame_id)
        return copy(index);
    } else
      ++self.annotation_cursor;
  }

  return {};
}

core::Result<ReplayEvent> FileReplay::next(core::Generation generation) {
  using Result = core::Result<ReplayEvent>;
  auto& self = *impl_;

  if (self.failed)
    return Result::failure(core::ErrorCode::fault, "Replay input fault latched");

  if (self.cursor == self.events.size())
    return Result::success(ReplayEnd{});

  try {
    const auto event = self.events[self.cursor++];
    const core::TimePoint at(event["at_ns"].as<std::int64_t>(), core::ClockDomain::replay);
    self.clock.advance_to(at);
    const auto sample_time = [&](const YAML::Node& node, const char* key) {
      const auto value = node[key] ? node[key].as<std::int64_t>() : at.nanoseconds();

      if (value > at.nanoseconds())
        throw std::invalid_argument("Sample time after replay event");

      return core::TimePoint(value, at.domain());
    };

    if (event["kind"].as<std::string>() == "uart") {
      const auto values = event["bytes"].as<std::vector<unsigned>>();
      std::vector<std::uint8_t> bytes;

      for (auto value : values) {
        if (value > 255)
          throw std::invalid_argument("UART byte outside [0,255]");
        bytes.push_back(static_cast<std::uint8_t>(value));
      }

      return Result::success(UartChunk{sample_time(event, "received_ns"), std::move(bytes)});
    }

    if (event["kind"].as<std::string>() == "button")
      return Result::success(ButtonSample{sample_time(event, "sampled_ns"),
                                           event["valid"].as<bool>(),
                                           event["intervening"].as<bool>(),
                                           event["pressed"].as<bool>()});

    if (event["kind"].as<std::string>() == "feedback") {
      const auto q = event["quaternion_wxyz"].as<std::vector<double>>();

      if (q.size() != 4)
        throw std::invalid_argument("Replay quaternion requires four components");

      std::optional<OperatorInput> input;

      if (event["operator"]) {
        const auto op = event["operator"];
        input.emplace(OperatorInput{sample_time(op, "sampled_ns"), op["valid"].as<bool>(),
                                    op["intervening"].as<bool>(),
                                    op["enable_event"].as<std::uint64_t>(),
                                    core::Evidence::declared()});
      }

      return Result::success(GimbalFeedback{sample_time(event, "sampled_ns"),
                                            generation,
                                            {q[0], q[1], q[2], q[3]},
                                            event["yaw_rad"].as<double>(),
                                            event["pitch_rad"].as<double>(),
                                            event["bullet_speed_mps"].as<double>(),
                                            event["pose_valid"].as<bool>(),
                                            event["status_valid"].as<bool>(),
                                            input});
    }

    const auto id = event["frame_id"].as<core::FrameId>();

    if (id <= self.previous_frame)
      throw std::invalid_argument("Replay frame id must advance");

    const core::Stamp source(
        id, generation,
        core::TimePoint(event["exposure_ns"].as<std::int64_t>(), core::ClockDomain::replay));

    const auto image_path = self.base / event["path"].as<std::string>();

    // imread 是本地文件读取，不调用 VideoCapture，避免设备索引或网络 URL 混入。
    const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);

    if (image.empty())
      throw std::runtime_error("Cannot decode replay image: " + image_path.string());

    const auto stride = static_cast<std::size_t>(image.cols) * 3;
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(stride * image.rows);

    for (int row = 0; row < image.rows; ++row)
      std::memcpy(pixels->data() + row * stride, image.ptr(row), stride);

    const auto origin = event["time_origin"].as<std::string>();

    if (origin != "receipt_estimated" && origin != "synthetic" && origin != "hardware_mapped")
      throw std::invalid_argument("Unknown replay time origin");

    core::CaptureMetadata capture;

    if (event["device_frame_id"])
      capture.device_frame_id = event["device_frame_id"].as<std::uint64_t>();
    if (event["device_timestamp_ticks"])
      capture.device_timestamp_ticks = event["device_timestamp_ticks"].as<std::uint64_t>();
    if (event["device_dropped_frames"])
      capture.device_dropped_frames = event["device_dropped_frames"].as<std::uint64_t>();
    if (event["pixel_format"])
      capture.pixel_format = event["pixel_format"].as<std::string>();
    if (event["roi_offset"]) {
      const auto offset = event["roi_offset"].as<std::vector<int>>();

      if (offset.size() != 2)
        throw std::invalid_argument("Frame ROI offset requires x,y");
      capture.roi_offset = std::array<int, 2>{offset[0], offset[1]};
    }
    if (event["device_frame_discontinuity"])
      capture.device_frame_discontinuity = event["device_frame_discontinuity"].as<bool>();

    const auto received =
        event["received_ns"]
            ? core::TimePoint(event["received_ns"].as<std::int64_t>(), at.domain())
            : at;

    if (received.nanoseconds() > at.nanoseconds())
      throw std::invalid_argument("Frame received after replay event");

    const auto frame = std::make_shared<const core::CapturedFrame>(
        source, core::Image(image.cols, image.rows, stride, pixels), received,
        origin == "synthetic"         ? core::TimeOrigin::synthetic
        : origin == "hardware_mapped" ? core::TimeOrigin::hardware_mapped
                                      : core::TimeOrigin::receipt_estimated,
        core::Seconds(event["timing_sigma_s"].as<double>()), core::Evidence::declared(), capture);

    self.previous_frame = id;

    return Result::success(frame);
  } catch (const std::exception& error) {
    self.failed = true;

    return Result::failure(core::ErrorCode::io, error.what());
  }
}
} // namespace autoaim::hal
