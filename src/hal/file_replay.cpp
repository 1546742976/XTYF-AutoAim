#include "autoaim/hal/file_replay.hpp"
#include "autoaim/core/config.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cstring>

namespace autoaim::hal {
struct FileReplay::Impl {
  std::filesystem::path base;
  ReplayClock& clock;
  std::vector<YAML::Node> events;
  std::size_t cursor = 0;
  core::FrameId previous_frame = 0;
  bool failed = false;
  Impl(std::filesystem::path parent, ReplayClock& source) : base(std::move(parent)), clock(source) {}
};
FileReplay::FileReplay(const std::filesystem::path& manifest, ReplayClock& clock)
    : impl_(std::make_unique<Impl>(manifest.parent_path(), clock)) {
  auto config = core::Config::load(manifest);
  if (!config) throw std::invalid_argument(config.error().message);
  if (config.value().require<std::string>("domain") != "replay")
    throw std::invalid_argument("File replay requires replay domain");
  impl_->events = config.value().require<std::vector<YAML::Node>>("events");
  auto previous = clock.now().nanoseconds();
  for (const auto& event : impl_->events) {
    const auto at = event["at_ns"].as<std::int64_t>();
    const auto kind = event["kind"].as<std::string>();
    if (at < previous || (kind != "image" && kind != "feedback"))
      throw std::invalid_argument("Replay event order/type invalid");
    previous = at;
  }
}
FileReplay::~FileReplay() = default;
core::Result<ReplayEvent> FileReplay::next(core::Generation generation) {
  using Result = core::Result<ReplayEvent>;
  auto& self = *impl_;
  if (self.failed) return Result::failure(core::ErrorCode::fault, "Replay input fault latched");
  if (self.cursor == self.events.size()) return Result::success(ReplayEnd{});
  try {
    const auto event = self.events[self.cursor++];
    const core::TimePoint at(event["at_ns"].as<std::int64_t>(), core::ClockDomain::replay);
    self.clock.advance_to(at);
    if (event["kind"].as<std::string>() == "feedback") {
      const auto q = event["quaternion_wxyz"].as<std::vector<double>>();
      if (q.size() != 4) throw std::invalid_argument("Replay quaternion requires four components");
      std::optional<OperatorInput> input;
      if (event["operator"]) {
        const auto op = event["operator"];
        input.emplace(OperatorInput{at, op["valid"].as<bool>(), op["intervening"].as<bool>(),
          op["enable_event"].as<std::uint64_t>(), core::Evidence::declared()});
      }
      return Result::success(GimbalFeedback{at, generation, {q[0], q[1], q[2], q[3]},
        event["yaw_rad"].as<double>(), event["pitch_rad"].as<double>(), event["bullet_speed_mps"].as<double>(),
        event["pose_valid"].as<bool>(), event["status_valid"].as<bool>(), input});
    }
    const auto id = event["frame_id"].as<core::FrameId>();
    if (id <= self.previous_frame) throw std::invalid_argument("Replay frame id must advance");
    const core::Stamp source(id, generation, core::TimePoint(event["exposure_ns"].as<std::int64_t>(), core::ClockDomain::replay));
    const auto image_path = self.base / event["path"].as<std::string>();
    // imread 是本地文件读取，不调用 VideoCapture，避免设备索引或网络 URL 混入。
    const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
    if (image.empty()) throw std::runtime_error("Cannot decode replay image: " + image_path.string());
    const auto stride = static_cast<std::size_t>(image.cols) * 3;
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(stride * image.rows);
    for (int row = 0; row < image.rows; ++row) std::memcpy(pixels->data() + row * stride, image.ptr(row), stride);
    const auto origin = event["time_origin"].as<std::string>();
    if (origin != "receipt_estimated" && origin != "synthetic" && origin != "hardware_mapped")
      throw std::invalid_argument("Unknown replay time origin");
    const auto frame = std::make_shared<const core::CapturedFrame>(source,
      core::Image(image.cols, image.rows, stride, pixels), at,
      origin == "synthetic" ? core::TimeOrigin::synthetic : origin == "hardware_mapped" ?
        core::TimeOrigin::hardware_mapped : core::TimeOrigin::receipt_estimated,
      core::Seconds(event["timing_sigma_s"].as<double>()), core::Evidence::declared());
    self.previous_frame = id;
    return Result::success(frame);
  } catch (const std::exception& error) {
    self.failed = true;
    return Result::failure(core::ErrorCode::io, error.what());
  }
}
}  // namespace autoaim::hal
