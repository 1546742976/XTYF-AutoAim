#include "autoaim/hal/hikrobot_camera.hpp"

namespace autoaim::hal {
namespace {
bool same_geometry(const CameraGeometry& a, const CameraGeometry& b) {
  return a.width == b.width && a.height == b.height && a.offset_x == b.offset_x &&
         a.offset_y == b.offset_y;
}
} // namespace

void validate_hikrobot_options(const HikrobotOptions& o) {
  const auto& g = o.geometry;

  if (o.serial_number.empty() || o.exposure.value() <= 0 || !std::isfinite(o.gain) || o.gain < 0 ||
      !std::isfinite(o.frame_rate_hz) || o.frame_rate_hz <= 0 || o.transport_delay.value() < 0 ||
      o.timing_uncertainty.value() < 0 || o.maximum_frame_bytes == 0 ||
      o.maximum_frame_bytes > UINT32_MAX || g.width <= 0 || g.height <= 0 ||
      g.offset_x < 0 || g.offset_y < 0 || g.width > 1440 || g.height > 1080 ||
      g.offset_x > 1440 - g.width || g.offset_y > 1080 - g.height ||
      std::uint64_t(g.width) * g.height * 3 > o.maximum_frame_bytes ||
      !same_geometry(g, o.calibration_geometry) ||
      (o.pixel_format != CameraPixelFormat::bayer_rg8 &&
       o.pixel_format != CameraPixelFormat::bgr8) ||
      (o.white_balance != WhiteBalancePolicy::manual &&
       o.white_balance != WhiteBalancePolicy::continuous) ||
      bool(o.balance_ratio) != (o.white_balance == WhiteBalancePolicy::manual))
    throw std::invalid_argument("Invalid MV-CS016-10UC parameters or calibration geometry");

  if (o.balance_ratio)
    for (const auto ratio : *o.balance_ratio)
      if (ratio <= 0 || ratio > UINT32_MAX)
        throw std::invalid_argument("Explicit positive RGB balance ratios required");
}

void validate_hikrobot_readback(const HikrobotOptions& requested,
                               const HikrobotReadback& actual) {
  validate_hikrobot_options(requested);

  if (!same_geometry(requested.geometry, actual.geometry) ||
      actual.pixel_format != requested.pixel_format ||
      actual.white_balance != requested.white_balance ||
      actual.balance_ratio != requested.balance_ratio || actual.exposure.value() <= 0 ||
      !std::isfinite(actual.gain) || actual.gain < 0 ||
      !std::isfinite(actual.frame_rate_hz) || actual.frame_rate_hz <= 0)
    throw std::invalid_argument("Camera readback differs from geometry/format/WB contract");
}

core::Result<HikrobotOptions> load_hikrobot_options(const core::Config& c) {
  try {
    if (c.require<std::string>("camera.model") != "MV-CS016-10UC")
      throw std::invalid_argument("Only MV-CS016-10UC is configured by this adapter");

    if (!c.require<YAML::Node>("camera.serial_number").IsScalar())
      throw std::invalid_argument("Actual camera serial number must be supplied");

    const auto geometry = [&](const std::string& prefix) {
      return CameraGeometry{c.require<int>(prefix + ".width"), c.require<int>(prefix + ".height"),
                            c.require<int>(prefix + ".offset_x"),
                            c.require<int>(prefix + ".offset_y")};
    };
    const auto format = c.require<std::string>("camera.pixel_format");
    const auto wb = c.require<std::string>("camera.white_balance");

    if ((format != "BayerRG8" && format != "BGR8") || (wb != "manual" && wb != "continuous"))
      throw std::invalid_argument("Explicit supported pixel format and WB policy required");

    HikrobotOptions result{
        c.require<std::string>("camera.serial_number"),
        core::Seconds(c.require<double>("camera.exposure_s")), c.require<double>("camera.gain"),
        c.require<double>("camera.frame_rate_hz"),
        core::Seconds(c.require<double>("camera.transport_delay_s")),
        core::Seconds(c.require<double>("camera.timing_uncertainty_s")),
        c.require<std::size_t>("camera.maximum_frame_bytes"), geometry("camera.roi"),
        geometry("camera.calibration_geometry"),
        format == "BayerRG8" ? CameraPixelFormat::bayer_rg8 : CameraPixelFormat::bgr8,
        wb == "manual" ? WhiteBalancePolicy::manual : WhiteBalancePolicy::continuous,
        std::nullopt};

    if (c.contains("camera.balance_ratio_rgb")) {
      const auto values = c.require<std::vector<std::int64_t>>("camera.balance_ratio_rgb");

      if (values.size() != 3)
        throw std::invalid_argument("Three RGB balance ratios required");
      result.balance_ratio = std::array<std::int64_t, 3>{values[0], values[1], values[2]};
    }
    validate_hikrobot_options(result);

    return core::Result<HikrobotOptions>::success(std::move(result));
  } catch (const std::exception& error) {
    return core::Result<HikrobotOptions>::failure(core::ErrorCode::invalid_input, error.what());
  }
}

core::CaptureMetadata hikrobot_metadata(std::optional<std::uint32_t> previous,
                                       std::uint32_t current, std::uint64_t ticks,
                                       const HikrobotReadback& actual) {
  core::CaptureMetadata metadata;
  metadata.device_frame_id = current;
  metadata.device_timestamp_ticks = ticks;
  metadata.pixel_format = actual.pixel_format == CameraPixelFormat::bayer_rg8 ? "BayerRG8" : "BGR8";
  metadata.roi_offset = std::array<int, 2>{actual.geometry.offset_x, actual.geometry.offset_y};

  if (previous) {
    const std::uint32_t delta = current - *previous;
    metadata.device_frame_discontinuity = delta != 1;

    if (delta > 0 && delta < (std::uint32_t(1) << 31))
      metadata.device_dropped_frames = delta - 1;
  }

  return metadata;
}
} // namespace autoaim::hal
