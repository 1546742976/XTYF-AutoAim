#pragma once

#include "autoaim/hal/camera.hpp"
#include "autoaim/hal/clock.hpp"
#include "autoaim/core/config.hpp"
#include <array>

namespace autoaim::hal {
struct CameraGeometry {
  int width;
  int height;
  int offset_x;
  int offset_y;
};

enum class CameraPixelFormat { bayer_rg8, bgr8 };
enum class WhiteBalancePolicy { manual, continuous };

struct HikrobotOptions {
  std::string serial_number;
  core::Seconds exposure;
  double gain;
  double frame_rate_hz;
  core::Seconds transport_delay;
  core::Seconds timing_uncertainty;
  std::size_t maximum_frame_bytes;
  CameraGeometry geometry;
  CameraGeometry calibration_geometry;
  CameraPixelFormat pixel_format;
  WhiteBalancePolicy white_balance;
  // 手动模式 RGB BalanceRatio 的 SDK 整数值；连续模式必须不提供。
  std::optional<std::array<std::int64_t, 3>> balance_ratio;
};

// 回读只记录实际生效值，不证明帧率达成或设备标定通过。
struct HikrobotReadback {
  CameraGeometry geometry;
  CameraPixelFormat pixel_format;
  WhiteBalancePolicy white_balance;
  std::optional<std::array<std::int64_t, 3>> balance_ratio;
  core::Seconds exposure;
  double gain;
  double frame_rate_hz;
};

void validate_hikrobot_options(const HikrobotOptions& options);
void validate_hikrobot_readback(const HikrobotOptions& requested,
                               const HikrobotReadback& actual);
core::Result<HikrobotOptions> load_hikrobot_options(const core::Config& config);

// 32 位设备计数回绕按模差处理；重复/复位仅标记不连续，不猜丢帧数。
core::CaptureMetadata hikrobot_metadata(std::optional<std::uint32_t> previous,
                                       std::uint32_t current, std::uint64_t ticks,
                                       const HikrobotReadback& actual);

// 对应旧步兵/哨兵配置的 USB Hikrobot；显式序列号，绝不自动选第一台。
// SDK 原始缓存只在 read_for 内借用，返回帧拥有独立 BGR8 像素。
// 所有方法由同一采集/生命周期线程串行调用；构造不枚举、不连接设备。
class HikrobotCamera final : public Camera {
public:
  HikrobotCamera(HikrobotOptions options, Clock& clock, core::Generation generation);
  ~HikrobotCamera();
  core::Result<bool> open_device();
  void close_device() noexcept;
  void set_generation(core::Generation generation);
  core::Result<FrameRead> read_for(core::Seconds timeout) override;
  std::optional<HikrobotReadback> readback() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace autoaim::hal
