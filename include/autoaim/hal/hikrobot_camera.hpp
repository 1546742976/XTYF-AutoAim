#pragma once

#include "autoaim/hal/camera.hpp"
#include "autoaim/hal/clock.hpp"

namespace autoaim::hal {
struct HikrobotOptions {
  std::string serial_number;
  core::Seconds exposure;
  double gain;
  double frame_rate_hz;
  core::Seconds transport_delay;
  core::Seconds timing_uncertainty;
  std::size_t maximum_frame_bytes;
};
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
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace autoaim::hal
