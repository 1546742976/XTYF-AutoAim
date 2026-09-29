#pragma once

#include "autoaim/core/image.hpp"
#include "autoaim/core/result.hpp"

namespace autoaim::hal {
enum class InputStatus { frame, timeout, end };

struct FrameRead {
  InputStatus status;
  std::shared_ptr<const core::CapturedFrame> frame;
};

class Camera {
public:
  virtual ~Camera() = default;

  // 必须有界返回。timeout/end 不等于硬件故障；frame 状态必须有拥有内存的帧。
  virtual core::Result<FrameRead> read_for(core::Seconds timeout) = 0;
};
} // namespace autoaim::hal
