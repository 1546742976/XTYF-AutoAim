#pragma once

#include "autoaim/hal/clock.hpp"
#include "autoaim/hal/gimbal_feedback.hpp"
#include "autoaim/core/image.hpp"
#include "autoaim/core/result.hpp"
#include <filesystem>
#include <variant>

namespace autoaim::hal {
struct ReplayEnd {};
using ReplayEvent = std::variant<ReplayEnd, GimbalFeedback, std::shared_ptr<const core::CapturedFrame>>;
// YAML 事件顺序即确定性消费顺序；同一 ns 的事件不重新排序。
// 离线文件输入，不打开设备；clock 的所有权归 pipeline。此对象单线程读取。
class FileReplay {
public:
  FileReplay(const std::filesystem::path& manifest, ReplayClock& clock);
  ~FileReplay();
  core::Result<ReplayEvent> next(core::Generation generation);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace autoaim::hal
