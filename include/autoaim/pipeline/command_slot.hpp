#pragma once

#include "autoaim/control/control_intent.hpp"
#include <memory>
#include <mutex>
#include <optional>

namespace autoaim::pipeline {
// 仅存待提交意图；已接纳指令及其许可由 publisher 持有，control 不反向依赖本槽。
class CommandSlot {
public:
  CommandSlot(core::Generation generation, core::TimePoint since);
  bool submit(control::ControlIntent intent, core::TimePoint now);
  std::shared_ptr<const control::ControlIntent> take(core::TimePoint now);
  void reset(core::Generation generation, core::TimePoint since);
  void close();

private:
  std::mutex mutex_;
  core::Generation generation_;
  core::TimePoint since_;
  std::optional<core::TimePoint> last_created_;
  core::FrameId last_frame_ = 0;
  bool closed_ = false;
  std::shared_ptr<const control::ControlIntent> latest_;
};
} // namespace autoaim::pipeline
