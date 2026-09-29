#pragma once

#include "autoaim/core/time.hpp"
#include <mutex>

namespace autoaim::hal {
class Clock {
public:
  virtual ~Clock() = default;
  virtual core::TimePoint now() const = 0;
};
class MonotonicClock final : public Clock {
public:
  core::TimePoint now() const override;
};
// 测试/回放时钟只向前推进；不得注入实时设备传输会话。
class ReplayClock final : public Clock {
public:
  explicit ReplayClock(core::TimePoint start);
  core::TimePoint now() const override;
  void advance(core::Seconds delta);
  void advance_to(core::TimePoint time);  // 事件回放使用整数 ns，不经浮点往返。
private:
  mutable std::mutex mutex_;
  core::TimePoint time_;
};
}  // namespace autoaim::hal
