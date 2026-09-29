#pragma once

#include "autoaim/core/units.hpp"
#include <cstdint>

namespace autoaim::core {
enum class ClockDomain { host_monotonic, replay };

// ns 为该时钟域的纪元偏移；曝光、反馈、消费时间必须属于同一域。
class TimePoint {
public:
  explicit TimePoint(std::int64_t ns, ClockDomain domain);
  std::int64_t nanoseconds() const noexcept { return ns_; }
  ClockDomain domain() const noexcept { return domain_; }
private:
  std::int64_t ns_;
  ClockDomain domain_;
};

Seconds elapsed(TimePoint later, TimePoint earlier);
TimePoint advance(TimePoint time, Seconds delta);
TimePoint monotonic_now();
// 未来时间、跨时钟域、负时效均不能被判为新鲜；上限相等时已到期。
bool fresh(TimePoint source, TimePoint now, Seconds max_age) noexcept;
bool same_time(TimePoint left, TimePoint right) noexcept;
}  // namespace autoaim::core
