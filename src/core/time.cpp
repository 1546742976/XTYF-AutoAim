#include "autoaim/core/time.hpp"
#include <chrono>
#include <limits>
#include <stdexcept>

namespace autoaim::core {
TimePoint::TimePoint(std::int64_t ns, ClockDomain domain) : ns_(ns), domain_(domain) {
  if (domain != ClockDomain::host_monotonic && domain != ClockDomain::replay)
    throw std::invalid_argument("Unknown clock domain");
}

Seconds elapsed(TimePoint later, TimePoint earlier) {
  if (later.domain() != earlier.domain())
    throw std::invalid_argument("Cannot subtract different clock domains");

  // 先提升再相减，避免有符号整数溢出。
  return Seconds(static_cast<double>(
      (static_cast<long double>(later.nanoseconds()) - earlier.nanoseconds()) / 1e9L));
}

TimePoint advance(TimePoint time, Seconds delta) {
  const long double ns = time.nanoseconds() + std::round(delta.value() * 1e9L);
  const auto signed_limit = std::ldexp(1.0L, 63);

  if (ns < -signed_limit || ns >= signed_limit)
    throw std::overflow_error("Time overflow");

  return TimePoint(static_cast<std::int64_t>(ns), time.domain());
}

TimePoint monotonic_now() {
  return TimePoint(std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count(),
                   ClockDomain::host_monotonic);
}

bool fresh(TimePoint source, TimePoint now, Seconds max_age) noexcept {
  if (source.domain() != now.domain() || max_age.value() <= 0 ||
      source.nanoseconds() > now.nanoseconds())
    return false;

  const auto age_ns = static_cast<long double>(now.nanoseconds()) - source.nanoseconds();

  // 按时间戳的纳秒分辨率量化期限，避免 0.1 的浮点表示使恰好到期的结果多活一拍。
  const auto limit_ns = std::round(max_age.value() * 1e9L);

  return age_ns < limit_ns;
}

bool same_time(TimePoint left, TimePoint right) noexcept {
  return left.domain() == right.domain() && left.nanoseconds() == right.nanoseconds();
}
} // namespace autoaim::core
