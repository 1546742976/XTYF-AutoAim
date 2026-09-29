#include "autoaim/core/time.hpp"
#include "test_support.hpp"
#include <limits>
#include <type_traits>

int main() {
  using namespace autoaim::core;
  static_assert(!std::is_convertible_v<Metres, Radians>);
  static_assert(!std::is_default_constructible_v<TimePoint>);

  return test::run([] {
    const TimePoint start(0, ClockDomain::replay);
    const auto end = advance(start, Seconds(0.25));
    CHECK_NEAR(elapsed(end, start).value(), 0.25, 1e-12);
    CHECK(fresh(start, end, Seconds(0.5)));
    CHECK(!fresh(start, end, Seconds(0.25)));
    CHECK(fresh(start, TimePoint(99999999, ClockDomain::replay), Seconds(0.1)));
    CHECK(!fresh(start, TimePoint(100000000, ClockDomain::replay), Seconds(0.1)));
    CHECK(!fresh(start, TimePoint(100000001, ClockDomain::replay), Seconds(0.1)));
    CHECK(!fresh(end, start, Seconds(1)));
    CHECK(!fresh(start, monotonic_now(), Seconds(1)));
    CHECK_THROWS(std::invalid_argument, elapsed(start, monotonic_now()));
    CHECK_THROWS(std::invalid_argument, Radians(std::numeric_limits<double>::infinity()));
    CHECK_THROWS(std::overflow_error,
                 advance(TimePoint(INT64_MAX, ClockDomain::replay), Seconds(1)));

    CHECK(elapsed(TimePoint(INT64_MAX, ClockDomain::replay),
                  TimePoint(INT64_MIN, ClockDomain::replay))
              .value() > 0);

    CHECK(same_time(start, advance(end, Seconds(-0.25))));
  });
}
