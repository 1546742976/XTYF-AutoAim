#include "autoaim/core/types.hpp"
#include "test_support.hpp"
#include <type_traits>

int main() {
  using namespace autoaim::core;
  static_assert(!std::is_default_constructible_v<Stamp>);
  static_assert(!std::is_assignable_v<Stamp&, Stamp>);
  return test::run([] {
    const Stamp stamp(10, 2, TimePoint(100, ClockDomain::replay));
    const auto copy = stamp;
    CHECK(copy.frame_id == 10 && copy.generation == 2);
    CHECK(same_time(copy.exposure, stamp.exposure));
    CHECK(next_generation(copy.generation) == 3);
    CHECK_THROWS(std::overflow_error, next_generation(UINT64_MAX));
    CHECK_THROWS(std::invalid_argument, Stamp(0, 0, stamp.exposure));
  });
}
