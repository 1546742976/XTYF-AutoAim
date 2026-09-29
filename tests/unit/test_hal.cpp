#include "autoaim/hal/clock.hpp"
#include "support/fake_hal.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
    clock.advance(core::Seconds(0.1));
    CHECK(clock.now().nanoseconds() == 100000000);
    CHECK_THROWS(std::invalid_argument, clock.advance(core::Seconds(-1)));
    CHECK_THROWS(std::invalid_argument, hal::ReplayClock(core::monotonic_now()));
    test::EmptyCamera camera;
    CHECK(camera.read_for(core::Seconds(1)).value().status == hal::InputStatus::end);
    test::RecordingTransport transport;
    CHECK(transport.write_all({1, 2}, core::Seconds(1)).bytes_written == 2);
    transport.fail = true;
    CHECK(transport.write_all({3}, core::Seconds(1)).status == hal::WriteStatus::failed);
    CHECK(transport.writes.size() == 1);
  });
}
