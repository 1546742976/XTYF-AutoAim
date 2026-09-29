#include "autoaim/hal/hikrobot_camera.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    hal::MonotonicClock clock;
    const hal::HikrobotOptions options{"not-opened", core::Seconds(0.001), 0, 100,
      core::Seconds(0.002), core::Seconds(0.003), 1920 * 1080 * 3};
    hal::HikrobotCamera camera(options, clock, 1);
    CHECK(!camera.read_for(core::Seconds(0.01)));
    camera.set_generation(2);
    CHECK_THROWS(std::invalid_argument, camera.set_generation(1));
    hal::ReplayClock replay(core::TimePoint(0, core::ClockDomain::replay));
    CHECK_THROWS(std::invalid_argument, hal::HikrobotCamera(options, replay, 1));
  });
}
