#include "autoaim/control/watchdog.hpp"
#include "support/intents.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const core::TimePoint source(1000000000, core::ClockDomain::replay);
    const auto command = test::intent(1, 0, source);
    control::Watchdog watchdog({core::Seconds(0.2), core::Seconds(0.1), core::Seconds(0.05)});
    const auto at = [&](double delta) {
      return core::advance(source, core::Seconds(delta));
    };

    CHECK(watchdog.evaluate(command, at(0.05)).fire_fresh);
    CHECK(!watchdog.evaluate(command, at(0.1)).fire_fresh);
    CHECK(watchdog.evaluate(command, at(0.1)).control_fresh);
    CHECK(!watchdog.evaluate(command, at(0.2)).control_fresh);
    CHECK(!watchdog.evaluate(command, at(-0.1)).control_fresh);
    CHECK(!watchdog.evaluate(command, core::monotonic_now()).control_fresh);
    CHECK(watchdog.feedback_fresh(at(0.02), at(0.03), source));
    CHECK(!watchdog.feedback_fresh(source, at(0.03), source));
    CHECK(!watchdog.feedback_fresh(at(0.01), at(0.1), source));
    CHECK_THROWS(std::invalid_argument,
                 control::Watchdog({core::Seconds(0.1), core::Seconds(0.2), core::Seconds(1)}));
  });
}
