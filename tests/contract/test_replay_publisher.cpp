#include "autoaim/control/publisher.hpp"
#include "support/guard_fixture.hpp"
#include "test_support.hpp"
#include <atomic>

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::TimePoint source(1000000000, core::ClockDomain::replay);
    hal::ReplayClock clock(source);
    std::atomic<int> calls{0}, shots{0};
    control::Publisher publisher(test::guard(), clock, [&] { return test::context(clock.now()); },
      [&](const control::Command& command) {
        ++calls; if (command.shoot) ++shots;
        return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
      }, core::Seconds(0.01), control::PublishSchedule::replay_events);
    publisher.start();
    CHECK(publisher.submit(test::intent(1, 0, source)));
    CHECK(calls == 0);
    CHECK(publisher.replay_tick());
    CHECK(calls == 1 && shots == 1);
    clock.advance(core::Seconds(0.11));
    CHECK(publisher.replay_tick());
    CHECK(calls == 2 && shots == 1);
    CHECK(publisher.status().first_writes == 1);
    publisher.close();
    CHECK(calls == 3 && publisher.status().stop_written);
  });
}
