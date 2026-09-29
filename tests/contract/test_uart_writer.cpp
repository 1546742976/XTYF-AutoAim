#include "autoaim/pipeline/uart_writer.hpp"
#include "autoaim/control/protocol.hpp"
#include "support/guard_fixture.hpp"
#include "support/fake_hal.hpp"
#include "test_support.hpp"

namespace {
class ShortTransport final : public autoaim::hal::Transport {
public:
  autoaim::core::Result<std::vector<std::uint8_t>> read_for(autoaim::core::Seconds) override {
    return autoaim::core::Result<std::vector<std::uint8_t>>::success({});
  }

  autoaim::hal::WriteResult write_all(const std::vector<std::uint8_t>&,
                                     autoaim::core::Seconds) override {
    return {autoaim::hal::WriteStatus::complete, 3, {}};
  }
};
} // namespace

int main() {
  using namespace autoaim;

  return test::run([] {
    test::RecordingTransport transport;
    const auto write = pipeline::make_uart14_writer(transport, core::Seconds(0.01));
    const core::TimePoint now(1000000000, core::ClockDomain::replay);
    auto guard = test::guard();
    auto admission = guard.admit(test::intent(1, 0, now), test::context(now), now);
    const auto command = guard.recheck(admission, test::context(now), now);
    CHECK(write(command.command, command.metadata).bytes_written == 14);
    CHECK(transport.writes.front() == control::encode(command.command, command.metadata).value());
    ShortTransport partial;
    hal::ReplayClock clock(now);
    control::Publisher publisher(test::guard(), clock, [&] {
        return test::context(clock.now());
      }, pipeline::make_uart14_writer(partial, core::Seconds(0.01)), core::Seconds(0.01),
      control::PublishSchedule::replay_events);
    publisher.start();
    CHECK(publisher.submit(test::intent(1, 0, now)));
    CHECK(!publisher.replay_tick());
    publisher.close();
    CHECK(publisher.status().first_failure && publisher.status().stop_attempted);
    CHECK(!publisher.status().stop_written);
  });
}
