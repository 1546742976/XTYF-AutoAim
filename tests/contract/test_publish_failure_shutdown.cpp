#include "autoaim/control/publisher.hpp"
#include "support/guard_fixture.hpp"
#include "test_support.hpp"
#include <future>
#include <atomic>

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::TimePoint source(1000000000, core::ClockDomain::replay);
    hal::ReplayClock clock(source);
    std::atomic<int> calls{0};
    std::promise<void> stopped;
    auto observed = stopped.get_future();
    control::Publisher failed(test::guard(), clock, [&] { return test::context(clock.now()); },
      [&](const control::Command& command) {
        ++calls;
        if (!command.control_enabled) stopped.set_value();
        return hal::WriteResult{hal::WriteStatus::failed, 0, command.control_enabled ? "first write" : "stop write"};
      }, core::Seconds(0.001));
    failed.start();
    CHECK(failed.submit(test::intent(1, 0, source)));
    const auto observed_stop = observed.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    failed.close();
    const auto status = failed.status();
    CHECK(observed_stop && calls == 2 && status.first_failure && status.stop_failure);
    CHECK(status.stop_attempted && !status.stop_written && !status.device_stop_confirmed);
    CHECK(!failed.submit(test::intent(2, 0, source)));
    try { std::rethrow_exception(status.first_failure); }
    catch (const std::runtime_error& error) { CHECK(std::string(error.what()) == "first write"); }

    std::promise<void> initial, revoked;
    auto initial_signal = initial.get_future(); auto revoked_signal = revoked.get_future();
    std::atomic<bool> sent_initial{false}, sent_revoked{false};
    std::atomic<bool> write_busy{false}, concurrent_write{false};
    control::Publisher publisher(test::guard(), clock, [&] { return test::context(clock.now()); },
      [&](const control::Command& command) {
        if (write_busy.exchange(true)) concurrent_write = true;
        if (command.shoot && !sent_initial.exchange(true)) initial.set_value();
        if (!command.shoot && command.control_enabled && !sent_revoked.exchange(true)) revoked.set_value();
        write_busy = false;
        return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
      }, core::Seconds(0.001));
    publisher.start();
    CHECK(publisher.submit(test::intent(1, 0, source)));
    const bool wrote = initial_signal.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    clock.advance(core::Seconds(0.11));
    const bool withdrew = revoked_signal.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    std::thread first([&] { publisher.close(); });
    std::thread second([&] { publisher.close(); });
    first.join(); second.join();
    CHECK(wrote && withdrew && !concurrent_write);
    CHECK(publisher.status().first_writes == 1);
    CHECK(publisher.status().stop_written && !publisher.status().device_stop_confirmed);
  });
}
