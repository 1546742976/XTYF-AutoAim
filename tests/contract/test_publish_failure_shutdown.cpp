#include "autoaim/control/publisher.hpp"
#include "support/guard_fixture.hpp"
#include "test_support.hpp"
#include <future>
#include <atomic>
#include <algorithm>
#include <vector>

namespace {
void check_invalidation_boundary(bool after_admission) {
  using namespace autoaim;
  const core::TimePoint now(1000000000, core::ClockDomain::replay);
  hal::ReplayClock clock(now);
  std::promise<void> entered, resume;
  auto entered_signal = entered.get_future();
  auto resume_signal = resume.get_future().share();
  std::atomic<int> contexts{0};
  std::atomic<bool> busy{false}, concurrent{false};
  std::vector<bool> controls;
  std::vector<std::thread::id> writers;
  control::Publisher publisher(test::guard(), clock,
      [&] {
        if (++contexts == 2 && !after_admission) {
          entered.set_value();
          resume_signal.wait(); // guard 复检期间，尚未跨过发送接纳边界。
        }

        return test::context(now);
      },
      [&](const control::Command& command, const control::CommandMetadata&) {
        if (busy.exchange(true))
          concurrent = true;
        if (command.control && after_admission) {
          entered.set_value();
          resume_signal.wait(); // 已跨过接纳边界，模拟正在进行的底层写。
        }
        controls.push_back(command.control);
        writers.push_back(std::this_thread::get_id());
        busy = false;

        return hal::WriteResult{hal::WriteStatus::complete, 14, {}};
      }, core::Seconds(0.001), control::PublishSchedule::replay_events);
  publisher.start();
  CHECK(publisher.submit(test::intent(1, 0, now)));
  auto tick = std::async(std::launch::async, [&] {
    return publisher.replay_tick();
  });
  const bool reached = entered_signal.wait_for(std::chrono::seconds(1)) ==
                       std::future_status::ready;
  publisher.invalidate();
  resume.set_value(); // 所有 CHECK 之前释放屏障，失败路径也不会在 close 中死锁。
  const bool completed = tick.get();
  const bool stopped = publisher.replay_tick();
  const bool idle = publisher.replay_tick();
  publisher.close();
  CHECK(reached && completed && stopped && idle && !concurrent);
  CHECK(!publisher.status().first_failure && publisher.status().stop_written);
  CHECK(!controls.empty() && !controls.back());
  CHECK(std::count(controls.begin(), controls.end(), true) == (after_admission ? 1 : 0));
  CHECK(!after_admission || controls.front());
  for (const auto writer : writers)
    CHECK(writer == writers.front() && writer != std::this_thread::get_id());
}
} // namespace

int main() {
  using namespace autoaim;

  return test::run([] {
    check_invalidation_boundary(false);
    check_invalidation_boundary(true);
    const core::TimePoint source(1000000000, core::ClockDomain::replay);
    hal::ReplayClock clock(source);
    std::atomic<int> calls{0};
    std::promise<void> stopped;
    auto observed = stopped.get_future();
    control::Publisher failed(
        test::guard(), clock,
        [&] {
          return test::context(clock.now());
        },
        [&](const control::Command& command, const control::CommandMetadata&) {
          ++calls;

          if (!command.control)
            stopped.set_value();

          return hal::WriteResult{hal::WriteStatus::failed, 0,
                                  command.control ? "first write" : "stop write"};
        },
        core::Seconds(0.001));

    failed.start();
    CHECK(failed.submit(test::intent(1, 0, source)));
    const auto observed_stop =
        observed.wait_for(std::chrono::seconds(2)) == std::future_status::ready;

    failed.close();
    const auto status = failed.status();
    CHECK(observed_stop && calls == 2 && status.first_failure && status.stop_failure);
    CHECK(status.stop_attempted && !status.stop_written && !status.device_stop_confirmed);
    CHECK(!failed.submit(test::intent(2, 0, source)));

    try {
      std::rethrow_exception(status.first_failure);
    } catch (const std::runtime_error& error) {
      CHECK(std::string(error.what()) == "first write");
    }

    std::promise<void> initial, revoked;
    auto initial_signal = initial.get_future();
    auto revoked_signal = revoked.get_future();
    std::atomic<bool> sent_initial{false}, sent_revoked{false};
    std::atomic<bool> write_busy{false}, concurrent_write{false};
    control::Publisher publisher(
        test::guard(), clock,
        [&] {
          return test::context(clock.now());
        },
        [&](const control::Command& command, const control::CommandMetadata&) {
          if (write_busy.exchange(true))
            concurrent_write = true;

          if (command.shoot && !sent_initial.exchange(true))
            initial.set_value();

          if (!command.shoot && command.control && !sent_revoked.exchange(true))
            revoked.set_value();

          write_busy = false;

          return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
        },
        core::Seconds(0.001));

    publisher.start();
    CHECK(publisher.submit(test::intent(1, 0, source)));
    const bool wrote =
        initial_signal.wait_for(std::chrono::seconds(2)) == std::future_status::ready;

    clock.advance(core::Seconds(0.11));
    const bool withdrew =
        revoked_signal.wait_for(std::chrono::seconds(2)) == std::future_status::ready;

    std::thread first([&] {
      publisher.close();
    });
    std::thread second([&] {
      publisher.close();
    });
    first.join();
    second.join();
    CHECK(wrote && withdrew && !concurrent_write);
    CHECK(publisher.status().first_writes == 1);
    CHECK(publisher.status().stop_written && !publisher.status().device_stop_confirmed);
  });
}
