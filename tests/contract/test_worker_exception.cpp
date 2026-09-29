#include "autoaim/pipeline/scheduler.hpp"
#include "test_support.hpp"
#include <future>

int main() {
  using autoaim::pipeline::Scheduler;
  return test::run([] {
    Scheduler scheduler;
    std::atomic<int> notifications{0};
    std::promise<void> notified;
    auto signal = notified.get_future();
    scheduler.start({[](const auto&) { throw std::runtime_error("worker failure"); },
      [](const auto& stop) { while (!stop.load()) std::this_thread::yield(); }}, [&] {
      ++notifications;
      notified.set_value();
      throw std::runtime_error("cleanup failure");
    });
    const bool responded = signal.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    scheduler.stop();
    CHECK(responded && scheduler.stopping() && notifications == 1);
    CHECK(scheduler.failure() && scheduler.cleanup_failure());
    try { std::rethrow_exception(scheduler.failure()); }
    catch (const std::runtime_error& error) { CHECK(std::string(error.what()) == "worker failure"); }
    scheduler.stop();
    CHECK_THROWS(std::logic_error, scheduler.start({[](const auto&) {}}, [] {}));
    Scheduler concurrent;
    concurrent.start({[](const auto& stop) { while (!stop.load()) std::this_thread::yield(); }}, [] {});
    std::thread first([&] { concurrent.stop(); });
    std::thread second([&] { concurrent.stop(); });
    first.join(); second.join();
    CHECK(!concurrent.failure());
  });
}
