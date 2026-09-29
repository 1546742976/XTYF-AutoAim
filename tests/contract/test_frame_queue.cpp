#include "autoaim/pipeline/queue.hpp"
#include "test_support.hpp"
#include <thread>
#include <atomic>

int main() {
  using namespace autoaim;
  return test::run([] {
    pipeline::FrameQueue queue({4, 2, 2, 1, 1, 3, core::Seconds(1)}, 0);
    const core::TimePoint now(1000, core::ClockDomain::replay);
    auto submit = [&](std::uint64_t id, std::uint64_t generation = 0) {
      return queue.submit(core::Stamp(id, generation, now), {1, 2, 3}, now,
        core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now);
    };
    CHECK(submit(101)); auto first = queue.take_latest(now);
    CHECK(submit(102)); auto second = queue.take_latest(now);
    CHECK(submit(103)); CHECK(submit(104)); CHECK(submit(105));
    CHECK(queue.in_use() == 4 && queue.in_flight() == 2);
    CHECK(queue.drops()[static_cast<std::size_t>(pipeline::DropReason::capacity)] == 1);
    CHECK(!queue.take_latest(now));
    auto reader1 = first->frame();
    auto reader2 = second->frame();
    first.reset(); second.reset();
    auto latest = queue.take_latest(now);
    CHECK(latest->frame()->stamp.frame_id == 105);
    auto older = queue.take_latest(now);
    CHECK(older->frame()->stamp.frame_id == 104);
    CHECK(!submit(106));  // 两个在途任务 + 两个其他读者，全池占用。
    CHECK(queue.drops()[static_cast<std::size_t>(pipeline::DropReason::no_buffer)] == 1);
    queue.reset(1);
    CHECK(queue.in_use() == 4);
    CHECK(!submit(107));
    latest.reset(); older.reset(); reader1.reset(); reader2.reset();
    CHECK(queue.in_use() == 0 && queue.in_flight() == 0);
    CHECK(submit(108, 1));
    CHECK(!queue.take_latest(core::advance(now, core::Seconds(1))));
    CHECK(queue.in_use() == 0);
    queue.close(); CHECK(!submit(109, 1));

    pipeline::FrameQueue raced({4, 2, 2, 1, 1, 3, core::Seconds(1)}, 0);
    std::atomic<bool> finished{false};
    std::thread producer([&] {
      for (std::uint64_t id = 1; id <= 500; ++id)
        raced.submit(core::Stamp(id, 0, now), {1, 2, 3}, now, core::TimeOrigin::synthetic,
          core::Seconds(0), core::Evidence::missing(), now);
      finished = true;
    });
    bool intact = true;
    do {
      auto task = raced.take_latest(now);
      if (task && task->frame()->image.pixels->at(2) != 3) intact = false;
    } while (!finished);
    producer.join();
    CHECK(intact && raced.in_use() <= 4 && raced.in_flight() == 0);
  });
}
