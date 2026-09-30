#include "autoaim/pipeline/queue.hpp"
#include "support/queue_prealloc_baseline.hpp"
#include "test_support.hpp"
#include <array>
#include <atomic>
#include <thread>

namespace {
using namespace autoaim;

core::TimePoint at(std::int64_t nanoseconds) {
  return core::TimePoint(nanoseconds, core::ClockDomain::replay);
}

template <class Queue> void observe(const Queue& queue, std::vector<std::uint64_t>& trace) {
  for (const auto count : queue.drops())
    trace.push_back(count);
  trace.push_back(queue.in_use());
  trace.push_back(queue.in_flight());
  const auto timing = queue.timings();
  trace.push_back(timing.pool_copy.samples);
  trace.push_back(timing.waiting.samples);
  // Wall-clock durations differ between runs; their validity and sample counts must agree.
  for (const auto& summary : {timing.pool_copy, timing.waiting}) {
    CHECK(std::isfinite(summary.total_ms) && summary.total_ms >= 0);
    CHECK(std::isfinite(summary.maximum_ms) && summary.maximum_ms >= 0);
    CHECK(summary.total_ms >= summary.maximum_ms);
  }
}

template <class Queue> std::vector<std::uint64_t> contract_trace() {
  Queue queue({4, 2, 2, 1, 1, 3, core::Seconds(1)}, 0);
  std::vector<std::uint64_t> trace;
  const auto now = at(2000000000);
  const auto submit = [&](std::uint64_t id, std::uint64_t generation = 0,
                          std::int64_t exposure = 2000000000, bool valid_pixels = true) {
    core::CaptureMetadata metadata;
    metadata.device_frame_id = static_cast<std::uint32_t>(id);
    metadata.device_timestamp_ticks = id * 100;
    const std::vector<std::uint8_t> pixels = valid_pixels
        ? std::vector<std::uint8_t>{static_cast<std::uint8_t>(id), 2, 3}
        : std::vector<std::uint8_t>{1};
    const bool accepted = queue.submit(core::Stamp(id, generation, at(exposure)), pixels, now,
        core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now, metadata);
    trace.push_back(accepted);
    observe(queue, trace);
    return accepted;
  };
  const auto take = [&](core::TimePoint time) {
    auto task = queue.take_latest(time);
    trace.push_back(task ? task->frame()->stamp.frame_id : 0);
    if (task) {
      const auto& frame = *task->frame();
      CHECK(frame.image.pixels->at(0) == static_cast<std::uint8_t>(frame.stamp.frame_id));
      CHECK(frame.image.pixels->at(2) == 3);
      CHECK(frame.capture.device_frame_id == frame.stamp.frame_id);
      CHECK(frame.capture.device_timestamp_ticks == frame.stamp.frame_id * 100);
      CHECK(frame.received_at.nanoseconds() == now.nanoseconds());
      CHECK(frame.time_origin == core::TimeOrigin::synthetic);
      trace.push_back(frame.stamp.generation);
      trace.push_back(static_cast<std::uint64_t>(frame.stamp.exposure.nanoseconds()));
    }
    observe(queue, trace);
    return task;
  };

  CHECK(submit(101));
  auto first = take(now);
  CHECK(submit(102));
  auto second = take(now);
  CHECK(submit(103));
  CHECK(submit(104));
  CHECK(submit(105));
  CHECK(!take(now)); // In-flight limit leaves both pending entries intact.
  auto reader1 = first->frame();
  auto reader2 = second->frame();
  first.reset();
  second.reset();
  observe(queue, trace);
  auto latest = take(now);
  auto older = take(now);
  CHECK(latest && latest->frame()->stamp.frame_id == 105);
  CHECK(older && older->frame()->stamp.frame_id == 104); // latest-first retains backlog.
  CHECK(!submit(106)); // Independent readers and tasks occupy all four pool slots.
  CHECK(!submit(106)); // No-buffer still consumed the input sequence number.
  CHECK(!submit(107, 0, 2000000000, false));
  CHECK(!submit(107, 9));
  CHECK(!submit(107, 0, 1000000000)); // Exact age limit is expired.
  CHECK(!submit(107, 0, 2000000001)); // Exposure after receipt is invalid.
  queue.reset(1);
  observe(queue, trace);
  CHECK_THROWS(std::invalid_argument, queue.reset(1));
  CHECK(!submit(107));
  CHECK(queue.in_use() == 4 && queue.in_flight() == 2);
  CHECK(reader1->image.pixels->at(0) == 101 && reader2->image.pixels->at(0) == 102);
  latest.reset();
  older.reset();
  reader1.reset();
  reader2.reset();
  CHECK(queue.in_use() == 0 && queue.in_flight() == 0);
  CHECK(submit(1, 1));
  CHECK(submit(2, 1));
  queue.reset(2); // Reset removes pending entries without reclaiming external readers.
  observe(queue, trace);
  CHECK(submit(1, 2));
  CHECK(submit(2, 2));
  queue.close();
  observe(queue, trace);
  CHECK(!submit(3, 2));
  CHECK(!take(now));
  queue.close(); // Closing twice does not count the cleared entries again.
  queue.reset(3); // A generation advance does not reopen the queue.
  CHECK(!submit(1, 3));
  observe(queue, trace);
  for (const auto count : queue.drops())
    CHECK(count > 0);

  Queue mixed({6, 6, 2, 1, 1, 3, core::Seconds(1)}, 0);
  const std::array<std::int64_t, 4> exposures{{1800000000, 1100000000,
                                             1900000000, 1200000000}};
  for (std::size_t i = 0; i < exposures.size(); ++i)
    CHECK(mixed.submit(core::Stamp(i + 1, 0, at(exposures[i])), {1, 2, 3}, now,
        core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now));
  auto newest_survivor = mixed.take_latest(at(2300000000));
  CHECK(newest_survivor && newest_survivor->frame()->stamp.frame_id == 3);
  observe(mixed, trace);
  auto oldest_survivor = mixed.take_latest(at(2300000000));
  CHECK(oldest_survivor && oldest_survivor->frame()->stamp.frame_id == 1);
  observe(mixed, trace);
  newest_survivor.reset();
  oldest_survivor.reset();
  CHECK(mixed.in_use() == 0);
  CHECK(mixed.drops()[static_cast<std::size_t>(pipeline::DropReason::expired)] == 2);
  observe(mixed, trace);
  return trace;
}

template <class Queue>
std::vector<std::uint64_t> varied_trace(std::size_t pool_capacity,
                                       std::size_t pending_capacity) {
  Queue queue({pool_capacity, pending_capacity, 2, 1, 1, 3, core::Seconds(0.1)}, 0);
  using Task = decltype(queue.take_latest(at(0)));
  std::array<Task, 4> tasks;
  std::array<std::shared_ptr<const core::CapturedFrame>, 4> readers;
  std::vector<std::uint64_t> trace;
  std::uint32_t random = 0x7a31b927;
  std::uint64_t generation = 0;
  std::uint64_t next_id = 1;
  std::int64_t time = 1000000000;
  for (int step = 0; step < 1800; ++step) {
    random ^= random << 13;
    random ^= random >> 17;
    random ^= random << 5;
    const auto slot = (random >> 8) % tasks.size();
    const auto action = random % 9;
    time += 1000000;
    if (action < 3) {
      const auto id = next_id++;
      const auto exposure = time - static_cast<std::int64_t>((random >> 16) % 130) * 1000000;
      trace.push_back(queue.submit(core::Stamp(id, generation, at(exposure)),
          {static_cast<std::uint8_t>(id), 2, 3}, at(time), core::TimeOrigin::synthetic,
          core::Seconds(0), core::Evidence::missing(), at(time)));
    } else if (action == 3) {
      tasks[slot].reset();
      tasks[slot] = queue.take_latest(at(time));
      trace.push_back(tasks[slot] ? tasks[slot]->frame()->stamp.frame_id : 0);
    } else if (action == 4) {
      if (tasks[slot])
        readers[slot] = tasks[slot]->frame();
      tasks[slot].reset();
    } else if (action == 5) {
      readers[slot].reset();
    } else if (action == 6) {
      queue.reset(++generation);
      next_id = 1;
    } else if (action == 7) {
      time += 100000000;
    } else {
      trace.push_back(queue.submit(core::Stamp(next_id, generation + 1, at(time)), {1, 2, 3},
          at(time), core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(),
          at(time)));
    }
    for (const auto& reader : readers)
      if (reader)
        CHECK(reader->image.pixels->at(0) == static_cast<std::uint8_t>(reader->stamp.frame_id));
    CHECK(queue.in_use() <= pool_capacity && queue.in_flight() <= 2);
    observe(queue, trace);
  }
  queue.close();
  for (auto& task : tasks)
    task.reset();
  for (auto& reader : readers)
    reader.reset();
  CHECK(queue.in_use() == 0 && queue.in_flight() == 0);
  observe(queue, trace);
  return trace;
}

template <class Queue> void concurrent_ownership() {
  Queue queue({8, 5, 3, 1, 1, 3, core::Seconds(1)}, 0);
  const auto now = at(1000000000);
  std::atomic<bool> produced{false};
  std::atomic<bool> intact{true};
  std::thread producer([&] {
    for (std::uint64_t id = 1; id <= 2500; ++id)
      queue.submit(core::Stamp(id, 0, now), {static_cast<std::uint8_t>(id), 2, 3}, now,
          core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now);
    produced = true;
  });
  const auto consume = [&] {
    do {
      auto task = queue.take_latest(now);
      if (task) {
        const auto reader = task->frame();
        task.reset();
        std::this_thread::yield();
        if (reader->image.pixels->at(0) != static_cast<std::uint8_t>(reader->stamp.frame_id) ||
            reader->image.pixels->at(2) != 3)
          intact = false;
      } else {
        std::this_thread::yield();
      }
    } while (!produced);
  };
  std::thread consumer1(consume);
  std::thread consumer2(consume);
  producer.join();
  consumer1.join();
  consumer2.join();
  CHECK(intact);
  queue.close();
  CHECK(queue.in_use() == 0 && queue.in_flight() == 0);

  Queue retained({2, 1, 1, 1, 1, 3, core::Seconds(1)}, 0);
  CHECK(retained.submit(core::Stamp(1, 0, now), {7, 8, 9}, now,
      core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now));
  auto task = retained.take_latest(now);
  auto reader = task->frame();
  retained.reset(1);
  retained.close();
  std::thread release_task([task = std::move(task)]() mutable { task.reset(); });
  release_task.join();
  CHECK(retained.in_flight() == 0 && retained.in_use() == 1);
  CHECK(reader->image.pixels->at(2) == 9);
  std::thread release_reader([reader = std::move(reader)]() mutable { reader.reset(); });
  release_reader.join();
  CHECK(retained.in_use() == 0);

  // Activity owns State: task and pixels can outlive the FrameQueue facade.
  using Task = decltype(retained.take_latest(now));
  Task survivor;
  {
    Queue temporary({1, 1, 1, 1, 1, 3, core::Seconds(1)}, 0);
    CHECK(temporary.submit(core::Stamp(1, 0, now), {4, 5, 6}, now,
        core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing(), now));
    survivor = temporary.take_latest(now);
  }
  CHECK(survivor && survivor->frame()->image.pixels->at(1) == 5);
  std::thread release_survivor([task = std::move(survivor)]() mutable { task.reset(); });
  release_survivor.join();
}
} // namespace

int main() {
  return test::run([] {
    CHECK(contract_trace<pipeline::FrameQueue>() == contract_trace<pipeline::BaselineFrameQueue>());
    for (const auto capacities : {std::pair<std::size_t, std::size_t>{4, 1}, {4, 3},
                                  {4, 8}, {4, SIZE_MAX}})
      CHECK(varied_trace<pipeline::FrameQueue>(capacities.first, capacities.second) ==
            varied_trace<pipeline::BaselineFrameQueue>(capacities.first, capacities.second));
    concurrent_ownership<pipeline::FrameQueue>();
    concurrent_ownership<pipeline::BaselineFrameQueue>();
  });
}
