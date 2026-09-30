#include "queue_prealloc_baseline.hpp"
#include <deque>
#include <mutex>
#include <chrono>

// Preserve the original queue behavior for differential tests; never link into production.
namespace autoaim::pipeline {
namespace {
std::size_t block_size(const QueueOptions& options) {
  if (options.width <= 0 || options.height <= 0 || options.pending_capacity == 0 ||
      options.in_flight_limit == 0 || options.maximum_age.value() <= 0 ||
      options.stride < static_cast<std::size_t>(options.width) * 3 ||
      options.stride > SIZE_MAX / static_cast<std::size_t>(options.height))
    throw std::invalid_argument("Invalid frame queue limits");

  return options.stride * static_cast<std::size_t>(options.height);
}
} // namespace

struct BaselineFrameQueue::State {
  State(QueueOptions configuration, core::Generation epoch)
      : options(configuration), pool(options.pool_capacity, block_size(options)),
        generation(epoch) {
  }

  void count(DropReason reason) {
    ++counts[static_cast<std::size_t>(reason)];
  }

  void purge(core::TimePoint now) {
    for (auto iterator = pending.begin(); iterator != pending.end();) {
      const auto& stamp = iterator->frame->stamp;

      if (stamp.generation != generation) {
        count(DropReason::old_generation);
        iterator = pending.erase(iterator);
      } else if (!core::fresh(stamp.exposure, now, options.maximum_age)) {
        count(DropReason::expired);
        iterator = pending.erase(iterator);
      } else
        ++iterator;
    }
  }

  QueueOptions options;
  core::BufferPool pool;
  core::Generation generation;
  std::mutex mutex;
  struct Pending {
    std::shared_ptr<const core::CapturedFrame> frame;
    std::chrono::steady_clock::time_point enqueued;
  };
  std::deque<Pending> pending;
  QueueTimings timings;
  DropCounts counts{};
  std::size_t active = 0;
  core::FrameId last_input = 0;
  bool closed = false;
};

struct BaselineFrameQueue::Activity {
  explicit Activity(std::shared_ptr<State> state) : owner(std::move(state)) {
  }

  ~Activity() {
    std::lock_guard<std::mutex> lock(owner->mutex);
    --owner->active;
  }

  std::shared_ptr<State> owner;
};

BaselineFrameQueue::BaselineFrameQueue(QueueOptions options, core::Generation generation)
    : state_(std::make_shared<State>(options, generation)) {
}

bool BaselineFrameQueue::submit(core::Stamp stamp, const std::vector<std::uint8_t>& pixels,
                        core::TimePoint received, core::TimeOrigin origin,
                        core::Seconds uncertainty, core::Evidence timing, core::TimePoint now,
                        core::CaptureMetadata capture) {
  auto& state = *state_;
  std::lock_guard<std::mutex> lock(state.mutex);
  auto reject = [&](DropReason reason) {
    state.count(reason);

    return false;
  };

  if (state.closed)
    return reject(DropReason::closed);

  if (stamp.generation != state.generation)
    return reject(DropReason::old_generation);

  if (pixels.size() != block_size(state.options) || uncertainty.value() < 0 ||
      received.domain() != stamp.exposure.domain() || received.domain() != now.domain() ||
      received.nanoseconds() < stamp.exposure.nanoseconds() ||
      received.nanoseconds() > now.nanoseconds() ||
      (origin == core::TimeOrigin::synthetic && now.domain() != core::ClockDomain::replay))
    return reject(DropReason::invalid);

  if (!core::fresh(stamp.exposure, now, state.options.maximum_age))
    return reject(DropReason::expired);

  if (stamp.frame_id <= state.last_input)
    return reject(DropReason::out_of_order);

  state.last_input = stamp.frame_id;
  state.purge(now);

  while (!state.pending.empty() && (state.pending.size() >= state.options.pending_capacity ||
                                    state.pool.in_use() >= state.pool.capacity())) {
    // pending 的图像未向外暴露且尚未领取，可安全释放；在途及读者租约不在此队列。
    state.pending.pop_front();
    state.count(DropReason::capacity);
  }

  const auto copy_begin = std::chrono::steady_clock::now();
  auto lease = state.pool.copy(pixels.data(), pixels.size());

  if (!lease)
    return reject(DropReason::no_buffer);

  state.timings.pool_copy.add(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - copy_begin).count());

  state.pending.push_back({std::make_shared<const core::CapturedFrame>(
      stamp,
      core::Image(state.options.width, state.options.height, state.options.stride,
                  std::move(lease).value()),
      received, origin, uncertainty, std::move(timing), std::move(capture)),
      std::chrono::steady_clock::now()});

  return true;
}

std::optional<BaselineFrameTask> BaselineFrameQueue::take_latest(core::TimePoint now) {
  auto& state = *state_;
  std::lock_guard<std::mutex> lock(state.mutex);
  state.purge(now);

  if (state.closed || state.pending.empty() || state.active >= state.options.in_flight_limit)
    return std::nullopt;

  auto activity = std::make_shared<Activity>(state_);
  auto frame = state.pending.back().frame;
  state.timings.waiting.add(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - state.pending.back().enqueued).count());
  state.pending.pop_back();
  ++state.active;

  return BaselineFrameTask(std::move(frame), std::move(activity));
}

void BaselineFrameQueue::reset(core::Generation generation) {
  std::lock_guard<std::mutex> lock(state_->mutex);

  if (generation <= state_->generation)
    throw std::invalid_argument("Generation must advance");

  state_->counts[static_cast<std::size_t>(DropReason::old_generation)] += state_->pending.size();
  state_->pending.clear();
  state_->generation = generation;
  state_->last_input = 0;
}

void BaselineFrameQueue::close() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->closed = true;
  state_->counts[static_cast<std::size_t>(DropReason::closed)] += state_->pending.size();
  state_->pending.clear();
}

DropCounts BaselineFrameQueue::drops() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->counts;
}

QueueTimings BaselineFrameQueue::timings() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->timings;
}

std::size_t BaselineFrameQueue::in_use() const {
  return state_->pool.in_use();
}

std::size_t BaselineFrameQueue::in_flight() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->active;
}
} // namespace autoaim::pipeline
