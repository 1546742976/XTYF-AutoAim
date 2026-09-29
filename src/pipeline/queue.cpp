#include "autoaim/pipeline/queue.hpp"
#include <deque>
#include <mutex>
#include <chrono>

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

void validate_queue_options(const QueueOptions& options) {
  core::validate_buffer_budget(options.pool_capacity, block_size(options));
}

void WallTimeSummary::add(double milliseconds) {
  if (!std::isfinite(milliseconds) || milliseconds < 0)
    throw std::invalid_argument("Invalid wall-time sample");
  ++samples;
  total_ms += milliseconds;
  maximum_ms = std::max(maximum_ms, milliseconds);
}

struct FrameQueue::State {
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

struct FrameQueue::Activity {
  explicit Activity(std::shared_ptr<State> state) : owner(std::move(state)) {
  }

  ~Activity() {
    std::lock_guard<std::mutex> lock(owner->mutex);
    --owner->active;
  }

  std::shared_ptr<State> owner;
};

FrameQueue::FrameQueue(QueueOptions options, core::Generation generation)
    : state_(std::make_shared<State>(options, generation)) {
}

bool FrameQueue::submit(core::Stamp stamp, const std::vector<std::uint8_t>& pixels,
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

std::optional<FrameTask> FrameQueue::take_latest(core::TimePoint now) {
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

  return FrameTask(std::move(frame), std::move(activity));
}

void FrameQueue::reset(core::Generation generation) {
  std::lock_guard<std::mutex> lock(state_->mutex);

  if (generation <= state_->generation)
    throw std::invalid_argument("Generation must advance");

  state_->counts[static_cast<std::size_t>(DropReason::old_generation)] += state_->pending.size();
  state_->pending.clear();
  state_->generation = generation;
  state_->last_input = 0;
}

void FrameQueue::close() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->closed = true;
  state_->counts[static_cast<std::size_t>(DropReason::closed)] += state_->pending.size();
  state_->pending.clear();
}

DropCounts FrameQueue::drops() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->counts;
}

QueueTimings FrameQueue::timings() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->timings;
}

std::size_t FrameQueue::in_use() const {
  return state_->pool.in_use();
}

std::size_t FrameQueue::in_flight() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return state_->active;
}

ResultAdmission::ResultAdmission(core::Generation generation, core::TimePoint since,
                                 core::Seconds age)
    : generation_(generation), since_(since), maximum_age_(age) {
  if (age.value() <= 0)
    throw std::invalid_argument("Invalid admission age");
}

bool ResultAdmission::accept(const core::Stamp& source, core::TimePoint now, bool valid) {
  auto reject = [&](DropReason reason) {
    ++drops_[static_cast<std::size_t>(reason)];

    return false;
  };

  if (source.generation != generation_)
    return reject(DropReason::old_generation);

  if (!valid || source.exposure.domain() != since_.domain() || now.domain() != since_.domain() ||
      source.exposure.nanoseconds() > now.nanoseconds())
    return reject(DropReason::invalid);

  if (source.exposure.nanoseconds() <= since_.nanoseconds())
    return reject(DropReason::old_generation);

  if (!core::fresh(source.exposure, now, maximum_age_))
    return reject(DropReason::expired);

  if (source.frame_id <= last_id_ ||
      (last_source_ && source.exposure.nanoseconds() <= last_source_->nanoseconds()))
    return reject(DropReason::out_of_order);

  last_id_ = source.frame_id;
  last_source_ = source.exposure;

  return true;
}

void ResultAdmission::reset(core::Generation generation, core::TimePoint since) {
  if (generation <= generation_ || core::elapsed(since, since_).value() < 0)
    throw std::invalid_argument("Admission epoch must advance");

  generation_ = generation;
  since_ = since;
  last_id_ = 0;
  last_source_.reset();
}
} // namespace autoaim::pipeline
