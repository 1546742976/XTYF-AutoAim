#include "autoaim/control/publisher.hpp"

namespace autoaim::control {
Publisher::Publisher(CommandGuard guard, hal::Clock& clock, Context context, Write write, core::Seconds period,
    PublishSchedule schedule)
    : guard_(std::move(guard)), clock_(clock), context_(std::move(context)), write_(std::move(write)), schedule_(schedule) {
  if (!context_ || !write_ || period.value() < 1e-6 || period.value() > 60)
    throw std::invalid_argument("Invalid publisher callbacks or period");
  period_ = std::chrono::nanoseconds(static_cast<std::int64_t>(std::round(period.value() * 1e9)));
  if (schedule == PublishSchedule::replay_events && clock.now().domain() != core::ClockDomain::replay)
    throw std::invalid_argument("Event publisher requires replay clock");
}
Publisher::~Publisher() { close(); }

void Publisher::start() {
  std::lock_guard<std::mutex> lifecycle(close_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.started || status_.closed) throw std::logic_error("Publisher session cannot restart");
  worker_ = std::thread([this] { run(); });
  status_.started = true;
}

bool Publisher::submit(ControlIntent intent) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.started || status_.closed || status_.first_failure) return false;
  }
  // 不在缓存锁内查询外部反馈，避免锁序反转。
  auto admitted = guard_.admit(std::move(intent), context_(), clock_.now());
  auto entry = std::make_shared<Entry>(std::move(admitted));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.closed || status_.first_failure) return false;
    const auto& candidate = entry->admission.intent();
    if (last_source_ && candidate.source.generation < last_source_->generation) return false;
    if (last_source_ && candidate.source.generation == last_source_->generation &&
        (candidate.source.frame_id < last_source_->frame_id ||
         (last_created_ && (candidate.created_at.domain() != last_created_->domain() ||
          candidate.created_at.nanoseconds() <= last_created_->nanoseconds())))) return false;
    last_source_.emplace(candidate.source);
    last_created_ = candidate.created_at;
    latest_ = std::move(entry);
    ++status_.accepted;
  }
  changed_.notify_all();
  return true;
}

void Publisher::invalidate() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.closed) return;
    latest_.reset();
    urgent_stop_ = true;
    status_.stop_requested = true;
  }
  changed_.notify_all();
}

bool Publisher::replay_tick() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (schedule_ != PublishSchedule::replay_events || !status_.started || status_.closed) return false;
  const auto ticket = ++requested_ticks_;
  changed_.notify_all();
  return changed_.wait_for(lock, std::chrono::seconds(2), [&] { return completed_ticks_ >= ticket || status_.closed; }) &&
    completed_ticks_ >= ticket && !status_.first_failure;
}

void Publisher::fail(std::exception_ptr error) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.first_failure) status_.first_failure = error ? error :
      std::make_exception_ptr(std::runtime_error("Unspecified pipeline failure"));
    status_.closed = true;
    status_.stop_requested = true;
    latest_.reset();
  }
  changed_.notify_all();
}

void Publisher::close() {
  std::lock_guard<std::mutex> lifecycle(close_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.closed = true;
    status_.stop_requested = true;
    latest_.reset();
  }
  changed_.notify_all();
  if (worker_.joinable()) worker_.join();
}
PublisherStatus Publisher::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

void Publisher::write_checked(const Command& command) {
  const auto result = write_(command);
  if (result.status != hal::WriteStatus::complete || result.bytes_written == 0)
    throw std::runtime_error(result.error.empty() ? "Incomplete control write" : result.error);
  std::lock_guard<std::mutex> lock(mutex_);
  ++status_.successful_writes;
}

void Publisher::run() {
  using WallClock = std::chrono::steady_clock;
  auto next_send = WallClock::now();
  std::shared_ptr<Entry> inactive;
  try {
    while (true) {
      std::shared_ptr<Entry> entry;
      std::optional<core::Stamp> stop_source;
      bool stopping = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [&] {
          return status_.closed || (schedule_ == PublishSchedule::replay_events ? requested_ticks_ > completed_ticks_ :
            urgent_stop_ || (latest_ && latest_ != inactive));
        });
        if (status_.closed) break;
        if (schedule_ == PublishSchedule::periodic)
          changed_.wait_until(lock, next_send, [&] { return status_.closed || urgent_stop_; });
        if (status_.closed) break;
        if (urgent_stop_) {
          urgent_stop_ = false;
          stopping = true;
          if (last_source_) stop_source.emplace(*last_source_);
        } else entry = latest_;
      }
      if (stopping) {
        write_checked(Command::stop(stop_source));
      } else if (entry) {
        const auto command = guard_.recheck(entry->admission, context_(), clock_.now());
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (status_.closed) break;
          if (urgent_stop_ || latest_ != entry) continue;
          // 发送接纳边界：已在此之前获准的写可完成，随后关闭/切换会发送停止。
          status_.last_inhibit_reasons = entry->admission.reasons();
        }
        write_checked(command);
        const auto completed_at = clock_.now();
        if (!entry->first_written && command.control_enabled) {
          entry->first_written = true;
          std::lock_guard<std::mutex> lock(mutex_);
          ++status_.first_writes;
          status_.last_first_send_age = core::elapsed(completed_at, entry->admission.intent().source.exposure);
        }
        if (!command.control_enabled) inactive = entry;
      }
      next_send = WallClock::now() + period_;
      if (schedule_ == PublishSchedule::replay_events) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++completed_ticks_;
        changed_.notify_all();
      }
    }
  } catch (...) { fail(std::current_exception()); }
  final_stop();
}

void Publisher::final_stop() {
  std::optional<core::Stamp> source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.stop_attempted = true;
    if (last_source_) source.emplace(*last_source_);
  }
  // 故障后至多再尝试一次停止写入；不能覆盖首次错误或循环重试。
  try {
    write_checked(Command::stop(source));
    std::lock_guard<std::mutex> lock(mutex_);
    status_.stop_written = true;
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.stop_failure = std::current_exception();
    if (!status_.first_failure) status_.first_failure = status_.stop_failure;
  }
}
}  // namespace autoaim::control
