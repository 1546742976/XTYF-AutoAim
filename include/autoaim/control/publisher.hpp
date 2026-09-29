#pragma once

#include "autoaim/control/command_guard.hpp"
#include "autoaim/hal/clock.hpp"
#include "autoaim/hal/transport.hpp"
#include <condition_variable>
#include <functional>
#include <thread>

namespace autoaim::control {
enum class PublishSchedule { periodic, replay_events };

struct PublisherStatus {
  bool started = false;
  bool closed = false;
  bool stop_requested = false;
  bool stop_attempted = false;
  bool stop_written = false;
  bool device_stop_confirmed = false; // 旧协议无回执，始终为 false。
  std::exception_ptr first_failure;
  std::exception_ptr stop_failure;
  std::uint64_t accepted = 0;
  std::uint64_t successful_writes = 0;
  std::uint64_t first_writes = 0;
  std::optional<core::Seconds> last_first_send_age;
  std::uint32_t last_inhibit_reasons = 0;
};

class Publisher {
public:
  using Context = std::function<GuardContext()>;
  // 第一参数严格为五字段输出命令，第二参数保留主机溯源与既有协议所需的旁路信息。
  // 两者只在回调期间有效；由唯一发布线程调用，消费方不得重新授予 control/shoot。
  using Write = std::function<hal::WriteResult(const Command&, const CommandMetadata&)>;

  // clock/context/write 生命周期覆盖本对象。回调不得调用 close/start；写回调必须有界。
  Publisher(CommandGuard guard, hal::Clock& clock, Context context, Write write,
            core::Seconds period, PublishSchedule schedule = PublishSchedule::periodic);

  ~Publisher();
  Publisher(const Publisher&) = delete;
  Publisher& operator=(const Publisher&) = delete;
  void start();
  bool submit(ControlIntent intent);
  void invalidate(); // 普通模式切换：清缓存并请求一次停止，不关闭会话。
  // 回放事件屏障：仍由同一发布线程执行同一 guard/write 路径；调用者不得并发推进逻辑时钟。
  // 每调用一次最多产生一次普通写，另有关闭/故障停止；等待最多 2 秒。
  bool replay_tick();
  void fail(std::exception_ptr error);
  void close(); // 终止会话；允许控制线程并发调用，不允许回调自身调用。
  PublisherStatus status() const;

private:
  struct Entry {
    explicit Entry(Admission value) : admission(std::move(value)) {
    }

    Admission admission;
    bool first_written = false;
  };

  void run();
  void write_checked(const CheckedCommand& checked, std::uint32_t reasons = 0, bool stop = false);
  void final_stop();
  CommandGuard guard_;
  hal::Clock& clock_;
  Context context_;
  Write write_;
  std::chrono::nanoseconds period_;
  PublishSchedule schedule_;
  std::uint64_t requested_ticks_ = 0;
  std::uint64_t completed_ticks_ = 0;
  mutable std::mutex mutex_;
  std::mutex close_mutex_;
  std::condition_variable changed_;
  PublisherStatus status_;
  std::shared_ptr<Entry> latest_;
  std::optional<core::Stamp> last_source_;
  std::optional<core::TimePoint> last_created_;
  bool urgent_stop_ = false;
  std::thread worker_;
};
} // namespace autoaim::control
