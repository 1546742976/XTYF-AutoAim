#pragma once

#include <mutex>
#include <ostream>
#include <string_view>

namespace autoaim::core {
enum class LogLevel { debug, info, warning, error };

// sink 生命周期须覆盖 Logger；仅保证此实例的整条日志不交错。
// 控制包写入后、且不持有联锁/队列锁时调用。日志失败不能抛出线程边界。
class Logger {
public:
  explicit Logger(std::ostream& sink, LogLevel minimum = LogLevel::info)
      : sink_(sink), minimum_(minimum) {}
  bool write(LogLevel level, std::string_view message) noexcept;
private:
  std::ostream& sink_;
  LogLevel minimum_;
  std::mutex mutex_;
};
}  // namespace autoaim::core
