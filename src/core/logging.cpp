#include "autoaim/core/logging.hpp"

namespace autoaim::core {
bool Logger::write(LogLevel level, std::string_view message) noexcept {
  if (level < minimum_)
    return true;

  try {
    const char* name = "unknown";

    switch (level) {
    case LogLevel::debug:
      name = "debug";
      break;

    case LogLevel::info:
      name = "info";
      break;

    case LogLevel::warning:
      name = "warning";
      break;

    case LogLevel::error:
      name = "error";
      break;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    sink_ << '[' << name << "] " << message << '\n';

    return static_cast<bool>(sink_);
  } catch (...) {
    return false;
  }
}
} // namespace autoaim::core
