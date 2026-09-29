#include "autoaim/core/result.hpp"

namespace autoaim::core {
const char* error_name(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::invalid_input: return "invalid_input";
    case ErrorCode::unavailable: return "unavailable";
    case ErrorCode::expired: return "expired";
    case ErrorCode::out_of_order: return "out_of_order";
    case ErrorCode::io: return "io";
    case ErrorCode::fault: return "fault";
  }
  return "unknown";
}
}  // namespace autoaim::core
