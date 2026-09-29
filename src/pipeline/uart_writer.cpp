#include "autoaim/pipeline/uart_writer.hpp"
#include "autoaim/control/protocol.hpp"

namespace autoaim::pipeline {
control::Publisher::Write make_uart14_writer(hal::Transport& transport, core::Seconds timeout) {
  if (timeout.value() <= 0 || timeout.value() > 1)
    throw std::invalid_argument("UART write timeout must be in (0,1] seconds");

  return [&transport, timeout](const control::Command& command,
                               const control::CommandMetadata& metadata) {
    const auto packet = control::encode(command, metadata, control::ProtocolKind::cboard_uart_v2);

    if (!packet)
      return hal::WriteResult{hal::WriteStatus::failed, 0, packet.error().message};

    auto result = transport.write_all(packet.value(), timeout);
    if (result.status == hal::WriteStatus::complete && result.bytes_written != 14)
      result = {hal::WriteStatus::failed, result.bytes_written, "Incomplete UART14 write"};

    return result;
  };
}
} // namespace autoaim::pipeline
