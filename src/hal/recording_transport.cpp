#include "autoaim/hal/recording_transport.hpp"
#include <iomanip>
#include <sstream>

namespace autoaim::hal {
RecordingTransport::RecordingTransport(std::ostream& stream, Clock& clock)
    : stream_(stream), clock_(clock) {
  if (clock.now().domain() != core::ClockDomain::replay)
    throw std::invalid_argument("Recorder requires replay clock");
}

WriteResult RecordingTransport::write_all(const std::vector<std::uint8_t>& bytes,
                                          core::Seconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (failed_ || timeout.value() <= 0)
    return {WriteStatus::failed, 0, "Recorder failed or invalid deadline"};

  try {
    std::ostringstream line;
    line << clock_.now().nanoseconds() << ' ' << bytes.size();

    for (const auto byte : bytes)
      line << ' ' << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);

    stream_ << line.str() << '\n';
    stream_.flush();

    if (!stream_)
      throw std::runtime_error("Record stream write failed");

    ++completed_;

    return {WriteStatus::complete, bytes.size(), {}};
  } catch (const std::exception& error) {
    failed_ = true;

    return {WriteStatus::failed, 0, error.what()};
  }
}

core::Result<std::vector<std::uint8_t>> RecordingTransport::read_for(core::Seconds) {
  return core::Result<std::vector<std::uint8_t>>::failure(core::ErrorCode::unavailable,
                                                          "Recorder has no device feedback");
}

std::uint64_t RecordingTransport::completed_writes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return completed_;
}
} // namespace autoaim::hal
