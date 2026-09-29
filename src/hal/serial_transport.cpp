#include "autoaim/hal/serial_transport.hpp"
#include <cerrno>
#include <cstring>
#include <chrono>
#include <algorithm>
#ifdef __linux__
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace autoaim::hal {
SerialTransport::SerialTransport(std::string device, unsigned baud) : device_(std::move(device)), baud_(baud) {
  if (device_.empty() || (baud != 115200 && baud != 921600 && baud != 1000000))
    throw std::invalid_argument("Explicit serial path and supported baud required");
}
SerialTransport::~SerialTransport() { close_device(); }
core::Result<bool> SerialTransport::open_device() {
  using Result = core::Result<bool>;
  if (is_open()) return Result::failure(core::ErrorCode::invalid_input, "Serial already open");
#ifdef __linux__
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) return Result::failure(core::ErrorCode::io, std::strerror(errno));
  termios settings{};
  if (tcgetattr(fd_, &settings) < 0) { const std::string error = std::strerror(errno); close_device(); return Result::failure(core::ErrorCode::io, error); }
  cfmakeraw(&settings);
  settings.c_cflag = (settings.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
  const speed_t speed = baud_ == 115200 ? B115200 : baud_ == 921600 ? B921600 : B1000000;
  cfsetispeed(&settings, speed); cfsetospeed(&settings, speed);
  if (tcsetattr(fd_, TCSANOW, &settings) < 0) { const std::string error = std::strerror(errno); close_device(); return Result::failure(core::ErrorCode::io, error); }
  return Result::success(true);
#else
  return Result::failure(core::ErrorCode::unavailable, "Serial backend requires Linux");
#endif
}
void SerialTransport::close_device() noexcept {
#ifdef __linux__
  if (fd_ >= 0) ::close(fd_);
#endif
  fd_ = -1;
}
WriteResult SerialTransport::write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) {
  if (!is_open() || timeout.value() <= 0 || timeout.value() > 60)
    return {WriteStatus::failed, 0, "Serial closed or timeout outside (0,60] seconds"};
  std::size_t written = 0;
#ifdef __linux__
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout.value());
  while (written < bytes.size()) {
    const auto remaining = std::chrono::duration<double>(end - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) return {WriteStatus::failed, written, "Serial write timeout"};
    pollfd descriptor{fd_, POLLOUT, 0};
    const int ready = ::poll(&descriptor, 1, static_cast<int>(std::ceil(remaining * 1000)));
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
      return {WriteStatus::failed, written, "Serial poll/write failed"};
    const auto count = ::write(fd_, bytes.data() + written, bytes.size() - written);
    if (count < 0 && (errno == EAGAIN || errno == EINTR)) continue;
    if (count <= 0) return {WriteStatus::failed, written, std::strerror(errno)};
    written += static_cast<std::size_t>(count);
  }
#endif
  return {WriteStatus::complete, written, {}};
}
core::Result<std::vector<std::uint8_t>> SerialTransport::read_for(core::Seconds timeout) {
  using Result = core::Result<std::vector<std::uint8_t>>;
  if (!is_open() || timeout.value() <= 0 || timeout.value() > 60)
    return Result::failure(core::ErrorCode::unavailable, "Serial closed or invalid timeout");
#ifdef __linux__
  pollfd descriptor{fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, static_cast<int>(std::ceil(timeout.value() * 1000)));
  if (ready == 0 || (ready < 0 && errno == EINTR)) return Result::success({});
  if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
    return Result::failure(core::ErrorCode::io, "Serial read poll failed");
  std::vector<std::uint8_t> bytes(4096);
  const auto count = ::read(fd_, bytes.data(), bytes.size());
  if (count < 0 && (errno == EAGAIN || errno == EINTR)) return Result::success({});
  if (count <= 0) return Result::failure(core::ErrorCode::io, "Serial closed/read failed");
  bytes.resize(static_cast<std::size_t>(count));
  return Result::success(std::move(bytes));
#else
  return Result::failure(core::ErrorCode::unavailable, "Serial backend requires Linux");
#endif
}
}  // namespace autoaim::hal
