#include "autoaim/hal/can_transport.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace autoaim::hal {
CanTransport::CanTransport(CanOptions options) : options_(std::move(options)) {
  if (options_.interface_name.empty() || options_.interface_name.size() > 15 ||
      options_.transmit_id > 0x7ff || options_.receive_ids.empty() ||
      std::any_of(options_.receive_ids.begin(), options_.receive_ids.end(), [](auto id) {
        return id > 0x7ff;
      }))
    throw std::invalid_argument("Invalid standard CAN options");
}

CanTransport::~CanTransport() {
  close_device();
}

core::Result<bool> CanTransport::open_device() {
  using Result = core::Result<bool>;

  if (is_open())
    return Result::failure(core::ErrorCode::invalid_input, "CAN already open");
#ifdef __linux__
  const auto index = if_nametoindex(options_.interface_name.c_str());

  if (!index)
    return Result::failure(core::ErrorCode::io, "CAN interface not found");

  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

  if (fd_ < 0)
    return Result::failure(core::ErrorCode::io, std::strerror(errno));

  std::vector<can_filter> filters;

  for (const auto id : options_.receive_ids)
    filters.push_back({id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);

  if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                 filters.size() * sizeof(can_filter)) < 0 ||
      ::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close_device();

    return Result::failure(core::ErrorCode::io, error);
  }

  return Result::success(true);
#else
  return Result::failure(core::ErrorCode::unavailable, "SocketCAN requires Linux");
#endif
}

void CanTransport::close_device() noexcept {
#ifdef __linux__
  if (fd_ >= 0)
    ::close(fd_);
#endif
  fd_ = -1;
}

WriteResult CanTransport::write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) {
  if (!is_open() || bytes.size() > 8 || timeout.value() <= 0 || timeout.value() > 60)
    return {WriteStatus::failed, 0, "CAN closed or invalid frame/timeout"};
#ifdef __linux__
  pollfd descriptor{fd_, POLLOUT, 0};

  if (::poll(&descriptor, 1, static_cast<int>(std::ceil(timeout.value() * 1000))) <= 0 ||
      (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
    return {WriteStatus::failed, 0, "CAN write poll failed"};

  can_frame frame{};
  frame.can_id = options_.transmit_id;
  frame.can_dlc = static_cast<unsigned char>(bytes.size());
  std::copy(bytes.begin(), bytes.end(), frame.data);

  if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame))
    return {WriteStatus::failed, 0, "CAN complete frame write failed"};

  return {WriteStatus::complete, bytes.size(), {}};
#else
  return {WriteStatus::failed, 0, "SocketCAN requires Linux"};
#endif
}

core::Result<std::vector<std::uint8_t>> CanTransport::read_for(core::Seconds timeout) {
  using Result = core::Result<std::vector<std::uint8_t>>;

  if (!is_open() || timeout.value() <= 0 || timeout.value() > 60)
    return Result::failure(core::ErrorCode::unavailable, "CAN closed or invalid timeout");
#ifdef __linux__
  pollfd descriptor{fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, static_cast<int>(std::ceil(timeout.value() * 1000)));

  if (ready == 0 || (ready < 0 && errno == EINTR))
    return Result::success({});

  if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
    return Result::failure(core::ErrorCode::io, "CAN read poll failed");

  can_frame frame{};
  const auto count = ::read(fd_, &frame, sizeof(frame));

  if (count < 0 && (errno == EAGAIN || errno == EINTR))
    return Result::success({});

  if (count != sizeof(frame) || frame.can_dlc > 8 || (frame.can_id & ~CAN_SFF_MASK))
    return Result::failure(core::ErrorCode::io, "Invalid standard CAN frame");

  std::vector<std::uint8_t> bytes(4 + frame.can_dlc);

  for (int i = 0; i < 4; ++i)
    bytes[i] = static_cast<std::uint8_t>(frame.can_id >> (8 * i));

  std::copy(frame.data, frame.data + frame.can_dlc, bytes.begin() + 4);

  return Result::success(std::move(bytes));
#else
  return Result::failure(core::ErrorCode::unavailable, "SocketCAN requires Linux");
#endif
}
} // namespace autoaim::hal
