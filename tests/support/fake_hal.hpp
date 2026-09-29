#pragma once

#include "autoaim/hal/camera.hpp"
#include "autoaim/hal/transport.hpp"

namespace test {
class EmptyCamera final : public autoaim::hal::Camera {
public:
  autoaim::core::Result<autoaim::hal::FrameRead> read_for(autoaim::core::Seconds timeout) override {
    if (timeout.value() <= 0) return autoaim::core::Result<autoaim::hal::FrameRead>::failure(
      autoaim::core::ErrorCode::invalid_input, "Invalid read timeout");
    return autoaim::core::Result<autoaim::hal::FrameRead>::success({autoaim::hal::InputStatus::end, nullptr});
  }
};
class RecordingTransport final : public autoaim::hal::Transport {
public:
  std::vector<std::vector<std::uint8_t>> writes;  // 唯一写线程；检查前先 join。
  bool fail = false;
  autoaim::hal::WriteResult write_all(const std::vector<std::uint8_t>& bytes,
                                     autoaim::core::Seconds) override {
    if (fail) return {autoaim::hal::WriteStatus::failed, 0, "injected failure"};
    writes.push_back(bytes);
    return {autoaim::hal::WriteStatus::complete, bytes.size(), {}};
  }
  autoaim::core::Result<std::vector<std::uint8_t>> read_for(autoaim::core::Seconds) override {
    return autoaim::core::Result<std::vector<std::uint8_t>>::success({});
  }
};
}  // namespace test
