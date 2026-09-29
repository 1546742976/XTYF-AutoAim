#pragma once

#include "autoaim/control/protocol.hpp"
#include "autoaim/core/config.hpp"
#include "autoaim/hal/file_replay.hpp"
#include "autoaim/math/se3.hpp"

namespace autoaim::pipeline {
enum class QuaternionDirection { body_to_reference, reference_to_body };

struct UartFeedbackOptions {
  QuaternionDirection direction;
  Eigen::Quaterniond packet_reference_to_world;
  Eigen::Quaterniond gimbal_to_packet_body;
  double yaw_sign;
  double pitch_sign;
  double yaw_offset_rad;
  double pitch_offset_rad;
  core::Seconds receive_delay; // 接收时刻回推的估计值，不是已验证设备时钟。
};

UartFeedbackOptions load_uart_feedback_options(const core::Config& config);

// 唯一输入线程调用。新世代丢弃半包，不把旧世代残留字节解释成新反馈。
// 原始 chunk 仍由记录器保存；返回 DTO 不包含按键或开火许可。
class UartFeedbackAdapter {
public:
  explicit UartFeedbackAdapter(UartFeedbackOptions options);
  std::vector<hal::GimbalFeedback> feed(const hal::UartChunk& chunk, core::Generation generation);
  std::uint64_t rejected() const noexcept;

private:
  UartFeedbackOptions options_;
  control::GimbalStreamParser parser_;
  std::optional<core::Generation> generation_;
  std::optional<core::TimePoint> received_;
  std::uint64_t previous_rejections_ = 0;
};
} // namespace autoaim::pipeline
