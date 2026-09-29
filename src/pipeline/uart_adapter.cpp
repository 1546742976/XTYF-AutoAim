#include "autoaim/pipeline/uart_adapter.hpp"

namespace autoaim::pipeline {
UartFeedbackOptions load_uart_feedback_options(const core::Config& c) {
  if (c.require<std::string>("uart_feedback.pose_source") != "packet_quaternion")
    throw std::invalid_argument("Explicit AB packet quaternion source required");

  const auto direction = c.require<std::string>("uart_feedback.quaternion_direction");
  if (direction != "body_to_reference" && direction != "reference_to_body")
    throw std::invalid_argument("Explicit quaternion direction required");

  const auto rotation = [&](const std::string& key) {
    const auto values = c.require<std::vector<double>>("uart_feedback." + key);

    if (values.size() != 4)
      throw std::invalid_argument("UART basis mapping requires a wxyz quaternion");

    return math::checked_rotation(Eigen::Quaterniond(values[0], values[1], values[2], values[3]));
  };

  return {direction == "body_to_reference" ? QuaternionDirection::body_to_reference :
                                            QuaternionDirection::reference_to_body,
          rotation("packet_reference_to_world_wxyz"), rotation("gimbal_to_packet_body_wxyz"),
          c.require<double>("uart_feedback.yaw_sign"),
          c.require<double>("uart_feedback.pitch_sign"),
          c.require<double>("uart_feedback.yaw_offset_rad"),
          c.require<double>("uart_feedback.pitch_offset_rad"),
          core::Seconds(c.require<double>("uart_feedback.receive_delay_s"))};
}

UartFeedbackAdapter::UartFeedbackAdapter(UartFeedbackOptions options)
    : options_(std::move(options)) {
  if ((options_.direction != QuaternionDirection::body_to_reference &&
       options_.direction != QuaternionDirection::reference_to_body) ||
      (options_.yaw_sign != 1 && options_.yaw_sign != -1) ||
      (options_.pitch_sign != 1 && options_.pitch_sign != -1) ||
      !std::isfinite(options_.yaw_offset_rad) || !std::isfinite(options_.pitch_offset_rad) ||
      options_.receive_delay.value() < 0)
    throw std::invalid_argument("Invalid explicit UART axis/timing mapping");

  options_.packet_reference_to_world = math::checked_rotation(options_.packet_reference_to_world);
  options_.gimbal_to_packet_body = math::checked_rotation(options_.gimbal_to_packet_body);
}

std::uint64_t UartFeedbackAdapter::rejected() const noexcept {
  return previous_rejections_ + parser_.rejected();
}

std::vector<hal::GimbalFeedback> UartFeedbackAdapter::feed(const hal::UartChunk& chunk,
                                                        core::Generation generation) {
  if (received_ && (received_->domain() != chunk.received_at.domain() ||
                   received_->nanoseconds() > chunk.received_at.nanoseconds()))
    throw std::invalid_argument("UART receive clock changed or rewound");

  if (generation_ && generation < *generation_)
    throw std::invalid_argument("UART generation moved backwards");

  if (generation_ != generation) {
    previous_rejections_ += parser_.rejected();
    parser_ = control::GimbalStreamParser{};
    generation_ = generation;
  }
  received_ = chunk.received_at;
  std::vector<hal::GimbalFeedback> result;

  for (const auto& packet : parser_.feed(chunk.bytes)) {
    const double yaw = options_.yaw_sign * packet.yaw_rad + options_.yaw_offset_rad;
    const double pitch = options_.pitch_sign * packet.pitch_rad + options_.pitch_offset_rad;
    auto q = packet.quaternion_wxyz;
    Eigen::Quaterniond pose(q[0], q[1], q[2], q[3]);
    const bool valid = pose.coeffs().allFinite() && std::abs(pose.squaredNorm() - 1) <= 0.02 &&
                       std::isfinite(yaw) && std::isfinite(pitch);

    if (valid) {
      pose.normalize();
      if (options_.direction == QuaternionDirection::reference_to_body)
        pose = pose.conjugate();
      pose = options_.packet_reference_to_world * pose * options_.gimbal_to_packet_body;
      q = {pose.w(), pose.x(), pose.y(), pose.z()};
    }

    const bool status = std::isfinite(packet.bullet_speed_mps) && packet.bullet_speed_mps > 0;
    result.push_back({core::advance(chunk.received_at,
                                   core::Seconds(-options_.receive_delay.value())),
                      generation, q, yaw, pitch, packet.bullet_speed_mps, valid, status,
                      std::nullopt});
  }

  return result;
}
} // namespace autoaim::pipeline
