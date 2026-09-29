#include "autoaim/control/command_guard.hpp"

namespace autoaim::control {
CommandGuard::CommandGuard(GuardOptions options)
    : options_(std::move(options)), watchdog_(options_.timing) {
  if (options_.device_id.empty() || options_.configuration_id.empty() ||
      options_.yaw_tolerance.value() <= 0 || options_.pitch_tolerance.value() <= 0 ||
      (options_.operator_maximum_age && options_.operator_maximum_age->value() <= 0))
    throw std::invalid_argument("Invalid guard configuration");
}

CommandGuard::Verdict CommandGuard::evaluate(const ControlIntent& intent,
                                             const GuardContext& context,
                                             core::TimePoint now) const {
  Verdict verdict{intent.control_requested, intent.fire_requested, 0};
  auto inhibit = [&](Inhibit reason, bool stop_control = false) {
    verdict.reasons |= static_cast<std::uint32_t>(reason);
    verdict.fire = false;

    if (stop_control)
      verdict.control = false;
  };

  const auto& authority = context.authority;

  if (!authority.control || authority.role != intent.role || authority.task != intent.task ||
      authority.mode != intent.mode || authority.space != intent.space)
    inhibit(Inhibit::permission, true);

  if (!authority.fire || intent.mode == core::ControlMode::assist)
    inhibit(Inhibit::permission);

  if (context.fault_latched)
    inhibit(Inhibit::fault, true);

  if (intent.source.generation != context.generation ||
      now.domain() != context.mode_since.domain() ||
      intent.source.exposure.domain() != context.mode_since.domain() ||
      intent.source.exposure.nanoseconds() <= context.mode_since.nanoseconds())
    inhibit(Inhibit::generation, true);

  const auto timing = watchdog_.evaluate(intent, now);

  if (!timing.control_fresh)
    inhibit(Inhibit::time, true);
  else if (!timing.fire_fresh)
    inhibit(Inhibit::time);

  if (!finite(intent.pointing))
    inhibit(Inhibit::numeric, true);

  auto qualified = [&](const core::Evidence& evidence) {
    return evidence.qualifies(now.domain(), options_.device_id, options_.configuration_id);
  };

  if (!qualified(context.control_channel))
    inhibit(Inhibit::capability, true);

  if (!qualified(intent.timing_evidence))
    inhibit(Inhibit::capability);

  if (!intent.facts.historical_pose_valid)
    inhibit(Inhibit::target, true);

  if (!intent.facts.pose_reliable || !intent.facts.identity_known ||
      !intent.facts.uncertainty_acceptable || !intent.facts.ballistic_solvable ||
      !intent.facts.settled)
    inhibit(Inhibit::target);

  if (!context.feedback)
    inhibit(Inhibit::feedback, true);
  else {
    const auto& feedback = *context.feedback;
    double norm2 = 0;

    for (double value : feedback.quaternion_wxyz)
      norm2 += value * value;

    if (feedback.generation != context.generation || !feedback.pose_valid ||
        !std::isfinite(norm2) || std::abs(norm2 - 1) > 0.02 || !std::isfinite(feedback.yaw_rad) ||
        !std::isfinite(feedback.pitch_rad) ||
        !watchdog_.feedback_fresh(feedback.sampled_at, now, context.mode_since))
      inhibit(Inhibit::feedback, true);

    if (!feedback.status_valid || !std::isfinite(feedback.bullet_speed_mps) ||
        feedback.bullet_speed_mps <= 0)
      inhibit(Inhibit::bullet_speed);

    // 首版无 MPC，绝对指令即瞄准解；误差使用设备反馈，不使用规划状态冒充实际姿态。
    constexpr double period = 6.28318530717958647692;
    auto error = [&](double desired, double actual) {
      return std::abs(
          std::remainder(std::remainder(desired, period) - std::remainder(actual, period), period));
    };

    // yaw 跨周使用环绕残差；pitch 是实际俯仰量，不能把相差一整周的反馈当作已对准。
    if (error(intent.pointing.yaw.value(), feedback.yaw_rad) > options_.yaw_tolerance.value() ||
        std::abs(intent.pointing.pitch.value() - feedback.pitch_rad) >
            options_.pitch_tolerance.value())
      inhibit(Inhibit::pointing);

    if (intent.role == core::Role::infantry && intent.mode == core::ControlMode::automatic) {
      const auto& input = context.independent_operator ? context.independent_operator :
                                                        feedback.operator_input;

      if (!input || !input->valid || input->intervening || !qualified(input->source_evidence) ||
          input->sampled_at.domain() != context.mode_since.domain() ||
          input->sampled_at.nanoseconds() <= context.mode_since.nanoseconds() ||
          !core::fresh(input->sampled_at, now,
                       options_.operator_maximum_age.value_or(options_.timing.feedback_age)))
        inhibit(Inhibit::intervention, true);
    }
  }

  if (!verdict.control)
    verdict.fire = false;

  return verdict;
}

Admission CommandGuard::admit(ControlIntent intent, const GuardContext& context,
                              core::TimePoint now) const {
  const auto result = evaluate(intent, context, now);

  return Admission(std::make_shared<const ControlIntent>(std::move(intent)), result.control,
                   result.fire, result.reasons);
}

CheckedCommand CommandGuard::recheck(Admission& admission, const GuardContext& context,
                                     core::TimePoint now) const {
  const auto result = evaluate(admission.intent(), context, now);

  // 同一接纳对象永久保存撤销结果，恢复反馈只能让新的意图重新首检。
  admission.control_ = admission.control_ && result.control;
  admission.fire_ = admission.fire_ && result.fire && admission.control_;
  admission.reasons_ |= result.reasons;

  if (!admission.control_)
    return CheckedCommand::stop(admission.intent().source);

  return CheckedCommand(admission.intent().source, admission.intent().pointing,
                        admission.intent().horizontal_distance, admission.intent().space, true,
                        admission.fire_);
}
} // namespace autoaim::control
