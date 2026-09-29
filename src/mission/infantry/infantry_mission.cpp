#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "autoaim/math/angle.hpp"

namespace autoaim::mission {
MissionRequest infantry_assist(const AimSolution& solution, const decision::MeasuredPointing& measured) {
  const bool enabled = solution.valid && measured.valid && measured.generation == solution.source.generation;
  const decision::AimAngles correction = enabled ? decision::AimAngles{
    math::angle_residual(solution.absolute_angles.yaw, measured.angles.yaw),
    core::Radians(solution.absolute_angles.pitch.value() - measured.angles.pitch.value())} :
    decision::AimAngles{core::Radians(0), core::Radians(0)};
  return {solution.source, infantry_assist_authority(), correction, solution.distance, enabled, false};
}
InfantryMission::InfantryMission(InfantryModeOptions options, core::Generation generation, core::TimePoint since)
    : options_(std::move(options)), generation_(generation), since_(since) {
  if (options_.input_maximum_age.value() <= 0 || options_.device_id.empty() || options_.configuration_id.empty())
    throw std::invalid_argument("Invalid infantry mode options");
}
core::Authority InfantryMission::authority() const {
  return mode_ == core::ControlMode::assist ? infantry_assist_authority() : infantry_automatic_authority();
}
bool InfantryMission::update(const std::optional<OperatorSignal>& input,
    const core::Evidence& automatic_channel, bool fault, core::TimePoint now) {
  if (now.domain() != since_.domain() || now.nanoseconds() < since_.nanoseconds())
    throw std::invalid_argument("Mode clock moved backwards or changed domain");
  const auto qualified = [&](const core::Evidence& evidence) {
    return evidence.qualifies(now.domain(), options_.device_id, options_.configuration_id);
  };
  const bool new_enable = input && input->enable_event > consumed_enable_;
  // 即使事件伴随介入/故障，也消耗它；条件恢复不能重新解释同一按钮事件。
  if (input) consumed_enable_ = std::max(consumed_enable_, input->enable_event);
  const bool usable = !fault && qualified(automatic_channel) && input && input->valid &&
    !input->intervening && input->generation == generation_ && qualified(input->evidence) &&
    core::fresh(input->sampled_at, now, options_.input_maximum_age);
  auto next = mode_;
  if (!usable) next = core::ControlMode::assist;
  else if (new_enable) next = core::ControlMode::automatic;
  if (next == mode_) return false;
  generation_ = core::next_generation(generation_);
  since_ = now;
  mode_ = next;
  return true;
}
MissionRequest InfantryMission::request(const AimSolution& solution,
    const decision::MeasuredPointing& measured, bool program_fire_requested) const {
  const bool current = solution.source.generation == generation_ &&
    solution.source.exposure.domain() == since_.domain() && solution.source.exposure.nanoseconds() > since_.nanoseconds();
  if (mode_ == core::ControlMode::assist) {
    auto output = infantry_assist(solution, measured);
    output.control_requested = output.control_requested && current;
    return output;
  }
  const bool control = current && solution.valid && measured.valid && measured.generation == generation_;
  return {solution.source, authority(), solution.absolute_angles, solution.distance, control,
    control && program_fire_requested};
}
}  // namespace autoaim::mission
