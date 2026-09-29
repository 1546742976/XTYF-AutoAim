#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "autoaim/math/angle.hpp"

namespace autoaim::mission {
MissionRequest infantry_assist(const AimSolution& solution,
                               const decision::MeasuredPointing& measured) {
  const bool enabled =
      solution.valid && measured.valid && measured.generation == solution.source.generation;

  const decision::AimAngles correction =
      enabled ? decision::AimAngles{math::angle_residual(solution.absolute_angles.yaw,
                                                         measured.angles.yaw),
                                    core::Radians(solution.absolute_angles.pitch.value() -
                                                  measured.angles.pitch.value())}
              : decision::AimAngles{core::Radians(0), core::Radians(0)};

  return {
      solution.source, infantry_assist_authority(), correction, solution.distance, enabled, false};
}

InfantryMission::InfantryMission(InfantryModeOptions options, core::Generation generation,
                                 core::TimePoint since)
    : options_(std::move(options)), generation_(generation), since_(since),
      button_(options_.button_mode) {
  if (options_.input_maximum_age.value() <= 0 || options_.device_id.empty() ||
      options_.configuration_id.empty())
    throw std::invalid_argument("Invalid infantry mode options");
}

core::Authority InfantryMission::authority() const {
  return mode_ == core::ControlMode::assist ? infantry_assist_authority()
                                            : infantry_automatic_authority();
}

bool InfantryMission::update(const std::optional<OperatorSignal>& input,
                             const core::Evidence& automatic_channel, bool fault,
                             core::TimePoint now) {
  if (now.domain() != since_.domain() || now.nanoseconds() < since_.nanoseconds())
    throw std::invalid_argument("Mode clock moved backwards or changed domain");

  const auto qualified = [&](const core::Evidence& evidence) {
    return evidence.qualifies(now.domain(), options_.device_id, options_.configuration_id);
  };

  const bool new_enable = input && input->enable_event > consumed_enable_;

  // 即使事件伴随介入/故障，也消耗它；条件恢复不能重新解释同一按钮事件。
  if (input)
    consumed_enable_ = std::max(consumed_enable_, input->enable_event);

  const bool usable = !fault && qualified(automatic_channel) && input && input->valid &&
                      !input->intervening &&
                      (input->button_pressed.has_value() || input->generation == generation_) &&
                      qualified(input->evidence) &&
                      core::fresh(input->sampled_at, now, options_.input_maximum_age);

  auto next = mode_;

  if (input && input->button_pressed) {
    // 即使没有中间回调，输入采样间隔跨过失效门限也必须撤销旧电平资格。
    if (last_button_sample_ &&
        !core::fresh(*last_button_sample_, input->sampled_at, options_.input_maximum_age)) {
      button_.update(true, false);
      next = core::ControlMode::assist;
    }
    last_button_sample_ = input->sampled_at;
    const auto action = button_.update(*input->button_pressed, usable);

    if (action == ButtonAction::disable)
      next = core::ControlMode::assist;
    else if (action == ButtonAction::enable)
      next = core::ControlMode::automatic;
    else if (action == ButtonAction::toggle)
      next = mode_ == core::ControlMode::assist ? core::ControlMode::automatic :
                                                core::ControlMode::assist;
  } else {
    last_button_sample_.reset();
    button_.update(true, false);

    if (usable && new_enable)
      next = core::ControlMode::automatic;
  }

  if (!usable)
    next = core::ControlMode::assist;

  if (next == mode_)
    return false;

  generation_ = core::next_generation(generation_);
  since_ = now;
  mode_ = next;

  return true;
}

MissionRequest InfantryMission::request(const AimSolution& solution,
                                        const decision::MeasuredPointing& measured,
                                        bool program_fire_requested) const {
  const bool current = solution.source.generation == generation_ &&
                       solution.source.exposure.domain() == since_.domain() &&
                       solution.source.exposure.nanoseconds() > since_.nanoseconds();

  if (mode_ == core::ControlMode::assist) {
    auto output = infantry_assist(solution, measured);
    output.control_requested = output.control_requested && current;

    return output;
  }

  const bool control =
      current && solution.valid && measured.valid && measured.generation == generation_;

  return {solution.source,   authority(), solution.absolute_angles,
          solution.distance, control,     control && program_fire_requested};
}
} // namespace autoaim::mission
