#include "autoaim/mission/mission_factory.hpp"

namespace autoaim::mission {
core::Result<core::Authority> validate_authority(core::Role role, core::Task task,
    core::ControlMode mode, core::CommandSpace space, bool control, bool fire) {
  using namespace core;
  bool allowed = false;
  if (task == Task::armor) {
    if (role == Role::infantry && mode == ControlMode::assist)
      allowed = space == CommandSpace::relative && !fire;
    else if ((role == Role::infantry || role == Role::sentry) && mode == ControlMode::automatic)
      allowed = space == CommandSpace::absolute && (!fire || control);
  }
  if (!allowed) return Result<Authority>::failure(ErrorCode::invalid_input, "Unsupported role/task/control authority combination");
  return Result<Authority>::success({role, task, mode, space, control, fire});
}
core::Authority infantry_assist_authority() {
  return validate_authority(core::Role::infantry, core::Task::armor, core::ControlMode::assist,
    core::CommandSpace::relative, true, false).value();
}
core::Authority infantry_automatic_authority() {
  return validate_authority(core::Role::infantry, core::Task::armor, core::ControlMode::automatic,
    core::CommandSpace::absolute, true, true).value();
}
core::Authority sentry_authority() {
  return validate_authority(core::Role::sentry, core::Task::armor, core::ControlMode::automatic,
    core::CommandSpace::absolute, true, true).value();
}
}  // namespace autoaim::mission
