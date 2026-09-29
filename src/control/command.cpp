#include "autoaim/control/command.hpp"

namespace autoaim::control {
bool finite(const Pointing& pointing) noexcept {
  return std::isfinite(pointing.yaw.value()) && std::isfinite(pointing.pitch.value()) &&
    std::isfinite(pointing.yaw_rate_radps) && std::isfinite(pointing.pitch_rate_radps) &&
    std::isfinite(pointing.yaw_acceleration_radps2) && std::isfinite(pointing.pitch_acceleration_radps2);
}
Command::Command(std::optional<core::Stamp> stamp, Pointing angles, core::Metres distance,
                 core::CommandSpace command_space, bool control, bool fire)
    : source(std::move(stamp)), pointing(std::move(angles)), horizontal_distance(distance),
      space(command_space), control_enabled(control), shoot(fire) {
  if ((control && !source) || (fire && !control) || !finite(pointing) || distance.value() < 0)
    throw std::invalid_argument("Invalid transport command");
}
Command Command::stop(std::optional<core::Stamp> source) {
  return Command(std::move(source), {core::Radians(0), core::Radians(0), 0, 0, 0, 0},
    core::Metres(0), core::CommandSpace::absolute, false, false);
}
}  // namespace autoaim::control
