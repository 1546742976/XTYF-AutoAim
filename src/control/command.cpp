#include "autoaim/control/checked_command.hpp"

namespace autoaim::control {
bool finite(const Pointing& pointing) noexcept {
  return std::isfinite(pointing.yaw.value()) && std::isfinite(pointing.pitch.value()) &&
         std::isfinite(pointing.yaw_rate_radps) && std::isfinite(pointing.pitch_rate_radps) &&
         std::isfinite(pointing.yaw_acceleration_radps2) &&
         std::isfinite(pointing.pitch_acceleration_radps2);
}

CheckedCommand::CheckedCommand(std::optional<core::Stamp> source, Pointing pointing,
                               core::Metres distance, core::CommandSpace space, bool control,
                               bool shoot)
    : command{control, shoot, pointing.yaw.value(), pointing.pitch.value(), distance.value()},
      metadata{std::move(source),
               space,
               pointing.yaw_rate_radps,
               pointing.pitch_rate_radps,
               pointing.yaw_acceleration_radps2,
               pointing.pitch_acceleration_radps2} {
  if ((control && !metadata.source) || (shoot && !control) || !finite(pointing) ||
      distance.value() < 0)
    throw std::invalid_argument("Invalid transport command");
}

CheckedCommand CheckedCommand::stop(std::optional<core::Stamp> source) {
  return CheckedCommand(std::move(source), {core::Radians(0), core::Radians(0), 0, 0, 0, 0},
                        core::Metres(0), core::CommandSpace::absolute, false, false);
}
} // namespace autoaim::control
