#include "autoaim/mission/button_policy.hpp"
#include <stdexcept>

namespace autoaim::mission {
ButtonPolicy::ButtonPolicy(ButtonMode mode) : mode_(mode) {
  if (mode != ButtonMode::toggle && mode != ButtonMode::hold)
    throw std::invalid_argument("Unsupported button mode");
}

ButtonAction ButtonPolicy::update(bool pressed, bool usable) {
  if (!usable) {
    released_ = false;
    pressed_ = true;

    return ButtonAction::disable;
  }

  if (!pressed) {
    released_ = true;
    pressed_ = false;

    return mode_ == ButtonMode::hold ? ButtonAction::disable : ButtonAction::none;
  }

  const bool rising = released_ && !pressed_;
  pressed_ = true;
  released_ = false;

  if (!rising)
    return ButtonAction::none;

  return mode_ == ButtonMode::toggle ? ButtonAction::toggle : ButtonAction::enable;
}
} // namespace autoaim::mission
