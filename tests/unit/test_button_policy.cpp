#include "autoaim/mission/button_policy.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::mission;

  return test::run([] {
    for (const auto mode : {ButtonMode::toggle, ButtonMode::hold}) {
      ButtonPolicy policy(mode);
      const auto press = mode == ButtonMode::toggle ? ButtonAction::toggle : ButtonAction::enable;
      const auto release = mode == ButtonMode::toggle ? ButtonAction::none : ButtonAction::disable;
      CHECK(policy.update(true, true) == ButtonAction::none);
      CHECK(policy.update(false, true) == release);
      CHECK(policy.update(true, true) == press);
      CHECK(policy.update(true, true) == ButtonAction::none);
      CHECK(policy.update(false, true) == release);
      CHECK(policy.update(true, true) == press);
      CHECK(policy.update(true, false) == ButtonAction::disable);
      CHECK(policy.update(true, true) == ButtonAction::none);
      CHECK(policy.update(false, false) == ButtonAction::disable);
      CHECK(policy.update(true, true) == ButtonAction::none);
      CHECK(policy.update(false, true) == release);
      CHECK(policy.update(true, true) == press);
    }
  });
}
