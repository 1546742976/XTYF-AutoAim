#include "autoaim/mission/mission_factory.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    CHECK(!mission::infantry_assist_authority().fire);
    CHECK(mission::infantry_automatic_authority().fire);
    CHECK(mission::sentry_authority().space == core::CommandSpace::absolute);
    CHECK(!mission::validate_authority(core::Role::sentry, core::Task::armor,
      core::ControlMode::assist, core::CommandSpace::relative, true, false));
    CHECK(!mission::validate_authority(core::Role::infantry, core::Task::armor,
      core::ControlMode::assist, core::CommandSpace::relative, true, true));
    CHECK(!mission::validate_authority(core::Role::infantry, core::Task::rune,
      core::ControlMode::automatic, core::CommandSpace::absolute, true, true));
    CHECK(!mission::validate_authority(core::Role::infantry, core::Task::armor,
      core::ControlMode::automatic, core::CommandSpace::relative, true, true));
  });
}
