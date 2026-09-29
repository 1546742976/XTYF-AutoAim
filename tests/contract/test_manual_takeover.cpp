#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto time = [](int ms) {
      return core::TimePoint(ms * 1000000, core::ClockDomain::replay);
    };

    const auto evidence = core::Evidence::from_report(
        core::MeasurementReport::evaluate({"sim", "v1", "2026-09-29", "simulated operator events"},
                                          {0, 0}, 0, core::ClockDomain::replay));

    mission::InfantryMission policy({core::Seconds(0.05), "sim", "v1"}, 1, time(0));
    const auto signal = [&](int ms, bool manual, std::uint64_t event) {
      return mission::OperatorSignal{time(ms), policy.generation(), true, manual, event, evidence};
    };

    CHECK(!policy.authority().fire);
    CHECK(policy.update(signal(10, false, 1), evidence, false, time(10)));
    CHECK(policy.authority().fire && policy.generation() == 2);
    const mission::AimSolution aim{
        core::Stamp(1, 2, time(11)), {core::Radians(0.2), core::Radians(0)}, core::Metres(3), true};

    const decision::MeasuredPointing measured{time(11), 2, aim.absolute_angles, true};
    CHECK(policy.request(aim, measured, true).fire_requested);
    CHECK(policy.update(signal(12, true, 1), evidence, false, time(12)));
    CHECK(policy.generation() == 3 && !policy.authority().fire);
    CHECK(!policy.request(aim, measured, true).control_requested);
    CHECK(!policy.update(signal(13, false, 1), evidence, false, time(13)));
    CHECK(!policy.authority().fire);
    CHECK(policy.update(signal(14, false, 2), evidence, false, time(14)));
    CHECK(policy.update(std::nullopt, evidence, false, time(15)));
    CHECK(!policy.update(signal(16, false, 2), evidence, false, time(16)));
    CHECK(!policy.update(signal(17, false, 3), core::Evidence::declared(), false, time(17)));
    CHECK(!policy.update(signal(18, false, 3), evidence, false, time(18)));
    CHECK(policy.update(signal(19, false, 4), evidence, false, time(19)));
    CHECK(policy.update(signal(19, false, 4), evidence, false, time(80)));
    CHECK(!policy.authority().fire);
  });
}
