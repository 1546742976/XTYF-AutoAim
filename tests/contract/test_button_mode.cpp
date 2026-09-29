#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
        {"sim", "v1", "2026-09-29", "button simulation"}, {0, 0}, 0, core::ClockDomain::replay));

    for (const auto mode : {mission::ButtonMode::toggle, mission::ButtonMode::hold}) {
      core::TimePoint now(0, core::ClockDomain::replay);
      mission::InfantryMission policy({core::Seconds(0.05), "sim", "v1", mode}, 1, now);
      const auto update = [&](bool pressed, bool manual = false, bool fault = false,
                               bool valid = true) {
        now = core::advance(now, core::Seconds(0.001));
        policy.update(mission::OperatorSignal{now, policy.generation(), valid, manual, 0,
                                               evidence, pressed}, evidence, fault, now);
      };
      update(true);
      CHECK(!policy.authority().fire);
      update(false);
      update(true);
      CHECK(policy.authority().fire && policy.generation() == 2);
      update(true);
      CHECK(policy.generation() == 2);
      update(false);
      CHECK(policy.authority().fire == (mode == mission::ButtonMode::toggle));
      update(true);
      CHECK(policy.authority().fire == (mode == mission::ButtonMode::hold));
      if (mode == mission::ButtonMode::toggle) {
        update(false);
        update(true);
      }
      CHECK(policy.authority().fire);
      update(true, true);
      CHECK(!policy.authority().fire);
      update(true);
      CHECK(!policy.authority().fire);
      update(false);
      update(true);
      CHECK(policy.authority().fire);
      update(true, false, true);
      update(true);
      CHECK(!policy.authority().fire);
      update(false);
      update(true);
      CHECK(policy.authority().fire);
      update(true, false, false, false);
      update(true);
      CHECK(!policy.authority().fire);
      update(false);
      update(true);
      now = core::advance(now, core::Seconds(0.1));
      CHECK(policy.update(std::nullopt, evidence, false, now));
      update(true);
      CHECK(!policy.authority().fire);
      update(false);
      now = core::advance(now, core::Seconds(0.1));
      update(true); // 中间没有失效回调，不能沿用超时前的释放电平。
      CHECK(!policy.authority().fire);
      update(false);
      update(true);
      CHECK(policy.authority().fire);
      now = core::advance(now, core::Seconds(0.1));
      update(true); // 自动状态下的输入空档也撤销模式，恢复按住不自恢复。
      CHECK(!policy.authority().fire);
    }
  });
}
