#include "autoaim/pipeline/command_slot.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::TimePoint start(0, core::ClockDomain::replay);
    pipeline::CommandSlot slot(0, start);
    auto intent = [&](core::FrameId id, core::Generation generation) {
      const auto time = core::advance(start, core::Seconds(0.01 * static_cast<double>(id)));
      return control::ControlIntent(core::Stamp(id, generation, time), core::Role::infantry,
        core::Task::armor, core::ControlMode::assist, core::CommandSpace::relative, true, false,
        {core::Radians(0), core::Radians(0), 0, 0, 0, 0}, time, core::advance(time, core::Seconds(1)),
        {core::Stamp(id, generation, time), time, true, false, false, false, false, false}, core::Evidence::missing(), core::Metres(3));
    };
    const auto now = core::advance(start, core::Seconds(0.1));
    CHECK(slot.submit(intent(1, 0), now));
    CHECK(slot.submit(intent(2, 0), now));
    CHECK(!slot.submit(intent(1, 0), now));
    CHECK(slot.take(now)->source.frame_id == 2);
    CHECK(!slot.take(now));
    CHECK(slot.submit(intent(3, 0), now));
    CHECK(!slot.take(core::advance(now, core::Seconds(2))));
    slot.reset(1, now);
    CHECK(!slot.submit(intent(4, 1), now));
    const auto later = core::advance(start, core::Seconds(0.2));
    CHECK(slot.submit(intent(11, 1), later));
    slot.close();
    CHECK(!slot.take(later));
    CHECK(!slot.submit(intent(12, 1), later));
  });
}
