#include "autoaim/pipeline/queue.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const core::TimePoint start(0, core::ClockDomain::replay);
    const auto now = core::advance(start, core::Seconds(0.2));
    auto source = [&](core::FrameId id, core::Generation generation = 0) {
      return core::Stamp(
          id, generation,
          core::TimePoint(static_cast<std::int64_t>(id) * 1000000, core::ClockDomain::replay));
    };

    pipeline::ResultAdmission first(0, start, core::Seconds(1));
    CHECK(first.accept(source(101), now, true));
    CHECK(!first.accept(source(100), now, true));
    pipeline::ResultAdmission second(0, start, core::Seconds(1));

    // 即使 101 已提交计算，100 仍可先被接纳；接纳器不观察 in-flight 的最大 id。
    CHECK(second.accept(source(100), now, true));
    CHECK(second.accept(source(101), now, true));
    CHECK(!second.accept(source(102), now, false));
    CHECK(second.last_accepted() == 101);
    CHECK(!second.accept(source(300), now, true));
    CHECK(!second.accept(source(102), core::advance(now, core::Seconds(2)), true));
    second.reset(1, now);
    const auto later = core::advance(now, core::Seconds(0.5));
    CHECK(!second.accept(source(301), later, true));
    CHECK(!second.accept(source(102, 1), later, true));
    CHECK(second.accept(source(302, 1), later, true));
    CHECK_THROWS(std::invalid_argument, second.reset(1, later));
  });
}
