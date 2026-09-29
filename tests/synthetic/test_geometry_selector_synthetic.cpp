#include "autoaim/estimation/geometry_selector.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto same = test::geometry(4, estimation::HeightLayout::same, false);
    const auto distinct = test::geometry(4, estimation::HeightLayout::distinct, true);
    std::vector<std::shared_ptr<const estimation::GeometryProfile>> profiles;
    profiles.push_back(std::make_shared<const estimation::GeometryProfile>("same", same.layout, 4,
      same.axis_to_world, same.plates, core::Evidence::declared()));
    profiles.push_back(std::make_shared<const estimation::GeometryProfile>("tilted-distinct", distinct.layout, 4,
      distinct.axis_to_world, distinct.plates, core::Evidence::declared()));
    estimation::GeometrySelector selector(profiles, {2, 0.5, 100, 0}, core::Seconds(0.1), 0.2, "v1");
    const auto stamp = [](std::uint64_t frame, std::int64_t ms) {
      return core::Stamp(frame, 1, core::TimePoint(ms * 1000000, core::ClockDomain::replay));
    };
    CHECK(selector.observe(stamp(1, 10), {0, 0}));
    CHECK(!selector.selection(core::ClockDomain::replay));
    CHECK(selector.observe(stamp(2, 20), {0, 0}));
    const auto first = selector.selection(core::ClockDomain::replay);
    CHECK(first && first->index == 0 && first->supported && !first->calibrated);
    CHECK(selector.observe(stamp(3, 30), {10, 0}));
    CHECK(selector.observe(stamp(4, 40), {10, 0}));
    CHECK(selector.selection(core::ClockDomain::replay)->index == 0);
    CHECK(!selector.selection(core::ClockDomain::replay)->supported);
    CHECK(selector.observe(stamp(5, 130), {10, 0}));
    CHECK(selector.selection(core::ClockDomain::replay)->index == 1);
    CHECK(selector.selection(core::ClockDomain::replay)->supported);
    CHECK(selector.observe(stamp(6, 140), {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
    CHECK(!selector.selection(core::ClockDomain::replay)->supported);
  });
}
