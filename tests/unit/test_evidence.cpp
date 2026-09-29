#include "autoaim/core/evidence.hpp"
#include "test_support.hpp"
#include <limits>
#include <type_traits>

int main() {
  using namespace autoaim::core;
  static_assert(!std::is_default_constructible_v<MeasurementReport>);
  static_assert(!std::is_constructible_v<Evidence, bool>);

  return test::run([] {
    MeasurementProvenance provenance{"synthetic-device", "v1", "2026-09-29", "synthetic residuals"};
    const auto report =
        MeasurementReport::evaluate(provenance, {0.01, -0.02}, 0.03, ClockDomain::replay);

    const auto evidence = Evidence::from_report(report);
    CHECK(evidence.qualifies(ClockDomain::replay, "synthetic-device", "v1"));
    CHECK(evidence.report() && evidence.report()->provenance().method == "synthetic residuals");
    CHECK(!evidence.qualifies(ClockDomain::host_monotonic, "synthetic-device", "v1"));
    CHECK(!evidence.qualifies(ClockDomain::replay, "other", "v1"));
    CHECK(!evidence.qualifies(ClockDomain::replay, "synthetic-device", "v2"));
    CHECK(!Evidence::declared().qualifies(ClockDomain::replay, "synthetic-device", "v1"));
    CHECK(!MeasurementReport::evaluate(provenance, {0.1, 0}, 0.03, ClockDomain::replay).passed());
    CHECK(!MeasurementReport::evaluate(provenance, {NAN, 0}, 0.03, ClockDomain::replay).passed());
    CHECK_THROWS(std::invalid_argument,
                 MeasurementReport::evaluate(provenance, {}, 0.03, ClockDomain::replay));
  });
}
