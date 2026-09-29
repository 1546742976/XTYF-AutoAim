#include "autoaim/core/config.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::core;
  return test::run([] {
    auto config = Config::parse("camera:\n  exposure: 0.01\n  calibrated: true\nname: replay\nbad: .nan\n");
    CHECK(config);
    const auto& value = config.value();
    CHECK_NEAR(value.number("camera.exposure", 0.001, 1), 0.01, 1e-12);
    CHECK(value.require<std::string>("name") == "replay");
    CHECK_NEAR(value.require<double>("camera.exposure"), 0.01, 1e-12);
    CHECK(value.declared_evidence("camera.calibrated").level() == EvidenceLevel::declared);
    CHECK_THROWS(std::invalid_argument, value.number("bad", 0, 1));
    CHECK_THROWS(std::invalid_argument, value.require<int>("camera.missing"));
    CHECK_THROWS(std::invalid_argument, value.require<int>("name"));
    CHECK(!Config::parse("a: 1\na: 2\n"));
    CHECK(!Config::parse("[]"));
    CHECK(!Config::parse("a: ["));
    CHECK(!Config::parse("a: &a {b: *a}"));
    CHECK(!Config::load("this-config-does-not-exist.yaml"));
  });
}
