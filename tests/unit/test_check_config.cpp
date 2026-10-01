#include "autoaim/pipeline/entry.hpp"
#include "support/config_fixture.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>

namespace {
struct TemporaryDirectory {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("autoaim-check-config-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  TemporaryDirectory() {
    std::filesystem::create_directory(path);
  }
  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};

struct CheckedOutput {
  int code;
  std::string output;
  std::string error;
};

CheckedOutput invoke(std::vector<std::string> arguments,
                     std::optional<autoaim::core::Role> role = std::nullopt) {
  std::vector<char*> argv;
  for (auto& argument : arguments)
    argv.push_back(argument.data());
  std::ostringstream output, error;
  auto* original_output = std::cout.rdbuf(output.rdbuf());
  auto* original_error = std::cerr.rdbuf(error.rdbuf());
  const int code = autoaim::pipeline::run_entry(static_cast<int>(argv.size()), argv.data(), role);
  std::cout.rdbuf(original_output);
  std::cerr.rdbuf(original_error);
  return {code, output.str(), error.str()};
}
} // namespace

int main() {
  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    TemporaryDirectory temporary;
    auto fixture = test::copy_config_fixture(root / "config/fast_choose.yaml", temporary.path);
    auto values = YAML::LoadFile(fixture.fast_choose.string());
    values["active_detector"] = "traditional";
    // Deliberately missing replay data proves checking never opens the replay source.
    values["common"]["input_manifest"] = (temporary.path / "missing-events.yaml").string();
    test::config_fixture_detail::write(fixture.fast_choose, values);
    const auto original = YAML::Dump(YAML::LoadFile(fixture.fast_choose.string()));
    const std::vector<std::string> arguments{
        "autoaim_node", "--check-config", "--config", fixture.fast_choose.string()};
    const auto checked = invoke(arguments);
    CHECK(checked.code == 0);
    CHECK(checked.error.empty());
    const auto report = YAML::Load(checked.output);
    CHECK(report["check_schema_version"].as<int>() == 1);
    CHECK(report["valid"].as<bool>());
    CHECK(report["effective_configuration"]["execution"].as<std::string>() == "replay");
    CHECK(report["effective_configuration"]["detector"]["kind"].as<std::string>() == "traditional");
    CHECK(original == YAML::Dump(YAML::LoadFile(fixture.fast_choose.string())));

    const auto destination = temporary.path / "must-not-be-created.tsv";
    for (const auto& option : {"--input", "--output", "--record-session", "--uart-output"}) {
      auto mixed = arguments;
      mixed.insert(mixed.end(), {option, destination.string()});
      const auto failed = invoke(mixed);
      CHECK(failed.code != 0 && failed.output.empty());
      CHECK(!std::filesystem::exists(destination));
    }
    CHECK(invoke(arguments, autoaim::core::Role::sentry).code != 0);
    CHECK(invoke({"autoaim_node", "--check-config"}).code != 0);
    CHECK(invoke({"autoaim_node", "--check-config", "--config",
                  (root / "config/hardware/disabled.yaml").string()}).code != 0);

    auto missing = YAML::Clone(values);
    missing["common"]["motion"].remove("maximum_horizon_s");
    test::config_fixture_detail::write(fixture.fast_choose, missing);
    CHECK(invoke(arguments).code != 0);

    auto inconsistent = YAML::Clone(values);
    inconsistent["common"]["tracker"]["convergence"]["maximum_gap_s"] = 0.4;
    test::config_fixture_detail::write(fixture.fast_choose, inconsistent);
    CHECK(invoke(arguments).code != 0);

    values["common"]["refinement"]["brightness"] = 256;
    test::config_fixture_detail::write(fixture.fast_choose, values);
    const auto invalid = invoke(arguments);
    CHECK(invalid.code != 0 && invalid.output.empty() && !invalid.error.empty());

    values["common"]["refinement"]["brightness"] = 100;
    values["active_detector"] = "yolov5";
    const auto model = temporary.path / "not-a-real-model.xml";
    {
      std::ofstream file(model);
      file << "Configuration-only test: never create an inference request";
    }
    values["detectors"]["yolov5"]["model_path"] = model.string();
    test::config_fixture_detail::write(fixture.fast_choose, values);
    const auto model_check = invoke(arguments);
    CHECK(model_check.code == 0);
    CHECK(YAML::Load(model_check.output)["effective_configuration"]["detector"]["kind"]
              .as<std::string>() == "yolov5");
  });
}
