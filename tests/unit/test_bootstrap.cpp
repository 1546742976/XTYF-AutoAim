#include "autoaim/pipeline/bootstrap.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);
    CHECK(config.value().role == core::Role::infantry);
    CHECK(config.value().profiles.size() == 1);
    CHECK(config.value().queue.width == 640);
    CHECK(!config.value().calibration.qualified(core::ClockDomain::replay, "synthetic-camera", "synthetic-v1"));
    const auto live = pipeline::load_pipeline_config(root / "config/hardware/disabled.yaml");
    CHECK(!live && live.error().code == core::ErrorCode::unavailable);
    CHECK(!pipeline::load_pipeline_config(root / "missing.yaml"));
    struct Temporary {
      std::filesystem::path path;
      ~Temporary() { std::filesystem::remove_all(path); }
    } temporary{std::filesystem::temp_directory_path() / ("autoaim-bootstrap-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))};
    CHECK(std::filesystem::create_directory(temporary.path));
    const auto reject = [&](const std::string& group, const std::string& key, const YAML::Node& value) {
      auto yaml = YAML::LoadFile((root / "config/offline/armor.yaml").string());
      yaml["calibration_file"] = (root / "config/offline/calibration.yaml").string();
      yaml["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
      yaml[group][key] = value;
      const auto file = temporary.path / "invalid.yaml";
      { std::ofstream output(file); output << yaml; CHECK(output.good()); }
      const auto result = pipeline::load_pipeline_config(file);
      CHECK(!result && result.error().code == core::ErrorCode::invalid_input);
    };
    reject("pnp", "maximum_rms_px", YAML::Load("0"));
    reject("prediction", "unknown_velocity_sigma_mps", YAML::Load("-1"));
    reject("prediction", "geometry_position_variance_m2", YAML::Load(".nan"));
    reject("impact", "aim_variance_rad2", YAML::Load("[-1, 0, 0]"));
    reject("intercept", "maximum_iterations", YAML::Load("65"));
    reject("refinement", "minimum_pixels", YAML::Load("2"));
  });
}
