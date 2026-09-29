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
    const auto& loaded = config.value();
    const auto simulated = core::Evidence::from_report(core::MeasurementReport::evaluate(
        {loaded.infantry.device_id, loaded.infantry.configuration_id,
         "2026-09-30", "bootstrap gate regression"}, {0, 0}, 0, core::ClockDomain::replay));
    // 分别保留一个默认 declared 证据；任一缺资格都不能进入自动模式。
    for (const bool qualified_channel : {false, true})
      for (const auto mode : {mission::ButtonMode::toggle, mission::ButtonMode::hold}) {
        auto options = loaded.infantry;
        options.button_mode = mode;
        core::TimePoint now(0, core::ClockDomain::replay);
        mission::InfantryMission policy(options, 1, now);
        const auto& channel = qualified_channel ? simulated : loaded.control_channel;
        const auto& input = qualified_channel ? loaded.button_evidence : simulated;
        for (const bool pressed : {false, true, false, true}) {
          now = core::advance(now, core::Seconds(0.001));
          policy.update(mission::OperatorSignal{now, policy.generation(), true, false, 0,
                            input, pressed}, channel, false, now);
          CHECK(policy.authority().mode == core::ControlMode::assist);
          CHECK(!policy.authority().fire && policy.generation() == 1);
        }
      }
    vision::Detection detection{{}, 1, vision::TeamColor::red, 3, false};
    CHECK(!pipeline::plate_dimensions(config.value(), detection));
    detection.armor_size = vision::ArmorSize::small;
    CHECK(pipeline::plate_dimensions(config.value(), detection));
    detection.armor_size = vision::ArmorSize::big;
    CHECK(!pipeline::plate_dimensions(config.value(), detection));
    CHECK(!config.value().calibration.qualified(core::ClockDomain::replay, "synthetic-camera",
                                                "synthetic-v1"));

    const auto live = pipeline::load_pipeline_config(root / "config/hardware/disabled.yaml");
    CHECK(!live && live.error().code == core::ErrorCode::unavailable);
    CHECK(!pipeline::load_pipeline_config(root / "missing.yaml"));
    struct Temporary {
      std::filesystem::path path;
      ~Temporary() {
        std::filesystem::remove_all(path);
      }
    } temporary{std::filesystem::temp_directory_path() /
                ("autoaim-bootstrap-" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))};

    CHECK(std::filesystem::create_directory(temporary.path));
    const auto reject = [&](const std::string& group, const std::string& key,
                            const YAML::Node& value) {
      auto yaml = YAML::LoadFile((root / "config/offline/armor.yaml").string());
      yaml["calibration_file"] = (root / "config/offline/calibration.yaml").string();
      yaml["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
      yaml[group][key] = value;
      const auto file = temporary.path / "invalid.yaml";
      {
        std::ofstream output(file);
        output << yaml;
        CHECK(output.good());
      }

      const auto result = pipeline::load_pipeline_config(file);
      CHECK(!result && result.error().code == core::ErrorCode::invalid_input);
    };

    reject("pnp", "maximum_rms_px", YAML::Load("0"));
    reject("prediction", "unknown_velocity_sigma_mps", YAML::Load("-1"));
    reject("prediction", "geometry_position_variance_m2", YAML::Load(".nan"));
    reject("impact", "aim_variance_rad2", YAML::Load("[-1, 0, 0]"));
    reject("intercept", "maximum_iterations", YAML::Load("65"));
    reject("refinement", "minimum_pixels", YAML::Load("2"));
    reject("queue", "pool_capacity", YAML::Load("0"));
    reject("queue", "pending_capacity", YAML::Load("0"));
    reject("queue", "in_flight_limit", YAML::Load("0"));
    reject("queue", "maximum_age_s", YAML::Load("0"));

    auto legacy = YAML::LoadFile((root / "config/offline/armor.yaml").string());
    legacy["calibration_file"] = (root / "config/offline/calibration.yaml").string();
    legacy["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
    legacy["detector"].remove("plate_type");
    legacy["plate_sizes"] = YAML::Load("[{class_id: -1, width_m: 0.135, height_m: 0.055}]");
    const auto check_legacy = [&] {
      {
        std::ofstream output(temporary.path / "legacy.yaml");
        output << legacy;
      }

      return pipeline::load_pipeline_config(temporary.path / "legacy.yaml");
    };
    auto old = check_legacy();
    CHECK(old && old.value().typed_plate_sizes.empty());
    detection.raw_class_id = -1;
    detection.armor_size = vision::ArmorSize::unknown;
    CHECK(pipeline::plate_dimensions(old.value(), detection));
    legacy["plate_sizes"].push_back(
        YAML::Load("{plate_type: big, width_m: 0.23, height_m: 0.055}"));
    CHECK(!check_legacy());
  });
}
