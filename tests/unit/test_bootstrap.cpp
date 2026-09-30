#include "autoaim/pipeline/bootstrap.hpp"
#include "support/config_fixture.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>

namespace {
using namespace autoaim;

void write_yaml(const std::filesystem::path& file, const YAML::Node& node) {
  test::config_fixture_detail::write(file, node);
}

void check_fast_choose(const std::filesystem::path& root, const std::filesystem::path& directory) {
  const YAML::Node repository_fast = YAML::LoadFile((root / "config/fast_choose.yaml").string());
  CHECK(repository_fast["active_detector"].as<std::string>() == "yolov5");
  auto fixture = test::copy_config_fixture(root / "config/offline/armor.yaml", directory);
  fixture.entry = fixture.fast_choose;
  auto defaults = YAML::LoadFile(fixture.fast_choose.string());
  CHECK(defaults["active_detector"].as<std::string>() == "yolov5");
  CHECK(std::filesystem::create_directory(directory / "models"));
  {
    std::ofstream model(directory / "models/model-v5.xml");
    model << "configuration-only fixture; no inference is started";
  }
  defaults["detectors"]["yolov5"]["model_path"] = "models/model-v5.xml";
  write_yaml(fixture.fast_choose, defaults);
  const auto selected_default = pipeline::load_pipeline_config(fixture.entry);
  CHECK(selected_default);
  CHECK(std::get<vision::YoloOptions>(selected_default.value().detector).format ==
        vision::YoloFormat::legacy_v5);
  CHECK(selected_default.value().configuration_snapshot->base.path == fixture.bases.at("yolov5"));
  CHECK(std::filesystem::path(std::get<vision::YoloOptions>(selected_default.value().detector).model_path)
        == directory / "models/model-v5.xml");

  // 传统检测语义在临时副本中显式选择，不依赖或改写仓库默认检测器。
  auto traditional = YAML::Clone(defaults);
  traditional["active_detector"] = "traditional";
  write_yaml(fixture.fast_choose, traditional);
  const YAML::Node original = YAML::Clone(traditional);
  const YAML::Node original_base = YAML::LoadFile(fixture.base.string());
  const auto fast = pipeline::load_pipeline_config(fixture.entry);
  const auto direct = pipeline::load_pipeline_config(fixture.base);
  CHECK(fast && direct);
  CHECK(std::holds_alternative<vision::TraditionalOptions>(fast.value().detector));
  CHECK(YAML::Dump(fast.value().configuration_snapshot->effective) ==
        YAML::Dump(direct.value().configuration_snapshot->effective));
  CHECK(fast.value().configuration_snapshot->entry.path == fixture.fast_choose);
  CHECK(fast.value().configuration_snapshot->base.path == fixture.base);
  CHECK(direct.value().configuration_snapshot->entry.path == fixture.base);
  CHECK(fast.value().configuration_snapshot->fast_choose->path == fixture.fast_choose);
  CHECK(fast.value().input_manifest.is_absolute());
  CHECK_NEAR(fast.value().tracker.eso.translation_bandwidth_radps, 10, 0);
  CHECK_NEAR(fast.value().tracker.eso.angular_bandwidth_radps, 10, 0);
  CHECK_NEAR(fast.value().tracker.eso.linear_jerk_psd, 1, 0);

  // 非选中配置和模型缺失时 traditional 仍可加载，不隐式打开无关依赖。
  auto isolated = YAML::Clone(original);
  for (const char* kind : {"yolov5", "yolo11"}) {
    isolated["detectors"][kind]["config_file"] = "missing-base.yaml";
    isolated["detectors"][kind]["model_path"] = "missing-model.xml";
  }
  write_yaml(fixture.fast_choose, isolated);
  CHECK(pipeline::load_pipeline_config(fixture.entry));
  isolated["active_detector"] = "yolov5";
  isolated["detectors"]["traditional"]["config_file"] = "also-missing-base.yaml";
  write_yaml(fixture.fast_choose, isolated);
  CHECK(pipeline::load_pipeline_config(fixture.base)); // 固定入口不被 config_file 重定向。
  CHECK(!pipeline::load_pipeline_config(fixture.entry));

  // 快调自身路径与基础场景路径有各自的声明目录；选中的模型仅检查显式文件存在。
  auto selected = YAML::Clone(original);
  selected["active_detector"] = "yolo11";
  selected["common"]["input_manifest"] = "dataset/events.yaml";
  selected["detectors"]["yolo11"]["config_file"] = fixture.bases.at("yolo11").filename().string();
  selected["detectors"]["yolo11"]["model_path"] = "models/model.xml";
  {
    std::ofstream model(directory / "models/model.xml");
    model << "configuration-only fixture; no inference is started";
  }
  auto yolo_base = YAML::LoadFile(fixture.bases.at("yolo11").string());
  std::filesystem::copy_file(root / "config/offline/calibration.yaml", directory / "calibration.yaml");
  std::filesystem::copy_file(root / "config/offline/geometry.yaml", directory / "geometry.yaml");
  yolo_base["calibration_file"] = "calibration.yaml";
  yolo_base["geometry_files"][0] = "geometry.yaml";
  write_yaml(fixture.bases.at("yolo11"), yolo_base);
  selected["eso"]["translation_bandwidth_radps"] = 7;
  selected["eso"]["angular_bandwidth_radps"] = 11;
  selected["eso"]["linear_jerk_psd"] = 0;
  write_yaml(fixture.fast_choose, selected);
  const auto selected_config = pipeline::load_pipeline_config(fixture.entry);
  CHECK(selected_config);
  CHECK(std::get<vision::YoloOptions>(selected_config.value().detector).format ==
        vision::YoloFormat::armor_v11);
  CHECK(std::filesystem::path(std::get<vision::YoloOptions>(selected_config.value().detector).model_path)
        == directory / "models/model.xml");
  CHECK(selected_config.value().input_manifest == directory / "dataset/events.yaml");
  CHECK(selected_config.value().configuration_snapshot->effective["calibration_file"].as<std::string>()
        == (directory / "calibration.yaml").string());
  CHECK(selected_config.value().configuration_snapshot->effective["geometry_files"][0]
        .as<std::string>() == (directory / "geometry.yaml").string());
  CHECK_NEAR(selected_config.value().tracker.eso.translation_bandwidth_radps, 7, 0);
  CHECK_NEAR(selected_config.value().tracker.eso.angular_bandwidth_radps, 11, 0);
  CHECK_NEAR(selected_config.value().tracker.eso.linear_jerk_psd, 0, 0);
  CHECK(pipeline::load_pipeline_config(fixture.base));

  // 新入口必须匹配选中的 kind 与规范化回指；不展开递归 include。
  auto wrong = YAML::Clone(original);
  wrong["detectors"]["traditional"]["config_file"] = fixture.bases.at("yolo11").string();
  write_yaml(fixture.fast_choose, wrong);
  CHECK(!pipeline::load_pipeline_config(fixture.entry));
  write_yaml(fixture.fast_choose, original);
  auto wrong_base = YAML::Clone(original_base);
  wrong_base["fast_choose_file"] = "other-fast.yaml";
  write_yaml(fixture.base, wrong_base);
  CHECK(!pipeline::load_pipeline_config(fixture.entry));
  write_yaml(fixture.base, original_base);

  const auto reject_fast = [&](auto change) {
    auto yaml = YAML::Clone(original);
    change(yaml);
    write_yaml(fixture.fast_choose, yaml);
    const auto rejected = pipeline::load_pipeline_config(fixture.base);
    CHECK(!rejected && rejected.error().code == core::ErrorCode::invalid_input);
    CHECK(rejected.error().message.find("Cannot open config") == std::string::npos);
  };
  reject_fast([](YAML::Node n) { n["fast_choose_version"] = 2; });
  reject_fast([](YAML::Node n) { n["active_detector"] = "unknown"; });
  reject_fast([](YAML::Node n) { n["unexpected"] = 1; });
  reject_fast([](YAML::Node n) { n["common"]["queue"]["typo"] = 1; });
  reject_fast([](YAML::Node n) { n["common"]["detector"]["kind"] = "yolo11"; });
  reject_fast([](YAML::Node n) { n["detectors"]["traditional"]["enemy"] = "red"; });
  reject_fast([](YAML::Node n) { n["common"]["queue"].remove("maximum_age_s"); });
  reject_fast([](YAML::Node n) { n["detectors"].remove("yolov5"); });
  reject_fast([](YAML::Node n) { n["common"]["pnp"]["maximum_rms_px"] = 0; });
  reject_fast([](YAML::Node n) { n["common"]["tracker"]["initial_variance"] = YAML::Load("[1, 2]"); });
  reject_fast([](YAML::Node n) { n["detectors"]["yolo11"]["confidence"] = YAML::Load(".nan"); });
  reject_fast([](YAML::Node n) { n["detectors"]["yolov5"]["maximum_detections"] = 0; });
  reject_fast([](YAML::Node n) { n["detectors"]["traditional"]["brightness"] = 256; });
  reject_fast([](YAML::Node n) { n["eso"].remove("linear_jerk_psd"); });
  reject_fast([](YAML::Node n) { n["eso"]["translation_bandwidth_radps"] = 0; });
  reject_fast([](YAML::Node n) { n["eso"]["angular_bandwidth_radps"] = YAML::Load(".inf"); });
  reject_fast([](YAML::Node n) { n["eso"]["linear_jerk_psd"] = -1; });
  write_yaml(fixture.fast_choose, original);
  {
    std::ofstream output(fixture.fast_choose, std::ios::app);
    output << "active_detector: traditional\n";
  }
  const auto duplicate = pipeline::load_pipeline_config(fixture.base);
  CHECK(!duplicate && duplicate.error().message.find("duplicate") != std::string::npos);
  write_yaml(fixture.fast_choose, original);
  for (const auto* duplicate_key : {"queue", "detector", "eso"}) {
    auto conflict = YAML::Clone(original_base);
    if (std::string(duplicate_key) == "queue")
      conflict["queue"]["maximum_age_s"] = 0.1;
    else if (std::string(duplicate_key) == "detector")
      conflict["detector"]["confidence"] = 0.7;
    else
      conflict["eso"]["linear_jerk_psd"] = 1;
    write_yaml(fixture.base, conflict);
    const auto rejected = pipeline::load_pipeline_config(fixture.base);
    CHECK(!rejected && rejected.error().message.find("Migrated key") != std::string::npos);
  }
  write_yaml(fixture.base, original_base);

  // 同次批量预加载保存同一规范路径和原始字节，后续文件编辑不改变既有快照。
  auto alias_base = YAML::Clone(original_base);
  CHECK(std::filesystem::create_directory(directory / "alias"));
  alias_base["fast_choose_file"] = "alias/../fast_choose.yaml";
  const auto alias_path = directory / "alias-base.yaml";
  write_yaml(alias_path, alias_base);
  const auto batch = pipeline::load_pipeline_configs({fixture.entry, fixture.base, alias_path});
  CHECK(batch && batch.value().size() == 3);
  const auto& bytes = batch.value()[0].configuration_snapshot->fast_choose->contents;
  for (const auto& config : batch.value()) {
    CHECK(config.configuration_snapshot->fast_choose->path == fixture.fast_choose);
    CHECK(config.configuration_snapshot->fast_choose->contents == bytes);
    CHECK(YAML::Dump(config.configuration_snapshot->effective) ==
          YAML::Dump(batch.value()[0].configuration_snapshot->effective));
  }
  auto changed = YAML::Clone(original);
  changed["common"]["publish_period_s"] = 0.02;
  write_yaml(fixture.fast_choose, changed);
  const auto reloaded = pipeline::load_pipeline_config(fixture.entry);
  CHECK(reloaded);
  CHECK_NEAR(reloaded.value().publish_period.value(), 0.02, 0);
  CHECK_NEAR(batch.value()[0].publish_period.value(), 0.01, 0);
  CHECK(reloaded.value().configuration_snapshot->fast_choose->contents != bytes);
  CHECK(pipeline::load_pipeline_configs({}).value().empty());
}
} // namespace

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
    CHECK(loaded.tracker.initial_covariance.rows() == 12);
    CHECK_NEAR(loaded.tracker.initial_covariance(9, 9), 1, 0);
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
    CHECK(loaded.configuration_snapshot && loaded.configuration_snapshot->fast_choose);
    const YAML::Node complete = YAML::Clone(loaded.configuration_snapshot->effective);
    write_yaml(temporary.path / "complete.yaml", complete);
    CHECK(pipeline::load_pipeline_config(temporary.path / "complete.yaml"));
    const auto reject = [&](const std::string& group, const std::string& key,
                            const YAML::Node& value) {
      auto yaml = YAML::Clone(complete);
      yaml[group][key] = value;
      const auto file = temporary.path / "invalid.yaml";
      {
        std::ofstream output(file);
        output << yaml;
        CHECK(output.good());
      }

      const auto result = pipeline::load_pipeline_config(file);
      CHECK(!result && result.error().code == core::ErrorCode::invalid_input);
      CHECK(result.error().message.find("Cannot open config") == std::string::npos);
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
    reject("tracker", "initial_variance", YAML::Load("[1, 1, 1, 1, 1, 1, 1, 1]"));
    reject("tracker", "initial_variance", YAML::Load("[1, 1, 1, 1, 1, 1, 1, 1, 1, 1]"));
    reject("tracker", "initial_variance", YAML::Load("[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1]"));
    reject("tracker", "initial_variance", YAML::Load("[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, .nan]"));
    reject("eso", "translation_bandwidth_radps", YAML::Load("0"));
    reject("eso", "angular_bandwidth_radps", YAML::Load(".nan"));
    reject("eso", "linear_jerk_psd", YAML::Load("-1"));

    auto legacy = YAML::Clone(complete);
    legacy.remove("eso");
    legacy["detector"].remove("plate_type");
    legacy["tracker"]["initial_variance"] = YAML::Load("[1, 2, 3, 4, 5, 6, 7, 8, 9]");
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
    CHECK(!old.value().configuration_snapshot->fast_choose);
    CHECK(old.value().configuration_snapshot->entry.path == old.value().configuration_snapshot->base.path);
    CHECK_NEAR(old.value().tracker.eso.translation_bandwidth_radps, 10, 0);
    CHECK_NEAR(old.value().tracker.eso.angular_bandwidth_radps, 10, 0);
    CHECK_NEAR(old.value().tracker.eso.linear_jerk_psd, 1, 0);
    for (int i = 0; i < 12; ++i)
      CHECK_NEAR(old.value().tracker.initial_covariance(i, i), i < 9 ? i + 1 : 1, 0);
    detection.raw_class_id = -1;
    detection.armor_size = vision::ArmorSize::unknown;
    CHECK(pipeline::plate_dimensions(old.value(), detection));
    legacy["plate_sizes"].push_back(
        YAML::Load("{plate_type: big, width_m: 0.23, height_m: 0.055}"));
    CHECK(!check_legacy());
    check_fast_choose(root, temporary.path / "fast-fixture");
  });
}
