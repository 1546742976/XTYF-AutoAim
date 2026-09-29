#include "autoaim/pipeline/run_metadata.hpp"
#include "autoaim/pipeline/pipeline.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>

namespace autoaim::pipeline {
namespace {
YAML::Node describe_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::invalid_argument("Cannot read run provenance file: " + path.string());
  std::uint64_t hash = 14695981039346656037ull;
  std::array<char, 65536> buffer;
  std::uint64_t bytes = 0;
  while (input.read(buffer.data(), buffer.size()) || input.gcount()) {
    for (std::streamsize i = 0; i < input.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(buffer[std::size_t(i)]);
      hash *= 1099511628211ull;
    }
    bytes += std::uint64_t(input.gcount());
  }
  if (!input.eof())
    throw std::runtime_error("Run provenance file read failed");
  std::ostringstream digest;
  digest << std::hex << std::setw(16) << std::setfill('0') << hash;
  YAML::Node node;
  node["path"] = std::filesystem::absolute(path).string();
  node["bytes"] = bytes;
  node["fnv1a64"] = digest.str();

  return node;
}

YAML::Node quaternion(const Eigen::Quaterniond& q) {
  return YAML::Node(std::vector<double>{q.w(), q.x(), q.y(), q.z()});
}

YAML::Node describe_calibration_report(const core::Config& config,
    const std::filesystem::path& base, const char* key, const core::Evidence& evidence,
    const PipelineConfig& loaded) {
  YAML::Node node;
  node["file"] = config.contains(key) ? describe_file(base / config.require<std::string>(key)) :
                                      YAML::Node(YAML::NodeType::Null);
  const char* level = "missing";
  switch (evidence.level()) {
  case core::EvidenceLevel::missing: break;
  case core::EvidenceLevel::declared: level = "declared"; break;
  case core::EvidenceLevel::measured: level = "measured"; break;
  case core::EvidenceLevel::simulation: level = "simulation"; break;
  }
  node["evidence_level"] = level;
  node["qualification_domain"] = "replay";
  node["qualified"] = evidence.qualifies(core::ClockDomain::replay,
      loaded.guard.device_id, loaded.guard.configuration_id);
  node["provenance"] = YAML::Node(YAML::NodeType::Null);
  if (const auto* report = evidence.report()) {
    const auto& provenance = report->provenance();
    node["provenance"]["device_id"] = provenance.device_id;
    node["provenance"]["configuration_id"] = provenance.configuration_id;
    node["provenance"]["date"] = provenance.date;
    node["provenance"]["method"] = provenance.method;
  }

  return node;
}

YAML::Node timing(const WallTimeSummary& value) {
  YAML::Node node;
  node["samples"] = value.samples;
  node["total_ms"] = value.total_ms;
  node["mean_ms"] = value.samples ? YAML::Node(value.total_ms / value.samples) :
                                   YAML::Node(YAML::NodeType::Null);
  node["maximum_ms"] = value.samples ? YAML::Node(value.maximum_ms) :
                                      YAML::Node(YAML::NodeType::Null);

  return node;
}
} // namespace

YAML::Node describe_run(const std::filesystem::path& configuration, const PipelineConfig& loaded) {
  YAML::Node node;
  node["run_metadata_schema_version"] = 2;
  node["configuration_file"] = describe_file(configuration);
  const auto config = YAML::LoadFile(configuration.string());
  node["configuration"] = config;
  node["device_id"] = loaded.guard.device_id;
  node["configuration_id"] = loaded.guard.configuration_id;
  node["input_manifest"] = std::filesystem::absolute(loaded.input_manifest).string();
  const auto calibration_file =
      configuration.parent_path() / config["calibration_file"].as<std::string>();
  node["calibration_file"] = describe_file(calibration_file);
  const auto calibration_config = core::Config::load(calibration_file);
  if (!calibration_config)
    throw std::invalid_argument(calibration_config.error().message);
  for (const auto& file : config["geometry_files"])
    node["geometry_files"].push_back(describe_file(
        configuration.parent_path() / file.as<std::string>()));
  const auto& calibration = loaded.calibration;
  node["calibration_reports"]["intrinsics"] = describe_calibration_report(
      calibration_config.value(), calibration_file.parent_path(),
      "calibration.intrinsic_report_file", calibration.intrinsic_evidence(), loaded);
  node["calibration_reports"]["extrinsics"] = describe_calibration_report(
      calibration_config.value(), calibration_file.parent_path(),
      "calibration.extrinsic_report_file", calibration.extrinsic_evidence(), loaded);
  node["calibration_parameters"]["width"] = calibration.width();
  node["calibration_parameters"]["height"] = calibration.height();
  node["calibration_parameters"]["roi_offset"] =
      std::vector<int>{calibration.roi_offset()[0], calibration.roi_offset()[1]};
  node["calibration_parameters"]["camera_matrix"] =
      std::vector<double>(calibration.intrinsic().val, calibration.intrinsic().val + 9);
  node["calibration_parameters"]["distortion"] = calibration.distortion();
  const auto& transform = calibration.camera_to_gimbal().value();
  node["calibration_parameters"]["camera_to_gimbal_quaternion_wxyz"] =
      quaternion(transform.rotation());
  const auto& t = transform.translation();
  node["calibration_parameters"]["camera_to_gimbal_translation_m"] =
      std::vector<double>{t.x(), t.y(), t.z()};

  if (const auto* yolo = std::get_if<vision::YoloOptions>(&loaded.detector)) {
    node["detector"] = yolo->format == vision::YoloFormat::armor_v11 ? "yolo11" : "yolov5";
    const std::filesystem::path model(yolo->model_path);
    node["model_file"] = describe_file(model);
    if (model.extension() == ".xml") {
      auto weights = model;
      weights.replace_extension(".bin");
      node["model_weights"] = describe_file(weights);
    }
  } else
    node["detector"] = "traditional";

  return node;
}

YAML::Node timing_report(const PipelineMetrics& metrics) {
  YAML::Node measured;
  measured["clock"] = "host steady clock; independent of replay logical time";
  measured["pool_copy"] = timing(metrics.queue_timings.pool_copy);
  measured["queue_wait"] = timing(metrics.queue_timings.waiting);
  measured["preprocessing"] = timing(metrics.preprocessing);
  measured["inference_wall"] = timing(metrics.inference_wall);
  measured["postprocessing"] = timing(metrics.postprocessing);

  return measured;
}
} // namespace autoaim::pipeline
