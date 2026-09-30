#pragma once

#include "autoaim/pipeline/bootstrap.hpp"
#include <fstream>
#include <iterator>
#include <set>
#include <utility>

namespace autoaim::pipeline::config_detail {
using Fields = std::set<std::string>;

inline std::filesystem::path absolute_path(const std::filesystem::path& directory,
                                         const std::filesystem::path& value) {
  return std::filesystem::weakly_canonical(std::filesystem::absolute(directory / value));
}

inline std::string nonempty_string(const YAML::Node& node, const std::string& name) {
  const auto value = node.as<std::string>();
  if (value.empty())
    throw std::invalid_argument("Empty configuration path/string: " + name);
  return value;
}

inline void add_fields(Fields& fields, const std::string& prefix,
                       std::initializer_list<const char*> names) {
  for (const auto* name : names)
    fields.insert(prefix.empty() ? name : prefix + "." + name);
}

inline Fields common_fields() {
  Fields fields;
  add_fields(fields, "", {"input_manifest", "program_fire_requested", "publish_period_s",
                          "intent_lifetime_s", "after_send_delay_s"});
  add_fields(fields, "operator_input", {"button_mode"});
  add_fields(fields, "detector", {"enemy"});
  add_fields(fields, "refinement", {"enabled", "brightness", "color_difference",
                                    "search_radius_px", "maximum_shift_px", "minimum_pixels"});
  add_fields(fields, "queue", {"pool_capacity", "pending_capacity", "in_flight_limit",
                               "maximum_age_s"});
  add_fields(fields, "history", {"capacity", "maximum_gap_s"});
  add_fields(fields, "pnp", {"maximum_rms_px", "maximum_corner_error_px", "minimum_edge_px",
                             "minimum_view_cosine", "ambiguity_rms_gap_px", "pixel_sigma_px",
                             "minimum_information_ratio", "prior_position_gate_m",
                             "prior_rotation_gate_rad", "prior_maximum_age_s",
                             "additional_pose_variance"});
  add_fields(fields, "motion", {"linear_accel_psd", "angular_accel_psd", "angular_jerk_psd",
                                "maximum_horizon_s", "maximum_alpha_radps2",
                                "rotation_threshold_radps", "acceleration_enter_radps2",
                                "acceleration_exit_radps2", "selection_observations",
                                "minimum_dwell_s", "maximum_gap_s"});
  add_fields(fields, "tracker", {"maximum_hypotheses", "initial_variance", "nis_limits",
                                 "lost_after_s", "geometry_dwell_s", "geometry_complexity_penalty",
                                 "association_margin", "initial_alpha_variance"});
  for (const char* group : {"tracker.identity", "tracker.geometry"})
    add_fields(fields, group, {"observations", "margin", "maximum_cost", "previous_weight"});
  add_fields(fields, "tracker.convergence", {"observations", "duration_s", "maximum_gap_s",
                                             "phase_enter_rad2", "phase_exit_rad2"});
  add_fields(fields, "tracker.health", {"maximum_rejections", "position_variance_m2",
                                        "phase_variance_rad2"});
  add_fields(fields, "targets", {"maximum_count", "association_margin",
                                 "maximum_same_target_separation_m", "maximum_source_age_s",
                                 "lost_timeout_s", "maximum_position_variance_m2"});
  add_fields(fields, "prediction", {"geometry_position_variance_m2",
                                    "geometry_rotation_variance_rad2", "unknown_velocity_sigma_mps"});
  add_fields(fields, "ballistic", {"maximum_flight_s", "minimum_range_m"});
  add_fields(fields, "intercept", {"maximum_iterations", "time_tolerance_s", "position_tolerance_m"});
  add_fields(fields, "armor", {"minimum_incidence_cosine", "direction_weight", "flight_time_weight",
                               "position_variance_weight", "switch_margin", "minimum_dwell_s"});
  add_fields(fields, "impact", {"speed_sigma_mps", "aim_variance_rad2", "origin_variance_m2",
                                "timing_sigma_s", "scatter_variance_m2"});
  add_fields(fields, "margin", {"sigma_multiplier", "reserved_edge_m", "minimum_normal_speed_mps"});
  add_fields(fields, "safety", {"control_age_s", "fire_age_s", "feedback_age_s", "operator_age_s",
                                "yaw_tolerance_rad", "pitch_tolerance_rad", "settled_observations",
                                "settled_duration_s", "maximum_gap_s"});
  add_fields(fields, "smoother", {"maximum_correction_rad", "maximum_rate_radps", "time_constant_s",
                                  "enter_rad", "leave_rad", "minimum_dwell_s"});
  return fields;
}

inline void collect_fields(const YAML::Node& node, const Fields& allowed, Fields& present,
                           const std::string& prefix, const std::string& context) {
  if (!node.IsMap())
    throw std::invalid_argument("Expected configuration map: " + context + prefix);
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    const auto path = prefix.empty() ? key : prefix + "." + key;
    if (allowed.count(path)) {
      if (entry.second.IsMap() || entry.second.IsNull())
        throw std::invalid_argument("Invalid configuration leaf: " + context + path);
      present.insert(path);
    } else {
      const auto descendant = allowed.lower_bound(path + ".");
      if (descendant == allowed.end() || descendant->compare(0, path.size() + 1, path + ".") != 0)
        throw std::invalid_argument("Unknown fast_choose key: " + context + path);
      collect_fields(entry.second, allowed, present, path, context);
    }
  }
}

inline void exact_fields(const YAML::Node& node, const Fields& allowed, const std::string& context) {
  Fields present;
  collect_fields(node, allowed, present, "", context);
  for (const auto& path : allowed)
    if (!present.count(path))
      throw std::invalid_argument("Missing fast_choose key: " + context + path);
}

inline void exact_map_keys(const YAML::Node& node, const Fields& allowed,
                           const std::string& context) {
  if (!node.IsMap())
    throw std::invalid_argument("Expected configuration map: " + context);
  for (const auto& entry : node)
    if (!allowed.count(entry.first.as<std::string>()))
      throw std::invalid_argument("Unknown fast_choose key: " + context + entry.first.as<std::string>());
  for (const auto& key : allowed)
    if (!node[key].IsDefined())
      throw std::invalid_argument("Missing fast_choose key: " + context + key);
}

inline const Fields& eso_fields() {
  static const Fields fields{"translation_bandwidth_radps", "angular_bandwidth_radps",
                              "linear_jerk_psd"};
  return fields;
}

inline void validate_eso(const YAML::Node& node) {
  exact_fields(node, eso_fields(), "eso.");
  for (const auto& key : eso_fields()) {
    const double value = node[key].as<double>();
    if (!std::isfinite(value) || (key == "linear_jerk_psd" ? value < 0 : value <= 0))
      throw std::invalid_argument("Invalid ESO configuration: eso." + key);
  }
}

inline bool detector_kind(const std::string& kind) {
  return kind == "traditional" || kind == "yolov5" || kind == "yolo11";
}

inline void validate_fast(const YAML::Node& root) {
  exact_map_keys(root, {"fast_choose_version", "active_detector", "common", "detectors", "eso"}, "");
  if (root["fast_choose_version"].as<int>() != 1)
    throw std::invalid_argument("Unsupported fast_choose_version");
  if (!detector_kind(root["active_detector"].as<std::string>()))
    throw std::invalid_argument("Unknown active_detector");
  exact_fields(root["common"], common_fields(), "common.");
  exact_map_keys(root["detectors"], {"traditional", "yolov5", "yolo11"}, "detectors.");
  for (const char* name : {"traditional", "yolov5", "yolo11"}) {
    const auto group = root["detectors"][name];
    const std::string context = std::string("detectors.") + name + ".";
    Fields allowed{"config_file"};
    if (std::string(name) == "traditional") {
      add_fields(allowed, "", {"brightness", "color_difference", "minimum_light_length_px",
                               "minimum_light_ratio", "minimum_pair_ratio", "maximum_pair_ratio",
                               "maximum_lightbars"});
      exact_fields(group, allowed, context);
      // 验证纯参数，不创建推理请求，也不打开未选配置/模型。
      vision::TraditionalDetector check({vision::TeamColor::red, group["brightness"].as<int>(),
          group["color_difference"].as<int>(), group["minimum_light_length_px"].as<double>(),
          group["minimum_light_ratio"].as<double>(), group["minimum_pair_ratio"].as<double>(),
          group["maximum_pair_ratio"].as<double>(), group["maximum_lightbars"].as<std::size_t>()});
    } else {
      add_fields(allowed, "", {"model_path", "device", "confidence", "nms_iou",
                               "maximum_candidates", "maximum_detections"});
      exact_fields(group, allowed, context);
      nonempty_string(group["model_path"], context + "model_path");
      nonempty_string(group["device"], context + "device");
      const double confidence = group["confidence"].as<double>();
      const double nms = group["nms_iou"].as<double>();
      const auto candidates = group["maximum_candidates"].as<std::size_t>();
      const auto detections = group["maximum_detections"].as<std::size_t>();
      if (!std::isfinite(confidence) || confidence <= 0 || confidence >= 1 ||
          !std::isfinite(nms) || nms <= 0 || nms >= 1 || candidates == 0 || detections == 0 ||
          detections > candidates)
        throw std::invalid_argument("Invalid fast_choose YOLO limits: " + context);
    }
    nonempty_string(group["config_file"], context + "config_file");
  }
  validate_eso(root["eso"]);
}

inline void absolute_paths(YAML::Node node, const std::filesystem::path& owner) {
  const auto base = owner.parent_path();
  for (const char* key : {"input_manifest", "calibration_file"})
    if (const YAML::Node value = std::as_const(node)[key]; value.IsDefined())
      node[key] = absolute_path(base, nonempty_string(value, key)).string();
  if (const YAML::Node geometries = std::as_const(node)["geometry_files"]; geometries.IsDefined()) {
    if (!geometries.IsSequence())
      throw std::invalid_argument("geometry_files must be a sequence");
    for (std::size_t i = 0; i < geometries.size(); ++i)
      node["geometry_files"][i] = absolute_path(
          base, nonempty_string(geometries[i], "geometry_files")).string();
  }
  const YAML::Node detector = std::as_const(node)["detector"];
  if (detector.IsDefined() && detector.IsMap() && detector["model_path"].IsDefined())
    node["detector"]["model_path"] = absolute_path(
        base, nonempty_string(detector["model_path"], "detector.model_path")).string();
}

inline void merge_unique(YAML::Node target, const YAML::Node& source, const std::string& prefix = "") {
  for (const auto& entry : source) {
    const auto key = entry.first.as<std::string>();
    const auto name = prefix.empty() ? key : prefix + "." + key;
    const YAML::Node existing = std::as_const(target)[key];
    if (entry.second.IsMap()) {
      if (existing.IsDefined() && !existing.IsMap())
        throw std::invalid_argument("Base conflicts with fast_choose: " + name);
      if (!existing.IsDefined())
        target[key] = YAML::Node(YAML::NodeType::Map);
      merge_unique(target[key], entry.second, name);
    } else {
      if (existing.IsDefined())
        throw std::invalid_argument("Migrated key occurs in base and fast_choose: " + name);
      target[key] = YAML::Clone(entry.second);
    }
  }
}

inline bool contains_path(const YAML::Node& root, const std::string& path) {
  YAML::Node cursor = root;
  std::size_t begin = 0;
  for (;;) {
    const auto end = path.find('.', begin);
    const YAML::Node view = cursor;
    if (!view.IsMap())
      return false;
    const YAML::Node next = view[path.substr(begin, end == std::string::npos ? end : end - begin)];
    if (!next.IsDefined())
      return false;
    if (end == std::string::npos)
      return true;
    cursor.reset(next);
    begin = end + 1;
  }
}

inline void reject_migrated_base_fields(const YAML::Node& base) {
  Fields fields = common_fields();
  add_fields(fields, "detector", {"brightness", "color_difference", "minimum_light_length_px",
                                  "minimum_light_ratio", "minimum_pair_ratio", "maximum_pair_ratio",
                                  "maximum_lightbars", "model_path", "device", "confidence",
                                  "nms_iou", "maximum_candidates", "maximum_detections"});
  for (const auto& key : eso_fields())
    fields.insert("eso." + key);
  for (const auto& path : fields)
    if (contains_path(base, path))
      throw std::invalid_argument("Migrated key occurs in base and fast_choose: " + path);
}

struct Document {
  ConfigurationFileSnapshot snapshot;
  YAML::Node parsed;
};

// 单次装载拥有缓存；所有实际解析和记录的指纹使用同一份 binary 原始字节。
class Composition {
public:
  std::shared_ptr<const PipelineConfigurationSnapshot> resolve(const std::filesystem::path& path) {
    const auto& entry = document(path);
    const Document* base = &entry;
    const Document* fast = nullptr;
    const YAML::Node entry_node = entry.parsed;
    if (entry_node["fast_choose_version"].IsDefined()) {
      fast = &entry;
      validate_cached(*fast);
      const auto kind = entry_node["active_detector"].as<std::string>();
      base = &document(absolute_path(entry.snapshot.path.parent_path(),
          entry_node["detectors"][kind]["config_file"].as<std::string>()));
      const YAML::Node base_node = base->parsed;
      if (base_node["fast_choose_version"].IsDefined() || !base_node["detector"].IsMap() ||
          base_node["detector"]["kind"].as<std::string>() != kind ||
          !base_node["fast_choose_file"].IsDefined() ||
          absolute_path(base->snapshot.path.parent_path(),
              base_node["fast_choose_file"].as<std::string>()) != fast->snapshot.path)
        throw std::invalid_argument("Selected base kind/back-reference does not match fast_choose");
    } else if (entry_node["fast_choose_file"].IsDefined()) {
      fast = &document(absolute_path(entry.snapshot.path.parent_path(),
          nonempty_string(entry_node["fast_choose_file"], "fast_choose_file")));
      validate_cached(*fast);
    }
    YAML::Node effective = YAML::Clone(base->parsed);
    absolute_paths(effective, base->snapshot.path);
    if (fast) {
      reject_migrated_base_fields(effective);
      const YAML::Node fast_node = fast->parsed;
      const auto kind = std::as_const(effective)["detector"]["kind"].as<std::string>();
      if (!detector_kind(kind))
        throw std::invalid_argument("Unknown base detector.kind");
      YAML::Node migrated = YAML::Clone(fast_node["common"]);
      YAML::Node selected = YAML::Clone(fast_node["detectors"][kind]);
      selected.remove("config_file");
      merge_unique(migrated["detector"], selected, "detector");
      migrated["eso"] = YAML::Clone(fast_node["eso"]);
      absolute_paths(migrated, fast->snapshot.path);
      merge_unique(effective, migrated);
      effective.remove("fast_choose_file");
    }
    return std::make_shared<const PipelineConfigurationSnapshot>(PipelineConfigurationSnapshot{
        entry.snapshot, base->snapshot,
        fast ? std::optional<ConfigurationFileSnapshot>(fast->snapshot) : std::nullopt,
        std::move(effective)});
  }

private:
  const Document& document(const std::filesystem::path& path) {
    const auto canonical = absolute_path({}, path);
    const auto existing = documents_.find(canonical);
    if (existing != documents_.end())
      return existing->second;
    std::ifstream input(canonical, std::ios::binary);
    if (!input)
      throw std::invalid_argument("Cannot open config: " + canonical.string());
    const std::string contents{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (input.bad())
      throw std::invalid_argument("Cannot read config: " + canonical.string());
    const auto checked = core::Config::parse(contents);
    if (!checked)
      throw std::invalid_argument(canonical.string() + ": " + checked.error().message);
    return documents_.emplace(canonical, Document{{canonical, contents}, YAML::Load(contents)})
        .first->second;
  }

  void validate_cached(const Document& document) {
    if (validated_.insert(document.snapshot.path).second)
      validate_fast(document.parsed);
  }

  std::map<std::filesystem::path, Document> documents_;
  std::set<std::filesystem::path> validated_;
};
} // namespace autoaim::pipeline::config_detail
