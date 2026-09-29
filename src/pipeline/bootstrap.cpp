#include "autoaim/pipeline/bootstrap.hpp"
#include "autoaim/pipeline/frame_sync.hpp"

namespace autoaim::pipeline {
core::Result<PipelineConfig> load_pipeline_config(const std::filesystem::path& path) {
  using namespace core;
  using Result = core::Result<PipelineConfig>;
  try {
    auto loaded = Config::load(path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    const auto& c = loaded.value();
    if (c.require<std::string>("execution") != "replay")
      return Result::failure(ErrorCode::unavailable, "Live entry disabled: device calibration/authority evidence not provided");
    const auto file = [&](const std::string& key) { return (path.parent_path() / c.require<std::string>(key)).lexically_normal(); };
    const auto number = [&](const std::string& key) {
      const auto value = c.require<double>(key);
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite configuration: " + key);
      return value;
    };
    const auto seconds = [&](const std::string& key) { return Seconds(number(key)); };
    const auto radians = [&](const std::string& key) { return Radians(number(key)); };
    const auto size = [&](const std::string& key) { return c.require<std::size_t>(key); };
    const auto values = [&](const std::string& key, std::size_t count) {
      const auto result = c.require<std::vector<double>>(key);
      if (result.size() != count || !std::all_of(result.begin(), result.end(), [](double x) { return std::isfinite(x); }))
        throw std::invalid_argument("Invalid dimensions/values: " + key);
      return result;
    };
    const auto vector3 = [&](const std::string& key) { const auto v = values(key, 3); return Eigen::Vector3d(v[0], v[1], v[2]); };
    const auto covariance3 = [&](const std::string& key) { return Eigen::Matrix3d(vector3(key).asDiagonal()); };
    const auto identity_limits = [&](const std::string& prefix) {
      return estimation::IdentityLimits{size(prefix + ".observations"), number(prefix + ".margin"),
        number(prefix + ".maximum_cost"), number(prefix + ".previous_weight")};
    };
    // 纯函数算法没有构造校验阶段；其配置错误必须在启动时报告，不能等到有目标后静默无解。
    for (const char* key : {"pnp.maximum_rms_px", "pnp.maximum_corner_error_px", "pnp.minimum_edge_px",
        "pnp.minimum_view_cosine", "pnp.ambiguity_rms_gap_px", "pnp.pixel_sigma_px", "pnp.minimum_information_ratio",
        "pnp.prior_position_gate_m", "pnp.prior_rotation_gate_rad", "pnp.prior_maximum_age_s",
        "refinement.search_radius_px", "refinement.maximum_shift_px", "intercept.time_tolerance_s",
        "intercept.position_tolerance_m", "margin.sigma_multiplier", "margin.minimum_normal_speed_mps"})
      if (number(key) <= 0) throw std::invalid_argument(std::string("Nonpositive configuration: ") + key);
    for (const char* key : {"prediction.geometry_position_variance_m2", "prediction.geometry_rotation_variance_rad2",
        "prediction.unknown_velocity_sigma_mps", "impact.speed_sigma_mps", "impact.timing_sigma_s", "margin.reserved_edge_m"})
      if (number(key) < 0) throw std::invalid_argument(std::string("Negative configuration: ") + key);
    for (const char* key : {"impact.aim_variance_rad2", "impact.origin_variance_m2", "impact.scatter_variance_m2"})
      if (!math::covariance_valid(covariance3(key))) throw std::invalid_argument(std::string("Invalid covariance: ") + key);
    if (number("pnp.minimum_view_cosine") > 1 || number("pnp.minimum_information_ratio") >= 1 ||
        size("intercept.maximum_iterations") == 0 || size("intercept.maximum_iterations") > 64 ||
        size("refinement.minimum_pixels") < 3)
      throw std::invalid_argument("Invalid PnP/refinement/intercept limits");
    for (const char* key : {"refinement.brightness", "refinement.color_difference"})
      if (c.require<int>(key) < 0 || c.require<int>(key) > 255)
        throw std::invalid_argument(std::string("Invalid pixel threshold: ") + key);
    const auto role_name = c.require<std::string>("role");
    const Role role = role_name == "infantry" ? Role::infantry : role_name == "sentry" ? Role::sentry :
      throw std::invalid_argument("Unknown role");
    const auto mode_name = c.require<std::string>("initial_mode");
    // 步兵启动必须辅助；自动权限由新的操作事件启用，配置不能代替这一事件。
    const auto authority = role == Role::infantry ? mission::infantry_assist_authority() : mission::sentry_authority();
    if ((authority.mode == ControlMode::assist && mode_name != "assist") ||
        (authority.mode == ControlMode::automatic && mode_name != "automatic"))
      throw std::invalid_argument("Initial mode conflicts with role/explicit-enable contract");
    auto calibration_file = Config::load(file("calibration_file"));
    if (!calibration_file) throw std::invalid_argument(calibration_file.error().message);
    auto calibration = vision::load_calibration(calibration_file.value());
    if (!calibration) throw std::invalid_argument(calibration.error().message);
    std::vector<std::shared_ptr<const estimation::GeometryProfile>> profiles;
    for (const auto& name : c.require<std::vector<std::string>>("geometry_files")) {
      auto data = Config::load(path.parent_path() / name);
      if (!data) throw std::invalid_argument(data.error().message);
      auto profile = estimation::load_geometry(data.value());
      if (!profile) throw std::invalid_argument(profile.error().message);
      profiles.push_back(std::make_shared<const estimation::GeometryProfile>(std::move(profile).value()));
    }
    const auto enemy_name = c.require<std::string>("detector.enemy");
    const auto enemy = enemy_name == "red" ? vision::TeamColor::red : enemy_name == "blue" ? vision::TeamColor::blue :
      throw std::invalid_argument("Detector enemy must be red or blue");
    const auto detector_name = c.require<std::string>("detector.kind");
    vision::DetectorOptions detector = vision::TraditionalOptions{enemy, 1, 1, 1, 1, 1, 2, 2};
    if (detector_name == "traditional") {
      detector = vision::TraditionalOptions{enemy, c.require<int>("detector.brightness"), c.require<int>("detector.color_difference"),
        number("detector.minimum_light_length_px"), number("detector.minimum_light_ratio"), number("detector.minimum_pair_ratio"),
        number("detector.maximum_pair_ratio"), size("detector.maximum_lightbars")};
      vision::TraditionalDetector check(std::get<vision::TraditionalOptions>(detector));
    } else if (detector_name == "yolov5") {
      detector = vision::Yolov5Options{file("detector.model_path").string(), c.require<std::string>("detector.device"), enemy,
        number("detector.confidence"), number("detector.nms_iou"), size("detector.maximum_candidates"), size("detector.maximum_detections")};
      if (!std::filesystem::is_regular_file(std::get<vision::Yolov5Options>(detector).model_path))
        throw std::invalid_argument("Explicit model path does not exist");
    } else throw std::invalid_argument("Unsupported detector.kind");
    const auto cv_model = std::make_shared<const estimation::MotionModel>(number("motion.linear_accel_psd"),
      number("motion.angular_accel_psd"), seconds("motion.maximum_horizon_s"));
    const auto ca_model = std::make_shared<const estimation::MotionModel>(number("motion.linear_accel_psd"),
      number("motion.angular_jerk_psd"), seconds("motion.maximum_horizon_s"), number("motion.maximum_alpha_radps2"));
    estimation::StateCovariance initial = estimation::StateCovariance::Zero();
    const auto initial_diagonal = values("tracker.initial_variance", 9);
    for (int i = 0; i < 9; ++i) initial(i, i) = initial_diagonal[i];
    const auto nis_values = values("tracker.nis_limits", 6);
    std::array<double, 6> nis{}; std::copy(nis_values.begin(), nis_values.end(), nis.begin());
    estimation::TrackerOptions tracker{size("tracker.maximum_hypotheses"), initial,
      identity_limits("tracker.identity"), identity_limits("tracker.geometry"),
      {size("tracker.convergence.observations"), seconds("tracker.convergence.duration_s"), seconds("tracker.convergence.maximum_gap_s"),
       seconds("tracker.lost_after_s"), number("tracker.convergence.phase_enter_rad2"), number("tracker.convergence.phase_exit_rad2")},
      {size("tracker.health.maximum_rejections"), number("tracker.health.position_variance_m2"), number("tracker.health.phase_variance_rad2")},
      {number("motion.rotation_threshold_radps"), number("motion.acceleration_enter_radps2"), number("motion.acceleration_exit_radps2"),
       size("motion.selection_observations"), seconds("motion.minimum_dwell_s"), seconds("motion.maximum_gap_s")},
      estimation::NisGate(nis), seconds("tracker.geometry_dwell_s"), number("tracker.geometry_complexity_penalty"),
      number("tracker.association_margin"), number("tracker.initial_alpha_variance"), c.require<std::string>("configuration_id")};
    estimation::TrackerSetOptions targets{size("targets.maximum_count"), number("targets.association_margin"),
      Metres(number("targets.maximum_same_target_separation_m"))};
    QueueOptions queue{size("queue.pool_capacity"), size("queue.pending_capacity"), size("queue.in_flight_limit"),
      calibration.value().width(), calibration.value().height(), static_cast<std::size_t>(calibration.value().width()) * 3,
      seconds("queue.maximum_age_s")};
    vision::PnpQualityOptions pnp{number("pnp.maximum_rms_px"), number("pnp.maximum_corner_error_px"), number("pnp.minimum_edge_px"),
      number("pnp.minimum_view_cosine"), number("pnp.ambiguity_rms_gap_px"), number("pnp.pixel_sigma_px"), number("pnp.minimum_information_ratio"),
      Metres(number("pnp.prior_position_gate_m")), radians("pnp.prior_rotation_gate_rad"), seconds("pnp.prior_maximum_age_s")};
    std::map<int, vision::PlateDimensions> sizes;
    for (const auto& node : c.require<std::vector<YAML::Node>>("plate_sizes")) {
      if (!sizes.emplace(node["class_id"].as<int>(), vision::PlateDimensions(Metres(node["width_m"].as<double>()),
          Metres(node["height_m"].as<double>()))).second) throw std::invalid_argument("Duplicate plate class size");
    }
    if (sizes.empty()) throw std::invalid_argument("Explicit plate dimensions required");
    const auto indices = c.require<std::vector<std::size_t>>("corners.indices");
    if (indices.size() != 4) throw std::invalid_argument("Corner mapping requires four indices");
    vision::PoseCovariance additional = vision::PoseCovariance::Zero();
    const auto additional_diagonal = values("pnp.additional_pose_variance", 6);
    for (int i = 0; i < 6; ++i) additional(i, i) = additional_diagonal[i];
    const auto reference = values("aim_reference_to_world_wxyz", 4);
    const auto tolerance_yaw = radians("safety.yaw_tolerance_rad"), tolerance_pitch = radians("safety.pitch_tolerance_rad");
    PipelineConfig result{file("input_manifest"), role, c.require<bool>("program_fire_requested"), detector,
      std::move(calibration).value(), profiles, cv_model, ca_model, tracker, targets, queue,
      size("history.capacity"), seconds("history.maximum_gap_s"), pnp, sizes,
      vision::CornerMapping({indices[0], indices[1], indices[2], indices[3]}, c.require<std::string>("corners.model_id"),
        c.require<std::string>("corners.mapping_id"), Evidence::declared()),
      {c.require<int>("refinement.brightness"), c.require<int>("refinement.color_difference"), number("refinement.search_radius_px"),
        number("refinement.maximum_shift_px"), size("refinement.minimum_pixels")}, c.require<bool>("refinement.enabled"), additional,
      {number("prediction.geometry_position_variance_m2"), number("prediction.geometry_rotation_variance_rad2"), number("prediction.unknown_velocity_sigma_mps")},
      decision::BallisticModel(vector3("ballistic.gravity_mps2"), seconds("ballistic.maximum_flight_s"), Metres(number("ballistic.minimum_range_m"))),
      {size("intercept.maximum_iterations"), seconds("intercept.time_tolerance_s"), Metres(number("intercept.position_tolerance_m"))},
      {number("armor.minimum_incidence_cosine"), number("armor.direction_weight"), number("armor.flight_time_weight"),
        number("armor.position_variance_weight"), number("armor.switch_margin"), seconds("armor.minimum_dwell_s")},
      {number("impact.speed_sigma_mps"), covariance3("impact.aim_variance_rad2"), covariance3("impact.origin_variance_m2"),
        number("impact.timing_sigma_s"), covariance3("impact.scatter_variance_m2")},
      {number("margin.sigma_multiplier"), Metres(number("margin.reserved_edge_m")), number("margin.minimum_normal_speed_mps")},
      {tolerance_yaw, tolerance_pitch, size("safety.settled_observations"), seconds("safety.settled_duration_s"), seconds("safety.maximum_gap_s"),
        seconds("safety.fire_age_s"), seconds("safety.feedback_age_s")},
      {seconds("targets.maximum_source_age_s"), seconds("targets.lost_timeout_s"), number("targets.maximum_position_variance_m2")},
      {seconds("safety.operator_age_s"), c.require<std::string>("device_id"), c.require<std::string>("configuration_id")},
      {{seconds("safety.control_age_s"), seconds("safety.fire_age_s"), seconds("safety.feedback_age_s")}, c.require<std::string>("device_id"),
        c.require<std::string>("configuration_id"), tolerance_yaw, tolerance_pitch},
      {radians("smoother.maximum_correction_rad"), number("smoother.maximum_rate_radps"), seconds("smoother.time_constant_s"),
        radians("smoother.enter_rad"), radians("smoother.leave_rad"), seconds("smoother.minimum_dwell_s")},
      seconds("publish_period_s"), seconds("intent_lifetime_s"), seconds("after_send_delay_s"),
      math::Point3<math::WorldFrame>(vector3("launch_origin_world_m")),
      math::checked_rotation(Eigen::Quaterniond(reference[0], reference[1], reference[2], reference[3])), Evidence::declared()};
    // 利用各模块构造校验器保持约束唯一；这些对象不会开启线程或硬件。
    FrameQueue check_queue(queue, 1);
    FrameSync check_history(result.history_capacity, result.history_maximum_gap, 1);
    estimation::TrackerSet check_tracker(1, profiles, cv_model, ca_model, tracker, targets);
    decision::ArmorSelector check_armor(result.armor_selection);
    decision::AimAdequacy check_aim(result.adequacy);
    mission::TargetSelector check_target(result.target_selection);
    mission::InfantryMission check_mode(result.infantry, 1, TimePoint(0, ClockDomain::replay));
    control::CommandGuard check_guard(result.guard);
    control::CorrectionSmoother check_smoother(result.smoother);
    if (!math::covariance_valid(additional) || result.publish_period.value() < 1e-6 ||
        result.publish_period.value() > 60 || result.intent_lifetime.value() <= 0 || result.after_send_delay.value() < 0)
      throw std::invalid_argument("Invalid pipeline timing/covariance configuration");
    return Result::success(std::move(result));
  } catch (const std::exception& error) { return Result::failure(ErrorCode::invalid_input, error.what()); }
}
}  // namespace autoaim::pipeline
