#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/pipeline/bootstrap.hpp"
#include <opencv2/calib3d.hpp>
#include "autoaim/vision/calibration_report.hpp"
#include <fstream>
#include <iostream>
#include <set>

namespace autoaim::pipeline {
int run_calibration_tool(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--solve-intrinsics" || std::string(argv[i]) == "--solve-hand-eye")
      return run_calibration_solver(argc, argv);

  try {
    std::filesystem::path config_path, corners_path;
    bool self_test = false;

    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);

      if (argument == "--help") {
        std::cout << "calibration_tool --solve-intrinsics DATA.yaml --output NEW_DIR\n"
                     "calibration_tool --solve-hand-eye DATA.yaml --intrinsics FILE"
                     " --output NEW_DIR\n"
                     "calibration_tool --config FILE (--corners YAML | --self-test)\nChecks "
                     "explicit calibration and IPPE candidates; does not calibrate hardware.\n";

        return 0;
      }

      if (argument == "--self-test") {
        self_test = true;
        continue;
      }

      if (++i == argc)
        throw std::invalid_argument("Missing argument");

      if (argument == "--config")
        config_path = argv[i];
      else if (argument == "--corners")
        corners_path = argv[i];
      else
        throw std::invalid_argument("Unknown argument");
    }

    auto loaded = load_pipeline_config(config_path);

    if (!loaded)
      throw std::invalid_argument(loaded.error().message);

    const auto& config = loaded.value();
    int class_id = config.plate_sizes.empty() ? -1 : config.plate_sizes.begin()->first;
    vision::Detection detection{{}, 1, vision::TeamColor::red, class_id, false};
    if (!config.typed_plate_sizes.empty())
      detection.armor_size = config.typed_plate_sizes.begin()->first;

    if (self_test) {
      const auto corners = vision::object_corners(*plate_dimensions(config, detection));
      const auto q =
          Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY())) *
          Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()));

      const auto matrix = q.toRotationMatrix();
      cv::Matx33d rotation;

      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          rotation(r, c) = matrix(r, c);

      cv::Vec3d rvec;
      cv::Rodrigues(rotation, rvec);
      std::vector<cv::Point2d> projected;
      cv::projectPoints(std::vector<cv::Point3d>(corners.begin(), corners.end()), rvec,
                        cv::Vec3d(0.1, 0.05, 3), config.calibration.intrinsic(),
                        config.calibration.distortion(), projected);

      for (std::size_t k = 0; k < 4; ++k)
        detection.corners[k] = projected[k];
    } else {
      auto data = core::Config::load(corners_path);

      if (!data)
        throw std::invalid_argument(data.error().message);

      if (config.typed_plate_sizes.empty())
        detection.raw_class_id = data.value().require<int>("class_id");
      else
        detection.armor_size =
            vision::parse_armor_size(data.value().require<std::string>("plate_type"));
      const auto corners =
          data.value().require<std::vector<std::vector<double>>>("corners_tl_tr_br_bl");

      if (corners.size() != 4)
        throw std::invalid_argument("Four ordered corners required");

      for (std::size_t k = 0; k < 4; ++k) {
        if (corners[k].size() != 2)
          throw std::invalid_argument("Corner requires x,y pixels");

        detection.corners[k] =
            cv::Point2f(static_cast<float>(corners[k][0]), static_cast<float>(corners[k][1]));
      }
    }

    const auto* dimensions = plate_dimensions(config, detection);

    if (!dimensions)
      throw std::invalid_argument("No explicit dimensions for detection plate type");

    const auto result = vision::solve_pose(
        core::Stamp(1, 1, core::TimePoint(1, core::ClockDomain::replay)), detection,
        *dimensions, config.calibration, config.pnp, config.guard.device_id,
        config.guard.configuration_id, std::nullopt, std::nullopt);

    std::cout << "valid=" << result.pose_valid << " reliable=" << result.pose_reliable
              << " ambiguous=" << result.ambiguous << " candidates=" << result.candidates.size()
              << '\n';

    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const auto& p = result.candidates[i];
      std::cout << i << " xyz_m=" << p.plate_to_camera.value().translation().transpose()
                << " rms_px=" << p.rms_px << " max_px=" << p.maximum_error_px
                << " view_cos=" << p.view_cosine << " covariance=" << p.covariance.has_value()
                << '\n';
    }

    if (!result.pose_valid)
      return 2;

    if (self_test && (result.pose_reliable ||
                      (result.candidates[*result.selected].plate_to_camera.value().translation() -
                       Eigen::Vector3d(0.1, 0.05, 3))
                              .norm() > 1e-4))
      throw std::runtime_error("Synthetic PnP check failed");

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';

    return 1;
  }
}

namespace {
void write_yaml(const std::filesystem::path& path, const YAML::Node& node) {
  if (std::filesystem::exists(path))
    throw std::invalid_argument("Refusing to overwrite calibration output");

  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << node;
  std::ofstream output(path);
  output << emitter.c_str() << '\n';
  output.flush();

  if (!emitter.good() || !output)
    throw std::runtime_error("Cannot write calibration output");
}

vision::IntrinsicSolution read_intrinsics(const core::Config& c) {
  vision::IntrinsicSolution value{{c.require<int>("calibration.width"),
                                   c.require<int>("calibration.height")}, {},
                                  c.require<std::vector<double>>("calibration.distortion"),
                                  0, {}, {}};
  const auto matrix = c.require<std::vector<double>>("calibration.camera_matrix");

  if (matrix.size() != 9)
    throw std::invalid_argument("Intrinsic matrix must contain nine values");

  std::copy(matrix.begin(), matrix.end(), value.matrix.val);
  if (c.contains("calibration.roi_offset")) {
    const auto roi = c.require<std::vector<int>>("calibration.roi_offset");

    if (roi.size() != 2 || roi[0] < 0 || roi[1] < 0)
      throw std::invalid_argument("Invalid intrinsic ROI offset");
    value.roi_offset = {roi[0], roi[1]};
  }

  return value;
}
} // namespace

int run_calibration_solver(int argc, char** argv) {
  try {
    std::filesystem::path dataset_path, intrinsic_path, output_directory;
    std::string operation;
    std::set<std::string> seen;

    for (int i = 1; i < argc; ++i) {
      const std::string key(argv[i]);

      if (!seen.insert(key).second || i + 1 == argc)
        throw std::invalid_argument("Repeated or incomplete calibration argument");

      const std::filesystem::path value(argv[++i]);

      if (key == "--solve-intrinsics" || key == "--solve-hand-eye") {
        if (!operation.empty())
          throw std::invalid_argument("Choose exactly one calibration operation");
        operation = key;
        dataset_path = value;
      } else if (key == "--intrinsics")
        intrinsic_path = value;
      else if (key == "--output")
        output_directory = value;
      else
        throw std::invalid_argument("Unknown calibration argument: " + key);
    }

    const bool hand_eye = operation == "--solve-hand-eye";

    if (dataset_path.empty() || output_directory.empty() ||
        (hand_eye == intrinsic_path.empty()))
      throw std::invalid_argument("Dataset/output required; only hand-eye needs --intrinsics");
    if (std::filesystem::exists(output_directory))
      throw std::invalid_argument("Calibration output directory must not exist");

    const auto loaded = core::Config::load(dataset_path);

    if (!loaded)
      throw std::invalid_argument(loaded.error().message);

    const auto& c = loaded.value();
    const auto data = vision::load_calibration_dataset(dataset_path);
    const core::MeasurementProvenance provenance{
        c.require<std::string>("provenance.device_id"),
        c.require<std::string>("provenance.configuration_id"),
        c.require<std::string>("provenance.date"),
        hand_eye ? "OpenCV Park AX=XB; independent fixed-board validation" :
                   "OpenCV pinhole five-coefficient calibration; held-out reprojection"};
    const auto domain = data.simulated ? core::ClockDomain::replay :
                                        core::ClockDomain::host_monotonic;
    YAML::Node output, report, previous_intrinsic_report;
    std::string report_name;

    if (!hand_eye) {
      const auto solution = vision::solve_intrinsics(data);
      report = vision::intrinsic_report(data, solution, provenance, domain,
                                        c.require<double>("validation_limits.rms_px"));
      output["calibration"] = YAML::Clone(report["parameters"]);
      report_name = "intrinsic-report.yaml";
      output["calibration"]["intrinsic_report_file"] = report_name;
      output["calibration"]["intrinsics_declared"] = true;
      // 未测外参不填单位旋转或零平移；此文件只能作为后续手眼输入。
    } else {
      const auto intrinsic_config = core::Config::load(intrinsic_path);

      if (!intrinsic_config)
        throw std::invalid_argument(intrinsic_config.error().message);

      const auto intrinsic = read_intrinsics(intrinsic_config.value());
      const auto solution = vision::solve_hand_eye(data, intrinsic);
      report = vision::hand_eye_report(data, intrinsic, solution, provenance, domain,
          c.require<double>("validation_limits.rotation_rad"),
          c.require<double>("validation_limits.translation_m"));
      output["calibration"] = YAML::Clone(report["parameters"]);
      report_name = "extrinsic-report.yaml";
      output["calibration"]["extrinsic_report_file"] = report_name;
      output["calibration"]["intrinsics_declared"] = true;
      output["calibration"]["extrinsics_declared"] = true;

      if (intrinsic_config.value().contains("calibration.intrinsic_report_file")) {
        const auto path = intrinsic_path.parent_path() /
            intrinsic_config.value().require<std::string>("calibration.intrinsic_report_file");
        const auto previous = core::Config::load(path);

        if (!previous)
          throw std::invalid_argument(previous.error().message);

        const vision::Calibration binding(intrinsic.image_size.width, intrinsic.image_size.height,
            intrinsic.matrix, intrinsic.distortion, solution.camera_to_gimbal,
            core::Evidence::missing(), core::Evidence::missing(), intrinsic.roi_offset);
        vision::calibration_report_evidence(previous.value(), binding,
                                            vision::CalibrationCapability::intrinsics);
        previous_intrinsic_report = YAML::LoadFile(path.string());
        output["calibration"]["intrinsic_report_file"] = "intrinsic-report.yaml";
      }
    }

    if (!std::filesystem::create_directory(output_directory))
      throw std::invalid_argument("Calibration output directory already exists");

    write_yaml(output_directory / report_name, report);

    if (previous_intrinsic_report.IsMap())
      write_yaml(output_directory / "intrinsic-report.yaml", previous_intrinsic_report);

    write_yaml(output_directory / (hand_eye ? "calibration.yaml" : "intrinsics.yaml"), output);
    std::cout << "fit_samples=" << report["fit_ids"].size()
              << " validation_samples=" << report["validation_ids"].size()
              << " rejected=" << data.rejected.size()
              << " source=" << (data.simulated ? "synthetic" : "recorded")
              << " output=" << output_directory.string()
              << "\nCandidate parameters and validation report written; no device enabled.\n";

    for (const auto& rejected : data.rejected)
      std::cout << "rejected: " << rejected << '\n';

    return 0;
  } catch (const std::exception& error) {
    std::cerr << "calibration: " << error.what() << '\n';

    return 1;
  }
}
} // namespace autoaim::pipeline
