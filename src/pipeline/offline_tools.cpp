#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/pipeline/bootstrap.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstring>
#include <iostream>

namespace autoaim::pipeline {
int run_calibration_tool(int argc, char** argv) {
  try {
    std::filesystem::path config_path, corners_path;
    bool self_test = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);
      if (argument == "--help") {
        std::cout << "calibration_tool --config FILE (--corners YAML | --self-test)\nChecks explicit calibration and IPPE candidates; does not calibrate hardware.\n";
        return 0;
      }
      if (argument == "--self-test") { self_test = true; continue; }
      if (++i == argc) throw std::invalid_argument("Missing argument");
      if (argument == "--config") config_path = argv[i];
      else if (argument == "--corners") corners_path = argv[i];
      else throw std::invalid_argument("Unknown argument");
    }
    auto loaded = load_pipeline_config(config_path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    const auto& config = loaded.value();
    int class_id = config.plate_sizes.begin()->first;
    vision::Detection detection{{}, 1, vision::TeamColor::red, class_id, false};
    if (self_test) {
      const auto corners = vision::object_corners(config.plate_sizes.begin()->second);
      const auto q = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()));
      const auto matrix = q.toRotationMatrix();
      cv::Matx33d rotation;
      for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) rotation(r, c) = matrix(r, c);
      cv::Vec3d rvec; cv::Rodrigues(rotation, rvec);
      std::vector<cv::Point2d> projected;
      cv::projectPoints(std::vector<cv::Point3d>(corners.begin(), corners.end()), rvec, cv::Vec3d(0.1, 0.05, 3),
        config.calibration.intrinsic(), config.calibration.distortion(), projected);
      for (std::size_t k = 0; k < 4; ++k) detection.corners[k] = projected[k];
    } else {
      auto data = core::Config::load(corners_path);
      if (!data) throw std::invalid_argument(data.error().message);
      class_id = data.value().require<int>("class_id"); detection.class_id = class_id;
      const auto corners = data.value().require<std::vector<std::vector<double>>>("corners_tl_tr_br_bl");
      if (corners.size() != 4) throw std::invalid_argument("Four ordered corners required");
      for (std::size_t k = 0; k < 4; ++k) {
        if (corners[k].size() != 2) throw std::invalid_argument("Corner requires x,y pixels");
        detection.corners[k] = cv::Point2f(static_cast<float>(corners[k][0]), static_cast<float>(corners[k][1]));
      }
    }
    const auto dimensions = config.plate_sizes.find(class_id);
    if (dimensions == config.plate_sizes.end()) throw std::invalid_argument("No configured dimensions for class_id");
    const auto result = vision::solve_pose(core::Stamp(1, 1, core::TimePoint(1, core::ClockDomain::replay)),
      detection, dimensions->second, config.calibration, config.pnp, config.guard.device_id, config.guard.configuration_id,
      std::nullopt, std::nullopt);
    std::cout << "valid=" << result.pose_valid << " reliable=" << result.pose_reliable << " ambiguous=" << result.ambiguous
      << " candidates=" << result.candidates.size() << '\n';
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const auto& p = result.candidates[i];
      std::cout << i << " xyz_m=" << p.plate_to_camera.value().translation().transpose() << " rms_px=" << p.rms_px
        << " max_px=" << p.maximum_error_px << " view_cos=" << p.view_cosine << " covariance=" << p.covariance.has_value() << '\n';
    }
    if (!result.pose_valid) return 2;
    if (self_test && (result.pose_reliable ||
        (result.candidates[*result.selected].plate_to_camera.value().translation() - Eigen::Vector3d(0.1, 0.05, 3)).norm() > 1e-4))
      throw std::runtime_error("Synthetic PnP check failed");
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
int run_detector_benchmark(int argc, char** argv) {
  try {
    std::filesystem::path config_path, image_path;
    std::size_t iterations = 100;
    bool self_test = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);
      if (argument == "--help") { std::cout << "bench_detector --config FILE --image FILE [--iterations N]\nLocal detector wall-time only; not NUC or full-pipeline FPS.\n"; return 0; }
      if (argument == "--self-test") { self_test = true; iterations = 3; continue; }
      if (++i == argc) throw std::invalid_argument("Missing argument");
      if (argument == "--config") config_path = argv[i];
      else if (argument == "--image") image_path = argv[i];
      else if (argument == "--iterations") iterations = std::stoul(argv[i]);
      else throw std::invalid_argument("Unknown argument");
    }
    if (!iterations || iterations > 10000) throw std::invalid_argument("Iterations must be 1..10000");
    auto loaded = load_pipeline_config(config_path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    const auto& config = loaded.value();
    cv::Mat image = self_test ? cv::Mat(config.calibration.height(), config.calibration.width(), CV_8UC3, cv::Scalar(0, 0, 0)) :
      cv::imread(image_path.string(), cv::IMREAD_COLOR);
    if (image.empty() || image.cols != config.calibration.width() || image.rows != config.calibration.height())
      throw std::invalid_argument("Image missing or differs from configured dimensions");
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(image.total() * 3);
    for (int row = 0; row < image.rows; ++row) std::memcpy(pixels->data() + row * image.cols * 3, image.ptr(row), image.cols * 3);
    const core::TimePoint time(1, core::ClockDomain::replay);
    const vision::FramePacket packet(core::CapturedFrame(core::Stamp(1, 1, time),
      core::Image(image.cols, image.rows, image.cols * 3, pixels), time, core::TimeOrigin::synthetic,
      core::Seconds(0), core::Evidence::missing()), std::nullopt);
    const auto start = std::chrono::steady_clock::now();
    auto detector = vision::make_detector(config.detector);
    const auto load_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    detector->detect(packet);  // 一次预热不混入稳态样本；模型加载单独报告。
    std::vector<double> milliseconds;
    std::size_t detections = 0;
    for (std::size_t i = 0; i < iterations; ++i) {
      const auto begin = std::chrono::steady_clock::now();
      const auto result = detector->detect(packet);
      milliseconds.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count());
      detections += result.detections.size();
    }
    std::sort(milliseconds.begin(), milliseconds.end());
    const auto percentile = [&](double q) { return milliseconds.at(static_cast<std::size_t>(std::ceil(q * milliseconds.size())) - 1); };
    std::cout << "local_detector_only iterations=" << iterations << " model_load_ms=" << load_ms
      << " p50_ms=" << percentile(0.5) << " p95_ms=" << percentile(0.95) << " max_ms=" << milliseconds.back()
      << " detection_count=" << detections << '\n';
    if (self_test && detections) throw std::runtime_error("Blank fixture unexpectedly detected armor");
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
}  // namespace autoaim::pipeline
