#include "autoaim/pipeline/bootstrap.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {
struct Rendered {
  cv::Mat image;
  std::size_t visible;
};
Rendered render(const autoaim::pipeline::PipelineConfig& config, double time, std::ostream* truth,
    std::uint64_t frame_id) {
  using namespace autoaim;
  const auto& calibration = config.calibration;
  const auto& profile = *config.profiles.front();
  // 合成轨迹定义独立于 EKF 的估计输出；不是训练数据或实机精度声明。
  const math::Point3<math::WorldFrame> center({3, 0.15 * std::sin(time), 0});
  const core::Radians phase(0.25 + 0.3 * time);
  const auto world_to_camera = calibration.camera_to_gimbal().inverse().value();
  cv::Mat image(calibration.height(), calibration.width(), CV_8UC3, cv::Scalar(0, 0, 0));
  std::size_t visible = 0;
  for (std::size_t board = 0; board < profile.plates.size(); ++board) {
    const auto pose = world_to_camera.compose(profile.plate_pose(center, phase, board).value());
    const auto position = pose.translation();
    if (position.z() <= 0 || (pose.rotation() * Eigen::Vector3d::UnitZ()).dot(-position.normalized()) < 0.15) continue;
    cv::Matx33d rotation;
    const auto matrix = pose.rotation().toRotationMatrix();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) rotation(r, c) = matrix(r, c);
    cv::Vec3d rvec; cv::Rodrigues(rotation, rvec);
    const auto corners = vision::object_corners(profile.plates[board].dimensions);
    std::vector<cv::Point2d> projected;
    cv::projectPoints(std::vector<cv::Point3d>(corners.begin(), corners.end()), rvec,
      cv::Vec3d(position.x(), position.y(), position.z()), calibration.intrinsic(), calibration.distortion(), projected);
    if (std::any_of(projected.begin(), projected.end(), [&](const auto& p) {
      return !std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 3 || p.y < 3 || p.x >= image.cols - 3 || p.y >= image.rows - 3;
    })) continue;
    const auto color = std::visit([](const auto& options) { return options.enemy; }, config.detector);
    const cv::Scalar light = color == vision::TeamColor::red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    cv::line(image, projected[0], projected[3], light, 3, cv::LINE_8);
    cv::line(image, projected[1], projected[2], light, 3, cv::LINE_8);
    ++visible;
    if (truth) {
      *truth << std::setprecision(17) << frame_id << '\t' << time << '\t' << board << '\t'
        << center.metres().x() << '\t' << center.metres().y() << '\t' << center.metres().z() << '\t' << phase.value();
      for (const auto& point : projected) *truth << '\t' << point.x << '\t' << point.y;
      *truth << '\n';
    }
  }
  return {image, visible};
}
std::string name(std::size_t index) {
  std::ostringstream output; output << "frame_" << std::setw(6) << std::setfill('0') << index << ".png"; return output.str();
}
}  // namespace

int main(int argc, char** argv) {
  using namespace autoaim;
  try {
    std::filesystem::path config_path, output;
    std::size_t frames = 30;
    bool self_test = false;
    for (int i = 1; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--self-test") { self_test = true; continue; }
      if (option == "--help") { std::cout << "synthetic_sim --config FILE --output NEW_DIR [--frames N]\n"; return 0; }
      if (++i == argc) throw std::invalid_argument("Missing argument");
      if (option == "--config") config_path = argv[i];
      else if (option == "--output") output = argv[i];
      else if (option == "--frames") frames = std::stoul(argv[i]);
      else throw std::invalid_argument("Unknown argument");
    }
    if (frames == 0 || frames > 10000) throw std::invalid_argument("Frames must be 1..10000");
    auto loaded = pipeline::load_pipeline_config(config_path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    const auto& config = loaded.value();
    if (self_test) {
      const auto frame = render(config, 0.1, nullptr, 1);
      std::vector<unsigned char> encoded;
      if (!frame.visible || !cv::imencode(".png", frame.image, encoded)) throw std::runtime_error("Synthetic render failed");
      const auto decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if (cv::norm(frame.image, decoded, cv::NORM_INF) != 0) throw std::runtime_error("Image round trip differs");
      std::cout << "synthetic projection/image round trip passed\n"; return 0;
    }
    if (output.empty() || std::filesystem::exists(output)) throw std::invalid_argument("Output must be a new directory");
    std::filesystem::create_directories(output);
    std::ofstream events(output / "events.yaml"), truth(output / "truth.tsv");
    if (!events || !truth) throw std::runtime_error("Cannot create synthetic output");
    events << "# Synthetic pixels and timing; no real calibration evidence.\ndomain: replay\nevents:\n";
    truth << "frame_id\ttime_s\tphysical_plate\tcenter_x_m\tcenter_y_m\tcenter_z_m\tphase_rad\ttl_x\ttl_y\ttr_x\ttr_y\tbr_x\tbr_y\tbl_x\tbl_y\n";
    for (std::size_t n = 1; n <= frames; ++n) {
      const auto ns = static_cast<std::int64_t>(n) * 20000000;
      const auto frame = render(config, ns * 1e-9, &truth, n);
      if (!cv::imwrite((output / name(n)).string(), frame.image)) throw std::runtime_error("Image write failed");
      events << "  - {at_ns: " << ns << ", kind: feedback, quaternion_wxyz: [1, 0, 0, 0], yaw_rad: 0, pitch_rad: 0, bullet_speed_mps: 20, pose_valid: true, status_valid: true}\n"
        << "  - {at_ns: " << ns + 2000000 << ", kind: image, frame_id: " << n << ", exposure_ns: " << ns
        << ", path: " << name(n) << ", time_origin: synthetic, timing_sigma_s: 0.001}\n";
    }
    events.flush(); truth.flush();
    if (!events || !truth) throw std::runtime_error("Synthetic metadata write failed");
    std::cout << "generated " << frames << " synthetic frames in " << output.string() << '\n'; return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
