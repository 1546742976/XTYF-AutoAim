#include "autoaim/vision/calibration_solver.hpp"
#include "autoaim/core/fingerprint.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <cstdint>

namespace autoaim::vision {
std::vector<cv::Point3f> CalibrationBoard::object_points() const {
  if ((kind != BoardKind::chessboard && kind != BoardKind::symmetric_circles) ||
      columns < 3 || rows < 3 || columns > 100 || rows > 100 ||
      !std::isfinite(spacing_m) || spacing_m <= 0 || spacing_m > 10)
    throw std::invalid_argument("Invalid calibration board dimensions/spacing_m");

  std::vector<cv::Point3f> points;

  for (int row = 0; row < rows; ++row)
    for (int col = 0; col < columns; ++col)
      points.emplace_back(float(col * spacing_m), float(row * spacing_m), 0);

  return points;
}

CalibrationDataset load_calibration_dataset(const std::filesystem::path& manifest) {
  auto config = core::Config::load(manifest);

  if (!config)
    throw std::invalid_argument(config.error().message);

  const auto& c = config.value();
  const auto name = c.require<std::string>("board.kind");

  if (name != "chessboard" && name != "symmetric_circles")
    throw std::invalid_argument("Board kind must be chessboard or symmetric_circles");

  CalibrationDataset data{{name == "chessboard" ? BoardKind::chessboard :
                                                  BoardKind::symmetric_circles,
                           c.require<int>("board.columns"), c.require<int>("board.rows"),
                           c.require<double>("board.spacing_m")}, {}, {}};
  const auto objects = data.board.object_points();
  if (c.contains("source_kind")) {
    const auto origin = c.require<std::string>("source_kind");

    if (origin != "synthetic" && origin != "recorded")
      throw std::invalid_argument("Calibration source_kind must be synthetic or recorded");
    data.simulated = origin == "synthetic";
  }
  const cv::Size image_size(c.require<int>("image_width"), c.require<int>("image_height"));

  if (c.contains("roi_offset") || !data.simulated) {
    const auto offset = c.require<std::vector<int>>("roi_offset");

    if (offset.size() != 2 || offset[0] < 0 || offset[1] < 0)
      throw std::invalid_argument("Explicit calibration ROI offset required");
    data.roi_offset = {offset[0], offset[1]};
  }

  if (image_size.width <= 0 || image_size.height <= 0)
    throw std::invalid_argument("Positive calibration image dimensions required");

  std::set<std::string> ids, paths;
  std::set<std::uint64_t> image_contents;

  for (const auto& sample : c.require<std::vector<YAML::Node>>("samples")) {
    const auto id = sample["id"].as<std::string>();
    const auto split = sample["split"].as<std::string>();

    if (id.empty() || !ids.insert(id).second || (split != "fit" && split != "validation"))
      throw std::invalid_argument("Duplicate/empty sample id or invalid split");
    if (bool(sample["path"]) == bool(sample["image_points"]))
      throw std::invalid_argument("Provide exactly one of path or image_points");

    CalibrationView view{id, image_size, {}, split == "validation", std::nullopt};

    if (sample["gimbal_to_reference"]) {
      const auto pose = sample["gimbal_to_reference"];
      const auto q = pose["quaternion_wxyz"].as<std::vector<double>>();
      const auto t = pose["translation_m"].as<std::vector<double>>();
      const auto image_ns = sample["image_time_ns"].as<std::int64_t>();
      const auto pose_ns = sample["pose_time_ns"].as<std::int64_t>();
      const double skew = c.require<double>("maximum_pair_skew_s");

      if (q.size() != 4 || t.size() != 3 || !std::isfinite(skew) || skew < 0 ||
          std::abs(static_cast<long double>(image_ns) - pose_ns) * 1e-9L > skew)
        throw std::invalid_argument("Incomplete or unsynchronized gimbal_to_reference pose");

      view.gimbal_to_reference.emplace(Eigen::Quaterniond(q[0], q[1], q[2], q[3]),
                                       Eigen::Vector3d(t[0], t[1], t[2]));
    }

    if (sample["path"]) {
      const auto path = std::filesystem::weakly_canonical(
          manifest.parent_path() / sample["path"].as<std::string>());

      if (!paths.insert(path.string()).second)
        throw std::invalid_argument("Same image reused across calibration samples");

      const auto image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);

      if (image.empty()) {
        data.rejected.push_back(id + ": cannot decode image");
        continue;
      }
      if (image.size() != image_size)
        throw std::invalid_argument("Mixed calibration image dimensions");

      // 对实际解码的灰度像素去重：改文件名或 PNG 编码不构成独立验证样本。
      core::Fingerprint fingerprint;
      for (int row = 0; row < image.rows; ++row)
        fingerprint.append(image.ptr<std::uint8_t>(row), static_cast<std::size_t>(image.cols));
      if (!image_contents.insert(fingerprint.value()).second)
        throw std::invalid_argument("Same decoded image reused across calibration samples");

      const cv::Size pattern(data.board.columns, data.board.rows);
      const bool found = data.board.kind == BoardKind::chessboard ?
          cv::findChessboardCorners(image, pattern, view.points) :
          cv::findCirclesGrid(image, pattern, view.points, cv::CALIB_CB_SYMMETRIC_GRID);

      if (!found) {
        data.rejected.push_back(id + ": board not detected");
        continue;
      }
      if (data.board.kind == BoardKind::chessboard)
        cv::cornerSubPix(image, view.points, {5, 5}, {-1, -1},
                        {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001});
    } else {
      for (const auto& point : sample["image_points"].as<std::vector<std::vector<double>>>()) {
        if (point.size() != 2)
          throw std::invalid_argument("Image point needs two coordinates");
        view.points.emplace_back(float(point[0]), float(point[1]));
      }
    }

    if (view.points.size() != objects.size())
      throw std::invalid_argument("Board/image point count mismatch");

    for (const auto& point : view.points)
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 ||
          point.x >= image_size.width || point.y >= image_size.height)
        throw std::invalid_argument("Calibration point outside image");

    data.views.push_back(std::move(view));
  }

  return data;
}
} // namespace autoaim::vision
