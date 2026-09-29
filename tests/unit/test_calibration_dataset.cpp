#include "autoaim/vision/calibration_solver.hpp"
#include "test_support.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::temp_directory_path() /
        ("autoaim-calibration-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(std::filesystem::create_directory(root));
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::filesystem::remove_all(path);
      }
    } cleanup{root};

    cv::Mat image(320, 400, CV_8UC1, cv::Scalar(255));

    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 8; ++col)
        if ((row + col) % 2 == 0)
          cv::rectangle(image, {40 + col * 40, 40 + row * 40, 40, 40}, cv::Scalar(0), -1);

    CHECK(cv::imwrite((root / "board.png").string(), image));
    const auto write = [&](const std::string& text) {
      std::ofstream output(root / "data.yaml");
      output << text;
      CHECK(output.good());
    };
    const std::string prefix =
        "board: {kind: chessboard, columns: 7, rows: 5, spacing_m: 0.02}\n"
        "image_width: 400\nimage_height: 320\nsamples:\n";
    write(prefix + "  - {id: fit1, split: fit, path: board.png}\n"
                   "  - {id: missing, split: validation, path: missing.png}\n");
    auto data = vision::load_calibration_dataset(root / "data.yaml");
    CHECK(data.views.size() == 1 && data.rejected.size() == 1);
    CHECK(data.views[0].points.size() == 35 && !data.views[0].gimbal_to_reference);
    CHECK_NEAR(data.board.object_points().back().x, 0.12, 1e-7);
    CHECK_NEAR(data.board.object_points().back().y, 0.08, 1e-7);
    write(prefix + "  - {id: fit1, split: fit, path: board.png}\n"
                   "  - {id: validation1, split: validation, path: board.png}\n");
    CHECK_THROWS(std::invalid_argument, vision::load_calibration_dataset(root / "data.yaml"));
    write(prefix + "  - {id: fit1, split: unknown, path: board.png}\n");
    CHECK_THROWS(std::invalid_argument, vision::load_calibration_dataset(root / "data.yaml"));
    CHECK_THROWS(std::invalid_argument,
                 vision::CalibrationBoard({vision::BoardKind::chessboard, 7, 5, 0})
                     .object_points());

    image.setTo(cv::Scalar(255));
    for (int row = 0; row < 5; ++row)
      for (int col = 0; col < 7; ++col)
        cv::circle(image, {80 + col * 40, 80 + row * 40}, 9, cv::Scalar(0), -1);
    CHECK(cv::imwrite((root / "circles.png").string(), image));
    write("board: {kind: symmetric_circles, columns: 7, rows: 5, spacing_m: 0.02}\n"
          "image_width: 400\nimage_height: 320\nsamples:\n"
          "  - {id: circles, split: fit, path: circles.png}\n");
    const auto circles = vision::load_calibration_dataset(root / "data.yaml");
    CHECK(circles.views.size() == 1 && circles.views[0].points.size() == 35);
  });
}
