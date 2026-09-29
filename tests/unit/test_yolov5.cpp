#include "autoaim/vision/detector_factory.hpp"
#include "test_support.hpp"
#include <limits>

int main(int argc, char** argv) {
  using namespace autoaim;
  return test::run([&] {
    vision::Yolov5Options options{"", "CPU", vision::TeamColor::red, 0.7, 0.3, 20, 8};
    cv::Mat output(3, 22, CV_32F, cv::Scalar(-10));
    for (int r = 0; r < output.rows; ++r) {
      const float row[] = {10, 10, 10, 30, 50, 30, 50, 10, 4, -2, 2, -2, -2,
                           -2, -2, -2, 2, -2, -2, -2, -2, -2};
      std::copy(std::begin(row), std::end(row), output.ptr<float>(r));
    }
    output.at<float>(2, 0) = std::numeric_limits<float>::quiet_NaN();
    const auto detections = vision::parse_yolov5(output, 0.5, {200, 100}, options);
    CHECK(detections.size() == 1 && detections[0].class_id == 3);
    CHECK(detections[0].corners[1] == cv::Point2f(100, 20));
    CHECK(!detections[0].corners_reliable);
    options.enemy = vision::TeamColor::blue;
    CHECK(vision::parse_yolov5(output, 1, {200, 100}, options).empty());
    CHECK_THROWS(std::invalid_argument, vision::parse_yolov5(output.colRange(0, 21), 1, {200, 100}, options));
    CHECK_THROWS(std::invalid_argument, vision::make_detector(options));
    if (argc == 2) {
      options.model_path = argv[1];
      auto detector = vision::make_detector(options);
      auto bytes = std::make_shared<const std::vector<std::uint8_t>>(80 * 60 * 3, 0);
      const core::TimePoint time(1, core::ClockDomain::replay);
      const vision::FramePacket packet(core::CapturedFrame(core::Stamp(9, 3, time),
        core::Image(80, 60, 240, bytes), time, core::TimeOrigin::synthetic,
        core::Seconds(0), core::Evidence::missing()), std::nullopt);
      const auto result = detector->detect(packet);
      CHECK(result.source.frame_id == 9 && result.source.generation == 3);
      for (const auto& detection : result.detections) CHECK(!detection.corners_reliable);
    }
  });
}
