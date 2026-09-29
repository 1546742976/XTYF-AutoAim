#include "autoaim/vision/detector_factory.hpp"
#include "test_support.hpp"

int main(int argc, char** argv) {
  using namespace autoaim;

  return test::run([&] {
    vision::YoloOptions options{"", "CPU", vision::TeamColor::red, 0.7, 0.3, 20, 8};
    options.format = vision::YoloFormat::armor_v11;
    cv::Mat output(50, 3, CV_32F, cv::Scalar(0));
    const float points[] = {10, 30, 10, 10, 50, 10, 50, 30};

    for (int col = 0; col < 3; ++col) {
      output.at<float>(0, col) = 30;
      output.at<float>(1, col) = 20;
      output.at<float>(2, col) = 40;
      output.at<float>(3, col) = 20;
      output.at<float>(14, col) = 0.9f - col * 0.01f; // 类 10：红色三号小板。
      for (int i = 0; i < 8; ++i)
        output.at<float>(42 + i, col) = points[i];
    }

    output.at<float>(2, 2) = NAN;
    const auto result = vision::parse_yolo11(output, 0.5, {200, 100}, options);
    CHECK(result.size() == 1);
    CHECK(result[0].raw_class_id == 10 && result[0].category == vision::TargetCategory::three);
    CHECK(result[0].armor_size == vision::ArmorSize::small && !result[0].corners_reliable);
    CHECK(result[0].corners[0] == cv::Point2f(20, 60)); // 不按图像 y 偷换关键点顺序。
    options.enemy = vision::TeamColor::blue;
    CHECK(vision::parse_yolo11(output, 1, {200, 100}, options).empty());
    options.enemy = vision::TeamColor::red;
    CHECK_THROWS(std::invalid_argument,
                 vision::parse_yolo11(output.rowRange(0, 49), 1, {200, 100}, options));
    CHECK_THROWS(std::invalid_argument,
                 vision::parse_yolo11(output, 0, {200, 100}, options));

    if (argc == 2) {
      options.model_path = argv[1];
      auto detector = vision::make_detector(options);
      const core::TimePoint time(1, core::ClockDomain::replay);
      const auto pixels = std::make_shared<const std::vector<std::uint8_t>>(80 * 60 * 3, 0);
      vision::FramePacket frame(core::CapturedFrame(core::Stamp(9, 1, time),
          core::Image(80, 60, 240, pixels), time, core::TimeOrigin::synthetic, core::Seconds(0),
          core::Evidence::missing()), std::nullopt);
      const auto detected = detector->detect(frame);
      CHECK(detected.source.frame_id == 9);
      for (const auto& detection : detected.detections)
        CHECK(!detection.corners_reliable);
    }
  });
}
