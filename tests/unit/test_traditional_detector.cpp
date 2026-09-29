#include "autoaim/vision/detector.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(100 * 80 * 3, 0);

    for (int y = 25; y < 55; ++y) {
      for (int x : {20, 21, 22, 23, 70, 71, 72, 73})
        (*pixels)[(y * 100 + x) * 3 + 2] = 255;
    }

    const core::TimePoint time(1, core::ClockDomain::replay);
    const vision::FramePacket frame(
        core::CapturedFrame(core::Stamp(7, 2, time), core::Image(100, 80, 300, pixels), time,
                            core::TimeOrigin::synthetic, core::Seconds(0),
                            core::Evidence::missing()),
        std::nullopt);

    vision::TraditionalDetector red({vision::TeamColor::red, 100, 50, 10, 2, 0.8, 5, 16});
    vision::TraditionalDetector blue({vision::TeamColor::blue, 100, 50, 10, 2, 0.8, 5, 16});
    const auto result = red.detect(frame);
    CHECK(result.detections.size() == 1);
    CHECK(result.source.frame_id == 7 && result.source.generation == 2);
    CHECK(!result.detections[0].corners_reliable && result.detections[0].raw_class_id == -1);
    CHECK(result.detections[0].corners[0].x < result.detections[0].corners[1].x);
    CHECK(blue.detect(frame).detections.empty());
    CHECK((*pixels)[(25 * 100 + 20) * 3 + 2] == 255);
  });
}
