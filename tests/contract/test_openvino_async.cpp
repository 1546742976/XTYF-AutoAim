#include "autoaim/vision/detector.hpp"
#include "test_support.hpp"
#include <chrono>
#include <set>
#include <thread>

int main(int argc, char** argv) {
  using namespace autoaim;
  return test::run([&] {
    CHECK(argc == 2);
    vision::OpenVinoDetector detector({argv[1], "CPU", vision::TeamColor::red, 0.7, 0.3, 20, 8}, 2);
    const auto make_packet = [](std::uint64_t id) {
      const core::TimePoint time(id, core::ClockDomain::replay);
      auto bytes = std::make_shared<const std::vector<std::uint8_t>>(80 * 60 * 3, 0);
      return std::make_shared<const vision::FramePacket>(core::CapturedFrame(core::Stamp(id, 1, time),
        core::Image(80, 60, 240, bytes), time, core::TimeOrigin::synthetic,
        core::Seconds(0), core::Evidence::missing()), std::nullopt);
    };
    auto first = make_packet(100);
    std::weak_ptr<const vision::FramePacket> retained = first;
    CHECK(detector.try_submit(first));
    first.reset();
    CHECK(!retained.expired());
    CHECK(detector.try_submit(make_packet(101)));
    CHECK(detector.in_flight() == 2 && !detector.try_submit(make_packet(102)));
    CHECK(detector.prepared_bytes() == 2 * 640 * 640 * 3);
    std::set<std::uint64_t> ids;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (detector.in_flight() && std::chrono::steady_clock::now() < deadline) {
      auto completed = detector.take_completed();
      if (completed) {
        CHECK(!completed->failure && completed->detections);
        CHECK(completed->packet->frame.stamp.frame_id == completed->detections->source.frame_id);
        ids.insert(completed->detections->source.frame_id);
      } else std::this_thread::yield();
    }
    CHECK(ids == std::set<std::uint64_t>({100, 101}));
    CHECK(retained.expired() && detector.in_flight() == 0);
    CHECK(detector.try_submit(make_packet(103)));
    detector.close();
    CHECK(detector.in_flight() == 0 && !detector.try_submit(make_packet(104)));
    detector.close();
  });
}
