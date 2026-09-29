#include "autoaim/hal/session_writer.hpp"
#include "test_support.hpp"
#include <chrono>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::temp_directory_path() /
                      ("autoaim-session-" + std::to_string(
                          std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(std::filesystem::create_directory(root));
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::filesystem::remove_all(path);
      }
    } cleanup{root};

    const auto fixture = std::filesystem::path(__FILE__).parent_path().parent_path() /
                         "fixtures/session_v2.yaml";
    hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay source(fixture, clock);
    const auto folder = root / "roundtrip";
    hal::SessionWriter writer(folder, source.metadata());
    CHECK_THROWS(std::invalid_argument, hal::SessionWriter(folder, {}));

    for (;;) {
      auto event = source.next(1);
      CHECK(event);
      if (std::holds_alternative<hal::ReplayEnd>(event.value()))
        break;
      writer.append(clock.now(), event.value(), source.annotations(1));
    }

    YAML::Node reason;
    reason["reason"] = "unqualified_evidence";
    writer.derived(clock.now(), "command", reason);
    hal::ReplayClock partial(core::TimePoint(0, core::ClockDomain::replay));
    CHECK_THROWS(std::invalid_argument, hal::FileReplay(folder / "events.yaml", partial));
    writer.finish();
    CHECK_THROWS(std::logic_error, writer.finish());

    hal::ReplayClock clock2(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay copy(folder / "events.yaml", clock2);
    CHECK(copy.metadata().model_id == source.metadata().model_id);
    CHECK(std::get<hal::UartChunk>(copy.next(1).value()).bytes ==
          std::vector<std::uint8_t>({65, 66, 0, 255}));
    CHECK(!std::get<hal::ButtonSample>(copy.next(1).value()).pressed);
    const auto image = std::get<std::shared_ptr<const core::CapturedFrame>>(copy.next(1).value());
    CHECK(image->image.pixels->at(2) == 255);
    CHECK(image->capture.device_frame_id == 27);
    CHECK(image->capture.roi_offset == std::optional<std::array<int, 2>>({2, 4}));
    CHECK(image->capture.device_frame_discontinuity == true);
    CHECK(image->received_at.nanoseconds() == 190);
    CHECK(image->stamp.exposure.nanoseconds() == 150);
    CHECK(copy.annotations(1)[0]["category"].as<std::string>() == "three");
    CHECK(std::holds_alternative<hal::ReplayEnd>(copy.next(1).value()));

    hal::SessionWriter precise(root / "precise", {});
    const double yaw = 0.12345678901234567;
    const double pitch = -0.23456789012345678;
    const hal::GimbalFeedback feedback{clock2.now(), 1, {1, 0, 0, 0}, yaw, pitch,
                                      20.123456789012345, true, true, std::nullopt};
    precise.append(clock2.now(), feedback);
    precise.finish();
    hal::ReplayClock clock4(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay precise_copy(root / "precise/events.yaml", clock4);
    const auto restored = std::get<hal::GimbalFeedback>(precise_copy.next(1).value());
    CHECK(restored.yaw_rad == yaw && restored.pitch_rad == pitch);
    CHECK(restored.bullet_speed_mps == feedback.bullet_speed_mps);

    hal::SessionWriter repeated(root / "repeated", {});
    repeated.append(clock2.now(), image);
    CHECK_THROWS(std::invalid_argument, repeated.append(clock2.now(), image));
    CHECK_THROWS(std::logic_error, repeated.finish());

    // 完成清单不能掩盖图像丢失；失败在读取点锁存。
    std::filesystem::remove(folder / "frame_1.png");
    hal::ReplayClock clock3(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay missing(folder / "events.yaml", clock3);
    CHECK(missing.next(1));
    CHECK(missing.next(1));
    CHECK(!missing.next(1));
    CHECK(!missing.next(1));
  });
}
