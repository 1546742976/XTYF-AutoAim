#include "autoaim/pipeline/pipeline.hpp"
#include "autoaim/pipeline/uart_writer.hpp"
#include "autoaim/control/crc.hpp"
#include "support/fake_hal.hpp"
#include "test_support.hpp"
#include <chrono>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
        {"synthetic-camera", "synthetic-v1", "2026-09-29", "offline input simulation"},
        {0, 0}, 0, core::ClockDomain::replay));
    std::vector<std::uint8_t> packet(41, 0);
    packet[0] = 'A';
    packet[1] = 'B';
    packet[5] = 0x80;
    packet[6] = 0x3f;
    packet[37] = 0xa0;
    packet[38] = 0x41;
    control::append_crc16(packet);

    for (const auto mode : {mission::ButtonMode::toggle, mission::ButtonMode::hold}) {
      auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
      CHECK(config);
      config.value().independent_button_input = true;
      config.value().infantry.button_mode = mode;
      config.value().button_evidence = evidence;
      config.value().control_channel = evidence;
      config.value().uart_feedback = pipeline::UartFeedbackOptions{
          pipeline::QuaternionDirection::body_to_reference, Eigen::Quaterniond::Identity(),
          Eigen::Quaterniond::Identity(), 1, 1, 0, 0, core::Seconds(0)};
      auto clock =
          std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
      auto* time = clock.get();
      test::RecordingTransport transport;
      pipeline::Pipeline runtime(std::move(config).value(), std::move(clock),
          pipeline::make_uart14_writer(transport, core::Seconds(0.01)),
          control::PublishSchedule::replay_events);
      runtime.start();
      const auto button = [&](bool pressed, bool intervening = false, bool valid = true) {
        time->advance(core::Seconds(0.001));
        runtime.button({time->now(), valid, intervening, pressed});
        CHECK(runtime.replay_tick());
      };
      button(true);
      CHECK(runtime.generation() == 1);
      button(false);
      button(true);
      CHECK(runtime.generation() == 2);
      time->advance(core::Seconds(0.001));
      runtime.uart({time->now(), packet});
      CHECK(runtime.replay_tick() && runtime.generation() == 2);
      button(true);
      CHECK(runtime.generation() == 2);
      button(true, true);
      CHECK(runtime.generation() == 3);
      button(true);
      CHECK(runtime.generation() == 3);
      button(false);
      button(true);
      CHECK(runtime.generation() == 4);
      button(false);
      CHECK(runtime.generation() == (mode == mission::ButtonMode::hold ? 5 : 4));
      if (mode == mission::ButtonMode::toggle)
        button(true);
      CHECK(runtime.generation() == 5);
      button(false);
      button(true);
      CHECK(runtime.generation() == 6);
      time->advance(core::Seconds(0.1));
      CHECK(runtime.replay_tick() && runtime.generation() == 7);
      button(true);
      CHECK(runtime.generation() == 7);
      runtime.close();
      CHECK(runtime.publisher_status().stop_written && !transport.writes.empty());
      for (const auto& bytes : transport.writes)
        CHECK(bytes.size() == 14 && bytes[2] == 0 && bytes[3] == 0);
    }

    const auto directory = std::filesystem::temp_directory_path() /
        ("autoaim-raw-input-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::filesystem::remove_all(path);
      }
    } cleanup{directory};
    hal::SessionWriter session(directory, {"synthetic-camera", "synthetic-v1", std::nullopt});
    const core::TimePoint at(10000000, core::ClockDomain::replay);
    session.append(at, hal::ButtonSample{at, true, false, false});
    session.append(at, hal::UartChunk{at, packet});
    session.finish();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);
    config.value().input_manifest = directory / "events.yaml";
    config.value().independent_button_input = true;
    config.value().uart_feedback = pipeline::UartFeedbackOptions{
        pipeline::QuaternionDirection::body_to_reference, Eigen::Quaterniond::Identity(),
        Eigen::Quaterniond::Identity(), 1, 1, 0, 0, core::Seconds(0)};
    test::RecordingTransport transport;
    pipeline::Pipeline replay(std::move(config).value(),
        std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay)),
        pipeline::make_uart14_writer(transport, core::Seconds(0.01)),
        control::PublishSchedule::replay_events);
    CHECK(replay.run_file() && replay.publisher_status().stop_written);
  });
}
