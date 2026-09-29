#include "autoaim/hal/file_replay.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>

int main() {
  using namespace autoaim;

  return test::run([] {
    hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay replay(std::filesystem::path(__FILE__).parent_path().parent_path() /
                               "fixtures/replay.yaml",
                           clock);

    auto first = replay.next(1);
    CHECK(!replay.metadata().device_id);
    CHECK(!replay.metadata().configuration_id);
    CHECK(!replay.metadata().model_id);
    CHECK(first && std::holds_alternative<hal::GimbalFeedback>(first.value()));
    CHECK(clock.now().nanoseconds() == 100);
    auto second = replay.next(2);
    CHECK(second);
    const auto frame = std::get<std::shared_ptr<const core::CapturedFrame>>(second.value());
    CHECK(frame->stamp.generation == 2 && frame->stamp.exposure.nanoseconds() == 150);
    CHECK(frame->image.width == 2 && frame->image.pixels->at(2) == 255);
    CHECK(frame->timing_evidence.level() == core::EvidenceLevel::declared);
    CHECK(!frame->capture.device_frame_id);
    CHECK(replay.next(2));
    CHECK(clock.now().nanoseconds() == 200);
    CHECK(std::holds_alternative<hal::ReplayEnd>(replay.next(2).value()));
    CHECK_THROWS(std::invalid_argument,
                 clock.advance_to(core::TimePoint(199, core::ClockDomain::replay)));

    hal::ReplayClock clock2(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay session(std::filesystem::path(__FILE__).parent_path().parent_path() /
                               "fixtures/session_v2.yaml", clock2);
    CHECK(session.metadata().device_id == "fixture-camera");
    CHECK(session.metadata().configuration_id == "fixture-v2");
    CHECK(session.metadata().model_id == "fixture-model");
    CHECK(std::get<hal::UartChunk>(session.next(1).value()).bytes.back() == 255);
    CHECK(!std::get<hal::ButtonSample>(session.next(1).value()).pressed);
    CHECK(clock2.now().nanoseconds() == 100);
    const auto recorded =
        std::get<std::shared_ptr<const core::CapturedFrame>>(session.next(1).value());
    CHECK(recorded->received_at.nanoseconds() == 190);
    CHECK(recorded->capture.device_frame_id == 27);
    CHECK(recorded->capture.device_timestamp_ticks == 3456);
    CHECK(recorded->capture.device_dropped_frames == 2);
    CHECK(recorded->capture.pixel_format == "BayerRG8");
    auto labels = session.annotations(1);
    CHECK(labels[0]["category"].as<std::string>() == "three");
    labels[0]["category"] = "one";
    CHECK(session.annotations(1)[0]["category"].as<std::string>() == "three");
    CHECK(std::holds_alternative<hal::ReplayEnd>(session.next(1).value()));

    struct Temporary {
      std::filesystem::path path;
      ~Temporary() {
        std::filesystem::remove_all(path);
      }
    } temporary{std::filesystem::temp_directory_path() /
                ("autoaim-replay-check-" + std::to_string(
                    std::chrono::steady_clock::now().time_since_epoch().count()))};
    CHECK(std::filesystem::create_directory(temporary.path));

    const auto manifest = temporary.path / "events.yaml";
    const auto write_manifest = [&](const std::string& text) {
      std::ofstream output(manifest);
      output << text;
      output.close();
      CHECK(output.good());
    };
    const auto reject = [&](const std::string& text) {
      write_manifest(text);
      hal::ReplayClock time(core::TimePoint(0, core::ClockDomain::replay));
      CHECK_THROWS(std::invalid_argument, hal::FileReplay(manifest, time));
    };

    reject("domain: replay\ndomain: replay\nevents: []\n");
    reject("domain: replay\nschema_version: 3\ncomplete: true\nevents: []\n");
    reject("domain: replay\nschema_version: 2\nevents: []\n");
    reject("domain: replay\nschema_version: 2\ncomplete: false\nevents: []\n");
    reject("domain: replay\nevents: [{at_ns: 2, kind: button}, {at_ns: 1, kind: button}]\n");
    reject("domain: replay\nevents: [{at_ns: 1, kind: unexpected}]\n");
    reject("domain: replay\nevents: [\n");

    std::string nested = "domain: replay\nevents: []\nextra: ";
    for (int depth = 0; depth < 34; ++depth)
      nested += "{x: ";
    nested += "0" + std::string(34, '}');
    reject(nested);

    write_manifest("domain: replay\nevents: []\n");
    hal::ReplayClock empty_time(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay empty(manifest, empty_time);
    CHECK(!empty.metadata().device_id);
    CHECK(std::holds_alternative<hal::ReplayEnd>(empty.next(1).value()));

    write_manifest("domain: replay\nevents:\n"
                   "  - {at_ns: 1, kind: image, frame_id: 10, annotations: [first]}\n"
                   "  - {at_ns: 1, kind: image, frame_id: 10, annotations: [second]}\n"
                   "  - {at_ns: 2, kind: image, frame_id: 20}\n"
                   "  - {at_ns: 3, kind: image, frame_id: invalid}\n");
    hal::ReplayClock query_time(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay indexed(manifest, query_time);
    const auto missing = indexed.annotations(20);
    CHECK(!missing.IsDefined() || missing.IsNull());
    CHECK(indexed.annotations(10)[0].as<std::string>() == "first");
    auto isolated = indexed.annotations(10);
    isolated[0] = "modified";
    CHECK(indexed.annotations(10)[0].as<std::string>() == "first");
    CHECK_THROWS(YAML::BadConversion, indexed.annotations(99));
    CHECK_THROWS(YAML::BadConversion, indexed.annotations(99));
    CHECK(indexed.annotations(10)[0].as<std::string>() == "first");
    CHECK(query_time.now().nanoseconds() == 0);

    // 有效清单的逆序查询和未知帧；不存在的标注不得修改清单节点。
    write_manifest("domain: replay\nevents:\n"
                   "  - {at_ns: 1, kind: image, frame_id: 1, annotations: []}\n"
                   "  - {at_ns: 2, kind: image, frame_id: 2, annotations: [last]}\n");
    hal::FileReplay ordered(manifest, query_time);
    CHECK(ordered.annotations(2)[0].as<std::string>() == "last");
    CHECK(ordered.annotations(1).IsSequence() && ordered.annotations(1).size() == 0);
    CHECK(ordered.annotations(99).IsNull());
    CHECK(ordered.annotations(2)[0].as<std::string>() == "last");

    const auto fixtures = std::filesystem::path(__FILE__).parent_path().parent_path() / "fixtures";
    auto repeated = YAML::LoadFile((fixtures / "session_v2.yaml").string());
    repeated["events"].push_back(YAML::Clone(repeated["events"][2]));
    std::filesystem::copy_file(fixtures / "tiny.ppm", temporary.path / "tiny.ppm");
    write_manifest(YAML::Dump(repeated));
    hal::FileReplay duplicate(manifest, query_time);
    CHECK(duplicate.annotations(1)[0]["category"].as<std::string>() == "three");
    CHECK(duplicate.next(1));
    CHECK(duplicate.next(1));
    CHECK(duplicate.next(1));
    const auto bad = duplicate.next(1);
    CHECK(!bad && bad.error().message == "Replay frame id must advance");
    CHECK(!duplicate.next(1) && duplicate.next(1).error().code == core::ErrorCode::fault);
  });
}
