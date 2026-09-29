#include "autoaim/hal/hikrobot_camera.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto config = YAML::Load(R"(
camera:
  model: MV-CS016-10UC
  serial_number: test-only-not-a-device
  exposure_s: 0.001
  gain: 0
  frame_rate_hz: 100
  transport_delay_s: 0.002
  timing_uncertainty_s: 0.003
  maximum_frame_bytes: 4665600
  roi: {width: 1440, height: 1080, offset_x: 0, offset_y: 0}
  calibration_geometry: {width: 1440, height: 1080, offset_x: 0, offset_y: 0}
  pixel_format: BayerRG8
  white_balance: continuous
)");
    const auto parsed = core::Config::parse(YAML::Dump(config));
    CHECK(parsed);
    const auto loaded = hal::load_hikrobot_options(parsed.value());
    CHECK(loaded && loaded.value().serial_number == "test-only-not-a-device");
    auto unset = YAML::Clone(config);
    unset["camera"]["serial_number"] = YAML::Node(YAML::NodeType::Null);
    CHECK(!hal::load_hikrobot_options(core::Config::parse(YAML::Dump(unset)).value()));

    hal::MonotonicClock clock;
    const hal::HikrobotOptions options{
        "not-opened",         core::Seconds(0.001), 0, 100, core::Seconds(0.002),
        core::Seconds(0.003), 1440 * 1080 * 3, {1440, 1080, 0, 0}, {1440, 1080, 0, 0},
        hal::CameraPixelFormat::bayer_rg8, hal::WhiteBalancePolicy::continuous, std::nullopt};

    hal::HikrobotCamera camera(options, clock, 1);
    CHECK(!camera.readback());
    hal::HikrobotReadback actual{options.geometry, options.pixel_format, options.white_balance,
                               options.balance_ratio, core::Seconds(0.00101), 0, 99.8};
    hal::validate_hikrobot_readback(options, actual);
    auto metadata = hal::hikrobot_metadata(5, 8, 12345, actual);
    CHECK(metadata.device_dropped_frames == 2 && metadata.device_frame_discontinuity == true);
    CHECK(metadata.device_timestamp_ticks == 12345 && metadata.pixel_format == "BayerRG8");
    CHECK(hal::hikrobot_metadata(UINT32_MAX, 0, 1, actual).device_dropped_frames == 0);
    CHECK(!hal::hikrobot_metadata(9, 1, 1, actual).device_dropped_frames);
    CHECK(!hal::hikrobot_metadata(std::nullopt, 1, 1, actual).device_frame_discontinuity);
    actual.geometry.offset_x = 2;
    CHECK_THROWS(std::invalid_argument, hal::validate_hikrobot_readback(options, actual));
    auto invalid = options;
    invalid.calibration_geometry.height = 800;
    CHECK_THROWS(std::invalid_argument, hal::validate_hikrobot_options(invalid));
    invalid = options;
    invalid.white_balance = hal::WhiteBalancePolicy::manual;
    CHECK_THROWS(std::invalid_argument, hal::validate_hikrobot_options(invalid));
    invalid.balance_ratio = std::array<std::int64_t, 3>{1000, 1000, 1000};
    hal::validate_hikrobot_options(invalid);
    CHECK(!camera.read_for(core::Seconds(0.01)));
    camera.set_generation(2);
    CHECK_THROWS(std::invalid_argument, camera.set_generation(1));
    hal::ReplayClock replay(core::TimePoint(0, core::ClockDomain::replay));
    CHECK_THROWS(std::invalid_argument, hal::HikrobotCamera(options, replay, 1));
  });
}
