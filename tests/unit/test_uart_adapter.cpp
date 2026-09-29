#include "autoaim/pipeline/uart_adapter.hpp"
#include "autoaim/control/crc.hpp"
#include "test_support.hpp"
#include <cstring>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto config = core::Config::parse(R"(
uart_feedback:
  pose_source: packet_quaternion
  quaternion_direction: body_to_reference
  packet_reference_to_world_wxyz: [1, 0, 0, 0]
  gimbal_to_packet_body_wxyz: [1, 0, 0, 0]
  yaw_sign: -1
  pitch_sign: 1
  yaw_offset_rad: 0.1
  pitch_offset_rad: 0.2
  receive_delay_s: 0.001
)");
    CHECK(config);
    auto options = pipeline::load_uart_feedback_options(config.value());
    pipeline::UartFeedbackAdapter adapter(options);
    std::vector<std::uint8_t> packet(41, 0);
    packet[0] = 'A';
    packet[1] = 'B';
    const auto set = [&](std::size_t offset, float value) {
      std::uint32_t bits;
      std::memcpy(&bits, &value, 4);
      for (int i = 0; i < 4; ++i)
        packet[offset + i] = std::uint8_t(bits >> (8 * i));
    };
    set(3, 1);
    set(19, 0.5);
    set(27, 0.25);
    set(35, 20);
    control::append_crc16(packet);
    const core::TimePoint at(10000000, core::ClockDomain::replay);
    CHECK(adapter.feed({at, {packet.begin(), packet.begin() + 13}}, 1).empty());
    const auto samples = adapter.feed({at, {packet.begin() + 13, packet.end()}}, 1);
    CHECK(samples.size() == 1 && samples[0].pose_valid && samples[0].status_valid);
    CHECK_NEAR(samples[0].yaw_rad, -0.4, 1e-8);
    CHECK_NEAR(samples[0].pitch_rad, 0.45, 1e-8);
    CHECK(samples[0].sampled_at.nanoseconds() == 9000000 && !samples[0].operator_input);
    auto bad = packet;
    bad[10] ^= 0xff;
    bad.insert(bad.end(), packet.begin(), packet.end());
    bad.insert(bad.end(), packet.begin(), packet.end());
    CHECK(adapter.feed({at, bad}, 1).size() == 2 && adapter.rejected() > 0);
    CHECK(adapter.feed({at, {packet.begin(), packet.begin() + 10}}, 1).empty());
    CHECK(adapter.feed({at, {packet.begin() + 10, packet.end()}}, 2).empty());
    options.yaw_sign = 0;
    CHECK_THROWS(std::invalid_argument, (void)pipeline::UartFeedbackAdapter{options});
    CHECK_THROWS(std::invalid_argument, adapter.feed({at, packet}, 1));
  });
}
