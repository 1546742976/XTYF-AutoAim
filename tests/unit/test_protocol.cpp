#include "autoaim/control/protocol.hpp"
#include "autoaim/control/crc.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::Stamp stamp(1, 0, core::TimePoint(1, core::ClockDomain::replay));
    const control::Command command(stamp, {core::Radians(1), core::Radians(-1), 0, 0, 0, 0},
      core::Metres(1), core::CommandSpace::absolute, true, true);
    auto ab = control::encode(command, control::ProtocolKind::gimbal_ab);
    CHECK(ab && ab.value().size() == 29 && ab.value()[2] == 2);
    CHECK(ab.value()[5] == 0x80 && ab.value()[6] == 0x3f);
    CHECK(control::check_crc16(ab.value()));
    auto can = control::encode(command, control::ProtocolKind::cboard_can);
    CHECK(can && can.value() == std::vector<std::uint8_t>({1, 1, 0x27, 0x10, 0xd8, 0xf0, 0x27, 0x10}));
    auto uart = control::encode(command, control::ProtocolKind::cboard_uart);
    CHECK(uart && uart.value().size() == 11 && uart.value()[3] == 0x10 && uart.value()[4] == 0x27);
    CHECK(control::check_crc16(uart.value()));
    auto v2 = control::encode(command, control::ProtocolKind::cboard_uart_v2);
    CHECK(v2 && v2.value().size() == 14 && v2.value()[1] == 14);
    CHECK(v2.value()[12] == 0x91 && v2.value()[13] == 0x78);
    CHECK(control::check_crc16(std::vector<std::uint8_t>(v2.value().begin(), v2.value().end() - 2)));
    CHECK(control::encode(control::Command::stop(stamp), control::ProtocolKind::gimbal_ab).value()[2] == 0);
    const control::Command relative(stamp, command.pointing, core::Metres(1), core::CommandSpace::relative, true, false);
    CHECK(!control::encode(relative, control::ProtocolKind::gimbal_ab));

    std::vector<std::uint8_t> feedback(41, 0);
    feedback[0] = 'A'; feedback[1] = 'B'; feedback[2] = 1;
    feedback[5] = 0x80; feedback[6] = 0x3f;  // q.w = 1，IEEE754 小端。
    control::append_crc16(feedback);
    CHECK(control::decode_gimbal(feedback).value().quaternion_wxyz[0] == 1);
    for (std::size_t cut = 0; cut <= feedback.size(); ++cut) {
      control::GimbalStreamParser parser;
      auto first = parser.feed({feedback.begin(), feedback.begin() + cut});
      auto second = parser.feed({feedback.begin() + cut, feedback.end()});
      CHECK(first.size() + second.size() == 1);
    }
    control::GimbalStreamParser parser;
    auto bad = feedback; bad[6] ^= 1;
    bad.insert(bad.end(), feedback.begin(), feedback.end());
    bad.insert(bad.end(), feedback.begin(), feedback.end());
    CHECK(parser.feed(bad).size() == 2 && parser.rejected() >= 1);
    CHECK(parser.feed(std::vector<std::uint8_t>(10000, 0)).empty());
    CHECK(parser.buffered() < 43);
  });
}
