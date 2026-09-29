#include "autoaim/control/protocol.hpp"
#include "autoaim/control/crc.hpp"
#include "test_support.hpp"
#include <limits>

int main() {
  using namespace autoaim;

  return test::run([] {
    const core::Stamp stamp(1, 0, core::TimePoint(1, core::ClockDomain::replay));
    const control::CheckedCommand checked(stamp, {core::Radians(1), core::Radians(-1), 0, 0, 0, 0},
                                          core::Metres(1), core::CommandSpace::absolute, true,
                                          true);
    const auto& command = checked.command;
    const auto& metadata = checked.metadata;

    auto ab = control::encode(command, metadata, control::ProtocolKind::gimbal_ab);
    CHECK(ab && ab.value().size() == 29 && ab.value()[2] == 2);
    CHECK(ab.value()[5] == 0x80 && ab.value()[6] == 0x3f);
    CHECK(control::check_crc16(ab.value()));
    auto can = control::encode(command, metadata, control::ProtocolKind::cboard_can);
    CHECK(can &&
          can.value() == std::vector<std::uint8_t>({1, 1, 0x27, 0x10, 0xd8, 0xf0, 0x27, 0x10}));

    auto uart = control::encode(command, metadata, control::ProtocolKind::cboard_uart);
    CHECK(uart && uart.value().size() == 11 && uart.value()[3] == 0x10 && uart.value()[4] == 0x27);
    CHECK(control::check_crc16(uart.value()));
    auto v2 = control::encode(command, metadata, control::ProtocolKind::cboard_uart_v2);
    CHECK(v2 && v2.value().size() == 14 && v2.value()[1] == 14);
    CHECK(v2.value()[12] == 0x91 && v2.value()[13] == 0x78);
    CHECK(
        control::check_crc16(std::vector<std::uint8_t>(v2.value().begin(), v2.value().end() - 2)));

    // 固定样例同时覆盖小端有符号角度、字段位置、前 10 字节 CRC 和不参与 CRC 的帧尾。
    const std::vector<std::uint8_t> expected_v2{0xa5, 0x0e, 0x01, 0x01, 0x10, 0x27, 0xf0,
                                                0xd8, 0x10, 0x27, 0xf7, 0x27, 0x91, 0x78};
    CHECK(v2.value() == expected_v2);
    CHECK(control::default_command_protocol == control::ProtocolKind::cboard_uart_v2);
    CHECK(control::encode(command, metadata).value() == expected_v2);

    const auto stop = control::CheckedCommand::stop(stamp);
    const std::vector<std::uint8_t> expected_stop{0xa5, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                  0x00, 0x00, 0x00, 0xaa, 0xd5, 0x91, 0x78};
    CHECK(control::encode(stop.command, stop.metadata).value() == expected_stop);
    CHECK(
        control::encode(stop.command, stop.metadata, control::ProtocolKind::gimbal_ab).value()[2] ==
        0);

    // 只在线协议边界沿用距离饱和，主机侧真实距离不能因此被修改。
    auto distant = command;
    distant.shoot = false;
    distant.horizon_distance = 5;
    const std::vector<std::uint8_t> expected_saturated{
        0xa5, 0x0e, 0x01, 0x00, 0x10, 0x27, 0xf0, 0xd8, 0xff, 0x7f, 0xbe, 0x0c, 0x91, 0x78};
    CHECK(control::encode(distant, metadata).value() == expected_saturated);
    CHECK(distant.horizon_distance == 5);

    const control::CheckedCommand relative(stamp, {core::Radians(1), core::Radians(-1), 0, 0, 0, 0},
                                           core::Metres(1), core::CommandSpace::relative, true,
                                           false);

    CHECK(!control::encode(relative.command, relative.metadata, control::ProtocolKind::gimbal_ab));

    // 六个非零角度/前馈量仍按旧 AB 线序发送，没有因五字段接口而丢失前馈。
    const control::CheckedCommand feedforward(
        stamp, {core::Radians(1), core::Radians(-1), 2, -2, 3, -3}, core::Metres(1),
        core::CommandSpace::absolute, true, true);
    std::vector<std::uint8_t> expected_ab{'A',  'B',  2,    0x00, 0x00, 0x80, 0x3f, 0x00, 0x00,
                                          0x00, 0x40, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80,
                                          0xbf, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x40, 0xc0};
    control::append_crc16(expected_ab);
    CHECK(
        control::encode(feedforward.command, feedforward.metadata, control::ProtocolKind::gimbal_ab)
            .value() == expected_ab);

    for (const auto kind :
         {control::ProtocolKind::gimbal_ab, control::ProtocolKind::cboard_uart,
          control::ProtocolKind::cboard_uart_v2, control::ProtocolKind::cboard_can}) {
      CHECK(!control::encode(relative.command, relative.metadata, kind));
      auto invalid = command;
      invalid.control = false;
      CHECK(!control::encode(invalid, metadata, kind));
      invalid = command;
      invalid.horizon_distance = -1;
      CHECK(!control::encode(invalid, metadata, kind));

      for (auto member : {&control::Command::yaw, &control::Command::pitch,
                          &control::Command::horizon_distance}) {
        invalid = command;
        invalid.*member = std::numeric_limits<double>::quiet_NaN();
        CHECK(!control::encode(invalid, metadata, kind));
      }

      auto no_source = metadata;
      no_source.source.reset();
      CHECK(!control::encode(command, no_source, kind));
      auto invalid_feedforward = metadata;
      invalid_feedforward.pitch_rate_radps = std::numeric_limits<double>::infinity();
      CHECK(!control::encode(command, invalid_feedforward, kind));
      CHECK(control::encode(stop.command, stop.metadata, kind));
    }

    CHECK(control::encode(stop.command, stop.metadata, control::ProtocolKind::cboard_can).value() ==
          std::vector<std::uint8_t>(8, 0));

    std::vector<std::uint8_t> feedback(41, 0);
    feedback[0] = 'A';
    feedback[1] = 'B';
    feedback[2] = 1;
    feedback[5] = 0x80;
    feedback[6] = 0x3f; // q.w = 1，IEEE754 小端。
    control::append_crc16(feedback);
    CHECK(control::decode_gimbal(feedback).value().quaternion_wxyz[0] == 1);

    for (std::size_t cut = 0; cut <= feedback.size(); ++cut) {
      control::GimbalStreamParser parser;
      auto first = parser.feed({feedback.begin(), feedback.begin() + cut});
      auto second = parser.feed({feedback.begin() + cut, feedback.end()});
      CHECK(first.size() + second.size() == 1);
    }

    control::GimbalStreamParser parser;
    auto bad = feedback;
    bad[6] ^= 1;
    bad.insert(bad.end(), feedback.begin(), feedback.end());
    bad.insert(bad.end(), feedback.begin(), feedback.end());
    CHECK(parser.feed(bad).size() == 2 && parser.rejected() >= 1);
    CHECK(parser.feed(std::vector<std::uint8_t>(10000, 0)).empty());
    CHECK(parser.buffered() < 43);
  });
}
