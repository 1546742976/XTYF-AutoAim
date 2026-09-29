#include "autoaim/control/protocol.hpp"
#include "autoaim/control/crc.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

int main(int argc, char** argv) {
  using namespace autoaim;

  try {
    if (argc == 3 && std::string(argv[1]) == "--decode-ab") {
      std::istringstream input(argv[2]);
      std::vector<std::uint8_t> bytes;
      std::string token;

      while (input >> token) {
        std::size_t end = 0;
        const auto value = std::stoul(token, &end, 16);

        if (end != token.size() || value > 255)
          throw std::invalid_argument("Expected space-separated hex bytes");

        bytes.push_back(static_cast<std::uint8_t>(value));
      }

      auto result = control::decode_gimbal(bytes);

      if (!result)
        throw std::invalid_argument(result.error().message);

      std::cout << "mode=" << static_cast<unsigned>(result.value().mode)
                << " yaw_rad=" << result.value().yaw_rad
                << " pitch_rad=" << result.value().pitch_rad
                << " bullet_speed_mps=" << result.value().bullet_speed_mps << '\n';

      return 0;
    }

    if (argc != 2 ||
        (std::string(argv[1]) != "--self-test" && std::string(argv[1]) != "--encode-stop")) {
      std::cout << "protocol_tester --self-test | --encode-stop | --decode-ab 'HEX BYTES'\nOffline "
                   "bytes only. --encode-stop uses cboard_uart_v2 (14 bytes).\n";

      return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 1;
    }

    const auto print_stop = [](control::ProtocolKind kind) {
      const auto stop = control::CheckedCommand::stop();
      const auto packet = control::encode(stop.command, stop.metadata, kind);

      if (!packet || packet.value().empty())
        throw std::runtime_error("Stop encoding failed");

      std::cout << "protocol=" << static_cast<int>(kind) << " bytes=" << packet.value().size();

      for (const auto byte : packet.value())
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(byte);

      std::cout << std::dec << '\n';
    };

    if (std::string(argv[1]) == "--encode-stop") {
      print_stop(control::default_command_protocol);

      return 0;
    }

    for (const auto kind :
         {control::ProtocolKind::gimbal_ab, control::ProtocolKind::cboard_uart,
          control::ProtocolKind::cboard_uart_v2, control::ProtocolKind::cboard_can})
      print_stop(kind);

    const auto q = control::decode_can_quaternion({0, 0, 0, 0, 0, 0, 0x27, 0x10});
    const auto status = control::decode_can_status({0x07, 0xd0, 1, 2, 0xd8, 0xf0});

    if (!q || q.value()[0] != 1 || !status || status.value().bullet_speed_mps != 20 ||
        status.value().ft_angle_rad != -1 || control::decode_can_quaternion({1}) ||
        control::decode_can_status({0, 0, 255, 0, 0, 0}))
      throw std::runtime_error("CAN golden payload check failed");

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';

    return 1;
  }
}
