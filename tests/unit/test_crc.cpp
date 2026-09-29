#include "autoaim/control/crc.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::control;
  return test::run([] {
    std::vector<std::uint8_t> packet{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(crc16(packet.data(), packet.size()) == 0x6f91);
    append_crc16(packet);
    CHECK(packet[9] == 0x91 && packet[10] == 0x6f);
    CHECK(check_crc16(packet));
    for (std::size_t byte = 0; byte < packet.size(); ++byte) {
      for (int bit = 0; bit < 8; ++bit) {
        auto corrupted = packet;
        corrupted[byte] ^= static_cast<std::uint8_t>(1u << bit);
        CHECK(!check_crc16(corrupted));
      }
    }
    CHECK(!check_crc16({})); CHECK(!check_crc16({1}));
    CHECK(crc16(nullptr, 0) == 0xffff);
    CHECK_THROWS(std::invalid_argument, crc16(nullptr, 1));
  });
}
