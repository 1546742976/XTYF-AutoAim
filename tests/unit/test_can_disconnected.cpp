#include "autoaim/hal/can_transport.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    hal::CanTransport can({"not-opened", 0x100, {0x101, 0x102}});
    CHECK(!can.is_open());
    CHECK(can.write_all({1}, core::Seconds(0.01)).status == hal::WriteStatus::failed);
    CHECK(!can.read_for(core::Seconds(0.01)));
    CHECK_THROWS(std::invalid_argument, hal::CanTransport({"can0", 0x800, {1}}));
    CHECK_THROWS(std::invalid_argument, hal::CanTransport({"can0", 1, {}}));
  });
}
