#include "autoaim/hal/serial_transport.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    hal::SerialTransport transport("/not-opened-offline-test", 115200);
    CHECK(!transport.is_open());
    CHECK(transport.write_all({1}, core::Seconds(0.01)).status == hal::WriteStatus::failed);
    CHECK(!transport.read_for(core::Seconds(0.01)));
    CHECK_THROWS(std::invalid_argument, hal::SerialTransport("", 115200));
    CHECK_THROWS(std::invalid_argument, hal::SerialTransport("not-opened", 0));
    transport.close_device();
  });
}
