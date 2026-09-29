#include "autoaim/hal/recording_transport.hpp"
#include "test_support.hpp"
#include <sstream>

int main() {
  using namespace autoaim;

  return test::run([] {
    hal::ReplayClock clock(core::TimePoint(12, core::ClockDomain::replay));
    std::ostringstream output;
    hal::RecordingTransport recorder(output, clock);
    CHECK(recorder.write_all({0xab, 0, 255}, core::Seconds(0.1)).status ==
          hal::WriteStatus::complete);

    CHECK(output.str() == "12 3 ab 00 ff\n");
    CHECK(recorder.completed_writes() == 1);
    output.setstate(std::ios::badbit);
    CHECK(recorder.write_all({1}, core::Seconds(0.1)).status == hal::WriteStatus::failed);
    output.clear();
    CHECK(recorder.write_all({1}, core::Seconds(0.1)).status == hal::WriteStatus::failed);
    CHECK(!recorder.read_for(core::Seconds(0)));
  });
}
