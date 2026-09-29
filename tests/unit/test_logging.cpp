#include "autoaim/core/logging.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <atomic>
#include <sstream>
#include <thread>

int main() {
  return test::run([] {
    using namespace autoaim::core;
    std::ostringstream stream;
    Logger logger(stream);
    CHECK(logger.write(LogLevel::debug, "hidden"));
    CHECK(stream.str().empty());
    std::atomic<bool> passed{true};
    auto write = [&] {
      for (int i = 0; i < 50; ++i)
        if (!logger.write(LogLevel::info, "line")) passed = false;
    };
    std::thread first(write), second(write);
    first.join();
    second.join();
    CHECK(passed);
    std::istringstream lines(stream.str());
    std::string line;
    int count = 0;
    while (std::getline(lines, line)) { CHECK(line == "[info] line"); ++count; }
    CHECK(count == 100);
    stream.setstate(std::ios::badbit);
    CHECK(!logger.write(LogLevel::error, "failed stream"));
  });
}
