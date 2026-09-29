#include "test_support.hpp"

int main(int argc, char**) {
  return test::run([&] {
    CHECK(argc == 1);
    CHECK_NEAR(0.5 + 0.5, 1.0, 1e-12);
    CHECK_THROWS(std::invalid_argument, throw std::invalid_argument("expected"));
  });
}
