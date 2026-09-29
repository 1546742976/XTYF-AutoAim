#pragma once

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace test {
inline void check(bool passed, const char* expression, const char* file, int line) {
  if (!passed) {
    throw std::runtime_error(std::string(file) + ":" + std::to_string(line) + ": " + expression);
  }
}

template <class Exception, class Function> bool throws(Function&& function) {
  try {
    std::forward<Function>(function)();
  } catch (const Exception&) {
    return true;
  }

  return false;
}

template <class Function> int run(Function&& function) {
  try {
    std::forward<Function>(function)();

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
  } catch (...) {
    std::cerr << "Unknown test exception\n";
  }

  return 1;
}
} // namespace test

// 不依赖 assert，Release/NDEBUG 下测试失败仍返回非零退出码。
#define CHECK(...) test::check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, epsilon)                                                                  \
  CHECK(std::isfinite(a) && std::isfinite(b) && std::abs((a) - (b)) <= (epsilon))
#define CHECK_THROWS(type, ...)                                                                    \
  CHECK(test::throws<type>([&] {                                                                   \
    __VA_ARGS__;                                                                                   \
  }))
