#include "autoaim/core/result.hpp"
#include "test_support.hpp"
#include <memory>
#include <type_traits>

int main() {
  using namespace autoaim::core;
  static_assert(!std::is_default_constructible_v<Result<int>>);

  return test::run([] {
    auto good = Result<int>::success(7);
    CHECK(good && good.value() == 7);
    auto bad = Result<int>::failure(ErrorCode::io, "short write");
    CHECK(!bad && bad.error().message == "short write");
    CHECK(std::string(error_name(bad.error().code)) == "io");
    CHECK_THROWS(std::bad_variant_access, bad.value());
    auto resource = Result<std::unique_ptr<int>>::success(std::make_unique<int>(9));
    auto moved = std::move(resource).value();
    CHECK(*moved == 9);
  });
}
