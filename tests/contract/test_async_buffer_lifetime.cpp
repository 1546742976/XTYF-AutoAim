#include "autoaim/core/buffer_pool.hpp"
#include "test_support.hpp"
#include <future>
#include <thread>

int main() {
  using namespace autoaim::core;

  return test::run([] {
    validate_buffer_budget(1, SIZE_MAX); // 只检查预算，不分配。
    CHECK_THROWS(std::invalid_argument, validate_buffer_budget(0, 3));
    CHECK_THROWS(std::invalid_argument, validate_buffer_budget(2, 0));
    CHECK_THROWS(std::invalid_argument, validate_buffer_budget(2, SIZE_MAX));
    CHECK_THROWS(std::invalid_argument, BufferPool(0, 3));
    CHECK_THROWS(std::invalid_argument, BufferPool(2, 0));
    CHECK_THROWS(std::invalid_argument, BufferPool(2, SIZE_MAX));

    BufferPool raw(2, 3), preprocessed(1, 3);
    const std::uint8_t bytes[]{1, 2, 3};
    auto first = raw.copy(bytes, 3).value();
    auto second = raw.copy(bytes, 3).value();
    auto input = preprocessed.copy(bytes, 3).value();
    auto duplicate = first;
    CHECK(raw.in_use() == 2);
    CHECK(!raw.copy(bytes, 3));
    first.reset();
    CHECK(raw.in_use() == 2);
    duplicate.reset();
    CHECK(raw.in_use() == 1);
    std::promise<void> release;
    auto ready = release.get_future();
    std::promise<int> result;
    auto checked = result.get_future();
    std::thread worker([owned = input, ready = std::move(ready), &result]() mutable {
      ready.wait();
      result.set_value(owned->at(2));
    });
    input.reset(); // 模拟模式切换丢弃逻辑结果；预处理缓冲仍由在途任务持有。
    const bool retained = preprocessed.in_use() == 1 && !preprocessed.copy(bytes, 3);
    release.set_value();
    worker.join();
    CHECK(retained);
    CHECK(checked.get() == 3 && preprocessed.in_use() == 0);
    PixelLease survivor;
    {
      BufferPool temporary(1, 3);
      survivor = temporary.copy(bytes, 3).value();
    }

    CHECK(survivor->at(0) == 1);
    CHECK(!raw.copy(nullptr, 3));
    CHECK(!raw.copy(bytes, 2));
  });
}
