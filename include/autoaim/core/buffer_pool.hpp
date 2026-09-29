#pragma once

#include "autoaim/core/image.hpp"
#include "autoaim/core/result.hpp"

namespace autoaim::core {
// 只校验块数及字节预算，不分配存储；无共享状态，可并发调用。
// 非法参数抛出与 BufferPool 构造相同的 invalid_argument。
void validate_buffer_budget(std::size_t capacity, std::size_t block_bytes);

// 固定大小、固定块数；原图/预处理输入使用不同实例分别计数。
// 最后一个租约释放才归还物理块，销毁 pool 不会使在途租约悬空。
class BufferPool {
public:
  BufferPool(std::size_t capacity, std::size_t block_bytes);
  Result<PixelLease> copy(const std::uint8_t* data, std::size_t size);
  std::size_t in_use() const;
  std::size_t capacity() const;

private:
  struct State;
  struct Ticket;
  std::shared_ptr<State> state_;
};
} // namespace autoaim::core
