#pragma once

#include "autoaim/core/image.hpp"
#include "autoaim/core/result.hpp"

namespace autoaim::core {
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
}  // namespace autoaim::core
