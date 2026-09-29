#include "autoaim/core/buffer_pool.hpp"
#include <algorithm>
#include <mutex>

namespace autoaim::core {
struct BufferPool::State {
  State(std::size_t capacity, std::size_t bytes) : blocks(capacity), busy(capacity, false) {
    for (auto& block : blocks)
      block.resize(bytes);
  }

  std::mutex mutex;
  std::vector<std::vector<std::uint8_t>> blocks;
  std::vector<bool> busy;
};

struct BufferPool::Ticket {
  Ticket(std::shared_ptr<State> owner, std::size_t block) : state(std::move(owner)), index(block) {
  }

  ~Ticket() {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->busy[index] = false;
  }

  std::shared_ptr<State> state;
  std::size_t index;
};

void validate_buffer_budget(std::size_t capacity, std::size_t bytes) {
  if (capacity == 0 || bytes == 0 || bytes > std::numeric_limits<std::size_t>::max() / capacity)
    throw std::invalid_argument("Invalid buffer budget");
}

BufferPool::BufferPool(std::size_t capacity, std::size_t bytes) {
  validate_buffer_budget(capacity, bytes);
  state_ = std::make_shared<State>(capacity, bytes);
}

Result<PixelLease> BufferPool::copy(const std::uint8_t* data, std::size_t size) {
  if (!data || size != state_->blocks.front().size())
    return Result<PixelLease>::failure(ErrorCode::invalid_input, "Buffer size mismatch");

  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto available = std::find(state_->busy.begin(), state_->busy.end(), false);

  if (available == state_->busy.end())
    return Result<PixelLease>::failure(ErrorCode::unavailable, "Buffer pool exhausted");

  const auto index = static_cast<std::size_t>(available - state_->busy.begin());
  auto ticket = std::make_shared<Ticket>(state_, index);
  state_->busy[index] = true;
  std::copy(data, data + size, state_->blocks[index].begin());

  // aliasing shared_ptr：所有引用计为一块，输出没有可变像素访问权。
  return Result<PixelLease>::success(PixelLease(std::move(ticket), &state_->blocks[index]));
}

std::size_t BufferPool::in_use() const {
  std::lock_guard<std::mutex> lock(state_->mutex);

  return static_cast<std::size_t>(std::count(state_->busy.begin(), state_->busy.end(), true));
}

std::size_t BufferPool::capacity() const {
  return state_->blocks.size();
}
} // namespace autoaim::core
