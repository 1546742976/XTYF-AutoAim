#pragma once

#include "autoaim/pipeline/queue.hpp"

// Test-only reference of the original deque queue, independent of the CMake selection.
namespace autoaim::pipeline {
class BaselineFrameTask {
public:
  const std::shared_ptr<const core::CapturedFrame>& frame() const noexcept {
    return frame_;
  }

private:
  friend class BaselineFrameQueue;

  BaselineFrameTask(std::shared_ptr<const core::CapturedFrame> frame, std::shared_ptr<void> activity)
      : frame_(std::move(frame)), activity_(std::move(activity)) {
  }

  std::shared_ptr<const core::CapturedFrame> frame_;
  std::shared_ptr<void> activity_;
};

class BaselineFrameQueue {
public:
  BaselineFrameQueue(QueueOptions options, core::Generation generation);

  // 源像素借用只持续到本调用返回；预算检查后才复制到池，绝不先超限分配。
  bool submit(core::Stamp stamp, const std::vector<std::uint8_t>& pixels, core::TimePoint received,
              core::TimeOrigin origin, core::Seconds uncertainty, core::Evidence timing,
              core::TimePoint now, core::CaptureMetadata capture = {});

  std::optional<BaselineFrameTask> take_latest(core::TimePoint now);
  void reset(core::Generation generation);
  void close();
  DropCounts drops() const;
  QueueTimings timings() const;
  std::size_t in_use() const;
  std::size_t in_flight() const;

private:
  struct State;
  struct Activity;
  std::shared_ptr<State> state_;
};

} // namespace autoaim::pipeline
