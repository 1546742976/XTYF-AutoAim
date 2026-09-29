#pragma once

#include "autoaim/core/buffer_pool.hpp"
#include <array>
#include <optional>

namespace autoaim::pipeline {
enum class DropReason : std::size_t {
  invalid,
  expired,
  old_generation,
  capacity,
  no_buffer,
  out_of_order,
  closed,
  count
};

using DropCounts = std::array<std::uint64_t, static_cast<std::size_t>(DropReason::count)>;

// 主机实际耗时汇总，常量空间，不参与逻辑时钟/许可判据。由各自所有者串行更新。
struct WallTimeSummary {
  std::uint64_t samples = 0;
  double total_ms = 0;
  double maximum_ms = 0;
  void add(double milliseconds);
};

struct QueueTimings {
  WallTimeSummary pool_copy;
  WallTimeSummary waiting;
};

struct QueueOptions {
  std::size_t pool_capacity;
  std::size_t pending_capacity;
  std::size_t in_flight_limit;
  int width;
  int height;
  std::size_t stride;
  core::Seconds maximum_age;
};

// 无分配、无共享状态；按构造器顺序校验尺寸/队列限制及池预算。
// 校验成功不代表设备有足够可用内存；实际池仍在 FrameQueue 构造时分配。
void validate_queue_options(const QueueOptions& options);

class FrameTask {
public:
  const std::shared_ptr<const core::CapturedFrame>& frame() const noexcept {
    return frame_;
  }

private:
  friend class FrameQueue;

  FrameTask(std::shared_ptr<const core::CapturedFrame> frame, std::shared_ptr<void> activity)
      : frame_(std::move(frame)), activity_(std::move(activity)) {
  }

  std::shared_ptr<const core::CapturedFrame> frame_;
  std::shared_ptr<void> activity_;
};

class FrameQueue {
public:
  FrameQueue(QueueOptions options, core::Generation generation);

  // 源像素借用只持续到本调用返回；预算检查后才复制到池，绝不先超限分配。
  bool submit(core::Stamp stamp, const std::vector<std::uint8_t>& pixels, core::TimePoint received,
              core::TimeOrigin origin, core::Seconds uncertainty, core::Evidence timing,
              core::TimePoint now, core::CaptureMetadata capture = {});

  std::optional<FrameTask> take_latest(core::TimePoint now);
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

// 仅由估计状态的单写者调用（含 reset）。不读取采集/提交的最新帧号。
class ResultAdmission {
public:
  ResultAdmission(core::Generation generation, core::TimePoint since, core::Seconds maximum_age);
  bool accept(const core::Stamp& source, core::TimePoint now, bool contract_valid);
  void reset(core::Generation generation, core::TimePoint since);

  const DropCounts& drops() const noexcept {
    return drops_;
  }

  core::FrameId last_accepted() const noexcept {
    return last_id_;
  }

private:
  core::Generation generation_;
  core::TimePoint since_;
  core::Seconds maximum_age_;
  core::FrameId last_id_ = 0;
  std::optional<core::TimePoint> last_source_;
  DropCounts drops_{};
};
} // namespace autoaim::pipeline
