#pragma once

#include "autoaim/hal/gimbal_feedback.hpp"
#include "autoaim/vision/frame_packet.hpp"
#include <deque>
#include <mutex>

namespace autoaim::pipeline {
class FrameSync {
public:
  FrameSync(std::size_t capacity, core::Seconds maximum_gap, core::Generation generation);
  bool push(hal::GimbalFeedback feedback);

  // 非消费式、线程安全。无区间、非法四元数、旧世代均返回空，不外推。
  std::optional<vision::AlignedPose> query(core::TimePoint exposure,
                                           core::Generation generation) const;

  void reset(core::Generation generation);

private:
  std::size_t capacity_;
  core::Seconds maximum_gap_;
  core::Generation generation_;
  mutable std::mutex mutex_;
  std::deque<hal::GimbalFeedback> history_;
};
} // namespace autoaim::pipeline
