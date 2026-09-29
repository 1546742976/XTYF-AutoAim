#pragma once

#include "autoaim/core/time.hpp"
#include <cstdint>
#include <stdexcept>

namespace autoaim::core {
using FrameId = std::uint64_t;
using Generation = std::uint64_t;
enum class Role { infantry, sentry };
enum class Task { armor, rune };
enum class ControlMode { assist, automatic };
enum class CommandSpace { relative, absolute };
enum class TimeOrigin { hardware_mapped, receipt_estimated, synthetic };

// 允许组合由 mission_factory 唯一判定；这里仅保存其输出，不自行推断权限。
struct Authority {
  Role role;
  Task task;
  ControlMode mode;
  CommandSpace space;
  bool control;
  bool fire;
};

// 只记录来源，不缓存“现在有效”。异步任务必须保留原对象而非重打时间戳。
struct Stamp {
  const FrameId frame_id;
  const Generation generation;
  const TimePoint exposure;

  Stamp(FrameId id, Generation epoch, TimePoint time)
      : frame_id(id), generation(epoch), exposure(time) {
    if (id == 0) throw std::invalid_argument("Frame id zero is reserved");
  }
};

inline Generation next_generation(Generation current) {
  if (current == UINT64_MAX) throw std::overflow_error("Generation exhausted");
  return current + 1;
}
}  // namespace autoaim::core
