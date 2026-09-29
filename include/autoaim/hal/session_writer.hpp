#pragma once

#include "autoaim/hal/file_replay.hpp"
#include <fstream>
#include <mutex>

namespace autoaim::hal {
// 同步离线写者；只接受 replay 域，不连接设备。内部串行化写操作；禁止并发析构。
// 原始输入写 events.yaml/PNG，派生结果写 derived.yaml；不持有调用者像素。
// 目录必须不存在。析构不补 complete，异常/中断产物保持不可回放。
class SessionWriter {
public:
  SessionWriter(const std::filesystem::path& directory, const SessionMetadata& metadata);
  void append(core::TimePoint at, const ReplayEvent& event, const YAML::Node& annotations = {});
  void derived(core::TimePoint at, const std::string& kind, const YAML::Node& values);
  void finish();

private:
  void check_time(core::TimePoint at) const;
  std::filesystem::path directory_;
  std::ofstream events_;
  std::ofstream derived_;
  std::int64_t previous_ns_ = 0;
  core::FrameId previous_frame_ = 0;
  bool have_events_ = false;
  bool finished_ = false;
  bool failed_ = false;
  std::mutex mutex_;
};
} // namespace autoaim::hal
