#pragma once

#include "autoaim/hal/clock.hpp"
#include "autoaim/hal/gimbal_feedback.hpp"
#include "autoaim/core/image.hpp"
#include "autoaim/core/result.hpp"
#include <filesystem>
#include <variant>
#include <yaml-cpp/yaml.h>

namespace autoaim::hal {
struct ReplayEnd {};

struct SessionMetadata {
  std::optional<std::string> device_id;
  std::optional<std::string> configuration_id;
  std::optional<std::string> model_id;
};

// 原始接收片段保留分包边界。HAL 不解释协议或产生控制权限。
struct UartChunk {
  core::TimePoint received_at;
  std::vector<std::uint8_t> bytes;
};

// 按键电平与旧 enable_event 独立；启动时按住不等于观察到了按下沿。
struct ButtonSample {
  core::TimePoint sampled_at;
  bool valid;
  bool intervening;
  bool pressed;
};

using ReplayEvent =
    std::variant<ReplayEnd, GimbalFeedback, std::shared_ptr<const core::CapturedFrame>,
                 UartChunk, ButtonSample>;

// YAML 事件顺序即确定性消费顺序；同一 ns 的事件不重新排序。
// 离线文件输入，不打开设备；clock 的所有权归 pipeline。此对象单线程读取。
class FileReplay {
public:
  FileReplay(const std::filesystem::path& manifest, ReplayClock& clock);
  ~FileReplay();
  core::Result<ReplayEvent> next(core::Generation generation);
  const SessionMetadata& metadata() const noexcept;
  // 返回副本，调用者不能修改读取器持有的标注；没有标注时为空节点。
  YAML::Node annotations(core::FrameId frame_id) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace autoaim::hal
