#pragma once

#include "autoaim/control/command.hpp"
#include "autoaim/control/control_intent.hpp"
#include <optional>

namespace autoaim::control {
// 主机旁路信息，不属于五字段 Command，也不向既有线协议添加字段。
// source 保留帧号、曝光时间和世代；space 防止把相对修正误当绝对角度发送。
// 前馈单位分别为 rad/s、rad/s^2，仅供已有 gimbal_ab 编码使用。
struct CommandMetadata {
  std::optional<core::Stamp> source;
  core::CommandSpace space;
  double yaw_rate_radps;
  double pitch_rate_radps;
  double yaw_acceleration_radps2;
  double pitch_acceleration_radps2;
  std::uint32_t inhibit_reasons = 0; // 主机诊断，不在线协议中发送。
  bool stop_record = false;
};

// 同一次复检产生的不可变载荷与元数据；publisher 持有至写回调返回。
// 运行链只由 command_guard 产生许可；协议测试可显式构造黄金样例。
struct CheckedCommand {
  const Command command;
  const CommandMetadata metadata;

  CheckedCommand(std::optional<core::Stamp> source, Pointing pointing, core::Metres distance,
                 core::CommandSpace space, bool control, bool shoot);

  // 会话尚无观测时 source 为空；已有观测的停止包保留来源，但所有输出量清零。
  static CheckedCommand stop(std::optional<core::Stamp> source = std::nullopt);
};

bool finite(const Pointing& pointing) noexcept;
} // namespace autoaim::control
