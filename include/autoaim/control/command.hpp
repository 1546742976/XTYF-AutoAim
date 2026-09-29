#pragma once

#include "autoaim/control/control_intent.hpp"
#include <optional>

namespace autoaim::control {
// 传输 DTO：运行链只能由 command_guard 产生使能值；协议测试可构造黄金样例。
// 会话尚无观测时停止包 source 为空；已有观测的停止包必须保留来源。
struct Command {
  const std::optional<core::Stamp> source;
  const Pointing pointing;
  const core::Metres horizontal_distance;
  const core::CommandSpace space;
  const bool control_enabled;
  const bool shoot;
  Command(std::optional<core::Stamp> stamp, Pointing angles, core::Metres distance,
          core::CommandSpace command_space, bool control, bool fire);
  static Command stop(std::optional<core::Stamp> source = std::nullopt);
};
bool finite(const Pointing& pointing) noexcept;
}  // namespace autoaim::control
