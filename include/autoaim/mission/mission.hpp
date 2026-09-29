#pragma once

#include "autoaim/decision/aim_adequacy.hpp"
#include "autoaim/mission/mission_factory.hpp"

namespace autoaim::mission {
// 决策结果的任务层输入；valid 是跟随可用性，不代表可开火。
struct AimSolution {
  const core::Stamp source;
  decision::AimAngles absolute_angles;
  core::Metres distance;
  bool valid;
};

// 请求与最终指令分离，pipeline 再附加证据并提交唯一 command_guard。
struct MissionRequest {
  const core::Stamp source;
  core::Authority authority;
  decision::AimAngles angles;
  core::Metres distance;
  bool control_requested;
  bool fire_requested;
};
} // namespace autoaim::mission
