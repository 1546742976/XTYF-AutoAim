#pragma once

#include "autoaim/mission/mission.hpp"

namespace autoaim::mission {
// 哨兵只形成请求；程序最终 shoot 仍完全由 guard 首检及发送复检决定。
MissionRequest sentry_request(const AimSolution& solution, core::Generation generation,
                              core::TimePoint mode_since, bool program_fire_requested);
} // namespace autoaim::mission
