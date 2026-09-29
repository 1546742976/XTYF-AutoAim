#pragma once

#include "autoaim/core/result.hpp"
#include "autoaim/core/types.hpp"

namespace autoaim::mission {
// 唯一的组合矩阵。返回的是模式上限，不是设备能力证据或最终开火许可。
core::Result<core::Authority> validate_authority(core::Role role, core::Task task,
                                                 core::ControlMode mode, core::CommandSpace space,
                                                 bool control, bool fire);

core::Authority infantry_assist_authority();
core::Authority infantry_automatic_authority();
core::Authority sentry_authority();
} // namespace autoaim::mission
