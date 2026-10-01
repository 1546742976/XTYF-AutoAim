#pragma once

#include "autoaim/core/types.hpp"
#include <optional>

namespace autoaim::pipeline {
// 所有入口共用一条装配/回放流程；薄角色入口仅检查配置角色，不复制权限逻辑。
// --check-config 仅校验并输出有效配置 YAML，不创建处理链或打开回放数据。
int run_entry(int argc, char** argv, std::optional<core::Role> required_role = std::nullopt);
} // namespace autoaim::pipeline
