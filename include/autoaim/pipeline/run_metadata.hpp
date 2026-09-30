#pragma once

#include "autoaim/pipeline/bootstrap.hpp"

namespace autoaim::pipeline {
struct PipelineMetrics;

// 本次启动的入口/基础/快调原文、有效配置、实际标定参数/证据及模型内容标识。
// run_metadata_schema_version=3 增加配置快照与估计器字段；FNV 用于复现定位，不是认证。
// 配置来自装载快照，实际输入反映 CLI 覆盖；引用报告/模型仍须在调用期间保持固定。
// 资格针对当前离线 replay 域，不授予设备能力。
YAML::Node describe_run(const std::filesystem::path& configuration, const PipelineConfig& loaded);
YAML::Node timing_report(const PipelineMetrics& metrics);
} // namespace autoaim::pipeline
