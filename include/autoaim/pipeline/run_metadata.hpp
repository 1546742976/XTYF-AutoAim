#pragma once

#include "autoaim/pipeline/bootstrap.hpp"

namespace autoaim::pipeline {
struct PipelineMetrics;

// 本次运行的配置副本、实际标定参数/证据、引用报告和模型内容标识。
// run_metadata_schema_version=2 只扩充来源字段；FNV 用于复现定位，不是认证。
// 调用期间配置与引用文件须保持固定；资格针对当前离线 replay 域，不授予设备能力。
YAML::Node describe_run(const std::filesystem::path& configuration, const PipelineConfig& loaded);
YAML::Node timing_report(const PipelineMetrics& metrics);
} // namespace autoaim::pipeline
