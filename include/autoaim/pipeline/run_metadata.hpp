#pragma once

#include "autoaim/pipeline/bootstrap.hpp"

namespace autoaim::pipeline {
struct PipelineMetrics;

// 本次运行的配置副本、实际标定参数和模型内容标识；FNV 仅用于复现定位，不是认证。
YAML::Node describe_run(const std::filesystem::path& configuration, const PipelineConfig& loaded);
YAML::Node timing_report(const PipelineMetrics& metrics);
} // namespace autoaim::pipeline
