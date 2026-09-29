#pragma once

#include "autoaim/core/types.hpp"
#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace autoaim::pipeline::annotation_detail {
// 离线工具私有契约：所有节点由本次调用持有，不向运行时传递审核声明。
struct ImageEntry {
  YAML::Node event;
  std::size_t line;
  std::filesystem::path relative_path;
  cv::Size size;
};

struct SessionSource {
  std::filesystem::path manifest;
  std::vector<std::string> lines; // 含原始换行；非 image 行按字节复制。
  std::vector<YAML::Node> events;
  std::map<core::FrameId, ImageEntry> images;
  YAML::Node fingerprint;
};

struct AnnotationCheck {
  YAML::Node document;
  YAML::Node problems{YAML::NodeType::Sequence};
  std::map<core::FrameId, YAML::Node> frames; // 仅存整帧校验通过的节点。
  bool source_matches = false;
};

AnnotationCheck check_annotations(const SessionSource& source,
                                  const std::filesystem::path& annotations,
                                  const YAML::Node& expected_source);
// 有 sidecar 则复核数据/标注绑定；旧会话无 sidecar 时返回未知审核状态。
YAML::Node annotation_provenance(const std::filesystem::path& manifest);

std::string read_bytes(const std::filesystem::path& path);
void write_bytes(const std::filesystem::path& path, const std::string& bytes);
void write_yaml(const std::filesystem::path& path, const YAML::Node& node);
YAML::Node read_document(const std::filesystem::path& path);
bool same_yaml(const YAML::Node& left, const YAML::Node& right);
YAML::Node file_fingerprint(const std::filesystem::path& path, const std::string& name);
// 不限制旧回放排版或图片路径；严格的标注工具输入限制仅在 read_session 中执行。
YAML::Node dataset_fingerprint(const std::filesystem::path& manifest);
SessionSource read_session(const std::filesystem::path& input);
bool within(const std::filesystem::path& child, const std::filesystem::path& parent);
void copy_images(const SessionSource& source, const std::filesystem::path& destination);

// 临时目录与目标在同一父目录。仅拥有本对象创建的目录，不清理已发布结果。
class OutputDirectory {
public:
  explicit OutputDirectory(const std::filesystem::path& destination);
  ~OutputDirectory();
  OutputDirectory(const OutputDirectory&) = delete;
  OutputDirectory& operator=(const OutputDirectory&) = delete;
  const std::filesystem::path& path() const;
  void publish(); // Linux renameat2(RENAME_NOREPLACE)，不支持时失败，不覆盖。

private:
  std::filesystem::path destination_;
  std::filesystem::path temporary_;
};
} // namespace autoaim::pipeline::annotation_detail
