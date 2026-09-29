#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace autoaim::pipeline::yaml_detail {
inline void write_text(const std::filesystem::path& path, const YAML::Node& node,
                       const char* error_message) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << node;
  std::ofstream output(path);
  output << emitter.c_str() << '\n';
  output.flush();
  if (!emitter.good() || !output)
    throw std::runtime_error(error_message);
}
} // namespace autoaim::pipeline::yaml_detail
