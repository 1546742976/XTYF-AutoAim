#pragma once

#include "autoaim/core/evidence.hpp"
#include "autoaim/core/result.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>

namespace autoaim::core {
// 配置仅提供数据；不创建硬件对象，不将配置声明升级为实测能力。
class Config {
public:
  static Result<Config> load(const std::filesystem::path& path);
  static Result<Config> parse(const std::string& text);

  template <class T> T require(const std::string& key) const {
    try {
      return lookup(key).as<T>();
    } catch (const YAML::Exception& error) {
      throw std::invalid_argument("Config '" + key + "': " + error.what());
    }
  }

  double number(const std::string& key, double minimum, double maximum) const;
  bool contains(const std::string& key) const;
  Evidence declared_evidence(const std::string& key) const;

private:
  explicit Config(YAML::Node root) : root_(std::move(root)) {
  }

  YAML::Node lookup(const std::string& key) const;
  YAML::Node root_;
};
} // namespace autoaim::core
