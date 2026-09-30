#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace test {
struct ConfigFixture {
  std::filesystem::path entry;
  std::filesystem::path base;
  std::filesystem::path fast_choose;
  std::map<std::string, std::filesystem::path> bases;
};

namespace config_fixture_detail {
inline std::filesystem::path resolve(const std::filesystem::path& directory,
                                     const std::filesystem::path& value) {
  return std::filesystem::weakly_canonical(std::filesystem::absolute(directory / value));
}

inline void write(const std::filesystem::path& path, const YAML::Node& node) {
  std::ofstream output(path, std::ios::binary);
  output << node << '\n';
  if (!output)
    throw std::runtime_error("Cannot write configuration fixture: " + path.string());
}

inline void absolute_paths(YAML::Node node, const std::filesystem::path& owner) {
  for (const char* key : {"input_manifest", "calibration_file"})
    if (const YAML::Node value = std::as_const(node)[key]; value.IsDefined())
      node[key] = resolve(owner.parent_path(), value.as<std::string>()).string();
  if (const YAML::Node files = std::as_const(node)["geometry_files"];
      files.IsDefined() && files.IsSequence())
    for (std::size_t i = 0; i < files.size(); ++i)
      node["geometry_files"][i] = resolve(owner.parent_path(), files[i].as<std::string>()).string();
  const YAML::Node detector = std::as_const(node)["detector"];
  if (detector.IsDefined() && detector.IsMap() && detector["model_path"].IsDefined())
    node["detector"]["model_path"] =
        resolve(owner.parent_path(), detector["model_path"].as<std::string>()).string();
}
} // namespace config_fixture_detail

// 深复制快调及三份场景声明；标定、几何和模型等资产仍指向明确的原始绝对路径。
// 修改快调参数时写 fixture.fast_choose，修改场景契约时写 fixture.base/bases。
inline ConfigFixture copy_config_fixture(const std::filesystem::path& original,
                                         const std::filesystem::path& destination) {
  namespace detail = config_fixture_detail;
  std::filesystem::create_directories(destination);
  const auto input = detail::resolve({}, original);
  const YAML::Node entry = YAML::LoadFile(input.string());
  ConfigFixture result;
  if (!entry["fast_choose_file"].IsDefined() && !entry["fast_choose_version"].IsDefined()) {
    auto copied = YAML::Clone(entry);
    detail::absolute_paths(copied, input);
    result.base = detail::resolve(destination, "base.yaml");
    result.entry = result.base;
    detail::write(result.base, copied);
    return result;
  }

  const bool fast_entry = entry["fast_choose_version"].IsDefined();
  const auto original_fast = fast_entry ? input : detail::resolve(
      input.parent_path(), entry["fast_choose_file"].as<std::string>());
  YAML::Node fast = YAML::LoadFile(original_fast.string());
  const std::string selected = fast_entry ? fast["active_detector"].as<std::string>()
                                          : entry["detector"]["kind"].as<std::string>();
  result.fast_choose = detail::resolve(destination, "fast_choose.yaml");
  detail::absolute_paths(fast["common"], original_fast);
  for (const char* kind : {"traditional", "yolov5", "yolo11"}) {
    YAML::Node detector = fast["detectors"][kind];
    const auto original_base = !fast_entry && selected == kind ? input : detail::resolve(
        original_fast.parent_path(), detector["config_file"].as<std::string>());
    auto base = YAML::LoadFile(original_base.string());
    detail::absolute_paths(base, original_base);
    base["fast_choose_file"] = result.fast_choose.string();
    const auto copied_base = detail::resolve(destination, std::string("base-") + kind + ".yaml");
    detail::write(copied_base, base);
    result.bases.emplace(kind, copied_base);
    detector["config_file"] = copied_base.string();
    const YAML::Node model = std::as_const(detector)["model_path"];
    if (model.IsDefined())
      detector["model_path"] =
          detail::resolve(original_fast.parent_path(), model.as<std::string>()).string();
  }
  detail::write(result.fast_choose, fast);
  result.base = result.bases.at(selected);
  result.entry = fast_entry ? result.fast_choose : result.base;
  return result;
}
} // namespace test
