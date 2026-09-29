#include "autoaim/core/config.hpp"
#include <fstream>
#include <iterator>
#include <set>

namespace autoaim::core {
namespace {
void validate_map(const YAML::Node& node, unsigned depth = 0) {
  if (depth > 32)
    throw std::invalid_argument("Config nesting exceeds 32 levels");

  if (node.IsMap()) {
    std::set<std::string> keys;

    for (const auto& entry : node) {
      if (!entry.first.IsScalar() || !keys.insert(entry.first.as<std::string>()).second)
        throw std::invalid_argument("Non-scalar or duplicate config key");

      validate_map(entry.second, depth + 1);
    }
  } else if (node.IsSequence()) {
    for (const auto& entry : node)
      validate_map(entry, depth + 1);
  }
}
} // namespace

Result<Config> Config::parse(const std::string& text) {
  try {
    auto root = YAML::Load(text);

    if (!root.IsMap())
      throw std::invalid_argument("Config root must be a map");

    validate_map(root);

    return Result<Config>::success(Config(std::move(root)));
  } catch (const YAML::Exception& error) {
    return Result<Config>::failure(ErrorCode::invalid_input, error.what());
  } catch (const std::invalid_argument& error) {
    return Result<Config>::failure(ErrorCode::invalid_input, error.what());
  }
}

Result<Config> Config::load(const std::filesystem::path& path) {
  std::ifstream input(path);

  if (!input)
    return Result<Config>::failure(ErrorCode::io, "Cannot open config: " + path.string());

  const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};

  if (input.bad())
    return Result<Config>::failure(ErrorCode::io, "Cannot read config: " + path.string());

  auto result = parse(text);

  if (!result)
    return Result<Config>::failure(result.error().code,
                                   path.string() + ": " + result.error().message);

  return result;
}

YAML::Node Config::lookup(const std::string& key) const {
  if (key.empty())
    throw std::invalid_argument("Empty config key");

  YAML::Node cursor = root_;
  std::size_t start = 0;

  while (start <= key.size()) {
    const auto end = key.find('.', start);
    const auto part = key.substr(start, end == std::string::npos ? end : end - start);
    const YAML::Node view = cursor;

    if (part.empty() || !view.IsMap() || !view[part].IsDefined())
      throw std::invalid_argument("Missing config key: " + key);

    // Node 的赋值会修改共享节点；reset 只改变游标绑定。
    cursor.reset(view[part]);

    if (end == std::string::npos)
      break;

    start = end + 1;
  }

  return cursor;
}

double Config::number(const std::string& key, double minimum, double maximum) const {
  const double value = require<double>(key);

  if (!std::isfinite(value) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
      minimum > maximum || value < minimum || value > maximum)
    throw std::invalid_argument("Config number outside range: " + key);

  return value;
}

Evidence Config::declared_evidence(const std::string& key) const {
  return require<bool>(key) ? Evidence::declared() : Evidence::missing();
}

bool Config::contains(const std::string& key) const {
  try {
    return lookup(key).IsDefined();
  } catch (const std::invalid_argument&) {
    return false;
  }
}
} // namespace autoaim::core
