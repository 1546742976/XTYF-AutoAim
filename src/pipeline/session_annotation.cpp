#include "session_annotation.hpp"
#include "autoaim/core/config.hpp"
#include "autoaim/core/fingerprint.hpp"
#include "autoaim/hal/file_replay.hpp"
#include "autoaim/vision/annotation.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <system_error>
#ifdef __linux__
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace autoaim::pipeline::annotation_detail {
namespace fs = std::filesystem;
namespace {
struct Fingerprint : core::Fingerprint {
  // 集合编码：每个字符串之前是固定 8 字节小端长度，避免路径/摘要拼接歧义。
  void field(const std::string& value) {
    std::array<char, 8> length{};
    for (std::size_t i = 0; i < length.size(); ++i)
      length[i] = static_cast<char>((std::uint64_t(value.size()) >> (8 * i)) & 255);
    append(length.data(), length.size());
    append(value.data(), value.size());
  }

  std::string hex() const {
    std::ostringstream stream;
    stream << std::hex << std::setw(16) << std::setfill('0') << value();

    return stream.str();
  }
};
} // namespace

std::string read_bytes(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot read file: " + path.string());
  const std::string bytes{std::istreambuf_iterator<char>(input), {}};
  if (input.bad())
    throw std::runtime_error("File read failed: " + path.string());

  return bytes;
}

void write_bytes(const fs::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  output.flush();
  if (!output)
    throw std::runtime_error("File write failed: " + path.string());
}

void write_yaml(const fs::path& path, const YAML::Node& node) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter.SetFloatPrecision(9);
  emitter << node;
  if (!emitter.good())
    throw std::runtime_error("YAML serialization failed: " + path.string());
  write_bytes(path, std::string(emitter.c_str()) + '\n');
}

YAML::Node read_document(const fs::path& path) {
  const auto bytes = read_bytes(path);
  const auto checked = core::Config::parse(bytes);
  if (!checked)
    throw std::invalid_argument(path.string() + ": " + checked.error().message);
  // 同一份字节再取私有根节点，不给 Config 增加公开的可变根访问器。
  const auto documents = YAML::LoadAll(bytes);
  if (documents.size() != 1)
    throw std::invalid_argument("Exactly one YAML document required: " + path.string());

  return documents.front();
}

bool same_yaml(const YAML::Node& left, const YAML::Node& right) {
  if (!left.IsDefined() || !right.IsDefined())
    return left.IsDefined() == right.IsDefined();
  if (left.Type() != right.Type())
    return false;
  if (left.IsScalar())
    return left.Scalar() == right.Scalar();
  if (left.size() != right.size())
    return false;
  if (left.IsSequence()) {
    for (std::size_t i = 0; i < left.size(); ++i)
      if (!same_yaml(left[i], right[i]))
        return false;
  } else if (left.IsMap()) {
    for (const auto& pair : left)
      if (!same_yaml(pair.second, right[pair.first.as<std::string>()]))
        return false;
  }

  return true;
}

YAML::Node file_fingerprint(const fs::path& path, const std::string& name) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot fingerprint file: " + path.string());
  Fingerprint digest;
  std::array<char, 65536> buffer;
  while (input.read(buffer.data(), buffer.size()) || input.gcount())
    digest.append(buffer.data(), static_cast<std::size_t>(input.gcount()));
  if (!input.eof())
    throw std::runtime_error("Fingerprint read failed: " + path.string());
  YAML::Node result;
  result["path"] = name;
  result["bytes"] = digest.bytes();
  result["fnv1a64"] = digest.hex();

  return result;
}

YAML::Node dataset_fingerprint(const fs::path& manifest) {
  const auto document = read_document(manifest);
  std::map<std::string, fs::path> paths;
  paths.emplace("manifest/" + manifest.filename().generic_string(), manifest);
  for (const auto& event : document["events"]) {
    if (event["kind"].as<std::string>() != "image")
      continue;
    const fs::path name(event["path"].as<std::string>());
    paths.emplace("image/" + name.lexically_normal().generic_string(),
                  manifest.parent_path() / name);
  }
  YAML::Node result;
  result["algorithm"] = "fnv1a64-length-prefixed-v1";
  result["files"] = YAML::Node(YAML::NodeType::Sequence);
  Fingerprint combined;
  combined.field("fnv1a64-length-prefixed-v1");
  for (const auto& entry : paths) {
    const auto item = file_fingerprint(entry.second, entry.first);
    combined.field(entry.first);
    combined.field(item["bytes"].as<std::string>());
    combined.field(item["fnv1a64"].as<std::string>());
    result["files"].push_back(item);
  }
  result["fnv1a64"] = combined.hex();

  return result;
}

bool within(const fs::path& child, const fs::path& parent) {
  const auto a = fs::weakly_canonical(child);
  const auto b = fs::weakly_canonical(parent);
  const auto mismatch = std::mismatch(b.begin(), b.end(), a.begin(), a.end());

  return mismatch.first == b.end();
}

SessionSource read_session(const fs::path& input) {
  SessionSource result;
  result.manifest = fs::canonical(fs::is_directory(input) ? input / "events.yaml" : input);
  const auto document = read_document(result.manifest);
  if (document["schema_version"].as<int>() != 2 ||
      document["domain"].as<std::string>() != "replay" || !document["complete"].as<bool>())
    throw std::invalid_argument("Annotation tool requires a complete v2 replay session");
  const auto bytes = read_bytes(result.manifest);
  for (std::size_t begin = 0; begin < bytes.size();) {
    const auto newline = bytes.find('\n', begin);
    const auto end = newline == std::string::npos ? bytes.size() : newline + 1;
    result.lines.push_back(bytes.substr(begin, end - begin));
    begin = end;
  }
  result.events = document["events"].as<std::vector<YAML::Node>>();
  for (const auto& event : result.events) {
    const auto line = static_cast<std::size_t>(event.Mark().line);
    const auto& raw = result.lines.at(line);
    const auto start = raw.find_first_not_of(" \t");
    if (start == std::string::npos || raw.compare(start, 3, "- {") != 0 ||
        !same_yaml(YAML::Load(raw.substr(start + 2)), event))
      throw std::invalid_argument("Annotation tool requires single-line flow events");
    if (event["kind"].as<std::string>() != "image")
      continue;
    const fs::path name(event["path"].as<std::string>());
    const auto image = result.manifest.parent_path() / name;
    if (name.is_absolute() || name.empty() ||
        std::find(name.begin(), name.end(), fs::path("..")) != name.end() ||
        name.extension() != ".png" ||
        !within(image, result.manifest.parent_path()) || !fs::is_regular_file(image))
      throw std::invalid_argument(
          "Image must be a PNG inside the source session: " + name.string());
    const auto id = event["frame_id"].as<core::FrameId>();
    if (!result.images.emplace(id, ImageEntry{event, line, name, {}}).second)
      throw std::invalid_argument("Repeated frame_id: " + std::to_string(id));
  }
  if (result.images.empty())
    throw std::invalid_argument("Annotation session has no images");

  // 构造只能检查清单；必须消费至末尾才能检查图片、帧号和每种事件的字段。
  hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
  hal::FileReplay replay(result.manifest, clock);
  for (;;) {
    const auto event = replay.next(1);
    if (!event)
      throw std::invalid_argument(event.error().message);
    if (std::holds_alternative<hal::ReplayEnd>(event.value()))
      break;
    if (const auto* image =
            std::get_if<std::shared_ptr<const core::CapturedFrame>>(&event.value()))
      result.images.at((*image)->stamp.frame_id).size =
          {(*image)->image.width, (*image)->image.height};
  }
  result.fingerprint = dataset_fingerprint(result.manifest);

  return result;
}

void copy_images(const SessionSource& source, const fs::path& destination) {
  std::set<fs::path> copied;
  for (const auto& entry : source.images) {
    const auto name = entry.second.relative_path.lexically_normal();
    if (!copied.insert(name).second)
      continue;
    const auto target = destination / name;
    fs::create_directories(target.parent_path());
    fs::copy_file(source.manifest.parent_path() / name, target, fs::copy_options::none);
    const auto original = file_fingerprint(source.manifest.parent_path() / name, name.string());
    if (!same_yaml(original, file_fingerprint(target, name.string())))
      throw std::runtime_error("Copied image differs: " + name.string());
  }
}

AnnotationCheck check_annotations(const SessionSource& source, const fs::path& annotations,
                                  const YAML::Node& expected_source) {
  AnnotationCheck result;
  result.document = read_document(annotations);
  const YAML::Node document = result.document;
  const auto problem = [&](core::FrameId id, std::size_t row, std::size_t item,
                           const std::string& reason) {
    YAML::Node value;
    value["frame_id"] = id ? YAML::Node(id) : YAML::Node(YAML::NodeType::Null);
    value["frame_entry"] = row ? YAML::Node(row) : YAML::Node(YAML::NodeType::Null);
    value["annotation_entry"] = item ? YAML::Node(item) : YAML::Node(YAML::NodeType::Null);
    value["reason"] = reason;
    result.problems.push_back(value);
  };
  try {
    if (document["schema_version"].as<int>() != 1)
      throw std::invalid_argument("Annotation schema_version must be 1");
  } catch (const std::exception& error) {
    problem(0, 0, 0, error.what());
  }
  result.source_matches = same_yaml(document["source_fingerprint"], expected_source);
  if (!result.source_matches)
    problem(0, 0, 0, "Source fingerprint mismatch");
  try {
    if (!document["reviewed"].as<bool>())
      throw std::invalid_argument("Explicit reviewed: true required");
  } catch (const std::exception& error) {
    problem(0, 0, 0, error.what());
  }
  if (!document["frames"].IsSequence()) {
    problem(0, 0, 0, "Frames must be an explicit sequence");

    return result;
  }

  std::set<core::FrameId> seen;
  for (std::size_t row = 0; row < document["frames"].size(); ++row) {
    core::FrameId id = 0;
    try {
      const auto frame = document["frames"][row];
      id = frame["frame_id"].as<core::FrameId>();
      if (!source.images.count(id))
        throw std::invalid_argument("Unknown frame_id");
      if (!seen.insert(id).second) {
        result.frames.erase(id);
        throw std::invalid_argument("Repeated frame_id in annotation list");
      }
      const auto values = frame["annotations"];
      if (!values.IsDefined())
        throw std::invalid_argument("Missing annotations key; omit unreviewed frame instead");
      if (values.IsNull())
        throw std::invalid_argument("Null annotations are unfinished drafts, not negative frames");
      if (!values.IsSequence())
        throw std::invalid_argument("Annotations must be an explicit sequence");
      const auto initial_errors = result.problems.size();
      YAML::Node valid(YAML::NodeType::Sequence);
      std::vector<std::size_t> ordinals;
      std::vector<std::string> identities;
      const auto size = source.images.at(id).size;
      for (std::size_t item = 0; item < values.size(); ++item) {
        try {
          YAML::Node single(YAML::NodeType::Sequence);
          single.push_back(YAML::Clone(values[item]));
          const auto parsed = vision::read_annotations(single, size);
          valid.push_back(YAML::Clone(values[item]));
          identities.push_back(parsed.front().track_id);
          ordinals.push_back(item + 1);
        } catch (const std::exception& error) {
          problem(id, row + 1, item + 1, error.what());
        }
      }
      try {
        vision::read_annotations(valid, size);
      } catch (const std::exception& error) {
        // 整帧规则仍由 vision 判定；下面只把重复身份错误定位回原条目。
        std::map<std::string, std::size_t> first;
        bool located = false;
        for (std::size_t i = 0; i < identities.size(); ++i) {
          if (identities[i].empty())
            continue;
          const auto inserted = first.emplace(identities[i], ordinals[i]);
          if (!inserted.second) {
            problem(id, row + 1, ordinals[i], "Repeated track_id; first annotation entry=" +
                                              std::to_string(inserted.first->second));
            located = true;
          }
        }
        if (!located)
          problem(id, row + 1, 0, error.what());
      }
      if (initial_errors == result.problems.size())
        result.frames.emplace(id, YAML::Clone(values));
    } catch (const std::exception& error) {
      problem(id, row + 1, 0, error.what());
    }
  }

  return result;
}

OutputDirectory::OutputDirectory(const fs::path& destination)
    : destination_(fs::absolute(destination).lexically_normal()) {
  if (fs::exists(destination_) || fs::is_symlink(fs::symlink_status(destination_)) ||
      !fs::is_directory(destination_.parent_path()))
    throw std::invalid_argument("Output must be a new directory with an existing parent");
#ifdef __linux__
  const auto pattern = destination_.string() + ".tmp-XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const auto created = ::mkdtemp(buffer.data());
  if (!created)
    throw std::system_error(errno, std::generic_category(), "Cannot create temporary directory");
  temporary_ = created;
#else
  throw std::runtime_error("Atomic annotation outputs require Linux/WSL");
#endif
}

YAML::Node annotation_provenance(const fs::path& manifest) {
  YAML::Node result;
  for (const auto* key : {"annotations_file", "annotations_reviewed", "annotations_fingerprint"})
    result[key] = YAML::Node(YAML::NodeType::Null);
  const auto sidecar_path = manifest.parent_path() / "annotation_provenance.yaml";
  if (!fs::exists(sidecar_path))
    return result;
  const auto provenance = read_document(sidecar_path);
  if (provenance["schema_version"].as<int>() != 1 || !provenance["reviewed"].as<bool>() ||
      provenance["annotations_file"].as<std::string>() != "annotations.yaml")
    throw std::invalid_argument("Unsupported annotation provenance");
  const auto source = read_session(manifest);
  if (!same_yaml(provenance["output_fingerprint"], source.fingerprint))
    throw std::invalid_argument("Annotated session fingerprint mismatch");
  const auto labels = manifest.parent_path() / "annotations.yaml";
  if (!within(labels, manifest.parent_path()))
    throw std::invalid_argument("Annotation sidecar must be inside the session");
  const auto fingerprint = file_fingerprint(labels, "annotations.yaml");
  if (!same_yaml(provenance["annotations_fingerprint"], fingerprint))
    throw std::invalid_argument("Annotation file fingerprint mismatch");
  if (!provenance["source_fingerprint"].IsMap())
    throw std::invalid_argument("Missing original source fingerprint");
  const auto checked = check_annotations(source, labels, provenance["source_fingerprint"]);
  if (checked.problems.size())
    throw std::invalid_argument(
        "Bound annotation validation failed: " + YAML::Dump(checked.problems));
  for (const auto& image : source.images) {
    const auto found = checked.frames.find(image.first);
    const YAML::Node embedded = image.second.event["annotations"];
    if ((found == checked.frames.end() && embedded.IsDefined()) ||
        (found != checked.frames.end() && !same_yaml(found->second, embedded)))
      throw std::invalid_argument("Embedded annotations differ from reviewed frame list");
  }
  result["annotations_file"] = fs::absolute(labels).string();
  result["annotations_reviewed"] = true;
  result["annotations_fingerprint"] = fingerprint;
  result["annotation_source_fingerprint"] = YAML::Clone(provenance["source_fingerprint"]);

  return result;
}

OutputDirectory::~OutputDirectory() {
  if (!temporary_.empty()) {
    std::error_code ignored;
    fs::remove_all(temporary_, ignored);
  }
}

const fs::path& OutputDirectory::path() const {
  return temporary_;
}

void OutputDirectory::publish() {
#ifdef __linux__
  if (::syscall(SYS_renameat2, AT_FDCWD, temporary_.c_str(), AT_FDCWD,
                destination_.c_str(), RENAME_NOREPLACE) != 0)
    throw std::system_error(errno, std::generic_category(), "Atomic no-overwrite publish failed");
  temporary_.clear();
#else
  throw std::runtime_error("Atomic annotation outputs require Linux/WSL");
#endif
}
} // namespace autoaim::pipeline::annotation_detail
