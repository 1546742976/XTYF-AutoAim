#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/core/fingerprint.hpp"
#include "autoaim/hal/session_writer.hpp"
#include "../../src/pipeline/session_annotation.hpp"
#include "test_support.hpp"

namespace {
int invoke(std::vector<std::string> arguments) {
  std::vector<char*> argv;
  for (auto& value : arguments)
    argv.push_back(value.data());

  return autoaim::pipeline::run_annotate_session(static_cast<int>(argv.size()), argv.data());
}
} // namespace

int main() {
  using namespace autoaim;
  namespace detail = pipeline::annotation_detail;

  return test::run([] {
    core::Fingerprint empty;
    CHECK(empty.value() == 0xcbf29ce484222325ULL);
    CHECK(empty.bytes() == 0);
    empty.append(nullptr, 0);
    CHECK(empty.value() == 0xcbf29ce484222325ULL);
    CHECK(empty.bytes() == 0);

    core::Fingerprint known;
    known.append("foobar", 6);
    CHECK(known.value() == 0x85944171f73967e8ULL);
    CHECK(known.bytes() == 6);
    core::Fingerprint split;
    split.append("foo", 3);
    split.append(nullptr, 0);
    split.append("bar", 3);
    CHECK(split.value() == known.value());
    CHECK(split.bytes() == 6);

    const unsigned char high_bytes[] = {0x00, 0x80, 0xff, 0x41};
    core::Fingerprint high;
    high.append(high_bytes, sizeof(high_bytes));
    CHECK(high.value() == 0xf8e5f57d2a3a01d5ULL);
    CHECK(high.bytes() == 4);

    detail::OutputDirectory sandbox(std::filesystem::temp_directory_path() / "annotation-unit");
    const auto root = sandbox.path();
    std::string boundary_bytes(65537, '\0');
    for (std::size_t i = 0; i < boundary_bytes.size(); ++i)
      boundary_bytes[i] = static_cast<char>((i * 37 + 11) & 255);
    struct FileCase {
      std::size_t bytes;
      const char* hash;
    };
    // 固定摘要由独立整数实现计算，覆盖 read/gcount 的 64 KiB 边界。
    const FileCase file_cases[] = {{0, "cbf29ce484222325"}, {65535, "49d1bbcaa0336b21"},
        {65536, "a2cde04e37602325"}, {65537, "03f752e8185bc72a"}};
    for (const auto& item : file_cases) {
      detail::write_bytes(root / "fingerprint.bin", boundary_bytes.substr(0, item.bytes));
      const auto fingerprint = detail::file_fingerprint(root / "fingerprint.bin", "blob");
      CHECK(fingerprint["path"].as<std::string>() == "blob");
      CHECK(fingerprint["bytes"].as<std::uint64_t>() == item.bytes);
      CHECK(fingerprint["fnv1a64"].as<std::string>() == item.hash);
    }

    const auto fixed = root / "fingerprint-set";
    CHECK(std::filesystem::create_directory(fixed));
    detail::write_bytes(fixed / "frame.bin",
        std::string(reinterpret_cast<const char*>(high_bytes), sizeof(high_bytes)));
    detail::write_bytes(fixed / "events.yaml",
        "events:\n  - {kind: image, path: ./frame.bin}\n"
        "  - {kind: image, path: frame.bin}\n");
    const auto fixed_fingerprint = detail::dataset_fingerprint(fixed / "events.yaml");
    CHECK(fixed_fingerprint["algorithm"].as<std::string>() == "fnv1a64-length-prefixed-v1");
    CHECK(fixed_fingerprint["fnv1a64"].as<std::string>() == "aa7de4a1369b4d99");
    const auto fixed_files = fixed_fingerprint["files"];
    CHECK(fixed_files.size() == 2);
    CHECK(fixed_files[0]["path"].as<std::string>() == "image/frame.bin");
    CHECK(fixed_files[0]["bytes"].as<std::uint64_t>() == 4);
    CHECK(fixed_files[0]["fnv1a64"].as<std::string>() == "f8e5f57d2a3a01d5");
    CHECK(fixed_files[1]["path"].as<std::string>() == "manifest/events.yaml");
    CHECK(fixed_files[1]["bytes"].as<std::uint64_t>() == 80);
    CHECK(fixed_files[1]["fnv1a64"].as<std::string>() == "5f90348620844bb5");

    const auto input = root / "source";
    hal::SessionWriter writer(input, {"unit", "v1", std::nullopt});
    for (int id = 1; id <= 3; ++id) {
      const core::TimePoint at(id * 1000, core::ClockDomain::replay);
      if (id == 1) {
        writer.append(at, hal::GimbalFeedback{at, 1, {1, 0, 0, 0}, 0, 0, 20, true, true, {}});
        writer.append(at, hal::UartChunk{at, {0xab, 1, 2}});
        writer.append(at, hal::ButtonSample{at, true, false, false});
      }
      const auto pixels = std::make_shared<std::vector<std::uint8_t>>(64 * 48 * 3, 0);
      core::CaptureMetadata capture;
      capture.device_timestamp_ticks = 9007199254740993ull;
      writer.append(at, std::make_shared<const core::CapturedFrame>(core::Stamp(id, 1, at),
          core::Image(64, 48, 192, pixels), at, core::TimeOrigin::synthetic,
          core::Seconds(0), core::Evidence::missing(), capture),
          id == 1 ? YAML::Node() : YAML::Load("[]"));
    }
    writer.finish();
    const auto raw = detail::read_bytes(input / "events.yaml");
    std::string crlf = "# Original non-image comment\r\n";
    for (const char byte : raw) {
      if (byte == '\n')
        crlf += '\r';
      crlf += byte;
    }
    detail::write_bytes(input / "events.yaml", crlf);
    const auto source_files = [&] {
      std::map<std::string, std::string> files;
      for (const auto& entry : std::filesystem::recursive_directory_iterator(input))
        if (entry.is_regular_file())
          files.emplace(entry.path().lexically_relative(input).generic_string(),
                        detail::read_bytes(entry.path()));

      return files;
    };
    const auto before_files = source_files();
    const auto original = detail::dataset_fingerprint(input / "events.yaml");
    const auto exported = root / "exported";
    CHECK(invoke({"annotate", "--export", exported.string(), "--session", input.string()}) == 0);
    const auto index = detail::read_document(exported / "index.yaml");
    CHECK(index["frames"].size() == 3);
    CHECK(index["frames"][0]["width"].as<int>() == 64);
    CHECK(!index["frames"][0]["feedback_reference"]["exact_pose"].IsNull());
    CHECK(index["frames"][1]["feedback_reference"]["exact_pose"].IsNull());

    // 采样时间可不同于接收顺序；同时间 before/after 取首条，exact 取首条显式有效姿态。
    const auto indexed_input = root / "feedback-source";
    hal::SessionWriter feedback_writer(indexed_input, {});
    const core::TimePoint received(10000, core::ClockDomain::replay);
    for (int id = 0; id < 5; ++id) {
      const std::int64_t times[] = {3000, 1000, 1000, 1000, 2000};
      feedback_writer.append(received, hal::GimbalFeedback{
          core::TimePoint(times[id], core::ClockDomain::replay), 1, {1, 0, 0, 0},
          double(id), 0, 20, id != 1, true, {}});
    }
    for (int id = 1; id <= 3; ++id) {
      const auto pixels = std::make_shared<std::vector<std::uint8_t>>(64 * 48 * 3, 0);
      const core::Stamp stamp(id, 1, core::TimePoint(id * 1000, core::ClockDomain::replay));
      feedback_writer.append(received, std::make_shared<const core::CapturedFrame>(stamp,
          core::Image(64, 48, 192, pixels), received, core::TimeOrigin::synthetic,
          core::Seconds(0), core::Evidence::missing()));
    }
    feedback_writer.finish();
    auto feedback_bytes = detail::read_bytes(indexed_input / "events.yaml");
    // 首条反馈缺 sampled_ns，合法回退到接收时间，不可当作曝光同时间精确姿态。
    const auto position = feedback_bytes.find("sampled_ns: 3000,");
    CHECK(position != std::string::npos);
    feedback_bytes.erase(position, std::string("sampled_ns: 3000,").size());
    detail::write_bytes(indexed_input / "events.yaml", feedback_bytes);
    const auto indexed_output = root / "feedback-export";
    CHECK(invoke({"annotate", "--export", indexed_output.string(),
                   "--session", indexed_input.string()}) == 0);
    const auto indexed = detail::read_document(indexed_output / "index.yaml")["frames"];
    const auto reference = indexed[0]["feedback_reference"];
    CHECK(reference["before"]["event"]["yaw_rad"].as<double>() == 1);
    CHECK(reference["after"]["event"]["yaw_rad"].as<double>() == 1);
    CHECK(reference["exact_pose"]["event"]["yaw_rad"].as<double>() == 2);
    CHECK(indexed[2]["feedback_reference"]["before"]["sampled_ns"].as<int>() == 2000);
    CHECK(indexed[2]["feedback_reference"]["after"]["sample_time_source"].as<std::string>() ==
          "replay_at_ns_fallback");
    CHECK(indexed[2]["feedback_reference"]["exact_pose"].IsNull());
    const auto draft = detail::read_document(exported / "annotations.yaml");
    CHECK(!draft["reviewed"].as<bool>());
    CHECK(draft["frames"][0]["annotations"].IsNull());
    CHECK(draft["frames"][1]["annotations"].IsSequence());
    CHECK(detail::same_yaml(original, draft["source_fingerprint"]));
    CHECK(detail::read_bytes(input / "frame_1.png") ==
          detail::read_bytes(exported / "frames/frame_1.png"));
    detail::write_bytes(exported / "frames/frame_1.png", "changed independently");
    CHECK(detail::same_yaml(original, detail::dataset_fingerprint(input / "events.yaml")));
    CHECK(invoke({"annotate", "--export", exported.string(), "--session", input.string()}) == 1);
    CHECK(invoke({"annotate", "--export", (input / "forbidden").string(),
                   "--session", input.string()}) == 1);
    auto block = detail::read_document(input / "events.yaml");
    block["events"].SetStyle(YAML::EmitterStyle::Block);
    block["events"][0].SetStyle(YAML::EmitterStyle::Block);
    detail::write_yaml(root / "block.yaml", block);
    CHECK_THROWS(std::exception, detail::read_session(root / "block.yaml"));
    auto reviewed = YAML::Clone(draft);
    reviewed["reviewed"] = true;
    reviewed["frames"].remove(2);
    reviewed["frames"][0]["annotations"] = YAML::Load(R"([
      {category: unknown, color: red, plate_type: small, track_id: board,
       corners: [[10,10],[30,10],[30,25],[10,25]], visible: [true,true,true,true]}])");
    const auto labels = root / "reviewed.yaml";
    detail::write_yaml(labels, reviewed);
    const auto source = detail::read_session(input);
    const auto check = [&] {
      detail::write_yaml(labels, reviewed);

      return detail::check_annotations(source, labels, source.fingerprint);
    };
    CHECK(check().problems.size() == 0);
    CHECK(invoke({"annotate", "--check", "--session", input.string(), "--annotations",
                   labels.string(), "--output", (root / "review").string()}) == 0);
    CHECK(std::filesystem::exists(root / "review/frames/frame_1.png"));
    const auto clean = YAML::Clone(reviewed);
    for (int mode = 0; mode < 11; ++mode) {
      reviewed = YAML::Clone(clean);
      if (mode == 0)
        reviewed["reviewed"] = false;
      else if (mode == 1)
        reviewed["source_fingerprint"]["fnv1a64"] = "wrong";
      else if (mode == 2)
        reviewed["frames"][0]["frame_id"] = 99;
      else if (mode == 3)
        reviewed["frames"].push_back(YAML::Clone(reviewed["frames"][0]));
      else if (mode == 4)
        reviewed["frames"][0].remove("annotations");
      else if (mode == 5)
        reviewed["frames"][0]["annotations"] = YAML::Node(YAML::NodeType::Null);
      else if (mode == 6)
        reviewed["frames"][0]["annotations"][0]["corners"].remove(0);
      else if (mode == 7)
        reviewed["frames"][0]["annotations"][0]["corners"][0][0] = 99;
      else if (mode == 8)
        reviewed["frames"][0]["annotations"][0]["color"] = "green";
      else if (mode == 9)
        reviewed["frames"][0]["annotations"].push_back(
            YAML::Clone(reviewed["frames"][0]["annotations"][0]));
      else
        reviewed["frames"][0]["annotations"][0]["corners"] =
            YAML::Load("[[10,10],[10,10],[10,10],[10,10]]");
      CHECK(check().problems.size() > 0);
    }
    reviewed = YAML::Clone(clean);
    reviewed["frames"][0]["annotations"][0]["corners"][0][0] = -100;
    reviewed["frames"][0]["annotations"][0]["visible"][0] = false;
    CHECK(check().problems.size() == 0);
    reviewed["frames"][0]["annotations"][0]["color"] = "green";
    reviewed["frames"][1]["annotations"] = YAML::Node(YAML::NodeType::Null);
    CHECK(check().problems.size() == 2);
    CHECK(invoke({"annotate", "--check", "--session", input.string(), "--annotations",
                   labels.string(), "--output", (root / "bad-review").string()}) == 1);
    CHECK(detail::read_document(root / "bad-review/check.yaml")["problems"].size() == 2);
    CHECK(!std::filesystem::exists(root / "bad-review/frames/frame_1.png"));
    reviewed = YAML::Clone(clean);
    // 改为人工常用 block 排版，应用后事件仍必须为单行 flow。
    reviewed["frames"][0]["annotations"].SetStyle(YAML::EmitterStyle::Block);
    reviewed["frames"][0]["annotations"][0].SetStyle(YAML::EmitterStyle::Block);
    CHECK(check().problems.size() == 0);
    const auto derived = detail::read_bytes(input / "derived.yaml");
    const auto applied = root / "applied";
    CHECK(invoke({"annotate", "--apply", "--session", input.string(), "--annotations",
                   labels.string(), "--output", applied.string()}) == 0);
    CHECK(!std::filesystem::exists(applied / "derived.yaml"));
    const auto copy = detail::read_session(applied);
    CHECK(copy.images.at(1).event["annotations"].size() == 1);
    CHECK(copy.images.at(2).event["annotations"].IsSequence());
    CHECK(!copy.images.at(3).event["annotations"].IsDefined());
    CHECK(source.images.at(3).event["annotations"].IsSequence());
    CHECK(copy.images.at(1).event["device_timestamp_ticks"].as<std::uint64_t>() ==
          9007199254740993ull);
    const auto binding = detail::annotation_provenance(applied / "events.yaml");
    CHECK(binding["annotations_reviewed"].as<bool>());
    CHECK(source.lines.size() == copy.lines.size());
    for (std::size_t line = 0; line < source.lines.size(); ++line) {
      bool image_line = false;
      for (const auto& image : source.images)
        image_line = image_line || image.second.line == line;
      if (!image_line)
        CHECK(source.lines[line] == copy.lines[line]);
    }
    CHECK(detail::same_yaml(original, detail::dataset_fingerprint(input / "events.yaml")));
    CHECK(derived == detail::read_bytes(input / "derived.yaml"));
    CHECK(source_files() == before_files);
    CHECK(invoke({"annotate", "--apply", "--session", input.string(), "--annotations",
                   labels.string(), "--output", applied.string()}) == 1);
    detail::write_bytes(applied / "frame_1.png", "independent applied bytes");
    CHECK(detail::same_yaml(original, detail::dataset_fingerprint(input / "events.yaml")));
    CHECK_THROWS(std::exception, detail::annotation_provenance(applied / "events.yaml"));

    // 竞争者创建目标目录后，发布必须失败；不得用 rename 覆盖其空目录。
    detail::OutputDirectory racing(root / "racing");
    std::filesystem::create_directory(root / "racing");
    CHECK_THROWS(std::system_error, racing.publish());
    CHECK(std::filesystem::is_directory(root / "racing"));

    const auto reject_source = [&](const std::string& name, auto change) {
      const auto bad = root / name;
      std::filesystem::create_directory(bad);
      detail::copy_images(source, bad);
      auto document = detail::read_document(source.manifest);
      change(document, bad);
      detail::write_yaml(bad / "events.yaml", document);
      CHECK(invoke({"annotate", "--export", (root / (name + "-out")).string(),
                     "--session", bad.string()}) == 1);
      CHECK(!std::filesystem::exists(root / (name + "-out")));
    };
    reject_source("missing-image", [](YAML::Node&, const auto& path) {
      std::filesystem::remove(path / "frame_2.png");
    });
    reject_source("incomplete", [](YAML::Node& doc, const auto&) {
      doc["complete"] = false;
    });
    reject_source("duplicate-id", [](YAML::Node& doc, const auto&) {
      doc["events"][4]["frame_id"] = 1;
    });
    reject_source("bad-order", [](YAML::Node& doc, const auto&) {
      doc["events"][4]["at_ns"] = 0;
    });
    reject_source("outside-image", [](YAML::Node& doc, const auto&) {
      doc["events"][3]["path"] = "../source/frame_1.png";
    });
    reject_source("absolute-image", [&](YAML::Node& doc, const auto&) {
      doc["events"][3]["path"] = (input / "frame_1.png").string();
    });
    CHECK(source_files() == before_files);
  });
}
