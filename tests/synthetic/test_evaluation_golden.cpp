#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/vision/evaluation.hpp"
#include "../../src/pipeline/session_annotation.hpp"
#include "test_support.hpp"

namespace {
void check_hand_calculated_metrics() {
  using namespace autoaim::vision;
  const auto truth = read_annotations(YAML::Load(R"([
    {category: unknown, color: red, plate_type: small, track_id: a,
     corners: [[10,10],[30,10],[30,30],[10,30]], visible: [true,true,true,true]}])"), {100, 100});
  Detection detection{truth[0].corners, 1, TeamColor::red, 0, false,
                       TargetCategory::unknown, ArmorSize::small};
  for (auto& point : detection.corners)
    point.x += 1;
  auto distant = detection;
  for (auto& point : distant.corners)
    point.x += 50;
  const auto frame = evaluate_frame(truth,
      {{detection, std::nullopt}, {distant, std::nullopt}}, {0.5});
  CHECK(frame.truths == 1 && frame.predictions == 2 && frame.matched == 1);
  CHECK(double(frame.matched) / frame.predictions == 0.5);
  CHECK(double(frame.matched) / frame.truths == 1.0);
  CHECK(frame.visible_corners == 4 && frame.corner_error_squared_px == 4.0);
  CHECK(std::sqrt(frame.corner_error_squared_px / frame.visible_corners) == 1.0);

  SequenceEvaluation sequence({0.5});
  const EvaluationFrameState state{true, false, false};
  // 有候选但零匹配也算连续漏检；未知帧打断连续统计，且不把其候选算作 FP。
  sequence.add(1, truth, {{distant, std::nullopt}}, state, InterventionGroup::unknown);
  sequence.add(2, truth, {}, state, InterventionGroup::unknown);
  sequence.add(3, std::nullopt, {{distant, std::nullopt}}, state, InterventionGroup::unknown);
  sequence.add(4, truth, {}, state, InterventionGroup::unknown);
  sequence.add(5, std::vector<ArmorAnnotation>{}, {{distant, std::nullopt}}, state,
                InterventionGroup::unknown);
  sequence.add(6, truth, {{detection, std::nullopt}}, state, InterventionGroup::unknown);
  CHECK(sequence.maximum_empty_detection_streak() == 2);
  CHECK(sequence.totals().unlabeled_frames == 1 && sequence.totals().negative_frames == 1);
  CHECK(sequence.totals().detections.truths == 4);
  CHECK(sequence.totals().detections.predictions == 3);
  CHECK(sequence.totals().detections.matched == 1);
}
} // namespace

int main() {
  using namespace autoaim;
  namespace detail = pipeline::annotation_detail;
  namespace fs = std::filesystem;

  return test::run([] {
    check_hand_calculated_metrics();
    const auto fixture = fs::path(__FILE__).parent_path().parent_path() /
                          "fixtures/evaluation-golden";
    const auto expected = detail::read_document(fixture / "expected.yaml");
    detail::OutputDirectory sandbox(fs::temp_directory_path() / "autoaim-evaluation-golden");
    const auto output = sandbox.path() / "report";
    const auto original = detail::dataset_fingerprint(fixture / "events.yaml");
    std::vector<std::string> arguments{"bench_detector", "--dataset",
        (fixture / "events.yaml").string(), "--config", (fixture / "armor.yaml").string(),
        "--iou", "0.5", "--output", output.string()};
    std::vector<char*> argv;
    for (auto& argument : arguments)
      argv.push_back(argument.data());
    CHECK(pipeline::run_detector_benchmark(int(argv.size()), argv.data()) == 0);
    const auto first = detail::read_bytes(output / "report.yaml");
    fs::rename(output, sandbox.path() / "first");
    CHECK(pipeline::run_detector_benchmark(int(argv.size()), argv.data()) == 0);
    CHECK(first == detail::read_bytes(output / "report.yaml"));
    const auto report = detail::read_document(output / "report.yaml");
    const auto run = report["runs"][0];
    for (const auto* key : {"frames", "truths", "predictions", "geometric_tp", "visible_corners"})
      CHECK(run["summary"][key].as<std::size_t>() == expected[key].as<std::size_t>());
    for (const auto* key : {"precision", "recall", "corner_rms_px"})
      CHECK(run["summary"][key].as<double>() == expected[key].as<double>());
    CHECK(run["maximum_empty_detection_streak"].as<std::size_t>() ==
          expected["maximum_empty_detection_streak"].as<std::size_t>());
    CHECK(report["pose_metrics"].as<std::string>() == "not_produced");
    CHECK(detail::same_yaml(original, detail::dataset_fingerprint(fixture / "events.yaml")));
  });
}
