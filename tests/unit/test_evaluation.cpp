#include "autoaim/vision/evaluation.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::vision;

  return test::run([] {
    auto truth = read_annotations(YAML::Load(R"([
      {category: three, color: red, plate_type: small,
       corners: [[10,10],[30,10],[30,20],[10,20]], visible: [true,true,false,true],
       pose_truth: {position_camera_m: [0,0,2], rotation_camera_wxyz: [1,0,0,0],
         reference_id: fixture, position_uncertainty_m: 0, rotation_uncertainty_rad: 0}}
    ])"), {100, 100});
    Detection detected{truth[0].corners, 0.9f, TeamColor::red, 100, false,
                       TargetCategory::three, ArmorSize::small};
    std::vector<EvaluatedDetection> predictions{{detected, truth[0].pose->plate_to_camera},
                                                {detected, std::nullopt}};
    auto result = evaluate_frame(truth, predictions, {0.5});
    CHECK(result.truths == 1 && result.predictions == 2 && result.matched == 1);
    CHECK(result.semantic_matched == 1 && result.visible_corners == 3 && result.pose_pairs == 0);
    CHECK(result.corner_error_sum_px == 0);
    result = evaluate_frame(truth, predictions, {0.5, PoseTruthLimits{"fixture", 0, 0}});
    CHECK(result.pose_pairs == 1 && result.position_error_sum_m == 0);
    predictions[0].detection.category = TargetCategory::four;
    predictions[0].detection.color = TeamColor::blue;
    predictions[0].detection.armor_size = ArmorSize::big;
    result = evaluate_frame(truth, predictions, {0.5});
    CHECK(result.category_errors == 1 && result.color_errors == 1 && result.size_errors == 1);
    CHECK(result.semantic_matched == 0); // IoU 相同选更早的预测，不用真值标签优化匹配。
    predictions.resize(1);
    for (auto& corner : predictions[0].detection.corners)
      corner.x += 1;
    result = evaluate_frame(truth, predictions, {0.5});
    CHECK_NEAR(result.corner_error_sum_px, 3, 1e-6);
    CHECK(evaluate_frame(truth, {}, {0.5}).matched == 0);
    CHECK(evaluate_frame({}, predictions, {0.5}).predictions == 1);
    CHECK_THROWS(std::invalid_argument, evaluate_frame(truth, predictions, {0}));
    truth[0].corners[3] = truth[0].corners[2];
    CHECK_THROWS(std::invalid_argument, evaluate_frame(truth, predictions, {0.5}));
  });
}
