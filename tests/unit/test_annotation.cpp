#include "autoaim/vision/annotation.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::vision;

  return test::run([] {
    auto node = YAML::Load(R"([
      {category: three, color: red, plate_type: small, track_id: board-1,
       corners: [[10,10],[30,10],[30,20],[10,20]], visible: [true,true,false,true]}
    ])");
    auto result = read_annotations(node, {100, 100});
    CHECK(result.size() == 1 && result[0].label.category == TargetCategory::three);
    CHECK(!result[0].visible[2] && !result[0].pose);
    auto degenerate = YAML::Clone(node);
    degenerate[0]["corners"] = YAML::Load("[[10,10],[10,10],[10,10],[10,10]]");
    CHECK_THROWS(std::invalid_argument, read_annotations(degenerate, {100, 100}));
    degenerate[0]["corners"] = YAML::Load("[[10,10],[20,20],[30,30],[40,40]]");
    CHECK_THROWS(std::invalid_argument, read_annotations(degenerate, {100, 100}));
    degenerate[0]["corners"] = YAML::Load("[[10,10],[30,10],[30,20],[30,20]]");
    CHECK_THROWS(std::invalid_argument, read_annotations(degenerate, {100, 100}));
    degenerate[0]["corners"] = YAML::Load("[[10,12],[30,5],[35,20],[15,30]]");
    CHECK(read_annotations(degenerate, {100, 100}).size() == 1);
    CHECK(read_annotations(YAML::Load("[]"), {100, 100}).empty());
    CHECK_THROWS(std::invalid_argument, read_annotations({}, {100, 100}));
    auto duplicate = YAML::Clone(node);
    duplicate.push_back(YAML::Clone(node[0]));
    CHECK_THROWS(std::invalid_argument, read_annotations(duplicate, {100, 100}));
    node[0]["corners"][0][0] = 200;
    CHECK_THROWS(std::invalid_argument, read_annotations(node, {100, 100}));
    node[0]["visible"][0] = false;
    CHECK(read_annotations(node, {100, 100}).size() == 1);
    node[0]["pose_truth"] = YAML::Load(R"({position_camera_m: [0,0,2],
      rotation_camera_wxyz: [1,0,0,0], reference_id: synthetic-projection,
      position_uncertainty_m: 0, rotation_uncertainty_rad: 0})");
    CHECK(read_annotations(node, {100, 100})[0].pose->reference_id == "synthetic-projection");
    node[0]["pose_truth"].remove("position_uncertainty_m");
    CHECK_THROWS(YAML::Exception, read_annotations(node, {100, 100}));
  });
}
