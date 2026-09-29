#include "autoaim/vision/evaluation.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::vision;

  return test::run([] {
    const auto truth = read_annotations(YAML::Load(R"([
      {category: unknown, color: red, plate_type: small, track_id: one,
       corners: [[10,10],[30,10],[30,20],[10,20]], visible: [true,true,true,true]}
    ])"), {100, 100});
    SequenceEvaluation sequence({0.5});
    sequence.add(1, truth, {}, {true, true, false}, InterventionGroup::manual);
    sequence.add(3, truth, {}, {false, false, true}, InterventionGroup::none);
    sequence.add(4, std::nullopt, {}, {true, false, false}, InterventionGroup::unknown);
    sequence.add(5, truth, {}, {true, true, false}, InterventionGroup::none);
    CHECK(sequence.maximum_empty_detection_streak() == 2);
    CHECK(sequence.maximum_track_loss_streak() == 2);
    const auto& total = sequence.totals();
    CHECK(total.frames == 4 && total.detections.truths == 3 && total.unlabeled_frames == 1);
    CHECK(total.expired_frames == 1 && total.rejected_or_dropped_frames == 2);
    CHECK(total.frames_with_misses == 3 && total.negative_frames == 0);
    CHECK(sequence.by_intervention()[std::size_t(InterventionGroup::manual)].frames == 1);
    Detection detection{truth[0].corners, 1, TeamColor::red, 0, false,
                         TargetCategory::unknown, ArmorSize::small};
    sequence.add(6, truth, {{detection, std::nullopt}}, {true, false, true},
                 InterventionGroup::none);
    CHECK(sequence.totals().detections.matched == 1 && sequence.totals().usable_matches == 0);
    sequence.add(7, std::vector<ArmorAnnotation>{}, {}, {true, true, false},
                 InterventionGroup::none);
    CHECK(sequence.totals().negative_frames == 1);
    CHECK_THROWS(std::invalid_argument,
        sequence.add(7, truth, {}, {true, true, false}, InterventionGroup::none));
  });
}
