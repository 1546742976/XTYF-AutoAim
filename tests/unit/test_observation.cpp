#include "autoaim/estimation/observation.hpp"
#include "support/vision_fixture.hpp"
#include "test_support.hpp"
#include <algorithm>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto calibration = test::synthetic_calibration();
    const core::TimePoint time(1, core::ClockDomain::replay);
    const core::Stamp source(1, 1, time);
    const vision::PlateDimensions size(core::Metres(0.135), core::Metres(0.055));
    const math::SE3 truth(Eigen::Quaterniond(Eigen::AngleAxisd(2.5, Eigen::Vector3d::UnitX())),
                          {0.1, 0.1, 1});

    const auto detection = test::project_plate(truth, size, calibration);
    const auto estimate = vision::solve_pose(source, detection, size, calibration,
                                             {3, 5, 2, 0.1, 0.001, 0.5, 1e-10, core::Metres(0.1),
                                              core::Radians(0.1), core::Seconds(0.1)},
                                             "synthetic-camera", "v1", std::nullopt, std::nullopt);

    auto bytes = std::make_shared<const std::vector<std::uint8_t>>(1280 * 1024 * 3, 0);
    const core::CapturedFrame raw(source, core::Image(1280, 1024, 1280 * 3, bytes), time,
                                  core::TimeOrigin::synthetic, core::Seconds(0),
                                  core::Evidence::missing());

    const vision::AlignedPose pose(
        time, time, time,
        math::Transform<math::GimbalFrame, math::WorldFrame>(math::SE3(
            Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())), {1, 2, 3})),
        true);

    const vision::FramePacket frame(raw, pose);
    const auto observation = estimation::make_observation(
        frame, detection, estimate, calibration, vision::PoseCovariance::Identity() * 0.001);

    CHECK(observation);
    CHECK(observation.value().candidate_reliable(*estimate.selected));
    CHECK(!observation.value().candidate_reliable(estimate.candidates.size()));
    for (std::size_t i = 0; i < estimate.candidates.size(); ++i)
      if (i != *estimate.selected)
        CHECK(!observation.value().candidate_reliable(i));

    // 重排候选并同步索引/世界坐标，证明必须跟随候选，不能依赖数组首项。
    auto reordered = estimate;
    auto world = observation.value().world_candidates;
    std::reverse(reordered.candidates.begin(), reordered.candidates.end());
    std::reverse(world.begin(), world.end());
    reordered.selected = reordered.candidates.size() - 1 - *estimate.selected;
    const estimation::Observation permuted(source, detection, reordered, world, pose,
        raw.time_origin, raw.timing_uncertainty, raw.timing_evidence);
    CHECK(permuted.candidate_reliable(*reordered.selected));

    CHECK(observation.value().world_candidates.size() == estimate.candidates.size());
    const auto& selected = observation.value().world_candidates[*estimate.selected];
    const auto expected = pose.gimbal_to_world.value().apply(truth.translation());
    CHECK((selected.plate_to_world.value().translation() - expected).norm() < 1e-4);
    CHECK(selected.covariance->diagonal().minCoeff() >= 0.001);
    CHECK(!estimation::make_observation(vision::FramePacket(raw, std::nullopt), detection, estimate,
                                        calibration, vision::PoseCovariance::Zero()));

    auto wrong = vision::PnpEstimate(core::Stamp(2, 1, time), size);
    CHECK(!estimation::make_observation(frame, detection, wrong, calibration,
                                        vision::PoseCovariance::Zero()));
  });
}
