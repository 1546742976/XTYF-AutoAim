#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    for (bool tilted : {false, true})
      for (const auto layout : {estimation::HeightLayout::same, estimation::HeightLayout::paired,
                                estimation::HeightLayout::distinct}) {
        const auto profile = test::geometry(4, layout, tilted);
        const math::Point3<math::WorldFrame> center({1, 2, 3});

        for (std::size_t i = 0; i < 4; ++i) {
          const auto pose = profile.plate_pose(center, core::Radians(0.7), i);
          const auto local =
              profile.axis_to_world.conjugate() * (pose.value().translation() - center.metres());

          CHECK_NEAR(local.z(), profile.plates[i].offset_axis_m.z(), 1e-12);
          CHECK_NEAR(profile.normal_world(core::Radians(0.7), i).norm(), 1, 1e-12);
        }
      }

    for (std::size_t count : {2, 3, 4}) {
      const auto profile = test::geometry(count, estimation::HeightLayout::same, true);
      CHECK(profile.plates.size() == count);
      CHECK_THROWS(std::out_of_range, profile.normal_world(core::Radians(0), count));
    }

    CHECK_THROWS(std::invalid_argument, test::geometry(3, estimation::HeightLayout::paired, false));
    CHECK_THROWS(std::invalid_argument, test::geometry(1, estimation::HeightLayout::same, false));
    const auto tilted = test::geometry(4, estimation::HeightLayout::same, true);
    const math::Point3<math::WorldFrame> zero(Eigen::Vector3d::Zero());
    CHECK(std::abs(tilted.plate_pose(zero, core::Radians(0), 0).value().translation().z() -
                   tilted.plate_pose(zero, core::Radians(math::pi), 0).value().translation().z()) >
          0.1);
  });
}
