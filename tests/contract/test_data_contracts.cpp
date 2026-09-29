#include "autoaim/control/control_intent.hpp"
#include "autoaim/estimation/target_snapshot.hpp"
#include "autoaim/vision/frame_packet.hpp"
#include "test_support.hpp"
#include <type_traits>

int main() {
  using namespace autoaim;
  static_assert(!std::is_default_constructible_v<vision::FramePacket>);
  static_assert(!std::is_default_constructible_v<estimation::TargetSnapshot>);
  static_assert(!std::is_default_constructible_v<control::ControlIntent>);
  static_assert(!std::is_copy_assignable_v<control::ControlIntent>);
  return test::run([] {
    const core::TimePoint time(100, core::ClockDomain::replay);
    auto pixels = std::make_shared<const std::vector<std::uint8_t>>(12, 42);
    const vision::FramePacket frame(core::CapturedFrame(core::Stamp(2, 3, time),
      core::Image(2, 2, 6, pixels), time, core::TimeOrigin::synthetic,
      core::Seconds(0), core::Evidence::missing()), std::nullopt);
    pixels.reset();
    CHECK(frame.frame.image.pixels->at(0) == 42);
    CHECK(frame.frame.stamp.frame_id == 2 && frame.frame.stamp.generation == 3);
    const control::ControlIntent intent(frame.frame.stamp, core::Role::infantry, core::Task::armor,
      core::ControlMode::assist, core::CommandSpace::relative, true, true,
      {core::Radians(0), core::Radians(0), 0, 0, 0, 0}, time, core::advance(time, core::Seconds(1)),
      {frame.frame.stamp, time, true, true, true, true, true, true}, core::Evidence::declared(), core::Metres(3));
    CHECK(intent.fire_requested);  // 请求可以是错误的；最终许可必须由 guard 撤销。
    CHECK(core::same_time(intent.source.exposure, frame.frame.stamp.exposure));
    CHECK_THROWS(std::invalid_argument, control::ControlIntent(core::Stamp(99, 3, time), intent.role,
      intent.task, intent.mode, intent.space, true, true, intent.pointing, intent.created_at,
      intent.deadline, intent.facts, intent.timing_evidence, intent.horizontal_distance));
    CHECK_THROWS(std::invalid_argument, core::Image(2, 2, 1, frame.frame.image.pixels));
    CHECK_THROWS(std::invalid_argument, vision::FramePacket(frame.frame,
      vision::AlignedPose(core::advance(time, core::Seconds(0.1)), time,
        core::advance(time, core::Seconds(0.2)),
        math::Transform<math::GimbalFrame, math::WorldFrame>(math::SE3(Eigen::Quaterniond::Identity(),
          Eigen::Vector3d::Zero())), true)));
  });
}
