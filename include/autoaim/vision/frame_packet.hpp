#pragma once

#include "autoaim/vision/frame.hpp"
#include "autoaim/math/transform.hpp"
#include <optional>

namespace autoaim::vision {
struct AlignedPose {
  const core::TimePoint sampled_at;
  const core::TimePoint bracket_begin;
  const core::TimePoint bracket_end;
  const math::Transform<math::GimbalFrame, math::WorldFrame> gimbal_to_world;
  const bool reliable;
  AlignedPose(core::TimePoint time, core::TimePoint first, core::TimePoint last,
              math::Transform<math::GimbalFrame, math::WorldFrame> pose, bool quality)
      : sampled_at(time), bracket_begin(first), bracket_end(last),
        gimbal_to_world(std::move(pose)), reliable(quality) {
    if (core::elapsed(time, first).value() < 0 || core::elapsed(last, time).value() < 0)
      throw std::invalid_argument("Pose interpolation outside bracket");
  }
};

struct FramePacket {
  const Frame frame;
  const std::optional<AlignedPose> pose;
  FramePacket(Frame source, std::optional<AlignedPose> aligned_pose)
      : frame(std::move(source)), pose(std::move(aligned_pose)) {
    if (pose && !core::same_time(pose->sampled_at, frame.stamp.exposure))
      throw std::invalid_argument("Pose is not aligned to exposure");
  }
};
}  // namespace autoaim::vision
