#pragma once

#include "autoaim/core/evidence.hpp"
#include "autoaim/core/types.hpp"

namespace autoaim::control {
struct Pointing {
  core::Radians yaw;
  core::Radians pitch;
  double yaw_rate_radps;
  double pitch_rate_radps;
  double yaw_acceleration_radps2;
  double pitch_acceleration_radps2;
};

// 历史事实不替代消费时的新鲜度/反馈检查；这里没有最终 fire_allowed。
struct AimFacts {
  core::Stamp source;
  core::TimePoint historical_pose_time;
  bool historical_pose_valid;
  bool pose_reliable;
  bool identity_known;
  bool uncertainty_acceptable;
  bool ballistic_solvable;
  bool settled;
};

struct ControlIntent {
  const core::Stamp source;
  const core::Role role;
  const core::Task task;
  const core::ControlMode mode;
  const core::CommandSpace space;
  const bool control_requested;
  const bool fire_requested;
  const Pointing pointing;
  const core::TimePoint created_at;
  const core::TimePoint deadline;
  const AimFacts facts;
  const core::Evidence timing_evidence;
  const core::Metres horizontal_distance; // 瞄准参考系水平距离；不得在发布端丢失为零。

  ControlIntent(core::Stamp stamp, core::Role robot_role, core::Task robot_task,
                core::ControlMode control_mode, core::CommandSpace command_space, bool control,
                bool fire, Pointing angles, core::TimePoint created, core::TimePoint expiry,
                AimFacts aim, core::Evidence timing, core::Metres distance)
      : source(stamp), role(robot_role), task(robot_task), mode(control_mode), space(command_space),
        control_requested(control), fire_requested(fire), pointing(std::move(angles)),
        created_at(created), deadline(expiry), facts(aim), timing_evidence(std::move(timing)),
        horizontal_distance(distance) {
    if (distance.value() < 0)
      throw std::invalid_argument("Negative aim distance");

    if (core::elapsed(created, stamp.exposure).value() < 0 ||
        core::elapsed(expiry, created).value() <= 0)
      throw std::invalid_argument("Invalid intent time interval");

    if (facts.source.frame_id != source.frame_id || facts.source.generation != source.generation ||
        !core::same_time(facts.source.exposure, source.exposure) ||
        (facts.historical_pose_valid &&
         !core::same_time(facts.historical_pose_time, source.exposure)))
      throw std::invalid_argument("Aim evidence belongs to a different observation");

    // 非有限前馈仍交 guard 统一拒绝，不能在异常路径中逃过其故障记录。
  }
};
} // namespace autoaim::control
