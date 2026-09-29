#pragma once

#include "autoaim/vision/pnp.hpp"
#include "autoaim/vision/frame_packet.hpp"

namespace autoaim::estimation {
struct ObservedPose {
  math::Transform<math::PlateFrame, math::WorldFrame> plate_to_world;
  std::optional<vision::PoseCovariance> covariance;
};

// 完整保留相机/世界三维候选和质量，测量模型随后选择观测分量。
// 不持有图像，像素租约可在视觉处理结束后释放；classifier id 不是物理板身份。
struct Observation {
  const core::Stamp source;
  const vision::Detection detection;
  const vision::PnpEstimate pnp;
  const std::vector<ObservedPose> world_candidates;
  const vision::AlignedPose historical_pose;
  const core::TimeOrigin time_origin;
  const core::Seconds timing_uncertainty;
  const core::Evidence timing_evidence;
  // 兼容字段：只描述 pnp.selected 的证明，不代表所有 world_candidates 都可靠。
  const bool reliable;

  Observation(core::Stamp stamp, vision::Detection pixels, vision::PnpEstimate estimate,
              std::vector<ObservedPose> candidates, vision::AlignedPose aligned,
              core::TimeOrigin origin, core::Seconds time_sigma, core::Evidence evidence);

  // 关联不是新的 PnP 消歧证明。改选候选仍可估计，但不能继承原选解的质量许可。
  bool candidate_reliable(std::size_t index) const noexcept;
};

// additional_covariance 是外参/历史姿态等在板中心处、世界系下的附加误差协方差。
// 不从配置声明制造零噪声实测事实；调用者须显式提供其评估值。
core::Result<Observation> make_observation(const vision::FramePacket& frame,
                                           const vision::Detection& detection,
                                           const vision::PnpEstimate& estimate,
                                           const vision::Calibration& calibration,
                                           const vision::PoseCovariance& additional_covariance);
} // namespace autoaim::estimation
