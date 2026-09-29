#pragma once

#include "autoaim/estimation/measurement_model.hpp"

namespace autoaim::estimation {
struct PhysicalHypothesis {
  std::size_t first_plate;
  TargetState state;
  double rotation_residual_rad;
};

// 枚举每个合法首板编号。单帧只能形成有限假设，不能断言首板就是 profile[0]。
std::vector<PhysicalHypothesis> initial_identity_hypotheses(const ObservedPose& observation,
                                                            const GeometryProfile& profile);

struct IdentityLimits {
  std::size_t minimum_observations;
  double minimum_score_margin;
  double maximum_cost_per_observation;
  double previous_score_weight;
};

class IdentityResolver {
public:
  IdentityResolver(std::size_t hypotheses, IdentityLimits limits);

  // 每个新曝光最多累积一次；scores 必须来自对应假设的更新前创新/几何残差。
  // 拒绝旧世代、旧帧和同帧重复；新目标/模式切换应构造新的 resolver。
  bool observe(const core::Stamp& source, const std::vector<double>& costs);
  std::optional<std::size_t> selected() const noexcept;

  const std::vector<double>& scores() const noexcept {
    return scores_;
  }

  std::size_t support() const noexcept {
    return support_;
  }

private:
  IdentityLimits limits_;
  std::vector<double> scores_;
  std::size_t support_ = 0;
  std::optional<std::size_t> winner_;
  std::optional<core::Stamp> last_source_;
};
} // namespace autoaim::estimation
