#pragma once

#include <Eigen/Core>

namespace autoaim::estimation {
// 实验初值始终可配置；仅在编译期选择 ESO 时使用，不表示设备调参或实机验收。
struct EsoOptions {
  double translation_bandwidth_radps = 10;
  double angular_bandwidth_radps = 10;
  double linear_jerk_psd = 1; // m²/s⁵
};

struct Innovation {
  Eigen::MatrixXd covariance;
  double nis;
};

struct UpdateReport {
  bool accepted;
  double prior_nis;
  int observation_dimension;
};
} // namespace autoaim::estimation
