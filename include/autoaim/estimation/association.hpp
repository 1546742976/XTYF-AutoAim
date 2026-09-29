#pragma once

#include "autoaim/estimation/health.hpp"

namespace autoaim::estimation {
struct AssociationCandidate {
  std::size_t physical_plate;
  std::size_t pose_candidate;
  double prior_nis;
  LinearizedMeasurement measurement;
};

// filter 必须已经传播到该观测时刻；本函数不修改滤波器，也不额外预测。
// 遍历真实板数与 PnP 候选数，返回所有通过创新门限的组合，暂不擅自消歧。
std::vector<AssociationCandidate>
association_candidates(const Ekf& filter, core::TimePoint predicted_at,
                       const GeometryProfile& geometry, const Observation& observation,
                       const MeasurementModel& measurement, const NisGate& gate);

struct Assignment {
  std::size_t track;
  std::size_t observation;
  double cost;
};

// 保守的一对一匹配：只接纳相互唯一最近邻，行/列都必须满足次优间隔。
// 有歧义时宁可暂不关联；+infinity 表示无候选，NaN/负代价为接口错误。
std::vector<Assignment> mutual_nearest_assignment(const Eigen::MatrixXd& costs, double maximum_cost,
                                                  double minimum_margin);
} // namespace autoaim::estimation
