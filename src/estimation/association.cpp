#include "autoaim/estimation/association.hpp"
#include <algorithm>
#include <limits>

namespace autoaim::estimation {
std::vector<AssociationCandidate>
association_candidates(const Ekf& filter, core::TimePoint predicted_at,
                       const GeometryProfile& geometry, const Observation& observation,
                       const MeasurementModel& measurement, const NisGate& gate) {
  std::vector<AssociationCandidate> result;

  if (!core::same_time(predicted_at, observation.source.exposure) || !observation.pnp.pose_valid)
    return result;

  for (std::size_t board = 0; board < geometry.plates.size(); ++board) {
    for (std::size_t candidate = 0; candidate < observation.world_candidates.size(); ++candidate) {
      if (!observation.pnp.candidates[candidate].geometry_accepted)
        continue;

      auto linearized = measurement.linearize(filter.state(), geometry, board,
                                              observation.world_candidates[candidate]);

      if (!linearized)
        continue;

      const auto innovation = filter.innovation(linearized.value());

      if (!innovation || innovation.value().nis > gate.limit(measurement.dimension()))
        continue;

      result.push_back({board, candidate, innovation.value().nis, std::move(linearized).value()});
    }
  }

  std::stable_sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.prior_nis < b.prior_nis;
  });

  return result;
}

std::vector<Assignment> mutual_nearest_assignment(const Eigen::MatrixXd& costs, double maximum_cost,
                                                  double minimum_margin) {
  if (!std::isfinite(maximum_cost) || maximum_cost <= 0 || !std::isfinite(minimum_margin) ||
      minimum_margin <= 0)
    throw std::invalid_argument("Invalid association gate/margin");

  for (Eigen::Index row = 0; row < costs.rows(); ++row)
    for (Eigen::Index col = 0; col < costs.cols(); ++col)
      if (std::isnan(costs(row, col)) || costs(row, col) < 0)
        throw std::invalid_argument("Invalid association cost");

  std::vector<Assignment> result;

  for (Eigen::Index row = 0; row < costs.rows(); ++row) {
    Eigen::Index best_col = -1;
    double best = std::numeric_limits<double>::infinity(), second = best;

    for (Eigen::Index col = 0; col < costs.cols(); ++col) {
      const double value = costs(row, col);

      if (value < best) {
        second = best;
        best = value;
        best_col = col;
      } else
        second = std::min(second, value);
    }

    if (best_col < 0 || best > maximum_cost || second - best < minimum_margin)
      continue;

    double competitor = std::numeric_limits<double>::infinity();

    for (Eigen::Index other = 0; other < costs.rows(); ++other)
      if (other != row)
        competitor = std::min(competitor, costs(other, best_col));

    if (competitor - best >= minimum_margin)
      result.push_back({static_cast<std::size_t>(row), static_cast<std::size_t>(best_col), best});
  }

  return result;
}
} // namespace autoaim::estimation
