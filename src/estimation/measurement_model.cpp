#include "autoaim/estimation/measurement_model.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::estimation {
namespace {
Eigen::VectorXd residual(const math::SE3& observation, const math::SE3& prediction, int dimension) {
  Eigen::VectorXd result(dimension);
  result.head<3>() = observation.translation() - prediction.translation();

  if (dimension == 6)
    result.tail<3>() =
        math::rotation_log(observation.rotation() * prediction.rotation().conjugate());

  return result;
}
} // namespace

MeasurementModel::MeasurementModel(MeasurementKind kind) : kind_(kind) {
  if (kind != MeasurementKind::position && kind != MeasurementKind::full_pose)
    throw std::invalid_argument("Unknown measurement model");
}

int MeasurementModel::dimension() const noexcept {
  return kind_ == MeasurementKind::position ? 3 : 6;
}

math::Transform<math::PlateFrame, math::WorldFrame>
MeasurementModel::predict(const TargetState& state, const GeometryProfile& geometry,
                          std::size_t physical_plate) const {
  if (!state_valid(state))
    throw std::invalid_argument("Invalid state for measurement prediction");

  return geometry.plate_pose(state.center, state.phase, physical_plate);
}

core::Result<LinearizedMeasurement>
MeasurementModel::linearize(const TargetState& state, const GeometryProfile& geometry,
                            std::size_t physical_plate, const ObservedPose& observation) const {
  using Result = core::Result<LinearizedMeasurement>;

  if (!state_valid(state) || physical_plate >= geometry.plates.size() || !observation.covariance ||
      !math::covariance_valid(*observation.covariance))
    return Result::failure(core::ErrorCode::invalid_input,
                           "Missing pose covariance or invalid measurement hypothesis");

  const int size = dimension();
  const auto nominal = predict(state, geometry, physical_plate).value();
  const auto& observed = observation.plate_to_world.value();
  LinearizedMeasurement result{residual(observed, nominal, size),
                               Eigen::MatrixXd::Zero(size, state_dimension),
                               Eigen::MatrixXd::Zero(size, size)};

  constexpr double step = 1e-5;

  // 对流形残差直接线性化，包含非零创新处的 log 映射导数；EKF 不知道几何细节。
  // 新增平移加速度不影响当前板位姿，其三列解析为零；保留原九列的线性化口径。
  for (int k = 0; k < component_index(StateComponent::ax); ++k) {
    StateVector delta = StateVector::Zero();
    delta[k] = step;
    const auto plus = predict(add_state_delta(state, delta), geometry, physical_plate).value();
    const auto minus = predict(add_state_delta(state, -delta), geometry, physical_plate).value();
    result.jacobian.col(k) =
        (residual(observed, minus, size) - residual(observed, plus, size)) / (2 * step);
  }

  Eigen::MatrixXd noise_transform(size, 6);

  for (int k = 0; k < 6; ++k) {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero(), rotation = translation;

    if (k < 3)
      translation[k] = step;
    else
      rotation[k - 3] = step;

    const math::SE3 plus(math::rotation_exp(rotation) * observed.rotation(),
                         observed.translation() + translation);

    const math::SE3 minus(math::rotation_exp(-rotation) * observed.rotation(),
                          observed.translation() - translation);

    noise_transform.col(k) =
        (residual(plus, nominal, size) - residual(minus, nominal, size)) / (2 * step);
  }

  result.noise = noise_transform * *observation.covariance * noise_transform.transpose();
  result.noise = ((result.noise + result.noise.transpose()) / 2).eval();

  if (!result.residual.allFinite() || !result.jacobian.allFinite() ||
      !math::covariance_valid(result.noise))
    return Result::failure(core::ErrorCode::invalid_input, "Invalid measurement linearization");

  return Result::success(std::move(result));
}
} // namespace autoaim::estimation
