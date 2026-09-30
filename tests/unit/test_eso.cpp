// CMake registers this test only when AUTOAIM_USE_ESO is enabled.
#include "autoaim/estimation/state_estimator.hpp"
#include "autoaim/decision/predictor.hpp"
#include "autoaim/math/angle.hpp"
#include "autoaim/math/numeric.hpp"
#include "support/snapshot_fixture.hpp"
#include "test_support.hpp"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <array>
#include <limits>
#include <type_traits>
#include <utility>

namespace {
using namespace autoaim;
using namespace autoaim::estimation;
constexpr double gate = 1e12;
constexpr int phase = component_index(StateComponent::phase);
constexpr int omega = component_index(StateComponent::omega);
constexpr int alpha = component_index(StateComponent::alpha);
constexpr int acceleration = component_index(StateComponent::ax);
using Injection = Eigen::Matrix<double, state_dimension, 4>;
using Selector = Eigen::Matrix<double, 4, state_dimension>;

static_assert(state_dimension == 12);
static_assert(std::is_same_v<StateEstimator, Eso>);
static_assert(std::is_copy_constructible_v<Eso> && std::is_copy_assignable_v<Eso>);
static_assert(std::is_same_v<decltype(std::declval<const Eso&>().state()),
                             decltype(std::declval<const Ekf&>().state())>);
static_assert(std::is_same_v<decltype(std::declval<const Eso&>().covariance()),
                             decltype(std::declval<const Ekf&>().covariance())>);
static_assert(std::is_same_v<decltype(std::declval<const Eso&>().motion()),
                             decltype(std::declval<const Ekf&>().motion())>);
static_assert(noexcept(std::declval<const Eso&>().state()));
static_assert(noexcept(std::declval<const Eso&>().covariance()));
static_assert(noexcept(std::declval<const Eso&>().motion()));
static_assert(std::is_same_v<decltype(std::declval<Eso&>().predict(core::Seconds(0))),
                             decltype(std::declval<Ekf&>().predict(core::Seconds(0)))>);
static_assert(std::is_same_v<decltype(std::declval<const Eso&>().innovation(
                                 std::declval<const LinearizedMeasurement&>())),
                             decltype(std::declval<const Ekf&>().innovation(
                                 std::declval<const LinearizedMeasurement&>()))>);
static_assert(std::is_same_v<decltype(std::declval<Eso&>().update(
                                 std::declval<const LinearizedMeasurement&>(), 1.0)),
                             decltype(std::declval<Ekf&>().update(
                                 std::declval<const LinearizedMeasurement&>(), 1.0))>);
static_assert(std::is_same_v<decltype(std::declval<Eso&>().change_motion(
                                 std::declval<std::shared_ptr<const MotionModel>>(), 1.0)),
                             decltype(std::declval<Ekf&>().change_motion(
                                 std::declval<std::shared_ptr<const MotionModel>>(), 1.0))>);

TargetState zero_state() {
  return {math::Point3<math::WorldFrame>({0, 0, 0}), Eigen::Vector3d::Zero(),
          core::Radians(0), 0, 0};
}

std::shared_ptr<const MotionModel> model(bool angular_ca = true) {
  if (angular_ca)
    return std::make_shared<const MotionModel>(0.01, 0.02, core::Seconds(1), 5);
  return std::make_shared<const MotionModel>(0.01, 0.02, core::Seconds(1));
}

Selector selector() {
  Selector result = Selector::Zero();
  result.block<3, 3>(0, 0).setIdentity();
  result(3, phase) = 1;
  return result;
}

// Independent reference for the current-observer (measurement after propagation) poles.
// The polynomial check below does not simply compare two copies of these formulas.
Eigen::Vector3d third_order(double bandwidth, double dt) {
  const double p = std::exp(-bandwidth * dt);
  const double q = -std::expm1(-bandwidth * dt);
  return {q * (1 + p + p * p), 1.5 * q * q * (1 + p) / dt, q * q * q / (dt * dt)};
}

Injection injection(double dt, bool angular_ca, const EsoOptions& options = {}) {
  Injection result = Injection::Zero();
  if (dt == 0) {
    result.block<3, 3>(0, 0).setIdentity();
    result(phase, 3) = 1;
    return result;
  }
  const Eigen::Vector3d translation = third_order(options.translation_bandwidth_radps, dt);
  for (int axis = 0; axis < 3; ++axis) {
    result(axis, axis) = translation[0];
    result(axis + 3, axis) = translation[1];
    result(axis + acceleration, axis) = translation[2];
  }
  if (angular_ca) {
    const Eigen::Vector3d rotation = third_order(options.angular_bandwidth_radps, dt);
    result(phase, 3) = rotation[0];
    result(omega, 3) = rotation[1];
    result(alpha, 3) = rotation[2];
  } else {
    const double p = std::exp(-options.angular_bandwidth_radps * dt);
    const double q = -std::expm1(-options.angular_bandwidth_radps * dt);
    result(phase, 3) = q * (1 + p);
    result(omega, 3) = q * q / dt;
  }
  return result;
}

LinearizedMeasurement measurement(const Eigen::Vector4d& correction, int plate = 0) {
  // Six-dimensional full-pose geometry structure: centre translation and phase only.
  Eigen::Matrix<double, 6, 4> geometry = Eigen::Matrix<double, 6, 4>::Zero();
  geometry.topLeftCorner<3, 3>().setIdentity();
  geometry(0, 3) = 0.13 * (plate + 1);
  geometry(1, 3) = -0.08 * (plate + 1);
  geometry(5, 3) = 1;
  Eigen::Matrix<double, 6, 6> noise_factor = Eigen::Matrix<double, 6, 6>::Identity();
  noise_factor(1, 0) = 0.15;
  noise_factor(5, 2) = -0.1;
  return {geometry * correction, geometry * selector(),
          (0.03 + 0.02 * plate) * noise_factor * noise_factor.transpose()};
}

LinearizedMeasurement at_current(const LinearizedMeasurement& at_base,
                                 const Eso& filter, const TargetState& base) {
  auto result = at_base;
  result.residual -= result.jacobian * state_difference(filter.state(), base);
  return result;
}

void same(const Eso& a, const Eso& b, double tolerance = 0) {
  CHECK(state_difference(a.state(), b.state()).norm() <= tolerance);
  CHECK((a.covariance() - b.covariance()).norm() <= tolerance);
  CHECK(a.motion() == b.motion());
}

void accept(Eso& filter, const LinearizedMeasurement& observation) {
  const auto prior = filter.innovation(observation);
  CHECK(prior);
  const auto report = filter.update(observation, gate);
  CHECK(report && report.value().accepted && report.value().observation_dimension == 6);
  CHECK_NEAR(report.value().prior_nis, prior.value().nis, 1e-10);
  CHECK(math::covariance_valid(filter.covariance()));
}

void check_poles_and_interface() {
  for (bool angular_ca : {false, true}) {
    for (double dt : {1e-8, 0.002, 0.03, 0.2}) {
      EsoOptions options;
      options.translation_bandwidth_radps = 7;
      options.angular_bandwidth_radps = 11;
      options.linear_jerk_psd = 0.4;
      const auto motion = model(angular_ca);
      auto filter = make_state_estimator(zero_state(), StateCovariance::Identity(), motion, options);
      CHECK(filter.motion() != motion && filter.motion()->linear_acceleration_enabled());
      CHECK(!motion->linear_acceleration_enabled());
      const auto expected_prediction = motion->with_linear_acceleration(options.linear_jerk_psd)
                                           .propagate(filter.state(), filter.covariance(),
                                                      core::Seconds(dt));
      CHECK(expected_prediction);
      CHECK(filter.predict(core::Seconds(dt)));
      CHECK((filter.covariance() - expected_prediction.value().covariance).norm() < 1e-12);
      const auto base = filter.state();
      // Four unit residual experiments recover the actual gain through the public API.
      Injection observed;
      for (int column = 0; column < 4; ++column) {
        auto copy = filter;
        Eigen::Vector4d residual = Eigen::Vector4d::Zero();
        residual[column] = 0.01;
        accept(copy, measurement(residual));
        observed.col(column) = state_difference(copy.state(), base) / 0.01;
      }
      CHECK((observed - injection(dt, angular_ca, options)).norm() < 1e-10);
      Eigen::Matrix3d transition;
      transition << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1;
      for (int column : {0, 3}) {
        if (column == 3 && !angular_ca)
          continue;
        const Eigen::Vector3d gain = column == 0
            ? Eigen::Vector3d(observed(0, 0), observed(3, 0), observed(acceleration, 0))
            : Eigen::Vector3d(observed(phase, 3), observed(omega, 3), observed(alpha, 3));
        const Eigen::Matrix3d closed =
            (Eigen::Matrix3d::Identity() - gain * Eigen::RowVector3d(1, 0, 0)) * transition;
        const double bandwidth = column == 0 ? options.translation_bandwidth_radps
                                             : options.angular_bandwidth_radps;
        const double pole = std::exp(-bandwidth * dt);
        // All three characteristic coefficients must equal (lambda-p)^3.
        CHECK_NEAR(closed.trace(), 3 * pole, 1e-11);
        CHECK_NEAR((closed.trace() * closed.trace() - (closed * closed).trace()) / 2,
                   3 * pole * pole, 1e-11);
        CHECK_NEAR(closed.determinant(), pole * pole * pole, 1e-11);
      }
      if (!angular_ca) {
        Eigen::Matrix2d transition2;
        transition2 << 1, dt, 0, 1;
        const Eigen::Vector2d gain(observed(phase, 3), observed(omega, 3));
        const Eigen::Matrix2d closed =
            (Eigen::Matrix2d::Identity() - gain * Eigen::RowVector2d(1, 0)) * transition2;
        const double pole = std::exp(-options.angular_bandwidth_radps * dt);
        CHECK_NEAR(closed.trace(), 2 * pole, 1e-11);
        CHECK_NEAR(closed.determinant(), pole * pole, 1e-11);
        CHECK(filter.covariance().row(alpha).isZero(0));
        CHECK(filter.covariance().col(alpha).isZero(0));
      }
      const auto untouched = filter;
      auto assigned = filter;
      assigned = untouched;
      accept(assigned, measurement(Eigen::Vector4d(0.1, 0.2, 0.3, 0.04)));
      same(filter, untouched);
    }
  }
}

void check_initialization_and_batch() {
  Eso initialized(zero_state(), StateCovariance::Identity(), model());
  const auto first = measurement(Eigen::Vector4d(0.2, -0.3, 0.1, 0.05));
  accept(initialized, first);
  CHECK((initialized.state().center.metres() - Eigen::Vector3d(0.2, -0.3, 0.1)).norm() < 1e-12);
  CHECK(initialized.state().velocity_mps.isZero(0));
  CHECK(initialized.state().acceleration_mps2.isZero(0));
  CHECK(initialized.state().omega_radps == 0 && initialized.state().alpha_radps2 == 0);
  CHECK_NEAR(initialized.state().phase.value(), 0.05, 1e-12);
  const auto saved = initialized;
  CHECK(initialized.predict(core::Seconds(0)));
  same(initialized, saved);

  // A non-diagonal prior checks centre/derivative and inter-axis cross covariance too.
  StateCovariance factor = StateCovariance::Identity();
  for (int row = 1; row < state_dimension; ++row)
    factor(row, row - 1) = 0.1 * (row + 1);
  Eso filter(zero_state(), factor * factor.transpose(), model());
  const double dt = 0.04;
  CHECK(filter.predict(core::Seconds(dt)));
  const auto base = filter.state();
  const StateCovariance prior = filter.covariance();
  const Injection gain = injection(dt, true);
  Eigen::MatrixXd joint_h(0, state_dimension), joint_r(0, 0);
  Eigen::VectorXd joint_residual(0);
  Eigen::Matrix4d information = Eigen::Matrix4d::Zero();
  Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
  for (int plate = 0; plate < 4; ++plate) {
    const Eigen::Vector4d delta(0.12 + 0.01 * plate, -0.03 * plate, 0.04,
                               0.025 + 0.001 * plate);
    const auto raw = measurement(delta, plate);
    const auto current = at_current(raw, filter, base);
    const Eigen::MatrixXd expected_s = raw.jacobian * prior * raw.jacobian.transpose() + raw.noise;
    const auto nis = filter.innovation(current);
    CHECK(nis);
    CHECK((nis.value().covariance - expected_s).norm() < 1e-10);
    CHECK_NEAR(nis.value().nis,
               raw.residual.dot(expected_s.ldlt().solve(raw.residual)), 1e-10);
    accept(filter, current);
    const Eigen::MatrixXd geometry = raw.jacobian * selector().transpose();
    information += geometry.transpose() * raw.noise.ldlt().solve(geometry);
    rhs += geometry.transpose() * raw.noise.ldlt().solve(raw.residual);
    const auto expected_state = add_state_delta(base, gain * information.ldlt().solve(rhs));
    CHECK(state_difference(filter.state(), expected_state).norm() < 1e-10);

    const int rows = 6 * (plate + 1);
    joint_h.conservativeResize(rows, state_dimension);
    joint_h.bottomRows(6) = raw.jacobian;
    Eigen::MatrixXd enlarged = Eigen::MatrixXd::Zero(rows, rows);
    if (plate > 0)
      enlarged.topLeftCorner(rows - 6, rows - 6) = joint_r;
    enlarged.bottomRightCorner(6, 6) = raw.noise;
    joint_r = std::move(enlarged);
    joint_residual.conservativeResize(rows);
    joint_residual.tail(6) = raw.residual;
    const Eigen::MatrixXd joint_geometry = joint_h * selector().transpose();
    // Direct stacked least squares and Joseph update, independent of accumulator layout.
    const Eigen::MatrixXd weight = information.ldlt().solve(
        joint_geometry.transpose() * joint_r.inverse());
    const Eigen::MatrixXd effective_gain = gain * weight;
    const StateCovariance remainder = StateCovariance::Identity() - effective_gain * joint_h;
    const StateCovariance expected_p = remainder * prior * remainder.transpose() +
                                      effective_gain * joint_r * effective_gain.transpose();
    CHECK((filter.covariance() - expected_p).norm() < 1e-9);
    CHECK(state_difference(filter.state(), add_state_delta(base, effective_gain * joint_residual))
              .norm() < 1e-10);
    const auto before_zero = filter;
    CHECK(filter.predict(core::Seconds(0)));
    same(filter, before_zero);
  }
}

void check_trajectory_and_wrap() {
  for (bool angular_ca : {false, true}) {
    for (bool translation_ca : {false, true}) {
      Eso filter(zero_state(), StateCovariance::Identity(), model(angular_ca));
      double time = 0;
      TargetState truth = zero_state();
      const Eigen::Vector3d velocity(0.8, -0.3, 0.15);
      const Eigen::Vector3d acc = translation_ca ? Eigen::Vector3d(0.2, 0.1, -0.04)
                                                : Eigen::Vector3d::Zero();
      for (int sample = 0; sample < 900; ++sample) {
        // Actual elapsed time, including dropped exposures, drives prediction and gains.
        const double dt = sample % 53 == 0 ? 0.09 : (sample % 2 == 0 ? 0.008 : 0.017);
        time += dt;
        CHECK(filter.predict(core::Seconds(dt)));
        truth.center = math::Point3<math::WorldFrame>(velocity * time + 0.5 * acc * time * time);
        truth.velocity_mps = velocity + acc * time;
        truth.acceleration_mps2 = acc;
        truth.phase = math::wrap_to_pi(core::Radians(3.0 + 0.5 * time +
                                                    (angular_ca ? 0.025 * time * time : 0)));
        truth.omega_radps = 0.5 + (angular_ca ? 0.05 * time : 0);
        truth.alpha_radps2 = angular_ca ? 0.05 : 0;
        // Start at phase 3 rad: the first correction uses the proper wrapped innovation.
        const Eigen::Vector4d correction = selector() * state_difference(truth, filter.state());
        accept(filter, measurement(correction, sample % 4));
      }
      // Noiseless polynomial input belongs exactly to the chosen observer model. After
      // > 100 bandwidth time constants, these tolerances only accommodate round-off.
      CHECK((filter.state().center.metres() - truth.center.metres()).norm() < 1e-7);
      CHECK((filter.state().velocity_mps - truth.velocity_mps).norm() < 1e-6);
      CHECK((filter.state().acceleration_mps2 - truth.acceleration_mps2).norm() < 1e-5);
      CHECK(std::abs(state_difference(filter.state(), truth)[phase]) < 1e-7);
      CHECK_NEAR(filter.state().omega_radps, truth.omega_radps, 1e-6);
      CHECK_NEAR(filter.state().alpha_radps2, truth.alpha_radps2, 1e-5);
    }
  }

  TargetState state = zero_state();
  state.phase = core::Radians(3.13);
  Eso crossing(state, StateCovariance::Identity(), model(false));
  CHECK(crossing.predict(core::Seconds(0.02)));
  const auto raw = measurement(Eigen::Vector4d(0, 0, 0, 0.04));
  accept(crossing, raw);
  const auto first = crossing.state();
  accept(crossing, at_current(raw, crossing, state));
  CHECK(state_difference(crossing.state(), first).norm() < 1e-10);
  CHECK(crossing.state().phase.value() < 0);
}

void check_rejections_and_elapsed() {
  Eso filter(zero_state(), StateCovariance::Identity(), model());
  CHECK(filter.predict(core::Seconds(0.03)));
  const auto base = filter.state();
  accept(filter, measurement(Eigen::Vector4d(0.03, 0.01, 0, 0.003)));
  const auto control = filter;
  auto huge = at_current(measurement(Eigen::Vector4d(100, 0, 0, 0.02)), filter, base);
  const auto innovation = filter.innovation(huge);
  CHECK(innovation);
  const auto rejected = filter.update(huge, 1);
  CHECK(rejected && !rejected.value().accepted);
  CHECK_NEAR(rejected.value().prior_nis, innovation.value().nis, 1e-10);
  same(filter, control);
  for (int invalid = 0; invalid < 10; ++invalid) {
    auto bad = measurement(Eigen::Vector4d(0.1, 0, 0, 0.01));
    if (invalid == 0 || invalid == 1) {
      const int dimension = invalid == 0 ? 1 : 3;
      bad.residual.conservativeResize(dimension);
      bad.jacobian.conservativeResize(dimension, state_dimension);
      bad.noise = Eigen::MatrixXd::Identity(dimension, dimension);
    } else if (invalid == 2) {
      bad.noise(0, 0) = 0; // Indefinite with the preserved nonzero correlation.
    } else if (invalid == 3) {
      bad.noise.setZero();
    } else if (invalid == 4) {
      bad.jacobian.col(phase).setZero();
    } else if (invalid == 5) {
      bad.jacobian(0, 3) = 1;
    } else if (invalid == 6) {
      bad.jacobian(0, acceleration) = 1;
    } else if (invalid == 7) {
      bad.residual[0] = std::numeric_limits<double>::quiet_NaN();
    } else if (invalid == 8) {
      bad.noise(0, 0) = std::numeric_limits<double>::infinity();
    } else {
      bad.jacobian.conservativeResize(6, state_dimension - 1);
    }
    CHECK(!filter.innovation(bad));
    CHECK(!filter.update(bad, gate));
    same(filter, control);
  }
  CHECK(!filter.predict(core::Seconds(-0.01)));
  CHECK(!filter.predict(core::Seconds(1.01)));
  CHECK(!filter.update(measurement(Eigen::Vector4d::Zero()), 0));
  CHECK(!filter.update(measurement(Eigen::Vector4d::Zero()),
                       std::numeric_limits<double>::quiet_NaN()));
  CHECK(!filter.change_motion(nullptr, 1));
  CHECK(!filter.change_motion(model(false), -1));
  same(filter, control);
  auto reference = control;
  const auto next = measurement(Eigen::Vector4d(-0.01, 0.03, 0.02, 0.004), 2);
  accept(filter, at_current(next, filter, base));
  accept(reference, at_current(next, reference, base));
  same(filter, reference, 1e-12); // Failed inputs also preserved accepted information.
  CHECK(filter.predict(core::Seconds(0.017)));
  CHECK(reference.predict(core::Seconds(0.017)));
  accept(filter, measurement(Eigen::Vector4d(0.02, 0, 0, 0.001)));
  accept(reference, measurement(Eigen::Vector4d(0.02, 0, 0, 0.001)));
  same(filter, reference, 1e-12); // Failure did not consume elapsed correction time.

  // With no effective correction, positive predicts accumulate their actual intervals.
  Eso elapsed(zero_state(), StateCovariance::Identity(), model());
  CHECK(elapsed.predict(core::Seconds(0.03)));
  CHECK(!elapsed.update(measurement(Eigen::Vector4d(10, 0, 0, 0)), 0));
  CHECK(elapsed.predict(core::Seconds(0.07)));
  const auto prior_state = elapsed.state();
  const Eigen::Vector4d residual(0.01, -0.02, 0.03, 0.004);
  accept(elapsed, measurement(residual));
  CHECK((state_difference(elapsed.state(), prior_state) - injection(0.1, true) * residual)
            .norm() < 1e-10);
}

void check_motion_and_future() {
  TargetState initial = zero_state();
  initial.center = math::Point3<math::WorldFrame>({3, 0, 1});
  initial.velocity_mps = {0.5, -0.1, 0.2};
  initial.acceleration_mps2 = {0.2, 0.3, -0.1};
  initial.omega_radps = 0.4;
  initial.alpha_radps2 = 0.2;
  StateCovariance factor = StateCovariance::Identity();
  factor(9, 0) = 0.3;
  factor(7, 10) = -0.2;
  factor(4, 11) = 0.15;
  Eso filter(initial, factor * factor.transpose(), model());
  const double dt = 0.13;
  StateCovariance transition = StateCovariance::Identity();
  StateCovariance noise = StateCovariance::Zero();
  const std::array<std::array<int, 3>, 4> axes{{{{0, 3, 9}}, {{1, 4, 10}},
                                               {{2, 5, 11}}, {{phase, omega, alpha}}}};
  for (int axis = 0; axis < 4; ++axis) {
    const auto& index = axes[axis];
    transition(index[0], index[1]) = dt;
    transition(index[0], index[2]) = dt * dt / 2;
    transition(index[1], index[2]) = dt;
    Eigen::Matrix3d block;
    block << std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6,
             std::pow(dt, 4) / 8, std::pow(dt, 3) / 3, dt * dt / 2,
             std::pow(dt, 3) / 6, dt * dt / 2, dt;
    block *= axis < 3 ? 1.0 : 0.02;
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        noise(index[row], index[column]) = block(row, column);
  }
  const StateCovariance expected = transition * filter.covariance() * transition.transpose() + noise;
  const auto fixture = test::snapshot();
  const TargetSnapshot snapshot(
      fixture->source, fixture->state_time, fixture->target_id, filter.state(), filter.covariance(),
      fixture->physical_plate, fixture->quality, fixture->pose_reliable, fixture->historical_pose,
      filter.motion(), fixture->geometry, fixture->visible_plate, fixture->visible_dimensions,
      fixture->time_origin, fixture->timing_uncertainty, fixture->timing_evidence);
  const auto future = decision::predict_future(
      snapshot, core::advance(snapshot.state_time, core::Seconds(dt)), {0, 0, 1});
  CHECK(future);
  CHECK((future.value().covariance - expected).norm() < 1e-11);
  CHECK((future.value().state.center.metres() - initial.center.metres() -
         initial.velocity_mps * dt - 0.5 * initial.acceleration_mps2 * dt * dt).norm() < 1e-12);
  CHECK((future.value().state.velocity_mps - initial.velocity_mps -
         initial.acceleration_mps2 * dt).norm() < 1e-12);
  CHECK(filter.predict(core::Seconds(dt)));
  CHECK(state_difference(future.value().state, filter.state()).norm() < 1e-12);
  CHECK((filter.covariance() - expected).norm() < 1e-11);
  CHECK(state_difference(snapshot.state, initial).norm() == 0);
  accept(filter, measurement(Eigen::Vector4d(0.01, -0.02, 0.03, 0.001)));
  const Eigen::Vector3d acceleration_before = filter.state().acceleration_mps2;
  const auto completed = filter.state();
  CHECK(filter.change_motion(model(false), 0.4));
  CHECK(filter.motion()->linear_acceleration_enabled());
  CHECK((filter.state().acceleration_mps2 - acceleration_before).norm() == 0);
  CHECK((filter.state().center.metres() - completed.center.metres()).norm() == 0);
  CHECK(filter.state().alpha_radps2 == 0);
  CHECK(filter.covariance().row(alpha).isZero(0) && filter.covariance().col(alpha).isZero(0));
  // A completed correction is not a fresh initialization after a zero-time model switch.
  // Both read-only innovation and update must fail atomically until time advances.
  const auto switched_without_time = filter;
  const auto premature = measurement(Eigen::Vector4d(0.01, 0, 0, 0.001));
  CHECK(!filter.innovation(premature));
  CHECK(!filter.update(premature, gate));
  CHECK(filter.predict(core::Seconds(0)));
  CHECK(!filter.update(premature, gate));
  same(filter, switched_without_time);
  CHECK(filter.change_motion(model(true), 0.4));
  CHECK(filter.state().acceleration_mps2 == acceleration_before);
  CHECK_NEAR(filter.covariance()(alpha, alpha), 0.4, 0);
  CHECK(filter.predict(core::Seconds(0.02)));
  const auto switched_base = filter.state();
  const Eigen::Vector4d delta(0.01, 0, 0, 0.001);
  accept(filter, measurement(delta));
  CHECK((state_difference(filter.state(), switched_base) - injection(0.02, true) * delta)
            .norm() < 1e-10);

  EsoOptions bad_options;
  bad_options.translation_bandwidth_radps = 0;
  CHECK_THROWS(std::invalid_argument,
               Eso(zero_state(), StateCovariance::Identity(), model(), bad_options));
}
} // namespace

int main() {
  return test::run([] {
    check_poles_and_interface();
    check_initialization_and_batch();
    check_trajectory_and_wrap();
    check_rejections_and_elapsed();
    check_motion_and_future();
  });
}
