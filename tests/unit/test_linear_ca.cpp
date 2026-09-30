#include "autoaim/decision/predictor.hpp"
#include "autoaim/estimation/ekf.hpp"
#include "autoaim/pipeline/bootstrap.hpp"
#include "support/geometry_fixture.hpp"
#include "support/tracker_fixture.hpp"
#include "test_support.hpp"
#include <array>
#include <filesystem>
#include <limits>
#include <yaml-cpp/yaml.h>

// 仅由 CMake 在启用 I3-LINEAR-CA 时注册。
namespace {
using namespace autoaim;
constexpr int acceleration = estimation::component_index(estimation::StateComponent::ax);
constexpr int alpha = estimation::component_index(estimation::StateComponent::alpha);

estimation::TargetState initial_state() {
  return {math::Point3<math::WorldFrame>({3, 0, 1}), Eigen::Vector3d::Zero(),
          core::Radians(0.2), 0.5, 0, Eigen::Vector3d::Zero()};
}

void check_bootstrap(const pipeline::PipelineConfig& loaded, const YAML::Node& parameters) {
  const auto motion = parameters["motion"];
  const double horizon = motion["maximum_horizon_s"].as<double>();
  auto state = initial_state();
  state.velocity_mps = {0.4, -0.2, 0.1};
  state.acceleration_mps2 = {0.8, -0.4, 0.3};
  state.alpha_radps2 = 1000;
  const auto zero = estimation::StateCovariance::Zero().eval();

  for (const auto& model : {loaded.cv_motion, loaded.ca_motion}) {
    CHECK(model && model->linear_acceleration_enabled());
    const bool bounded = model == loaded.ca_motion;
    CHECK(model->kind() == (bounded ? estimation::MotionKind::bounded_acceleration
                                    : estimation::MotionKind::constant_velocity));
    const estimation::MotionModel angular_reference = bounded
        ? estimation::MotionModel(motion["linear_accel_psd"].as<double>(),
                                  motion["angular_jerk_psd"].as<double>(), core::Seconds(horizon),
                                  motion["maximum_alpha_radps2"].as<double>())
        : estimation::MotionModel(motion["linear_accel_psd"].as<double>(),
                                  motion["angular_accel_psd"].as<double>(), core::Seconds(horizon));
    const double dt = horizon / 4;
    const auto actual = model->propagate(state, zero, core::Seconds(dt));
    const auto angular = angular_reference.propagate(state, zero, core::Seconds(dt));
    CHECK(actual && angular);
    CHECK((actual.value().state.center.metres() - state.center.metres() -
           state.velocity_mps * dt - state.acceleration_mps2 * dt * dt / 2).norm() < 1e-12);
    CHECK((actual.value().state.velocity_mps - state.velocity_mps -
           state.acceleration_mps2 * dt).norm() < 1e-12);
    CHECK((actual.value().state.acceleration_mps2 - state.acceleration_mps2).norm() == 0);
    CHECK_NEAR(actual.value().covariance(0, 0), std::pow(dt, 5) / 20, 1e-12);
    CHECK_NEAR(actual.value().covariance(3, 3), std::pow(dt, 3) / 3, 1e-12);
    CHECK_NEAR(actual.value().covariance(acceleration, acceleration), dt, 1e-12);
    CHECK_NEAR(actual.value().state.phase.value(), angular.value().state.phase.value(), 1e-12);
    CHECK_NEAR(actual.value().state.omega_radps, angular.value().state.omega_radps, 1e-12);
    CHECK_NEAR(actual.value().state.alpha_radps2, angular.value().state.alpha_radps2, 1e-12);
    CHECK((actual.value().covariance.block<3, 3>(6, 6) -
           angular.value().covariance.block<3, 3>(6, 6)).norm() < 1e-12);
    CHECK(model->propagate(state, zero, core::Seconds(horizon)));
    CHECK(!model->propagate(state, zero, core::Seconds(horizon + 0.01)));
  }
}

void check_filter(const pipeline::PipelineConfig& loaded, double maximum_horizon) {
  estimation::Ekf filter(initial_state(), loaded.tracker.initial_covariance, loaded.cv_motion);
  const Eigen::Vector3d truth_acceleration(0.8, -0.4, 0.3);
  estimation::LinearizedMeasurement measurement{
      Eigen::Vector3d::Zero(), Eigen::MatrixXd::Zero(3, estimation::state_dimension),
      Eigen::Matrix3d::Identity() * 1e-5};
  measurement.jacobian.leftCols<3>().setIdentity();
  const double dt = 0.01;
  for (int step = 1; step <= 400; ++step) {
    const double elapsed = step * dt;
    CHECK(filter.predict(core::Seconds(dt)));
    const Eigen::Vector3d truth_position = initial_state().center.metres() +
                                          truth_acceleration * elapsed * elapsed / 2;
    measurement.residual = truth_position - filter.state().center.metres();
    const auto corrected = filter.update(measurement, loaded.tracker.nis.limit(3));
    CHECK(corrected && corrected.value().accepted);
    CHECK(math::covariance_valid(filter.covariance()));
  }
  CHECK((filter.state().acceleration_mps2 - truth_acceleration).norm() < 1e-3);
  CHECK((filter.state().velocity_mps - truth_acceleration * 4).norm() < 1e-3);

  // 切换角模型只能映射角加速度；平移 CA 状态及完整平移协方差保持。
  const std::array<int, 9> translation{{0, 1, 2, 3, 4, 5, 9, 10, 11}};
  for (const auto& replacement : {loaded.ca_motion, loaded.cv_motion, loaded.ca_motion}) {
    const auto before_state = filter.state();
    const auto before_covariance = filter.covariance();
    CHECK(filter.change_motion(replacement, 0.2));
    CHECK(filter.motion() == replacement && filter.motion()->linear_acceleration_enabled());
    CHECK((filter.state().acceleration_mps2 - before_state.acceleration_mps2).norm() == 0);
    CHECK((filter.state().center.metres() - before_state.center.metres()).norm() == 0);
    CHECK((filter.state().velocity_mps - before_state.velocity_mps).norm() == 0);
    for (const int row : translation)
      for (const int column : translation)
        CHECK_NEAR(filter.covariance()(row, column), before_covariance(row, column), 0);
    CHECK_NEAR(filter.covariance()(alpha, alpha),
               replacement->kind() == estimation::MotionKind::bounded_acceleration ? 0.2 : 0, 0);
  }

  const auto before_state = filter.state();
  const auto before_covariance = filter.covariance();
  const auto before_motion = filter.motion();
  const auto unchanged = [&] {
    CHECK(estimation::state_difference(filter.state(), before_state).norm() == 0);
    CHECK((filter.covariance() - before_covariance).norm() == 0);
    CHECK(filter.motion() == before_motion);
  };
  CHECK(filter.predict(core::Seconds(0)));
  unchanged();
  CHECK(!filter.predict(core::Seconds(-0.01)));
  unchanged();
  CHECK(!filter.predict(core::Seconds(maximum_horizon + 0.01)));
  unchanged();
  measurement.residual.setConstant(1e6);
  const auto rejected = filter.update(measurement, loaded.tracker.nis.limit(3));
  CHECK(rejected && !rejected.value().accepted && rejected.value().observation_dimension == 3);
  unchanged();
  CHECK(!filter.update(measurement, -1));
  unchanged();
  measurement.noise(0, 0) = -1;
  CHECK(!filter.update(measurement, loaded.tracker.nis.limit(3)));
  unchanged();
  measurement.noise(0, 0) = 1e-5;
  measurement.residual[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(!filter.update(measurement, loaded.tracker.nis.limit(3)));
  unchanged();
  CHECK(!filter.change_motion(nullptr, 0.2));
  unchanged();
  CHECK(!filter.change_motion(loaded.cv_motion, -1));
  unchanged();
}

void check_snapshot_prediction(const pipeline::PipelineConfig& loaded) {
  auto profile = std::make_shared<const estimation::GeometryProfile>(
      test::geometry(4, estimation::HeightLayout::distinct, true));
  estimation::Tracker tracker(7, {profile}, loaded.cv_motion, loaded.ca_motion,
                               test::tracker_options());
  std::shared_ptr<const estimation::TargetSnapshot> snapshot;
  for (std::uint64_t frame = 1; frame <= 60; ++frame) {
    const double elapsed = frame * 0.02;
    const core::Stamp source(frame, 1,
        core::TimePoint(frame * 20000000, core::ClockDomain::replay));
    const math::Point3<math::WorldFrame> center({3 + 0.1 * elapsed + 0.05 * elapsed * elapsed,
                                               0, 1});
    estimation::ObservationBatch observations;
    for (const std::size_t board : {2, 3, 0, 1})
      observations.push_back(test::observation(*profile, source, center,
                                               core::Radians(0.2 + 0.5 * elapsed), board));
    const auto updated = tracker.update(observations);
    CHECK(updated && updated.value());
    snapshot = tracker.snapshot(source.exposure);
    CHECK(snapshot && snapshot->source.frame_id == frame);
    CHECK(snapshot->motion == loaded.cv_motion || snapshot->motion == loaded.ca_motion);
    CHECK(snapshot->motion->linear_acceleration_enabled());
  }
  CHECK(snapshot->physical_plate && snapshot->pose_reliable);
  CHECK(snapshot->quality == estimation::TrackingQuality::converged);
  CHECK_NEAR(snapshot->state.acceleration_mps2.x(), 0.1, 0.03);
  const auto state_before = snapshot->state;
  const auto covariance_before = snapshot->covariance;
  const double dt = 0.1;
  const auto when = core::advance(snapshot->state_time, core::Seconds(dt));
  const auto actual = decision::predict_future(*snapshot, when, {0, 0, 1});
  const auto model = snapshot->motion->propagate(snapshot->state, snapshot->covariance,
                                                 core::Seconds(dt));
  CHECK(actual && model && actual.value().plates.size() == 4);
  CHECK(actual.value().source.frame_id == snapshot->source.frame_id);
  CHECK(core::same_time(actual.value().source.exposure, snapshot->source.exposure));
  CHECK(estimation::state_difference(actual.value().state, model.value().state).norm() < 1e-12);
  CHECK((actual.value().covariance - model.value().covariance).norm() < 1e-12);
  CHECK((actual.value().state.center.metres() - state_before.center.metres() -
         state_before.velocity_mps * dt - state_before.acceleration_mps2 * dt * dt / 2).norm()
        < 1e-12);
  CHECK((actual.value().state.velocity_mps - state_before.velocity_mps -
         state_before.acceleration_mps2 * dt).norm() < 1e-12);
  const auto& p = covariance_before;
  const double expected_position_variance = p(0, 0) + 2 * dt * p(0, 3) +
      dt * dt * (p(3, 3) + p(0, 9)) + std::pow(dt, 3) * p(3, 9) +
      std::pow(dt, 4) * p(9, 9) / 4 + std::pow(dt, 5) / 20;
  CHECK_NEAR(actual.value().covariance(0, 0), expected_position_variance, 1e-12);
  for (const auto& plate : actual.value().plates)
    CHECK(plate.reliable && math::covariance_valid(plate.covariance));
  CHECK(!decision::predict_future(*snapshot,
          core::advance(snapshot->state_time, core::Seconds(-0.01)), {0, 0, 1}));
  CHECK(estimation::state_difference(snapshot->state, state_before).norm() == 0);
  CHECK((snapshot->covariance - covariance_before).norm() == 0);
}
} // namespace

int main() {
  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto path = root / "config/offline/armor.yaml";
    const auto config = autoaim::pipeline::load_pipeline_config(path);
    CHECK(config);
    CHECK(config.value().configuration_snapshot);
    const auto parameters = config.value().configuration_snapshot->effective;
    check_bootstrap(config.value(), parameters);
    check_filter(config.value(), parameters["motion"]["maximum_horizon_s"].as<double>());
    check_snapshot_prediction(config.value());
  });
}
