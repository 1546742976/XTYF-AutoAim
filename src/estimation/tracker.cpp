#include "autoaim/estimation/tracker.hpp"
#include "autoaim/math/numeric.hpp"
#include <algorithm>
#include <limits>

namespace autoaim::estimation {
namespace {
constexpr double infinity = std::numeric_limits<double>::infinity();
struct Hypothesis {
  Ekf filter;
  HealthMonitor health;
  double cost = infinity;
  bool updated = false;
  bool reset_required = false;
  std::size_t last_plate = 0;
  std::size_t last_candidate = 0;
  std::shared_ptr<const Observation> last_observation;
  Hypothesis(TargetState state, const TrackerOptions& options, std::shared_ptr<const MotionModel> motion)
      : filter(std::move(state), options.initial_covariance, std::move(motion)), health(options.health) {}
};
struct ProfileBank {
  std::shared_ptr<const GeometryProfile> geometry;
  std::vector<Hypothesis> hypotheses;
  std::unique_ptr<IdentityResolver> identity;
};
}  // namespace

class Tracker::Impl {
public:
  Impl(std::uint64_t id, std::vector<std::shared_ptr<const GeometryProfile>> profiles,
      std::shared_ptr<const MotionModel> cv, std::shared_ptr<const MotionModel> ca, TrackerOptions options)
      : target_id(id), profiles(std::move(profiles)), cv(std::move(cv)), ca(std::move(ca)), options(std::move(options)),
        state(std::make_unique<TrackingStateMachine>(this->options.tracking)),
        motion_selection(std::make_unique<MotionSelector>(this->options.motion_selection)) {
    if (id == 0 || !this->cv || !this->ca || this->cv->kind() != MotionKind::constant_velocity ||
        this->ca->kind() != MotionKind::bounded_acceleration || this->profiles.empty() ||
        !this->options.maximum_hypotheses || !math::covariance_valid(this->options.initial_covariance) ||
        !std::isfinite(this->options.association_margin) || this->options.association_margin <= 0 ||
        !std::isfinite(this->options.initial_alpha_variance) || this->options.initial_alpha_variance <= 0)
      throw std::invalid_argument("Invalid tracker construction");
    geometry_selection = make_selector();
  }
  std::unique_ptr<GeometrySelector> make_selector() const {
    return std::make_unique<GeometrySelector>(profiles, options.geometry, options.geometry_minimum_dwell,
      options.geometry_complexity_penalty, options.calibration_id);
  }
  bool initialize(const Observation& observation) {
    std::size_t count = 0;
    for (const auto& profile : profiles) for (std::size_t i = 0; i < observation.world_candidates.size(); ++i)
      if (observation.pnp.candidates[i].geometry_accepted && observation.world_candidates[i].covariance)
        count += profile->plates.size();
    if (!count || count > options.maximum_hypotheses) return false;
    banks.clear(); banks.reserve(profiles.size());
    for (const auto& profile : profiles) {
      ProfileBank bank; bank.geometry = profile;
      for (std::size_t candidate = 0; candidate < observation.world_candidates.size(); ++candidate) {
        if (!observation.pnp.candidates[candidate].geometry_accepted || !observation.world_candidates[candidate].covariance) continue;
        for (const auto& seed : initial_identity_hypotheses(observation.world_candidates[candidate], *profile))
          bank.hypotheses.emplace_back(seed.state, options, cv);
      }
      bank.identity = std::make_unique<IdentityResolver>(bank.hypotheses.size(), options.identity);
      banks.push_back(std::move(bank));
    }
    geometry_selection = make_selector();
    state = std::make_unique<TrackingStateMachine>(options.tracking);
    motion_selection = std::make_unique<MotionSelector>(options.motion_selection);
    state_time = observation.source.exposure;
    last_snapshot.reset();
    return true;
  }

  void update_hypothesis(Hypothesis& hypothesis, const GeometryProfile& geometry, const ObservationBatch& observations) {
    if (hypothesis.reset_required) {
      hypothesis.updated = false; hypothesis.cost = options.identity.maximum_cost_per_observation;
      hypothesis.last_observation.reset(); return;
    }
    const std::size_t boards = geometry.plates.size(), seen = observations.size();
    Eigen::MatrixXd costs = Eigen::MatrixXd::Constant(boards, seen, infinity);
    std::vector<std::vector<std::optional<AssociationCandidate>>> choices(boards);
    for (auto& row : choices) row.resize(seen);
    for (std::size_t i = 0; i < seen; ++i) {
      const auto candidates = association_candidates(hypothesis.filter, *state_time, geometry,
        *observations[i], measurement, options.nis);
      for (const auto& candidate : candidates) {
        if (candidate.prior_nis < costs(candidate.physical_plate, i)) {
          costs(candidate.physical_plate, i) = candidate.prior_nis;
          choices[candidate.physical_plate][i] = candidate;
        }
      }
    }
    const auto assignments = mutual_nearest_assignment(costs, options.nis.limit(measurement.dimension()), options.association_margin);
    hypothesis.updated = false;
    hypothesis.last_observation.reset();
    // 未解释的观测也计入假设代价，避免只挑易解释的一块板虚增支持。
    double total = (seen - assignments.size()) * options.identity.maximum_cost_per_observation;
    for (const auto& assignment : assignments) total += assignment.cost;
    hypothesis.cost = total / seen;
    for (const auto& assignment : assignments) {
      const auto& choice = *choices[assignment.track][assignment.observation];
      const auto& observation = observations[assignment.observation];
      auto current = measurement.linearize(hypothesis.filter.state(), geometry, choice.physical_plate,
        observation->world_candidates[choice.pose_candidate]);
      if (!current) continue;
      // 同帧多个板共用姿态/标定误差，乘观测数给出保守界，不把这些误差当作独立重复测量。
      current.value().noise *= assignments.size();
      auto report = hypothesis.filter.update(current.value(), options.nis.limit(measurement.dimension()));
      if (!report || !report.value().accepted) continue;
      hypothesis.updated = true;
      hypothesis.last_plate = choice.physical_plate;
      hypothesis.last_candidate = choice.pose_candidate;
      hypothesis.last_observation = observation;
    }
    const auto health = hypothesis.health.observe(hypothesis.filter.state(), hypothesis.filter.covariance(), hypothesis.updated);
    if (health.reset_required) hypothesis.reset_required = true;
    if (health.reset_required || !hypothesis.updated) {
      hypothesis.updated = false; hypothesis.cost = options.identity.maximum_cost_per_observation;
    }
  }

  std::uint64_t target_id;
  std::vector<std::shared_ptr<const GeometryProfile>> profiles;
  std::shared_ptr<const MotionModel> cv, ca;
  TrackerOptions options;
  MeasurementModel measurement{MeasurementKind::full_pose};
  std::vector<ProfileBank> banks;
  std::unique_ptr<GeometrySelector> geometry_selection;
  std::unique_ptr<TrackingStateMachine> state;
  std::unique_ptr<MotionSelector> motion_selection;
  std::optional<core::TimePoint> state_time;
  std::optional<core::Stamp> last_processed;
  std::shared_ptr<const TargetSnapshot> last_snapshot;
};

Tracker::Tracker(std::uint64_t id, std::vector<std::shared_ptr<const GeometryProfile>> profiles,
    std::shared_ptr<const MotionModel> cv, std::shared_ptr<const MotionModel> ca, TrackerOptions options)
    : impl_(std::make_unique<Impl>(id, std::move(profiles), std::move(cv), std::move(ca), std::move(options))) {}
Tracker::~Tracker() = default;
std::uint64_t Tracker::id() const noexcept { return impl_->target_id; }
std::size_t Tracker::hypothesis_count() const noexcept {
  std::size_t count = 0; for (const auto& bank : impl_->banks) count += bank.hypotheses.size(); return count;
}

core::Result<bool> Tracker::update(const ObservationBatch& observations) {
  using Result = core::Result<bool>;
  if (observations.empty() || observations.size() > 4 || !observations.front())
    return Result::failure(core::ErrorCode::invalid_input, "Tracker expects 1..4 same-target observations");
  const auto& source = observations.front()->source;
  for (const auto& observation : observations)
    if (!observation || !observation->pnp.pose_valid || observation->source.frame_id != source.frame_id ||
        observation->source.generation != source.generation || !core::same_time(observation->source.exposure, source.exposure))
      return Result::failure(core::ErrorCode::invalid_input, "Tracker batch source mismatch");
  auto& data = *impl_;
  if (data.last_processed && (source.generation != data.last_processed->generation ||
      source.frame_id <= data.last_processed->frame_id || source.exposure.domain() != data.last_processed->exposure.domain() ||
      core::elapsed(source.exposure, data.last_processed->exposure).value() <= 0))
    return Result::failure(core::ErrorCode::out_of_order, "Tracker rejected old observation");
  bool any_healthy = false;
  for (const auto& bank : data.banks) for (const auto& hypothesis : bank.hypotheses)
    any_healthy = any_healthy || !hypothesis.reset_required;
  if (!any_healthy) {
    if (!data.initialize(*observations.front())) return Result::failure(core::ErrorCode::unavailable, "No usable bounded identity hypotheses");
  } else {
    const auto dt = core::elapsed(source.exposure, *data.state_time);
    bool predicted = true;
    for (auto& bank : data.banks) for (auto& hypothesis : bank.hypotheses)
      if (!hypothesis.reset_required && !hypothesis.filter.predict(dt)) predicted = false;
    if (!predicted && !data.initialize(*observations.front()))
      return Result::failure(core::ErrorCode::unavailable, "Tracker reinitialization failed after prediction rejection");
    data.state_time = source.exposure;
  }
  data.last_processed.emplace(source);
  std::vector<double> geometry_costs;
  for (auto& bank : data.banks) {
    std::vector<double> identity_costs;
    for (auto& hypothesis : bank.hypotheses) {
      data.update_hypothesis(hypothesis, *bank.geometry, observations);
      identity_costs.push_back(hypothesis.cost);
    }
    bank.identity->observe(source, identity_costs);
    geometry_costs.push_back(*std::min_element(identity_costs.begin(), identity_costs.end()));
  }
  data.geometry_selection->observe(source, geometry_costs);
  const auto geometry_choice = data.geometry_selection->selection(source.exposure.domain());
  const auto geometry_index = geometry_choice ? geometry_choice->index :
    static_cast<std::size_t>(std::min_element(geometry_costs.begin(), geometry_costs.end()) - geometry_costs.begin());
  auto& bank = data.banks[geometry_index];
  const auto identity = bank.identity->selected();
  const auto best = identity ? *identity : static_cast<std::size_t>(std::min_element(bank.hypotheses.begin(), bank.hypotheses.end(),
    [](const auto& a, const auto& b) { return a.cost < b.cost; }) - bank.hypotheses.begin());
  auto& chosen = bank.hypotheses[best];
  const bool geometry_known = geometry_choice && geometry_choice->supported && geometry_choice->calibrated;
  if (!chosen.updated || !chosen.last_observation) {
    data.state->observe(source, {false, false, false, false, false, 0});
    return Result::success(false);
  }
  const auto& observation = *chosen.last_observation;
  constexpr int phase = component_index(StateComponent::phase);
  data.state->observe(source, {true, observation.reliable, bool(identity), geometry_known, false,
    chosen.filter.covariance()(phase, phase)});
  data.motion_selection->observe(source, chosen.filter.state().omega_radps,
    bool(identity) && geometry_known && observation.reliable);
  const auto motion = data.motion_selection->kind() == MotionKind::constant_velocity ? data.cv : data.ca;
  for (auto& profile : data.banks) for (auto& hypothesis : profile.hypotheses)
    if (hypothesis.filter.motion()->kind() != motion->kind()) {
      auto changed = hypothesis.filter.change_motion(motion, data.options.initial_alpha_variance);
      if (!changed) return Result::failure(changed.error().code, changed.error().message);
    }
  const bool reliable = observation.reliable && bool(identity) && geometry_known;
  data.last_snapshot = std::make_shared<const TargetSnapshot>(source, *data.state_time, data.target_id,
    chosen.filter.state(), chosen.filter.covariance(), identity ? std::optional<std::size_t>(chosen.last_plate) : std::nullopt,
    data.state->quality(), reliable, observation.historical_pose, chosen.filter.motion(), bank.geometry,
    observation.world_candidates[chosen.last_candidate], observation.pnp.dimensions, observation.time_origin,
    observation.timing_uncertainty, observation.timing_evidence);
  return Result::success(true);
}

double Tracker::association_cost(const Observation& observation) const {
  if (!impl_->state_time || !impl_->last_processed || observation.source.generation != impl_->last_processed->generation ||
      observation.source.exposure.domain() != impl_->state_time->domain() ||
      core::elapsed(observation.source.exposure, *impl_->state_time).value() < 0) return infinity;
  double best = infinity;
  for (const auto& bank : impl_->banks) for (const auto& hypothesis : bank.hypotheses) {
    if (hypothesis.reset_required) continue;
    auto temporary = hypothesis.filter;
    if (!temporary.predict(core::elapsed(observation.source.exposure, *impl_->state_time))) continue;
    const auto candidates = association_candidates(temporary, observation.source.exposure, *bank.geometry,
      observation, impl_->measurement, impl_->options.nis);
    if (!candidates.empty()) best = std::min(best, candidates.front().prior_nis);
  }
  return best;
}
std::shared_ptr<const TargetSnapshot> Tracker::snapshot(core::TimePoint now) {
  impl_->state->tick(now);
  if (impl_->state->lifecycle() == TrackLifecycle::lost || !impl_->last_snapshot) return {};
  const auto& previous = *impl_->last_snapshot;
  if (previous.quality != impl_->state->quality()) {
    impl_->last_snapshot = std::make_shared<const TargetSnapshot>(previous.source, previous.state_time, previous.target_id,
      previous.state, previous.covariance, previous.physical_plate, impl_->state->quality(), previous.pose_reliable,
      previous.historical_pose, previous.motion, previous.geometry, previous.visible_plate, previous.visible_dimensions, previous.time_origin,
      previous.timing_uncertainty, previous.timing_evidence);
  }
  return impl_->last_snapshot;
}

class TrackerSet::Impl {
public:
  struct Entry {
    std::unique_ptr<Tracker> tracker;
    int class_id;
    vision::TeamColor color;
  };
  Impl(core::Generation epoch, std::vector<std::shared_ptr<const GeometryProfile>> geometry,
      std::shared_ptr<const MotionModel> cv_model, std::shared_ptr<const MotionModel> ca_model,
      TrackerOptions per_target, TrackerSetOptions limits)
      : generation(epoch), profiles(std::move(geometry)), cv(std::move(cv_model)), ca(std::move(ca_model)),
        tracker_options(std::move(per_target)), options(limits) {
    if (!options.maximum_targets || !std::isfinite(options.association_margin) || options.association_margin <= 0 ||
        options.maximum_same_target_separation.value() <= 0) throw std::invalid_argument("Invalid tracker set limits");
    Tracker validate(1, profiles, cv, ca, tracker_options);
  }
  bool compatible(const Entry& entry, const Observation& observation) const {
    return entry.color == observation.detection.color &&
      (entry.class_id < 0 || observation.detection.class_id < 0 || entry.class_id == observation.detection.class_id);
  }
  core::Generation generation;
  std::vector<std::shared_ptr<const GeometryProfile>> profiles;
  std::shared_ptr<const MotionModel> cv, ca;
  TrackerOptions tracker_options;
  TrackerSetOptions options;
  std::vector<Entry> entries;
  std::optional<core::Stamp> last_source;
  std::uint64_t next_id = 1;
  std::size_t dropped = 0;
};
TrackerSet::TrackerSet(core::Generation generation, std::vector<std::shared_ptr<const GeometryProfile>> profiles,
    std::shared_ptr<const MotionModel> cv, std::shared_ptr<const MotionModel> ca,
    TrackerOptions per_target, TrackerSetOptions options)
    : impl_(std::make_unique<Impl>(generation, std::move(profiles), std::move(cv), std::move(ca), std::move(per_target), options)) {}
TrackerSet::~TrackerSet() = default;
core::Result<bool> TrackerSet::update(const core::Stamp& source, const ObservationBatch& observations) {
  using Result = core::Result<bool>;
  auto& data = *impl_;
  if (source.generation != data.generation || (data.last_source && (source.frame_id <= data.last_source->frame_id ||
      source.exposure.domain() != data.last_source->exposure.domain() ||
      core::elapsed(source.exposure, data.last_source->exposure).value() <= 0)))
    return Result::failure(core::ErrorCode::out_of_order, "Tracker set rejected source order/generation");
  for (const auto& observation : observations)
    if (!observation || !observation->pnp.pose_valid || !observation->pnp.selected ||
        observation->source.frame_id != source.frame_id || observation->source.generation != source.generation ||
        !core::same_time(observation->source.exposure, source.exposure))
      return Result::failure(core::ErrorCode::invalid_input, "Tracker set batch mismatch");
  data.last_source.emplace(source);
  data.entries.erase(std::remove_if(data.entries.begin(), data.entries.end(), [&](auto& entry) {
    return !entry.tracker->snapshot(source.exposure);
  }), data.entries.end());
  std::vector<ObservationBatch> assigned(data.entries.size());
  std::vector<bool> used(observations.size(), false);
  const double gate = data.tracker_options.nis.limit(6);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    double best = infinity, second = infinity;
    std::size_t winner = 0;
    for (std::size_t t = 0; t < data.entries.size(); ++t) {
      if (!data.compatible(data.entries[t], *observations[i])) continue;
      const double cost = data.entries[t].tracker->association_cost(*observations[i]);
      if (cost < best) { second = best; best = cost; winner = t; }
      else second = std::min(second, cost);
    }
    if (best > gate) continue;
    used[i] = true;
    if (second - best < data.options.association_margin || assigned[winner].size() == 4) { ++data.dropped; continue; }
    assigned[winner].push_back(observations[i]);
  }
  for (std::size_t t = 0; t < assigned.size(); ++t) {
    if (assigned[t].empty()) continue;
    auto result = data.entries[t].tracker->update(assigned[t]);
    if (!result) return result;
  }
  for (std::size_t seed = 0; seed < observations.size(); ++seed) {
    if (used[seed]) continue;
    if (data.entries.size() >= data.options.maximum_targets) { used[seed] = true; ++data.dropped; continue; }
    if (data.next_id == UINT64_MAX) return Result::failure(core::ErrorCode::fault, "Target id exhausted");
    const auto& first = *observations[seed];
    Impl::Entry entry{std::make_unique<Tracker>(data.next_id, data.profiles, data.cv, data.ca, data.tracker_options),
      first.detection.class_id, first.detection.color};
    auto initialized = entry.tracker->update({observations[seed]});
    used[seed] = true;
    if (!initialized || !initialized.value()) { ++data.dropped; continue; }
    ObservationBatch group{observations[seed]};
    const auto center = first.world_candidates[*first.pnp.selected].plate_to_world.value().translation();
    for (std::size_t next = seed + 1; next < observations.size(); ++next) {
      if (used[next] || group.size() == 4 || !data.compatible(entry, *observations[next])) continue;
      const auto position = observations[next]->world_candidates[*observations[next]->pnp.selected].plate_to_world.value().translation();
      if ((position - center).norm() > data.options.maximum_same_target_separation.value() ||
          entry.tracker->association_cost(*observations[next]) > gate) continue;
      used[next] = true; group.push_back(observations[next]);
    }
    if (group.size() > 1) {
      // 种子探测不算一次额外观测支持；完整同帧组从新的单写者状态初始化。
      entry.tracker.reset();
      entry.tracker = std::make_unique<Tracker>(data.next_id, data.profiles, data.cv, data.ca, data.tracker_options);
      auto updated = entry.tracker->update(group);
      if (!updated) return updated;
      if (!updated.value()) { data.dropped += group.size(); continue; }
    }
    ++data.next_id;
    data.entries.push_back(std::move(entry));
  }
  return Result::success(true);
}
std::vector<std::shared_ptr<const TargetSnapshot>> TrackerSet::snapshots(core::TimePoint now) {
  std::vector<std::shared_ptr<const TargetSnapshot>> result;
  for (auto& entry : impl_->entries) if (auto snapshot = entry.tracker->snapshot(now)) result.push_back(std::move(snapshot));
  return result;
}
void TrackerSet::reset(core::Generation next_generation) {
  if (next_generation <= impl_->generation) throw std::invalid_argument("Tracker generation must increase");
  impl_->entries.clear(); impl_->last_source.reset(); impl_->generation = next_generation;
}
std::size_t TrackerSet::dropped_observations() const noexcept { return impl_->dropped; }
}  // namespace autoaim::estimation
