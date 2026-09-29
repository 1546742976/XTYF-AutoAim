#include "autoaim/pipeline/pipeline.hpp"
#include "autoaim/pipeline/frame_sync.hpp"
#include "autoaim/pipeline/command_slot.hpp"
#include "autoaim/pipeline/scheduler.hpp"
#include "autoaim/mission/sentry/sentry_mission.hpp"
#include "autoaim/hal/file_replay.hpp"
#include <future>

namespace autoaim::pipeline {
struct Pipeline::Impl {
  PipelineConfig config;
  std::unique_ptr<hal::Clock> clock;
  FrameQueue queue;
  FrameSync history;
  ResultAdmission admission;
  CommandSlot commands;
  std::unique_ptr<vision::Detector> detector;
  estimation::TrackerSet trackers;
  mission::TargetSelector target_selector;
  mission::InfantryMission infantry;
  decision::ArmorSelector armor_selector;
  decision::AimAdequacy adequacy;
  control::CorrectionSmoother yaw_smoother;
  control::CorrectionSmoother pitch_smoother;
  mutable std::mutex context_mutex;
  control::GuardContext context;
  control::Publisher publisher;
  Scheduler scheduler;
  bool started = false;
  bool closed = false;
  PipelineMetrics metrics;
  Observer observer;
  std::map<std::pair<core::Generation, core::FrameId>, FrameTask> active_frames;

  control::GuardContext current_context() const {
    std::lock_guard<std::mutex> lock(context_mutex); return context;
  }
  void fail(std::exception_ptr error) {
    { std::lock_guard<std::mutex> lock(context_mutex); context.fault_latched = true; }
    queue.close(); commands.close(); publisher.fail(error);
  }
  void withdraw() {
    const auto now = clock->now();
    adequacy.reset(); yaw_smoother.emergency_stop(now); pitch_smoother.emergency_stop(now);
    publisher.invalidate(); ++metrics.no_solution;
  }
  void refresh_mode() {
    if (config.role != core::Role::infantry) return;
    const auto state = current_context();
    const auto now = clock->now();
    std::optional<mission::OperatorSignal> input;
    if (state.feedback && state.feedback->operator_input) {
      const auto& op = *state.feedback->operator_input;
      input.emplace(mission::OperatorSignal{op.sampled_at, state.feedback->generation, op.valid,
        op.intervening, op.enable_event, op.source_evidence});
    }
    if (!infantry.update(input, state.control_channel, state.fault_latched, now)) return;
    const auto generation = infantry.generation();
    // 先撤销发布槽，再公开新模式。已跨过发送接纳边界的写可完成，未接纳的旧写不得穿过切换。
    publisher.invalidate();
    {
      std::lock_guard<std::mutex> lock(context_mutex);
      context.authority = infantry.authority(); context.generation = generation; context.mode_since = now;
      context.feedback.reset();  // 触发切换的旧世代反馈不能被重新标成新世代。
    }
    queue.reset(generation); commands.reset(generation, now); admission.reset(generation, now);
    history.reset(generation); trackers.reset(generation);
    target_selector.reset(); armor_selector.reset(); adequacy.reset();
    yaw_smoother.emergency_stop(now); pitch_smoother.emergency_stop(now);
  }
  void decide() {
    const auto now = clock->now();
    const auto state = current_context();
    if (!state.feedback) { withdraw(); return; }
    const auto& feedback = *state.feedback;
    if (!feedback.pose_valid || !std::isfinite(feedback.yaw_rad) || !std::isfinite(feedback.pitch_rad) ||
        !std::isfinite(feedback.bullet_speed_mps) || feedback.bullet_speed_mps <= 0 ||
        !core::fresh(feedback.sampled_at, now, config.guard.timing.feedback_age)) { withdraw(); return; }
    const Eigen::Vector3d actual_reference(std::cos(feedback.pitch_rad) * std::cos(feedback.yaw_rad),
      std::cos(feedback.pitch_rad) * std::sin(feedback.yaw_rad), std::sin(feedback.pitch_rad));
    const Eigen::Vector3d actual_world = config.aim_reference_to_world * actual_reference;
    const auto snapshot = target_selector.select(trackers.snapshots(now), state.generation, now,
      config.launch_origin.metres(), actual_world);
    if (!snapshot) { withdraw(); return; }
    const auto predicted = decision::predict_future(*snapshot, now, config.prediction);
    if (!predicted) { withdraw(); return; }
    std::vector<decision::InterceptSolution> candidates;
    for (const auto& plate : predicted.value().plates) {
      auto solution = decision::solve_intercept(*snapshot, {now, config.after_send_delay, config.launch_origin,
        core::MetresPerSecond(feedback.bullet_speed_mps), plate.physical_plate}, config.prediction, config.ballistic, config.intercept);
      if (solution) candidates.push_back(std::move(solution).value());
    }
    const auto selected = armor_selector.select(snapshot->source, snapshot->target_id, snapshot->geometry->id,
      candidates, actual_world, config.ballistic.gravity());
    if (!selected) { withdraw(); return; }
    const auto& solution = candidates[*selected];
    const auto angles = decision::direction_to_angles(config.aim_reference_to_world.conjugate() * solution.ballistic.initial_velocity_world_mps);
    const auto measured = decision::MeasuredPointing{feedback.sampled_at, feedback.generation,
      {core::Radians(feedback.yaw_rad), core::Radians(feedback.pitch_rad)}, feedback.pose_valid};
    const auto support = adequacy.evaluate(snapshot->source, {snapshot->target_id, snapshot->geometry->id, solution.plate.physical_plate},
      angles, measured, now);
    auto uncertainty = config.impact;
    // 曝光时序不确定度未被显式纳入 EKF 时间状态：在落点代理中保守合入时间误差。
    uncertainty.launch_timing_sigma_s = std::hypot(uncertainty.launch_timing_sigma_s, snapshot->timing_uncertainty.value());
    const auto margin = decision::evaluate_impact_margin(solution, config.launch_origin, config.ballistic, uncertainty, config.margin);
    const Eigen::Vector3d offset = config.aim_reference_to_world.conjugate() *
      (solution.plate.pose.value().translation() - config.launch_origin.metres());
    const mission::AimSolution aim{snapshot->source, angles, core::Metres(std::hypot(offset.x(), offset.y())), true};
    auto request = config.role == core::Role::infantry ? infantry.request(aim, measured, config.program_fire_requested) :
      mission::sentry_request(aim, state.generation, state.mode_since, config.program_fire_requested);
    if (request.authority.mode == core::ControlMode::assist) {
      request.angles.yaw = yaw_smoother.update(request.angles.yaw, now, request.control_requested);
      request.angles.pitch = pitch_smoother.update(request.angles.pitch, now, request.control_requested);
    }
    const control::AimFacts facts{snapshot->source, snapshot->source.exposure,
      snapshot->historical_pose && snapshot->historical_pose->reliable, solution.plate.reliable,
      solution.plate.physical_plate.has_value(), margin && margin.value().inside, true, support.settled};
    control::ControlIntent intent(snapshot->source, request.authority.role, request.authority.task,
      request.authority.mode, request.authority.space, request.control_requested, request.fire_requested,
      {request.angles.yaw, request.angles.pitch, 0, 0, 0, 0}, now, core::advance(now, config.intent_lifetime),
      facts, snapshot->timing_evidence, request.distance);
    ++metrics.decisions;
    if (commands.submit(std::move(intent), now)) {
      if (auto pending = commands.take(now); pending && publisher.submit(*pending)) ++metrics.submitted_intents;
    }
  }
  void consume(const vision::FramePacket& packet, const vision::DetectionBatch& batch) {
    const auto& source = packet.frame.stamp;
    const auto now = clock->now();
    const bool matched = batch.source.frame_id == source.frame_id && batch.source.generation == source.generation &&
      core::same_time(batch.source.exposure, source.exposure);
    if (!admission.accept(source, now, matched && packet.pose.has_value())) return;
    ++metrics.accepted_results;
    metrics.last_observation_age_s = core::elapsed(now, source.exposure).value();
    metrics.maximum_observation_age_s = std::max(metrics.maximum_observation_age_s, *metrics.last_observation_age_s);
    metrics.total_observation_age_s += *metrics.last_observation_age_s;
    metrics.detections += batch.detections.size();
    estimation::ObservationBatch observations;
    for (const auto& detection : batch.detections) {
      const auto size = config.plate_sizes.find(detection.class_id);
      if (size == config.plate_sizes.end()) continue;  // 未提供该分类尺寸，不猜大小装甲板。
      auto corners = config.corner_mapping.apply(detection, now.domain());
      if (config.refine) corners = vision::refine_corners(packet.frame.image, corners, config.refinement).detection;
      const auto pose = vision::solve_pose(source, corners, size->second, config.calibration, config.pnp,
        config.guard.device_id, config.guard.configuration_id, std::nullopt, std::nullopt);
      if (!pose.pose_valid) continue;
      ++metrics.valid_poses;
      auto observation = estimation::make_observation(packet, corners, pose, config.calibration, config.additional_pose_covariance);
      if (observation) observations.push_back(std::make_shared<const estimation::Observation>(std::move(observation).value()));
    }
    metrics.observations += observations.size();
    const auto updated = trackers.update(source, observations);
    if (!updated) throw std::runtime_error(updated.error().message);
    decide();
    if (observer) observer(packet, batch, observations);
  }

  Impl(PipelineConfig options, std::unique_ptr<hal::Clock> time, control::Publisher::Write write,
      control::PublishSchedule schedule, Observer view)
      : config(std::move(options)), clock(std::move(time)), queue(config.queue, 1),
        history(config.history_capacity, config.history_maximum_gap, 1),
        admission(1, clock->now(), config.queue.maximum_age), commands(1, clock->now()),
        detector(std::holds_alternative<vision::Yolov5Options>(config.detector) ?
          std::unique_ptr<vision::Detector>(std::make_unique<vision::OpenVinoDetector>(
            std::get<vision::Yolov5Options>(config.detector), config.queue.in_flight_limit)) : vision::make_detector(config.detector)),
        trackers(1, config.profiles, config.cv_motion, config.ca_motion, config.tracker, config.targets),
        target_selector(config.target_selection), infantry(config.infantry, 1, clock->now()),
        armor_selector(config.armor_selection), adequacy(config.adequacy),
        yaw_smoother(config.smoother), pitch_smoother(config.smoother),
        context{config.role == core::Role::infantry ? infantry.authority() : mission::sentry_authority(),
          1, clock->now(), std::nullopt, config.control_channel, false},
        publisher(control::CommandGuard(config.guard), *clock, [this] {
          std::lock_guard<std::mutex> lock(context_mutex); return context;
        }, std::move(write), config.publish_period, schedule), observer(std::move(view)) {}
};
Pipeline::Pipeline(PipelineConfig config, std::unique_ptr<hal::Clock> clock,
    control::Publisher::Write write, control::PublishSchedule schedule, Observer observer) {
  if (!clock) throw std::invalid_argument("Pipeline clock ownership required");
  impl_ = std::make_unique<Impl>(std::move(config), std::move(clock), std::move(write), schedule, std::move(observer));
}
Pipeline::~Pipeline() { close(); }
void Pipeline::start() {
  auto& self = *impl_;
  if (self.started || self.closed) throw std::logic_error("Pipeline cannot restart");
  self.publisher.start(); self.started = true;
}
void Pipeline::close() {
  auto& self = *impl_;
  if (self.closed) return;
  self.closed = true;
  // 先停止接纳和输出，再回收工作线程与实际推理输入。
  self.queue.close(); self.commands.close();
  self.scheduler.request_stop();
  // 停止输出先于等待耗时推理结束，避免回收期间仍保留旧许可。
  self.publisher.close();
  self.scheduler.stop();
  if (auto* async = dynamic_cast<vision::OpenVinoDetector*>(self.detector.get())) async->close();
  self.active_frames.clear();
}
control::PublisherStatus Pipeline::publisher_status() const { return impl_->publisher.status(); }
void Pipeline::feedback(hal::GimbalFeedback sample) {
  auto& self = *impl_;
  if (!self.started || self.closed) throw std::logic_error("Pipeline not active");
  const auto now = self.clock->now();
  // 输入适配层必须提供同域主机时间；拒绝未来样本，避免污染历史并阻挡后续正常反馈。
  if (sample.sampled_at.domain() != now.domain() || sample.sampled_at.nanoseconds() > now.nanoseconds()) return;
  if (!self.history.push(sample)) return;
  {
    std::lock_guard<std::mutex> lock(self.context_mutex);
    self.context.feedback.emplace(std::move(sample));
  }
  self.refresh_mode();
}
bool Pipeline::frame(const core::CapturedFrame& frame) {
  auto& self = *impl_;
  if (!self.started || self.closed) return false;
  ++self.metrics.received_frames;
  if (frame.image.width != self.config.queue.width || frame.image.height != self.config.queue.height ||
      frame.image.stride != self.config.queue.stride) throw std::invalid_argument("Frame format differs from configured calibration/pool");
  return self.queue.submit(frame.stamp, *frame.image.pixels, frame.received_at, frame.time_origin,
    frame.timing_uncertainty, frame.timing_evidence, self.clock->now());
}
void Pipeline::process_pending() {
  auto& self = *impl_;
  if (!self.started || self.closed) throw std::logic_error("Pipeline not active");
  try {
    if (auto* async = dynamic_cast<vision::OpenVinoDetector*>(self.detector.get())) {
      while (auto completed = async->take_completed()) {
        const auto& source = completed->packet->frame.stamp;
        const auto key = std::make_pair(source.generation, source.frame_id);
        if (completed->failure) std::rethrow_exception(completed->failure);
        if (!completed->detections) throw std::runtime_error("Inference completion lacks result");
        self.consume(*completed->packet, *completed->detections);
        self.active_frames.erase(key);
      }
      while (async->in_flight() < self.config.queue.in_flight_limit) {
        auto task = self.queue.take_latest(self.clock->now());
        if (!task) break;
        const auto& frame = *task->frame();
        const auto packet = std::make_shared<const vision::FramePacket>(frame,
          self.history.query(frame.stamp.exposure, frame.stamp.generation));
        const auto key = std::make_pair(frame.stamp.generation, frame.stamp.frame_id);
        self.active_frames.emplace(key, std::move(*task));
        if (!async->try_submit(packet)) throw std::logic_error("Inference slot/queue contract mismatch");
      }
      return;
    }
    while (auto task = self.queue.take_latest(self.clock->now())) {
      const auto& frame = *task->frame();
      const vision::FramePacket packet(frame, self.history.query(frame.stamp.exposure, frame.stamp.generation));
      const auto detections = self.detector->detect(packet);
      self.consume(packet, detections);
    }
  } catch (...) {
    self.fail(std::current_exception());
    if (auto* async = dynamic_cast<vision::OpenVinoDetector*>(self.detector.get())) async->close();
    self.active_frames.clear();
    throw;
  }
}
void Pipeline::finish_pending(core::Seconds timeout) {
  if (timeout.value() <= 0 || timeout.value() > 60) throw std::invalid_argument("Inference drain timeout outside (0,60]");
  process_pending();
  auto* async = dynamic_cast<vision::OpenVinoDetector*>(impl_->detector.get());
  if (!async) return;
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout.value());
  while (async->in_flight()) {
    if (std::chrono::steady_clock::now() >= until) {
      const auto error = std::make_exception_ptr(std::runtime_error("Offline inference drain timed out"));
      impl_->fail(error);
      std::rethrow_exception(error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    process_pending();
  }
}
bool Pipeline::replay_tick() {
  auto& self = *impl_;
  // 同一切换事件已经检查输入；等下一事件再检查新世代反馈，期间 guard 保持禁用。
  if (!core::same_time(self.clock->now(), self.current_context().mode_since)) self.refresh_mode();
  return self.publisher.replay_tick();
}
core::Generation Pipeline::generation() const { return impl_->current_context().generation; }
PipelineMetrics Pipeline::metrics() const {
  auto result = impl_->metrics;
  result.queue_drops = impl_->queue.drops(); result.result_drops = impl_->admission.drops();
  return result;
}
core::Result<PipelineMetrics> Pipeline::run_file() {
  using Result = core::Result<PipelineMetrics>;
  auto& self = *impl_;
  auto* replay_clock = dynamic_cast<hal::ReplayClock*>(self.clock.get());
  if (!replay_clock || self.started || self.closed)
    return Result::failure(core::ErrorCode::invalid_input, "File replay needs a new pipeline with owned replay clock");
  try {
    hal::FileReplay input(self.config.input_manifest, *replay_clock);
    start();
    std::promise<void> done;
    auto completion = done.get_future();
    self.scheduler.start({[&](const std::atomic<bool>& stop) {
      try {
        while (!stop.load()) {
          auto event = input.next(generation());
          if (!event) throw std::runtime_error(event.error().message);
          if (std::holds_alternative<hal::ReplayEnd>(event.value())) break;
          if (const auto* sample = std::get_if<hal::GimbalFeedback>(&event.value())) feedback(*sample);
          else {
            frame(*std::get<std::shared_ptr<const core::CapturedFrame>>(event.value()));
            // 正确性回放统一串行屏障，避免 CPU 完成速度改变事件顺序。
            // 实时异步入口仍为 frame/process_pending，使用相同 consume/decide 链。
            finish_pending(core::Seconds(30));
          }
          if (!replay_tick()) throw std::runtime_error("Replay publish barrier failed");
        }
        done.set_value();
      } catch (...) { done.set_exception(std::current_exception()); throw; }
    }}, [&] { self.fail(self.scheduler.failure()); });
    completion.get();
    close();
    if (const auto failure = self.publisher.status().first_failure) std::rethrow_exception(failure);
    return Result::success(metrics());
  } catch (const std::exception& error) {
    close();
    return Result::failure(core::ErrorCode::fault, error.what());
  }
}
}  // namespace autoaim::pipeline
