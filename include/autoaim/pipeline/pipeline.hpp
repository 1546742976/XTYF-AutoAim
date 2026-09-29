#pragma once

#include "autoaim/pipeline/bootstrap.hpp"
#include "autoaim/control/publisher.hpp"
#include "autoaim/hal/session_writer.hpp"
#include "autoaim/vision/evaluation.hpp"

namespace autoaim::pipeline {
struct PipelineMetrics {
  std::uint64_t received_frames = 0;
  std::uint64_t accepted_results = 0;
  std::uint64_t detections = 0;
  std::uint64_t valid_poses = 0;
  std::uint64_t observations = 0;
  std::uint64_t decisions = 0;
  std::uint64_t submitted_intents = 0;
  std::uint64_t no_solution = 0;
  std::optional<double> last_observation_age_s;
  double maximum_observation_age_s = 0;
  double total_observation_age_s = 0;
  DropCounts queue_drops{};
  DropCounts result_drops{};
  QueueTimings queue_timings;
  WallTimeSummary preprocessing;
  WallTimeSummary inference_wall;
  WallTimeSummary postprocessing;
};

// pipeline 持有实例并协调生命周期；scheduler 管工作线程，publisher 是唯一写者。
// start/close 由同一生命周期线程串行调用；外部 write 回调的资源须覆盖本对象。
// 手工调用数据接口时，调用者须先停止并 join 外部处理线程，再 close；run_file 自管调度线程。
class Pipeline {
public:
  // 可选只读离线观察器：引用仅在回调内有效，不得修改像素或重入管线。
  using Observer = std::function<void(const vision::FramePacket&, const vision::DetectionBatch&,
                                      const estimation::ObservationBatch&)>;
  // 评测旁路包含被接纳点拒绝的已完成检测；不持有图像，不向运行链反馈真值。
  using EvaluationObserver = std::function<void(const vision::FramePacket&,
      const std::vector<vision::EvaluatedDetection>&, vision::EvaluationFrameState)>;

  Pipeline(PipelineConfig config, std::unique_ptr<hal::Clock> clock,
           control::Publisher::Write write, control::PublishSchedule schedule,
           Observer observer = {}, EvaluationObserver evaluation_observer = {});

  ~Pipeline();
  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;
  void start();
  void close();

  // 以下数据处理方法由唯一处理线程串行调用；已发布输入不可修改或重打时间戳。
  void feedback(hal::GimbalFeedback sample);
  void uart(const hal::UartChunk& chunk);
  void button(const hal::ButtonSample& sample);
  bool frame(const core::CapturedFrame& frame);
  void process_pending();

  // 回放屏障冻结逻辑时钟，只等实际推理结束；不是实机时延测量。
  void finish_pending(core::Seconds wall_timeout);
  bool replay_tick();
  core::Generation generation() const;
  PipelineMetrics metrics() const; // 处理线程或停止后读取。
  // 独占运行整个文件，内部 start/close；逻辑时钟冻结于每次推理/发布事件屏障。
  // 可选记录器由调用者持有至返回，记录失败按管线故障处理。
  core::Result<PipelineMetrics> run_file(hal::SessionWriter* recording = nullptr);
  control::PublisherStatus publisher_status() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace autoaim::pipeline
