#pragma once

#include "autoaim/vision/detection.hpp"
#include "autoaim/vision/frame_packet.hpp"
#include <memory>
#include <string>
#include <exception>

namespace autoaim::vision {
class Detector {
public:
  virtual ~Detector() = default;
  // 实例不并发重入；异步调度为每个在途请求提供独立实例/请求槽。
  virtual DetectionBatch detect(const FramePacket& frame) = 0;
};
struct TraditionalOptions {
  TeamColor enemy;
  int brightness_threshold;
  int color_difference;
  double minimum_light_length;
  double minimum_light_ratio;
  double minimum_pair_ratio;
  double maximum_pair_ratio;
  std::size_t maximum_lightbars;
};
class TraditionalDetector final : public Detector {
public:
  explicit TraditionalDetector(TraditionalOptions options);
  DetectionBatch detect(const FramePacket& frame) override;
private:
  TraditionalOptions options_;
};

struct Yolov5Options {
  std::string model_path;
  std::string device;
  TeamColor enemy;
  double confidence_threshold;
  double nms_iou_threshold;
  std::size_t maximum_candidates;
  std::size_t maximum_detections;
};

// 旧模型的 22 列格式：8 个角点坐标、objectness logit、4 色、9 类。
// 输出为原图像素；兼容重排不等于标签语义已验证，corners_reliable 始终为 false。
std::vector<Detection> parse_yolov5(const cv::Mat& output, double resize_scale,
                                  cv::Size original_size, const Yolov5Options& options);

class OpenVinoDetector final : public Detector {
public:
  struct Completion {
    std::shared_ptr<const FramePacket> packet;
    std::optional<DetectionBatch> detections;
    std::exception_ptr failure;
  };
  explicit OpenVinoDetector(Yolov5Options options, std::size_t request_capacity = 1);
  ~OpenVinoDetector() override;
  DetectionBatch detect(const FramePacket& frame) override;
  // 以下方法由一个调度线程调用；OpenVINO 执行计算。无内部等待帧队列。
  // 满槽返回 false，已完成但尚未领取的请求仍占槽；原图与预处理输入都保留到结束。
  bool try_submit(std::shared_ptr<const FramePacket> frame);
  std::optional<Completion> take_completed();
  std::size_t in_flight() const noexcept;
  std::size_t prepared_bytes() const noexcept;
  // 终止接纳并等待实际请求完成，不假设 cancel 已释放输入；不得与上述方法并发。
  void close() noexcept;
private:
  // 每实例独占请求和预处理缓冲；输入在 infer 返回之前不会析构或复用。
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace autoaim::vision
