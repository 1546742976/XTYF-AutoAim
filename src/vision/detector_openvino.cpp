#include "autoaim/vision/detector.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <opencv2/imgproc.hpp>
#ifdef AUTOAIM_HAS_OPENVINO
#include <openvino/openvino.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#endif

namespace autoaim::vision {
namespace {
void validate(const Yolov5Options& options) {
  if ((options.format != YoloFormat::legacy_v5 && options.format != YoloFormat::armor_v11) ||
      (options.enemy != TeamColor::red && options.enemy != TeamColor::blue) ||
      !std::isfinite(options.confidence_threshold) || options.confidence_threshold <= 0 ||
      options.confidence_threshold >= 1 || !std::isfinite(options.nms_iou_threshold) ||
      options.nms_iou_threshold <= 0 || options.nms_iou_threshold >= 1 ||
      options.maximum_candidates == 0 || options.maximum_detections == 0 ||
      options.maximum_detections > options.maximum_candidates)
      throw std::invalid_argument("Invalid YOLO limits");

  if (options.format == YoloFormat::armor_v11 && !options.plate_type_by_class.empty())
    throw std::invalid_argument("YOLO11 uses its own size labels, not YOLOv5 overrides");

  for (const auto& mapping : options.plate_type_by_class)
    if (mapping.first < 0 || mapping.first >= 8 ||
        (mapping.second != ArmorSize::small && mapping.second != ArmorSize::big))
      throw std::invalid_argument("Invalid explicit YOLOv5 plate type mapping");
}

double sigmoid(double x) {
  return x >= 0 ? 1 / (1 + std::exp(-x)) : std::exp(x) / (1 + std::exp(x));
}

cv::Rect2f bounds(const Detection& detection) {
  float left = detection.corners[0].x, right = left;
  float top = detection.corners[0].y, bottom = top;

  for (const auto& p : detection.corners) {
    left = std::min(left, p.x);
    right = std::max(right, p.x);
    top = std::min(top, p.y);
    bottom = std::max(bottom, p.y);
  }

  return {left, top, right - left, bottom - top};
}
} // namespace

std::vector<Detection> parse_yolov5(const cv::Mat& output, double scale, cv::Size size,
                                    const Yolov5Options& options) {
  validate(options);

  if (options.format != YoloFormat::legacy_v5 || output.dims != 2 ||
      output.type() != CV_32FC1 || output.cols != 22 || !std::isfinite(scale) ||
      scale <= 0 || size.width <= 0 || size.height <= 0)
    throw std::invalid_argument("YOLOv5 requires a float32 N x 22 output and valid resize");

  std::vector<Detection> candidates;
  const int enemy_index = options.enemy == TeamColor::blue ? 0 : 1;

  for (int row = 0; row < output.rows; ++row) {
    const auto* values = output.ptr<float>(row);

    if (!std::all_of(values, values + 22, [](float v) {
          return std::isfinite(v);
        }))
      continue;

    const auto confidence = sigmoid(values[8]);

    if (confidence < options.confidence_threshold)
      continue;

    const auto color = std::max_element(values + 9, values + 13) - (values + 9);
    const auto id = std::max_element(values + 13, values + 22) - (values + 13);

    if (color != enemy_index || id == 8)
      continue;

    Detection detection{
        {}, static_cast<float>(confidence), options.enemy, static_cast<int>(id), false};
    detection.category = yolov5_label(int(color), int(id)).category;
    const auto type = options.plate_type_by_class.find(int(id));

    if (type != options.plate_type_by_class.end())
      detection.armor_size = type->second;

    constexpr int mapping[] = {0, 3, 2, 1};
    bool in_image = true;

    for (int k = 0; k < 4; ++k) {
      const double x = values[2 * mapping[k]] / scale;
      const double y = values[2 * mapping[k] + 1] / scale;
      in_image = in_image && x >= 0 && x < size.width && y >= 0 && y < size.height;
      detection.corners[k] = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
    }

    const auto box = bounds(detection);

    if (!in_image || box.width <= 0 || box.height <= 0)
      continue;

    candidates.push_back(detection);
  }

  // 只在有限高分候选上做二次 NMS，避免低分输出带来二次复杂度。
  const auto higher = [](const Detection& a, const Detection& b) {
    return a.confidence > b.confidence;
  };

  if (candidates.size() > options.maximum_candidates) {
    std::partial_sort(candidates.begin(), candidates.begin() + options.maximum_candidates,
                      candidates.end(), higher);

    candidates.resize(options.maximum_candidates);
  } else {
    std::stable_sort(candidates.begin(), candidates.end(), higher);
  }

  std::vector<Detection> selected;

  for (const auto& candidate : candidates) {
    const auto box = bounds(candidate);
    bool suppressed = false;

    for (const auto& accepted : selected) {
      const auto other = bounds(accepted);
      const double intersection = (box & other).area();
      const double union_area = box.area() + other.area() - intersection;

      if (intersection / union_area > options.nms_iou_threshold) {
        suppressed = true;
        break;
      }
    }

    if (!suppressed)
      selected.push_back(candidate);

    if (selected.size() == options.maximum_detections)
      break;
  }

  return selected;
}

std::vector<Detection> parse_yolo11(const cv::Mat& output, double scale, cv::Size size,
                                   const YoloOptions& options) {
  validate(options);

  if (options.format != YoloFormat::armor_v11 || output.dims != 2 ||
      output.type() != CV_32FC1 || output.rows != 50 || !std::isfinite(scale) || scale <= 0 ||
      size.width <= 0 || size.height <= 0)
    throw std::invalid_argument("Armor YOLO11 requires float32 50 x N output");

  struct Candidate {
    Detection detection;
    cv::Rect2f box;
  };
  std::vector<Candidate> candidates;

  for (int col = 0; col < output.cols; ++col) {
    std::array<float, 50> values;

    for (int row = 0; row < 50; ++row)
      values[row] = output.at<float>(row, col);
    if (!std::all_of(values.begin(), values.end(), [](float value) {
          return std::isfinite(value);
        }))
      continue;

    const auto score = std::max_element(values.begin() + 4, values.begin() + 42);
    const int id = int(score - values.begin() - 4);
    const auto label = yolo11_label(id);

    if (*score < options.confidence_threshold || *score > 1 || label.color != options.enemy ||
        values[2] <= 0 || values[3] <= 0)
      continue;

    Detection detection{{}, *score, label.color, id, false, label.category, label.size};
    bool inside = true;

    for (int k = 0; k < 4; ++k) {
      const double x = values[42 + 2 * k] / scale;
      const double y = values[43 + 2 * k] / scale;
      inside = inside && x >= 0 && y >= 0 && x < size.width && y < size.height;
      detection.corners[k] = {float(x), float(y)};
    }

    if (!inside)
      continue;

    const cv::Rect2f box(float((values[0] - values[2] / 2) / scale),
                         float((values[1] - values[3] / 2) / scale),
                         float(values[2] / scale), float(values[3] / scale));

    if (!std::isfinite(box.area()) || box.area() <= 0)
      continue;
    candidates.push_back({detection, box});
  }

  std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.detection.confidence > b.detection.confidence;
  });
  if (candidates.size() > options.maximum_candidates)
    candidates.resize(options.maximum_candidates);

  std::vector<Detection> selected;
  std::vector<cv::Rect2f> boxes;

  for (const auto& candidate : candidates) {
    bool suppressed = false;

    for (const auto& box : boxes) {
      const double intersection = (box & candidate.box).area();
      const double united = box.area() + candidate.box.area() - intersection;

      if (united > 0 && intersection / united > options.nms_iou_threshold) {
        suppressed = true;
        break;
      }
    }

    if (!suppressed) {
      boxes.push_back(candidate.box);
      selected.push_back(candidate.detection);
    }
    if (selected.size() == options.maximum_detections)
      break;
  }

  return selected;
}

class OpenVinoDetector::Impl {
public:
  explicit Impl(Yolov5Options supplied, std::size_t capacity) : options(std::move(supplied)) {
    validate(options);

    if (options.model_path.empty() || options.device.empty() || capacity == 0)
      throw std::invalid_argument("OpenVINO model path and device must be explicit");
#ifdef AUTOAIM_HAS_OPENVINO
    auto model = core.read_model(options.model_path);

    if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
        model->input().get_partial_shape() != ov::PartialShape({1, 3, 640, 640}))
      throw std::invalid_argument("Unsupported armor YOLO input layout");

    const auto shape = model->output().get_partial_shape();

    if (shape.is_dynamic() || shape.rank().get_length() != 3 || shape[0] != 1)
      throw std::invalid_argument("Unsupported armor YOLO output rank");

    if ((options.format == YoloFormat::legacy_v5 && shape[2] != 22) ||
        (options.format == YoloFormat::armor_v11 &&
         shape != ov::PartialShape({1, 50, 8400})))
      throw std::invalid_argument("Output layout does not match selected YOLO format");

    ov::preprocess::PrePostProcessor preprocessing(model);
    preprocessing.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, 640, 640, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    preprocessing.input().model().set_layout("NCHW");
    preprocessing.input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale(255.0f);

    preprocessing.output().tensor().set_element_type(ov::element::f32);
    compiled = core.compile_model(preprocessing.build(), options.device,
                                  ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

    slots.reserve(capacity);

    for (std::size_t i = 0; i < capacity; ++i)
      slots.push_back({compiled.create_infer_request(), cv::Mat(640, 640, CV_8UC3), nullptr, 1});
#else
    throw std::runtime_error("OpenVINO backend was disabled at build time");
#endif
  }

  Yolov5Options options;
  bool closed = false;
#ifdef AUTOAIM_HAS_OPENVINO
  struct Slot {
    ov::InferRequest request;
    cv::Mat input;
    std::shared_ptr<const FramePacket> packet;
    double scale;
    double preprocessing_ms = 0;
    std::chrono::steady_clock::time_point started{};
  };

  ov::Core core;
  ov::CompiledModel compiled;
  std::vector<Slot> slots;

  void prepare(Slot& slot, const FramePacket& packet) {
    const auto begin = std::chrono::steady_clock::now();
    const auto& image = packet.frame.image;
    slot.scale = std::min(640.0 / image.width, 640.0 / image.height);
    const int width = std::max(1, static_cast<int>(image.width * slot.scale));
    const int height = std::max(1, static_cast<int>(image.height * slot.scale));
    cv::Mat source(image.height, image.width, CV_8UC3,
                   const_cast<std::uint8_t*>(image.pixels->data()), image.stride);

    slot.input.setTo(cv::Scalar::all(0));
    cv::resize(source, slot.input(cv::Rect(0, 0, width, height)), {width, height});
    slot.request.set_input_tensor(ov::Tensor(ov::element::u8, {1, 640, 640, 3}, slot.input.data));
    slot.preprocessing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
  }

  DetectionBatch result(Slot& slot, const FramePacket& packet) {
    const auto begin = std::chrono::steady_clock::now();
    const double inference_ms =
        std::chrono::duration<double, std::milli>(begin - slot.started).count();
    auto output = slot.request.get_output_tensor();
    const auto shape = output.get_shape();
    cv::Mat matrix(static_cast<int>(shape[1]), static_cast<int>(shape[2]), CV_32F,
                   output.data<float>());
    const cv::Size size(packet.frame.image.width, packet.frame.image.height);
    auto detections = options.format == YoloFormat::legacy_v5
                          ? parse_yolov5(matrix, slot.scale, size, options)
                          : parse_yolo11(matrix, slot.scale, size, options);

    const double post_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();

    return DetectionBatch(packet.frame.stamp, std::move(detections),
                           {true, slot.preprocessing_ms, inference_ms, post_ms});
  }
#endif
};

OpenVinoDetector::OpenVinoDetector(Yolov5Options options, std::size_t capacity)
    : impl_(std::make_unique<Impl>(std::move(options), capacity)) {
}

OpenVinoDetector::~OpenVinoDetector() {
  close();
}

DetectionBatch OpenVinoDetector::detect(const FramePacket& packet) {
#ifdef AUTOAIM_HAS_OPENVINO
  if (impl_->closed || in_flight())
    throw std::logic_error("Detector closed or asynchronous requests active");

  auto& slot = impl_->slots.front();
  impl_->prepare(slot, packet);
  slot.started = std::chrono::steady_clock::now();
  slot.request.infer();

  return impl_->result(slot, packet);
#else
  (void)packet;
  throw std::runtime_error("OpenVINO backend was disabled at build time");
#endif
}

bool OpenVinoDetector::try_submit(std::shared_ptr<const FramePacket> packet) {
  if (!packet)
    throw std::invalid_argument("Null inference input");
#ifdef AUTOAIM_HAS_OPENVINO
  if (impl_->closed)
    return false;

  for (auto& slot : impl_->slots) {
    if (slot.packet)
      continue;

    impl_->prepare(slot, *packet);
    slot.packet = std::move(packet);
    slot.started = std::chrono::steady_clock::now();

    try {
      slot.request.start_async();
    } catch (...) {
      // start_async 抛出时也先确认请求退出，不能以抛出异常推断缓冲已不在用。
      try {
        slot.request.wait();
      } catch (...) {
      }

      slot.packet.reset();
      throw;
    }

    return true;
  }
#endif
  return false;
}

std::optional<OpenVinoDetector::Completion> OpenVinoDetector::take_completed() {
#ifdef AUTOAIM_HAS_OPENVINO
  for (auto& slot : impl_->slots) {
    if (!slot.packet)
      continue;

    try {
      // 跳过未完成槽，不等待更早的慢帧；过期/世代/顺序归 pipeline 的接纳点。
      if (!slot.request.wait_for(std::chrono::milliseconds(0)))
        continue;

      auto batch = impl_->result(slot, *slot.packet);
      auto packet = std::move(slot.packet);

      return Completion{std::move(packet), std::move(batch), nullptr};
    } catch (...) {
      const auto failure = std::current_exception();

      // 查询/读取输出异常也先等待实际请求退出，不把异常当作输入已归还的证明。
      try {
        slot.request.wait();
      } catch (...) {
      }

      auto packet = std::move(slot.packet);

      return Completion{std::move(packet), std::nullopt, failure};
    }
  }
#endif
  return std::nullopt;
}

std::size_t OpenVinoDetector::in_flight() const noexcept {
#ifdef AUTOAIM_HAS_OPENVINO
  return std::count_if(impl_->slots.begin(), impl_->slots.end(), [](const auto& slot) {
    return bool(slot.packet);
  });
#else
  return 0;
#endif
}

std::size_t OpenVinoDetector::prepared_bytes() const noexcept {
#ifdef AUTOAIM_HAS_OPENVINO
  return impl_->slots.size() * 640 * 640 * 3;
#else
  return 0;
#endif
}

void OpenVinoDetector::close() noexcept {
  impl_->closed = true;
#ifdef AUTOAIM_HAS_OPENVINO
  for (auto& slot : impl_->slots) {
    if (!slot.packet)
      continue;

    try {
      slot.request.wait();
    } catch (...) {
    }

    slot.packet.reset();
  }
#endif
}
} // namespace autoaim::vision
