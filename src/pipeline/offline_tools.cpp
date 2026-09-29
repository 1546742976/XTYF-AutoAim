#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/pipeline/bootstrap.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace autoaim::pipeline {
int run_detector_benchmark(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--dataset")
      return run_batch_benchmark(argc, argv);

  try {
    std::filesystem::path config_path, image_path;
    std::size_t iterations = 100;
    bool self_test = false;

    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);

      if (argument == "--help") {
        std::cout << "bench_detector --config FILE --image FILE [--iterations N]\nLocal detector "
                     "wall-time only; not NUC or full-pipeline FPS.\n"
                     "Batch: --dataset EVENTS --config A [--config B] --iou 0.5 --output NEW_DIR\n";

        return 0;
      }

      if (argument == "--self-test") {
        self_test = true;
        iterations = 3;
        continue;
      }

      if (++i == argc)
        throw std::invalid_argument("Missing argument");

      if (argument == "--config")
        config_path = argv[i];
      else if (argument == "--image")
        image_path = argv[i];
      else if (argument == "--iterations")
        iterations = std::stoul(argv[i]);
      else
        throw std::invalid_argument("Unknown argument");
    }

    if (!iterations || iterations > 10000)
      throw std::invalid_argument("Iterations must be 1..10000");

    auto loaded = load_pipeline_config(config_path);

    if (!loaded)
      throw std::invalid_argument(loaded.error().message);

    const auto& config = loaded.value();
    cv::Mat image = self_test ? cv::Mat(config.calibration.height(), config.calibration.width(),
                                        CV_8UC3, cv::Scalar(0, 0, 0))
                              : cv::imread(image_path.string(), cv::IMREAD_COLOR);

    if (image.empty() || image.cols != config.calibration.width() ||
        image.rows != config.calibration.height())
      throw std::invalid_argument("Image missing or differs from configured dimensions");

    auto pixels = std::make_shared<std::vector<std::uint8_t>>(image.total() * 3);

    for (int row = 0; row < image.rows; ++row)
      std::memcpy(pixels->data() + row * image.cols * 3, image.ptr(row), image.cols * 3);

    const core::TimePoint time(1, core::ClockDomain::replay);
    const vision::FramePacket packet(
        core::CapturedFrame(
            core::Stamp(1, 1, time), core::Image(image.cols, image.rows, image.cols * 3, pixels),
            time, core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing()),
        std::nullopt);

    const auto start = std::chrono::steady_clock::now();
    auto detector = vision::make_detector(config.detector);
    const auto load_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    detector->detect(packet); // 一次预热不混入稳态样本；模型加载单独报告。
    std::vector<double> milliseconds;
    std::size_t detections = 0;

    for (std::size_t i = 0; i < iterations; ++i) {
      const auto begin = std::chrono::steady_clock::now();
      const auto result = detector->detect(packet);
      milliseconds.push_back(
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count());

      detections += result.detections.size();
    }

    std::sort(milliseconds.begin(), milliseconds.end());
    const auto percentile = [&](double q) {
      return milliseconds.at(static_cast<std::size_t>(std::ceil(q * milliseconds.size())) - 1);
    };

    std::cout << "local_detector_only iterations=" << iterations << " model_load_ms=" << load_ms
              << " p50_ms=" << percentile(0.5) << " p95_ms=" << percentile(0.95)
              << " max_ms=" << milliseconds.back() << " detection_count=" << detections << '\n';

    if (self_test && detections)
      throw std::runtime_error("Blank fixture unexpectedly detected armor");

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';

    return 1;
  }
}
} // namespace autoaim::pipeline
