#include "autoaim/pipeline/pipeline.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

int main(int argc, char** argv) {
  using namespace autoaim;
  try {
    std::filesystem::path config_path, input, output;
    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);
      if (argument == "--help") { std::cout << "replay_visualizer --config FILE [--input EVENTS] --output NEW_DIR\nSame processing chain; offline annotated PNGs only.\n"; return 0; }
      if (++i == argc) throw std::invalid_argument("Missing argument");
      if (argument == "--config") config_path = argv[i];
      else if (argument == "--input") input = argv[i];
      else if (argument == "--output") output = argv[i];
      else throw std::invalid_argument("Unknown argument");
    }
    auto loaded = pipeline::load_pipeline_config(config_path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    auto config = std::move(loaded).value();
    if (!input.empty()) config.input_manifest = input;
    if (output.empty() || std::filesystem::exists(output)) throw std::invalid_argument("Output must be a new directory");
    std::filesystem::create_directories(output);
    std::size_t annotated = 0;
    pipeline::Pipeline pipeline(std::move(config), std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay)),
      [](const control::Command&) { return hal::WriteResult{hal::WriteStatus::complete, 1, {}}; },
      control::PublishSchedule::replay_events,
      [&](const vision::FramePacket& packet, const vision::DetectionBatch& batch, const estimation::ObservationBatch& observations) {
        const auto& source = packet.frame;
        cv::Mat image = cv::Mat(source.image.height, source.image.width, CV_8UC3,
          const_cast<std::uint8_t*>(source.image.pixels->data()), source.image.stride).clone();
        for (const auto& detection : batch.detections) {
          for (int k = 0; k < 4; ++k) cv::line(image, detection.corners[k], detection.corners[(k + 1) % 4], cv::Scalar(120, 120, 120), 1);
        }
        for (const auto& observation : observations) {
          const auto color = observation->reliable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255);
          for (int k = 0; k < 4; ++k) {
            cv::circle(image, observation->detection.corners[k], 2, color, -1);
            cv::putText(image, std::to_string(k), observation->detection.corners[k], cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1);
          }
        }
        const std::string label = "frame=" + std::to_string(source.stamp.frame_id) + " epoch=" + std::to_string(source.stamp.generation) +
          " detections=" + std::to_string(batch.detections.size()) + " poses=" + std::to_string(observations.size());
        cv::putText(image, label, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 255, 255}, 1);
        cv::putText(image, "gray: detector  orange: valid/unverified  green: reliable", {8, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.35, {255, 255, 255}, 1);
        const auto filename = std::to_string(source.stamp.generation) + "_" + std::to_string(source.stamp.frame_id) + ".png";
        if (!cv::imwrite((output / filename).string(), image)) throw std::runtime_error("Annotation write failed");
        ++annotated;
      });
    const auto result = pipeline.run_file();
    if (!result) throw std::runtime_error(result.error().message);
    std::cout << "annotated=" << annotated << " accepted=" << result.value().accepted_results << '\n'; return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
