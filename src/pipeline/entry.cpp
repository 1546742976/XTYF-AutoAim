#include "autoaim/pipeline/entry.hpp"
#include "autoaim/pipeline/pipeline.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace autoaim::pipeline {
int run_entry(int argc, char** argv, std::optional<core::Role> required_role) {
  try {
    std::filesystem::path config_path, input_override, output_path;
    for (int i = 1; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--help") {
        std::cout << "Usage: " << argv[0] << " --config FILE [--input EVENTS.yaml] [--output NEW.tsv]\n"
          "Offline only. Missing output writes TSV to stdout. No camera/serial/CAN connection.\n";
        return 0;
      }
      if ((option != "--config" && option != "--input" && option != "--output") || i + 1 == argc)
        throw std::invalid_argument("Unknown or incomplete argument: " + option);
      const auto value = std::filesystem::path(argv[++i]);
      if (option == "--config") config_path = value;
      else if (option == "--input") input_override = value;
      else output_path = value;
    }
    if (config_path.empty()) throw std::invalid_argument("--config is required");
    auto loaded = load_pipeline_config(config_path);
    if (!loaded) throw std::invalid_argument(loaded.error().message);
    auto config = std::move(loaded).value();
    if (required_role && config.role != *required_role) throw std::invalid_argument("Configuration role differs from thin entry role");
    if (!input_override.empty()) config.input_manifest = input_override;
    std::ofstream file;
    std::ostream* output = &std::cout;
    if (!output_path.empty()) {
      if (std::filesystem::exists(output_path)) throw std::invalid_argument("Refusing to overwrite existing output");
      file.open(output_path);
      if (!file) throw std::runtime_error("Cannot create output file; parent directory must exist");
      output = &file;
    }
    *output << "write_ns\tframe_id\tgeneration\texposure_ns\tcontrol\tshoot\tspace\tyaw_rad\tpitch_rad\tdistance_m\tage_s\n";
    auto clock = std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
    auto* time = clock.get();
    Pipeline pipeline(std::move(config), std::move(clock), [&](const control::Command& command) {
      std::ostringstream line;
      const auto now = time->now();
      line << std::setprecision(17) << now.nanoseconds() << '\t' << (command.source ? command.source->frame_id : 0) << '\t'
        << (command.source ? command.source->generation : 0) << '\t' << (command.source ? command.source->exposure.nanoseconds() : 0)
        << '\t' << command.control_enabled << '\t' << command.shoot << '\t'
        << (command.space == core::CommandSpace::relative ? "relative" : "absolute") << '\t'
        << command.pointing.yaw.value() << '\t' << command.pointing.pitch.value() << '\t' << command.horizontal_distance.value() << '\t';
      if (command.source) line << core::elapsed(now, command.source->exposure).value(); else line << "NA";
      line << '\n';
      const auto text = line.str();
      *output << text; output->flush();
      return hal::WriteResult{*output ? hal::WriteStatus::complete : hal::WriteStatus::failed,
        *output ? text.size() : 0, *output ? "" : "Offline command recording failed"};
    }, control::PublishSchedule::replay_events);
    auto result = pipeline.run_file();
    if (!result) throw std::runtime_error(result.error().message);
    const auto& m = result.value();
    const auto status = pipeline.publisher_status();
    std::cerr << "replay frames=" << m.received_frames << " accepted=" << m.accepted_results << " detections=" << m.detections
      << " valid_poses=" << m.valid_poses << " observations=" << m.observations << " decisions=" << m.decisions
      << " intents=" << m.submitted_intents << " no_solution=" << m.no_solution << " writes=" << status.successful_writes
      << " first_control_writes=" << status.first_writes << " stop_written=" << status.stop_written
      << " device_stop_confirmed=" << status.device_stop_confirmed << '\n';
    std::cerr << "logical_observation_age_mean_s=" << (m.accepted_results ? m.total_observation_age_s / m.accepted_results : 0)
      << " logical_observation_age_max_s=" << m.maximum_observation_age_s << " last_inhibit_mask=" << status.last_inhibit_reasons;
    if (status.last_first_send_age) std::cerr << " last_first_control_write_age_s=" << status.last_first_send_age->value();
    else std::cerr << " last_first_control_write_age_s=NA";
    const auto expired = static_cast<std::size_t>(DropReason::expired);
    std::cerr << " expired_input_ratio=" << (m.received_frames ?
      double(m.queue_drops[expired] + m.result_drops[expired]) / m.received_frames : 0) << '\n';
    constexpr const char* reasons[] = {"invalid", "expired", "old_generation", "capacity", "no_buffer", "out_of_order", "closed"};
    for (std::size_t i = 0; i < m.queue_drops.size(); ++i)
      std::cerr << "drop_reason=" << reasons[i] << " queue=" << m.queue_drops[i] << " result=" << m.result_drops[i] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "autoaim: " << error.what() << '\n'; return 1;
  }
}
}  // namespace autoaim::pipeline
