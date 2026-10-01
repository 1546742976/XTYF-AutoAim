#include "autoaim/pipeline/entry.hpp"
#include "autoaim/pipeline/pipeline.hpp"
#include "autoaim/pipeline/uart_writer.hpp"
#include "autoaim/hal/recording_transport.hpp"
#include "autoaim/pipeline/run_metadata.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace autoaim::pipeline {
int run_entry(int argc, char** argv, std::optional<core::Role> required_role) {
  try {
    std::filesystem::path config_path, input_override, output_path, session_path, uart_path;
    bool check_config = false;
    bool replay_options = false;

    for (int i = 1; i < argc; ++i) {
      const std::string option(argv[i]);

      if (option == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " --config FILE [--input EVENTS.yaml] [--output NEW.tsv]"
                     " [--record-session NEW_DIR] [--uart-output NEW.hex]\n"
                     "       --check-config --config FILE\n"
                     "Configuration check prints YAML without running the pipeline.\n"
                     "Offline only. Missing output writes TSV to stdout. No camera/serial/CAN "
                     "connection.\n";

        return 0;
      }

      if (option == "--check-config") {
        if (check_config)
          throw std::invalid_argument("Duplicate --check-config");
        check_config = true;
        continue;
      }

      if ((option != "--config" && option != "--input" && option != "--output" &&
           option != "--record-session" && option != "--uart-output") || i + 1 == argc)
        throw std::invalid_argument("Unknown or incomplete argument: " + option);

      const auto value = std::filesystem::path(argv[++i]);
      replay_options = replay_options || option != "--config";

      if (option == "--config")
        config_path = value;
      else if (option == "--input")
        input_override = value;
      else if (option == "--output")
        output_path = value;
      else if (option == "--uart-output")
        uart_path = value;
      else
        session_path = value;
    }

    if (config_path.empty())
      throw std::invalid_argument("--config is required");

    if (check_config && replay_options)
      throw std::invalid_argument(
          "--check-config cannot be combined with replay input/output options");

    auto loaded = load_pipeline_config(config_path);

    if (!loaded)
      throw std::invalid_argument(loaded.error().message);

    auto config = std::move(loaded).value();

    if (required_role && config.role != *required_role)
      throw std::invalid_argument("Configuration role differs from thin entry role");

    if (check_config) {
      const auto& snapshot = *config.configuration_snapshot;
      YAML::Node report;
      report["check_schema_version"] = 1;
      report["valid"] = true;
      report["configuration_file"] = snapshot.entry.path.string();
      report["base_configuration_file"] = snapshot.base.path.string();
      report["fast_choose_file"] = snapshot.fast_choose
          ? YAML::Node(snapshot.fast_choose->path.string()) : YAML::Node(YAML::NodeType::Null);
      report["effective_configuration"] = YAML::Clone(snapshot.effective);
      YAML::Emitter emitter;
      emitter.SetDoublePrecision(17);
      emitter << report;
      if (!emitter.good())
        throw std::runtime_error("Cannot serialize configuration check");
      std::cout << emitter.c_str() << '\n';
      std::cout.flush();
      if (!std::cout)
        throw std::runtime_error("Cannot write configuration check");
      return 0;
    }

    if (!input_override.empty())
      config.input_manifest = input_override;

    std::unique_ptr<hal::SessionWriter> recording;

    if (!session_path.empty()) {
      hal::ReplayClock inspect(core::TimePoint(0, core::ClockDomain::replay));
      hal::FileReplay source(config.input_manifest, inspect);
      recording = std::make_unique<hal::SessionWriter>(session_path, source.metadata());
      recording->derived(core::TimePoint(0, core::ClockDomain::replay), "run",
                           describe_run(config_path, config));
    }

    std::ofstream file;
    std::ostream* output = &std::cout;

    if (!output_path.empty()) {
      if (std::filesystem::exists(output_path))
        throw std::invalid_argument("Refusing to overwrite existing output");

      file.open(output_path);

      if (!file)
        throw std::runtime_error("Cannot create output file; parent directory must exist");

      output = &file;
    }

    *output << "write_ns\tframe_id\tgeneration\texposure_ns\tcontrol\tshoot\tspace\tyaw_rad\tpitch_"
               "rad\tdistance_m\tage_s\n";

    auto clock = std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
    auto* time = clock.get();
    std::ofstream uart_file;
    std::unique_ptr<hal::RecordingTransport> uart_recording;
    control::Publisher::Write wire_writer;

    if (!uart_path.empty()) {
      if (std::filesystem::exists(uart_path))
        throw std::invalid_argument("Refusing to overwrite UART recording");
      uart_file.open(uart_path);
      if (!uart_file)
        throw std::runtime_error("Cannot create UART recording");
      uart_recording = std::make_unique<hal::RecordingTransport>(uart_file, *time);
      wire_writer = make_uart14_writer(*uart_recording, core::Seconds(0.1));
    }

    Pipeline pipeline(
        std::move(config), std::move(clock),
        [&](const control::Command& command, const control::CommandMetadata& metadata) {
          std::ostringstream line;
          const auto now = time->now();
          line << std::setprecision(17) << now.nanoseconds() << '\t'
               << (metadata.source ? metadata.source->frame_id : 0) << '\t'
               << (metadata.source ? metadata.source->generation : 0) << '\t'
               << (metadata.source ? metadata.source->exposure.nanoseconds() : 0) << '\t'
               << command.control << '\t' << command.shoot << '\t'
               << (metadata.space == core::CommandSpace::relative ? "relative" : "absolute") << '\t'
               << command.yaw << '\t' << command.pitch << '\t' << command.horizon_distance << '\t';

          if (metadata.source)
            line << core::elapsed(now, metadata.source->exposure).value();
          else
            line << "NA";

          line << '\n';
          const auto text = line.str();
          *output << text;
          output->flush();
          const auto written = wire_writer ? wire_writer(command, metadata) :
              hal::WriteResult{hal::WriteStatus::complete, text.size(), {}};

          if (recording) {
            YAML::Node values;
            values["frame_id"] = metadata.source ? metadata.source->frame_id : 0;
            values["generation"] = metadata.source ? metadata.source->generation : 0;
            values["exposure_ns"] = metadata.source
                ? YAML::Node(metadata.source->exposure.nanoseconds())
                : YAML::Node(YAML::NodeType::Null);
            values["space"] =
                metadata.space == core::CommandSpace::relative ? "relative" : "absolute";
            values["control"] = command.control;
            values["shoot"] = command.shoot;
            values["yaw_rad"] = command.yaw;
            values["pitch_rad"] = command.pitch;
            values["distance_m"] = command.horizon_distance;
            values["inhibit_reasons"] = metadata.inhibit_reasons;
            values["stop"] = metadata.stop_record;
            values["write_complete"] = *output && written.status == hal::WriteStatus::complete;
            recording->derived(now, "command", values);
          }

          if (written.status == hal::WriteStatus::failed)
            return written;

          return hal::WriteResult{*output ? hal::WriteStatus::complete : hal::WriteStatus::failed,
                                  *output ? text.size() : 0,
                                  *output ? "" : "Offline command recording failed"};
        },
        control::PublishSchedule::replay_events);

    auto result = pipeline.run_file(recording.get());

    if (!result)
      throw std::runtime_error(result.error().message);

    if (recording)
      recording->finish();

    const auto& m = result.value();
    const auto status = pipeline.publisher_status();
    std::cerr << "replay frames=" << m.received_frames << " accepted=" << m.accepted_results
              << " detections=" << m.detections << " valid_poses=" << m.valid_poses
              << " observations=" << m.observations << " decisions=" << m.decisions
              << " intents=" << m.submitted_intents << " no_solution=" << m.no_solution
              << " writes=" << status.successful_writes
              << " first_control_writes=" << status.first_writes
              << " stop_written=" << status.stop_written
              << " device_stop_confirmed=" << status.device_stop_confirmed << '\n';

    std::cerr << "logical_observation_age_mean_s="
              << (m.accepted_results ? m.total_observation_age_s / m.accepted_results : 0)
              << " logical_observation_age_max_s=" << m.maximum_observation_age_s
              << " last_inhibit_mask=" << status.last_inhibit_reasons;

    if (status.last_first_send_age)
      std::cerr << " last_first_control_write_age_s=" << status.last_first_send_age->value();
    else
      std::cerr << " last_first_control_write_age_s=NA";

    const auto expired = static_cast<std::size_t>(DropReason::expired);
    std::cerr << " expired_input_ratio="
              << (m.received_frames
                      ? double(m.queue_drops[expired] + m.result_drops[expired]) / m.received_frames
                      : 0)
              << '\n';

    constexpr const char* reasons[] = {"invalid",   "expired",      "old_generation", "capacity",
                                       "no_buffer", "out_of_order", "closed"};

    for (std::size_t i = 0; i < m.queue_drops.size(); ++i)
      std::cerr << "drop_reason=" << reasons[i] << " queue=" << m.queue_drops[i]
                << " result=" << m.result_drops[i] << '\n';

    return 0;
  } catch (const std::exception& error) {
    std::cerr << "autoaim: " << error.what() << '\n';

    return 1;
  }
}
} // namespace autoaim::pipeline
