#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

namespace {
constexpr const char* header = "write_ns\tframe_id\tgeneration\texposure_ns\tcontrol\tshoot\tspace\tyaw_rad\tpitch_rad\tdistance_m\tage_s";
struct Metrics {
  std::size_t rows = 0, shots = 0, expired = 0;
  std::vector<double> ages, first_control_ages;
};
Metrics read_metrics(std::istream& input, double fire_age) {
  if (!std::isfinite(fire_age) || fire_age <= 0) throw std::invalid_argument("Positive finite fire age required");
  std::string line;
  if (!std::getline(input, line) || line != header) throw std::invalid_argument("Unknown command TSV schema");
  Metrics result;
  std::set<std::pair<std::uint64_t, std::uint64_t>> sent;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    std::int64_t at, exposure;
    std::uint64_t frame, generation;
    int control, shoot;
    std::string space, age_text;
    double yaw, pitch, distance;
    if (!(row >> at >> frame >> generation >> exposure >> control >> shoot >> space >> yaw >> pitch >> distance >> age_text) ||
        (control != 0 && control != 1) || (shoot != 0 && shoot != 1) || shoot > control ||
        (space != "absolute" && space != "relative") || !std::isfinite(yaw) || !std::isfinite(pitch) ||
        !std::isfinite(distance) || distance < 0 || at < exposure)
      throw std::invalid_argument("Invalid command TSV row");
    std::string extra; if (row >> extra) throw std::invalid_argument("Extra command TSV column");
    ++result.rows; result.shots += shoot;
    if (!frame) {
      if (age_text != "NA" || control || shoot) throw std::invalid_argument("Sourceless control/age invalid");
      continue;
    }
    std::size_t end;
    const double age = std::stod(age_text, &end);
    if (end != age_text.size() || !std::isfinite(age) || age < 0 ||
        std::abs(age - (static_cast<long double>(at) - exposure) * 1e-9L) > 1e-8)
      throw std::invalid_argument("Source age differs from original exposure");
    result.ages.push_back(age);
    if (age >= fire_age) ++result.expired;
    if (control && sent.emplace(generation, frame).second) result.first_control_ages.push_back(age);
  }
  return result;
}
void summarize(const char* name, std::vector<double> values) {
  std::cout << name << "_count=" << values.size();
  if (values.empty()) { std::cout << " p50_s=NA p95_s=NA max_s=NA\n"; return; }
  std::sort(values.begin(), values.end());
  const auto percentile = [&](double q) { return values.at(static_cast<std::size_t>(std::ceil(values.size() * q)) - 1); };
  std::cout << " p50_s=" << percentile(0.5) << " p95_s=" << percentile(0.95) << " max_s=" << values.back() << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      std::istringstream fixture(std::string(header) + "\n"
        "20000000\t1\t1\t10000000\t1\t0\trelative\t0\t0\t3\t0.01\n"
        "30000000\t1\t1\t10000000\t1\t0\trelative\t0\t0\t3\t0.02\n"
        "200000000\t2\t1\t50000000\t0\t0\tabsolute\t0\t0\t0\t0.15\n");
      const auto result = read_metrics(fixture, 0.1);
      if (result.rows != 3 || result.expired != 1 || result.first_control_ages.size() != 1 || result.first_control_ages[0] != 0.01)
        throw std::runtime_error("First-write/expiry accounting self-test failed");
      std::cout << "metrics self-test passed\n"; return 0;
    }
    if (argc != 4 || std::string(argv[2]) != "--fire-age-s") {
      std::cout << "pipeline_metrics COMMANDS.tsv --fire-age-s SECONDS\nTSV ages are replay logical-time measurements, not device execution delays.\n";
      return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 1;
    }
    std::ifstream input(argv[1]);
    const auto result = read_metrics(input, std::stod(argv[3]));
    std::cout << "written_rows=" << result.rows << " shoot_rows=" << result.shots << " stale_source_write_ratio="
      << (result.ages.empty() ? 0 : double(result.expired) / result.ages.size()) << '\n';
    summarize("source_age", result.ages);
    summarize("first_complete_control_write_age", result.first_control_ages);
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
