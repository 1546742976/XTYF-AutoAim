#pragma once

#include "autoaim/core/time.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoaim::core {
enum class EvidenceLevel { missing, declared, measured, simulation };

struct MeasurementProvenance {
  std::string device_id;
  std::string configuration_id;
  std::string date;
  std::string method;
};

// 报告只由测量样本评估产生，不能由 calibrated=true 构造。
// 样本真实性仍需人工审查；此类型不是密码学认证或设备执行回执。
class MeasurementReport {
public:
  static MeasurementReport evaluate(MeasurementProvenance provenance,
                                    const std::vector<double>& residuals, double absolute_limit,
                                    ClockDomain domain) {
    if (provenance.device_id.empty() || provenance.configuration_id.empty() ||
        provenance.date.empty() || provenance.method.empty() || residuals.size() < 2 ||
        !std::isfinite(absolute_limit) || absolute_limit < 0 ||
        (domain != ClockDomain::host_monotonic && domain != ClockDomain::replay))
      throw std::invalid_argument("Incomplete measurement report");

    const bool passed = std::all_of(residuals.begin(), residuals.end(), [&](double residual) {
      return std::isfinite(residual) && std::abs(residual) <= absolute_limit;
    });

    return MeasurementReport(std::move(provenance), passed, domain);
  }

  const MeasurementProvenance& provenance() const noexcept {
    return provenance_;
  }

  bool passed() const noexcept {
    return passed_;
  }

  ClockDomain domain() const noexcept {
    return domain_;
  }

private:
  MeasurementReport(MeasurementProvenance provenance, bool passed, ClockDomain domain)
      : provenance_(std::move(provenance)), passed_(passed), domain_(domain) {
  }

  MeasurementProvenance provenance_;
  bool passed_;
  ClockDomain domain_;
};

class Evidence {
public:
  static Evidence missing() {
    return Evidence(EvidenceLevel::missing, {}, {});
  }

  static Evidence declared() {
    return Evidence(EvidenceLevel::declared, {}, {});
  }

  static Evidence from_report(const MeasurementReport& report) {
    if (!report.passed())
      return missing();

    return Evidence(report.domain() == ClockDomain::replay ? EvidenceLevel::simulation
                                                           : EvidenceLevel::measured,
                    report.provenance().device_id, report.provenance().configuration_id,
                    std::make_shared<const MeasurementReport>(report));
  }

  EvidenceLevel level() const noexcept {
    return level_;
  }

  const MeasurementReport* report() const noexcept {
    return report_.get();
  }

  bool qualifies(ClockDomain domain, const std::string& device,
                 const std::string& configuration) const noexcept {
    if (domain != ClockDomain::host_monotonic && domain != ClockDomain::replay)
      return false;

    const auto required =
        domain == ClockDomain::replay ? EvidenceLevel::simulation : EvidenceLevel::measured;

    return !device.empty() && !configuration.empty() && level_ == required && device_ == device &&
           configuration_ == configuration;
  }

private:
  Evidence(EvidenceLevel level, std::string device, std::string configuration,
           std::shared_ptr<const MeasurementReport> report = {})
      : level_(level), device_(std::move(device)), configuration_(std::move(configuration)),
        report_(std::move(report)) {
  }

  EvidenceLevel level_;
  std::string device_;
  std::string configuration_;
  std::shared_ptr<const MeasurementReport> report_;
};
} // namespace autoaim::core
