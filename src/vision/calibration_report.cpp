#include "autoaim/vision/calibration_report.hpp"
#include "calibration_quality.hpp"
#include <set>

namespace autoaim::vision {
namespace {
YAML::Node report_base(const CalibrationDataset& data, const IntrinsicSolution& intrinsic,
                       const core::MeasurementProvenance& provenance, core::ClockDomain domain) {
  if (provenance.device_id.empty() || provenance.configuration_id.empty() ||
      provenance.date.empty() || provenance.method.empty() ||
      (domain != core::ClockDomain::replay && domain != core::ClockDomain::host_monotonic) ||
      (data.simulated && domain != core::ClockDomain::replay))
    throw std::invalid_argument("Incomplete provenance or synthetic-to-measured promotion");
  YAML::Node report;
  report["schema_version"] = 1;
  report["source_kind"] = data.simulated ? "synthetic" : "recorded";
  report["domain"] = domain == core::ClockDomain::replay ? "replay" : "host_monotonic";
  report["provenance"]["device_id"] = provenance.device_id;
  report["provenance"]["configuration_id"] = provenance.configuration_id;
  report["provenance"]["date"] = provenance.date;
  report["provenance"]["method"] = provenance.method;
  report["parameters"]["width"] = intrinsic.image_size.width;
  report["parameters"]["height"] = intrinsic.image_size.height;
  report["parameters"]["roi_offset"] =
      std::vector<int>{intrinsic.roi_offset[0], intrinsic.roi_offset[1]};
  report["parameters"]["camera_matrix"] =
      std::vector<double>(intrinsic.matrix.val, intrinsic.matrix.val + 9);
  report["parameters"]["distortion"] = intrinsic.distortion;
  report["fit_ids"] = std::vector<std::string>{};
  report["validation_ids"] = std::vector<std::string>{};
  report["rejected"] = data.rejected;

  for (const auto& view : data.views)
    report[view.validation ? "validation_ids" : "fit_ids"].push_back(view.id);

  return report;
}

void limit(double value) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Calibration validation limit must be positive and finite");
}
} // namespace

YAML::Node intrinsic_report(const CalibrationDataset& data, const IntrinsicSolution& solution,
                            core::MeasurementProvenance provenance, core::ClockDomain domain,
                            double maximum_rms_px) {
  limit(maximum_rms_px);
  auto report = report_base(data, solution, provenance, domain);
  report["capability"] = "intrinsics";
  report["fit_rms_px"] = solution.rms_px;
  report["fit_per_view_rms_px"] = solution.per_view_rms_px;
  report["validation"]["rms_px"] = validate_intrinsics(data, solution);
  report["limits"]["rms_px"] = maximum_rms_px;

  const auto information = calibration_detail::intrinsic_information(data, solution);
  auto quality = report["intrinsic_quality"];
  quality["method"] = "profiled-pinhole5-column-normalized-v1";
  quality["minimum_information_ratio"] = calibration_detail::minimum_information_ratio;
  quality["information"] = std::vector<std::vector<double>>{};
  for (int row = 0; row < 9; ++row) {
    std::vector<double> values;
    for (int col = 0; col < 9; ++col)
      values.push_back(information(row, col));
    quality["information"].push_back(values);
  }
  for (const auto& view : data.views) {
    YAML::Node sample;
    sample["id"] = view.id;
    sample["split"] = view.validation ? "validation" : "fit";
    sample["points_fingerprint"] = calibration_detail::point_fingerprint(view.points);
    quality["samples"].push_back(sample);
  }

  return report;
}

YAML::Node hand_eye_report(const CalibrationDataset& data, const IntrinsicSolution& intrinsic,
                           const HandEyeSolution& solution, core::MeasurementProvenance provenance,
                           core::ClockDomain domain, double maximum_rotation_rad,
                           double maximum_translation_m) {
  limit(maximum_rotation_rad);
  limit(maximum_translation_m);
  auto report = report_base(data, intrinsic, provenance, domain);
  report["capability"] = "extrinsics";
  const auto& pose = solution.camera_to_gimbal.value();
  const auto& q = pose.rotation();
  const auto& t = pose.translation();
  report["parameters"]["camera_to_gimbal_quaternion_wxyz"] =
      std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  report["parameters"]["camera_to_gimbal_translation_m"] =
      std::vector<double>{t.x(), t.y(), t.z()};
  const auto residuals = validate_hand_eye(data, intrinsic, solution);
  report["validation"]["rotation_rad"] = residuals.rotation_rad;
  report["validation"]["translation_m"] = residuals.translation_m;
  report["fit_rotation_rad"] = solution.fit_rotation_rad;
  report["fit_translation_m"] = solution.fit_translation_m;
  report["limits"]["rotation_rad"] = maximum_rotation_rad;
  report["limits"]["translation_m"] = maximum_translation_m;

  return report;
}

core::Evidence calibration_report_evidence(const core::Config& report, const Calibration& binding,
                                           CalibrationCapability capability) {
  const bool intrinsic = capability == CalibrationCapability::intrinsics;
  const auto roi = report.contains("parameters.roi_offset")
                       ? report.require<std::vector<int>>("parameters.roi_offset")
                       : std::vector<int>{0, 0};

  if (roi != std::vector<int>{binding.roi_offset()[0], binding.roi_offset()[1]} ||
      report.require<int>("schema_version") != 1 ||
      report.require<std::string>("capability") != (intrinsic ? "intrinsics" : "extrinsics") ||
      report.require<int>("parameters.width") != binding.width() ||
      report.require<int>("parameters.height") != binding.height() ||
      report.require<std::vector<double>>("parameters.camera_matrix") !=
          std::vector<double>(binding.intrinsic().val, binding.intrinsic().val + 9) ||
      report.require<std::vector<double>>("parameters.distortion") != binding.distortion())
    throw std::invalid_argument("Calibration report capability/parameter binding mismatch");

  if (!intrinsic) {
    const auto q = report.require<std::vector<double>>(
        "parameters.camera_to_gimbal_quaternion_wxyz");
    const auto t = report.require<std::vector<double>>("parameters.camera_to_gimbal_translation_m");

    if (q.size() != 4 || t.size() != 3)
      throw std::invalid_argument("Incomplete extrinsic report binding");

    const math::SE3 recorded(Eigen::Quaterniond(q[0], q[1], q[2], q[3]), {t[0], t[1], t[2]});
    const auto& expected = binding.camera_to_gimbal().value();

    if ((recorded.translation() - expected.translation()).norm() > 1e-12 ||
        math::rotation_log(recorded.rotation().conjugate() * expected.rotation()).norm() > 1e-12)
      throw std::invalid_argument("Extrinsic report parameter binding mismatch");
  }

  const auto fit = report.require<std::vector<std::string>>("fit_ids");
  const auto validation = report.require<std::vector<std::string>>("validation_ids");
  std::set<std::string> ids;

  for (const auto& list : {fit, validation})
    for (const auto& id : list)
      if (id.empty() || !ids.insert(id).second)
        throw std::invalid_argument("Report sample ids overlap or are empty");

  core::MeasurementProvenance provenance{report.require<std::string>("provenance.device_id"),
      report.require<std::string>("provenance.configuration_id"),
      report.require<std::string>("provenance.date"),
      report.require<std::string>("provenance.method")};
  const auto name = report.require<std::string>("domain");

  if (name != "replay" && name != "host_monotonic")
    throw std::invalid_argument("Unknown calibration evidence domain");

  const auto source_kind = report.require<std::string>("source_kind");

  if ((source_kind != "synthetic" && source_kind != "recorded") ||
      (source_kind == "synthetic" && name != "replay"))
    throw std::invalid_argument("Invalid evidence origin or synthetic-to-measured promotion");

  const auto domain = name == "replay" ? core::ClockDomain::replay :
                                         core::ClockDomain::host_monotonic;
  std::vector<double> normalized;
  const auto append = [&](const char* key) {
    const auto residuals = report.require<std::vector<double>>(std::string("validation.") + key);
    const double maximum = report.require<double>(std::string("limits.") + key);
    limit(maximum);

    if (residuals.size() != validation.size())
      throw std::invalid_argument("Report residual/sample count mismatch");

    for (double residual : residuals) {
      if (!std::isfinite(residual) || residual < 0)
        throw std::invalid_argument("Invalid calibration validation residual");
      normalized.push_back(residual / maximum);
    }
  };

  if (intrinsic)
    append("rms_px");
  else {
    append("rotation_rad");
    append("translation_m");
  }

  if (fit.size() < 3 || validation.size() < 2)
    return core::Evidence::missing();

  if (intrinsic) {
    // 旧报告仍可绑定/读取参数，但没有新质量依据时不能授予内参能力。
    if (!report.contains("intrinsic_quality"))
      return core::Evidence::missing();
    if (report.require<std::string>("intrinsic_quality.method") !=
            "profiled-pinhole5-column-normalized-v1" ||
        report.require<double>("intrinsic_quality.minimum_information_ratio") !=
            calibration_detail::minimum_information_ratio)
      throw std::invalid_argument("Unknown intrinsic quality method/threshold");

    const auto rows = report.require<std::vector<std::vector<double>>>(
        "intrinsic_quality.information");
    if (rows.size() != 9)
      throw std::invalid_argument("Intrinsic information must have nine rows");
    calibration_detail::IntrinsicInformation information;
    for (int row = 0; row < 9; ++row) {
      if (rows[row].size() != 9)
        throw std::invalid_argument("Intrinsic information must have nine columns");
      for (int col = 0; col < 9; ++col)
        information(row, col) = rows[row][col];
    }
    std::vector<std::string> quality_fit, quality_validation;
    std::set<std::string> contents;
    for (const auto& sample : report.require<std::vector<YAML::Node>>(
             "intrinsic_quality.samples")) {
      const auto split = sample["split"].as<std::string>();
      const auto content = sample["points_fingerprint"].as<std::string>();
      if ((split != "fit" && split != "validation") || content.empty() ||
          !contents.insert(content).second)
        throw std::invalid_argument("Invalid/overlapping intrinsic sample content");
      (split == "fit" ? quality_fit : quality_validation).push_back(sample["id"].as<std::string>());
    }
    if (quality_fit != fit || quality_validation != validation)
      throw std::invalid_argument("Intrinsic quality sample binding mismatch");
    if (!calibration_detail::information_usable(information))
      return core::Evidence::missing();
  }

  return core::Evidence::from_report(core::MeasurementReport::evaluate(
      std::move(provenance), normalized, 1, domain));
}
} // namespace autoaim::vision
