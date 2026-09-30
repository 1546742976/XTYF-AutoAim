#include "autoaim/vision/corner_refine.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace autoaim::vision {
// Defined in tests/support/corner_refine_baseline.cpp and linked only for this
// candidate regression test; the production library has one CMake-selected implementation.
RefinementResult refine_corners_baseline(const core::Image& image, const Detection& original,
                                         const RefinementOptions& options);
} // namespace autoaim::vision

namespace {
using namespace autoaim;

struct Canvas {
  static constexpr int width = 100, height = 80;
  const std::size_t stride = 307;
  std::shared_ptr<std::vector<std::uint8_t>> bytes =
      std::make_shared<std::vector<std::uint8_t>>(stride * height, 0);

  Canvas() {
    for (int y = 0; y < height; ++y)
      std::fill(bytes->begin() + y * stride + width * 3,
                bytes->begin() + (y + 1) * stride, std::uint8_t(203));
  }

  void pixel(int x, int y, int channel, std::uint8_t value) {
    (*bytes)[y * stride + x * 3 + channel] = value;
  }

  void bars(int channel, bool contamination = false) {
    for (int y = 20; y <= 50; ++y) {
      for (int x : {19, 20, 21, 69, 70, 71})
        pixel(x, y, channel, 255);
      if (contamination)
        for (int x : {17, 67})
          pixel(x, y, channel, 110);
    }
  }

  core::Image image() const {
    return {width, height, stride, bytes};
  }
};

vision::Detection detection() {
  return {{cv::Point2f(21, 21), {71, 21}, {71, 49}, {21, 49}},
          0.73f, vision::TeamColor::red, 7, true, vision::TargetCategory::three,
          vision::ArmorSize::small};
}

void same_metadata(const vision::Detection& actual, const vision::Detection& original) {
  CHECK(actual.confidence == original.confidence);
  CHECK(actual.color == original.color);
  CHECK(actual.raw_class_id == original.raw_class_id);
  CHECK(actual.category == original.category);
  CHECK(actual.armor_size == original.armor_size);
}

void unchanged_failure(const vision::RefinementResult& result,
                       const vision::Detection& original) {
  CHECK(!result.refined && !result.detection.corners_reliable);
  CHECK(result.detection.corners == original.corners);
  same_metadata(result.detection, original);
}

double transverse_error(const vision::Detection& actual) {
  return (std::abs(actual.corners[0].x - 20.0) + std::abs(actual.corners[3].x - 20.0) +
          std::abs(actual.corners[1].x - 70.0) + std::abs(actual.corners[2].x - 70.0)) / 4;
}
} // namespace

int main() {
  return test::run([] {
    const vision::RefinementOptions options{100, 50, 4, 4, 10};
    const auto original = detection();

    // A same-color, dim, parallel contaminant passes the unchanged pixel thresholds.
    // Its contrast must matter: merely copying the old Huber fitter cannot pass this test.
    Canvas contaminated;
    contaminated.bars(2, true);
    const auto before = *contaminated.bytes;
    const auto baseline = vision::refine_corners_baseline(contaminated.image(), original, options);
    const auto weighted = vision::refine_corners(contaminated.image(), original, options);
    CHECK(baseline.refined && weighted.refined && weighted.detection.corners_reliable);
    const double original_error = transverse_error(baseline.detection);
    const double weighted_error = transverse_error(weighted.detection);
    CHECK(original_error > 0.1);
    CHECK(weighted_error + 0.05 < original_error);
    CHECK(weighted_error < 0.3);
    for (int index : {0, 1})
      CHECK_NEAR(weighted.detection.corners[index].y, 20.0, 0.01);
    for (int index : {2, 3})
      CHECK_NEAR(weighted.detection.corners[index].y, 50.0, 0.01);
    same_metadata(weighted.detection, original);
    CHECK(*contaminated.bytes == before); // Includes stride padding, not just visible pixels.

    // Equal weights on an uncontaminated bar preserve the original centerline endpoints.
    Canvas clean;
    clean.bars(2);
    const auto clean_baseline = vision::refine_corners_baseline(clean.image(), original, options);
    const auto clean_weighted = vision::refine_corners(clean.image(), original, options);
    CHECK(clean_baseline.refined && clean_weighted.refined);
    for (std::size_t i = 0; i < 4; ++i)
      CHECK(clean_baseline.detection.corners[i] == clean_weighted.detection.corners[i]);
    CHECK(transverse_error(clean_weighted.detection) < 0.001);

    Canvas dim_endpoints;
    dim_endpoints.bars(2);
    for (int x : {20, 70})
      for (int y : {18, 52})
        dim_endpoints.pixel(x, y, 2, 110);
    const auto full_extent = vision::refine_corners(dim_endpoints.image(), original, options);
    CHECK(full_extent.refined);
    for (int index : {0, 1})
      CHECK_NEAR(full_extent.detection.corners[index].y, 18.0, 0.001);
    for (int index : {2, 3})
      CHECK_NEAR(full_extent.detection.corners[index].y, 52.0, 0.001);

    Canvas blue_pixels;
    blue_pixels.bars(0, true);
    auto blue = original;
    blue.color = vision::TeamColor::blue;
    const auto blue_result = vision::refine_corners(blue_pixels.image(), blue, options);
    CHECK(blue_result.refined && blue_result.detection.corners_reliable);
    for (std::size_t i = 0; i < 4; ++i)
      CHECK(cv::norm(blue_result.detection.corners[i] - weighted.detection.corners[i]) < 0.001);
    same_metadata(blue_result.detection, blue);
    unchanged_failure(vision::refine_corners(clean.image(), blue, options), blue);

    // Physical indices can be inverted in the image after a half-turn.
    auto rolled = original;
    rolled.corners = {original.corners[2], original.corners[3], original.corners[0],
                      original.corners[1]};
    const auto inverted = vision::refine_corners(clean.image(), rolled, options);
    CHECK(inverted.refined && inverted.detection.corners[0].y > inverted.detection.corners[3].y);
    CHECK(cv::norm(inverted.detection.corners[0] - clean_weighted.detection.corners[2]) < 0.001);
    same_metadata(inverted.detection, rolled);

    // Perspective permits both unequal lengths and nonparallel image axes.
    Canvas perspective;
    cv::Mat view(Canvas::height, Canvas::width, CV_8UC3, perspective.bytes->data(),
                 perspective.stride);
    cv::line(view, {20, 20}, {25, 50}, cv::Scalar(0, 0, 255), 3, cv::LINE_8);
    cv::line(view, {70, 27}, {64, 48}, cv::Scalar(0, 0, 255), 3, cv::LINE_8);
    auto projected = original;
    projected.corners = {cv::Point2f(20, 20), {70, 27}, {64, 48}, {25, 50}};
    const auto oblique_baseline =
        vision::refine_corners_baseline(perspective.image(), projected, options);
    const auto oblique = vision::refine_corners(perspective.image(), projected, options);
    CHECK(oblique_baseline.refined && oblique.refined && oblique.detection.corners_reliable);
    // Non-axis-aligned equal-contrast bars must preserve not just the ROI rule but
    // the original float boundary selection and endpoint results, bit for bit.
    for (std::size_t i = 0; i < 4; ++i)
      CHECK(oblique.detection.corners[i] == oblique_baseline.detection.corners[i]);
    const auto left_axis = oblique.detection.corners[3] - oblique.detection.corners[0];
    const auto right_axis = oblique.detection.corners[2] - oblique.detection.corners[1];
    CHECK(cv::norm(left_axis) - cv::norm(right_axis) > 5);
    CHECK(left_axis.dot(right_axis) / (cv::norm(left_axis) * cv::norm(right_axis)) < 0.95);

    // An accepted update never upgrades an unqualified corner mapping.
    auto unqualified = original;
    unqualified.corners_reliable = false;
    const auto still_unqualified = vision::refine_corners(clean.image(), unqualified, options);
    CHECK(still_unqualified.refined && !still_unqualified.detection.corners_reliable);

    auto strict = options;
    strict.maximum_shift_px = 0.1;
    unchanged_failure(vision::refine_corners(clean.image(), original, strict), original);
    Canvas one_bar;
    for (int y = 20; y <= 50; ++y)
      for (int x : {19, 20, 21})
        one_bar.pixel(x, y, 2, 255);
    // The left side can fit, but failure on the right must discard the partial update.
    unchanged_failure(vision::refine_corners(one_bar.image(), original, options), original);

    Canvas isotropic;
    for (int center : {20, 70}) {
      for (int x = center - 1; x <= center + 1; ++x)
        isotropic.pixel(x, 35, 2, 255);
      for (int y : {34, 36})
        isotropic.pixel(center, y, 2, 255);
    }
    auto short_bars = original;
    short_bars.corners = {cv::Point2f(20, 30), {70, 30}, {70, 40}, {20, 40}};
    auto sparse_options = options;
    sparse_options.minimum_pixels = 3;
    unchanged_failure(vision::refine_corners(isotropic.image(), short_bars, sparse_options),
                      short_bars);

    auto unknown = original;
    unknown.color = vision::TeamColor::extinguished;
    unchanged_failure(vision::refine_corners(clean.image(), unknown, options), unknown);
    auto invalid = original;
    invalid.corners[0] = invalid.corners[3];
    unchanged_failure(vision::refine_corners(clean.image(), invalid, options), invalid);
    invalid = original;
    invalid.corners[0].x = -1;
    unchanged_failure(vision::refine_corners(clean.image(), invalid, options), invalid);
    invalid = original;
    invalid.corners[0].x = std::numeric_limits<float>::infinity();
    unchanged_failure(vision::refine_corners(clean.image(), invalid, options), invalid);
    invalid = original;
    invalid.corners[0].x = std::numeric_limits<float>::quiet_NaN();
    const auto nan_result = vision::refine_corners(clean.image(), invalid, options);
    CHECK(!nan_result.refined && !nan_result.detection.corners_reliable);
    CHECK(std::isnan(nan_result.detection.corners[0].x));
    CHECK(nan_result.detection.corners[1] == invalid.corners[1]);
    same_metadata(nan_result.detection, invalid);

    // Zero contrast at explicitly zero thresholds still has finite, positive base weights.
    Canvas black;
    const vision::RefinementOptions zero_thresholds{0, 0, 1, 2, 3};
    const auto zero_contrast = vision::refine_corners(black.image(), original, zero_thresholds);
    CHECK(zero_contrast.refined);
    for (const auto& corner : zero_contrast.detection.corners)
      CHECK(std::isfinite(corner.x) && std::isfinite(corner.y));

    auto invalid_options = options;
    invalid_options.minimum_pixels = 2;
    CHECK_THROWS(std::invalid_argument,
                 vision::refine_corners(clean.image(), original, invalid_options));
    invalid_options = options;
    invalid_options.search_radius_px = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(std::invalid_argument,
                 vision::refine_corners(clean.image(), original, invalid_options));
  });
}
