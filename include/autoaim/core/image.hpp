#pragma once

#include "autoaim/core/evidence.hpp"
#include "autoaim/core/types.hpp"
#include <limits>
#include <memory>
#include <vector>

namespace autoaim::core {
using PixelLease = std::shared_ptr<const std::vector<std::uint8_t>>;

// 首版统一 BGR8。发布后像素只读；租约负责延长实际缓冲区生命周期。
struct Image {
  const int width;
  const int height;
  const std::size_t stride;
  const PixelLease pixels;
  Image(int columns, int rows, std::size_t step, PixelLease lease)
      : width(columns), height(rows), stride(step), pixels(std::move(lease)) {
    if (width <= 0 || height <= 0 || !pixels ||
        stride < static_cast<std::size_t>(width) * 3 ||
        stride > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height) ||
        pixels->size() < stride * static_cast<std::size_t>(height))
      throw std::invalid_argument("Invalid owned BGR8 image");
  }
};

struct CapturedFrame {
  const Stamp stamp;
  const Image image;
  const TimePoint received_at;
  const TimeOrigin time_origin;
  const Seconds timing_uncertainty;
  const Evidence timing_evidence;
  CapturedFrame(Stamp source, Image owned_image, TimePoint received, TimeOrigin origin,
                Seconds uncertainty, Evidence evidence)
      : stamp(source), image(std::move(owned_image)), received_at(received), time_origin(origin),
        timing_uncertainty(uncertainty), timing_evidence(std::move(evidence)) {
    if (uncertainty.value() < 0 || elapsed(received_at, stamp.exposure).value() < 0)
      throw std::invalid_argument("Invalid exposure/receive timing");
    if (origin == TimeOrigin::synthetic && stamp.exposure.domain() != ClockDomain::replay)
      throw std::invalid_argument("Synthetic frame in live clock domain");
  }
};
}  // namespace autoaim::core
