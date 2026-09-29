#include "autoaim/vision/detector_factory.hpp"

namespace autoaim::vision {
std::unique_ptr<Detector> make_detector(const DetectorOptions& options) {
  return std::visit(
      [](const auto& selected) -> std::unique_ptr<Detector> {
        using Selected = std::decay_t<decltype(selected)>;

        if constexpr (std::is_same_v<Selected, TraditionalOptions>)
          return std::make_unique<TraditionalDetector>(selected);
        else
          return std::make_unique<OpenVinoDetector>(selected);
      },
      options);
}
} // namespace autoaim::vision
