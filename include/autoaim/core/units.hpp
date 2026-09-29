#pragma once

#include <cmath>
#include <stdexcept>

namespace autoaim::core {
// 单位必须显式转换；有限值约束不等于业务范围验证（例如弹速仍须 > 0）。
template <class Tag>
class Quantity {
public:
  explicit Quantity(double value) : value_(value) {
    if (!std::isfinite(value)) throw std::invalid_argument("Non-finite quantity");
  }
  double value() const noexcept { return value_; }
private:
  double value_;
};
struct RadiansTag {};
struct MetresTag {};
struct SecondsTag {};
struct MetresPerSecondTag {};
using Radians = Quantity<RadiansTag>;
using Metres = Quantity<MetresTag>;
using Seconds = Quantity<SecondsTag>;
using MetresPerSecond = Quantity<MetresPerSecondTag>;
}  // namespace autoaim::core
