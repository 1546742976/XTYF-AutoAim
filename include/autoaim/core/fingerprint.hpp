#pragma once

#include <cstddef>
#include <cstdint>

namespace autoaim::core {
class Fingerprint {
public:
  void append(const void* data, std::size_t size) noexcept {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
      value_ ^= bytes[i];
      value_ *= 1099511628211ULL;
    }
    bytes_ += static_cast<std::uint64_t>(size);
  }

  std::uint64_t value() const noexcept {
    return value_;
  }

  std::uint64_t bytes() const noexcept {
    return bytes_;
  }

private:
  std::uint64_t value_ = 14695981039346656037ULL;
  std::uint64_t bytes_ = 0;
};
} // namespace autoaim::core
