#pragma once

#include "autoaim/core/result.hpp"
#include "autoaim/core/time.hpp"
#include <cstdint>
#include <vector>

namespace autoaim::hal {
enum class WriteStatus { complete, failed };
struct WriteResult {
  WriteStatus status;
  std::size_t bytes_written;
  std::string error;
};
class Transport {
public:
  virtual ~Transport() = default;
  // 由唯一发布线程调用；超时/短写均 failed。complete 只表示主机完整写入。
  virtual WriteResult write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) = 0;
  virtual core::Result<std::vector<std::uint8_t>> read_for(core::Seconds timeout) = 0;
};
}  // namespace autoaim::hal
