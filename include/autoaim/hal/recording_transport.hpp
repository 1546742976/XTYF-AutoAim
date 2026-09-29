#pragma once

#include "autoaim/hal/transport.hpp"
#include "autoaim/hal/clock.hpp"
#include <mutex>
#include <ostream>

namespace autoaim::hal {
// 离线输出端：借用 stream/clock，二者必须存活至析构。记录不代表设备执行。
// 不缓存历史包；文件阻塞时间受操作系统影响，不用于实时设备传输。
class RecordingTransport final : public Transport {
public:
  RecordingTransport(std::ostream& stream, Clock& clock);
  WriteResult write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) override;
  core::Result<std::vector<std::uint8_t>> read_for(core::Seconds timeout) override;
  std::uint64_t completed_writes() const;

private:
  std::ostream& stream_;
  Clock& clock_;
  mutable std::mutex mutex_;
  bool failed_ = false;
  std::uint64_t completed_ = 0;
};
} // namespace autoaim::hal
