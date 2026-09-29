#pragma once

#include "autoaim/hal/transport.hpp"

namespace autoaim::hal {
// Linux 原始 8N1 字节通道，不解释协议。构造只校验参数，不访问设备。
// open/close 必须由生命周期线程串行执行；可有一个读线程和一个发布写线程。
// 关闭前须停止并 join 读写线程，禁止 close 与传输调用并发。
class SerialTransport final : public Transport {
public:
  SerialTransport(std::string device, unsigned baud);
  ~SerialTransport();
  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;
  core::Result<bool> open_device();
  void close_device() noexcept;
  bool is_open() const { return fd_ >= 0; }
  WriteResult write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) override;
  core::Result<std::vector<std::uint8_t>> read_for(core::Seconds timeout) override;
private:
  std::string device_;
  unsigned baud_;
  int fd_ = -1;
};
}  // namespace autoaim::hal
