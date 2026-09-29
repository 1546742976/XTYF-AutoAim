#pragma once

#include "autoaim/hal/transport.hpp"

namespace autoaim::hal {
struct CanOptions {
  std::string interface_name;
  std::uint32_t transmit_id;
  std::vector<std::uint32_t> receive_ids;
};

// 首版标准 11-bit CAN，写入仅为 0..8 字节 payload，发送 id 显式配置。
// read_for 返回 [4 字节小端 CAN id, payload]，保留帧边界而不解释业务协议。
// 构造不连接；open/close 外部串行，读写各至多一个调用线程。
// 关闭前须停止并 join 读写线程，禁止 close 与传输调用并发。
class CanTransport final : public Transport {
public:
  explicit CanTransport(CanOptions options);
  ~CanTransport();
  CanTransport(const CanTransport&) = delete;
  CanTransport& operator=(const CanTransport&) = delete;
  core::Result<bool> open_device();
  void close_device() noexcept;

  bool is_open() const {
    return fd_ >= 0;
  }

  WriteResult write_all(const std::vector<std::uint8_t>& bytes, core::Seconds timeout) override;
  core::Result<std::vector<std::uint8_t>> read_for(core::Seconds timeout) override;

private:
  CanOptions options_;
  int fd_ = -1;
};
} // namespace autoaim::hal
