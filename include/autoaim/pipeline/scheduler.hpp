#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace autoaim::pipeline {
// 生命周期由 pipeline 的控制线程管理；不在工作线程内销毁/stop 本对象。
// Worker 必须定期检查 stop，任何外部等待必须有界。
class Scheduler {
public:
  using Worker = std::function<void(const std::atomic<bool>& stop)>;
  using FailureHandler = std::function<void()>;
  Scheduler() = default;
  ~Scheduler();
  Scheduler(const Scheduler&) = delete;
  Scheduler& operator=(const Scheduler&) = delete;
  void start(std::vector<Worker> workers, FailureHandler on_failure);
  void request_stop() noexcept;
  void stop();
  bool stopping() const noexcept { return stopping_.load(); }
  std::exception_ptr failure() const;
  std::exception_ptr cleanup_failure() const;
private:
  void record_failure(std::exception_ptr error);
  std::atomic<bool> stopping_{false};
  mutable std::mutex failure_mutex_;
  std::mutex lifecycle_mutex_;
  std::exception_ptr first_failure_;
  std::exception_ptr cleanup_failure_;
  FailureHandler on_failure_;
  std::vector<std::thread> workers_;
  bool started_ = false;
  bool closed_ = false;
};
}  // namespace autoaim::pipeline
