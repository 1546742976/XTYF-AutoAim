#include "autoaim/pipeline/scheduler.hpp"
#include <stdexcept>

namespace autoaim::pipeline {
Scheduler::~Scheduler() { stop(); }

void Scheduler::start(std::vector<Worker> workers, FailureHandler on_failure) {
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  if (started_ || closed_ || workers.empty() || !on_failure)
    throw std::logic_error("Invalid scheduler start");
  for (const auto& worker : workers)
    if (!worker) throw std::invalid_argument("Empty worker");
  workers_.reserve(workers.size());
  on_failure_ = std::move(on_failure);
  started_ = true;
  try {
    for (auto& worker : workers) {
      workers_.emplace_back([this, function = std::move(worker)] {
        try { function(stopping_); }
        catch (...) { record_failure(std::current_exception()); }
      });
    }
  } catch (...) {
    request_stop();
    for (auto& worker : workers_) if (worker.joinable()) worker.join();
    closed_ = true;
    throw;
  }
}

void Scheduler::record_failure(std::exception_ptr error) {
  bool first = false;
  {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    if (!first_failure_) { first_failure_ = error; first = true; }
  }
  request_stop();
  if (!first) return;
  // 只通知一次，且不在故障锁内调用管线清理/停止请求。
  try { on_failure_(); }
  catch (...) {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    cleanup_failure_ = std::current_exception();
  }
}

void Scheduler::request_stop() noexcept { stopping_.store(true); }
void Scheduler::stop() {
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  request_stop();
  for (auto& worker : workers_) {
    if (worker.joinable()) worker.join();
  }
  closed_ = true;
}
std::exception_ptr Scheduler::failure() const {
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return first_failure_;
}
std::exception_ptr Scheduler::cleanup_failure() const {
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return cleanup_failure_;
}
}  // namespace autoaim::pipeline
