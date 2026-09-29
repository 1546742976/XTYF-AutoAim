#include "autoaim/pipeline/command_slot.hpp"

namespace autoaim::pipeline {
namespace {
bool current(const control::ControlIntent& intent, core::TimePoint now) {
  return intent.created_at.domain() == now.domain() &&
    intent.created_at.nanoseconds() <= now.nanoseconds() && now.nanoseconds() < intent.deadline.nanoseconds();
}
}  // namespace
CommandSlot::CommandSlot(core::Generation generation, core::TimePoint since)
    : generation_(generation), since_(since) {}

bool CommandSlot::submit(control::ControlIntent intent, core::TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_ || intent.source.generation != generation_ || now.domain() != since_.domain() ||
      !current(intent, now) || intent.source.exposure.nanoseconds() <= since_.nanoseconds() ||
      intent.source.frame_id < last_frame_ ||
      (last_created_ && intent.created_at.nanoseconds() <= last_created_->nanoseconds())) return false;
  auto replacement = std::make_shared<const control::ControlIntent>(std::move(intent));
  last_created_ = replacement->created_at;
  last_frame_ = replacement->source.frame_id;
  latest_ = std::move(replacement);
  return true;
}

std::shared_ptr<const control::ControlIntent> CommandSlot::take(core::TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(latest_);
  if (closed_ || !result || !current(*result, now)) return {};
  return result;
}

void CommandSlot::reset(core::Generation generation, core::TimePoint since) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation <= generation_ || core::elapsed(since, since_).value() < 0)
    throw std::invalid_argument("Command epoch must advance");
  latest_.reset();
  generation_ = generation;
  since_ = since;
  last_created_.reset();
  last_frame_ = 0;
}
void CommandSlot::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  closed_ = true;
  latest_.reset();
}
}  // namespace autoaim::pipeline
