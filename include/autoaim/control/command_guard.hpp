#pragma once

#include "autoaim/control/checked_command.hpp"
#include "autoaim/control/watchdog.hpp"
#include "autoaim/hal/gimbal_feedback.hpp"
#include <memory>

namespace autoaim::control {
enum class Inhibit : std::uint32_t {
  permission = 1u << 0,
  generation = 1u << 1,
  time = 1u << 2,
  feedback = 1u << 3,
  numeric = 1u << 4,
  capability = 1u << 5,
  target = 1u << 6,
  bullet_speed = 1u << 7,
  intervention = 1u << 8,
  fault = 1u << 9,
  pointing = 1u << 10,
};

struct GuardOptions {
  TimingLimits timing;
  std::string device_id;
  std::string configuration_id;
  core::Radians yaw_tolerance;
  core::Radians pitch_tolerance;
  std::optional<core::Seconds> operator_maximum_age{};
};

struct GuardContext {
  core::Authority authority;
  core::Generation generation;
  core::TimePoint mode_since;
  std::optional<hal::GimbalFeedback> feedback;
  core::Evidence control_channel;
  bool fault_latched;
  std::optional<hal::OperatorInput> independent_operator{};
};

class Admission {
public:
  const ControlIntent& intent() const noexcept {
    return *intent_;
  }

  bool fire_allowed() const noexcept {
    return fire_;
  }

  bool control_allowed() const noexcept {
    return control_;
  }

  std::uint32_t reasons() const noexcept {
    return reasons_;
  }

private:
  friend class CommandGuard;

  Admission(std::shared_ptr<const ControlIntent> intent, bool control, bool fire,
            std::uint32_t reasons)
      : intent_(std::move(intent)), control_(control), fire_(fire), reasons_(reasons) {
  }

  std::shared_ptr<const ControlIntent> intent_;
  bool control_;
  bool fire_;
  std::uint32_t reasons_;
};

// 无 I/O、无线程；publisher 用原子一致的上下文快照调用首检和每次发送复检。
class CommandGuard {
public:
  explicit CommandGuard(GuardOptions options);
  Admission admit(ControlIntent intent, const GuardContext& context, core::TimePoint now) const;
  CheckedCommand recheck(Admission& admission, const GuardContext& context,
                         core::TimePoint now) const;

private:
  struct Verdict {
    bool control;
    bool fire;
    std::uint32_t reasons;
  };

  Verdict evaluate(const ControlIntent& intent, const GuardContext& context,
                   core::TimePoint now) const;

  GuardOptions options_;
  Watchdog watchdog_;
};
} // namespace autoaim::control
