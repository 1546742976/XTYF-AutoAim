#pragma once

#include "autoaim/mission/mission.hpp"
#include "autoaim/mission/button_policy.hpp"
#include "autoaim/core/evidence.hpp"

namespace autoaim::mission {
// 只计算相对于实测指向的修正，不合成人工绝对控制量；程序开火恒 false。
MissionRequest infantry_assist(const AimSolution& solution,
                               const decision::MeasuredPointing& measured);

// HAL 操作输入由 pipeline 转为此 DTO；不以缺失/恒零输入代表无人干预。
struct OperatorSignal {
  core::TimePoint sampled_at;
  core::Generation generation;
  bool valid;
  bool intervening;
  std::uint64_t enable_event;
  core::Evidence evidence;
  // 独立操作手电平不隶属于图像世代；源时间不重打。空值保留旧显式启用事件语义。
  std::optional<bool> button_pressed{};
};

struct InfantryModeOptions {
  core::Seconds input_maximum_age;
  std::string device_id;
  std::string configuration_id;
  ButtonMode button_mode = ButtonMode::toggle;
};

// 单线程模式所有者；调用者必须在 changed=true 时清理旧世代在途结果和命令。
class InfantryMission {
public:
  InfantryMission(InfantryModeOptions options, core::Generation generation, core::TimePoint since);
  bool update(const std::optional<OperatorSignal>& input, const core::Evidence& automatic_channel,
              bool fault, core::TimePoint now);

  MissionRequest request(const AimSolution& solution, const decision::MeasuredPointing& measured,
                         bool program_fire_requested) const;

  core::Authority authority() const;

  core::Generation generation() const {
    return generation_;
  }

  core::TimePoint mode_since() const {
    return since_;
  }

private:
  InfantryModeOptions options_;
  core::Generation generation_;
  core::TimePoint since_;
  core::ControlMode mode_ = core::ControlMode::assist;
  std::uint64_t consumed_enable_ = 0;
  ButtonPolicy button_;
  std::optional<core::TimePoint> last_button_sample_;
};
} // namespace autoaim::mission
