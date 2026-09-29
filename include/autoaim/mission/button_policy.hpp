#pragma once

namespace autoaim::mission {
enum class ButtonMode { toggle, hold };
enum class ButtonAction { none, toggle, enable, disable };

// 纯电平到事件转换，由模式所有者单线程调用。usable 包含输入时效/有效性、
// 人工接管和故障检查；失效后必须观察到可用的释放电平才能再次接纳按下沿。
class ButtonPolicy {
public:
  explicit ButtonPolicy(ButtonMode mode);
  ButtonAction update(bool pressed, bool usable);

private:
  ButtonMode mode_;
  bool released_ = false;
  bool pressed_ = true; // 启动时按住不能凭空制造一次按下沿。
};
} // namespace autoaim::mission
