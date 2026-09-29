#pragma once

namespace autoaim::control {
// 最终输出载荷：字段名、类型、顺序和距离默认值与 sp_vision_25/io/command.hpp 一致。
// yaw/pitch 使用 rad，horizon_distance 使用 m；指令空间由旁路元数据说明。
// 用 Command{} 初始化停止值；运行链中的 control/shoot 只能来自 command_guard 复检。
// 这是 C++ 值对象，不是打包的线协议结构，禁止直接按 sizeof(Command) 写入设备。
struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0; // 参考接口保留的水平距离字段（无人机专有）。
};
} // namespace autoaim::control
