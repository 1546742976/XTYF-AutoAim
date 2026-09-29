#pragma once

#include "autoaim/control/publisher.hpp"

namespace autoaim::pipeline {
// 仅适配最终复检命令到 UART14；transport 必须覆盖 Publisher 生命周期。
// 回调只交给唯一发布线程，不改变 control/shoot，不重试已部分写出的指令。
control::Publisher::Write make_uart14_writer(hal::Transport& transport, core::Seconds timeout);
} // namespace autoaim::pipeline
