#pragma once

namespace autoaim::pipeline {
// 离线标注 CLI；不打开设备，不把标注或审核声明传入控制链。
int run_annotate_session(int argc, char** argv);
int run_calibration_tool(int argc, char** argv);
int run_calibration_solver(int argc, char** argv);
int run_detector_benchmark(int argc, char** argv);
int run_batch_benchmark(int argc, char** argv);
} // namespace autoaim::pipeline
