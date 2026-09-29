#pragma once

namespace autoaim::pipeline {
int run_calibration_tool(int argc, char** argv);
int run_calibration_solver(int argc, char** argv);
int run_detector_benchmark(int argc, char** argv);
int run_batch_benchmark(int argc, char** argv);
} // namespace autoaim::pipeline
