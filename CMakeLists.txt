cmake_minimum_required(VERSION 3.22)
project(XTYF_AutoAim VERSION 1.1.0 LANGUAGES CXX)

# 保留已验证的默认版本；允许配置时使用 -DCMAKE_CXX_STANDARD=20。
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ language standard (17 by default, 20 allowed)")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CTest)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d imgproc imgcodecs)
option(AUTOAIM_OPENVINO "Build the OpenVINO YOLOv5/YOLO11 backends" ON)

if(AUTOAIM_OPENVINO)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
endif()

# 模块构建辅助函数与依赖白名单。
# 只编译显式列出的实现；后续任务的空占位文件不纳入构建。
function(autoaim_module name)
  add_library(autoaim_${name} STATIC ${ARGN})

  target_include_directories(autoaim_${name} PUBLIC ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(autoaim_${name} PUBLIC autoaim_options)
endfunction()

function(autoaim_dependencies name)
  set(allowed_core)
  set(allowed_math core)
  set(allowed_hal core)
  set(allowed_vision math)
  set(allowed_estimation vision)
  set(allowed_decision estimation)
  set(allowed_control hal)
  set(allowed_mission decision)
  set(allowed_pipeline mission control hal)

  foreach(dependency IN LISTS ARGN)
    if(NOT dependency IN_LIST allowed_${name})
      message(FATAL_ERROR "Forbidden module dependency: ${name} -> ${dependency}")
    endif()

    target_link_libraries(autoaim_${name} PUBLIC autoaim_${dependency})
  endforeach()
endfunction()

add_library(autoaim_options INTERFACE)
target_compile_options(autoaim_options INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
option(AUTOAIM_SANITIZERS
  "Enable AddressSanitizer and UndefinedBehaviorSanitizer for offline tests" OFF)
option(AUTOAIM_THREAD_SANITIZER "Enable ThreadSanitizer for offline concurrency tests" OFF)
if(AUTOAIM_SANITIZERS AND AUTOAIM_THREAD_SANITIZER)
  message(FATAL_ERROR "ThreadSanitizer and AddressSanitizer/UBSan must use separate builds")
endif()
if(AUTOAIM_THREAD_SANITIZER)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "AUTOAIM_THREAD_SANITIZER requires GCC or Clang")
  endif()
  target_compile_options(autoaim_options INTERFACE -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(autoaim_options INTERFACE -fsanitize=thread)
endif()
if(AUTOAIM_SANITIZERS)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "AUTOAIM_SANITIZERS requires GCC or Clang")
  endif()
  target_compile_options(autoaim_options INTERFACE -fsanitize=address,undefined
    -fno-omit-frame-pointer)
  target_link_options(autoaim_options INTERFACE -fsanitize=address,undefined)
endif()

autoaim_module(core src/core/result.cpp src/core/time.cpp src/core/logging.cpp src/core/config.cpp
  src/core/buffer_pool.cpp)
target_link_libraries(autoaim_core PUBLIC Threads::Threads yaml-cpp)
autoaim_module(math src/math/angle.cpp src/math/numeric.cpp src/math/se3.cpp src/math/transform.cpp)
target_link_libraries(autoaim_math PUBLIC Eigen3::Eigen)
autoaim_module(hal src/hal/clock_impl.cpp)
target_sources(autoaim_hal PRIVATE src/hal/file_replay.cpp)
target_sources(autoaim_hal PRIVATE src/hal/session_writer.cpp)
target_sources(autoaim_hal PRIVATE src/hal/recording_transport.cpp)
target_sources(autoaim_hal PRIVATE src/hal/serial_transport.cpp)
target_sources(autoaim_hal PRIVATE src/hal/can_transport.cpp)
target_sources(autoaim_hal PRIVATE src/hal/camera_impl.cpp)
target_sources(autoaim_hal PRIVATE src/hal/camera_parameters.cpp)
option(AUTOAIM_HIKROBOT "Compile optional Hikrobot backend; never enables hardware in replay" OFF)
set(AUTOAIM_HIKROBOT_INCLUDE "" CACHE PATH "Explicit SDK include directory")
set(AUTOAIM_HIKROBOT_LIBRARY "" CACHE FILEPATH "Explicit SDK library file")
if(AUTOAIM_HIKROBOT)
  if(NOT EXISTS "${AUTOAIM_HIKROBOT_INCLUDE}/MvCameraControl.h" OR NOT EXISTS
    "${AUTOAIM_HIKROBOT_LIBRARY}")
    message(FATAL_ERROR "Explicit Hikrobot SDK include/library paths are required")
  endif()
  target_compile_definitions(autoaim_hal PRIVATE AUTOAIM_HAS_HIKROBOT=1)
  target_include_directories(autoaim_hal SYSTEM PRIVATE "${AUTOAIM_HIKROBOT_INCLUDE}")
  target_link_libraries(autoaim_hal PRIVATE "${AUTOAIM_HIKROBOT_LIBRARY}")
endif()
target_include_directories(autoaim_hal SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(autoaim_hal PRIVATE opencv_core opencv_imgcodecs)
autoaim_module(vision src/vision/calibration.cpp src/vision/detector_traditional.cpp)
target_sources(autoaim_vision PRIVATE src/vision/calibration_dataset.cpp)
target_sources(autoaim_vision PRIVATE src/vision/calibration_solver.cpp)
target_sources(autoaim_vision PRIVATE src/vision/calibration_report.cpp)
target_sources(autoaim_vision PRIVATE src/vision/detection_labels.cpp)
target_sources(autoaim_vision PRIVATE src/vision/detector_openvino.cpp
  src/vision/detector_factory.cpp)
target_sources(autoaim_vision PRIVATE src/vision/corner_refine.cpp)
target_sources(autoaim_vision PRIVATE src/vision/pnp.cpp)
if(AUTOAIM_OPENVINO)
  target_compile_definitions(autoaim_vision PRIVATE AUTOAIM_HAS_OPENVINO=1)
  target_link_libraries(autoaim_vision PRIVATE openvino::runtime)
endif()
target_include_directories(autoaim_vision SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(autoaim_vision PUBLIC ${OpenCV_LIBS})
autoaim_module(estimation src/estimation/observation.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/geometry_model.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/motion_model.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/measurement_model.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/ekf.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/health.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/association.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/armor_id.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/geometry_selector.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/state_machine.cpp)
target_sources(autoaim_estimation PRIVATE src/estimation/tracker.cpp)
autoaim_module(decision src/decision/predictor.cpp)
target_sources(autoaim_decision PRIVATE src/decision/ballistic.cpp)
target_sources(autoaim_decision PRIVATE src/decision/armor_selector.cpp)
target_sources(autoaim_decision PRIVATE src/decision/hit_probability.cpp)
target_sources(autoaim_decision PRIVATE src/decision/aim_adequacy.cpp)
autoaim_module(control src/control/crc.cpp src/control/command.cpp src/control/protocol.cpp
  src/control/watchdog.cpp)
target_sources(autoaim_control PRIVATE src/control/smoother.cpp)
target_sources(autoaim_control PRIVATE src/control/command_guard.cpp)
target_sources(autoaim_control PRIVATE src/control/publisher.cpp)
autoaim_module(mission src/mission/mission_factory.cpp)
target_sources(autoaim_mission PRIVATE src/mission/target_selector.cpp)
target_sources(autoaim_mission PRIVATE src/mission/infantry/infantry_mission.cpp)
target_sources(autoaim_mission PRIVATE src/mission/sentry/sentry_mission.cpp)
autoaim_module(pipeline src/pipeline/frame_sync.cpp src/pipeline/queue.cpp src/pipeline/command_slot.cpp
  src/pipeline/scheduler.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/bootstrap.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/pipeline.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/entry.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/offline_tools.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/batch_benchmark.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/session_annotation.cpp
  src/pipeline/annotate_session.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/run_metadata.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/calibration_tools.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/uart_adapter.cpp)
target_sources(autoaim_pipeline PRIVATE src/pipeline/uart_writer.cpp)

# 报告来源只记录内容和构建参数，不包含配置时间。源码改变会触发重新配置。
file(GLOB_RECURSE autoaim_provenance_sources CONFIGURE_DEPENDS
  RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp" "src/*.hpp" "include/*.hpp"
  "apps/*.cpp" "tools/*.cpp")
list(APPEND autoaim_provenance_sources CMakeLists.txt)
list(SORT autoaim_provenance_sources)
set(autoaim_source_inventory "")
foreach(source IN LISTS autoaim_provenance_sources)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/${source}")
  file(SHA256 "${PROJECT_SOURCE_DIR}/${source}" digest)
  string(APPEND autoaim_source_inventory "${source}:${digest}\n")
endforeach()
string(SHA256 autoaim_source_digest "${autoaim_source_inventory}")
set(autoaim_build_json "{}")
function(autoaim_build_field key value)
  string(REPLACE "\\" "\\\\" value "${value}")
  string(REPLACE "\"" "\\\"" value "${value}")
  string(REPLACE "\n" "\\n" value "${value}")
  string(REPLACE "\r" "\\r" value "${value}")
  string(REPLACE "\t" "\\t" value "${value}")
  string(JSON updated SET "${autoaim_build_json}" "${key}" "\"${value}\"")
  set(autoaim_build_json "${updated}" PARENT_SCOPE)
endfunction()
autoaim_build_field(project_version "${PROJECT_VERSION}")
autoaim_build_field(source_sha256 "${autoaim_source_digest}")
autoaim_build_field(source_inventory "${autoaim_source_inventory}")
autoaim_build_field(compiler "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
autoaim_build_field(compiler_path "${CMAKE_CXX_COMPILER}")
autoaim_build_field(cmake_version "${CMAKE_VERSION}")
autoaim_build_field(cxx_standard "${CMAKE_CXX_STANDARD}")
autoaim_build_field(build_type "${CMAKE_BUILD_TYPE}")
string(TOUPPER "${CMAKE_BUILD_TYPE}" autoaim_build_type_upper)
autoaim_build_field(cxx_flags "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${autoaim_build_type_upper}}")
autoaim_build_field(system "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
autoaim_build_field(openvino "${AUTOAIM_OPENVINO}")
autoaim_build_field(hikrobot "${AUTOAIM_HIKROBOT}")
autoaim_build_field(sanitizers "${AUTOAIM_SANITIZERS}")
autoaim_build_field(thread_sanitizer "${AUTOAIM_THREAD_SANITIZER}")
autoaim_build_field(opencv_version "${OpenCV_VERSION}")
autoaim_build_field(eigen_version "${Eigen3_VERSION}")
if(DEFINED yaml-cpp_VERSION)
  autoaim_build_field(yaml_cpp_version "${yaml-cpp_VERSION}")
else()
  autoaim_build_field(yaml_cpp_version "unknown")
endif()
if(AUTOAIM_OPENVINO)
  autoaim_build_field(openvino_version "${OpenVINO_VERSION}")
endif()
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/autoaim_build_information.hpp"
  CONTENT "#pragma once\ninline constexpr const char* autoaim_build_information =
R\"autoaim_build(${autoaim_build_json})autoaim_build\";\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/autoaim_build_information.json"
  CONTENT "${autoaim_build_json}\n")
target_include_directories(autoaim_pipeline PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_sources(autoaim_vision PRIVATE src/vision/annotation.cpp)
target_sources(autoaim_vision PRIVATE src/vision/evaluation.cpp)
target_sources(autoaim_vision PRIVATE src/vision/sequence_evaluation.cpp)
target_sources(autoaim_mission PRIVATE src/mission/button_policy.cpp)
autoaim_dependencies(math core)
autoaim_dependencies(hal core)
autoaim_dependencies(vision math)
autoaim_dependencies(estimation vision)
autoaim_dependencies(decision estimation)
autoaim_dependencies(control hal)
autoaim_dependencies(mission decision)
autoaim_dependencies(pipeline mission control hal)

foreach(entry autoaim_node offline_replay autoaim_infantry autoaim_sentry calibration_tool
  bench_detector annotate_session)
  add_executable(${entry} apps/${entry}.cpp)
  target_link_libraries(${entry} PRIVATE autoaim_pipeline)
endforeach()
add_executable(synthetic_sim tools/synthetic_sim/main.cpp)
target_link_libraries(synthetic_sim PRIVATE autoaim_pipeline)
add_executable(protocol_tester tools/protocol_tester/main.cpp)
target_link_libraries(protocol_tester PRIVATE autoaim_control)
add_executable(replay_visualizer tools/replay_visualizer/main.cpp)
target_link_libraries(replay_visualizer PRIVATE autoaim_pipeline)
add_executable(pipeline_metrics tools/pipeline_metrics/main.cpp)
target_link_libraries(pipeline_metrics PRIVATE autoaim_options)

# CTest 注册及依赖边界检查器；BUILD_TESTING=OFF 时不生成检查脚本。
function(autoaim_test name source)
  add_executable(${name} ${source})
  target_include_directories(${name} PRIVATE tests)
  target_link_libraries(${name} PRIVATE autoaim_options ${ARGN})
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 30)
endfunction()

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # Python 检查器源码在此统一维护；仅生成到构建目录，不在源码树重建 cmake/。
  # bracket 参数保留 Python 正则/转义原文，file(GENERATE) 在配置生成阶段写出。
  set(autoaim_dependency_checker "${CMAKE_CURRENT_BINARY_DIR}/check_dependencies.py")
  file(GENERATE OUTPUT "${autoaim_dependency_checker}" CONTENT [====[
"""检查项目内包含边界；不将文本检查冒充完整的 C++ 语义分析。"""
import pathlib
import re
import sys
import tempfile

EDGES = {
    "core": set(),
    "math": {"core"},
    "hal": {"core"},
    "vision": {"math"},
    "estimation": {"vision"},
    "decision": {"estimation"},
    "control": {"hal"},
    "mission": {"decision"},
    "pipeline": {"mission", "control", "hal"},
}
TOOL_EDGES = {
    "tools/synthetic_sim": {"pipeline"},
    "tools/protocol_tester": {"control"},
    "tools/replay_visualizer": {"pipeline"},
    "tools/pipeline_metrics": set(),
}


def allowed(module):
    result = {module}

    for child in (EDGES | TOOL_EDGES)[module]:
        result |= allowed(child)

    return result


def owner(path, root):
    parts = path.relative_to(root).parts

    if parts[:2] == ("include", "autoaim"):
        return parts[2]

    if parts[0] == "src":
        return parts[1]

    if parts[0] == "tools":
        return "/".join(parts[:2])

    return "pipeline"  # apps 是编排层的薄入口。


def check(root):
    failures = []
    cmake = root / "CMakeLists.txt"

    if cmake.exists():
        # 构建文件内嵌本检查器；先跳过 CMake bracket 参数，避免把自测字符串当成声明。
        content = re.sub(r"\[(=*)\[.*?\]\1\]", "", cmake.read_text(encoding="utf-8"), flags=re.S)
        content = re.sub(r"#[^\n]*", "", content)

        declarations = re.findall(r"set\(allowed_(\w+)\s*([^)]*)\)", content)
        whitelist = {name: set(deps.split()) for name, deps in declarations}
        if whitelist != EDGES or len(declarations) != len(EDGES):
            failures.append(f"{cmake}: CMake/checker dependency whitelist drift")

        links = {}
        for declaration in re.findall(r"target_link_libraries\(([^)]+)\)", content):
            target, *dependencies = declaration.split()
            if "tools/" + target in TOOL_EDGES:
                modules = {d.removeprefix("autoaim_") for d in dependencies
                           if d.startswith("autoaim_") and d != "autoaim_options"}
                links.setdefault("tools/" + target, set()).update(modules)
        for tool, modules in TOOL_EDGES.items():
            if (root / tool).exists() and links.get(tool) != modules:
                failures.append(f"{cmake}: tool link/checker drift: {tool}")

        for declaration in re.findall(r"autoaim_dependencies\(([^)]+)\)", content):
            source, *dependencies = declaration.split()
            for destination in dependencies:
                if destination not in EDGES.get(source, set()):
                    failures.append(f"{cmake}: forbidden target {source} -> {destination}")

    for directory in ("include/autoaim", "src", "apps", "tools"):
        for path in (root / directory).rglob("*"):
            if path.suffix not in (".hpp", ".cpp", ".h"):
                continue

            source = owner(path, root)
            content = re.sub(
                r"/\*.*?\*/|//[^\n]*", "", path.read_text(encoding="utf-8"), flags=re.S
            )

            for name in re.findall(r'^\s*#\s*include\s*[<"]([^>"\n]+)[>"]', content, re.M):
                candidates = (path.parent / name, root / "include" / name, root / name)
                target = next((p.resolve() for p in candidates if p.is_file()), None)

                if target is None:
                    if name.startswith("autoaim/"):
                        failures.append(f"{path}: missing {name}")
                    continue

                if root not in target.parents:
                    failures.append(f"{path}: project include escapes root: {name}")
                    continue

                destination = owner(target, root)
                if source not in (EDGES | TOOL_EDGES) or destination not in allowed(source):
                    failures.append(f"{path}: forbidden {source} -> {destination}: {name}")

    return failures


def self_test():
    # 临时负例证明检查器能拒绝逆向依赖和相对路径绕过；不修改项目源码。
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp).resolve()
        core = root / "include/autoaim/core/a.hpp"
        control = root / "include/autoaim/control/b.hpp"
        core.parent.mkdir(parents=True)
        control.parent.mkdir(parents=True)
        control.write_text('#include "autoaim/core/a.hpp"', encoding="utf-8")
        core.write_text("", encoding="utf-8")

        if check(root):
            raise RuntimeError("checker rejected permitted edge")

        for name in ("autoaim/control/b.hpp", "../control/b.hpp"):
            core.write_text(f'#include "{name}"', encoding="utf-8")
            if not check(root):
                raise RuntimeError("checker missed forbidden edge")

        core.write_text("", encoding="utf-8")
        cmake = root / "CMakeLists.txt"
        whitelist = "\n".join(f"set(allowed_{name} {' '.join(sorted(deps))})"
                              for name, deps in EDGES.items()) + "\n"
        cmake.write_text(whitelist + "autoaim_dependencies(control hal)", encoding="utf-8")
        if check(root):
            raise RuntimeError("checker rejected permitted target edge")

        cmake.write_text(whitelist + "autoaim_dependencies(hal vision)", encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker missed forbidden target edge")

        embedded = "file(GENERATE OUTPUT checker.py CONTENT [==[autoaim_dependencies(hal vision)]==])\n"
        cmake.write_text(whitelist + embedded + "autoaim_dependencies(control hal)",
                         encoding="utf-8")
        if check(root):
            raise RuntimeError("checker interpreted embedded script text as a target edge")

        cmake.write_text(whitelist + embedded + "autoaim_dependencies(hal vision)",
                         encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker missed forbidden target edge after embedded script")

        cmake.write_text(whitelist.replace("set(allowed_hal core)", "set(allowed_hal vision)"),
                         encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker missed whitelist drift")

        tool = root / "tools/protocol_tester/main.cpp"
        tool.parent.mkdir(parents=True)
        tool.write_text('#include "autoaim/control/b.hpp"', encoding="utf-8")
        permitted = whitelist + "target_link_libraries(protocol_tester PRIVATE autoaim_control)"
        cmake.write_text(permitted, encoding="utf-8")
        if check(root):
            raise RuntimeError("checker rejected tool linked capability")
        vision = root / "include/autoaim/vision/c.hpp"
        vision.parent.mkdir(parents=True)
        vision.write_text("", encoding="utf-8")
        tool.write_text('#include "autoaim/vision/c.hpp"', encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker granted all modules to tool")
        tool.write_text('#include "autoaim/control/b.hpp"', encoding="utf-8")
        cmake.write_text(permitted.replace("autoaim_control", "autoaim_options"), encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker missed tool link drift")


if __name__ == "__main__":
    if sys.argv[1] == "--self-test":
        self_test()
    else:
        errors = check(pathlib.Path(sys.argv[1]).resolve())
        print("\n".join(errors) if errors else "Module include boundaries passed")
        sys.exit(bool(errors))
]====])

  if(UNIX AND NOT APPLE)
    add_test(NAME test_install_script COMMAND ${Python3_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/tests/unit/test_install_script.py)
    set_tests_properties(test_install_script PROPERTIES TIMEOUT 30)
  endif()

  add_test(NAME module_boundaries COMMAND ${Python3_EXECUTABLE}
    "${autoaim_dependency_checker}" "${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME boundary_checker_self_test COMMAND ${Python3_EXECUTABLE}
    "${autoaim_dependency_checker}" --self-test)
  add_test(NAME test_verification_manifest COMMAND ${Python3_EXECUTABLE}
    "${PROJECT_SOURCE_DIR}/tests/verification_manifest.py" --self-test)
  set_tests_properties(test_verification_manifest PROPERTIES TIMEOUT 30)
  # 显式构建此目标保存验收材料；它列出测试，不运行测试或改变运行报告指纹。
  add_custom_target(verification_manifest
    COMMAND ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tests/verification_manifest.py"
      --source "${PROJECT_SOURCE_DIR}" --build "${CMAKE_CURRENT_BINARY_DIR}"
      --ctest "${CMAKE_CTEST_COMMAND}" --configuration "$<CONFIG>"
      --output "${CMAKE_CURRENT_BINARY_DIR}/verification_manifest.json"
    VERBATIM)

  autoaim_test(test_smoke tests/unit/test_smoke.cpp)
  autoaim_test(test_result tests/unit/test_result.cpp autoaim_core)
  autoaim_test(test_time tests/unit/test_time.cpp autoaim_core)
  autoaim_test(test_stamp tests/unit/test_stamp.cpp autoaim_core)
  autoaim_test(test_evidence tests/unit/test_evidence.cpp autoaim_core)
  autoaim_test(test_logging tests/unit/test_logging.cpp autoaim_core)
  autoaim_test(test_config tests/unit/test_config.cpp autoaim_core)

  autoaim_test(test_angle tests/unit/test_angle.cpp autoaim_math)
  autoaim_test(test_numeric tests/unit/test_numeric.cpp autoaim_math)
  autoaim_test(test_se3 tests/unit/test_se3.cpp autoaim_math)
  autoaim_test(test_transform tests/unit/test_transform.cpp autoaim_math)

  autoaim_test(test_data_contracts tests/contract/test_data_contracts.cpp autoaim_pipeline)
  autoaim_test(test_hal tests/unit/test_hal.cpp autoaim_hal)
  autoaim_test(test_frame_sync tests/contract/test_frame_sync.cpp autoaim_pipeline)
  autoaim_test(test_async_buffer_lifetime tests/contract/test_async_buffer_lifetime.cpp
    autoaim_core)
  autoaim_test(test_frame_queue tests/contract/test_frame_queue.cpp autoaim_pipeline)
  autoaim_test(test_out_of_order_completion tests/contract/test_out_of_order_completion.cpp
    autoaim_pipeline)
  autoaim_test(test_command_slot tests/contract/test_command_slot.cpp autoaim_pipeline)
  autoaim_test(test_worker_exception tests/contract/test_worker_exception.cpp autoaim_pipeline)

  autoaim_test(test_crc tests/unit/test_crc.cpp autoaim_control)
  autoaim_test(test_command tests/unit/test_command.cpp autoaim_control)
  autoaim_test(test_protocol tests/unit/test_protocol.cpp autoaim_control)
  autoaim_test(test_watchdog tests/unit/test_watchdog.cpp autoaim_control)
  autoaim_test(test_smoother tests/unit/test_smoother.cpp autoaim_control)
  autoaim_test(test_mode_switch_staleness tests/contract/test_mode_switch_staleness.cpp
    autoaim_control)
  autoaim_test(test_publish_failure_shutdown tests/contract/test_publish_failure_shutdown.cpp
    autoaim_control)
  autoaim_test(test_calibration tests/unit/test_calibration.cpp autoaim_vision)
  autoaim_test(test_calibration_dataset tests/unit/test_calibration_dataset.cpp autoaim_vision)
  autoaim_test(test_intrinsic_solver tests/unit/test_intrinsic_solver.cpp autoaim_vision)
  autoaim_test(test_hand_eye_solver tests/unit/test_hand_eye_solver.cpp autoaim_vision)
  autoaim_test(test_calibration_report tests/unit/test_calibration_report.cpp autoaim_vision)
  autoaim_test(test_calibration_solver_cli tests/unit/test_calibration_solver_cli.cpp autoaim_pipeline)
  autoaim_test(test_traditional_detector tests/unit/test_traditional_detector.cpp autoaim_vision)
  autoaim_test(test_yolov5 tests/unit/test_yolov5.cpp autoaim_vision)
  autoaim_test(test_yolo11 tests/unit/test_yolo11.cpp autoaim_vision)
  autoaim_test(test_annotation tests/unit/test_annotation.cpp autoaim_vision)
  autoaim_test(test_evaluation tests/unit/test_evaluation.cpp autoaim_vision)
  autoaim_test(test_sequence_evaluation tests/unit/test_sequence_evaluation.cpp autoaim_vision)
  autoaim_test(test_batch_benchmark tests/unit/test_batch_benchmark.cpp autoaim_pipeline)
  autoaim_test(test_annotate_session tests/unit/test_annotate_session.cpp autoaim_pipeline)
  autoaim_test(test_evaluation_golden tests/synthetic/test_evaluation_golden.cpp autoaim_pipeline)
  autoaim_test(test_detection_labels tests/unit/test_detection_labels.cpp autoaim_vision)
  autoaim_test(test_corner_semantics tests/unit/test_corner_semantics.cpp autoaim_vision)
  autoaim_test(test_corner_refine tests/unit/test_corner_refine.cpp autoaim_vision)
  autoaim_test(test_pnp tests/unit/test_pnp.cpp autoaim_vision)
  autoaim_test(test_pose_quality tests/unit/test_pose_quality.cpp autoaim_vision)
  autoaim_test(test_observation tests/unit/test_observation.cpp autoaim_estimation)
  autoaim_test(test_geometry tests/unit/test_geometry.cpp autoaim_estimation)
  autoaim_test(test_motion_model tests/unit/test_motion_model.cpp autoaim_estimation)
  autoaim_test(test_acceleration_model tests/unit/test_acceleration_model.cpp autoaim_estimation)
  autoaim_test(test_measurement_model tests/unit/test_measurement_model.cpp autoaim_estimation)
  autoaim_test(test_ekf tests/unit/test_ekf.cpp autoaim_estimation)
  autoaim_test(test_health tests/unit/test_health.cpp autoaim_estimation)
  autoaim_test(test_association tests/unit/test_association.cpp autoaim_estimation)
  autoaim_test(test_armor_id_synthetic tests/synthetic/test_armor_id_synthetic.cpp
    autoaim_estimation)
  autoaim_test(test_geometry_selector_synthetic
    tests/synthetic/test_geometry_selector_synthetic.cpp autoaim_estimation)
  autoaim_test(test_state_machine tests/unit/test_state_machine.cpp autoaim_estimation)
  autoaim_test(test_motion_selection tests/unit/test_motion_selection.cpp autoaim_estimation)
  autoaim_test(test_tracker tests/synthetic/test_tracker.cpp autoaim_estimation)
  autoaim_test(test_tracker_set tests/synthetic/test_tracker_set.cpp autoaim_estimation)
  autoaim_test(test_predictor tests/unit/test_predictor.cpp autoaim_decision)
  autoaim_test(test_ballistic tests/unit/test_ballistic.cpp autoaim_decision)
  autoaim_test(test_intercept tests/unit/test_intercept.cpp autoaim_decision)
  autoaim_test(test_armor_selector tests/unit/test_armor_selector.cpp autoaim_decision)
  autoaim_test(test_aim_adequacy tests/unit/test_aim_adequacy.cpp autoaim_decision)
  autoaim_test(test_mission_factory tests/unit/test_mission_factory.cpp autoaim_mission)
  autoaim_test(test_target_selector tests/unit/test_target_selector.cpp autoaim_mission)
  autoaim_test(test_infantry_assist tests/unit/test_infantry_assist.cpp autoaim_mission)
  autoaim_test(test_button_policy tests/unit/test_button_policy.cpp autoaim_mission)
  autoaim_test(test_button_mode tests/contract/test_button_mode.cpp autoaim_mission)
  autoaim_test(test_manual_takeover tests/contract/test_manual_takeover.cpp autoaim_mission)
  autoaim_test(test_sentry_mission tests/unit/test_sentry_mission.cpp autoaim_mission)
  autoaim_test(test_file_replay tests/unit/test_file_replay.cpp autoaim_hal)
  autoaim_test(test_session_writer tests/unit/test_session_writer.cpp autoaim_hal)
  autoaim_test(test_recording_transport tests/unit/test_recording_transport.cpp autoaim_hal)
  autoaim_test(test_serial_disconnected tests/unit/test_serial_disconnected.cpp autoaim_hal)
  autoaim_test(test_can_disconnected tests/unit/test_can_disconnected.cpp autoaim_hal)
  autoaim_test(test_camera_disconnected tests/unit/test_camera_disconnected.cpp autoaim_hal)
  autoaim_test(test_bootstrap tests/unit/test_bootstrap.cpp autoaim_pipeline)
  autoaim_test(test_uart_adapter tests/unit/test_uart_adapter.cpp autoaim_pipeline)
  autoaim_test(test_uart_writer tests/contract/test_uart_writer.cpp autoaim_pipeline)
  autoaim_test(test_replay_publisher tests/contract/test_replay_publisher.cpp autoaim_control)
  autoaim_test(test_pipeline_lifecycle tests/contract/test_pipeline_lifecycle.cpp autoaim_pipeline)
  autoaim_test(test_pipeline_chain tests/contract/test_pipeline_chain.cpp autoaim_pipeline)
  autoaim_test(test_pipeline_mode_exit tests/contract/test_pipeline_mode_exit.cpp autoaim_pipeline)
  autoaim_test(test_pipeline_button_uart tests/contract/test_pipeline_button_uart.cpp autoaim_pipeline)
  autoaim_test(test_file_pipeline tests/contract/test_file_pipeline.cpp autoaim_pipeline)
  add_test(NAME test_entry_help COMMAND autoaim_node --help)
  add_test(NAME test_synthetic_tool COMMAND synthetic_sim --config
    ${PROJECT_SOURCE_DIR}/config/offline/armor.yaml --self-test)
  add_test(NAME test_protocol_tool COMMAND protocol_tester --self-test)
  add_test(NAME test_protocol_tool_default_uart COMMAND protocol_tester --encode-stop)
  set_tests_properties(test_protocol_tool_default_uart PROPERTIES
    PASS_REGULAR_EXPRESSION
      "^protocol=2 bytes=14 a5 0e 00 00 00 00 00 00 00 00 aa d5 91 78")
  add_test(NAME test_calibration_tool COMMAND calibration_tool --config
    ${PROJECT_SOURCE_DIR}/config/offline/armor.yaml --self-test)
  add_test(NAME test_detector_benchmark COMMAND bench_detector --config
    ${PROJECT_SOURCE_DIR}/config/offline/armor.yaml --self-test)
  add_test(NAME test_visualizer_help COMMAND replay_visualizer --help)
  add_test(NAME test_metrics_tool COMMAND pipeline_metrics --self-test)
  add_test(NAME test_annotate_tool COMMAND annotate_session --self-test)
  add_test(NAME test_live_disabled COMMAND autoaim_node --config
    ${PROJECT_SOURCE_DIR}/config/hardware/disabled.yaml)
  set_tests_properties(test_live_disabled PROPERTIES WILL_FAIL TRUE TIMEOUT 5)
  autoaim_test(test_hit_probability_synthetic tests/synthetic/test_hit_probability_synthetic.cpp
    autoaim_decision)
  set(AUTOAIM_TEST_YOLOV5_MODEL "" CACHE FILEPATH
    "Optional explicit model path for offline inference smoke test")
  set(AUTOAIM_TEST_YOLO11_MODEL "" CACHE FILEPATH
    "Optional explicit armor YOLO11 model for offline CPU tests")
  if(AUTOAIM_OPENVINO AND (AUTOAIM_TEST_YOLOV5_MODEL OR AUTOAIM_TEST_YOLO11_MODEL))
    add_executable(test_openvino_async tests/contract/test_openvino_async.cpp)
    target_include_directories(test_openvino_async PRIVATE tests)
    target_link_libraries(test_openvino_async PRIVATE autoaim_vision)
    add_executable(test_pipeline_async tests/contract/test_pipeline_async.cpp)
    target_include_directories(test_pipeline_async PRIVATE tests)
    target_link_libraries(test_pipeline_async PRIVATE autoaim_pipeline)
    if(AUTOAIM_TEST_YOLOV5_MODEL)
      add_test(NAME test_openvino_model COMMAND test_yolov5 "${AUTOAIM_TEST_YOLOV5_MODEL}")
      add_test(NAME test_openvino_async COMMAND test_openvino_async "${AUTOAIM_TEST_YOLOV5_MODEL}")
      add_test(NAME test_pipeline_async COMMAND test_pipeline_async "${AUTOAIM_TEST_YOLOV5_MODEL}")
      set_tests_properties(test_openvino_model test_openvino_async test_pipeline_async
        PROPERTIES TIMEOUT 60)
    endif()
    if(AUTOAIM_TEST_YOLO11_MODEL)
      add_test(NAME test_openvino_yolo11_model COMMAND test_yolo11 "${AUTOAIM_TEST_YOLO11_MODEL}")
      add_test(NAME test_openvino_yolo11_async COMMAND test_openvino_async
        "${AUTOAIM_TEST_YOLO11_MODEL}" yolo11)
      add_test(NAME test_pipeline_yolo11_async COMMAND test_pipeline_async
        "${AUTOAIM_TEST_YOLO11_MODEL}" yolo11)
      set_tests_properties(test_openvino_yolo11_model test_openvino_yolo11_async
        test_pipeline_yolo11_async PROPERTIES TIMEOUT 60)
    endif()
    if(AUTOAIM_TEST_YOLOV5_MODEL AND AUTOAIM_TEST_YOLO11_MODEL)
      add_test(NAME test_batch_external_models COMMAND test_batch_benchmark
        "${AUTOAIM_TEST_YOLOV5_MODEL}" "${AUTOAIM_TEST_YOLO11_MODEL}")
      set_tests_properties(test_batch_external_models PROPERTIES TIMEOUT 120)
    endif()
  endif()
  add_test(NAME test_check_failure COMMAND test_smoke --fail)
  set_tests_properties(test_check_failure PROPERTIES WILL_FAIL TRUE TIMEOUT 5)
endif()
