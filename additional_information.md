# XTYF-AutoAim 补充手册：模块、规格与协作约定

<a id="quick-algorithm-swap"></a>

## 快速替换

**所有替换入口集中在根 [CMakeLists.txt](CMakeLists.txt)，六个开关默认均为 `OFF`。**
默认仍使用原角点精修、IPPE、平移 CV 的 EKF、OpenVINO `LATENCY` 和 deque 队列。
启用或回退只需重新配置对应构建目录并编译，不修改 cpp/hpp、块注释或估计器别名；
`config/fast_choose.yaml` 负责运行参数，不负责选择这些编译期算法。
修改根 CMake 中 `option()` 的默认值只影响新的构建目录；已有目录会沿用缓存，
必须用下表的 `-D...=ON/OFF` 显式覆盖，或先清除对应缓存项，再重新配置。

| 方案 | CMake 开关与实现位置 | 启用操作 | 回退操作 | 验证入口 |
| --- | --- | --- | --- | --- |
| [I1-CONTRAST-IRLS](#alternative-i1-contrast-irls) | `AUTOAIM_I1_CONTRAST_IRLS`；[corner_refine.cpp](src/vision/corner_refine.cpp) | `-DAUTOAIM_I1_CONTRAST_IRLS=ON` | `-DAUTOAIM_I1_CONTRAST_IRLS=OFF` | `--method I1-CONTRAST-IRLS` |
| [I2-LM](#alternative-i2-lm) | `AUTOAIM_I2_LM`；[pnp.cpp](src/vision/pnp.cpp) | `-DAUTOAIM_I2_LM=ON` | `-DAUTOAIM_I2_LM=OFF` | `--method I2-LM` |
| [I3-LINEAR-CA](#alternative-i3-linear-ca) | `AUTOAIM_I3_LINEAR_CA`；[bootstrap.cpp](src/pipeline/bootstrap.cpp) | `-DAUTOAIM_I3_LINEAR_CA=ON` | `-DAUTOAIM_I3_LINEAR_CA=OFF` | `--method I3-LINEAR-CA` |
| [I9-THROUGHPUT](#alternative-i9-throughput) | `AUTOAIM_I9_THROUGHPUT`；[detector_openvino.cpp](src/vision/detector_openvino.cpp) | `-DAUTOAIM_I9_THROUGHPUT=ON`，同时开启 OpenVINO | `-DAUTOAIM_I9_THROUGHPUT=OFF` | `--method I9-THROUGHPUT` |
| [I9-PREALLOC](#alternative-i9-prealloc) | `AUTOAIM_I9_PREALLOC`；[queue.cpp](src/pipeline/queue.cpp) | `-DAUTOAIM_I9_PREALLOC=ON` | `-DAUTOAIM_I9_PREALLOC=OFF` | `--method I9-PREALLOC` |
| [ESO](#ekf-与-eso-的替换方法) | `AUTOAIM_USE_ESO`；[state_estimator.hpp](include/autoaim/estimation/state_estimator.hpp) | `-DAUTOAIM_USE_ESO=ON` | `-DAUTOAIM_USE_ESO=OFF` | `--method ESO`，兼容 `verify_eso.py` |

**每个替换点的生产调用只使用一份实现；原算法对照仅供测试。** `AUTOAIM_I9_THROUGHPUT=ON` 与 `AUTOAIM_OPENVINO=OFF`
同时出现时配置失败。`AUTOAIM_I3_LINEAR_CA=ON` 与 `AUTOAIM_USE_ESO=ON` 也会被拒绝：
ESO 使用自己的平移 jerk 参数，会覆盖 I3 的实验模型，不能作为有效的 EKF 对照。
其它组合能配置不代表已经通过组合验证。详细算法、缺口与限制见[后文](#algorithm-alternatives)。

以下在 WSL/Linux 的仓库根目录执行。直接使用 CMake 的独立构建示例：

```bash
algorithm_builds=$(mktemp -d /tmp/autoaim-algorithm-builds.XXXXXX)
algorithm_defaults=(
  -DAUTOAIM_I1_CONTRAST_IRLS=OFF -DAUTOAIM_I2_LM=OFF
  -DAUTOAIM_I3_LINEAR_CA=OFF -DAUTOAIM_I9_THROUGHPUT=OFF
  -DAUTOAIM_I9_PREALLOC=OFF -DAUTOAIM_USE_ESO=OFF
)
# 单独选择 I1；其它方案使用表内对应 ON 参数。两个 I9 的双模型验收命令见下方。
cmake -S . -B "$algorithm_builds/i1" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=17 \
  -DBUILD_TESTING=ON -DAUTOAIM_OPENVINO=OFF -DAUTOAIM_HIKROBOT=OFF \
  "${algorithm_defaults[@]}" -DAUTOAIM_I1_CONTRAST_IRLS=ON
cmake --build "$algorithm_builds/i1" --parallel 2
ctest --test-dir "$algorithm_builds/i1" --output-on-failure
# 回退此构建目录：六个候选恢复 OFF，重新编译和验证。
cmake -S . -B "$algorithm_builds/i1" "${algorithm_defaults[@]}"
cmake --build "$algorithm_builds/i1" --parallel 2
ctest --test-dir "$algorithm_builds/i1" --output-on-failure
```

需要保留源码摘要、完整命令和日志时使用统一验证脚本。脚本复制当前源码，但不改写副本源码；
每次显式关闭六个候选，再仅启用指定项，由 CMake 注册相应专项测试。输出目录须全新且在源码之外：

```bash
swap_runs=$(mktemp -d /tmp/autoaim-alternatives.XXXXXX)
python3 tests/verify_alternatives.py --self-test
python3 tests/verify_alternatives.py --source . --method DEFAULT \
  --output "$swap_runs/default" --jobs 2
python3 tests/verify_alternatives.py --source . --method I1-CONTRAST-IRLS \
  --output "$swap_runs/i1-contrast-irls" --jobs 2
python3 tests/verify_alternatives.py --source . --method I2-LM \
  --output "$swap_runs/i2-lm" --jobs 2
python3 tests/verify_alternatives.py --source . --method I3-LINEAR-CA \
  --output "$swap_runs/i3-linear-ca" --jobs 2
python3 tests/verify_alternatives.py --source . --method ESO \
  --output "$swap_runs/eso" --jobs 2

# 本机已有模型路径示例；其它机器改为实际 XML，配套 BIN 必须同时存在。
model_dir='/mnt/e/大学/巡天御风/视觉组培训/RM2026-AutoAim/assets'
python3 tests/verify_alternatives.py --source . --method I9-THROUGHPUT \
  --output "$swap_runs/i9-throughput" --jobs 2 \
  --openvino-dir /usr/lib/cmake/openvino2026.3.1 \
  --yolov5-model "$model_dir/yolov5.xml" --yolo11-model "$model_dir/yolo11.xml"
python3 tests/verify_alternatives.py --source . --method I9-PREALLOC \
  --output "$swap_runs/i9-prealloc" --jobs 2 \
  --openvino-dir /usr/lib/cmake/openvino2026.3.1 \
  --yolov5-model "$model_dir/yolov5.xml" --yolo11-model "$model_dir/yolo11.xml"
```

`tests/verify_eso.py` 保留原 CLI，作为统一脚本的 ESO 兼容入口。查看各目录的 `summary.json`、
`CMakeCache.txt`、构建报告及测试日志；构建报告包含六个开关的实际值。`/tmp` 材料需归档。
统一脚本默认使用 C++17 Debug，可追加 `--build-type Release`；本次集中入口验收使用
C++17 Release，实际结果以记录为准。脚本显式设置全部六项，不受源码中默认值改动影响。
六个候选均已单项构建并通过相关全量回归，标记为“可替换且已验证（软件）”。
当前集中选择版本的环境、源码指纹与测试数见[集中入口验收](build_history.md#central-selection-20260930)，
不沿用此前手动解除注释版本的通过记录。软件验证不代表目标 NUC 性能或实物效果。

<a id="fast-choose"></a>

## 快速调参：config/fast_choose.yaml

日常参数统一在 [fast_choose.yaml](config/fast_choose.yaml) 调整，**启动读取，修改后重启生效**。
从仓库根目录运行：

```bash
build-debug/autoaim_node --config config/fast_choose.yaml
```

- `active_detector` 选择 `traditional`、`yolov5` 或 `yolo11`，默认 yolov5；
  默认运行需要 OpenVINO 与 `detectors.yolov5.model_path` 指定的模型及配套权重。
  对应 `detectors.<类型>.config_file` 指向基础契约，阈值和模型路径直接写在同组。
- `common` 是输入、检测颜色、跟踪、预测、选板、控制及容量参数的唯一来源；配置注释标明
  单位、合法范围及增减影响。基础配置保留标定、几何、身份、角点、板型、坐标与重力契约。
- `eso` 的带宽与 jerk PSD 经 Tracker 传入估计器。默认 EKF 不使用这些参数；ESO 启用方式
  由 CMake 的 `AUTOAIM_USE_ESO` 控制，调参文件不能选择编译期算法。
- 原 `--config config/offline/armor.yaml`、`yolov5.yaml`、`yolo11.yaml` 固定选择其检测器，
  忽略快调文件的 `active_detector`，但使用同一份快调参数。日常一键切换请使用上面的统一入口。

快调格式标记为 `fast_choose_version: 1`。基础文件通过 `fast_choose_file: ../fast_choose.yaml`
引用它；两者的同一调参叶子不能重复定义，不存在“改了却被另一份默认值覆盖”的优先级。
未知快调键、重复键、缺项及非法参数在启动时报告。没有快调引用的旧完整配置继续兼容。
只检查实际选择的模型文件与后端，traditional 不要求备用 YOLO 权重存在。

相对路径按**声明文件**解析：快调里的 `offline/armor.yaml`、`../models/yolo11.xml` 和
`../out/synthetic/events.yaml` 相对 `config/`；基础文件的标定/几何引用相对 `config/offline/`。
CLI `--input` 与批量 `--dataset` 仍覆盖最终输入，采用原有工作目录语义。配置文件不递归继承。
迁移的是离线默认值，调整按钮模式或开火请求不会提供合格证据、提升资格或开放设备入口。

批量命令在运行第一项之前预加载全部配置，同一路径快调文件共享一份不可变启动快照。
`run_metadata_schema_version: 3` 同时记录入口、基础和快调原文/字节指纹、有效配置及估计器，
有效输入反映 CLI 覆盖；运行中改文件不改变已加载快照，下次启动才生效。只改注释也会改变
文件指纹，但不改变有效参数。标定报告和模型仍沿用原有来源记录规则。

## EKF 与 ESO 的替换方法

**默认 `AUTOAIM_USE_ESO=OFF` 使用 EKF。** 开关为 `ON` 时 CMake 选择 ESO，统一头文件
据编译定义选择 `StateEstimator`；不需要手动修改别名或源码。公共状态保持 12 维。

先运行[快速替换](#quick-algorithm-swap)中的 `--method ESO` 独立验证，也可使用兼容命令：

```bash
# 输出目录必须尚不存在，且位于项目目录之外。
python3 tests/verify_eso.py --source . --output /tmp/autoaim-eso-check --jobs 2
```

兼容入口同样只通过 CMake 开关选择算法，不改变原源码或副本源码。
`BUILD_TESTING=ON` 且 `AUTOAIM_USE_ESO=ON` 时自动注册 `test_eso`；
EKF 专项仍直接测试 `Ekf`。切回时将同一构建目录的 `AUTOAIM_USE_ESO` 设为 `OFF`，
重新编译并运行 CTest。CMake 缓存会保留上次选择，不能以省略参数代替显式回退。

ESO 参数来自 `config/fast_choose.yaml` 的 `eso` 分组，仍使用现有参数接口；
设置参数不会自行启用 ESO。I3-LINEAR-CA 与 ESO 同时启用会在配置阶段报错。
ESO 只接受当前 6 维完整位姿观测，使用平移 CA 与批先验门控；默认 EKF 保持平移 CV。
两者数值结果不要求相同。实验带宽、融合规则及限制见 [§5.6](#experimental-eso)，
当前集中选择版本的验收见[构建历史](build_history.md#central-selection-20260930)。

---

[README.md](README.md) 负责入门、构建运行、配置工具和简明导航；本手册集中说明九模块的文件职责、
技术契约、协作规则及实施历史。[build_history.md](build_history.md) 集中保存从 README 迁出的
构建日志、测试矩阵和逐批修复记录。文档互相链接，不重复维护逐文件说明。

**当前交付边界是软件离线链路，不是设备已上线。** 2026-09-30 续修完成 C++17
Debug / Release、C++20、ASan/UBSan、现有 YOLOv5/YOLO11 离线回归及 SDK 编译。
本次矩阵见 [§11.6](#history-repair)；TSan 因运行时内存映射错误未验收，多配置未验证。
实机缺口见 [§9](#unverified)。

## 阅读导航

| 你想了解什么 | 从哪里开始 |
| --- | --- |
| 第一次使用，如何编译和回放 | [README 的入门示例](README.md#2-编译并跑通第一个示例) |
| 哪个模块、哪个文件负责什么 | [M1—M9：九模块与文件详解](#module-guide) |
| 修改前必须遵守哪些边界 | [A1—A8：协作与工程约定](#engineering-rules) |
| 时间、数据、算法和控制要求 | [§1—§10：技术规格与验收要求](#technical-specification) |
| 已确认的选择与仍缺的实测 | [§10](#decisions)，其中 [§10.4](#spec-status-notes) 记录旧表述的待核查点 |
| 过去做过什么、验证到了哪里 | [§11：实施与验收历史](#implementation-history) |
| 构建环境、测试矩阵与逐批日志 | [构建与验证记录](build_history.md) |

M 编号只用于文件导航，A 编号用于协作规则，§编号用于技术规格和历史章节，互不混用。
规格中的 **[定]** 是已确认设计，**[推]** 是可调整实现取舍，**[开]** 是尚缺具体选择，
**[未验]** 是尚无验证证据；批准设计不等于功能已实现，更不等于设备已实测。
历史批准只说明相应阶段的授权和结果，不能自动授权新的任务；后续修改以用户当次要求为准。

---

<a id="module-guide"></a>

## 九模块与文件详解

九个模块各自编译成 `autoaim_<模块名>` 静态库。**模块不是九个独立运行的程序，也不是运行时加载的插件。**
应用选择配置后由 pipeline 组合它们；依赖由根 CMake 声明：

```text
apps → pipeline
         ├→ mission → decision → estimation → vision → math → core
         ├→ control → hal → core
         └→ hal
```

箭头表示“允许依赖谁”，不是执行时间顺序。HAL 不依赖 vision/control；
字节传输、协议解释和业务决策分别放在不同层，不能为了方便互相包含。

<a id="module-core"></a>

### M1. core：基础数据、时间和内存

所有模块都会用到的基础能力，不负责识别装甲板。

| 文件 | 负责什么 |
| --- | --- |
| [result.hpp](include/autoaim/core/result.hpp)、[result.cpp](src/core/result.cpp) | 成功/失败结果及错误类型，让调用者处理失败原因 |
| [units.hpp](include/autoaim/core/units.hpp)、[time.hpp](include/autoaim/core/time.hpp)、[time.cpp](src/core/time.cpp) | 米、弧度、秒等单位；区分主机单调时间和回放逻辑时间，计算数据年龄 |
| [types.hpp](include/autoaim/core/types.hpp) | 帧号、世代号、Stamp 来源信息，以及角色/模式等公共枚举 |
| [image.hpp](include/autoaim/core/image.hpp) | 图片字节、尺寸、步长和 CapturedFrame 的曝光/接收信息；不是图像检测算法 |
| [buffer_pool.hpp](include/autoaim/core/buffer_pool.hpp)、[buffer_pool.cpp](src/core/buffer_pool.cpp) | 固定容量图像内存池与租约；最后一个使用者释放后才能复用内存 |
| [evidence.hpp](include/autoaim/core/evidence.hpp) | 区分缺失、声明、实测、模拟证据；配置里写 true 不能变成实测 |
| [config.cpp](src/core/config.cpp)、[logging.cpp](src/core/logging.cpp) | 通用 YAML 读取/错误报告与基础日志；业务参数解释放在 bootstrap |
| [fingerprint.hpp](include/autoaim/core/fingerprint.hpp) | 仅头文件 FNV-1a64 字节累计与 uint64 字节计数；路径、字符串编码和格式化由调用方负责 |

文件来源与会话文件保持 binary、64 KiB 缓冲及 EOF/错误检查；图片去重逐行累计
解码后的灰度像素，不包含行间 padding。标定点保持点序、x/y 顺序、IEEE754 检查、
正负零归一和四字节小端编码，输出仍是 16 位小写十六进制加冒号、十进制字节数。
会话集合继续在 pipeline 按规范化路径排序，每个字符串先写八字节小端长度，
再写原始字节；文件长度的十进制字符串、摘要的十六进制字符串也按该规则参与累计。
`fnv1a64`、`fnv1a64-length-prefixed-v1` 及各调用方 locale 行为保持不变；
共享基元不解释这些格式，不增加长度前缀或字符串重载。

<a id="module-math"></a>

### M2. math：坐标和数值计算

解决“这个位置/旋转在哪个坐标系里、如何转换”，不直接接触相机或命令发送。

| 文件 | 负责什么 |
| --- | --- |
| [angle.cpp](src/math/angle.cpp) | 角度归一化与跨 ±π 的误差计算，避免把相邻方向算成接近一整圈 |
| [numeric.cpp](src/math/numeric.cpp) | 有限数、矩阵性质等数值检查 |
| [se3.cpp](src/math/se3.cpp) | 旋转和平移组成的刚体变换：组合、求逆、作用于点 |
| [transform.hpp](include/autoaim/math/transform.hpp)、[transform.cpp](src/math/transform.cpp) | 带相机/云台/世界/板坐标标签的变换，以及插值等操作 |

<a id="module-hal"></a>

### M3. hal：输入输出与设备隔离

HAL 是“硬件抽象层”。上层通过统一接口取得图片、时间和输入，或写出字节；
离线时换成文件和记录端，不必换一套检测算法。

| 文件 | 负责什么 |
| --- | --- |
| [clock.hpp](include/autoaim/hal/clock.hpp)、[clock_impl.cpp](src/hal/clock_impl.cpp) | 实时时钟和可推进的回放逻辑时钟 |
| [camera.hpp](include/autoaim/hal/camera.hpp)、[hikrobot_camera.hpp](include/autoaim/hal/hikrobot_camera.hpp) | 相机接口、请求参数、回读参数及帧元数据 |
| [camera_parameters.cpp](src/hal/camera_parameters.cpp)、[camera_impl.cpp](src/hal/camera_impl.cpp) | 校验尺寸/ROI/像素格式等参数；可选海康 SDK 的设置、回读、取帧和 BGR8 转换 |
| [gimbal_feedback.hpp](include/autoaim/hal/gimbal_feedback.hpp) | 云台姿态/状态、操作输入、独立按键和原始 UART 块的数据结构 |
| [file_replay.cpp](src/hal/file_replay.cpp) | 读取 YAML 事件与本地图片，按原顺序推进逻辑时钟；查询逐帧标注 |
| [session_writer.cpp](src/hal/session_writer.cpp) | 将原始图片/事件和派生结果分开录制；完成标志只在成功结束时写出 |
| [transport.hpp](include/autoaim/hal/transport.hpp)、[recording_transport.cpp](src/hal/recording_transport.cpp) | 字节传输接口、完整/失败写入结果，以及不连接设备的记录实现 |
| [serial_transport.cpp](src/hal/serial_transport.cpp)、[can_transport.cpp](src/hal/can_transport.cpp) | Linux 串口/CAN 传输后端；不负责判断目标或开火许可 |

`src/hal/gimbal_feedback_impl.cpp` 是旧空占位，不参与构建；实际协议到反馈的转换在 pipeline 的 `uart_adapter.cpp`，不要往占位文件里找解析代码。

<a id="module-vision"></a>

### M4. vision：从图片到装甲板观测

输入图片及标定信息，输出二维检测和三维位姿候选；“算出位置”与“位置足够可靠”是两个标志。

| 文件 | 负责什么 |
| --- | --- |
| [frame_packet.hpp](include/autoaim/vision/frame_packet.hpp)、[frame.hpp](include/autoaim/vision/frame.hpp) | 图片及曝光时刻对齐姿态的输入契约；Frame 是 CapturedFrame 的兼容别名 |
| [detector.hpp](include/autoaim/vision/detector.hpp)、[detector_factory.cpp](src/vision/detector_factory.cpp) | 检测器统一接口、参数和按配置创建实现 |
| [detector_traditional.cpp](src/vision/detector_traditional.cpp) | 用亮度/颜色找灯条并配对；没有数字分类器，不能把每个候选当成正确目标 |
| [detector_openvino.cpp](src/vision/detector_openvino.cpp) | YOLOv5/YOLO11 输出解析、预处理、OpenVINO 推理及有界异步请求；两种模型共用此实现文件 |
| [detection.hpp](include/autoaim/vision/detection.hpp)、[detection_labels.cpp](src/vision/detection_labels.cpp) | 原始类别到统一类别/颜色/大小板的映射；模型原始整数不充当跨模型身份 |
| [corner_refine.cpp](src/vision/corner_refine.cpp) | 显式角点顺序映射和局部灯条精修；失败时保留原角点并降低质量 |
| [pnp.cpp](src/vision/pnp.cpp) | IPPE 多解、重投影误差、候选筛选和位姿不确定度；未知板尺寸不生成可靠三维结果 |
| [calibration.cpp](src/vision/calibration.cpp) | 加载/校验相机内参、畸变、相机到云台外参、图像尺寸及 ROI |
| [calibration_dataset.cpp](src/vision/calibration_dataset.cpp)、[calibration_solver.cpp](src/vision/calibration_solver.cpp) | 读取标定样本，求相机内参和 Park 手眼外参；接口在 calibration_solver.hpp |
| [calibration_report.cpp](src/vision/calibration_report.cpp) | 校验标定报告与实际参数、设备、验证残差是否匹配 |
| [annotation.cpp](src/vision/annotation.cpp)、[evaluation.cpp](src/vision/evaluation.cpp)、[sequence_evaluation.cpp](src/vision/sequence_evaluation.cpp) | 读取真值标注，计算逐帧匹配、类别/角点误差和连续漏检等序列指标 |

检测器选择由配置决定，**当前 armor.yaml 示例是传统检测**；实际 YOLO 路径有单独模板。
YOLOv5 适配旧 22 列角点格式，YOLO11 适配输入 `1×3×640×640`、输出 `1×50×8400` 的参考装甲模型（38 类、四角点），不是任意同名 YOLO 导出模型。未提供权重或加载失败会报错。

<a id="module-estimation"></a>

### M5. estimation：跟踪目标并估计运动

把多帧观测串成同一个目标。当前使用 EKF；恒角速度和有界角加速度是运动模型选择，不是另外两种滤波器。

| 文件 | 负责什么 |
| --- | --- |
| [observation.cpp](src/estimation/observation.cpp) | 把视觉结果和姿态变换成带来源、质量及不确定度的完整观测 |
| [geometry_model.cpp](src/estimation/geometry_model.cpp) | 统一几何 profile：板数、板偏移/法向、等高/成对分层/逐板高度及竖直或倾斜旋转轴 |
| [motion_model.cpp](src/estimation/motion_model.cpp) | 状态分量定义、运动外推和噪声传播；恒角速度/有界短时角加速度及模型选择 |
| [measurement_model.cpp](src/estimation/measurement_model.cpp) | 由状态预测观测，计算残差与线性化；不把这些逻辑塞进滤波器 |
| [ekf.cpp](src/estimation/ekf.cpp)、[health.cpp](src/estimation/health.cpp) | EKF/Joseph 更新；更新前 NIS（残差一致性）检查、数值和发散健康检查 |
| [association.cpp](src/estimation/association.cpp)、[armor_id.cpp](src/estimation/armor_id.cpp) | 判断观测属于谁，维护物理板身份假设；识别到“3号车”不等于确定其第几块板 |
| [geometry_selector.cpp](src/estimation/geometry_selector.cpp)、[state_machine.cpp](src/estimation/state_machine.cpp) | 几何假设选择，跟踪收敛、质量降级和丢失状态 |
| [tracker.cpp](src/estimation/tracker.cpp)、[target_snapshot.hpp](include/autoaim/estimation/target_snapshot.hpp) | 单目标 Tracker、有界多目标 TrackerSet；发布固定状态/模型/几何的不可变快照 |

<a id="module-decision"></a>

### M6. decision：预测未来并选择瞄准方案

输入目标快照，输出候选拦截方案和质量信息，不发串口命令，也不直接批准开火。

| 文件 | 负责什么 |
| --- | --- |
| [predictor.cpp](src/decision/predictor.cpp) | 在快照副本上预测未来板位置；有界迭代联合求解目标位置与飞行时间（solve_intercept 也在这里） |
| [ballistic.cpp](src/decision/ballistic.cpp) | 无空气阻力弹道；检查非法弹速、无解和飞行时间范围 |
| [armor_selector.cpp](src/decision/armor_selector.cpp) | 在一个目标的板之间选择，使用切换滞回避免来回跳板 |
| [hit_probability.cpp](src/decision/hit_probability.cpp) | 传播落点协方差，检查误差范围与板边距；名称沿用旧文件名，**不是经过校准的真实命中概率** |
| [aim_adequacy.cpp](src/decision/aim_adequacy.cpp) | 用实测反馈判断当前指向和稳定持续时间是否足够，不拿程序自己的指令当反馈 |

<a id="module-mission"></a>

### M7. mission：角色、操作方式和目标锁定

回答“步兵还是哨兵、辅助还是自动、当前锁谁”，将决策结果转为控制请求。

| 文件 | 负责什么 |
| --- | --- |
| [mission.hpp](include/autoaim/mission/mission.hpp)、[mission_factory.cpp](src/mission/mission_factory.cpp) | 任务公共输入/结果和角色、任务、权限组合校验 |
| [target_selector.cpp](src/mission/target_selector.cpp) | 多目标之间选谁：优先可靠且接近当前瞄准方向的目标，保持锁定，失效超时后重选 |
| [button_policy.cpp](src/mission/button_policy.cpp) | 独立按键的 toggle/hold 转换，启动按住、过期和重新按下资格 |
| [infantry_mission.cpp](src/mission/infantry/infantry_mission.cpp) | 步兵辅助/可选自动、人工接管、明确重新启用 |
| [sentry_mission.cpp](src/mission/sentry/sentry_mission.cpp) | 哨兵自动任务请求，最终 shoot 仍交控制层决定 |

不要混淆两个“选择器”：`mission/target_selector` 选**目标**，`decision/armor_selector` 选该目标的**装甲板**。

打符三个预留文件：[autoaim_rune.cpp](apps/autoaim_rune.cpp)、[rune_mission.hpp](include/autoaim/mission/rune/rune_mission.hpp)、[rune_mission.cpp](src/mission/rune/rune_mission.cpp)。它们目前为空、不构建，继续保留。

<a id="module-control"></a>

### M8. control：最终许可与发布

输入请求和证据，形成最终 Command，并统一处理过期、故障与发送。

| 文件 | 负责什么 |
| --- | --- |
| [control_intent.hpp](include/autoaim/control/control_intent.hpp) | 上游意图：来源、目标角度、请求及证据；请求不等于最终 shoot |
| [command.hpp](include/autoaim/control/command.hpp)、[command.cpp](src/control/command.cpp)、[checked_command.hpp](include/autoaim/control/checked_command.hpp) | 五字段 Command、来源元数据和已检查不可变命令 |
| [watchdog.cpp](src/control/watchdog.cpp)、[smoother.cpp](src/control/smoother.cpp) | 主机数据时效检查、正常指令修正平滑；不是下位机固件看门狗 |
| [command_guard.cpp](src/control/command_guard.cpp) | 唯一形成首次控制/开火许可的位置；发送前复检只能保留或撤销许可 |
| [publisher.cpp](src/control/publisher.cpp) | 唯一发布线程、重发/停止、写失败锁存和退出；调用适配的写出回调 |
| [crc.cpp](src/control/crc.cpp)、[protocol.cpp](src/control/protocol.cpp) | CRC、UART/CAN 兼容编码、AB43 反馈解码与半包/粘包解析；不直接操作串口 |

<a id="module-pipeline"></a>

### M9. pipeline：组装、时序和生命周期

这里是整条链的“接线处”，不是再写一份检测/跟踪算法的地方。

| 文件 | 负责什么 |
| --- | --- |
| [bootstrap.cpp](src/pipeline/bootstrap.cpp) | 读取业务配置、解析路径/参数、校验模块参数，确定板型尺寸；只校验队列预算，不临时分配图像池 |
| [pipeline.cpp](src/pipeline/pipeline.cpp) | 持有模块实例、连接图片到命令、处理模式世代、协调启动/故障/关闭，以及同链文件回放 |
| [frame_sync.cpp](src/pipeline/frame_sync.cpp) | 保存反馈历史并查询曝光时刻姿态；不在历史覆盖范围外外推 |
| [queue.cpp](src/pipeline/queue.cpp) | 有界最新帧队列、缓冲租约、完成结果接纳和丢帧原因；结果接纳类也在同名头里 |
| [command_slot.cpp](src/pipeline/command_slot.cpp) | 暂存待提交的最新指令，避免旧世代或过期指令继续进入发布链 |
| [scheduler.cpp](src/pipeline/scheduler.cpp) | 工作线程启停、停止请求和线程边界异常收集；不决定业务许可 |
| [uart_adapter.cpp](src/pipeline/uart_adapter.cpp)、[uart_writer.cpp](src/pipeline/uart_writer.cpp) | 组合 HAL 与协议：原始 UART→明确轴向/符号的反馈；最终 Command→UART14 字节写出 |
| [entry.cpp](src/pipeline/entry.cpp) | 公共 CLI 参数、角色检查、TSV/会话/UART 记录输出；apps 只是它的薄入口 |
| [offline_tools.cpp](src/pipeline/offline_tools.cpp)、[batch_benchmark.cpp](src/pipeline/batch_benchmark.cpp) | 单图检测计时与多配置整链评测；批量预读事实在一次调用内共享，不缓存像素 |
| [calibration_tools.cpp](src/pipeline/calibration_tools.cpp)、[run_metadata.cpp](src/pipeline/run_metadata.cpp) | 标定工具 CLI 与运行配置/模型/标定内容标识；标定求解算法仍在 vision |
| [yaml_output.hpp](src/pipeline/yaml_output.hpp) | 私有内联文本 YAML 写出，只供标定与批量评测共享；不承担覆盖策略或标注发布 |
| [annotate_session.cpp](src/pipeline/annotate_session.cpp) | 离线标注导出、审核回看、整体替换应用与自检；只创建源会话外的新目录，不调用模型 |
| [session_annotation.cpp](src/pipeline/session_annotation.cpp)、[私有头](src/pipeline/session_annotation.hpp) | 严格 v2 单行 flow 读取、数据指纹、审核绑定检查、独立 PNG 复制及 Linux 无覆盖原子发布；不成为公共 HAL 或运行时接口 |

标定与评测的文本写出使用 DoublePrecision 17，保留默认 Float 精度、文本流、
末尾换行与 flush，之后检查 emitter 和流。标定在调用前拒绝已存在文件，
仍抛 `invalid_argument("Refusing to overwrite calibration output")`；写出失败分别为
`Cannot write calibration output`、`Cannot write evaluation report`。
CLI 对已存在输出目录的拒绝是另一层检查，不能与上述文件检查混同。
标注独立保留 Double17/Float9、序列化前置检查与二进制写出；HAL 会话写出也不参与此共享。

并发规则集中理解即可，不必在每个模块各实现一遍：

- 池满时先清失效等待帧，再淘汰最旧可回收等待帧；全池都在用则丢新帧，不覆盖推理中的图片。
- 工作线程领取最新等待帧；乱序完成相对“最后已接纳帧”判断，不因更晚帧仅在途就拒绝当前结果。
- OpenVINO 请求槽有上限，原图和预处理输入保留到实际推理结束。
- 数据处理接口由单一处理线程串行调用，估计状态单写者；推理后端和发布线程可以异步工作。
- 回放使用逻辑时钟和完成屏障保证重复结果一致，因此回放逻辑延迟不代表真实计算耗时。

构建验证辅助：[verification_manifest.py](tests/verification_manifest.py) 不属于运行模块；
根 CMake 的 `verification_manifest` 目标调用它保存测试及夹具内容标识、CTest 实际
注册项、CMake 缓存与生产源码/构建标识。它只记录材料，不执行测试或授予设备能力。
源码与生成的生产清单不符时拒绝生成，提示先重建；测试/夹具不进入运行报告生产指纹。

---

<a id="engineering-rules"></a>

## 协作与工程约定

## A1. 开工前先确认工程状态

先检查当前文件、工作区变更和用户授权范围，保留已有修改。报告时区分「已实现」「已验证」与「设计如此」；
历史测试结果只说明对应日期和环境，不是当前任务的新验证结果。

- 模块与文件现状见 M1—M9；测试用法与待完成项见 [README §8](README.md#8-测试已验证范围与待完成项)，
  阶段构建与验收记录见 [build_history.md](build_history.md)。
- 初始空工程基线及已完成的实施顺序收在 [§11](#implementation-history)，不作为当前待办重复执行。
- 硬件仍未完成验收，见 [§9](#unverified)；软件回放通过不能解除设备入口限制。
- 2026-09-30 的文档重组阶段仅改文档；后续代码工作按另行授权记录于 §11.5、§11.6。

---

## A2. 硬约束

以下每条都是前面评审的结论，违反即返工。右列是**如何机械检查**——做不到机械检查的约定一定会退化。

| # | 约束 | 检查方式 |
| --- | --- | --- |
| 1 | 依赖只能向下：`apps → pipeline → {mission → decision → estimation → vision → math → core, control → hal → core, hal}` | 按九个模块声明 target 与可见接口，结合依赖白名单和包含检查；不能承诺越层必然链接失败 |
| 2 | 跨模块溯源字段不可缺省；新鲜度/到期在消费点现算，采集与解算时的事实/质量可以保存 | 契约类型无默认构造；`Stamp` 必填 |
| 3 | 最终开火使能只有一个来源：`control/command_guard`。上游提供请求与证据，不设置最终使能；发送复检只能保留或撤销同一条已接纳指令的许可 | `ControlIntent` 不携带上游可设置的最终使能；`publisher` 调用 `command_guard` 检查，`command_guard` 不反向调用 `publisher` |
| 4 | 生命周期四个负责人各管一段：`bootstrap` 配置与构建、`pipeline` 持有实例并协调启停、`scheduler` 执行线程启停、`publisher` 管理发送线程与故障状态 | 同一职责出现第二个负责人即为返工 |
| 5 | 角色/任务/权限的允许组合只在 `mission_factory` 判定一处实现 | 其他组件调用它，不得复制判定 |
| 6 | 并发：计算可并行，估计状态只能由单一线程更新；接收帧序**单调前进且允许跳帧**，不为等待更早的慢帧而阻塞；接纳点唯一，但**发送端仍会再次检查过期** | `tests/contract/test_out_of_order_completion.cpp` 等契约测试 |
| 7 | `Tracker` 把状态预测到**当前观测时刻**后做关联与更新；向**未来时刻**的外推归 `decision/predictor` | `predictor` 不得写回跟踪状态 |
| 8 | 测量模型负责**由状态预测观测、定义残差与所需线性化**，并从观测提取量测与噪声 | 实现 EKF 时不得把这些塞回 `ekf.cpp` |
| 9 | `TargetSnapshot` 必须同时固定该快照对应的**运动模型、几何参数与状态布局** | 防止"旧状态配新模型"；不可变快照 + 模型引用 |
| 10 | 能力证据分级：配置声明最高只能到「仅声明」，**安全判据只接受「已实测」** | 类型上没有从配置直达 `Measured` 的路径 |
| 11 | 控制权限由角色、任务与当前控制模式共同确定：步兵默认辅助且不自动开火，可明确启用自动控制；人工介入即退回辅助，必须明确重新启用；哨兵开火依赖程序最终 `shoot` | 按 §6 的模式矩阵与接管规则检查，不凭角色/任务名自动授予权限；指向判据使用实测反馈 |
| 12 | 状态按**分量名**访问，不按下标 | 否则替换运动模型后下游会静默取错分量 |

### 范围边界（很重要）

项目功能范围以本文件 §1.3 为准。代理只实施用户已授权的条目；发送阶段联锁只能保留或撤销已有许可。算法优化仍须满足时效、模式、故障处理与设备约束。

开火请求、首次许可与发送复检的完整契约见 §4.3、§5.9：请求不等于许可，发送复检不能恢复已撤销的许可。

---

## A3. 归档中真实出现过的故障

这些不是假想风险，是旧项目实际发生过的问题。每条都对应本框架的一个约束——**写代码时必须保证对应防线存在**。

| 归档故障 | 本框架的防线 |
| --- | --- |
| 两块装甲板时固定访问三个候选，越界 | 契约按实际板数；`geometry_model` 校验向量数等于该兵种板数 |
| 用错误的半径状态索引判断旋转速度 | 约束 12：状态按分量名访问 |
| 弹道不可解、弹速非法或 NaN/Inf 仍被继续使用 | `ballistic` 返回可解性；`aim_adequacy` 只出证据；`command_guard` 唯一使能 |
| NIS 在状态更新**之后**才计算，阈值 `0.711` 与注释「自由度 4，95%」不符（4 维卡方 95% 上侧约为 `9.49`） | 测量模型拥有残差与线性化；`health` 做**更新前**一致性检验，阈值按**实际观测维度**取 |
| 把「后验减先验」当成 NEES | NEES 需要对齐的独立真值，可用于合成或有真值的实测/回放；无真值在线运行只做 NIS 等检查 |
| 模式切换后旧指令继续存在 | 世代号进契约；`command_guard` 检查 `previous_mode`；接纳点按世代丢弃 |
| 异步线程异常直接 `std::terminate` | 工作线程边界捕获异常并置故障锁存；另外注意 `noexcept` 函数抛异常是第二条 terminate 路径 |
| 通信写入失败只打印日志，发送线程不知情 | `publisher` 的写结果区分 `complete`/`failed` 并锁存首次故障 |
| FIFO 队列积压旧图像，增加曝光到发送的延迟 | `queue` 对待处理图像最新帧优先；完成结果进入估计器前统一接纳，不等待更早的慢帧 |
| 异步推理结果没有统一的源帧号、源时间和质量标记 | 契约溯源约定：帧号、世代、源时间、曝光时间、质量随结果携带 |
| 图像缓冲区、异步请求与推理结果的生命周期未绑定；**仅保存原图不足以保证推理输入的生命周期** | `detector` 持有预处理输入缓冲区直到推理结束 |
| 打符预测在同一个已被修改的目标上重复调用 | 快照不可变；`predictor` 只读快照，不改跟踪状态 |
| 角点上下关系只按图像纵坐标判断 | 四角顺序来自模型标签约定；未证明语义顺序的路径只允许保守跟踪，**不授权开火** |
| 局部角点精修失败被包装成"更精确的角点" | `corner_refine` 失败时保留原角点并降级质量；`pose_valid` 与 `pose_reliable` 分开，不合并成一个"检测成功" |
| 目标角误差与规划输出误差分处判断，且容差由生产者自己给出，组合边界接近两者之和 | 开火使能唯一来源；容差来自安全配置，**不由产出轨迹的组件提供** |
| Tracker 未证明是上一帧同一块物理装甲板，却用上一帧 PnP 当先验 | 先验必须带同板证据；`association` 与 `armor_id` 明确身份，身份不确定时退回可见板跟随 |
| 各入口使用不同的固定提前量 | 取消固定提前量，由曝光时间戳计算 |
| 一次故障处理需要改 **14 个运行入口** | 入口只选配置；公共启动流程在 `bootstrap` |
| 主机进程冻结或断电时无法保证继续发送停止指令 | 下位机独立通信超时清零须核实；主机内部序号不等于线协议已有序号，协议扩展须另行确认（**未验证**）；主机只能记录"未确认"，不得宣称"设备已停止" |
| 相机曝光时间靠接收时刻回推，`camera_timing_calibrated` 只是配置声明 | 约束 10：声明不等于实测 |

零容忍验收项统一见 §8.1。

---

## A4. 未验证假设（不得当成事实）

完整清单见 §9；硬件前提另见 §6.2。只能按未验证前提设计，不能据此把能力标记为已实测或授予许可。遇到证据缺口时留 TODO 并报告，不猜默认值。

---

## A5. 工作方式

Docker 环境打包仅增加构建/安装/离线使用入口；默认不包含海康 SDK、设备映射或
模型数据。SDK 只通过独立只读构建输入做编译检查，不因此开放硬件入口。

容器不改变九模块职责、配置解释或算法候选启用规则。根 CMake 的 `Runtime` 组件只安装
11 个既有入口、显式列出的示例配置和生产构建标识，不导出静态库、头文件、测试或打符目标。
示例配置包含公共的 `config/fast_choose.yaml`，保持离线配置的 `../fast_choose.yaml` 引用。
`tests/test_install_layout.py` 验证安装、搬移目录后启动与回放；不需要 Docker 或 root。
`tools/container/runtime-packages.sh` 固定开发镜像实际安装的运行包版本，显式保留
OpenVINO CPU 插件和 IR 前端；不依据静态链接清单删除动态加载组件。
`tools/container/verify.sh` 只在明确调用时执行容器矩阵；`runtime-smoke.sh` 为外部挂载的
运行镜像检查，不安装进运行镜像。开发容器 `/tmp` 允许执行安装搬移测试及安装器替身，
保留 nosuid/nodev；运行容器不授予该临时目录执行权限。该区别不改变算法或硬件权限。
基础镜像摘要、包清单与程序构建标识分开保存，
同镜像同路径重复性不等于跨编译器/依赖版本的全文报告一致。
Windows 绑定目录可能缺少标注工具的原子无覆盖重命名能力；工具保持失败语义，
容器使用 Linux 卷完成发布后再显式导出产物，不将普通复制冒充原子发布。
操作命令见 [README §2.4](README.md#24-docker-环境)，实际结果见
[Docker 构建历史](build_history.md#docker-20260930)。

- **先契约、再实现。** 每个模块先把接口与数据结构定下来，能被别的模块编译引用，再写实现。
- **代理不自行提交。** 改动交用户审查，未经明确授权不执行 Git 提交。
- **禁止代理运行连硬件程序。** 只允许无硬件构建、测试与回放；实机验收由人执行。
- **一次只动一个关注点。** 不要把"顺手重构"混进功能改动。
- **不写投机抽象。** 不要加"以后可能有用"的接口、插件系统、配置项。CKF 是离线对照，不需要滤波器插件框架。
- **候选默认由 CMake 关闭。** 后续新增算法候选须保持替换对象的接口与数据语义，将完整实现及必要依赖放在对应编译期分支，并在根 CMake 登记默认 `OFF` 的唯一选择开关；不再通过手动修改 cpp/hpp 注释或别名切换。启用前在独立构建目录运行候选专项与回归；默认构建不替代候选验证。同步快速替换表、条件测试注册和构建报告开关值；缺接口、数据或硬件的方案只记录缺口，不生成空壳。
- **不做大重写。** 旧项目里已有成熟实现的安全组件（如停在传输边界的许可检查、异步检测的单槽覆盖与世代校验）应当被**移植**，不是重新发明。
- **每次修改同步文档。** README 与本手册各自的维护范围及状态表述统一见 §10.3，不借文档改动扩展任务范围。
- **按当次授权推进。** 已授权任务内按接口、实现、测试、验证逐项完成，不逐文件重复确认；遇到关键矛盾、资料缺失或验证失败时停在当前项。§11 只记录过去阶段，不是自动执行指令。

### 一个模块算"定义完成"的标准

1. 接口在 `include/` 中可被上层包含，且不引入任何越层依赖；
2. 数据结构中所有跨模块字段都满足约束 2 与 9；
3. 至少有一条契约测试或断言覆盖它的关键不变量；
4. 本手册 M1—M9 的文件职责与实际接口一致，README 的模块速查及使用入口同步；不能只改代码而保留失效说明。

---

## A6. 验证与验收

正确性在 WSL 验证，性能与命中结论只来自目标 i5-12450H NUC。只报告实际环境、配置、素材与测得结果；完整测试分层、统计口径、零容忍项和禁止结论见 §8。

契约测试优先覆盖 A3 的已知故障；设计完成、代码存在、编译通过、回放通过与实机通过必须分别记录，不得互相代替。

---

## A7. 参考（只读，不要照搬结构）

| 资料 | 用途 |
| --- | --- |
| `E:\大学\巡天御风\视觉组培训\RM2026-AutoAim` | 旧实现：可查具体算法与安全组件的既有做法 |
| [旧项目修复记录](<E:/大学/巡天御风/视觉组培训/RM2026-AutoAim/docs/auto_aim_upgrade.md>) | 修复记录：故障现象、已验证/未验证边界、WSL 验证命令 |

参考它们的**行为与教训**，不要照搬目录结构——本仓库的分层是重画过的，旧结构是按任务切的纵向副本。

---

## A8. 工程约定与待补

依据仓库已有问答记录，以下选择已作答，不再重复列为未知：

| 细节块 | 当前约定 |
| --- | --- |
| 工具链与依赖 | 默认 C++17，允许 C++20，可按实际需要使用 C++20 特性，不为升级版本改写已有实现；通过 CMAKE_CXX_STANDARD 选择。OpenCV、Eigen、yaml-cpp、可选 OpenVINO；不引入 fmt/spdlog/nlohmann_json。版本与实际链接已在 WSL 核实，见 README |
| 测试框架 | CTest + 轻量 C++ 检查工具，不新增第三方框架；Release 下检查仍生效 |
| 代码风格 | 两空格缩进、类型 UpperCamelCase、函数和变量 snake_case；C++ 使用根 .clang-format 的 100 字符行宽，return 独占一行，逻辑段之间留空行；hpp 注释契约/单位/所有权/线程要求，cpp 注释依据与边界，不复制旧格式配置；纯排版不得改变算法、接口、字面量值或执行顺序 |
| 变更与硬件边界 | 统一遵守 A5 的小步修改、提交限制和禁止代理连接硬件要求；文档分工见 §10.3 |
| 构建命令 | 九模块库、离线薄入口、工具与 CTest 已连接；构建辅助函数、依赖检查器源码和测试注册统一维护在根 CMakeLists.txt，不保留源码树的 cmake/ 目录；命令见 README，最近完整回归记录见 §11.6 |

### 构建与依赖说明

构建命令和选项统一维护在 [README §2](README.md#2-编译并跑通第一个示例)，
2026-09-30 续修验收环境与矩阵见 [构建与验证记录](build_history.md#分批修复进度2026-09-30)。
根 CMake 将每个模块的完整显式源文件清单集中在对应 `autoaim_module(...)` 中，
保留源码顺序，不用 GLOB 决定编译目标。`autoaim_test_target(...)` 只创建测试程序、
设置测试包含目录并链接显式参数；`autoaim_test(...)` 额外显式链接 `autoaim_options`、
注册同名测试并设置 30 秒超时。模型测试单独按已有条件注册，保留 60/120 秒超时。
四个工具的字面链接声明和依赖白名单供内嵌边界检查器核对，不能只为简写而隐藏进变量循环。

2026-09-29 记录的 OpenVINO_DIR 为 `/usr/lib/cmake/openvino2026.3.1`，它是本机路径，换机必须核对；
关闭 OpenVINO 只用于明确选择传统检测的构建，不代表 YOLO 可用。

旧项目的 ROS2 / `sp_msgs` 命令只适用于其旧适配，本项目不因复用命令而新增 ROS2 依赖。
离线输入是本地图片和 YAML 事件，输出为 Command TSV 等记录；不提供代理连接硬件的运行步骤。

依赖安装脚本为 `install_dependence.sh`（文件名按用户要求统一），由人显式运行；支持预览、软件依赖、OpenVINO、Hikrobot MVS 本地 `.deb`/ZIP 或显式 HTTPS 直链，以及 `--check-camera-only` 无安装检查。相机包在系统安装前校验，ZIP 按 DEB 元数据选择唯一 amd64 包；不硬编码旧 SDK 链接、不绕过官网验证、不执行 ZIP 中的 setup.sh。只覆盖当前项目依赖，不照搬旧项目的 ROS2/额外库或删除服务步骤。脚本测试不执行系统安装，不以驱动文件存在替代相机、固件或设备能力证据。具体选项和官方源见 README。

---

<a id="technical-specification"></a>

## 技术规格与验收要求

以下 §1—§10 保留已确认的目标、数据契约、算法和验收要求，不是全部功能的实现声明。
文件职责见 M1—M9，历史完成记录见 §11。原规格中尚未与实现完全对齐的状态表述，
在 [§10.4](#spec-status-notes) 单独提示，不借整理文档更改算法要求。

---

## 1. 要达成什么

### 1.1 目标 **[定]**

在 Intel i5-12450H NUC 上，让自动瞄准系统具备：

1. 更稳定的装甲板识别与 PnP 位姿解算；
2. 对非匀速、突变与不规则运动的**短时**预测；
3. 缩短相机曝光到发送云台指令的端到端延迟；
4. 控制旧帧、过期结果与异常状态，避免错误指令继续下发；
5. 仅在条件满足时允许自动开火，失效时立即撤销；
6. 保留 C++、OpenVINO、串口/CAN 协议与主要工程结构。

### 1.2 非目标 **[定]**

首个装甲板闭环可交付版本的必备能力不包含：完整 ROS2 迁移、大型端到端网络、CKF、因子图、短滑窗联合重投影优化、模型压缩/INT8/动态 ROI。它们只在有实测证据显示收益时才进入下一轮。“首个可交付版本”不等于 §11 的第一阶段；第一阶段只完成工程骨架、契约与运行流程，不代表已经达到上线条件。

### 1.3 项目范围与联锁职责 **[定]**

本项目面向 RoboMaster 工程游戏。选靶、弹道预测、命中评估与效果优化、开火时机策略均属于项目功能范围，按阶段实施，并通过回放和实机数据验收。首版任务范围见 §1.5，工程推进顺序见 §11，二者不混用。

“许可只做减法”限定于发送阶段的联锁：它只能保留或撤销本次请求已经取得的许可，不得自行恢复已撤销的许可。请求来源、首次许可形成与最终输出的关系，在控制契约中另行规定。

算法改进应满足正确性、实时性和设备约束，不以是否增加命中率或射击机会判断其是否超出项目范围。

### 1.4 一个必须承认的边界 **[定]**

有界随机转速、或无法辨识的隐藏转速规则，**不会因为换滤波器而变得可准确长时预测**。因此采用短时外推 + 不确定度约束：相位不确定度过大时允许继续跟踪，但不得继续开火。

### 1.5 首版交付范围 **[定]**

首版先完成装甲板闭环，覆盖**步兵辅助瞄准、步兵可选自动控制、哨兵自瞄**。打符保留为项目后续任务，不要求与首版装甲板闭环同时交付，也不因延后而删除对应架构职责。

三种装甲板高度布局与竖直/倾斜转轴、大角度 PnP 的质量判别、EKF 状态估计与运动模型外推仍按 §5 实施和验收。首版不加入 MPC。默认初次选择最接近当前瞄准方向的可靠敌方目标，保持有效锁定，失效超时后重新选择，不附加兵种优先级。

### 1.6 离线优化执行包 v1 补充契约 **[定]**

2026-09-29 的离线优化执行包 v1 约定：缺少真实素材不阻断独立软件开发，但相应实测验收必须保持未完成；不以合成测试
替代设备证据。仍保留九模块、EKF、不可变快照、有界队列、唯一发布线程与 UART14。

| 范围 | 必须遵守的实现规则 |
| --- | --- |
| 数据 | PNG＋YAML v2，会话来源与派生执行身份分开；保留源帧/曝光/接收/SDK 诊断、原始 UART 与独立按键；缺字段为未知。新会话目录不可覆盖，中断未 finish 不可回放，兼容旧清单 |
| 标定 | 显式棋盘内角点/对称圆点阵尺寸及实测间距；针孔五参数内参、OpenCV Park 相机到云台手眼。位姿必须完整含平移且同步，拒绝重复/单轴退化，拟合与独立验证分开 |
| 报告 | 绑定设备/配置、实际参数、分辨率/ROI、方法、样本与残差；导入复核门限而非 passed 布尔值。至少三份拟合/两份独立验证才形成通过证据，内参不能授予外参/时间/控制能力，合成不能升为实测 |
| 语义 | 模型原始类别、统一目标类别、颜色、板型分别表达；物理板身份由估计维护。PnP 只按已知板型取尺寸，未知保留二维；旧 class_id 配置兼容但不能混用新尺寸表 |
| 模型 | YOLOv5 保留基线，参考 YOLO11 固定 640 输入/50×8400 输出/38 类四角，独立标签字典，共享请求槽/输入寿命。YOLO11 原角点顺序经显式映射处理，图像排序不升级可靠性；初始化失败不回退 |
| 相机 | MV-CS016-10UC 显式 ROI/像素格式/白平衡，请求与回读分开，内部 BGR8，已知几何不匹配即拒绝；tick 未映射只作诊断，150fps 不预先标达成，该执行包不改缓冲所有权或去复制 |
| 按键 | AB43 CRC 接收/14 字节发送及五字段 Command 不变；坐标/符号必须显式映射。独立按键默认 toggle、备选 hold 都编译，启动按住/接管/故障/失效后先释放再按下；旧 enable_event 仍为显式启用，切换先撤销再更新世代 |
| 评测 | 同一 Pipeline、多配置、显式 IoU、确定性一对一匹配；未知标注不是负样本，几何/语义/角点/有条件位姿指标分列，按人工干预分层，保留无目标/丢帧/过期样本。逻辑结果与主机墙钟耗时分开 |

不新增打符、MPC、模型训练、INT8、动态 ROI 或滤波器体系。新增依赖才修改安装脚本；
不安装/下载、不接硬件、不修改参考仓库、不提交 Git。验证矩阵与实施记录见 §11.2。

---

## 2. 运行环境

| 项 | 内容 |
| --- | --- |
| 目标设备 | Intel Core i5-12450H NUC **[定]** |
| 相机型号 | 海康 MV-CS016-10UC，彩色款，用户已确认；不是 MV-CS016-10UM 黑白款 **[定]** |
| 目标采集帧率 | 150fps **[定]**；实际采集帧率和整链路处理能力仍 **[未验]** |
| 开发/编辑 | Windows 工作站 **[定]** |
| 正确性验证 | WSL（Ubuntu 22.04 / GCC 11.4）**[定]** |
| 性能与命中结论 | **只能**来自目标 NUC **[定]** |
| 推理 | OpenVINO，模型文件与 CPU/GPU 选择来自配置 **[定]** |

相机技术参数截图标注 USB 3.0、全局快门、1440×1080，以及该分辨率下 Bayer RG 8 的最高 249.1fps。这些是用户提供的产品资料，不是设备实测，也不取代项目 150fps 的目标。序列号、镜头/焦距、实际分辨率/ROI、输出像素格式、曝光/增益、MVS SDK 版本及内外参和时间标定仍待补充；不能据此替换合成配置或解除硬件入口限制。

**WSL 的耗时不能代替 NUC 的实时性能或命中效果。** **[定]**

---

## 3. 端到端数据流与时间轴

### 3.1 时间轴定义 **[定]**

| 时间 | 定义 |
| --- | --- |
| `t_img` | 图像曝光中点；随观测传播，不得被处理、预测或重发时刻覆盖 |
| `t_state` | 状态估计所对应的时刻；目标外推从此时刻起算 |
| `t_create` | 控制意图生成时刻 |
| `t_send` | 本次主机完整写入成功的时刻，不代表设备已经执行 |
| `t_send_est` | 生成意图时使用的预计发送时刻，不冒充事后实测的 `t_send` |

预测时间轴：

```text
t_fire = t_send_est + d_after_send
t_hit  = t_fire + 子弹飞行时间
```

这里的 `t_fire`、`t_hit` 是预计时刻。`d_after_send` 的标定起点必须与主机完整写入的定义一致，覆盖此后剩余的通信、执行与发射延迟；不得漏算或重复计入排队时间。

曝光时间优先使用语义明确、已映射到主机单调时钟的硬件时间戳，并按其曝光起点/终点语义换算到曝光中点。不可用时，才使用经标定的接收时间回推：

```text
t_img = t_receive - d_transport - T_exposure / 2
```

该式要求 `d_transport` 表示从曝光结束到所记录 SDK 接收事件的延迟，必须核对具体 SDK 的时间语义并实测。随帧保留时间来源、标定证据和不确定度；不能仅靠配置值宣称完成标定。

参与比较或相减的时间必须处于同一时钟域。设备时钟 tick 未映射到主机单调时钟时，不能与主机姿态时间直接相减；回放时通过可替换时钟保持一致的逻辑时间语义。

端到端延迟取每条指令“首次完整写入成功时间 − 源曝光时间”；重发不刷新首次记录，未成功写出的指令另计失败或过期，不混入成功延迟样本。曝光时间来自估计时，报告应注明该口径和时间不确定度。

### 3.2 数据流 **[定]**

```text
相机图像（拥有内存）+ 曝光时刻与来源 + frame_id + generation
  → FramePacket（含同一时刻姿态）
  → 最新帧检测 / 角点与 PnP 质量
  → TargetSnapshot（源时刻、状态与协方差、身份、质量）
  → 预测目标到达时刻 / 瞄准解算
  → ControlIntent（源时刻、有效截止、控制/开火请求与证据）
  → publisher 调用 command_guard 首次形成许可
  → 唯一发送线程调用 command_guard 复检，保留或撤销许可
  → 兼容原协议的数据包
```

`frame_id` 标识**采集顺序**；源时间标识这份结果**实际观测了哪个时刻**。**不得**在发送、重发或预测时把旧目标的源时间刷新成现在。 **[定]**

### 3.3 每个环节的权威 **[推]**

| 环节 | 谁有权决定 |
| --- | --- |
| 打哪个敌人 | mission |
| 打哪块板、时机策略与可解性证据 | decision；不设置最终使能 |
| 是否请求开火 | mission 根据当前授权模式、任务策略与操作输入形成请求；步兵辅助模式不请求程序开火 |
| 是否真的允许开火 | control 的许可检查（唯一来源） |
| 云台最终指向 | 哨兵自瞄或步兵自动模式由程序控制；步兵辅助模式以人工为主并提供修正；步兵人工接管撤销自动控制权 |

---

## 4. 数据契约

通用规则 **[定]**：

- 契约类型**无默认构造**，溯源字段必须显式提供；
- 新鲜度、是否到期及当前模式是否匹配，在**消费点**按当前时间与世代重新计算，不以缓存的 `valid` 布尔量代替；
- 采集/解算当时的事实和质量可以保存为布尔量或枚举，如 `sensor_ok`、`pose_valid`、`pose_reliable`，但必须与时间戳和来源绑定；这些事实不代表消费时仍然新鲜或拥有开火许可；
- 坐标系与单位进类型，防止量纲与参考系混用。

### 4.1 FramePacket **[定]**

| 字段 | 说明 |
| --- | --- |
| 图像所有权 | 缓冲租约；异步推理在途时不得回收 |
| `frame_id` | 采集顺序，单调 |
| `generation` | 在输入帧进入处理链时绑定当前模式世代，异步任务、观测、快照和意图全程原样携带，不得在完成时补填新世代 |
| 接收时间 | SDK 语义 |
| 曝光中点时间与来源 | 采用 §3.1 的硬件时间戳优先、已标定回推兜底策略 |
| 曝光时间不确定度 | 随帧保留的质量描述 |
| 是否完成时间标定 | 证据等级，非布尔声明 |
| 对应姿态 | 自带采样时刻、插值来源与质量事实；历史同步和当前反馈新鲜度分别检查 |
| 姿态质量事实 | 来自反馈链路及同步结果；不替代消费时的新鲜度检查 |

### 4.2 TargetSnapshot **[定]**

| 字段 | 说明 |
| --- | --- |
| 目标状态 | 按分量名访问，不按下标 |
| 状态时刻 `t_state` | 与不可刷新的源观测时刻分开保存；外推不得改变源观测年龄 |
| 协方差 | 必须可传播到落点 |
| **装甲板身份** | 当前认为观测到的是哪块板，含不确定度 |
| **世代** | 属于哪个模式世代 |
| 源帧号 / 源时间 | 不可被刷新 |
| 姿态质量事实 | 保留来源与采样时刻 |
| 跟踪质量 / 开火候选证据 | 分列；不直接表示当前可用性或最终开火使能 |
| **模型解释** | 该快照对应的运动模型、几何参数与状态布局一并固定 |

最后一条的理由 **[推]**：几何支持在线慢修正、模型支持在线选择，而决策必须从**不可变**快照外推。若快照只有状态与协方差，预测时读取已被更新的全局几何参数，就会出现"旧状态配新模型"。采用小型参数副本或不可变模型引用即可，不需要版本管理系统。

### 4.3 ControlIntent **[定]**

| 字段 | 说明 |
| --- | --- |
| 是否控制云台 | |
| 开火请求 `fire_requested` | 由 mission 形成，不等于最终开火许可 |
| 当前控制模式 | 辅助 / 自动；结合角色、任务、权限和当前世代校验，不由单帧算法自行授予自动控制权 |
| 开火权威 | 自主 / 受监督 |
| **指令空间** | 绝对指向 / 相对修正 |
| **世代** | 模式切换的失效边界 |
| yaw / pitch 或修正量 | 取决于指令空间 |
| 速度与加速度前馈 | |
| 源帧号 / 源时间 | 全程保留，不以新意图生成时间替代 |
| 意图生成时刻 `t_create` | 与源时间、预计发送时间分开 |
| 有效截止时间 | 消费和发送时重新检查 |
| 瞄准与姿态证据 | 包含事实、来源、采样时刻与质量，不设置最终使能 |

许可检查结果与输入契约分开：由 `command_guard` 输出 `fire_allowed` 与停火原因诊断位，并保留源帧、源时间与世代供诊断。详见 §5.9。

### 4.4 最终输出 Command **[定]**

用户指定 `E:\大学\巡天御风\自瞄构建计划\sp_vision_25\io\command.hpp` 为输出字段参考（只读）。本项目 `autoaim::control::Command` 固定按顺序包含 `bool control`、`bool shoot`、`double yaw`、`double pitch`、`double horizon_distance = 0`，与参考字段名、类型、顺序及距离默认值一致。保留本项目命名空间，不添加对旧项目路径的构建依赖。内部契约继续使用强单位，只有兼容输出边界转换为 double：角度 rad、距离 m；已给出的源意图水平距离不得在转换时丢失。

`Command` 是五字段值对象，不包含溯源或前馈，也不是可以直接写入设备的内存包。`CheckedCommand` 保存同次复检产生的不可变 Command 和独立 `CommandMetadata`；元数据保留源帧/曝光时间/世代、指令空间和既有 AB 协议前馈。唯一发布回调接收 `(const Command&, const CommandMetadata&)`，引用仅在回调期间有效。`control/shoot` 仍只能由 guard 形成许可，复检只保留或撤销；停止命令清零五字段并在旁路保留已有来源。既有线协议格式、相对通道未验证限制及主机时效规则不变，TSV 可单独记录命令及诊断元数据。

---

## 5. 算法规格

### 5.1 检测与角点 **[定]**

当前传统检测由灯条最小外接矩形的长轴中心线端点构造四角，不是灯条外侧边缘。
YOLOv5 解析器先执行 `[0,3,2,1]` 兼容置换，YOLO11 解析器保留原始关键点顺序，
随后两者分别应用 `corners.indices` 配置映射。排列合法、凸性或绕向检查通过，
均不等于物理端点语义已核验。训练标签所指的物理端点尚需资料核验；
不能由输出张量或低 PnP 重投影误差推导真实尺寸。配置尺寸须与所用端点定义一致，
目前不添加猜测的外缘补偿。矩形同比缩放尺寸和距离仍可得到相同投影；合成测试
仅验证坐标/尺度契约，不证明实机尺寸或绝对物理板身份。

模型对接须区分类别字典、模型元数据和训练标注定义：类别字典规定输出索引的解释，
当前 YOLO11 字典的 24、28 分别为紫色大基地、紫色小基地，与已核对的历史参考 IR 标签声明一致；
该对照不代表任意 YOLO11 或拟部署版本均已核验。模型元数据提供导出文件携带的标签与预处理声明，
不能代替原始训练管线；关键点物理位置、顺序及板型尺寸所量端点仍需训练标注定义和对应样例。

当前推理采用左上对齐、零填充，缩放宽高取整后执行 resize，坐标解析使用名义缩放比例。
历史参考 YOLO11 IR 元数据声明 `pad_value=114`，与当前填充值存在差异；
补边策略、填充值差异及尺寸取整对坐标与精度的影响，均留待结合实际部署模型和离线样例核验，
不据此认定代码缺陷或宣称反变换对所有尺寸精确。反变换须匹配实际推理预处理，
不能仅因训练或元数据提到不同补边方式，就单独添加偏移项。

- 路线：**网络发现目标 + 传统灯条局部精修**。精修是**可组合步骤**，不是替代检测器。
- 精修失败时**保留原角点并降低质量**，不得把失败结果包装成"更精确的角点"。
- 四角物理语义顺序必须是 **TL, TR, BR, BL**，且与模型训练标签、PnP 物体点定义**一致**。
- 不同 YOLO 导出模型的关键点索引**不可互相套用**；未知映射时保留图像 y 排序的兼容路径，但必须置 `corners_reliable = false`，**不允许开火**。
- 图像旋转后仅按像素 y 排序会交换物理角点——对应关系错了，PnP 低残差也未必是正确物理姿态。 **[定]**

输出必须分成两个量 **[定]**：

- `pose_valid`：存在可供后续处理的几何解；
- `pose_reliable`：该解足够可靠。

两者**不得**合并为一个"检测成功"布尔值。

### 5.2 PnP 候选生成与判别 **[定]**

平面 PnP（IPPE）给出多个候选，按下述条件过滤与评分：

数值实现采用等价求解坐标：θ 是输入 TL→TR 的像素边方向，先令 p′=Sp，
单次 IPPE 后恢复 R=R′S、t 不变；深度、法向及重投影误差仍用原物理点计算。

```text
S = [ cosθ   sinθ   0
      sinθ  -cosθ   0
        0      0  -1 ]
```

S 是 det=+1 的纯旋转，用于避开已复现的半周数值异常；不重排角点、不证明物理
语义，不增加 LM/单解/自动回退。零长度或非有限 TL→TR 边返回空候选。

| 判据 | 作用 |
| --- | --- |
| 正深度 | 全部四角在相机前方 |
| 逐角重投影误差 | 四角 RMS + 单角最大值，两个阈值分别设 |
| 投影退化程度 | 最短投影边长度下限 |
| 视线与法向夹角余弦 | 强侧视时降权 |
| 与**同一物理板**的先验连续性 | 距离 + 角度双门控 |

规则 **[定]**：

- 先验**只能**用于区分候选，**不能**用另一块板或另一个目标的姿态作连续性依据；
- 两个候选不可区分时，标记歧义并显式降低质量，不强行选一条；
- PnP 的可靠性证明绑定到其已检验的 `selected` 候选。Tracker 改选其他候选时，
  可以保留估计结果，但不得继承该证明；关联成功不等于新的角点/标定/消歧证明。
- **取消**对普通装甲板强制固定俯仰/零滚转的 yaw 修正——该先验对斜板、斜轴不成立；取消它**不代表**斜轴整车运动已被正确建模。

**大角度不是独立拒绝条件** **[定]**：允许真实存在的大 yaw / pitch / roll 姿态参与完整三维 PnP 求解，不强制固定俯仰或零滚转。角点对应、正深度、投影非退化、重投影误差、候选歧义和同板连续性满足要求时，可以接受该观测；板的真实存在本身不能证明某个 PnP 候选可靠，更不直接授予开火许可。

欠定时的处理 **[定]**：几何置信度不足、候选无法区分，或持续创新拒绝且当前几何/运动模型不能解释观测时，退回当前可见板保守跟随；该状态不允许开火，也不得从单帧制造其余板的位置。“明显滚转即降级”仅是未配置匹配几何模型的旧基线路径限制，不是新框架对滚转姿态的通用禁令。强侧视造成严重压缩、遮挡或病态解时仍须降权或拒绝。

输出 **[推]**：候选集合 + 协方差 + 四角顺序可信度。协方差用一阶传播

```text
Σ_pose ≈ (J_πᵀ Σ_u⁻¹ J_π)⁻¹
```

`J_π` 是像素对位姿扰动的雅可比。矩阵病态时**直接拒绝或降权，不强行反演**。该近似仅在线性化有效、角点对应正确时成立；存在多模态解时，单高斯不足以表达"两个可能姿态"，必须用多候选表示。 **[定]**

### 5.3 整车几何模型 **[定]**

```text
p_i(t) = c(t) + B(a) R_z(θ(t)) o_i
```

| 符号 | 含义 |
| --- | --- |
| `c(t)` | 旋转中心 |
| `a` | 转轴方向（世界系） |
| `B(a)` | 标定得到的右手正交轴基，z 轴对齐 `a`；还必须固定横向参考轴、相位零点与正旋方向，不能仅由轴方向任意选择绕轴旋转 |
| `θ(t)` | 旋转相位 |
| `o_i` | 第 i 块板在轴坐标系中、相位为零时的**已标定偏移** |
| 板法向 | **另行给出**，不可与位置混为一谈 |

**布局与轴方向是两个可组合维度** **[定]**：同高 / 两两同高 / 各自不同，与竖直轴 / 倾斜轴可以任意组合——同高装甲板同样可以绕倾斜轴旋转。两者不是互斥选项，必须分开表达。

三种布局均按轴坐标系中的板偏移表达：所有板同高、两两同高、各板不同高。倾斜轴下，轴坐标系高度不能被当作恒定的世界系高度。三种布局均须覆盖合成与回放用例，不能只预留三个配置名称。

板数约束 **[定]**：向量数必须等于该兵种的板数（两板 / 三板 / 四板）。提供四个高度数组**不等于**问题已解决。

在线与离线分工 **[定]**：

- 离线：机械测量/标定得到 `o_i` 与法向；
- 在线：**只**慢速修正少数参数，并对复杂模型加复杂度惩罚；
- **不**在线自由拟合全部高度与任意倾轴。

可观测性边界 **[定]**：只有一块板或短视角观测时，整车身份、相位与高度偏移**互相代偿**，不可联合观测；相同外观的四块板还存在相位/编号对称性。

首板身份 **[定]**：临时跟踪编号 0 不等于标定 profile 的物理编号 0。新框架通过身份关联、标定约定或有限身份假设确定两者对应；对应不确定时保留歧义，不得据此推断不对称布局的其余板并授权开火。旧实现“首块观测固定映射 profile 第 0 块”的路径，仅在这一对应有外部保证时兼容使用，不作为新框架的永久架构约束。

### 5.4 测量模型 **[定]**

测量模型负责：

1. **由状态预测观测** `h(x)`；
2. **定义残差**：位置为 `ν = z − h(x⁻)`，姿态为 `log(R_observed R_predicted⁻¹)`；
3. **提供所需线性化** `H = −∂ν/∂x`，在当前非零残差处求导；欧氏残差时等于 `∂h/∂x`；
4. 从观测中**提取量测与噪声** `R`。

`R` 随角点置信度、板面透视压缩、遮挡程度变化，**不对所有观测使用同一固定可信度**。 **[定]**

当前像素噪声按检测分数、视角余弦、角点可靠性启发式缩放；遮挡通过质量降级间接
反映，没有独立的遮挡程度估计量。传统 alignment、YOLOv5 objectness 和 YOLO11
最大类分数含义不同，均不当作已校准概率。PnP 使用未列归一化的 m/rad 信息矩阵比，
与标定的列归一化信息比区分，配置键及默认门限不因此改变。

职责分离的准确含义 **[定]**：运动模型、测量模型、数值实现是**职责分离但接口兼容**——不是可以任意组合不匹配的状态与量测。

观测契约应先**保留完整候选与质量**（三维姿态、PnP 多解），再由具体测量模型提取需要的分量；**不得**在观测结构里写死"位置 + yaw + 协方差"，否则斜板、完整三维姿态与多解信息过早丢失。 **[定]**

### 5.5 运动模型与状态 **[定]**

- **恒角速度是默认基线，不废弃。** 对明显非线性转速，增加**有界的**短时角加速度估计：

```text
θ(t+Δt) = θ(t) + ω Δt + ½ α Δt²
ω(t+Δt) = ω(t) + α Δt
α(t+Δt) = α(t) + w_α
```

- 角度残差必须按 `wrapToPi` 处理；旋转轴与刚体姿态应在**旋转流形**上更新，**不能**把欧拉角当无界普通向量相减。 **[定]**
- 运动状态划分为：平移或慢转 / 稳定旋转 / 突变或不确定。 **[定]**
- 正常运动模型选择带滞回与最小驻留时间，但不得推迟故障、过期或模式失配导致的撤销。运动模型改变不必然改变测量模型；相同状态布局保持兼容传播，状态维度/含义变化时必须显式映射状态与协方差，并为新增分量初始化不确定度，不能仅靠“膨胀协方差”完成转换。 **[推]**
- 自适应过程噪声必须带**硬上下界与滞回**，否则 Q 膨胀会把滤波器发散伪装成"收敛"。 **[推]**
- **按预测时长调整模型**：跨度一大，不可预测的转速变化迅速支配误差；应随预测时长扩张不确定度。 **[定]**

状态按**分量名**访问 **[定]**：否则替换运动模型或改变状态维数后，按下标访问的下游会静默取错分量。

公共状态现为 12 维：`x,y,z,vx,vy,vz,phase,omega,alpha,ax,ay,az`。原九个索引不变，
新增的 `acceleration_mps2` 默认零值。默认平移 CV 将 `ax/ay/az` 的状态和协方差行列
双向清零；实验平移 CA 传播 `p+v·dt+a·dt²/2`、`v+a·dt`、`a`，采用连续白 jerk
的完整积分噪声，保留位置/速度/加速度交叉协方差。角 CV/CA 的切换独立于平移模式，
不会抹除仍然激活的平移加速度。几何位姿量测对新增三维的当前时刻导数为零。

`tracker.initial_variance` 按上述顺序接受 12 项非负有限方差，也兼容原 9 项并追加
三个 `1 m²/s⁴`；默认 CV 随后清零失活项。其他长度或负数、非有限数拒绝。
显式启用平移加速度模式时，初次激活的不确定度独立初始化；关闭时清除对应交叉项。
不可变快照携带完整状态、协方差和匹配的运动模型，未来预测使用同一模型。

### 5.6 滤波与一致性检验 **[定]**

- **EKF 为默认数值实现**；CKF 作为**相同输入下**的离线对照，用于观察强非线性与大姿态变化下的残差、协方差与稳定性。
- CKF **不是**错角点、时间偏差、板身份混淆或错误旋转轴的补救手段；若只增加计算量而无改善，不替换 EKF。
- **第一版目标估计主路径选 EKF，不默认加入 ESO。** 这是阶段取舍，不是“目标不由我们驱动，所以 ESO 数学上不可用”的结论。ESO 或跟踪微分器用于短时加速度/总扰动估计，只作为后续有界、可验证的对照候选；云台控制环的扰动补偿是另一职责，不能与目标状态估计混为一谈。

<a id="experimental-eso"></a>

**实验 ESO（默认 CMake 关闭）**：`StateEstimator` 根据 `AUTOAIM_USE_ESO` 的编译定义选择，默认指向 `Ekf`。
`Eso` 提供相同的构造和 `state/covariance/motion/predict/innovation/update/change_motion`
接口及值复制语义；`Innovation`、`UpdateReport` 共用。它是目标运动估计器，不包含控制器。

- 平移使用三阶位置/速度/加速度 ESO；角 CV 使用二阶相位/角速度，角 CA 使用三阶。
  ESO 复制传入角运动模型并启用平移 CA，保留预测时域和角加速度约束。
- `EsoOptions` 的实验初值为平移、转动带宽各 `10 rad/s`，平移 jerk 功率谱密度
  `1 m²/s⁵`；额外构造重载允许离线实验显式传参。这些不是设备调参结果。
- 使用[离散极点配置](https://arxiv.org/html/2211.07309v2)而非连续增益直接乘固定帧率。
  `p=exp(-ωT)`；二阶增益为 `[1-p²,(1-p)²/T]`，三阶为
  `[1-p³,3(1-p)²(1+p)/(2T),(1-p)³/T²]`。实现用 `expm1` 与 `(1-p)/T`
  处理小时间步。首次 `T=0` 是独立初始化规则：只校正可观测位置和相位，不估导数。
- 首版只接受 6 维完整位姿残差及当前几何结构的雅可比：非零列仅允许中心位置和相位，
  噪声须正定，四维加权信息矩阵须可解。仅位置、退化或不支持的量测明确失败，不回退 EKF。
- 每次正时间 `predict` 固定该批状态 `x₀`、协方差 `P₀`，增益使用距上次有效校正的累计时间；
  `predict(0)` 不开启新批。同批后续残差回锚为 `r₀=r+H·difference(x,x₀)`，
  用 `J=H[:,x,y,z,phase]` 累计 `A=ΣJᵀR⁻¹J`、`b=ΣJᵀR⁻¹r₀`。
  各板沿用 Tracker 已缩放的噪声，但噪声缩放不能代替批量处理固定观测器增益。
- 每次接受观测均从同一先验重算 `x=x₀+G·solve(A,b)`，
  `P=(I-GC)P₀(I-GC)ᵀ+G·solve(A,I)·Gᵀ`，其中 `C` 选择四个可观测坐标。
  相位增量按环绕处理，角加速度按原模型约束；这是一阶局部误差传播，不宣称全局统计一致性。
- `innovation` 与 `update` 都在吸收当前观测前使用 `r₀` 和 `H P₀ Hᵀ+R` 计算同一 NIS。
  此处为批先验门控，与 EKF 的逐观测门控不同；回锚是一阶近似，不保证非线性观测严格顺序无关。
  拒绝或失败保留输出、模型、计时及已融合信息。模型切换先映射后验再清批；已经有效校正过的
  滤波器在零时间模型切换后，需要正时间预测才能再次更新。

`eso.hpp` 的完整算法及专用包含由 `AUTOAIM_USE_ESO` 编译定义控制。
`EsoOptions` 仍是正常编译的共享参数类型，快调配置通过 `TrackerOptions` 和构造辅助函数
传递；参数装载不会启用 ESO，也不改变已有三参数构造的默认值。
`tests/verify_alternatives.py --method ESO` 与兼容入口 `tests/verify_eso.py` 在独立副本中
通过 CMake 选择 ESO，保持副本源码不变，运行 OpenVINO OFF 验证；记录源码摘要、命令、
退出码、注册清单和失败输出，并检查原源码未被修改。CMake 按开关注册 `test_eso`；
不能将默认构建通过当成 ESO 已运行。
统一 12 维矩阵会影响计算规模；不以静态结构宣称性能、精度或 NUC 验收收益。

一致性检验 **[定]**：

```text
ν = z − h(x⁻)          ← 更新前
S = H P⁻ Hᵀ + R
NIS = νᵀ S⁻¹ ν
```

- 检验必须在**更新前**进行，用于在吸收该观测**之前**发现异常；
- 卡方阈值按**实际观测维度**选取，不是固定值；
- 复合观测需正确处理角度环绕与相关噪声；
- 若实际残差分布与卡方假设不符，用**真实残差分布**标定阈值，而不是只凭理论数值强行替换门限；
- **NEES 需要独立且对齐的状态真值**，可用于合成数据或具有可靠外部真值的实测/回放数据；无真值的在线运行不计算 NEES。把“后验减先验”当作 NEES 是错的，也不能作为估计一致性证据。 **[定]**

协方差更新使用 Joseph 形式。 **[定]**

健康监测 **[定]**：发散检测、NaN/Inf 检查、复位；估计器使用者与各任务的拒绝策略**分开**，避免修改共享滤波器时无意改变其他模块行为。

### 5.7 预测与拦截解算 **[定]**

`Tracker + EKF` 负责将状态预测到当前观测时刻并融合观测，输出不可变快照；`decision/predictor` 从快照的 `t_state` 出发，按对应运动模型向未来外推，不写回跟踪状态。EKF 是状态估计的数值实现，恒角速度/有界短时角加速度等是动力学假设，二者不是互斥替代方案。

核心条件：子弹与第 `i` 块板在**同一时刻**相交，且交点位于可击中的内部区域。

```text
g_i(τ, q) = n_i(t_fire + τ)ᵀ · [ b(τ; q, v₀) − p_i(t_fire + τ) ] = 0
```

- `b(τ; q, v₀)` 是给定发射方向 `q` 与初速 `v₀` 的弹道；飞行时间与目标未来位置**相互依赖**，用少数次定点/牛顿迭代；
- 必须有**迭代上限、收敛判据、无解返回**；
- **无解时不得代入未初始化或虚构的飞行时间与弹速**；弹速非法、弹道不可解、结果为 NaN/Inf 时，**不得继续使用**；
- 同时要求交点在板局部多边形**内部**，且云台能在 `t_fire` 到达该方向；
- 各入口共用同一套拦截计算，**不再**由入口各自额外减 1 ms 或加固定提前量。

弹道模型 **[定]**：第一版用无阻力抛体；只有靶场标定表明空气阻力或初速变化成为主要误差时，才换用

```text
v̇ = g − κ‖v‖v
```

或实测射表。**若角点、外参和延迟的系统误差仍大，先拟合阻力系数收益有限。**

预测的参考量 **[定]**：规划参考误差**不能**替代实际云台误差。必须用**实测云台反馈**判断当前指向。

### 5.8 瞄准充分性 **[定]**

`decision` 只输出**证据**，不产生开火使能。证据内容：

| 证据 | 判据 |
| --- | --- |
| 指向误差 | 目标角 − **实测云台姿态**（不用规划参考） |
| 持续性 | 误差连续 N 个被接纳的观测在容差内，同时限定最短持续时间与最大观测间隔；长间隔、过期或无效观测中断连续证据，不能靠重复发送同帧凑数 |
| 人工接管情况 | 步兵自动模式必须具备可靠、新鲜的人工介入判据；发现介入时按 §6.4 撤销自动控制。信号来源与阈值待核实，不以缺失或恒零数据冒充“无人干预” |
| 可解性 | 弹道可解、弹速合法 |

两条硬规定 **[定]**：

1. **容差来自安全配置，不由产出轨迹的组件提供**。否则生产者既生成轨迹、又提供评判自己轨迹的标准，是自我认证而非判据。
2. **"可以跟踪"不等于"可以开火"**；两者分层。

不确定度的用途 **[定]**：相位不确定度覆盖多个可能装甲板窗口时，降低置信度并进入保守跟随、预瞄窗口或暂停开火状态。当前不确定度门限**主要约束相位**，尚未完成位置、时钟、弹速、散布的统一三维命中概率计算——**因此不得把"所有软件门限通过"表述为保证命中**。

上述“主要约束相位”保留旧规格表述；与现有落点实现的区别见 [§10.4](#spec-status-notes)，不作为当前实现的完整描述。

### 5.9 开火许可与联锁 **[定]**

**最终开火使能只有一个来源**：`control/command_guard`。

**请求不等于许可。** `mission` 根据当前授权模式、任务策略和操作输入形成 `fire_requested`；步兵辅助模式下该请求恒为 false，不因瞄准充分性通过而提升。`decision` 提供瞄准充分性等证据。`pipeline` 组装 `ControlIntent` 并向 `publisher` 提交。

`ControlIntent` 是许可检查的输入，表达请求，不携带上游可设置的最终开火使能。`command_guard` 的检查结果才包含 `fire_allowed` 和拒绝原因；两者不得混为同一个字段。

程序对外输出的最终 `shoot` 取本次发送复检后的 `fire_allowed`，不得直接使用上游的 `fire_requested`。哨兵的程序开火依据该最终值；步兵自动模式同样经过许可检查，辅助模式的程序 `shoot` 保持 false，手动开火由操作手决定。

首次检查由 `command_guard` 根据请求、证据与联锁条件形成许可：

```text
首次许可 = fire_requested AND 证据合格 AND 联锁通过
```

对同一次接纳、缓存的指令，发送前仍由 `command_guard` 复检，并将结果保留用于该指令的后续发送：

```text
本次发送许可 = 上次许可 AND 当前检查通过
```

已撤销的许可不得因反馈恢复而自动恢复；必须提交新意图并重新接受首次检查。新意图不得刷新旧观测的源时间，也不能绕过故障锁存等联锁。未请求开火时，无论证据是否合格，都不得产生开火许可。

**调用关系**：`pipeline` 向 `publisher` 提交意图；`publisher` 调用 `command_guard` 完成首次检查和发送前复检，并由其唯一发送线程通过注入的写回调执行写出。`command_guard` 只负责检查，不反向调用 `publisher`，也不执行传输写入。

拒绝条件（下表任一条件**成立**即撤销程序开火许可，且不会在同一条已接纳指令的后续发送中重新提升）：

| 类别 | 拒绝条件 |
| --- | --- |
| 权限 | 当前模式不允许程序自动开火（如步兵辅助模式），或步兵已经发生人工接管 |
| 能力 | 时间标定未完成（未实测） |
| 时间 | 源帧时间缺失、来源过旧、控制时效或开火时效超限、源时间来自未来、指令到期 |
| 姿态 | 姿态反馈无效 / 过期；图像对应的历史姿态与当前反馈分别检查 |
| 弹速 | 弹速非有限、不大于零 |
| 世代 | 属于上一模式的结果 |
| 数值 | 控制量非有限 |
| 目标 | 目标不可用于开火、位姿不可靠 |
| 故障 | 故障锁存器已置位 |

规则 **[定]**：

- 缺少显式源帧时间的兼容指令**可以**受跟踪时效限制，但**不能**通过补填最新帧时间获得开火许可；
- 首次检查依据请求与证据形成许可；发送复检只**保留或撤销**同一次接纳指令的已有许可，不得重新提升；
- 模式切换以主机确认时刻为分界，清除此前的帧有效性与反馈有效性；分界之后需要新的有效帧与有效反馈才能恢复；
- 拒绝后的停止指令**保留源帧号与源时间**供诊断，但控制量清零；
- 诊断原因位只在本机内部传递，**不改变协议**；
- 日志在控制包写入**之后**输出，不在联锁锁内做日志 I/O。

故障处理 **[定]**：

- 写回调结果区分 `complete` 与 `failed`；首次故障被保存，缓存指令丢弃，后续提交被拒绝；
- 发送线程**至多再尝试一次**停止写入；停止写入失败单独保存，**不覆盖首次故障，不循环重试**；
- 三种状态必须分开记录：**停止请求已提交** / **主机完整写入** / **设备已确认停止**。第三项在没有回执时**永远是"未确认"**，日志不得宣称设备已停止；
- `close` 是本次运行会话的终止操作，仅用于退出/故障，**不用于普通模式切换**；
- 工作线程边界捕获异常，清空待处理输入、使在途结果失效、提交一次停止请求；
- 入口在等待图像**之前**检查工作线程与通信线程故障；无图像不得无限推迟故障检查。

### 5.10 控制与发布链 **[定]**

- **唯一写出**：`publisher` 管理发送线程，只保存最新指令，限制最高发送频率；
- **再次检查**：唯一发送线程在写出前**再次**检查有效性——"唯一接纳点"不妨碍这一点；
- 生产线程卡顿后，发送端仍能按源时间撤销开火；超过跟踪时效后输出停止指令；
- 正常修正通道的启停采用限幅、限速、低通、死区与滞回，正常退出可平滑归零；故障、过期、模式失配的撤销/停止优先，不得为等待驻留期或平滑完成而保留失效使能或控制量；
- 主机内部保留单调指令序号用于溯源；旧线协议没有相应字段时不擅自增加。下位机序号、有效期或回执属于需双方确认的协议扩展；序号只能辅助判断顺序，不能代替通信超时看门狗。

**主机侧不能保证的事** **[定]**：如果整个主机或进程停止调度，主机**无法**保证继续发送停止包。这必须由下位机独立的通信看门狗保证——该能力尚未取得固件证据。

### 5.11 数据关联与身份 **[定]**

关联解决的是"这个观测对应哪块**物理**装甲板"，它**不是**分类器标签的转述。

| 步骤 | 内容 |
| --- | --- |
| 门控 | 马氏距离 `d² = νᵀ S⁻¹ ν`，阈值按卡方分布与**实际观测维度**取 |
| 代价 | 位置 + 姿态 + 身份假设的联合代价；观测数与目标数都很小，不追求复杂分配算法 |
| 输出 | 关联结果**与身份不确定度** |

硬规定 **[定]**：

- 只有在**能证明是上一帧同一块物理装甲板**时，才可用上一帧 PnP 结果作为先验；
- 关联不能证明同一性时，默认不传先验；完成并验证同板关联后可传带来源的先验。旧 Tracker 尚未实现这项能力，是旧实现状态，不是禁止新框架使用关联先验的永久限制；
- 身份不确定或几何证据不足 → 退回当前可见板跟随，该状态**不允许开火**。

对称性提醒 **[定]**：外观相同的四块板存在相位/编号对称；四块同高时循环置换几乎不可观测。如果首次观测认错了板，profile 的整体偏移会被循环置换，而滤波器仍会输出连续、协方差也不大的结果——**对称歧义下，低残差和小协方差不能证明身份正确，必须结合身份检验、标定与相位证据**。

### 5.12 跟踪状态机 **[定]**

- 跟踪生命周期与估计质量分开表达：生命周期为未初始化 / 初始化中 / 跟踪 / 暂时丢失；质量为未收敛 / 已收敛 / 降级。已收敛不是与“跟踪”互斥的生命周期。
- 迁移条件明确最小检测次数、最大观测间隔、丢失时限、收敛与退化判据；计数不能代替真实时间。
- 每次迁移是显式事件，携带新状态、理由及必要的状态/协方差处理。仅在模型转换或质量变化需要时调整协方差，不对每次状态变化无条件注入噪声。
- 正常恢复、模型选择与选板切换采用滞回和最小驻留，故障、过期、数值非法及模式失配立即走撤销路径，不受驻留限制。
- 目标丢失超时后清除目标与相关指令；模式切换使上一模式的结果失效。
- “暂时丢失”期间可以用预测维持跟踪，但不得用预测结果刷新源时间或延长开火许可。

### 5.13 选板 **[定]**

选板的决策时刻取同一批拦截候选的 `estimated_send`，必须一致且严格前进。
同一源观测可以在新决策时刻重新外推；不得改变源帧号/曝光时间或回退源观测。
Pipeline 在无新观测的预测保持阶段只请求瞄准控制，不请求新的开火许可；指令到期
受原曝光控制年龄上限约束。重复观测不增加瞄准充分性的支持帧数。

选板承担候选过滤、稳定切换与策略评分；具体评分目标及开火时机策略按阶段范围实现和验收。

| 环节 | 内容 |
| --- | --- |
| 候选过滤 | 可见性、板面朝向、云台可达性、被遮挡程度 |
| 稳定性 | 切换需要**持续**优势，带滞回；避免在两块板之间反复跳变 |
| 安全判定 | 预计落点是否落在板内、相位不确定度是否覆盖多个窗口 |

策略输出仍须经过瞄准充分性评估和控制许可检查。

### 5.14 命中概率与落点 **[定]**

理想规则：

```text
Pr{ 落点在有效装甲多边形内 | 当前观测 } ≥ η
```

实现路径 **[定]**：

1. 滤波协方差传播到 `t_hit`；
2. 经交点雅可比映射到装甲板**局部二维平面**，得到落点均值 `μ_hit` 与协方差 `Σ_hit`；
3. 实时实现**不**每帧做大量蒙特卡洛，而是二者之一：
   - 计算均值点到板边界的**最小安全边距**，要求该边距覆盖相应方向的 `kσ`；
   - 或在临近开火时对少量候选做**固定数量**采样。

近似方法与概率门限的关系 **[定]**：`kσ` 边距和固定数量采样不是自动等价的概率判据。前者须说明分布假设、各边覆盖与联合覆盖的关系，后者须说明采样方式及有限样本误差；未建立与 `η` 的映射并校准前，只能报告为几何/不确定度代理指标。尚未建模的时钟、弹速和散布误差不得当成零。

边界 **[定]**：该机制**不能**保证真实命中率等于计算概率，概率必须结合打靶数据校准。当前阶段不确定度门限**主要约束相位**，位置、时钟、弹速与散布的统一三维命中概率**尚未完成**——因此不得把门限通过当作命中保证。

本段与现有局部二维协方差实现的区别同样见 [§10.4](#spec-status-notes)；软件代理指标不能替代真实概率校准。

### 5.15 标定 **[定]**

内参拟合必须检查跨拟合/验证集的重复解码图片及有序点集。低 RMS 不单独证明
内参可辨识：针对针孔五畸变参数，消去各图六维位姿，再对九维内参信息矩阵按列
归一化并检查数值秩。`1e-10` 是数值病态下限，不是实测误差门限。
内参报告的 `intrinsic_quality` 保存方法、信息矩阵和样本内容指纹；导入端重新检查
矩阵、固定数值门限及样本划分绑定，不信任 `passed`。旧报告缺少质量块时仅作候选
参数，不授予内参能力。FNV-1a64 去重不是密码学签名，样本真实性仍需人工核对。

每一项标定都必须产出**证据**（设备标识、配置版本、测量日期、测量方法），而不是一个布尔声明。

| 标定项 | 内容 |
| --- | --- |
| 相机内参 | 内参矩阵与畸变系数 |
| 相机→云台外参 | 旋转 + 平移 |
| 云台→世界/车体 | 姿态约定、轴方向、旋转方向 |
| 时间 | 曝光中点与姿态时钟的固定偏移与抖动；接收时刻定义 |
| 几何 | 各板偏移 `o_i`、板法向、转轴方向 |
| 弹道 | 真实弹速、通信/执行/发弹延迟 |

姿态约定必须逐项核实**不能由代码编译通过代替**。外置 IMU 路径使用其对应时刻的完整姿态；包内四元数只有在轴方向、旋转方向与外参均确认一致后才能启用（仅验证范数正确不够）。

### 5.16 协议与校验 **[定]**

- 用户已选择 14 字节 CBoard UART v2 发送格式，对应旧参考代码的 `NEW_UART_PROTOCOL`：`A5 / length=14 / control / shoot / yaw:int16 / pitch:int16 / dist:int16 / CRC16 / tail=0x7891`。多字节字段小端，CRC 仅覆盖前 10 字节；角度 rad、距离 m 均乘 10000 后向零截断，距离沿用 `[0, 3.2767]` m 饱和。主机内部保留原距离。
- `default_command_protocol = ProtocolKind::cboard_uart_v2`；未显式指定协议的 `encode(command, metadata)` 和离线工具 `--encode-stop` 使用该选择。其余既有协议只作为显式兼容选项保留。接收仍为 `AB` 43 字节反馈；选择格式不代表固件语义、权限或实机能力已经验证，不开放硬件入口。
- 帧格式与现有串口/CAN 协议保持兼容；
- CRC16-CCITT 校验；
- 流式解析必须处理**半包与粘包**；
- 保持既有线协议不变；主机内部序号不等于协议已支持序号。若增加序号、有效期或回执，必须先确认固件支持和兼容方案；无论是否有序号，下位机独立超时清零能力均须取得固件与实测证据。

---

## 6. 任务与控制权威

### 6.1 首版控制模式矩阵 **[定]**

| 角色 / 模式 | 云台控制与指令空间 | 程序开火权限 |
| --- | --- | --- |
| 哨兵自瞄 | 程序控制云台，使用自主控制的绝对指向通道 | 开火完全依赖程序最终 `shoot`；该值来自 §5.9 的许可检查及发送复检 |
| 步兵辅助模式（默认） | 人工为主，程序仅提供相对瞄准修正，不占用独立自动控制权 | 不允许程序自动开火，`fire_requested` 与程序最终 `shoot` 均为 false；开火由操作手决定 |
| 步兵自动模式（可选） | 经明确启用后，程序使用自动控制云台的绝对指向通道；人工介入可撤销该控制权 | 允许程序申请自动开火，但最终 `shoot` 仍由许可检查决定 |

“步兵只拥有相对修正通道”仅适用于步兵辅助模式，不能作为对步兵角色的永久限制。打符是后续任务，不能仅凭其任务名称获得自主控制权限。

### 6.2 人工优先与接口证据

**行为要求 [定]**：步兵以人工控制为主，默认处于辅助模式；自动控制云台并开火是必须明确启用的可选模式。步兵的程序 `shoot = false` 表示撤销程序开火，不用于否决操作手的手动开火，也不能被当成设备必然没有发射的反馈。

**接口前提 [未验]**：辅助模式的相对修正如何与人工输入叠加、自动模式的绝对指向如何获得和让出控制权、手动开火是否绕开 NUC、操作输入从哪里回传，以及 `shoot` 如何映射到既有协议字段，都须核对本机固件与接线。用户确认的控制意图不能代替这些接口的实测证据；但接口未知也不能把“可选自动模式”悄悄改回“步兵永远只能辅助”。

辅助模式不在 NUC 内重新合成人工的绝对指向；操作输入用于干预检测与权限仲裁。自动模式由程序生成自己的指向目标，但判断当前指向仍须使用实测云台反馈。主机输出、主机完整写入与设备实际执行分别记录，遵循 §5.9–§5.10 的通信边界。

### 6.3 角色 / 任务 / 权限分离 **[定]**

`--profile=infantry|sentry|rune` 混合了机器人角色与任务模式。应分别表达**机器人角色**、**任务模式**、**控制模式（辅助 / 自动）**和**控制权限**，并校验允许的组合。步兵辅助与步兵自动使用同一角色的不同控制模式；后续切换到打符任务也不自动改变权限。具体组合须显式配置，并验证对应的固件与操作输入契约。

**允许的组合只在一处判定**，其他组件调用它。入口只负责选择配置，统一走公共启动流程——归档中一次故障处理需要修改 **14 个运行入口**，这正是要避免的维护成本。

### 6.4 步兵人工接管与重新启用 **[定]**

1. 只有操作手明确启用并满足当前许可条件，才可从辅助模式进入自动模式；瞄准就绪、重新检测到目标或输入回到死区都不是启用操作。
2. 自动模式中识别到人工介入时，立即撤销程序开火、让出自动云台控制权并退回辅助模式；不等待普通平滑归零或最小驻留时间结束。
3. 该切换推进模式世代，使此前自动模式的意图、缓存指令和在途结果失效；程序不得因旧结果晚到而重新控制云台或开火。相对辅助修正也须来自新模式下满足时效与质量条件的结果。
4. 操作手停止干预后保持辅助模式；只有新的明确启用操作才可重新进入自动模式，并重新检查证据与联锁，不恢复旧许可。
5. 人工介入信号的来源、有效性、判定阈值以及明确启用操作的具体输入形式仍待确认。信号缺失、过期或来源未验证时，不得用默认零值证明“无人干预”并授予自动开火许可。

以上定义软件控制权的转移；实际交权与停火生效须由固件适配和人工验收确认，不能仅凭主机改变一个变量宣称设备已经执行。

---

## 7. 并发与时序规格

以下不变量及 §7.1–§7.3 的调度策略已随实施计划确认；具体设备容量、时效阈值与后端取消能力仍待测量。合成测试参数不作为实机默认值。

| 规则 | 说明 |
| --- | --- |
| 计算可并行，**状态只有一个写者** | 检测/PnP/精修可多线程；估计状态只能由单一线程更新 |
| **帧序单调前进，允许跳帧** | 不为等待更早的慢帧而阻塞 |
| 接纳点唯一 | 过期结果在唯一接纳点丢弃；**发送端仍会再次检查过期** |
| 姿态查询非消费式 | 同一图像被检测、瞄准、调试多次读取，不因第一次读取而弹出样本 |
| 姿态历史 | 用非消费式历史插值；**范围之外、采样间隔不合理、非法四元数返回无效**，不靠无限等待或外推制造同步成功 |
| 异步输入生命周期 | 异步任务必须持有**预处理后的**输入缓冲区直到推理结束；**仅保存原图不足够** |
| 队列策略 | 图像走最新帧优先的有界待处理队列，异步推理限制在途请求数；待处理帧淘汰与已完成结果接纳分别判断，不能统一简写为“凡不是最新采集帧都丢弃”；不靠积压旧帧换取表面 FPS |
| 丢帧记账 | 丢弃必须可计数：记录丢帧数与过期比例，否则"少发困难帧"会伪装成延迟改善 |

### 7.1 容量与所有权 **[定]**

- 分别限制图像缓冲池容量、待处理队列容量和异步在途任务数；三者不是同一个数量，不能只限制等待队列却允许在途任务无限增长。
- 缓冲池占用按实际缓冲块计数，不因同一块内存有多个引用而重复计数；原图和预处理输入的存储分别管理，均不得超出其预算。具体容量不在文档中猜定，由帧大小、处理链和目标机测量确定。
- 入队、淘汰、任务领取以及相关容量记账必须由一致的同步机制保护，防止某帧刚被工作线程领取又被回收。
- 逻辑丢帧不等于物理回收：正在推理、PnP 或仍被其他读者使用的缓冲区，必须等使用者结束且租约归还后才能复用；过期、模式切换或结果作废都不能提前解除该生命周期约束。
- 是否主动取消在途请求须根据后端的真实能力和取消完成语义决定；未确认取消完成前，不按“已经释放”计算容量。

### 7.2 待处理帧与缓冲池超限 **[定]**

在接纳新帧之前检查容量，不先超限分配再事后补救。新帧按以下顺序处理：

| 顺序 | 条件 | 处理 |
| --- | --- | --- |
| 1 | 新帧数据无效、已经过期或属于旧世代 | 直接丢弃新帧，不启动新的计算任务 |
| 2 | 等待队列中已有过期或旧世代帧 | 先将这些帧移出等待队列；仅在租约允许时回收内存 |
| 3 | 仍达到队列或缓冲池上限，存在可安全回收的待处理帧 | 从最旧的可回收待处理帧开始淘汰，直到本次接纳满足所有容量限制 |
| 4 | 已无可安全回收的待处理帧，缓冲区均被在途任务或其他读者占用 | 丢弃新帧；不覆盖在用内存，不无限等待，也不临时突破容量上限 |
| 5 | 已有可用缓冲区且各项容量检查通过 | 接纳新帧，保持其原始帧号、世代与曝光时间；工作线程领取当前最新的有效待处理帧 |

示例仅说明规则，不设定项目容量：容量为 4，101、102 正在处理，103、104 等待处理，105 到来时淘汰可回收的 103，保留 104、105；不能因 101 更旧就覆盖它。若所有缓冲区都仍有使用者，则丢弃 105。

尚未进入检测的图像不按“检测置信度最高”挑选，因为此时没有检测结果。淘汰依据为世代、时效、处理状态与采集帧序，不能通过跳过困难画面来美化性能统计。

### 7.3 完成结果接纳与丢帧记录 **[定]**

完成结果在进入估计器前，由统一接纳点检查当前世代、源时间与时效、帧序及契约质量；通过后才进入单写者的跟踪处理序列。

- 乱序判断相对**同一世代最后已接纳帧**，不是相对最新已采集帧或最新已提交任务。结果帧号必须向前推进；进入新世代时重新建立该世代的接纳记录，不能因此接纳旧世代结果。
- 101 已先被接纳时，迟到的 100 必须丢弃；若 101 仅开始计算、100 此时完成且满足其他条件，则不得仅因已有 101 在途而丢弃 100。
- 不为等待更早的慢帧而阻塞已满足条件的较新结果；Tracker / EKF 仍由单一线程更新，并按实际观测时间间隔传播，不把跳帧误当成固定帧率更新。
- 结果过期或失效时拒绝使用其内容，但仍按 §7.1 归还租约；任何丢弃、外推或重发都不刷新源曝光时间，也不延长旧结果的开火许可。
- 分别记录容量淘汰、过期、旧世代、乱序、输入无效和无空闲缓冲区等原因。区分“帧被丢弃”与“同一缓冲区仍被占用”，同一次丢弃不重复计数。

---

## 8. 验证与验收 **[定]**

### 8.1 必须为零

越界、未初始化的飞行时间、非有限的控制指令、已过期的开火结果。

### 8.2 测试分层

| 层 | 覆盖 |
| --- | --- |
| 单元 | 各模块基本正确性 |
| 合成 | 合成真值验收：三种板高布局 × 竖直/倾斜转轴，覆盖身份歧义与模型切换 |
| PnP 专项 | 大 yaw/pitch/roll 的非退化真板应可被接纳；强侧视退化、错角点、多解与遮挡应按质量降级或拒绝，不按“大角度”一刀切 |
| 回放 | **复用在线处理链**，只替换输入、时钟与输出端；结果可复现 |
| **契约** | 模式切换后的旧结果、乱序完成、异步缓冲区生命周期、工作线程异常、发送失败与关闭竞态；步兵辅助禁止程序开火、人工接管与明确重新启用 |
| 故障注入 | 丢帧、坏帧、野值、CRC 位翻转、半包、未收敛 |

这里的分层表示覆盖职责，不要求每层有独立同名目录。回放与故障场景已分布在
现有单元、契约和合成测试中；删除未注册的空测试占位不代表删除这些覆盖。
实际测试名称与数量以对应构建的 CTest 注册清单为准。

已确认的控制模式验收应覆盖：辅助模式即使上游误提交开火请求也不能得到许可；自动模式中人工介入会撤销程序开火并使旧世代结果失效；人工松手不能自动恢复，明确重新启用仍须重新接受许可检查。

§7 调度策略必须覆盖契约用例：淘汰最旧可回收待处理帧、全池在用时丢弃新帧、领取/淘汰竞态、101 已接纳后拒绝 100，以及 101 仅在途时仍可接纳有效的 100。设计覆盖不等于测试已经存在或通过。

### 8.3 指标 **[定]**

关注**源曝光到首次完整写入成功的 P95 延迟、有效控制比例、过期结果比例**，而不是平均 FPS，时间口径遵循 §3.1。

保留旧项目“P95 降低至少 20%”作为优化目标，不作为已达成结果。对比必须使用同一台目标 NUC、同一素材、同一输出/丢帧口径；具体基线与门槛随验收方案记录，不能通过减少困难帧输出来制造改善。

指标必须**按"操作手是否在干预"分层统计**，否则人工操作会污染对自瞄本身的评估。仅统计实际发送的有效目标结果会产生选择偏差，必须同时记录无目标比例、丢帧/过期数量。

### 8.4 禁止写出的结论 **[定]**

- 未在 NUC 实测的 FPS、P95 延迟、命中率数字；
- 用积压旧帧换来的表面 FPS 提升（这是退步，不是优化）；
- 把"所有软件门限通过"表述为保证命中；
- 把 WSL 的耗时当作目标机的实时性能。

---

<a id="unverified"></a>

## 9. 未验证假设 **[未验]**

| # | 假设 | 若为假的后果 |
| --- | --- | --- |
| 1 | 下位机独立超时清零；若扩展协议，能正确消费序号/有效期 | 主机冻结后，单靠主机逻辑不能保证停止；序号本身不提供超时保障 |
| 2 | 云台回传里的操作手输入量在实机上会变化 | 该字段恒零则"未受干扰"永远成立，是最危险的 fail-open |
| 3 | 时间同步误差可测且已知 | "曝光到发送 P95"含未知偏置，实为"估计曝光到成功发送" |
| 4 | 三种布局下的板身份可观测性 | 四块同高时循环置换几乎不可观测；`axis_in_world` 默认值会静默断言"转轴竖直" |

以下项目**仍需实际设备和样本**才能完成，不能把"代码存在"当作已验收：相机曝光中点与姿态时钟的偏移及抖动标定；云台坐标约定、反馈时效、通信/发弹延迟、真实弹速标定；倾斜转轴及不同安装高度的多视角/多相位标定；同一台 NUC 上 CPU/GPU 与各检测模型的整链路 A/B；真实目标、噪声、遮挡、强侧视、丢帧与模式切换下的实机命中验收。

---

<a id="decisions"></a>

## 10. 已有决策与仍缺的具体选择

### 10.1 已确认的工程约定 **[定]**

已回答的问题不再列为“未作答”。工具链、测试框架、代码风格和变更边界统一见 A5、A8。
详细规格集中维护在 `additional_information.md`，README 保留入门与简明导航，不维护第二份逐文件清单。

| 已确认选择 | 依据 |
| --- | --- |
| 首版装甲板闭环；打符、MPC 不在首版实现范围 | §1.5；打符预留文件见 M7 |
| 步兵默认辅助、可选自动、人工接管后不自恢复；哨兵依据最终 shoot | §6、§5.9 |
| 中心优先选可靠目标并保持锁定，不增加兵种优先 | §1.5、M7 |
| 三种高度布局与竖直/倾斜轴使用统一 profile | §5.3 |
| 独立按键默认 toggle、备选 hold，旧 enable_event 保留显式启用语义 | §1.6；软件状态不代表真实输入来源已验证 |
| 有界队列与丢帧策略 | §7.1—§7.3；设备容量和时效阈值仍需实测 |

### 10.2 仍需具体选择或实测的项目 **[开]**

| 项目 | 已知边界与仍缺内容 |
| --- | --- |
| NUC 与 SDK 运行环境 | WSL 工具链记录见 A8、§11；实际部署和 SDK 运行时仍待人工验证 |
| 性能验收 | §8.3 保留 P95 至少降低 20% 的目标；仍缺同机基线素材、测量口径与实际门槛记录 |
| 人工介入与启用输入 | 软件按键方式已确认，仍缺设备信号来源、采样时效、介入阈值与物理按键映射；退回辅助且不自动恢复的行为不再开放讨论 |
| 固件与权限适配 | §6 控制矩阵已确认；实际相对/绝对通道、交权、手动路径及 shoot 映射须核实，后续打符权限另行定义 |
| 异步调度实测 | §7 策略已确认；设备容量、时效阈值与后端取消能力待实测，测试参数不作为实机参数 |
| 协议扩展 | 增加序号/有效期/回执前须核实固件支持及兼容方案；未确认前保持现有协议 |
| 硬件证据 | 时间标定、下位机看门狗、回执及其它设备能力见 §9，审批文档不能升级为实测事实 |

### 10.3 README 同步规则

每次修改项目内容，在同一轮同步 README 的相关说明；不能等到全部功能完成后才集中补文档。
三份文档分工如下：

- README：入门、构建运行、配置工具、模块速查及当前验证边界。
- 本手册：M1—M9 的逐文件职责、技术契约、协作约定和按阶段记录的实施历史。
- build_history.md：构建环境、测试矩阵、逐批修复日志及证据入口；保留日期和未验收限制。
- 修改接口或职责时同步本手册；新增构建验收日志写入 build_history.md，README 保留概要与入口，
  不重复维护详细日志或清单。
- 状态必须区分已实现、已验证、待完成；只读讨论不产生完成记录，历史测试不当作本次执行结果。

首版只有标定/PnP 检查，2026-09-29 的 S2 才增加内参、手眼与报告求解能力，见 §11.1、§11.2。
旧版“标定求解未实现”的描述属于首版历史，不再用作当前功能说明。
完整命中概率、阻力弹道、在线几何慢修正仍不能由现有文件名或设计要求推定为已实现；
首版不含打符，角色或任务名称也不自动授予权限。

<a id="spec-status-notes"></a>

### 10.4 旧规格状态表述的待核查点

以下区别在文档整理时明确标出，不修改原算法要求、不增加能力证据：

| 原表述 | 阅读边界与后续核查 |
| --- | --- |
| §5.8、§5.14 的不确定度“主要约束相位” | [hit_probability.cpp](src/decision/hit_probability.cpp) 已有位姿、指向、原点、弹速、发射时序与散布项的局部二维协方差传播，M6 说明的是该实现；旧表述不能直接作为现状结论。各误差项的实际覆盖、相关性与实测标定仍需专项核查，不等于完成统一三维命中概率或真实命中率校准。 |
| §6.4 第 5 条将明确启用的输入形式列为待确认 | toggle/hold 与旧 enable_event 的软件语义已在 §1.6、M7 明确；待确认的是设备输入来源、时效、阈值与物理映射，不要求重新选择软件模式。 |
| §9 第 2 项关于云台回传操作量的旧假设 | 当前 AB43 适配不从不存在的字段推导操作手输入，独立输入见 M7、M9。该项仍提醒核实真实信号，不证明 UART 已提供该字段。 |

---

<a id="implementation-history"></a>

## 11. 实施与验收历史

本节按实施日期保存已完成阶段的授权、实现与验证记录，**不是当前待执行清单**。
首版记录见 §11.1，离线优化包见 §11.2，最近的分批瘦身验收及回滚依据见 §11.4。
下文测试数字均保留其原阶段含义，不表示 2026-09-30 的文档整理重跑了测试。

这些阶段记录的验证环境为 WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / OpenCV 4.5.4 /
Eigen 3.4.0 / yaml-cpp 0.7.0 / OpenVINO 2026.3.1；具体测试配置及后端范围见各阶段记录。

### 首版采用的实施顺序（历史）

2026-09-29 首版采用以下次序，替代更早的三阶段顺序：

1. 最小工程：约定同步 → 最小构建/测试 → 九模块依赖检查。
2. 基础类型：结果 → 单位/时间 → 溯源 → 证据 → 日志 → 配置。
3. 基础数学：角度 → 数值检查 → 刚体变换 → 坐标转换。
4. 数据与时序：主契约 → HAL/替身 → 姿态历史 → 缓冲租约 → 有界队列 → 完成接纳 → 命令槽 → 调度器。
5. 控制通信：CRC → 协议 → 时效 → 平滑 → 许可检查 → 发布器。
6. 视觉：标定 → 传统检测 → OpenVINO/工厂 → 有界异步 → 角点语义/精修 → PnP/质量。
7. 估计：观测 → 几何 → 运动/测量模型 → EKF/健康 → 关联/身份 → 几何选择 → 状态机 → Tracker/快照。
8. 决策：外推 → 弹道/拦截 → 选板 → 落点代理指标 → 瞄准充分性。
9. 任务：组合校验 → 选目标/锁定 → 步兵辅助/自动/接管 → 哨兵。
10. 适配：文件输入/逻辑时钟 → 记录输出 → 已知串口/CAN → 相机；硬件只构建不运行。
11. 集成：bootstrap → pipeline → 全链路/切换/退出 → 薄入口 → 同链路回放。
12. 收尾：合成/协议/标定/检测/可视化/指标工具 → 完整构建/配置/README → 全项目审视与回归。

首版按箭头间的小项独立完成接口、实现、测试和验证后继续。设备接口缺失时只完成离线逻辑，实机能力禁用，不伪造字段或标定。

<a id="history-baseline"></a>

### 11.0 实施前基线（2026-09-29；非当前状态）

| 实施前基线（2026-09-29） | 含义 |
| --- | --- |
| 138 个源文件为 0 字节占位 | 还没有任何实现 |
| `CMakeLists.txt` 为 0 字节 | 还不能构建，依赖方向尚未被机械强制 |
| 没有构建脚本、没有 CI | 任何"能编译""能跑"的声明目前都不成立 |

这张表只记录首版开工前的状态，不能用来判断当前仓库是否已实现或能否构建。

<a id="history-first"></a>

### 11.1 首版历史实施记录（2026-09-29）

首版第 1–12 项的软件离线实现已逐项推进、验证后连接。以下为该阶段状态，不包含真实精度、设备接入与实机验收；后续能力及结果分别见 §11.2、§11.4。

| 项目 | 首版阶段状态 |
| --- | --- |
| 最小工程/基础类型/数学 | 九模块静态库、包含/直接依赖白名单、强契约、单位/时间/证据、日志/配置、SO3/SE3/数值检查已实现 |
| 数据时序/控制通信 | 历史姿态、有界租约/队列/完成接纳、命令槽/调度器、旧 CRC/协议、平滑、首次许可/发送复检、唯一写者与故障/退出已实现 |
| 视觉 | 传统灯条、旧 YOLOv5 OpenVINO、固定有界异步输入、角点映射/精修、IPPE 多候选与质量/协方差已实现 |
| 估计/决策 | 统一布局/转轴、恒角速度及有界角加速度、测量模型、EKF/Joseph/更新前 NIS、身份/几何/生命周期/模型选择、有界 TrackerSet、只读外推/拦截/选板/落点代理/瞄准充分性已实现 |
| 任务 | 中心优先锁定、步兵默认辅助/显式自动/人工接管后不自恢复、哨兵程序请求；最终 shoot 仍只由 guard 产生 |
| 适配 | 文件事件/逻辑时钟/记录端、Linux 串口和 CAN、可选 Hikrobot；实机入口明确禁用 |
| 集成 | bootstrap、Pipeline 实例持有/协调启停、单写者估计、有界异步、模式切换/退出、统一/步兵/哨兵薄入口、同链确定性回放已连接 |
| 工具 | 合成数据、协议、标定/PnP 检查、检测耗时、同链可视化、TSV 时效/首次写入指标已实现 |

验证记录：

- 用户已放宽语言版本约束，允许 C++20；CMake 默认仍为 C++17，命令行可选择 20。独立 build-cxx20 使用实际 -std=c++20 完成 Debug 全量构建，82/82 项测试通过（含 OpenVINO 模型测试）；未为版本切换改写业务代码。下列原 Debug/Release/ASan 记录属于 C++17，不据此宣称 C++20 Release/ASan 已验证。
- 首版 Debug **82/82**、Release **82/82** 通过，含旧模型 CPU 同步推理、有界异步、整链异步三项可选测试。该次无外部模型或关闭 OpenVINO 时为 79 项。Release 检查不依赖 assert。
- 后续新增安装脚本（当前文件名 `install_dependence.sh`）：首次 Bash 语法检查、8 项无安装测试及预览通过；加入安装器测试后的 Debug CTest **83/83**。只生成安装脚本，不执行 APT/厂商安装、驱动加载或连接硬件。脚本支持 Ubuntu 22.04/24.04 amd64，当前无安装验证在 WSL Ubuntu 22.04 完成；不据此宣称新系统安装成功。
- 相机下载复核：旧 `MvCamCtrlSDK_STD_V4.7.0_251113.zip` 直链在本环境 HEAD/GET 均为 HTTP 403 HTML 安全拦截页，未取得真实包，不能断言其内部文件名或永久失效。已增加指定 HTTPS 下载、本地 ZIP 子目录/架构选包及仅检查模式，同步脚本改名后的测试、README 和 LF 属性。18 项无安装测试覆盖成功替身及 HTTP/HTML/损坏/歧义失败，修复后 WSL Ubuntu 22.04 Debug CTest 全量 **83/83** 通过；不代表真实下载、APT/厂商维护脚本或硬件验证。
- 按用户要求将 cmake/ 的模块函数、测试注册及 Python 边界检查器源码全部并入根 CMakeLists.txt，删除原目录及三个文件；检查器在开启测试时生成到构建目录。新增自测确认内嵌文本不会被误读为依赖声明，真实逆向依赖仍被拒绝。WSL Ubuntu 22.04 的 Debug/Release 重新配置、构建成功，CTest 均 **83/83**；九模块链接关系和业务代码不变。
- 按用户要求完成纯排版：整理 211 个非空 C++ 文件及 Bash、Python、根 CMake 文件，拆分长行、展开紧凑语句、return 独占一行并按逻辑留空行；新增根 .clang-format。C++ 超过 100 字符的行由 761 行降为 0；对照该次修改前快照，除空白与等价字符串拼接外词法内容一致，独立及内嵌 Python 的 AST、CMake 外层非空白内容一致，Bash 帮助与三组无安装预览输出逐字节一致。219 个 C++ 文件格式检查、Bash 语法检查、18 项安装器无安装测试通过；WSL Ubuntu 22.04 Debug/Release 全量构建及 CTest 均 **83/83**。未改变算法、接口、参数值或执行顺序，未安装依赖、连接硬件或提交 Git；本次未重跑 C++20 或 ASan/UBSan。
- 模型路径只通过 AUTOAIM_TEST_YOLOV5_MODEL 传入本机缓存，未硬编码或下载；Hikrobot SDK 开启的 build-sdk 静态 HAL 目标编译通过，未运行设备程序。
- 按用户指定的 sp_vision_25/io/command.hpp 对齐最终输出五字段接口，新增 CheckedCommand/CommandMetadata 分离主机元数据；同步 guard、publisher、编码器、离线记录及其调用方。新增 test_command 检查字段数量、类型、顺序、默认距离、无损转换和停止来源保留，扩充非零 AB 前馈与非法载荷检查；既有 CAN 黄金样例和串口格式保持通过。WSL Ubuntu 22.04 Debug/Release 构建成功，CTest 均 **84/84**（含三项外部模型测试）。仅修改接口及必要调用，未改变许可策略或线协议，未修改参考项目、连接硬件、安装依赖或提交 Git；本次未重跑 C++20 与 ASan/UBSan。
- 用户选定 14 字节 UART 后，将默认发送编码明确设为 `cboard_uart_v2`，离线 `protocol_tester --encode-stop` 只输出该格式；保留其他显式兼容协议及 43 字节 AB 接收解析。扩充非零载荷、CRC/帧尾、停止包和距离饱和固定样例，新增默认工具输出测试；同步 README 的字节表、使用说明、验证记录和每次修改同步文档的规则。WSL Ubuntu 22.04 Debug/Release 全量构建成功，CTest 均 **85/85**（含三项外部模型测试）；未修改线协议或权限联锁、连接硬件、安装依赖、修改旧项目或提交 Git，本次未重跑 C++20 与 ASan/UBSan。协议版本选择已确认，固件语义与设备实测仍未验证。
- 最终 Release 对 30 帧合成素材连续回放两次，TSV 逐字节相同：30 帧接纳、30 条决策/意图、60 条记录、各类丢帧为 0。96 个候选/有效位姿不等于 96 个正确目标，传统灯条存在额外跨板配对。
- 示例最终 control/shoot 均禁用，首次有效控制写入年龄为 NA；源观测逻辑年龄为 2 ms，含重发/停止记录的源年龄 P95 为 20 ms。它们只验证事件时序，不能作为处理性能。停止写入成功但 device_stop_confirmed=false。
- 30 张可视化标注 PNG 已生成并抽查。整链测试用仅 replay 域有效的模拟通道证据验证辅助控制请求，未把文件声明升级为实测。
- 最终 ASan/UBSan **79/79** 通过（Debug、AUTOAIM_OPENVINO=OFF、detect_leaks=1、halt_on_error=1）；包含最后的 pitch 联锁修订，未检测到测试覆盖路径中的地址/泄漏/未定义行为错误，不包含第三方 OpenVINO 运行时或硬件运行。

收尾审视已修正：时效边界浮点比较、PnP 同板先验时间顺序、ControlIntent 水平距离丢失、异步异常路径实际等待输入释放、关闭先停输出再等推理、模式切换先撤销再公布新世代、拒绝未来/异域反馈、无目标前即拒绝非法算法配置、发送联锁中 pitch 不应沿用 yaw 的整周环绕。补齐外部线程必须先停止/join 再关闭接口的注释。README 已从占位设计清单同步为实际实现与运行指南；未添加新任务范围。

外部资料与人工验收缺口：

- 缺真实录像/标签，传统候选误检、真实 PnP 精度和模型角点语义尚未验收；缺可靠证据时不授权开火。
- 缺设备内外参、曝光中点/姿态时钟/发送后延迟/弹速标定及倾斜轴多视角几何实测。
- 缺固件操作输入/明确启用事件、相对修正通道、手动优先/交权和独立超时清零的已核实映射；不伪造适配，不开放硬件入口。
- NUC 性能、按实际人工干预分层的对比、实机命中及设备停止回执均需人工验收。打符/MPC/CKF/阻力/在线几何慢修正不在首版实现。
- 未运行连接硬件的程序、未修改旧仓库、未提交 Git、未恢复 docs/。保留原有未编译的后续占位与用户已有变更。

<a id="history-offline-v1"></a>

### 11.2 完整离线优化执行包 v1（2026-09-29）

该阶段已执行顺序：S1 数据/会话 → S2 离线标定/报告 → S3 检测语义 → S4 YOLO11 →
S5 彩色相机配置 → S6 UART/按键 → S7 批量评测/耗时 → S8 集成回归。
各小项按接口、实现、测试、编译、记录依次完成。缺少实测资料未阻断独立软件开发；
不伪造设备参数或证据，实测验收保留未完成，硬件入口禁用。

阶段最终结果见下方验收汇总。展开项保存当时的推进与排错过程，其中“正在/待跑”是历史状态，不是当前待办。

<details>
<summary>2026-09-29 S1—S8 分项推进记录</summary>

S1 清单扩展已写入，保留旧格式，v2 区分原始 UART/按键与派生结果，
新增会话标识、逐帧标注及可选 SDK 诊断字段。WSL Debug 清单读取和旧整链回放
2/2 通过。接着实现同步离线 SessionWriter，原始与派生结果分离，目录禁止覆盖，
显式 finish 后才可回放；写出与故障测试通过，相关 Debug 3/3。
正在接入原 Pipeline 的可选会话记录，命令旁路元数据保留复检原因与停止标记，
不改变五字段 Command 或线协议。相关 Debug 回归 5/5，发布元数据专项 1/1；
30 帧合成数据写出/再回放的命令 TSV 逐字节一致，S1 软件验证完成。
原始 UART/按键解释仍保留到 S6。S2 标定数据读取测试通过；内参拟合/独立验证
测试通过（含参数恢复与噪声）。Park 手眼拟合与独立验证已实现，正在验证已知
变换恢复、重复姿态、单轴退化和反向位姿均通过，内参/手眼专项 Debug 2/2。
报告生成/导入测试通过，绑定参数、能力和独立验证残差；合成数据/报告不得升级
到真实设备域。CLI 已接入内参/手眼两种离线求解，旧 PnP 检查保留。
标定相关 Debug 7/7、两步 CLI 联合导入和圆点阵补充 2/2 通过；S2 软件验证完成。
S3 检测语义拆分与规范化标签/关联 Debug 4/4 通过。新增显式板型尺寸表，保留
旧 class_id 配置兼容并拒绝混用；传统合成示例已迁移，PnP/整链 5/5 和补充 2/2
通过，S3 软件验证完成。S4 纯解析 Debug 2/2 通过；已接入共享 OpenVINO 请求槽
和配置工厂，固定输入/输出形状分别为 1×3×640×640、1×50×8400。两种外部模型的
同步/异步/整链 Debug 6/6 通过；YOLO 模板独立提供显式路径，不复制权重，不声称
YOLO11 精度更高。S4 软件接入完成，真实标注评测未验收。
S5 相机请求/回读契约已写入，ROI 与标定几何不一致直接拒绝，不自动调整内参。
手动/连续白平衡、原始像素格式及实际曝光/帧率分开表达，参数专项 Debug 1/1 通过。
SDK 设置/回读已实现，模板不填未知设备值；只做 SDK 编译检查，不连接相机。
SDK HAL 编译通过。帧诊断元数据与 ROI 标定/报告绑定已接入，已知 ROI 不匹配
拒绝进入 Pipeline；旧元数据未知不补造成设备测量，设备 tick 仍仅诊断。
相机/会话/标定/整链 Debug 5/5 通过，补充 ROI 误配和元数据往返专项回归。
补充 3/3 及带元数据 SDK HAL 编译通过，S5 软件项完成，真实相机参数仍未验收。
S6 已接入显式坐标/时间映射的 AB43 UART 接收适配，世代变化丢弃半包，CRC 不绕过，
反馈绝不产生操作手输入。适配测试待跑，下一小项为发布端 UART14 编码适配。
新增目标已重新配置；修正 Config 头文件遗漏及测试构造表达式歧义后重测。
接收适配 Debug 1/1 通过。UART14 发布适配及 CLI 离线十六进制输出已写入，
不改变五字段/开火许可，短写须锁存失败，专项测试待跑。
修正短写替身不能继承 final 测试类的编译问题，改为直接实现 HAL Transport。
UART14 发布短写测试 Debug 1/1 通过。按键电平转换已实现 toggle/hold，先单独验证
启动按住、重复电平和失效重启用规则，再接入 InfantryMission。
按键转换 Debug 1/1 通过。InfantryMission 接入两种模式，旧 enable_event 语义保留；
人工接管/故障/输入失效清除按下资格，恢复后继续按住不能重新启用，专项待跑。
模式/旧事件 Debug 2/2 通过。Pipeline 按显式输入来源连接 UART/按键，按键不刷新
姿态，guard 单独检查操作输入年龄；配置默认 toggle、注释 hold，整链专项待跑。
整链 Debug 4/4 通过，补充原始会话同时间 UART/按键回放及 guard 独立输入时效回归。
补充专项 Debug 3/3 通过，S6 软件项完成。S7 标注契约/读取已实现，缺失与空标注
分开；可选位姿真值必须有来源和不确定度，读取不等于可信或设备证据，专项待跑。
标注 Debug 1/1 通过。单帧几何/语义匹配与角点/有条件位姿误差已实现，专项待跑。
单帧专项 Debug 1/1 通过。序列累计/连续漏检/人工介入分层已实现，未知标注
不冒充无目标，原始检测指标与管线可用匹配分开，序列专项待跑。
序列 Debug 1/1 通过。阶段耗时埋点已写入，仅汇总主机墙钟，不参与许可/逻辑时钟；
复制、等待、预处理、推理墙钟、后处理各自计样本/总量/最大值，专项待跑。
耗时/队列/两种 YOLO 异步 Debug 4/4 通过。批量 CLI 复用 Pipeline 的只读评测旁路，
逻辑报告与墙钟报告分开；位姿真值接纳需显式参考 ID/门限，批量专项待跑。
批量工具及评测旁路已编译，新增同输入双运行逻辑一致性、缺姿态二维评测与输出
禁止覆盖的端到端测试；不使用真实精度或设备性能作为测试结论。
批量专项发现零字节评测输出被发布器正确拒绝，已改为复用 UART14 记录适配后重测，
不修改发布器完整写入规则。
批量/原整链 Debug 2/2 通过。补充该次执行的配置/实际标定/模型内容标识与阶段耗时
记录；原始会话身份不被派生执行身份覆盖，FNV 内容标识不当作密码学认证。
元数据/会话 Debug 2/2 通过。合成工具开始输出带标注的完整 v2 会话，类别 unknown，
位姿参考只属于 synthetic-projection；用于评测软件回归，不用于模型精度验收。
新增可选三检测器批量 CPU 回归，外部模型通过 CMake 测试参数注入，不写入仓库。
三路径批量/普通批量 Debug 2/2 通过，S1–S7 软件能力已落地。S8 审视发现并修正
按键采样跨越失效区间但没有中间回调时的边界，须清除旧电平资格后重新释放/按下。
该边界与旧人工接管专项 2/2 通过；新增代码按 100 字符长行约定收尾整理，随后全量回归。
S8 增补会话浮点完整精度、相机/UART 配置读取回归；相机序列号 YAML null 明确拒绝。
README 已整合为 S1–S7 使用指南，新增离线标定数据格式模板，所有实际尺寸/设备参数
保留待填项；历史测试数与本轮验证矩阵分开。未新增依赖，安装脚本无需更改。

</details>

#### S1—S8 阶段最终验收汇总（2026-09-29）

S1–S8 软件离线闭环完成，保留既有工作区变更。收尾修复按键跨超时间隔、浮点记录
精度、相机 null 序列号检查，新增相关回归。根 CMake 后端说明同步 YOLOv5/YOLO11；
配置、使用命令、模块职责、源码注释与 README 已同步，没有重建 docs/ 或 cmake/。

| 分类 | 结果 |
| --- | --- |
| 已实现 | v2 原始/派生会话、内参/Park 手眼/独立验证/报告导入、统一检测标签与板型、YOLO11 共享有界推理、彩色相机请求/回读/元数据、AB43/UART14 离线适配、toggle/hold、同链批量评测与阶段耗时 |
| 已验证 | WSL C++17 Debug **106/106**、Release **106/106**；C++20 Debug **106/106**；关闭 OpenVINO 的 ASan/UBSan **99/99**（detect_leaks=1、halt_on_error=1）；可用海康 SDK 的 autoaim_hal 静态编译通过 |
| 缺外部资料未验收 | 真实视频/标签上的检测与角点精度、模型比较；实际板尺寸与设备内外参、时间标定；固件姿态/坐标/相对通道/权限映射 |
| 仍需人工实机验收 | 相机节点/ROI/曝光/增益/白平衡和时钟映射、NUC 延迟/FPS/150fps 目标、人工接管/看门狗/设备停止回执、发弹延迟/弹速及实机效果 |

106 项包含两种外部权重的同步/异步/整链 CPU 测试共六项及三检测路径批量比较一项；
99 项不包含 OpenVINO 运行时测试。既有三种高度布局/倾斜轴、大角度与多解 PnP、
乱序/旧世代及同条指令许可不可恢复等回归均包含在全量结果中。

`out/s8-validation-1790690925505/` 保存 S1–S8 阶段的 30 帧带标注合成产物：两次回放 TSV、
UART14、原始事件以及排除 timing 的 62 条派生记录一致，录制会话再回放命令一致；
双运行批量逻辑报告一致，模拟位姿参考评测已执行。30 帧接纳、无丢帧，最终
control/shoot 均禁用，首次有效控制写入为 NA。该合成场景的 60 个标注、96 个候选
只匹配 30 个，报告保留 66 个额外候选/30 个漏检；不把传统合成精度或墙钟值当作
真实模型优劣、NUC 性能或设备验收。

10 份配置 YAML 与 README 本地链接检查通过，长行/return 静态扫描及差异空白检查
通过。没有安装/下载依赖、连接设备、修改参考项目或提交 Git；不宣称硬件已上线。

### 11.3 原阶段验收依据（保留验收目的，不作为新执行顺序）

三个阶段服务于 §1.5 的首版装甲板闭环，首版验收同时包含步兵辅助、步兵自动和哨兵自瞄的规定行为；打符的具体实施与验收安排留待后续，不由“三阶段”这一顺序自动承诺首版交付。

| 阶段 / 子步骤 | 内容 | 结束判据 |
| --- | --- | --- |
| **1a 工程骨架** | 建立 CMake；按 core、math、hal、vision、estimation、decision、control、pipeline、mission 九个模块声明 target 边界；允许按实际实现选择普通库或接口库 | 基础构建通过，依赖白名单与包含边界检查可执行 |
| **1b 契约与运行流程** | core 类型/时间/单位/配置/证据；三个主要契约、帧同步、启动组装、许可检查与发布器；控制模式与人工接管 | 模式世代、人工接管、明确重新启用、时效、异常与发布失败等基础契约可验证；不要求此时完成全部算法 |
| **2 回放与契约测试** | 复用在线处理链，替换输入、时钟与输出；需要的检测/估计由最小可运行基线、已录制中间结果或明确标记的测试替身供给 | 端到端回放与故障契约通过；记录观测年龄、首次成功写入延迟、丢帧与过期；使用替身不代表算法精度达标 |
| **3 模型与后端扩展** | 在基线上扩展几何/测量/运动模型、推理后端、角点精修与数值对照，逐项替换测试替身 | 三种布局、大角度 PnP、运动模型及目标性能的合成/回放验收；算法结论必须来自真实待验实现 |
| **独立人工实机验收** | 人工开展时间/外参/几何标定、NUC 测量、固件看门狗及设备联调 | 明确设备、配置、素材、方法、结果与残余未验证项；不得由代理运行连硬件程序 |

**依赖检查边界**：CMake target 的链接依赖和可见包含目录有助于约束接口，但不能自动发现所有越层包含、头文件或业务依赖。应配合明确的依赖白名单和包含检查；不得承诺“越层必然链接失败”，也不得将头文件库视为不受约束。

**顺序依据**：

- 时间和外参未对齐时，复杂动力学可能解释错误时间造成的假运动；
- 角点不稳时，复杂几何优化可能拟合噪声；
- 未证明阻力误差占主导前，不急于增加复杂空气阻力模型；
- 回放链的契约验收和完整算法的效果验收分别记录，不能互相冒充。

阶段 1–3 的代理工作仅允许无硬件构建、测试与回放。后续实机验收由人执行；“有人在场”不自动授权代理运行连接相机、串口、CAN 或云台的程序。

---

<a id="history-slimming"></a>

### 11.4 分批瘦身实施与验收（2026-09-29）

B0–B10 已执行：一次一个关注点，验证后继续；未暂存、提交或重置 Git。
保持公开接口、CLI、报告、控制及异常语义；全部 rune 预留文件保留。
有效时效/世代/权限检查不删，不合并 CommandSlot 与 Publisher，不替换重依赖。
每批同步 README；回滚基于修改前工作区快照并检查后续编辑，不基于旧 HEAD。
证据目录为 `out/slimming-20260929-224127/`；最终保留 Debug/Release，
辅助配置验收后仅清理经核对的生成产物，不删除缓存参数、日志和 out 证据。

| 批次 | 实施记录 |
| --- | --- |
| B0 | 原始源码快照 285 文件；当时 Debug/Release 各 106/106，离线基线独立保存 |
| B1 | 移除未使用的 OpenCV videoio 配置要求；其他组件不变 |
| B2 | HAL 只声明 OpenCV core/imgcodecs 依赖，保持 SDK 和九模块边界 |
| B3 | benchmark 复用 describe_run 元数据，每套配置减少一次 YAML 文件解析，保留独有字段 |
| B3b | 九模块显式 STATIC，移除未用空模块分支；保留 autoaim_options INTERFACE 和依赖白名单 |
| B4 | 标定 CLI 与 benchmark 分离编译单元，复用现有标定实现文件；入口和求解逻辑不变 |
| B5a | 回放清单只解析一次，元数据来自已校验 Config 节点；保留旧格式和错误检查 |
| B5b | 标注使用独立惰性索引，首次匹配/副本隔离/非法帧号异常顺序不变；不推进回放时钟 |
| B6 | benchmark 不累积未消费的命令文本；仍使用真实 UART14 writer 和 RecordingTransport 完整写入计数 |
| B7 | bootstrap 复用无分配队列/池预算校验；实际构造仍校验，避免合成配置 5,529,600 字节临时池 |
| B8 | 固定输入的一次 benchmark 内共享预读事实；标注仍在首次逐帧读取时校验，不提前/延后首个错误；后续运行仍走原 Pipeline |
| B9 | 仅清理已证明无用的实现侧重头并补直接标准库依赖；不因引用扇出大而删除公共包含 |
| 最终回归 | Debug/Release/C++20 各 106/106，ASan/UBSan（OpenVINO 关闭）99/99；海康 SDK 编译及 79 个公共头自包含检查通过 |
| B10 | 仅清理已验收的 C++20/sanitizer/SDK 生成产物，减少逻辑大小 6,941,600,337 字节；缓存/测试日志哈希保持，Debug/Release 保留 |

最终固定 120 帧双配置比较的 YAML 文件解析 8→2、图像解码 480→360；回放命令、
逻辑事件及非耗时报告与基线一致，重复运行也一致。Release 标定程序
1,735,104→1,095,640 字节，无 benchmark/OpenVINO 检测器符号及 OpenVINO 直接动态依赖。
默认配置的 OpenVINO 查找要求不变，benchmark 仍保留该后端。

本地五次墙钟中位数 3.452→2.485 秒（范围 3.085–3.532→2.474–2.563 秒），
不能作为 NUC/设备验收。编译单元仍为 169；惰性索引和共享预读使用 O(N) 元数据，
不缓存像素，不将减少重复工作解释为所有场景峰值内存或代码行数下降。

B0–B10 已完成，逐批 diff、快照、命令、测试与指标见
`out/slimming-20260929-224127/REPORT.md`。辅助配置已清理，使用保留的 CMakeCache
重新运行 `cmake --build build-cxx20 --parallel 2`、
`cmake --build build-sanitize --parallel 2`、
`cmake --build build-sdk --target autoaim_hal --parallel 2` 恢复；不声称其二进制仍在。
源码按批次逆序、核对修改后哈希再恢复 before 快照；并发编辑不覆盖。
瘦身阶段未安装依赖、未连接设备、未改参考项目，真实精度和设备性能仍待人工验收。

<a id="history-measurement"></a>
### 11.5 测量闭环实施与验收（2026-09-30）

用户已解除仅规划限制并授权 WSL 离线验证；不接硬件、不安装依赖、不提交 Git。
修改前快照及本轮日志位于 `out/measurement-20260930-implementation/`。
Debug/Release、C++17、OpenVINO OFF 基线各 99/99；读取、指纹及导出专项 3/3 通过。
逐条/整帧审核与回看专项 3/3 通过；整体替换、完整回放自检、无覆盖发布和
标注来源绑定专项 4/4 通过；显式 IoU、审核绑定验证与构建/数据来源报告专项 5/5 通过。
固定六帧合成 golden 和可手算统计断言专项 6/6 通过。
端到端检查确认本机 `/mnt/e` 不支持 RENAME_NOREPLACE 目录发布，工具拒绝而不覆盖；
验证输出改用 WSL 原生 `/tmp`，不放宽原子发布契约。
合成回看发现窄小目标角点文字重叠，已调整为向外排布文字及较小标记，不改变坐标或评测；
最终显示效果已回看，调整后的 Debug / Release / ASan-UBSan 全量各 102/102 通过。
内存检查启用 `ASAN_OPTIONS=detect_leaks=1` 与 `UBSAN_OPTIONS=halt_on_error=1`，无报错。
新增标注单测、工具自检和 golden 回归三项，原有模块边界与负例检查仍通过。
合成端到端源目录全部文件 SHA-256 保持不变；相同完整参数和输出路径复跑的
`report.yaml` 全文一致，没有排除字段。修改输出 PNG 不影响源图由单测验证。
源码/配置快照比对确认：检测、估计、控制、HAL、指标算法及三个打符预留文件未改动。
原有项目版本 1.1.0 与文档重组变更已保留；未提交、暂存或重置 Git。

本阶段口径：标注只供离线评测，审核声明不产生设备能力。二维指标不以姿态存在为前提，
但三维/整链可用性仍按原规则判定；真实素材没有合格独立位姿参考时不产出位姿误差。
无显式位姿评测参数时报告 `pose_metrics: not_produced`，既有空误差和 pose_pairs 计数保留。
使用说明统一见 [README §6.4](README.md#64-人工标注与可信评测)。
本阶段软件离线闭环已验收；真实人工标注质量、识别/位姿精度、命中率和 NUC 性能未验收。
本阶段不包含 OpenVINO ON、C++20 或 SDK 的重新验证，不把 §11.4 历史结果升级为本次结果。
准确命令、日志、快照和回滚说明见
[测量闭环验证记录](out/measurement-20260930-implementation/VALIDATION.md)。

<a id="history-repair"></a>

### 11.6 分批正确性修复（2026-09-30；软件修改完成）

用户已授权按 B0—B11 逐项执行。保留原工作区和三个打符文件，不提交 Git、
不安装依赖、不连接设备；C 类身份、同板先验、相机模式和锁外复制仍单独审批。
B0：工作区完整源码/配置/文档快照已保存，独立 C++17 Debug / Release 基线各
102/102；固定六帧回放、UART 输出及完整报告重复一致检查通过。
B1：新增 Tracker 改选强侧视候选的回归用例，原实现实际失败于可靠性断言。
实现增加 `Observation::candidate_reliable`，状态、运动选择和快照均查询最终候选；
保留兼容字段 `reliable`，明确其仅描述 PnP 原选解。失败用例修复后通过，
候选排列同步调整测试通过，Debug 全量 102/102；Release 和扩展矩阵待最终回归。
后续修复尚未验收，不能沿用此基线声明完成。
B2：三个新增用例已在原实现实际失败（改名复用图片、重复点集、旧报告缺质量块）。
内容去重、约化内参信息检查及报告导入已完成，标定专项 5/5 通过，包含已知参数恢复、
噪声、重复样本、正视/近正视退化、旧报告降级与质量字段篡改；不修改手眼方法或设备参数。
B3：四点重合标注在原读取器被接受的用例已复现。读取和直接评测共用
`validate_annotation_corners`，拒绝非有限、重复点、零面积真值；不改变 IoU 定义、
角点顺序或合法图外不可见点。标注、评测、标注工具及 golden 4/4 通过。
B4：原实现接受 `--pose-position-limit-m 0.5junk` 的失败用例已复现。
两个位姿门限现要求完整有限非负数，不套用 IoU 的 (0,1] 范围；批量评测单测通过，
含两种参数的尾随垃圾、空值、非数值、NaN/Inf、负数、溢出拒绝及零门限保留。
B5：整链两帧漏检导致决策未继续的失败用例已复现，修复后专项 6/6 通过。
选板区分决策时间和源观测，
保留公开 select 接口；Pipeline 无新观测时不请求开火，原始源时效不刷新。
B6：来源元数据版本 2 增加内外参引用报告内容标识及已装载证据/资格（replay 域）。
总报告版本和指标定义不变；引用文件单独改变也可定位，未引用为 null，旧报告无质量
信息仍为 missing。专项 3/3 通过，不把文件存在或模拟报告描述为实机验收通过。
B7：标注导出按采样时间稳定排序并建立本次调用的私有索引；不假定接收顺序即采样
顺序，不插值、不解释 UART。相邻反馈取同时间首条，精确姿态取首条显式有效反馈。
复杂度由 O(帧数×事件数) 改为 O(事件数+反馈数 log 反馈数+帧数 log 反馈数)，专项 2/2。
256 帧/256 反馈，预热后五次导出中位数 2.083→0.664 s；范围为 1.910—2.155 s
和 0.633—0.693 s。完整 index.yaml 的 SHA-256 相同，不代表设备端吞吐量。
B8：补独立物理点及针孔算式的 PnP 参照，覆盖大小板、滚转、镜像输入显式映射、
尺度/距离同倍歧义及传统中心线端点，不调用生产物体点函数生成这些真值。
**未验收**：专项 3/4；test_pnp 在第一个新增小板/零滚转案例失败，其后新增断言
尚未执行通过，不能声称已覆盖完成。只链接 OpenCV 4.5.4 的独立复现也失败：
精确半周旋转的 IPPE 候选 RMS 为 0.460504、0.536219 px，同输入 ITERATIVE 约
7.58e-7 px。增加 0.15 rad 非奇异俯仰的对照输入，IPPE 最佳 RMS 约 1.96e-6 px。
初次验收停在 B8；以上为当时失败记录，不覆盖原日志。

B8N（2026-09-30）：按续修 v2 授权采用等价坐标重参数化。以 TL→TR 像素边
方向定义正交旋转 S，对物体点求解后恢复 R=R′S；平移不变，深度、法向和残差
均在原板系计算。OpenCV 4.5.4 的 IPPE 内部 rot2vec 含 sin(θ) 分母且没有半周
专门分支；只针对此已复现路径，不宣称其他版本均有问题。保留单次 IPPE 双候选，
不使用 LM 或回退，不把数值重参数化当成物理角点证明。原 B8 专项从 3/4 到 4/4。
B8V：扩展专项 4/4，360 组独立五参数投影覆盖正视/倾斜、两种尺寸、五种滚转、
半周 ±1e-6 rad、非零畸变及确定性像素噪声。无噪声最大 RMS 0.000174012 px、
平移 1.60531e-6 m、旋转 1.16535e-5 rad；每组仍有两个不同候选。
保留原错序/未知语义/尺度歧义/可靠性负例；原有门限未放宽。证据另存 B8V。
B9：数值契约明确 Detection.confidence 的路径差异与像素噪声启发式；PnP 是未列
归一化的 m/rad 信息矩阵，标定质量是消去位姿后列归一化的矩阵，两者门限不通用。
量测雅可比为 H=-∂residual/∂state；新增非零残差双步长差分、噪声缩放、半正定及
信息比边界检查。固定线性化公共 6D 噪声示例只证明整块膨胀的保守性质，不证明
逐次重线性化 EKF 的普遍一致性。精修开关、成功、失败保留原角点与不等长灯条
分别回归，不改变默认参数；本批专项 5/5。首轮 4/5 的失败是新增夹具首帧与启动
同时间被既有世代门禁拒绝，推进夹具时钟后通过，未更改生产时序规则。
B10：新增发布接纳边界前后的确定性屏障，分别验证零旧写与一次在途写后停止，
回调始终由同一发布线程执行。移动目标在宽松时间门限下仍受严格位置残差约束；
lost 阈值与模型时域之间的空档重新创建目标，不继承原板身份；CV/CA 映射检查
alpha 行列与交叉项、半正定性。默认装配任一 declared 证据均不能自动启用；保留
模拟证据 toggle/hold 正例与非法真值报错测试。专项 10/10，回归确认无需修改生产行为。
B11：tools/ 纳入包含边界，protocol_tester 仅有 control 的传递能力，pipeline_metrics
不获九模块权限；CMake 白名单与检查器逐项比对，并有工具越层、链接漂移和白名单
漂移负例。独立验收清单目标及自测已加入，专项 3/3。新增可选
AUTOAIM_THREAD_SANITIZER，与 ASan/UBSan 互斥；同时启用时配置被拒绝的负例通过。
旧 build-debug 为 OpenVINO ON 的早期 C++17 Debug，测量闭环 debug 为 OpenVINO OFF，
repair debug 为续修阶段 OpenVINO OFF；均海康 OFF，不能以目录名代替源码与选项标识。
多配置生成器和线程检查若缺少运行环境，必须保留具体限制，不以普通 CTest 替代。

最终矩阵（所有配置生产源码和测试标识一致）：C++17 Debug/Release 各 103/103；
C++20 Debug 103/103；OpenVINO OFF 的 ASan/UBSan 103/103，detect_leaks=1、
halt_on_error=1；OpenVINO ON 配现有两个模型 110/110（含七个条件测试）；海康
SDK 仅 autoaim_hal 编译通过。WSL/GCC/OpenCV 等版本沿用本节环境，未安装依赖。
TSan 五个离线并发目标编译通过，但宿主运行时报告 unexpected memory mapping；
首轮一项段错误的单次诊断也报同一错误，线程检查未验收。无 Ninja，多配置未验证。
Debug 最后增量确认没有待重编译项或时钟偏差警告。

同构建、同参数和路径重复：report.yaml 全文、命令、UART、原始事件一致；derived
仅墙钟阶段毫秒数不同，逻辑事件及样本数一致。跨 B0 报告只变化来源/构建字段和
输出路径，固定六帧指标不变；IPPE 数值修复另由 B8/B8V 的独立几何对照证明。
未自动更新 golden，三个打符文件保留。逐批差异、快照、真实命令、日志、指标及
逆序回滚步骤见 [续修交付清单](out/repair-20260930-v1/Final/DELIVERY.md)。
真实模型端点语义、设备标定/精度/吞吐量仍缺实测资料；物理板身份重设计、先验接入、
相机模式调整仍待单独审批，不因本次离线通过而开放。
命令、日志、批次快照、diff 和回滚依据见
[修复执行记录](out/repair-20260930-v1/STATUS.md)。

<a id="algorithm-alternatives"></a>

## 算法替换详细手册（2026-09-30）

[快速替换与命令](#quick-algorithm-swap) 位于本文最前面；方案依据与审核修正见
[possible_method.md](possible_method.md)。本章覆盖 I0–I10 和 A–C，区分完整的局部候选与
尚需接口/数据的完整提案。算法公共接口、运行配置与协议保持不变；根 CMake 统一控制编译期选择，六项默认均为 OFF。

### 统一切换及回退规则

1. 使用全新的构建目录，明确传入六个开关的 OFF 值，再仅将本次候选设为 ON；
   根 CMake 选择同接口实现并注册该候选专项，源文件保持不变。
2. 统一验证脚本复制当前工作区（含未提交源码），不改写副本；保存源码摘要、实际配置、
   构建/测试命令及结果。`DEFAULT` 关闭全部候选，`ESO` 使用统一选择规则。
3. 回退时在同一构建目录显式将对应开关设为 OFF，重新编译和执行 CTest；
   CMake 缓存保留旧值，仅省略参数或修改 `option()` 默认值不会覆盖已有缓存。
   无需移动函数或更改注释、别名。
4. 差分测试使用 `tests/support` 中固定保存的原始实现，只链接测试目标；
   不由脚本提取或改写生产代码，也不向生产模块增加第二套公共入口。
5. 构建报告记录六个开关。被 CMake 拒绝的组合不能启用；其它组合仍需另行验证。

<a id="alternative-i1-contrast-irls"></a>

### I1-CONTRAST-IRLS：中心轴对比度加权精修

替换 [corner_refine.cpp](src/vision/corner_refine.cpp) 的完整 `refine_corners`；仍接收
`const core::Image&`、`const Detection&`、`const RefinementOptions&` 并返回 `RefinementResult`。
`AUTOAIM_I1_CONTRAST_IRLS=ON` 选择完整候选，OFF 选择原实现。

筛选像素、ROI 和初始 Huber 轴沿用原实现，基础权重为 `max(1, 通道差)/255`；
使用 Huber 截断 `1.345 px` 的加权 TLS，最多 10 轮，轴角变化小于 `1e-6 rad`
且中心变化小于 `1e-4 px` 时结束。迭代上限只表示使用最后有效结果，不声明已收敛。
全部对比度权重相等时，检查退化后复用原 Huber 与原浮点端点计算，保持等权基线数值；
非等权才执行新增 IRLS。像素筛选的浮点边界也与原实现一致。
非有限权重/统计、无空间分散或主轴不可确定均触发原整体失败返回。

端点仍是全部原筛选点的轴投影 min/max，不重排物理索引、不强制两条图像灯条平行等长；
最终沿用方向、长度和 `maximum_shift_px` 限制。一侧失败保留全部原角点并将可靠性置为 false，
成功也不把原来不可靠的角点升级。其它 Detection 字段和输入像素不变。
此项只比较局部中心线拟合，不能证明模型标签、板外缘或实物尺寸已经匹配。

启用/回退只改变 CMake 开关。专项使用固定保存的原函数
`refine_corners_baseline`，比较暗色平行污染下的真实轴偏差，并覆盖蓝灯、透视、索引翻转、
stride 与退化输入；软件及实物验证状态分开记录。

<a id="alternative-i3-linear-ca"></a>

### I3-LINEAR-CA：默认 EKF 的平移恒加速度对照

替换 [bootstrap.cpp](src/pipeline/bootstrap.cpp) 中 `cv_model` 与 `ca_model` 两个构造声明，
由 `AUTOAIM_I3_LINEAR_CA` 同时选择两个构造；`PipelineConfig` 与估计器接口不变。
仍使用 EKF，两个现有角运动模型分别追加 `with_linear_acceleration(1.0)`，保留角噪声、
角加速度上限和时间范围。`1.0 m²/s⁵` 是候选内部实验白 jerk PSD，不是经过设备标定的参数。

原 `motion.linear_accel_psd` 的单位是 `m²/s³`，仍读取和校验，不被偷换为 jerk；
本候选平移 CA 实际使用上述独立常数。12 维状态、加速度初始方差及完整白 jerk 离散传播
均复用现有代码。Tracker 发布实际滤波模型，`predict_future` 使用快照中的同一模型，
不单独改写快照或使用另一条外推公式。此项没有 IMM 权重、相位多峰或混合快照。

一个 CMake 开关成组选择两个声明。专项检查真实装配、恒加速度估计、
角 CV/CA 切换对平移加速度及协方差的保留、快照预测一致性与拒绝/时间边界原子性。
CMake 拒绝与 ESO 同时启用；后者会使用自身的平移 CA 和 jerk 参数覆盖 I3 选择，
不能把这种组合视为有效的 EKF 平移 CA 对照。

<a id="alternative-i2-lm"></a>

### I2-LM：IPPE 双初值的角点重投影精修

替换点为 [pnp.cpp](src/vision/pnp.cpp) 中完整函数，原接口保持：

```cpp
std::vector<PoseCandidate> ippe_candidates(const Detection& detection,
                                         const PlateDimensions& dimensions,
                                         const Calibration& calibration);
```

`AUTOAIM_I2_LM=ON` 选择完整 LM 候选，OFF 恢复原 IPPE。候选保留等价板系、IPPE 双初值，
分别使用 `solvePnPRefineLM`（最多 20 次、终止精度 `1e-6`），再重算重投影误差、正深度与朝向。
优化无效、负深度、四角 RMS 或最大单角重投影误差任一增大时保留相应原候选；
不同初值优化合并时保留原候选组。代码中的合并容差仅用于数值判重，不是实物质量门限。
候选排序后仍由 `solve_pose` 的现有门限、协方差、同板先验和证据条件决定有效性与可靠性。

这是 **I2 的角点重投影 LM 子方案**，没有逐边梯度权重、历史噪声标定或生产同板先验回传。
不会因为优化残差变小就升级角点/设备证据。接口与算法依据见
[OpenCV PnP 文档](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。
CMake 在启用 LM 且构建测试时注册专项；原函数固定保存在 `tests/support` 中作为测试基线，
生产代码不增加第二套入口。

<a id="alternative-i9-throughput"></a>

### I9-THROUGHPUT：OpenVINO 性能提示对照

替换点为 [detector_openvino.cpp](src/vision/detector_openvino.cpp) 中原 `compiled = core.compile_model(...)`
语句，由 `AUTOAIM_I9_THROUGHPUT` 选择。启用时必须同时开启 `AUTOAIM_OPENVINO`，否则配置失败。
唯一算法选项差别是 `ov::hint::PerformanceMode::LATENCY` → `THROUGHPUT`；
`options.device`、请求池容量、缓冲所有权及完成结果接纳契约不变。

这只是 I9 的运行时提示实验，不包含无锁队列、线程亲和、量化或自动异构调度实现。
已有 `detector.device` 可选择后端，CPU 是默认值。吞吐提示不保证源数据年龄或 P95 降低，
需要目标 NUC 同机、同素材、同模型对照后再选择。

<a id="alternative-i9-prealloc"></a>

### I9-PREALLOC：保留队列行为的 pending 容器预分配

`AUTOAIM_I9_PREALLOC` 成组选择 [queue.cpp](src/pipeline/queue.cpp) 中的 `FrameQueue::State`、
`Activity` 和队列方法：ON 使用 vector 候选，OFF 使用原 deque 实现。
公共 `FrameQueue`/`FrameTask`/`QueueOptions` 不变。

pending 使用 vector，构造时预留 `min(pending_capacity, pool_capacity)`，因为 pending
不可能超过图像池容量。取任务仍从尾部领取并保留旧积压；容量满时从头部淘汰，purge
按原顺序稳定压缩并逐个累计同样的丢弃原因。mutex、在途限制、额外读者租约、跨线程释放、
reset/close 和 `ResultAdmission` 语义不变。头部移除/清理允许 O(capacity) 移动。

它只减少 pending 容器分配，CapturedFrame、Activity 等仍有分配；不是无锁或全链零分配。
`tests/support/queue_prealloc_baseline.hpp/.cpp` 固定保存原实现，仅向专项测试提供
`BaselineFrameQueue`/`BaselineFrameTask` 对照，比较出队、
丢弃、在途、池占用和像素寿命；墙钟耗时数值不要求逐字相同。另运行 YOLOv5/YOLO11
同步、异步、整链与缓冲寿命回归。性能是否改善需目标 NUC 同机比较。

启用或回退只修改 CMake 开关；条件编译始终只保留一个内部 State 和一套方法定义。
本轮单独使用默认 LATENCY，不与 I9-THROUGHPUT 叠加作为验收结论。

### 其它方案的替换条件

以下剩余部分均为“待接口/资产”，没有可供 CMake 选择的完整算法，也不列出虚假的启用命令。
后续条件具备时须重新生成完整候选，按统一规则验证和回退；不得借已有局部候选标记为完成。

| 方案 | 当前接口与缺口 | 后续接入步骤 | 验证入口/场景 |
| --- | --- | --- | --- |
| I0 证据通路 | `Evidence` 与标定报告装载已存在；角点、几何、控制装配缺来源绑定 | 复用 `intrinsic_report_file` / `extrinsic_report_file`；replay 测试用 `MeasurementReport::evaluate` → `Evidence::from_report`；真实装配另设计报告绑定，不加布尔升级开关 | `test_evidence`、`test_bootstrap`、`test_calibration_report`；错设备/配置与 host 域拒绝 simulation |
| I1 物理精修 | `refine_corners` 有图像，但无物理尺寸/投影模型；IRLS 仍为中心线端点 | 核验模型端点标签与实物尺寸，再设计一致的投影和端点输入 | `test_corner_refine`、`test_pnp`；真实标签及独立 pose reference |
| I2 梯度/先验/噪声 | PnP 无原图/边权重；同板先验 API 已有但当前检测缺可信板身份；协方差计算无历史 | 提供边样本；建立当前检测到物理板的可信关联及曝光坐标转换；定义按设备/标定/模型隔离的统计所有者与重置 | `test_pnp`、`test_pose_quality`；错目标/板/世代/时间先验拒绝、独立残差集 |
| I3 IMM/多峰 | 当前快照只有选中状态/模型；I3-LINEAR-CA 和 ESO 均不输出混合权重 | 先定义模型集、圆周分量、权重及快照，再同步未来混合传播 | `test_tracker`、`test_motion_selection`；阶跃/变转速、多峰区分及预测一致性 |
| I4 四维/自适应 | position 是 3 维；4 维需真实残差/H/R及类型；关联筛选和同曝光多次更新不能直接当无偏历史 | 显式扩量测语义；定义可信样本、曝光边界与历史重置；预测包络另收校准集 | 按实际维度做 NIS、有真值做 NEES；同时统计误关联/拒绝/失锁，禁止当前残差放宽当前门控 |
| I5 命中概率 | `ImpactMargin`/`MarginOptions` 表达 kσ 边距，缺概率/η；现有噪声传播已存在 | 扩概率与阈值契约，明确分布和二维相关积分，再用打靶数据校准 | 高相关/奇异协方差/尾概率与蒙特卡洛对照；独立打靶校准 |
| I6 时机搜索 | `fire_at` 是预测值，发布链无未来执行、取消或到期复检 | 先设计决策到执行的时间契约及有限窗口，再实现择时 | 延迟/丢帧、等待期间目标失效与许可撤销；检查真实执行时刻 |
| I7 伺服辨识 | `AimAdequacy` 仅检查实测反馈；缺命令/发送结果/反馈历史和辨识状态 | 收集同时间/坐标序列，辨识有界响应和延迟不确定度，再评估前馈 | 合成响应与设备阶跃/正弦实验；规划参考不能替代实测反馈 |
| I8 采集链 | 静态 ROI/曝光/格式已配置；ROI 须与标定几何一致，binning/decimation 当前为 1；tick 未映射 host 时钟 | 核验型号/带宽/照明/ROI 标定，建立硬件时间映射后再研究动态 ROI/触发 | `test_camera_disconnected` 为软件检查；相机实测曝光中点、抖动、吞吐、图质和 P95，不保证 200–249 fps |
| I9 设备/无锁/INT8 | `options.device` 可配置；无锁涉及共享池/租约；INT8 缺资产 | 设备沿用现有配置；亲和先取 NUC 拓扑和允许 CPU 集合；无锁另设计同步；固定 NNCF/OpenVINO 工具链及量化/独立验证集后生成同 I/O IR，由 `model_path` 替换 | 双模型同步/异步/缓冲回归；同机吞吐、源年龄、丢帧与整链 P95；量化另比较精度 |
| I10 数据闭环 | 录制、标注和批量评测接口已有；缺本轮真实素材/审核标签/设备真值 | 使用下方现有命令；多配置可做有限网格比较，调参集与验收集分开；不重复开发工具 | 指纹/审核绑定、P/R、角点与合格位姿真值误差；普通录像不能替代 I5/I7 的专项实验 |
| A 位姿回归 | `DetectionBatch` 只有二维检测，无位姿/协方差输出；缺训练/导出与校准资产 | 准备数据和模型，设计检测+PnP 联合替代边界及输出契约 | 独立六自由度真值、尺度/坐标一致性、分布外和不确定度检查 |
| B 学习残差 | `predict_future` 接收只读单峰快照，缺轨迹特征和模型资产 | 定义历史输入、训练/导出、残差与不确定度契约；常数补偿不算学习 | 独立轨迹、多步误差、分布外失效及协方差一致性 |
| C 滑窗/因子图 | Tracker 无相机标定/投影输入；单存姿态历史不足联合重投影 | 定义投影、窗口、边缘化、跨世代清理及联合协方差预算 | 含跨帧真值的遮挡/歧义/重置场景和计算预算；不把单帧 LM 称为滑窗 |

### I0、I8、I9、I10 的现有入口

I0 的报告字段在标定 YAML 中使用，不是开启生产可靠性的通用开关。已有软件检查可运行：

```bash
ctest --test-dir build-debug --output-on-failure \
  -R '^(test_evidence|test_bootstrap|test_calibration_report|test_camera_disconnected)$'
```

I8 复用 `config/hardware/camera.yaml` 的静态选项，须先满足设备/标定条件；公共硬件运行
入口仍有现有限制。I9 在独立配置副本中改变 `detector.device` 或同 I/O 的 `detector.model_path`；
本轮不假定 GPU/AUTO 可用，也不硬编码 CPU 亲和核号。实际量化产物尚未生成。

I10 使用已有离线录制和标注流程。以下 `/path` 替换为实际来源，输出必须全新；
标注发布使用 WSL 原生文件系统，不能以 `/mnt/e` 支持原子发布为前提：

```bash
dataset_runs=$(mktemp -d /tmp/autoaim-dataset.XXXXXX)
build-debug/offline_replay --config /path/config.yaml --input /path/events.yaml \
  --output "$dataset_runs/commands.tsv" --record-session "$dataset_runs/session"
build-debug/annotate_session --export "$dataset_runs/draft" --session "$dataset_runs/session"
# 人工按物理 TL/TR/BR/BL 标注、核验，并设置 reviewed；本命令不替代人工审核。
build-debug/annotate_session --check --session "$dataset_runs/session" \
  --annotations "$dataset_runs/draft/annotations.yaml" --output "$dataset_runs/review"
build-debug/annotate_session --apply --session "$dataset_runs/session" \
  --annotations "$dataset_runs/draft/annotations.yaml" --output "$dataset_runs/labeled"
build-debug/bench_detector --dataset "$dataset_runs/labeled/events.yaml" \
  --config /path/config-a.yaml --config /path/config-b.yaml --iou 0.5 \
  --output "$dataset_runs/comparison"
build-debug/pipeline_metrics "$dataset_runs/commands.tsv" --fire-age-s 0.08
```

上述录制回放已有输入，不等于采集了新的实机素材；`/tmp` 产物需归档。
有合格独立位姿真值时才额外提供 `--pose-reference`、`--pose-position-limit-m` 和
`--pose-rotation-limit-rad`；没有时保留不产生位姿指标的状态。

### 组合与验证状态

默认版本与六个候选分别使用独立配置验证，普通默认构建不证明候选分支已经运行。
I9-THROUGHPUT 要求 OpenVINO ON；I3-LINEAR-CA 与 ESO 互斥，二者不构成有效组合。
其它开关组合也不能借单项通过记录认定通过。ESO 只接受当前 6 维完整位姿观测，
不能直接与 I4 的四维提案组合。许可来源、源曝光时间、不可变快照和模块依赖方向继续遵循原契约。

当前六个候选均已完成单项软件验收，配置、结果与证据统一见
[集中入口验收](build_history.md#central-selection-20260930)；未实现方案仍为“待接口/资产”。
此前[首轮](build_history.md#algorithm-alternatives-20260930)与
[剩余候选](build_history.md#remaining-alternatives-20260930)记录针对当时手动解除注释的版本，
不能代替本次 CMake 条件分支、专项注册和组合拒绝检查。目标 NUC 性能、设备效果与真实精度
仍需独立实测，不以软件回归替代。

## 附：一句话概括

**从"像素如何由运动中的三维刚体产生，以及子弹何时与哪块板相交"出发，保留足够的几何、时间与不确定度信息；用快路径、有限候选与有界窗口把计算限制在实时预算内；最终使能统一由控制层形成，发送阶段联锁只保留或撤销已有许可。**
