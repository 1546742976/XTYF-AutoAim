# XTYF-AutoAim

面向 RoboMaster 工程游戏的装甲板软件离线闭环。默认 C++17，允许 C++20；规格与协作约定见 [agent.md](agent.md)。

当前已连接：文件图像/姿态 → 检测/角点/IPPE → 关联与 EKF → 预测/拦截/选板 → 任务请求 → 许可检查 → 唯一发布线程 → 离线记录。首版不包含打符、MPC、CKF 或设备上线。

## 已确认相机

用户已确认采用海康 **MV-CS016-10UC 彩色工业相机**，不是同图中的 MV-CS016-10UM 黑白款。

| 项目 | 当前依据 |
| --- | --- |
| 接口、快门 | 用户提供的技术参数截图：USB 3.0、全局快门 |
| 标称分辨率 | 截图：1440×1080；不代表已确定实际采集分辨率或 ROI |
| 标称最高帧率 | 截图：249.1fps，条件为 1440×1080、Bayer RG 8；不是本项目实测结果 |
| 项目目标采集帧率 | 用户要求 150fps；不等于检测或整链路处理达到 150fps |

序列号、镜头/焦距、实际分辨率/ROI、输出像素格式、曝光/增益、MVS SDK 版本及内外参和时间标定仍待补充。相机参数、SDK 设置/回读和帧元数据已实现，不将截图参数写成已验证配置；保留 640×480 合成示例和硬件入口禁用状态。

## 构建与回归

已使用 WSL Ubuntu 22.04、GCC 11.4、CMake 3.22.1、OpenCV 4.5.4、Eigen 3.4、yaml-cpp 0.7、OpenVINO 2026.3.1。测试使用 Python 3 和内置 C++ 检查工具，无第三方测试框架；Release 下检查仍生效。依赖须预先安装，构建不会下载模型或 SDK。

### 安装依赖

安装入口为 [install_dependence.sh](install_dependence.sh)（文件名按用户要求统一）。支持 Ubuntu 22.04/24.04 amd64，默认安装 C++17/20 工具链、CMake、OpenCV、Eigen、yaml-cpp、Python3、OpenVINO 2026.3.1、USB/CAN 配套库；支持 Hikrobot MVS 本地 `.deb`、ZIP 或指定 HTTPS 直链，并可安装 Intel OpenCL 用户态运行库。不安装当前项目未使用的 ROS2、Ceres、fmt/spdlog 等组件。

```bash
# 只预览，不安装软件或修改系统配置
bash install_dependence.sh --dry-run --skip-camera

# WSL/纯离线开发：安装软件依赖，跳过相机 SDK
bash install_dependence.sh --yes --skip-camera

# 原生 Ubuntu 相机环境：先用浏览器按官网流程下载 Linux SDK ZIP，仅检查不安装
bash install_dependence.sh --check-camera-only --mvs-archive /absolute/path/MvCamCtrlSDK.zip

# 确认来源与许可后安装；ZIP 中按元数据选择唯一的 amd64 MVS DEB
bash install_dependence.sh --mvs-archive /absolute/path/MvCamCtrlSDK.zip

# 如果已经解压，可直接提供 DEB
bash install_dependence.sh --mvs-deb /absolute/path/MvCamCtrlSDK_Runtime.deb
```

默认要求完整 MVS SDK：已有头文件/库时复用，不能只凭 `/opt/MVS` 目录存在就跳过；否则需提供相机包来源。包从 [Hikrobot 官方下载中心](https://www.hikrobotics.com/en/machinevision/service/download?module=0) 获取并由使用者检查来源及许可。支持 ZIP 子目录，不猜固定包名；有多个匹配包时拒绝任选，需要解压后以 `--mvs-deb` 指明。仅含 tar.gz/setup.sh 的发行包须按厂商说明人工处理，脚本不擅自执行其中的安装器。

2026-09-29 复核旧项目 `MvCamCtrlSDK_STD_V4.7.0_251113.zip` 固定直链：本环境的 HEAD/GET 均返回 HTTP 403、`text/html` 的站点安全拦截页，无法核实该远程包内部结构；不能据此断言文件永久失效。当前脚本不硬编码该链接、不尝试绕过验证或改用第三方镜像。若从官网下载页取得可访问的 HTTPS 直链，可用 `bash install_dependence.sh --check-camera-only --mvs-url '完整HTTPS直链'` 先下载检查，去掉 `--check-camera-only` 才会继续系统安装；若仍为 403，请按上面的浏览器下载、本地 ZIP 路径操作。

下载/解包与校验在任何 APT/sudo 步骤之前完成。拒绝 HTTP 错误、HTML 假包、所检查 ZIP 成员的 CRC 错误、损坏的 DEB 和错误架构；元数据校验不证明厂商来源或设备可用。临时下载/解包文件在成功或失败退出时清理，不覆盖用户原文件。相机包检查需要已有 `python3`/`dpkg-deb`，下载另需 `curl` 与系统 CA 证书；缺少时可先运行 `--skip-camera` 准备软件环境。`--dry-run` 不联网，不能验证远程链接；与 `--check-camera-only` 互斥。本地包的预览检查会使用临时文件。

可选参数：`--intel-gpu` 安装 Ubuntu OpenCL 运行库，`--skip-openvino` 仅保留传统检测依赖，`--openvino-version X.Y.Z` 明确选择其它版本，`--mvs-root DIR` 指定已安装 SDK 根目录。GPU 选项不升级内核或 Windows 显卡驱动，不代表 GPU 推理已通过验收；设备权限仍需人工核实。

OpenVINO 使用 [Intel 官方 APT 方式](https://docs.openvino.ai/2026/get-started/install-openvino/install-openvino-apt.html)。脚本按新 OpenVINO 工具链要求检查 CMake ≥3.26，必要时使用 [Kitware 官方源](https://apt.kitware.com/)；这与上文已验证的现有 CMake 3.22 环境区分。仓库公钥使用独立 `signed-by`，不执行下载脚本，不自动修改 `.bashrc`、禁用服务或重启。

安装完成会打印实际 OpenVINO/MVS 的 CMake 参数，不自动构建或运行项目。Bash 语法、预览及 18 项无安装测试通过，包含模拟下载成功、403/连接中断、HTML 假包、嵌套/多架构 ZIP、歧义选包与损坏归档；下载成功用例使用测试包，不冒充真实 SDK 下载成功。修复后 WSL Ubuntu 22.04 Debug CTest 全量 **83/83** 通过。**本轮未实际安装软件或驱动**；Ubuntu 24.04、真实 SDK 下载/包维护脚本及设备运行效果尚未验证。

### 编译

在仓库根目录执行，OpenVINO_DIR 必须指向实际安装的 CMake 配置目录：

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug \
  -DOpenVINO_DIR=/usr/lib/cmake/openvino2026.3.1
cmake --build build-debug -j2
ctest --test-dir build-debug --output-on-failure

cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/usr/lib/cmake/openvino2026.3.1
cmake --build build-release -j2
ctest --test-dir build-release --output-on-failure
```

使用 C++20：配置时加 `-DCMAKE_CXX_STANDARD=20`，建议使用独立构建目录 `build-cxx20`。默认仍为 C++17，不要求将现有代码统一改写为 C++20 风格；本轮结果见下方验证矩阵。

无 OpenVINO 的纯传统检测构建：配置时加 `-DAUTOAIM_OPENVINO=OFF`。该构建选择任一 YOLO 后端会明确失败，不会悄悄替换检测器。

| CMake 选项 | 作用 |
| --- | --- |
| `BUILD_TESTING` | 默认 ON；CTest、模块边界和故障契约检查 |
| `AUTOAIM_OPENVINO` | 默认 ON；固定旧 YOLOv5 22 列和参考 YOLO11 50×8400 输出格式，不是任意 YOLO 模型 |
| `AUTOAIM_TEST_YOLOV5_MODEL` | 可选外部 XML 绝对路径；增加同步、异步和整链异步三项 CPU 冒烟，BIN 与 XML 相邻 |
| `AUTOAIM_TEST_YOLO11_MODEL` | 同样增加 YOLO11 三项 CPU 冒烟；两个模型均提供时另增加三路径批量比较测试 |
| `AUTOAIM_SANITIZERS` | 默认 OFF；GCC/Clang 地址与未定义行为检查 |
| `AUTOAIM_HIKROBOT` | 默认 OFF；编译相机后端，不解锁设备运行入口 |
| `AUTOAIM_HIKROBOT_INCLUDE/LIBRARY` | 显式 SDK 头目录和库文件；开启后端时必填 |

模型测试通过配置参数传入，例如 `-DAUTOAIM_TEST_YOLOV5_MODEL=/absolute/path/yolov5.xml`；不在源码中固定旧仓库路径。

内存检查使用独立构建；关闭 OpenVINO，范围为本项目离线代码，不声称检查了第三方推理运行时：

```bash
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
  -DAUTOAIM_OPENVINO=OFF -DAUTOAIM_SANITIZERS=ON
cmake --build build-sanitize -j2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-sanitize --output-on-failure
```

2026-09-29 本轮最终验证（WSL Ubuntu 22.04）：

| 构建配置 | 结果 |
| --- | --- |
| C++17 Debug，OpenVINO ON | 全量构建，CTest **106/106** |
| C++17 Release，OpenVINO ON | 全量构建，CTest **106/106** |
| C++20 Debug，OpenVINO ON | 全量构建，CTest **106/106**，确认使用 `-std=c++20` |
| C++17 Debug，OpenVINO OFF，ASan/UBSan | 全量构建，CTest **99/99**；`detect_leaks=1`、`halt_on_error=1` |
| Hikrobot SDK ON | `autoaim_hal` 静态目标编译通过，未运行设备程序 |

106 项包含由显式外部模型路径启用的 7 项 CPU 模型/批量测试；未提供模型时不包含
这些项目。内存检查不覆盖第三方 OpenVINO 运行时。根 CMake 的后端开关说明已同步
YOLOv5/YOLO11，安装脚本未增加依赖。

30 帧 v2 合成会话连续回放两次：TSV、UART14 字节、原始事件和 62 条非耗时派生
记录一致；录制会话再回放命令一致，两次批量逻辑报告一致。产物位于
`out/s8-validation-1790690925505/`（忽略的本地验证目录）。30 帧均接纳，无丢帧；
最终 control/shoot 仍禁用，首次有效控制写入年龄为 NA。批量工具如实记录传统
灯条的额外配对和漏检，不将合成结果解释为真实模型精度。

配置 YAML、本地文档链接、模块边界及 `git diff --check` 通过；C++ 长行与同一行
return 扫描通过。首版历史记录另见 agent.md §11.1；WSL 正确性、逻辑时延与本机
检测耗时均不代表 NUC 性能、150fps 达成或真实命中结果。

## 离线优化能力与用法

S1–S8 软件离线闭环已完成。新增规格集中于 agent.md §1.6，逐项实施记录及最终
验收分类见 §11.2；真实精度、相机采集与设备控制仍未验收。

### 会话与标注（S1）

`hal::SessionWriter` 在不存在的新目录保存 PNG、`events.yaml` 和独立的
`derived.yaml`。只有显式 `finish` 成功才写入完成标志，中断会话不能回放。
旧无版本事件清单仍可读取；缺少新增元数据时保持未知，不制造设备证据。

v2 清单使用 `schema_version: 2`、`domain: replay`、`complete: true`，
可选 `session: {device_id, configuration_id, model_id}`。事件顺序按非递减
`at_ns`，同时间保持文件顺序，图像 `frame_id` 必须严格递增：

- `image`：保留原有曝光与时间来源，增加可选 `received_ns`、`annotations`、
  `device_frame_id`、`device_timestamp_ticks`、`device_dropped_frames`、
  `device_frame_discontinuity`、`pixel_format`、`roi_offset: [x,y]`。
- `uart`：`bytes: [0..255, ...]` 与可选 `received_ns`，保存原始接收块。
- `button`：`valid`、`intervening`、`pressed` 及可选 `sampled_ns`。
- `feedback`：保留旧解码反馈/操作输入格式，不将它冒充原始 UART。

会话录制与 UART14 离线字节记录可同时开启，所有输出路径必须不存在：

```bash
build-debug/offline_replay --config config/offline/armor.yaml \
  --input /path/events.yaml --output /path/new-commands.tsv \
  --record-session /path/new-session --uart-output /path/new-uart.hex
```

派生记录分为 `run`（实际配置/标定/模型内容标识）、`command`（复检后命令、
inhibit 原因掩码、停止标志与写入结果）、`drop`（分原因累计）和 `timing`
（阶段墙钟汇总）。原始会话来源不被当前运行模型覆盖。FNV-1a 内容标识用于复现定位，
不是安全认证；重复运行比对逻辑结果时排除 timing。浮点写出保留完整精度。
停止记录的原因掩码为零不表示获得许可。

### 离线内参与手眼标定（S2）

`calibration_tool` 保留原 PnP 检查用法，新增：

```bash
build-debug/calibration_tool --solve-intrinsics /path/intrinsic-data.yaml \
  --output /path/new-intrinsics
build-debug/calibration_tool --solve-hand-eye /path/hand-eye-data.yaml \
  --intrinsics /path/new-intrinsics/intrinsics.yaml --output /path/new-calibration
```

数据清单模板见 [calibration-dataset.yaml](config/offline/calibration-dataset.yaml)。
必须填写棋盘格内角点/对称圆点阵的行列数、实测间距（m）、分辨率、ROI 和设备/配置
标识；图像路径相对清单。实际间距不能从重投影误差反推是否填写正确，须人工测量。

先将样本分为 `split: fit` 与 `validation`，不要把同一张图用于两组；缺少验证集
仍可输出候选参数，但不形成通过证据。每项报告至少三份拟合、两份独立验证样本。
内参采用针孔与五参数畸变，输出 K、D、总体/逐图 RMS、验证误差及失败样本。

手眼使用 OpenCV Park，输入每张板图同步的完整 `gimbal_to_reference`
（wxyz 四元数与 m 平移）及 `image_time_ns`、`pose_time_ns`。
`maximum_pair_skew_s` 必须显式提供；缺平移不能补零。拒绝重复姿态、单轴退化，
使用未参与拟合的固定板观测验证方向与残差。拟合时保持板在参考系固定，云台位姿要
覆盖多个非平行旋转轴；只有 yaw/pitch 数字不能代替完整位姿测量。

输出目录必须不存在。内参输出 `intrinsics.yaml`，不伪造外参；手眼输出
`calibration.yaml`，可携带先前内参报告。加载标定可选
`calibration.intrinsic_report_file` / `extrinsic_report_file`，路径相对标定文件。
导入重新检查实际参数、ROI、样本划分、独立残差及显式门限，不相信 `passed: true`。
内参报告只证明内参；合成来源不能升级到真实设备证据。报告不认证样本真实性，
实测仍需人工审查。

### 检测语义与 YOLO 切换（S3–S4）

检测将 `raw_class_id`、统一 `category`、`color` 和 `armor_size` 分开。
关联比较统一类别/颜色，物理板身份仍由估计层维护。
新尺寸配置为 `plate_sizes[].plate_type: small|big`；未知板型只保留二维检测。
旧 class_id 尺寸表可读并提示迁移，与新表混用直接拒绝。

- [yolov5.yaml](config/offline/yolov5.yaml)：推荐实际图像处理基线模板，固定旧 22 列输出。
  v5 无独立板型输出，必须显式填写 `detector.plate_type_by_class`，不按数字猜大小板。
- [yolo11.yaml](config/offline/yolo11.yaml)：可切换的参考装甲模型适配，
  固定输入 `1×3×640×640`、输出 `1×50×8400`，38 类和四角点；不是通用 YOLO11。
- [armor.yaml](config/offline/armor.yaml)：保留无需权重的传统灯条合成示例。
  传统检测解算需明确 `detector.plate_type`；局部灯条精修仍可选。

两个 YOLO 后端共享有界请求槽、输入生命周期和关闭等待。模型 XML/BIN 由显式路径
提供，不复制进仓库、不自动下载；加载失败不切换算法。
两种格式使用各自标签映射。YOLO11 保留原始角点顺序；v5 保留旧兼容预排
`[0,3,2,1]`，配置的 `corners.indices` 作用于预排后的候选。两者最终都需显式物理映射，
图像坐标排序不证明物理角点可靠。模板的尺寸、标定和阈值仅属于合成测试，不能用于实际相机。
CPU 推理通过不证明 YOLO11 比 YOLOv5 更准确，比较结论须来自同一真实标注集。

### 彩色相机与 UART/按键（S5–S6）

[相机模板](config/hardware/camera.yaml) 面向 MV-CS016-10UC，包含 ROI、
BayerRG8/BGR8、手动/连续白平衡；内部保持 BGR8。请求值与 SDK 回读分开保存，
图像尺寸和 ROI 必须与标定完全一致，不能自动缩放内参。
SDK 设置/回读包括 binning/decimation 固定为 1；不支持的节点明确失败。
序列号、曝光、增益、白平衡和时间参数待实测填写，YAML null 不是有效序列号。
设备帧号/跳变、原始 tick 和像素格式可记录；tick 未建立映射前只用于诊断。
150fps 是目标值，本轮只编译 SDK 和测试替身，不枚举/打开相机。

[UART 映射模板](config/hardware/uart-mapping.yaml) 要求显式填写 AB43 四元数方向、
参考系/轴基变换、yaw/pitch 符号与偏移、接收延迟估计。CRC 校验保留；
同一接收块多个完整包取最后一份，不制造不同采样时间。
HAL 只处理字节，Pipeline 解释协议；没有映射时拒绝原始 UART 回放。
发送仍为 14 字节、五字段 Command，仅唯一 publisher 写出，短写锁存失败。

`operator_input.kind: legacy_enable_event`（缺项时默认）保留旧事件的显式启用含义。
`kind: button` 使用独立电平输入，禁止从 UART 的 mode 等字段猜按键。
两种实现都编译，配置默认 `button_mode: toggle`，旁边注释说明换为 `hold`：

- toggle：每次有效按下切换辅助/自动，长按不重复切换。
- hold：按下进入自动，松开退回辅助。
- 启动按住、人工接管、故障或输入过期均不自动启用；恢复后须先有效释放再按下。
  即使两次采样直接跨过超时、没有中间回调，旧电平资格也失效。

模式变化先撤销旧指令并更新世代；按键不刷新姿态时间，发送复检单独检查其年龄。
配置仅是声明，不授予实机权限。适配器不设置最终 shoot；相对修正的固件映射仍未
核实，UART 编码器不能将其编码为启用控制包。硬件运行入口继续禁用。

### 批量评测与耗时（S7）

同一事件清单可以比较多份配置；使用原 Pipeline，不另建检测/解算算法链：

```bash
build-debug/bench_detector --dataset /path/events.yaml \
  --config /path/yolov5.yaml --config /path/yolo11.yaml \
  --config /path/traditional.yaml --iou 0.5 --output /path/new-comparison
```

每帧 `annotations` 中包含类别、颜色、板型、物理 TL/TR/BR/BL 四角、
四个 `visible` 标志及可选 `track_id`。例如：

```yaml
annotations:
  - category: three
    color: red
    plate_type: small
    corners: [[100,100], [180,100], [180,130], [100,130]]
    visible: [true, true, true, true]
    track_id: target-1-board-0
```

空列表是已标注无目标，缺项是未标注，不计入 Precision/Recall 分母。
匹配使用显式 IoU 门限：四角轴对齐包围框、IoU 降序、索引打破平局的一对一匹配。
`report.yaml` 保存几何/语义 Precision/Recall、分类错误、可见角点误差、漏检及
连续丢失，区分原始检测与管线可用匹配，并按人工介入/无介入/未知分层。
同时保留无目标、未标注、丢帧、过期和分原因计数。

默认不报告位姿误差。仅在标注提供 `pose_truth`（`position_camera_m`、
`rotation_camera_wxyz`、`reference_id`、`position_uncertainty_m`、
`rotation_uncertainty_rad`）且使用者同时提供
`--pose-reference ID --pose-position-limit-m V --pose-rotation-limit-rad V` 时，
才接纳满足参考标识和不确定度条件的真值。

`timing.yaml` 独立保存初始化、回放墙钟、池复制（含租约管理）、排队、预处理、
推理和后处理耗时。OpenVINO 融合颜色转换计入推理，异步推理墙钟包括完成轮询等待，
不是算子内核用时。原曝光到首次完整有效控制写入指标保留；没有有效写入时为 null。
测量不修改逻辑时钟、队列容量、线程数，也不证明 NUC 性能。

`synthetic_sim` 可生成带四角/可见性/轨迹 ID/模拟位姿的 v2 会话；它只画灯条，
因此数字类别明确为 unknown。这用于软件回归，不能代替真实精度验收。

## 可复现的离线示例

首次执行以下命令；输出路径必须不存在，重复运行请换新目录/文件，工具不覆盖已有结果：

```bash
build-debug/synthetic_sim --config config/offline/armor.yaml --output out/synthetic --frames 30
build-debug/offline_replay --config config/offline/armor.yaml --output out/synthetic/commands.tsv
build-debug/replay_visualizer --config config/offline/armor.yaml --output out/visualized
build-debug/pipeline_metrics out/synthetic/commands.tsv --fire-age-s 0.08
```

生成内容：BGR 图像 PNG、事件清单 events.yaml、独立几何真值 truth.tsv、发布指令 commands.tsv，以及可视化标注 PNG。可视化复用 Pipeline 的只读观察器，不另做一条检测链。

默认示例参数仅属于合成场景。配置中的 `calibrated: true` 等字段最多表示“声明”，不能成为实测证据。因此示例会产生内部决策与意图，但最终 `control=0, shoot=0`；首次有效控制写入年龄为 NA，而不是虚构零延迟。整链契约测试另用仅在 replay 时钟域有效的模拟通道证据验证控制路径。

回放采用整数逻辑时钟、文件事件顺序、推理完成屏障和发布事件屏障。它验证确定性处理，不测真实算力延迟。实时异步路径使用同一个 process_pending/consume/decide，另有多在途推理、乱序完成与缓冲生命周期测试。

## 入口与工具

| 目标 | 实际功能 |
| --- | --- |
| autoaim_node / offline_replay | 公共离线入口：`--config FILE [--input EVENTS.yaml] [--output NEW.tsv] [--record-session NEW_DIR] [--uart-output NEW.hex]`；无 output 时写 stdout |
| autoaim_infantry / autoaim_sentry | 同一入口的角色校验薄封装，配置角色不符则拒绝 |
| synthetic_sim | 按配置 profile 投影合成灯条图像、曝光事件和真值；不是实拍精度验收 |
| protocol_tester | `--self-test` 检查所有兼容协议；`--encode-stop` 只输出默认 14 字节 UART 停止包；`--decode-ab 'HEX BYTES'` 解码反馈；均不开串口 |
| calibration_tool | 保留 PnP 检查；新增 `--solve-intrinsics` / `--solve-hand-eye` 离线求解与报告，不采集硬件 |
| bench_detector | 单图加载/预热后 P50/P95；另支持 `--dataset`、多份 `--config`、显式 `--iou` 和新输出目录的同链批量比较 |
| replay_visualizer | 检测角点及有效/可靠位姿着色：灰为原始候选、橙为有效但未可靠、绿为可靠 |
| pipeline_metrics | 从 TSV 计算源年龄、每个源帧首次完整有效控制写入年龄和过期源写入比例 |

PnP 输入 YAML 使用 `plate_type: small|big` 和四个 `corners_tl_tr_br_bl: [[x,y], ...]`，
顺序为物理左上、右上、右下、左下；旧尺寸配置继续读取 class_id。仅改变图像点排序不能证明物理语义。

示例以 infantry/assist 启动。哨兵离线配置应明确设置 `role: sentry`、`initial_mode: automatic` 和 `program_fire_requested`，其余参数仍需完整提供。步兵不允许通过 initial_mode 跳过明确启用事件。

## 配置和数据格式

- `config/offline/armor.yaml`：完整合成参数，包括时效、容量、关联、噪声、拦截和安全容差。
- `config/offline/calibration.yaml`：640×480 合成内外参，不是设备标定。
- `config/offline/geometry.yaml`：合成四板等高竖直轴；统一 profile 同时支持成对分层、逐板高度及倾斜轴，组合有合成单元测试。
- `config/offline/yolov5.yaml` / `yolo11.yaml`：YOLO 离线配置模板，需提供外部模型和真实数据/标定。
- `config/offline/calibration-dataset.yaml`：内参/手眼数据格式模板，未知实测字段为 null，不可直接运行。
- `config/hardware/camera.yaml` / `uart-mapping.yaml`：待填写的设备参数/固件映射，不开启硬件。
- `config/hardware/disabled.yaml`：明确禁用设备入口，无猜测的设备默认值。

配置文件引用的路径相对配置所在目录；事件图像路径相对事件文件。CLI 的 --input/--output 相对当前工作目录。单位：长度 m、角度 rad、时长 s、时间戳 ns；四元数 w,x,y,z。相机系 x右/y下/z前，板系 x物理右/y物理上/z外法向；云台到世界姿态与参考轴必须显式提供。

最小事件格式（图像路径须真实存在）：

```yaml
domain: replay
events:
  - {at_ns: 1000000, kind: feedback, quaternion_wxyz: [1, 0, 0, 0], yaw_rad: 0, pitch_rad: 0, bullet_speed_mps: 20, pose_valid: true, status_valid: true}
  - {at_ns: 2000000, kind: image, frame_id: 1, exposure_ns: 1000000, path: frame.png, time_origin: synthetic, timing_sigma_s: 0.001}
```

at_ns 非递减，相同时间保持文件顺序；frame_id 严格递增。姿态必须先到达，且覆盖曝光时刻；范围外不外推。可选反馈 operator 包含 valid、intervening、enable_event，但文件声明不授予自动控制证据。回放只读取本地图片，不把设备索引或 URL 当视频输入。

### 输出命令接口

`include/autoaim/control/command.hpp` 中的 `autoaim::control::Command` 与用户指定的 `sp_vision_25/io/command.hpp` 保持相同字段名、类型、顺序和距离默认值；保留本项目命名空间，不依赖参考项目的绝对路径：

```cpp
struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};
```

`control` 和 `shoot` 均取发送复检后的最终许可；`yaw/pitch` 单位为 rad，`horizon_distance` 为 m，保留源意图给出的水平距离。四项聚合初始化时距离默认为 0；完整停止值使用 `Command{}`，不要使用未初始化的 `Command command;`。

内部 `CheckedCommand` 将该载荷和 `CommandMetadata` 一起保持不可变。发布回调签名为 `WriteResult(const Command&, const CommandMetadata&)`：第一参数只包含上述五个字段；第二参数保存源帧/曝光时间/世代、指令空间，以及已有 AB 协议的速度和加速度前馈，不添加到五字段接口中。回调参数仅在调用期间有效。协议编码使用 `encode(command, metadata, protocol)`，保持原串口/CAN 字节布局；不能直接序列化 C++ 结构体内存。相对指令的实机映射仍未验证，编码器继续拒绝启用的相对控制包。

### 已选择的 UART 发送协议

用户已确认使用 **14 字节 CBoard UART v2**，对应参考代码启用 `NEW_UART_PROTOCOL` 的格式。本项目用 `ProtocolKind::cboard_uart_v2` 表达，不需要另加编译宏；`default_command_protocol` 固定为该值，`encode(command, metadata)` 默认选择它。11 字节 UART、AB 发送和 CAN 编码保留为显式指定的兼容选项，不会自动回退或探测设备。

| 字节偏移（从 0 开始） | 字段 | 编码 |
| --- | --- | --- |
| 0 | header | `0xA5` |
| 1 | length | `0x0E`，总长 14 字节 |
| 2 | control | `uint8`，0/1 |
| 3 | shoot | `uint8`，0/1 |
| 4–5 | yaw | `int16` 小端，rad × 10000，向零截断 |
| 6–7 | pitch | `int16` 小端，rad × 10000，向零截断 |
| 8–9 | dist | `int16` 小端，水平距离 m × 10000，向零截断 |
| 10–11 | checksum | 前 10 字节的既有 CRC16，小端 |
| 12–13 | tail | `0x7891`，线上字节为 `91 78`，不参与 CRC |

CRC 沿用初值 `0xFFFF`、反射多项式 `0x8408`、无末尾异或。参考串口波特率为 115200；UART 距离沿用 `[0, 3.2767]` m 饱和，启用控制时超出 ±3.2767 rad 的角度被拒绝，主机内部距离不截断。停止包清零控制字段和载荷，保留帧头、长度、CRC 与帧尾。

以下工具仅输出字节，不打开串口：

```bash
build-debug/protocol_tester --encode-stop
# protocol=2 bytes=14 a5 0e 00 00 00 00 00 00 00 00 aa d5 91 78
```

14 字节只指上位机发送包。接收仍使用既有 `AB` 帧头的 43 字节反馈及半包/粘包解析，不能把收发长度混为一谈。离线 Pipeline 默认输出 TSV，可另写 UART14 十六进制记录；两者都不打开串口。协议版本已经选定，但固件的绝对/相对角度语义、人工接管映射、独立超时清零与设备执行回执仍需核实。

### 验证与离线记录

此前五字段接口对齐在 WSL Ubuntu 22.04 完成 Debug/Release 构建，CTest 均 **84/84**；包含五字段契约、非零前馈编码、原协议样例、过期撤销、发布故障和整链回放测试。

UART v2 默认选择的历史回归为 Debug/Release **85/85**，本轮扩大后的结果见构建章节。完整 14 字节非零指令、停止包、距离饱和、CRC、半包粘包、短写与关闭仍纳入测试。

TSV 是最终命令与主机元数据的诊断记录，字段保持为 write_ns、frame_id、generation、exposure_ns、control、shoot、space、yaw_rad、pitch_rad、distance_m、age_s。其中 yaw_rad、pitch_rad、distance_m 分别记录 Command 的 yaw、pitch、horizon_distance。停止包无源时 frame_id=0、age_s=NA；有源时保留来源，控制量清零。TSV 不是新增的设备协议。

stderr 另外记录接收/接纳帧数、检测/有效位姿/观测/决策/意图、无解次数、逻辑年龄、首次写入、停止写出状态、许可原因位及各原因丢帧数。过期输入比例使用接收帧数作分母；TSV 的过期源写入比例含重发/停止记录，不等同于“过期开火比例”。重发不刷新源时间，也不重复计入首次控制写入。

## 模块与关键契约

```text
apps → pipeline
         ├→ mission → decision → estimation → vision → math → core
         ├→ control → hal → core
         └→ hal
```

| 模块 | 已实现职责 / 主要接口 |
| --- | --- |
| core | Result、强单位、TimePoint/Stamp、Evidence、配置/日志、Image/BufferPool 租约 |
| math | 角度环绕、SO3/SE3、类型化坐标变换、有限性与半正定/条件数检查 |
| hal | Clock、图像/反馈/独立按键数据、文件回放/会话写出、记录端、Linux 串口/CAN、可选 Hikrobot 参数/回读/诊断元数据 |
| vision | 传统灯条、YOLOv5/YOLO11/OpenVINO、统一标签/角点语义、局部精修、内参/Park 手眼/报告、IPPE/协方差、标注/评测 |
| estimation | 统一 GeometryProfile、恒角速度/有界角加速度、测量模型、EKF/Joseph、更新前 NIS、关联/身份/几何选择、TrackerSet 和不可变快照 |
| decision | 只读未来外推、无阻力弹道、有界拦截迭代、选板滞回、落点协方差/边距代理、实测反馈瞄准充分性 |
| mission | 唯一角色/权限矩阵、中心优先目标锁定、toggle/hold、步兵辅助/自动/人工接管、哨兵请求 |
| control | CRC/既有协议、时效、修正平滑、唯一 CommandGuard 和 Publisher |
| pipeline | bootstrap、FrameSync、FrameQueue/ResultAdmission、CommandSlot、Scheduler、UART/按键组合、装配/退出、公共入口、标定/批量工具和运行元数据 |

hpp 定义契约、单位、所有权及线程要求；cpp 标注算法依据与非显然边界。源码按 include/autoaim 与 src 镜像组织。构建逻辑统一维护在根目录 `CMakeLists.txt`：显式源码清单、模块链接白名单、可选后端和 CTest 注册均在同一文件中，不再保留源码树的 `cmake/` 目录。合并构建文件不改变九模块分层或链接关系。

包含与直接依赖声明检查器的 Python 源码也内嵌在 `CMakeLists.txt`，仅在 `BUILD_TESTING=ON` 时生成到当前构建目录的 `check_dependencies.py`，由 `module_boundaries` 和 `boundary_checker_self_test` 调用。修改检查规则应编辑根 CMake 文件并重新配置，不直接编辑生成文件；检查器会跳过 CMake 内嵌文本，避免把自测中的反例误判为项目依赖。

构建文件合并的历史回归为 Debug/Release **83/83**；本轮新增功能继续遵守同一依赖白名单，当前结果见构建章节。

- FramePacket 固定源帧/曝光/世代和像素租约；TargetSnapshot 固定状态、模型、几何与质量；ControlIntent 只有请求与证据。
- 新帧入池：清理失效等待帧 → 淘汰最旧可回收等待帧 → 无空闲块则丢新帧；绝不回收在用输入。工作线程领取最新等待帧。
- 完成结果相对“最后已接纳帧”判断乱序，不相对最新采集/在途帧。101 仅在途不阻止有效的 100；101 已接纳则拒绝 100。
- OpenVINO 固定有界请求槽，原图和预处理张量持有到实际完成；仅取消逻辑结果不会提前释放内存。
- Pipeline 数据处理接口由一个处理线程串行使用；估计状态单写者，推理后端与发布线程异步。close 的调用方须遵守生命周期契约。
- command_guard 首次形成许可，发送复检只能撤销；publisher 唯一写出，失败锁存并至多再尝试一次停止。主机停止写入成功不等于设备确认停止。
- 未确认物理板身份时仅保守跟随可见板；不预测不存在的板，不授权开火。
- hit_probability 文件名沿用占位路径，实际实现是协方差传播与 k-sigma 边距代理，不是校准命中概率。

### 代码排版

根目录 `.clang-format` 统一 C++ 排版：两空格缩进、100 字符行宽、函数及分支展开，`return` 语句独占一行；函数、校验、计算和结果返回之间按逻辑留空行。长字符串使用相邻字面量拼接，字符串值不变。保留原有命名、包含顺序、注释和控制结构；Bash、Python 与 CMake 同样按逻辑分段，长命令和参数列表折行，配置值与命令参数不变。

先前纯排版阶段完成 219 个 C++ 文件检查及 Debug/Release **83/83**；本轮为功能实现，新增代码仍保持两空格、拆分长行、return 独占一行，不将先前纯排版验证冒充本轮算法验证。

## 控制模式

| 模式 | 控制请求 | 程序开火 |
| --- | --- | --- |
| 步兵辅助（默认） | 相对修正，人工为主 | 禁止 |
| 步兵明确启用自动 | 绝对指向 | 仍须全部联锁通过 |
| 人工接管/输入过期或失效 | 退出自动、切换世代 | 撤销；松手不自动恢复 |
| 哨兵自动 | 绝对指向 | 由程序请求和最终 shoot 决定 |

步兵再次自动控制要求新的明确启用事件及合格输入/通道证据。硬件反馈字段与固件权限映射未核实，不能用默认零输入当作“无人介入”。

## 验证范围与剩余验收

已实现并有测试的专项包括：三种高度布局×两类转轴、2/3/4 板边界、大角度非退化 PnP/多解/错角点/遮挡、角度跨界/变步长/模型切换/Joseph/NIS、非法弹速/无解/不收敛、池满/竞态/乱序/旧世代、模式切换/人工接管/许可不可恢复、CRC/半包粘包/写失败/关闭、同链回放和工具自检。

已知边界：

1. 缺真实录像、角点标签和设备标定，未完成真实检测/PnP 精度验收。传统灯条首版输出兼容几何候选，合成图中仍可见跨板配对；没有数字分类器，raw_class_id=-1、category=unknown，不能将候选数量当正确目标数量。
2. YOLOv5/YOLO11 的 CPU 加载/推理测试不证明真实标签语义、GPU 支持或精度；未验证的角点保持不可靠。批量测量能力已交付，真实准确率比较尚未完成。
3. 相机时间、云台坐标、操作输入、相对通道、手动控制优先级、固件超时清零和回执缺实测证据。CLI 禁止硬件运行；串口/CAN 仅未连接测试，Hikrobot 仅 SDK 静态目标编译。
4. 真实目标按人工干预分层的指标、NUC P95/FPS、发弹延迟/弹速、实机命中均待人工验收。逻辑回放不能证明这些结果。
5. 打符、MPC、CKF、阻力弹道、在线几何慢修正不在本轮实现。rune 及旧 gimbal_feedback_impl 空占位未纳入构建；HAL 不解释上层协议。
6. 不建立伪造实测证据的配置开关。后续设备接入须提供证据导入和已核实的协议适配；当前离线输出不能直接替换为实机闭环。

本轮未连接设备、未修改旧项目、未提交 Git；不恢复 docs/。缺失资料与人工实机验收不标作已完成。

## 分批瘦身（2026-09-29）

本轮保持外部行为、九模块及所有打符预留文件，不提交 Git。以当前工作区而非
Git HEAD 为基线；快照、逐批 diff、命令和结果保存在
`out/slimming-20260929-224127/`。各批源码修改与验证结果分开记录，不将结构优化
自动解释为运行加速。基线 Debug/Release 均为 106/106。

| 批次 | 修改范围 |
| --- | --- |
| B0 | 保存含未跟踪源码的快照、原构建参数及离线基线；不改变生产行为 |
| B1 | 移除未使用的 OpenCV videoio 配置要求；其他组件不变 |
| B2 | HAL 只声明 OpenCV core/imgcodecs 依赖，保持 SDK 和九模块边界 |
| B3 | benchmark 复用 describe_run 元数据，每套配置减少一次 YAML 文件解析，保留独有字段 |
| B3b | 九模块显式 STATIC，移除未用空模块分支；保留 autoaim_options INTERFACE 和依赖白名单 |
| B4 | 标定 CLI 与 benchmark 分离编译单元，复用现有标定实现文件；入口和求解逻辑不变 |
| B5a | 回放清单只解析一次，元数据来自已校验 Config 节点；保留旧格式和错误检查 |
| B5b | 标注使用独立惰性索引，首次匹配/副本隔离/非法帧号异常顺序不变；不推进回放时钟 |
| B6 | benchmark 不累积未消费的命令文本；仍使用真实 UART14 writer 和 RecordingTransport 完整写入计数 |
| B7 | bootstrap 复用无分配队列/池预算校验；实际构造仍校验，避免合成配置 5,529,600 字节临时池 |
| B8 | 单次比较共享一次完整预读的标注/时间/操作事实，不缓存像素；按配置重算干预分层和过期状态 |
| B9 | benchmark 实现不再包含仅标定需要的 calib3d 头，显式包含 algorithm/cmath；公共传递包含保持不变 |
| 最终回归 | Debug/Release/C++20 各 106/106；关闭 OpenVINO 的 ASan/UBSan 99/99；海康 SDK 目标编译及 79 个公共头自包含检查通过 |
| B10 | 验收后仅执行 C++20/sanitizer/SDK 的 clean 目标，减少生成文件逻辑大小 6,941,600,337 字节；缓存/日志保留，Debug/Release 产物保留 |

固定 120 帧、两套配置的离线比较：YAML 文件解析 8→2，图片解码 480→360；
修改前后及修改后连续两次的命令、逻辑事件和非耗时报告一致。Release 标定程序
由 1,735,104 降至 1,095,640 字节，不再直接依赖 OpenVINO；benchmark 保留推理后端。
该变化不取消默认 CMake 配置对 OpenVINO 的要求。

同一 WSL 环境预热后各测五次，双配置 benchmark 墙钟中位数 3.452→2.485 秒，
范围分别为 3.085–3.532 和 2.474–2.563 秒，仅是本地合成数据结果。
编译单元仍为 169；惰性索引和预读事实增加 O(N) 元数据，不缓存像素，不承诺
所有程序变小、整体峰值内存下降或 NUC 性能提升。详细命令、补丁和回滚依据见证据目录。

辅助配置已验收但已清理，不再是现成可运行产物。恢复时在 WSL 项目根目录执行：

```bash
cmake --build build-cxx20 --parallel 2
cmake --build build-sanitize --parallel 2
cmake --build build-sdk --target autoaim_hal --parallel 2
```

逐批交付见 [瘦身报告](out/slimming-20260929-224127/REPORT.md)。`out/` 不纳入 Git，
需要迁移本轮证据时应另行备份。源码回滚按批次逆序核对修改后哈希，再恢复对应
`before/` 中的受影响文件；存在后续用户编辑时停止，不用 Git HEAD 覆盖工作区。
