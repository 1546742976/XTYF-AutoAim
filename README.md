# XTYF-AutoAim

面向 RoboMaster 工程游戏的装甲板软件离线闭环。默认 C++17，允许 C++20；规格与协作约定见 [agent.md](agent.md)。

当前已连接：文件图像/姿态 → 检测/角点/IPPE → 关联与 EKF → 预测/拦截/选板 → 任务请求 → 许可检查 → 唯一发布线程 → 离线记录。首版不包含打符、MPC、CKF 或设备上线。

## 构建与回归

已使用 WSL Ubuntu 22.04、GCC 11.4、CMake 3.22.1、OpenCV 4.5.4、Eigen 3.4、yaml-cpp 0.7、OpenVINO 2026.3.1。测试使用 Python 3 和内置 C++ 检查工具，无第三方测试框架；Release 下检查仍生效。依赖须预先安装，构建不会下载模型或 SDK。

### 安装依赖

安装入口为 [install_indenpence.sh](install_indenpence.sh)（文件名按约定保留）。支持 Ubuntu 22.04/24.04 amd64，默认安装 C++17/20 工具链、CMake、OpenCV、Eigen、yaml-cpp、Python3、OpenVINO 2026.3.1、USB/CAN 配套库；可安装本地 Hikrobot MVS 包和 Intel OpenCL 用户态运行库。不安装当前项目未使用的 ROS2、Ceres、fmt/spdlog 等组件。

```bash
# 只预览，不安装软件或修改系统配置
bash install_indenpence.sh --dry-run --skip-camera

# WSL/纯离线开发：安装软件依赖，跳过相机 SDK
bash install_indenpence.sh --yes --skip-camera

# 原生 Ubuntu 相机环境：先下载官方 Linux amd64 SDK，解压得到 .deb
bash install_indenpence.sh --mvs-deb /absolute/path/MvCamCtrlSDK_Runtime.deb
```

默认要求完整 MVS SDK：已有头文件/库时复用；否则缺少本地包会在系统安装开始前报错，不能把“仅装软件库”报告为“相机驱动已安装”。包从 [Hikrobot 官方下载中心](https://www.hikrobotics.com/en/machinevision/service/download?module=0) 获取并由使用者检查许可；不同版本包名不同，不固定过期下载直链。只支持 `.deb` 安装路径，不擅自执行未知 `setup.sh`。

可选参数：`--intel-gpu` 安装 Ubuntu OpenCL 运行库，`--skip-openvino` 仅保留传统检测依赖，`--openvino-version X.Y.Z` 明确选择其它版本，`--mvs-root DIR` 指定已安装 SDK 根目录。GPU 选项不升级内核或 Windows 显卡驱动，不代表 GPU 推理已通过验收；设备权限仍需人工核实。

OpenVINO 使用 [Intel 官方 APT 方式](https://docs.openvino.ai/2026/get-started/install-openvino/install-openvino-apt.html)。脚本按新 OpenVINO 工具链要求检查 CMake ≥3.26，必要时使用 [Kitware 官方源](https://apt.kitware.com/)；这与上文已验证的现有 CMake 3.22 环境区分。仓库公钥使用独立 `signed-by`，不执行下载脚本，不自动修改 `.bashrc`、禁用服务或重启。

安装完成会打印实际 OpenVINO/MVS 的 CMake 参数，不自动构建或运行项目。Bash 语法、预览及 8 项无安装测试通过，新增测试接入后的 Debug CTest 为 83/83。**本轮未实际安装软件或驱动**；Ubuntu 24.04、MVS 包维护脚本及设备运行效果尚未实机验证。

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

使用 C++20：配置时加 `-DCMAKE_CXX_STANDARD=20`，建议使用独立构建目录 `build-cxx20`。已在上述 WSL/GCC 11.4 环境完成 C++20 Debug 全量构建及 82/82 项测试（含三项外部模型测试）。这不要求将现有代码统一改写为 C++20 风格。

无 OpenVINO 的纯传统检测构建：配置时加 `-DAUTOAIM_OPENVINO=OFF`。该构建选择 YOLOv5 后端会明确失败，不会悄悄替换检测器。

| CMake 选项 | 作用 |
| --- | --- |
| `BUILD_TESTING` | 默认 ON；CTest、模块边界和故障契约检查 |
| `AUTOAIM_OPENVINO` | 默认 ON；固定旧 YOLOv5 22 列输出格式，不是任意 YOLO 模型 |
| `AUTOAIM_TEST_YOLOV5_MODEL` | 可选外部 XML 绝对路径；增加同步、异步和整链异步三项 CPU 冒烟，BIN 与 XML 相邻 |
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

2026-09-29 最终验证：Debug 82/82、Release 82/82（均含三项外部模型测试），ASan/UBSan 79/79（关闭 OpenVINO）。30 帧合成回放重复输出逐字节一致，SDK 静态 HAL 目标构建通过。完整记录见 agent.md §11.1；WSL 正确性、逻辑时延与本机检测耗时均不代表 NUC 性能或真实命中结果。

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
| autoaim_node / offline_replay | 公共离线入口：`--config FILE [--input EVENTS.yaml] [--output NEW.tsv]`；无 output 时写 stdout |
| autoaim_infantry / autoaim_sentry | 同一入口的角色校验薄封装，配置角色不符则拒绝 |
| synthetic_sim | 按配置 profile 投影合成灯条图像、曝光事件和真值；不是实拍精度验收 |
| protocol_tester | `--self-test`、`--encode-stop`、`--decode-ab 'HEX BYTES'`；只处理字节，不开串口 |
| calibration_tool | `--config FILE --corners YAML`，检查现有标定和 IPPE 多候选；不执行相机/外参实测标定 |
| bench_detector | `--config FILE --image FILE --iterations 100`，分开报告加载和预热后 P50/P95；无标签不输出 Precision/Recall |
| replay_visualizer | 检测角点及有效/可靠位姿着色：灰为原始候选、橙为有效但未可靠、绿为可靠 |
| pipeline_metrics | 从 TSV 计算源年龄、每个源帧首次完整有效控制写入年龄和过期源写入比例 |

PnP 输入 YAML 使用 `class_id` 和四个 `corners_tl_tr_br_bl: [[x,y], ...]`，顺序为物理左上、右上、右下、左下；类 ID 必须在配置中显式对应尺寸。仅改变图像点排序不能证明物理语义。

示例以 infantry/assist 启动。哨兵离线配置应明确设置 `role: sentry`、`initial_mode: automatic` 和 `program_fire_requested`，其余参数仍需完整提供。步兵不允许通过 initial_mode 跳过明确启用事件。

## 配置和数据格式

- `config/offline/armor.yaml`：完整合成参数，包括时效、容量、关联、噪声、拦截和安全容差。
- `config/offline/calibration.yaml`：640×480 合成内外参，不是设备标定。
- `config/offline/geometry.yaml`：合成四板等高竖直轴；统一 profile 同时支持成对分层、逐板高度及倾斜轴，组合有合成单元测试。
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

TSV 字段：write_ns、frame_id、generation、exposure_ns、control、shoot、space、yaw_rad、pitch_rad、distance_m、age_s。停止包无源时 frame_id=0、age_s=NA。记录的是最终 Command，不冒充已核实的相对控制线协议。

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
| hal | Clock、图像/反馈/操作手数据、文件回放、记录端、Linux 串口/CAN、可选 Hikrobot |
| vision | 传统灯条、旧 YOLOv5/OpenVINO、角点语义/局部精修、标定读取、IPPE 多解和协方差 |
| estimation | 统一 GeometryProfile、恒角速度/有界角加速度、测量模型、EKF/Joseph、更新前 NIS、关联/身份/几何选择、TrackerSet 和不可变快照 |
| decision | 只读未来外推、无阻力弹道、有界拦截迭代、选板滞回、落点协方差/边距代理、实测反馈瞄准充分性 |
| mission | 唯一角色/权限矩阵、中心优先目标锁定、步兵辅助/自动/人工接管、哨兵请求 |
| control | CRC/既有协议、时效、修正平滑、唯一 CommandGuard 和 Publisher |
| pipeline | bootstrap、FrameSync、FrameQueue/ResultAdmission、CommandSlot、Scheduler、装配/切换/退出、公共入口和离线工具 |

hpp 定义契约、单位、所有权及线程要求；cpp 标注算法依据与非显然边界。源码按 include/autoaim 与 src 镜像组织。CMake 显式列出实现，cmake/Modules.cmake 限定模块链接，cmake/check_dependencies.py 检查包含与直接依赖声明，cmake/Tests.cmake 注册测试。

- FramePacket 固定源帧/曝光/世代和像素租约；TargetSnapshot 固定状态、模型、几何与质量；ControlIntent 只有请求与证据。
- 新帧入池：清理失效等待帧 → 淘汰最旧可回收等待帧 → 无空闲块则丢新帧；绝不回收在用输入。工作线程领取最新等待帧。
- 完成结果相对“最后已接纳帧”判断乱序，不相对最新采集/在途帧。101 仅在途不阻止有效的 100；101 已接纳则拒绝 100。
- OpenVINO 固定有界请求槽，原图和预处理张量持有到实际完成；仅取消逻辑结果不会提前释放内存。
- Pipeline 数据处理接口由一个处理线程串行使用；估计状态单写者，推理后端与发布线程异步。close 的调用方须遵守生命周期契约。
- command_guard 首次形成许可，发送复检只能撤销；publisher 唯一写出，失败锁存并至多再尝试一次停止。主机停止写入成功不等于设备确认停止。
- 未确认物理板身份时仅保守跟随可见板；不预测不存在的板，不授权开火。
- hit_probability 文件名沿用占位路径，实际实现是协方差传播与 k-sigma 边距代理，不是校准命中概率。

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

1. 缺真实录像、角点标签和设备标定，未完成真实检测/PnP 精度验收。传统灯条首版输出兼容几何候选，合成图中仍可见跨板配对；没有分类器，class_id=-1，不能将候选数量当正确目标数量。
2. 旧 YOLOv5 的 CPU 加载/推理测试不证明真实标签语义、GPU 支持或精度；未验证的角点保持不可靠。
3. 相机时间、云台坐标、操作输入、相对通道、手动控制优先级、固件超时清零和回执缺实测证据。CLI 禁止硬件运行；串口/CAN 仅未连接测试，Hikrobot 仅 SDK 静态目标编译。
4. 真实目标按人工干预分层的指标、NUC P95/FPS、发弹延迟/弹速、实机命中均待人工验收。逻辑回放不能证明这些结果。
5. 打符、MPC、CKF、阻力弹道、在线几何慢修正不在本轮实现。rune 及旧 gimbal_feedback_impl 空占位未纳入构建；HAL 不解释上层协议。
6. 不建立伪造实测证据的配置开关。后续设备接入须提供证据导入和已核实的协议适配；当前离线输出不能直接替换为实机闭环。

本轮未连接设备、未修改旧项目、未提交 Git；不恢复 docs/。缺失资料与人工实机验收不标作已完成。
