
# AutoAim

模块化 RoboMaster 自瞄框架：单向依赖、接口隔离、后端可插拔、每层可独立测试。

## 项目结构

```text
autoaim/
├── CMakeLists.txt
├── cmake/
├── config/
├── include/
│   └── autoaim/
│       ├── core/
│       ├── math/
│       ├── hal/
│       ├── vision/
│       ├── estimation/
│       ├── decision/
│       ├── control/
│       ├── pipeline/
│       └── mission/
├── src/
│   └── ...                 # 与 include 对应
├── apps/
├── tests/
└── tools/
```

## 目录说明

| 目录 | 说明 |
| --- | --- |
| `CMakeLists.txt` | 顶层构建入口，定义目标、依赖与安装规则 |
| `cmake/` | CMake 模块、工具链、编译选项 |
| `config/` | 运行期配置：相机、检测、跟踪、弹道、协议 |
| `include/` | 公共接口与数据结构，按模块分目录 |
| `src/` | 模块实现，编译为 `autoaim_<module>` 库 |
| `apps/` | 可执行入口：在线、回放、标定、基准 |
| `tests/` | 单元、合成、回放、故障注入测试 |
| `tools/` | 可视化、仿真、协议测试等辅助工具 |

## 核心模块

| 模块 | 职责 |
| --- | --- |
| `core` | 时间、结果、配置、日志、基础类型、溯源与证据分级 |
| `math` | 角度、变换、SE3、数值方法 |
| `hal` | 相机、云台反馈与操作手输入、传输、时钟抽象 |
| `vision` | 帧、检测、PnP 候选、标定 |
| `estimation` | 观测、关联、Tracker、运动模型、EKF、状态机 |
| `decision` | 预测、选板、弹道、瞄准充分性 |
| `control` | 指令、安全门控、平滑、协议、CRC、看门狗 |
| `pipeline` | 主循环、队列、调度、命令槽、契约组装 |
| `mission` | 任务策略、目标优先级、开火权威 |

## 依赖方向

```text
apps → pipeline
         ├→ mission → decision → estimation → vision → math → core
         ├→ control → hal → core
         └→ hal
```

依赖只能向下，同层通过接口通信。`pipeline` 是唯一编排层：独占持有传输、组装契约（含 `ControlIntent`）并注入下层。

## 横切约定

| 约定 | 内容 | 落点 |
| --- | --- | --- |
| 契约溯源 | 跨模块对象携带帧号、世代、曝光时间、有效截止、坐标系与单位；「是否有效」在消费点按当前时间计算，不存成 bool | `core/time.hpp`、`core/units.hpp`、各契约 |
| 开火权威与指令空间 | 自主（哨兵、打符）独占云台与开火位；受监督（步兵）只拥有相对修正通道，云台指向由操作手输入与修正量叠加决定 | `core/types.hpp`、`mission/`、`control/control_intent.hpp` |
| 能力证据分级 | 能力区分「未标定 / 仅声明 / 已实测」；配置声明最高只能到「仅声明」，安全判据只接受「已实测」 | `core/config.hpp` |
| 并发不变量 | 计算可并行，估计状态只能由单一线程按采集顺序更新；过期结果的丢弃点唯一 | `pipeline/` |

## 快速开始

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/apps/autoaim_node --config config/autoaim.yaml
```

## 测试

```bash
ctest --test-dir build --output-on-failure
```

## include/autoaim/

### core

| 文件 | 职责 |
| --- | --- |
| `core/types.hpp` | 基础类型、ID、错误码、开火权威与指令空间 |
| `core/result.hpp` | `Result<T>` / `Status` 统一失败表达 |
| `core/time.hpp` | 单调时钟封装、时间换算、溯源时间戳 |
| `core/config.hpp` | 配置读取、启动校验、能力证据分级 |
| `core/logging.hpp` | 结构化日志、分级输出 |
| `core/units.hpp` | 强类型单位与参考系，防量纲与坐标系错误 |

对应 `src/core/`：

| 文件 | 职责 |
| --- | --- |
| `core/result.cpp` | `Result<T>` / `Status` 实现 |
| `core/time.cpp` | 时钟与时间换算实现 |
| `core/config.cpp` | YAML 读取、校验实现 |
| `core/logging.cpp` | 日志输出实现 |

### math

| 文件 | 职责 |
| --- | --- |
| `math/angle.hpp` | 角度环绕、最短有向差、pitch 限幅 |
| `math/transform.hpp` | 齐次变换、链式法则、逆变换 |
| `math/se3.hpp` | SO3/SE3、四元数、slerp |
| `math/numeric.hpp` | 数值积分、二分求根、有限差分 |

对应 `src/math/`：

| 文件 | 职责 |
| --- | --- |
| `math/angle.cpp` | 角度环绕、最短差实现 |
| `math/transform.cpp` | 齐次变换、链式法则实现 |
| `math/se3.cpp` | SO3/SE3、四元数、slerp 实现 |
| `math/numeric.cpp` | RK4、二分、有限差分实现 |

### hal

| 文件 | 职责 |
| --- | --- |
| `hal/camera.hpp` | 相机抽象接口 |
| `hal/gimbal_feedback.hpp` | 云台姿态反馈与操作手输入抽象 |
| `hal/transport.hpp` | 串口/网络发送抽象，出站帧带序号 |
| `hal/clock.hpp` | 可注入时钟，支持离线回放 |

对应 `src/hal/`：

| 文件 | 职责 |
| --- | --- |
| `hal/camera_impl.cpp` | 相机具体后端实现 |
| `hal/gimbal_feedback_impl.cpp` | 云台反馈具体后端实现 |
| `hal/serial_transport.cpp` | 串口传输实现 |
| `hal/clock_impl.cpp` | 系统时钟/回放时钟实现 |

### vision

| 文件 | 职责 |
| --- | --- |
| `vision/frame.hpp` | 图像帧契约（图像 + 曝光时间戳） |
| `vision/frame_packet.hpp` | `FramePacket`：图像、曝光时间、姿态（自带时间戳与有效性）、内存所有权 |
| `vision/detection.hpp` | 检测输出结构（颜色、类别、四角点、置信度） |
| `vision/detector.hpp` | 检测器抽象接口 |
| `vision/detector_factory.hpp` | 检测后端工厂 |
| `vision/pnp.hpp` | PnP 候选生成：候选集合、协方差、四角物理顺序可信度 |
| `vision/calibration.hpp` | 相机模型与标定结果 |

对应 `src/vision/`：

| 文件 | 职责 |
| --- | --- |
| `vision/detector_onnx.cpp` | ONNX 检测后端 |
| `vision/detector_traditional.cpp` | 传统灯条检测后端 |
| `vision/detector_factory.cpp` | 检测器工厂实现 |
| `vision/pnp.cpp` | PnP 候选生成、多解判别、重投影验证、协方差输出 |
| `vision/calibration.cpp` | 标定、内参/外参计算 |

### estimation

| 文件 | 职责 |
| --- | --- |
| `estimation/observation.hpp` | 观测结构（位置 + yaw + 协方差） |
| `estimation/target_snapshot.hpp` | `TargetSnapshot`：目标状态、协方差、装甲板身份、世代、源帧、质量 |
| `estimation/association.hpp` | 数据关联接口 |
| `estimation/tracker.hpp` | 跟踪器抽象（跨运动模型 / 测量模型 / 数值实现三轴） |
| `estimation/motion_model.hpp` | 运动模型：恒速、恒角速度、含角加速度、少数运动模式 |
| `estimation/ekf.hpp` | EKF 数值实现（数值轴的一种，可替换） |
| `estimation/state_machine.hpp` | 跟踪状态机，含滞回与切换事件 |
| `estimation/health.hpp` | 健康监测与发散恢复 |
| `estimation/armor_id.hpp` | 自动板身份辨识 |
| `estimation/geometry_selector.hpp` | 在线几何选择（同高/两两同高/四板独立/斜轴），含几何情况声明与证据等级 |

对应 `src/estimation/`：

| 文件 | 职责 |
| --- | --- |
| `estimation/observation.cpp` | 观测构造与协方差设置 |
| `estimation/association.cpp` | 马氏距离、门控、代价矩阵 |
| `estimation/tracker.cpp` | Tracker 装配与预测/更新 |
| `estimation/motion_model.cpp` | 各运动模型的状态转移与过程噪声 |
| `estimation/ekf.cpp` | EKF 预测、更新、Joseph 形式 |
| `estimation/state_machine.cpp` | 状态机迁移与滞回 |
| `estimation/health.cpp` | 发散检测、NaN/Inf、复位 |
| `estimation/armor_id.cpp` | 板身份评分与消歧 |
| `estimation/geometry_selector.cpp` | 多几何假设评分与切换 |

### decision

| 文件 | 职责 |
| --- | --- |
| `decision/predictor.hpp` | 延迟补偿、未来状态外推、飞行时间迭代 |
| `decision/armor_selector.hpp` | 候选过滤、评分、滞回 |
| `decision/ballistic.hpp` | 弹道解算接口与结果 |
| `decision/aim_adequacy.hpp` | 瞄准充分性评估：只输出证据，不产生开火使能 |
| `decision/hit_probability.hpp` | 完整命中概率模型 |

对应 `src/decision/`：

| 文件 | 职责 |
| --- | --- |
| `decision/predictor.cpp` | 延迟补偿、外推、飞行时间迭代 |
| `decision/armor_selector.cpp` | 候选过滤、评分、滞回 |
| `decision/ballistic.cpp` | 无阻力/带阻力弹道、正反解 |
| `decision/aim_adequacy.cpp` | 误差持续性与受扰判定，输出证据与失效原因 |
| `decision/hit_probability.cpp` | 落点分布积分与实时近似 |

### control

| 文件 | 职责 |
| --- | --- |
| `control/command.hpp` | 控制指令结构，唯一产生开火使能 |
| `control/control_intent.hpp` | `ControlIntent`：开火权威、指令空间、世代、期望角、前馈、有效截止时间 |
| `control/smoother.hpp` | 指令平滑（限速、低通、死区） |
| `control/protocol.hpp` | 协议编解码、流式解析 |
| `control/crc.hpp` | CRC16-CCITT |
| `control/watchdog.hpp` | 看门狗、故障锁存、超时清零控制 |

对应 `src/control/`：

| 文件 | 职责 |
| --- | --- |
| `control/command.cpp` | 控制指令构造、开火使能合成与有效位 |
| `control/smoother.cpp` | 限速、低通、死区实现 |
| `control/protocol.cpp` | 帧打包、拆包、流式解析 |
| `control/crc.cpp` | CRC16-CCITT 实现 |
| `control/watchdog.cpp` | 超时清零、故障锁存、安全默认 |

### pipeline

| 文件 | 职责 |
| --- | --- |
| `pipeline/pipeline.hpp` | 主循环与流水线，唯一编排层，组装契约并注入下层 |
| `pipeline/queue.hpp` | 最新帧队列、有界队列，唯一丢弃点与丢帧记账 |
| `pipeline/scheduler.hpp` | 线程调度，计算并行而估计状态单写者 |
| `pipeline/command_slot.hpp` | 命令槽（单槽覆盖） |

对应 `src/pipeline/`：

| 文件 | 职责 |
| --- | --- |
| `pipeline/pipeline.cpp` | 主循环、单帧流程 |
| `pipeline/queue.cpp` | 最新帧覆盖、有界丢帧 |
| `pipeline/scheduler.cpp` | 线程启停、退出协议 |
| `pipeline/command_slot.cpp` | 单槽覆盖、提交/取出 |

### mission

| 文件 | 职责 |
| --- | --- |
| `mission/mission.hpp` | `IMission`、`MissionContext`、`MissionOutput` |
| `mission/mission_factory.hpp` | 根据 profile 创建任务 |

对应 `src/mission/`：

| 文件 | 职责 |
| --- | --- |
| `mission/mission_factory.cpp` | 任务工厂实现 |

#### mission/infantry

| 文件 | 职责 |
| --- | --- |
| `mission/infantry/infantry_mission.hpp` | 步兵任务接口 |
| `src/mission/infantry/infantry_mission.cpp` | 步兵：目标策略、开火权威（受监督 + 叠加修正） |

#### mission/sentry

| 文件 | 职责 |
| --- | --- |
| `mission/sentry/sentry_mission.hpp` | 哨兵任务接口 |
| `src/mission/sentry/sentry_mission.cpp` | 哨兵：多目标优先级、巡逻/防守策略、自主开火权威 |

#### mission/rune

| 文件 | 职责 |
| --- | --- |
| `mission/rune/rune_mission.hpp` | 打符任务接口 |
| `src/mission/rune/rune_mission.cpp` | 打符：符盘检测、旋转预测、击打时机、自主开火权威 |

## apps/

| 文件 | 职责 |
| --- | --- |
| `apps/autoaim_node.cpp` | 统一入口，`--profile=infantry | sentry | rune` 选择任务 |
| `apps/autoaim_infantry.cpp` | 步兵专用入口 |
| `apps/autoaim_sentry.cpp` | 哨兵专用入口 |
| `apps/autoaim_rune.cpp` | 打符专用入口 |
| `apps/offline_replay.cpp` | 离线回放：固定视频 + 固定时间戳 |
| `apps/calibration_tool.cpp` | 标定工具：相机标定、PnP 验证、外参标定 |
| `apps/bench_detector.cpp` | 检测基准：延迟、Precision/Recall、角点误差 |

## tests/

```text
tests/
├── unit/
├── synthetic/
├── replay/
└── fault_injection/
```

| 目录 | 典型文件 | 职责 |
| --- | --- | --- |
| `unit/` | `test_angle.cpp`、`test_transform.cpp`、`test_pnp.cpp`、`test_ekf.cpp`、`test_ballistic.cpp`、`test_protocol.cpp`、`test_crc.cpp` | 各模块单元测试 |
| `synthetic/` | `test_ekf_synthetic.cpp`、`test_armor_id_synthetic.cpp`、`test_geometry_selector_synthetic.cpp`、`test_hit_probability_synthetic.cpp` | 合成真值验收 |
| `replay/` | `test_replay_pipeline.cpp` | 视频 + 姿态日志回放，结果可复现 |
| `fault_injection/` | `test_fault_injection.cpp` | 丢帧、坏帧、野值、CRC 位翻转、半包、未收敛 |

## tools/

| 文件 | 职责 |
| --- | --- |
| `tools/replay_visualizer/main.cpp` | 可视化回放：图像、检测、跟踪、预测、开火窗口 |
| `tools/synthetic_sim/main.cpp` | 合成目标运动与观测，生成真值 |
| `tools/protocol_tester/main.cpp` | 串口协议收发、CRC、半包/粘包测试 |
