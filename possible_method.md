# XTYF-AutoAim 替换方案集（possible_method）

> **文档状态：已审核的方案集。** 以下区分代码现状、可替换候选与尚缺接口或数据的提案；审核不等于算法收益或设备验收。
> 完整编译期候选包括 **I1-CONTRAST-IRLS（中心轴对比度精修）**、**I2-LM（角点重投影精修）**、**I3-LINEAR-CA（EKF 平移恒加速度对照）**、**I9-THROUGHPUT（OpenVINO 编译提示）**、**I9-PREALLOC（pending 容器预分配）**；既有 ESO 单独登记。它们是各提案的局部候选，其余部分逐项列明接口、数据和硬件缺口。
> 启用、替换、回滚与验证方式统一见 [additional_information.md：算法替换登记](additional_information.md#algorithm-alternatives)。候选通过根 CMake 的六个布尔开关统一选择，默认均为 OFF；启用/回退不再修改 cpp/hpp 注释或估计器别名。当前验证状态见 [集中入口验收](build_history.md#central-selection-20260930)。
>
> **硬约束**：Intel Core i5-12450H NUC、C++、既有许可链与时效契约。
> **依赖前提**：本轮候选仅使用现有依赖（OpenCV 4 / Eigen3 / yaml-cpp / OpenVINO / Threads）。后续训练、量化等提案的工具链另行核验，不视为仓库已具备。

---

## 目录

- [1. 目标口径与约束](#1-目标口径与约束)
- [2. 审核后的代码事实（F1–F5）](#2-审核后的代码事实f1f5)
- [3. I0：补齐需要的证据装配通路](#3-i0补齐需要的证据装配通路)
- [4. 替换方案 I1–I10](#4-替换方案-i1i10)
- [5. 备选高风险方案 A–C](#5-备选高风险方案-ac)
- [6. 优先级与组合](#6-优先级与组合)
- [7. 未验证边界](#7-未验证边界)
- [8. 锚点索引](#8-锚点索引)

---

## 1. 目标口径与约束

### 1.1 六条目标（打分口径）

来自 [additional_information.md §1.1](additional_information.md)：

| 编号 | 目标 |
| --- | --- |
| ① | 更稳定的装甲板识别与 PnP 位姿解算 |
| ② | 对非匀速、突变与不规则运动的**短时**预测 |
| ③ | 缩短相机曝光到发送云台指令的端到端延迟 |
| ④ | 控制旧帧、过期结果与异常状态，避免错误指令继续下发 |
| ⑤ | 仅在条件满足时允许自动开火，失效时立即撤销 |
| ⑥ | 保留 C++、OpenVINO、串口/CAN 协议与主要工程结构 |

### 1.2 不可替换的三件事

1. **NUC + C++**（用户约束）。
2. **许可链结构**：最终开火使能唯一来源是 `command_guard`，发送阶段只做减法（§5.9）。算法只提供证据，不改变“请求 ≠ 许可”。
3. **契约语义**：不可变快照、源时间不可刷新、世代边界、消费点重新计算新鲜度（§3.2、§4）。

### 1.3 时间预算：延迟影响许可的新鲜度

150 fps 对应约 6.67 ms 的采样周期。当前示例配置为（[armor.yaml](config/offline/armor.yaml)）：

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `safety.fire_age_s` | 0.08 | 超过即不允许开火 |
| `safety.control_age_s` | 0.15 | 超过即不允许控制 |
| `queue.maximum_age_s` | 0.1 | 超过即不接纳该结果 |
| `publish_period_s` | 0.01 | 发布线程周期 |
| `after_send_delay_s` | 0.01 | 预计发送后延迟（当前为常数） |
| `intent_lifetime_s` | 0.05 | 指令有效期 |

80 ms 相当于 **12 个 150 fps 采样周期**，不是 5–6 帧。门限按时间判断，不保证允许积压某个固定帧数；采集、处理、发布和写出共同消耗源年龄预算，实际可用预算由各消费点的复检决定。

---

## 2. 审核后的代码事实（F1–F5）

以下结论以当前工作区为准，包含已有但尚未提交的 `StateEstimator`/ESO 接口改动。代码事实与需数据验证的推论分开描述。

### F1　默认离线装配证据不足，但标定报告装载与局部度量已经存在

`Evidence::qualifies()` 检查与时钟域匹配的证据等级，以及非空且匹配的 device/configuration；`Evidence::declared()` 不满足该判定（[evidence.hpp](include/autoaim/core/evidence.hpp)）。默认示例的相关情况为：

| 路径 | 当前装配 | 边界 |
| --- | --- | --- |
| 角点映射证据 | [bootstrap.cpp](src/pipeline/bootstrap.cpp) 使用 `Evidence::declared()` | 默认装配不能据此获得 `corners_reliable` |
| 标定证据 | 示例 [calibration.yaml](config/offline/calibration.yaml) 只有声明 | `load_calibration` **已有** `intrinsic_report_file` / `extrinsic_report_file` 入口，报告须通过参数、来源和验证残差等检查；不能说全项目无装载入口 |
| 几何证据 | 配置解析使用 `declared_evidence("geometry.calibrated")` | 默认配置不能通过 `GeometrySelector` 的合格证据检查 |
| 控制通道证据 | 默认装配仍为声明 | `command_guard` 对证据不足保持抑制；不是数值算法故障 |

因此，默认配置不会进入要求收敛、已知身份和合格几何证据的完整预测/许可路径。但合成夹具可显式构造对应证据，检测、PnP、滤波数值、延迟和保守分支均可独立度量。补证据入口不是所有算法评测的共同前置，也不能由离线配置未接线推出可靠分支从未执行。

### F2　瞬时量测雅可比秩至多 4，不改变完整 6 维 NIS 的自由度

`MeasurementModel::predict()` 仅依赖 `center` 和 `phase`（[measurement_model.cpp](src/estimation/measurement_model.cpp)）；速度、角速度及加速度列为零，故瞬时 `rank(H) ≤ 4`。这不是整个动态系统的可观测性结论。

当前完整姿态残差为 6 维，创新协方差是 `S = HPHᵀ + R`。当 S 满秩且噪声模型校准时，NIS 使用 **6 维**卡方门限；不能仅凭 `rank(H)` 改用 4 维门限。状态模型不能解释的姿态误差可能用于检测模型不匹配，但是否造成误拒须核查 R、标定偏差和真实残差，不能由秩直接断言。参见 [NIS 定义与自由度，§III.A](https://arxiv.org/pdf/2306.07225)。

示例健康门限为连续 4 次未更新（[health.cpp](src/estimation/health.cpp)、[armor.yaml](config/offline/armor.yaml)），作用于各假设；[tracker.cpp](src/estimation/tracker.cpp) 在所有假设不健康或传播失败等条件下才重新初始化，不是任一假设被拒就重置整个 tracker。

### F3　默认 EKF 关闭平移加速度，实验 ESO 使用对应模型

默认 `AUTOAIM_USE_ESO=OFF`，`StateEstimator` 选择 `Ekf`，装配的运动模型关闭平移 CA；`MotionModel::propagate/constrain` 显式清零 `ax/ay/az` 及对应协方差方向（[motion_model.cpp](src/estimation/motion_model.cpp)）。因此默认路径不会利用这三维初始方差。

12 维布局与 `with_linear_acceleration()` 已实现。现有 [eso.hpp](include/autoaim/estimation/eso.hpp) 由 `AUTOAIM_USE_ESO` 的编译定义控制，并在构造和换模型时启用平移 CA；其测试也覆盖加速度传播。应表述为“默认 EKF 未启用，实验 ESO 可启用”，不应删除这些维度或称它们永远无效。

### F4　种子残差字段未直接消费，姿态不一致仍进入身份评分

`rotation_residual_rad` 在 [armor_id.cpp](src/estimation/armor_id.cpp) 聚合初始化后未被直接读取；[tracker.cpp](src/estimation/tracker.cpp) 仅取种子状态，各种子共用配置的 `initial_covariance`。

但完整姿态残差仍通过更新前 NIS、关联代价进入 `IdentityResolver` 的 EMA 评分。因此不能说身份不确定度完全丢失。该残差是姿态不匹配范数，不是已标定的相位标准差；直接平方填进 phase 方差，或在同次量测 NIS 外再计一次分，都需先处理误差方向和重复用证据的问题。

### F5　`degraded` 快照可携带 `pose_reliable == true`（当前链路有质量门控）

`snapshot(now)` 重建时仅替换 `quality`，保留 `pose_reliable`（[tracker.cpp](src/estimation/tracker.cpp)）；下游 `predict_future` 还要求 `quality == converged` 才进入已知几何预测（[predictor.cpp](src/decision/predictor.cpp)）。保留这一区别，用契约回归覆盖，不把历史位姿质量与当前跟踪质量混为一项。

### 2.1 已否决的改动依据

| 疑点 | 复核结论 |
| --- | --- |
| “YOLO11 标签字典下标越界” | 不成立。`colors[raw_class_id % 3]` 对 18/19/20 取 0/1/2，`colors[24-21]` 取 3，均在 4 项数组内（[detection_labels.cpp](src/vision/detection_labels.cpp)）。 |
| “检测器与精修颜色判据相反” | 不成立。传统检测的饱和通道差与精修的正阈值有符号通道差具有相同的敌方颜色判定方向（[detector_traditional.cpp](src/vision/detector_traditional.cpp)、[corner_refine.cpp](src/vision/corner_refine.cpp)）。 |

---

## 3. I0：补齐需要的证据装配通路

- **替换对象**：默认配置尚未装配的角点、几何和控制能力证据；已有标定报告入口继续复用。
- **现状锚点**：F1；[calibration.cpp](src/vision/calibration.cpp)、[calibration_report.cpp](src/vision/calibration_report.cpp)、[bootstrap.cpp](src/pipeline/bootstrap.cpp)。
- **方案**：为确需覆盖的 replay 完整链路补受限 simulation 证据入口；或在评测侧单独输出“假定所需证据成立”的结果，并明确其不构成生产许可。不能用普通 YAML 布尔值将声明升级为实测。
- **服务目标**：补齐特定可靠分支与许可契约的离线覆盖；检测、数值滤波和耗时比较不必等待此项。
- **落地边界**：涉及装配、证据来源与配置契约，本轮不作为同接口算法候选。
- **验证**：replay 合格证据正例、错误设备/配置拒绝、host_monotonic 拒绝 simulation；既有标定报告绑定检查保持有效。

---

## 4. 替换方案 I1–I10

每项保留现状、候选方向、落地边界和验证路径。只有已列为完整编译期候选的部分可通过 CMake 开关直接切换；其余提案不代表接口已经具备。

### I1　感知前端：粗定位 + 物理语义一致的边缘角点精修

- **现状**：传统路径四角来自灯条长轴中心线端点；精修对两条灯条边做 Huber 直线拟合并限制移动量（[detector_traditional.cpp](src/vision/detector_traditional.cpp)、[corner_refine.cpp](src/vision/corner_refine.cpp)）。PnP 以配置 `pixel_sigma` 为噪声基准，并按置信度、视角和可靠性缩放。训练端点定义与 `plate_sizes` 的实物对应仍未核验，不能断言配置必为外缘尺寸或已经存在固定尺度偏置。
- **方案**：核验模型标签与实物尺寸后，在灯条 ROI 中拟合边缘，得到与板模型一致的物理角点；利用三维板几何及透视投影约束检查候选。三维平行、等长的灯条在图像中一般不平行、不等长，不能直接强加图像空间约束。
- **服务目标**：①。
- **完整局部候选 I1-CONTRAST-IRLS**：保持 `refine_corners` 签名、中心线端点和原筛选条件，以颜色对比度加权 Huber IRLS/TLS 拟合轴，最多 10 轮；仍由全部筛选点投影 min/max 给端点，原位移/可靠性/整体失败语义不变。完整候选由 `AUTOAIM_I1_CONTRAST_IRLS` 选择，默认 OFF；不输出新协方差或外缘语义。
- **落地边界**：需要端点/尺寸证据以及边缘质量信息；本轮不增加外缘补偿或新角点语义。强侧视、遮挡退化仍须保留可靠性降级。
- **验证**：`test_corner_refine`、`test_pnp`、`calibration_tool --self-test`，并以真实角点标签、`--pose-reference` 核验绝对尺寸与位姿。

### I2　位姿识别：IPPE 多候选 + LM 精修；梯度权重和同板先验另列

- **现状**：IPPE 保留多个候选，协方差使用数值雅可比和启发式像素噪声（[pnp.cpp](src/vision/pnp.cpp)）；`SamePlatePrior` API 已有身份、世代和源时间检查，但生产 `solve_pose(..., std::nullopt, std::nullopt)` 尚未接入先验。这不等于先验 API 未实现。
- **本轮候选 I2-LM**：以每个 IPPE 候选为初值执行 OpenCV LM 角点重投影精修；保持被替换函数的签名、角点物理语义、候选/歧义及质量检查约定。完整代码与原实现由 `AUTOAIM_I2_LM` 条件选择，默认 OFF。具体替换和失败处理见 [算法替换登记](additional_information.md#algorithm-alternatives)。
- **后续方案**：边缘正交距离、梯度权重需额外图像/边缘信息；同板先验需从 tracker 向 vision 回传可证明的物理板身份；历史残差噪声校准需样本和持久统计。当前 LM 候选均不冒充已完成这些能力。
- **服务目标**：①；精修的收益和多解退化必须实测比较。
- **验证**：`test_pnp`、`test_pose_quality`；核验坐标约定、正深度、旋转有效性、精修失败、候选歧义及有噪声重投影，真实回放比较位姿误差。先验只能用于候选区分，不能替代身份依据。

### I3　状态表示：IMM 模型集 + 相位多峰表示

- **现状**：默认平移 CV，角运动在 CV/有界 CA 间按滞回和驻留规则切换（[motion_model.cpp](src/estimation/motion_model.cpp)、[state_machine.cpp](src/estimation/state_machine.cpp)）。每个假设维护单一局部状态/协方差；tracker **持续保留并更新整个假设库**，仅向快照发布一个选中假设，当前没有概率混合权重输出。
- **方案**：设计明确的运动模型集、概率混合和圆周相位后验；若 decision 需对不同物理板或相位峰进行概率规划，快照也须保存混合分量与权重。不能把单一均值和协方差输出称为完整多峰规划。
- **服务目标**：②及⑤的预测证据；原有证据、收敛和许可约束仍需满足。
- **完整局部候选 I3-LINEAR-CA**：`AUTOAIM_I3_LINEAR_CA=ON` 时只在装配中将既有两个角模型追加 `with_linear_acceleration(1.0)`，仍使用 EKF；`1.0 m²/s⁵` 为实验白 jerk PSD，不改变配置字段含义。Tracker 快照发布实际模型，未来预测沿用它，因此滤波和外推一致。
- **落地边界**：完整 IMM/多峰方案改变模型与 `TargetSnapshot` 契约，本轮不生成。F4 的种子残差不直接转成概率或方差。I3-LINEAR-CA 和已有 ESO 都**不是 IMM**，也不启用混合快照；两者分别验证；CMake 拒绝同时启用，因为 ESO 自身 jerk 参数会覆盖 I3 的模型选择。
- **验证**：`test_tracker`、`test_motion_selection` 与合成阶跃转向/变转速场景；ESO 使用 `tests/verify_alternatives.py --method ESO` 或兼容入口 `tests/verify_eso.py` 独立副本验收。软件数值测试无需先完成 I0，真实性能和精度需 NUC 数据。

### I4　预测不确定度：量测模型比较 + 有界自适应与数据标定

- **现状**：完整姿态使用 6 维残差和相应门限，F2 不构成门限错误；过程噪声与几何/未知速度等外推不确定度主要由配置给出（[armor.yaml](config/offline/armor.yaml)、[predictor.cpp](src/decision/predictor.cpp)）。
- **方案**：比较显式“位置 3 + 沿轴相位 1”量测与完整 6 维量测；前者须实际投影残差、H 与 R 后使用 4 维门限，后者须校准现有 R 或引入有依据的模型误差协方差，不能仅因 H 秩较低就随意增大 R。另可研究基于历史 NIS 的有界 Q/R 自适应，以及按距离、转速、外推时间标定的预测误差包络。
- **服务目标**：②、④、⑤。
- **落地边界**：现有 `MeasurementKind::position` 是 **3 维**；4 维量测需同步量测种类、维度和统计语义，且现有 ESO 只接受 6 维完整姿态。`linearize` 没有历史序列状态；关联会多次调用只读 `innovation` 并筛选候选，`update` 又可能同曝光多次发生，不能用这些调用数直接作无偏时间序列。自适应需明确同曝光采样边界、统计生命周期和更新前门控，不能为通过当前门控而随当前残差膨胀 R。包络表缺校准数据，本轮均不生成。
- **验证**：有真值时做 NEES，按实际残差维数做 NIS 一致性检查；同时统计误关联、拒绝率和失锁。不得仅以接受率上升证明改善。

### I5　开火判据：kσ 边距 → 矩形命中概率

- **现状**：[hit_probability.cpp](src/decision/hit_probability.cpp) 已将板姿态、发射原点、瞄准角、弹速、发射相对时间和散布等误差传播到二维落点协方差；显式噪声块采用独立近似。当前判据使用各轴 kσ 边距，**不返回概率**，不是“尚未传播这些误差”。
- **方案**：在明确的分布假设下，对保留相关项的二维落点分布计算板面矩形概率，以经打靶数据校准的 η 判断；结果仍只向 guard 提供证据。数值积分或近似须覆盖高相关、退化协方差和尾部精度，不承诺“约 50 行闭式公式”即可完成。
- **服务目标**：⑤。
- **落地边界**：`ImpactMargin`/`MarginOptions` 当前表达边距而非 `P(hit)`/η；不能偷偷复用 `sigma_multiplier` 的含义。需要类型、报告和配置语义扩展，以及概率校准数据，本轮不生成。
- **验证**：二维概率与大样本蒙特卡洛对照，再用真实打靶数据检查概率校准。数值一致不等于真实命中率可靠。

### I6　开火时机：固定预计发射时刻 → 未来窗口择时

- **现状**：`solve_intercept` 按 `estimated_send + after_send_delay` 得到固定预计发射时刻，再迭代求交（[predictor.cpp](src/decision/predictor.cpp)）；当前不断重新决策仍可能等到满足条件的窗口，但没有显式输出未来执行计划。
- **方案**：在有限未来窗口搜索发射时刻，比较瞄准可达性、弹道与命中证据；若输出延后执行计划，须同时设计到期复检、取消和过期处理。
- **服务目标**：②、⑤。
- **落地边界**：当前 `Command`/发布链没有未来执行调度语义；仅把预测时刻后移而立即发送 `shoot` 不构成择时实现。需扩决策到执行的契约并核对 `intent_lifetime`、`fire_age`，本轮不生成。多次外推成本待测，不预报百微秒收益。
- **验证**：合成旋转目标、丢帧/延迟变化、等待期间目标失效和许可撤销；统计实际执行时刻及机会，不只检查规划时刻。

### I7　控制-决策联合：常数延迟 → 伺服辨识与前馈补偿

- **现状**：`after_send_delay_s` 为配置常数；瞄准充分性使用实测反馈（[aim_adequacy.cpp](src/decision/aim_adequacy.cpp)），平滑器在辅助模式使用。当前尚无基于命令—反馈历史的在线伺服辨识通路。
- **方案**：收集带一致时间和坐标口径的命令、发送结果及反馈序列，辨识有界伺服响应与延迟不确定度，再评估预测补偿；保留“规划参考不能替代实测反馈”。
- **服务目标**：⑤及指向误差控制；补偿不意味着实际曝光到写出的耗时缩短。
- **落地边界**：需要历史状态、参数/不确定度接口和设备激励数据，不是把某个常数换成无状态函数。本轮不生成。
- **验证**：合成响应检验数值，实机阶跃/正弦响应验证辨识和补偿；NUC 与实际伺服证据缺一不可。

### I8　采集链：ROI、曝光与时间戳

- **现状**：[camera.yaml](config/hardware/camera.yaml) 配置目标 150 fps；曝光时间和传输延迟口径仍需设备标定（§3.1、§9）。
- **方案**：评估 ROI/降采样、补光与短曝光、外部触发或硬件时间戳。原提案的 **200–249 fps 是未核验目标**，249.1 fps 不是本项目已测能力；须先核验具体型号、格式、接口带宽和 ROI 条件，不能写成保证值。
- **服务目标**：①、③；尚无整链路剖析证明采集端是最大单项延迟。
- **落地边界**：需要硬件能力、照明条件、时间戳来源和标定证据；动态 ROI 还涉及内参与端点坐标同步及首版非目标约束。本轮不生成。
- **验证**：NUC + 相机测曝光中点误差、时间戳抖动、吞吐、图像质量和整链路 P95；产品规格不能代替这些测量。

### I9　运行时：保留有界 latest-first 队列，比较推理模式

- **现状**：`FrameQueue::take_latest` 取 `pending.back()`，保留尚未领取的旧帧；容量不足时先淘汰旧 pending，过期另行清理（[queue.cpp](src/pipeline/queue.cpp)）。这是 **latest-first + 有界 backlog**，不是严格只保存一个最新值。OpenVINO 默认配置为 CPU，但 `options.device` 已可配置；默认编译提示为 LATENCY（[detector_openvino.cpp](src/vision/detector_openvino.cpp)）。
- **本轮候选 I9-THROUGHPUT**：保留相同 `compile_model` 输入、可配置 device、请求槽和队列契约，仅把编译性能提示换成 THROUGHPUT；由 `AUTOAIM_I9_THROUGHPUT` 选择，默认 OFF；开启时必须同时启用 OpenVINO。它是可测配置候选，不保证帧率或 P95 改善，也不是新增异步/异构调度器。
- **完整局部候选 I9-PREALLOC**：`AUTOAIM_I9_PREALLOC=ON` 时 pending 使用 vector，预留 `min(pending_capacity, pool_capacity)`；稳定清理过期项，容量淘汰最旧项，仍从尾部领取。mutex、积压、计数、重置/关闭、任务及独立读者租约不变；其它对象仍有分配，头部移除允许 O(capacity)，不称无锁或全链零分配。
- **后续方案**：以剖析决定是否需要无锁或零分配路径、设备配置比较、线程调度调整；严格 latest-only 会改变现有 backlog 行为，必须单独设计。AUTO/异构设备须按实际插件能力核验；INT8/NNCF 需模型、工具链和数据，不属本轮候选。
- **服务目标**：③、④。
- **验证**：OpenVINO ON 构建、真实模型加载与同机 A/B；并列记录吞吐、单请求延迟、队列源年龄、丢帧和端到端 P95。OFF 构建或更高吞吐不能证明该候选降低延迟。

### I10　数据闭环：真实录制、审核标注与回归

- **现状**：[annotate_session.cpp](src/pipeline/annotate_session.cpp)、[batch_benchmark.cpp](src/pipeline/batch_benchmark.cpp) 和 evaluation-golden 已具备。仓库记录仍将真实素材、端点标签和设备标定验收列为缺口；不能据此断言用户没有任何录像，也不否定已有软件数值验证。
- **方案**：录制图像/反馈/按键等一致来源 session，用现有标注流程审核候选；加入遮挡、侧视、旋转、丢帧片段，在独立验证片段上比较配置。先用现有依赖可完成的有限网格搜索，不预设新增优化库；有设备真值时使用 `--pose-reference`。
- **服务目标**：①–⑤ 的真实验收与参数标定。它是证据采集流程，不保证任一算法方案必然有效。
- **落地边界**：需要素材、人工标签与设备测量，本轮不生成算法空壳。I5 的 η、I7 的伺服参数等还需各自合适的打靶或激励实验，普通录像不一定足够。
- **验证**：报告指纹/审核绑定、独立样本 P/R 与位姿误差、运行时指标及实机打靶结果分别报告。

---

## 5. 备选高风险方案 A–C

| 编号 | 方案 | 可研究的能力 | 当前边界 |
| --- | --- | --- | --- |
| A | 端到端 6DoF 位姿回归网络替换检测 + PnP | 联合学习检测与姿态 | 仍依赖正确坐标、尺寸和训练标签，不能绕过物理语义；需模型、数据、导出和不确定度校准，当前接口不具备完整替换条件。 |
| B | 学习式残差预测器叠加在状态外推之上 | 学习数据覆盖范围内的未建模残差 | 需轨迹数据、模型资产、分布外失效及不确定度验证；学习方法不自动违反契约，但不能用模型分数替代许可依据。 |
| C | 短滑窗联合重投影 / 因子图 | 利用跨帧约束研究噪声抑制和歧义处理 | 不保证精度上限优于其它方法；需历史窗口、联合误差和计算预算设计，且 §1.2 明确列为首版非目标。 |

三者本轮均不生成；先补适配的数据与对照基线，再决定是否扩接口和依赖。合成数据可做可行性实验，实机收益仍需实测。

---

## 6. 优先级与组合

| 顺序 | 项 | 本轮或后续动作 | 依据 |
| --- | --- | --- | --- |
| 1 | 审核基线与已有候选 | 更正 F1–F4；登记已有 ESO，不改默认选择 | 防止根据错误秩推论、已过时入口描述或未证实尺寸改代码 |
| 2 | I1-CONTRAST-IRLS、I2-LM、I3-LINEAR-CA、I9-THROUGHPUT、I9-PREALLOC | 通过默认 OFF 的 CMake 开关选择完整同接口局部候选，并提供独立构建与回退步骤 | 现有输入与依赖可支持；不把局部候选冒称完整提案，收益由比较得出 |
| 3 | I10 数据及 I1 物理语义核验 | 核验部署模型预处理/端点定义、实物尺寸，收集审核数据 | 决定精度结论是否可迁移到设备；不凭历史 IR 声明断言当前预处理错误 |
| 4 | I0 特定分支证据装配 | 按待覆盖分支补齐，复用已有标定报告入口 | 扩完整链路覆盖，不阻断局部数值和耗时测试 |
| 5 | I3、I4、I5、I6 | 数据支持后分别设计状态、量测、概率和调度契约 | 不能把接口/语义变化伪装为同名函数替换 |
| 6 | I7、I8 及 I9 后续设备优化 | 结合 NUC 剖析和设备实验推进 | 数据与硬件能力决定可行性，不预设最大延迟来源 |

**组合约束**：

- I0 按需提供证据，不是全部方案的共同前置。
- I3/I4 若同时实施，需同步残差维数、噪声统计与快照语义；当前 ESO 与新 4 维量测不直接兼容。
- I5 可为 I6 提供概率指标，但 I6 仍需独立的执行调度契约；其它择时评分不自动需要 I5。
- I8/I9 可分别比较采集与主机处理，但最终必须共同检查源年龄和设备时效。
- 12 维状态已有 ESO/平移 CA 使用者，本轮不删维；只启用经过独立验证的候选组合。
- CMake 拒绝 I3-LINEAR-CA 与 ESO 同时启用，也拒绝 THROUGHPUT 与 OpenVINO OFF。I9-PREALLOC 与 THROUGHPUT 分别改变队列存储和推理提示；允许配置的组合仍需独立验证。

---

## 7. 未验证边界

1. **性能**：本文未提供新的 NUC 测量。时间预算和候选目标不是实测，THROUGHPUT 不保证较低延迟。
2. **精度**：重投影改善或合成测试通过不证明真实端点语义、绝对尺度或命中率改善。
3. **实机可用性**：公共运行入口仍对硬件模式有证据限制；存在相机或通信代码不等于设备已验收。
4. **证据**：I0 可补特定分支覆盖，不自动提升证据等级或带来算法收益。
5. **验证状态**：本文审核依据为代码和契约检查，不自行宣称本轮测试通过；候选测试的实际环境、结果和限制以 [算法替换登记](additional_information.md#algorithm-alternatives) 及验收记录为准。默认构建通过不证明关闭的条件分支已编译运行；此前手动注释版本的通过记录也不代替当前 CMake 集中选择版本的验证。
6. **既有未验收项**：真实模型端点语义、设备标定与吞吐、固件看门狗与回执等仍以 §9 和 [build_history.md](build_history.md) 的具体记录为准；历史测试不充当本轮执行结果。

---

## 8. 锚点索引

| 主题 | 关键位置 |
| --- | --- |
| 替换步骤与验收状态 | [additional_information.md：算法替换登记](additional_information.md#algorithm-alternatives) |
| 目标、非目标、时间和契约 | [additional_information.md](additional_information.md) §1–§4 |
| 检测与角点 | [detector_traditional.cpp](src/vision/detector_traditional.cpp)、[detector_openvino.cpp](src/vision/detector_openvino.cpp)、[corner_refine.cpp](src/vision/corner_refine.cpp) |
| PnP 与协方差 | [pnp.cpp](src/vision/pnp.cpp)、§5.2 |
| 几何与身份 | [geometry_model.cpp](src/estimation/geometry_model.cpp)、[geometry_selector.cpp](src/estimation/geometry_selector.cpp)、[armor_id.cpp](src/estimation/armor_id.cpp)、[tracker.cpp](src/estimation/tracker.cpp) |
| 量测与滤波 | [measurement_model.cpp](src/estimation/measurement_model.cpp)、[ekf.cpp](src/estimation/ekf.cpp)、[motion_model.cpp](src/estimation/motion_model.cpp)、[state_estimator.hpp](include/autoaim/estimation/state_estimator.hpp)、[eso.hpp](include/autoaim/estimation/eso.hpp) |
| 预测与命中判据 | [predictor.cpp](src/decision/predictor.cpp)、[hit_probability.cpp](src/decision/hit_probability.cpp)、[aim_adequacy.cpp](src/decision/aim_adequacy.cpp) |
| 许可、装配与队列 | [command_guard.cpp](src/control/command_guard.cpp)、[bootstrap.cpp](src/pipeline/bootstrap.cpp)、[queue.cpp](src/pipeline/queue.cpp) |
| 证据与标定报告 | [evidence.hpp](include/autoaim/core/evidence.hpp)、[calibration.cpp](src/vision/calibration.cpp)、[calibration_report.cpp](src/vision/calibration_report.cpp) |
| 评测 | [evaluation.cpp](src/vision/evaluation.cpp)、[batch_benchmark.cpp](src/pipeline/batch_benchmark.cpp)、[annotate_session.cpp](src/pipeline/annotate_session.cpp) |

---

## 附：采纳时的文档同步要求

按 §10.3 同步相关 README、接口职责说明和实际验收日志；替换说明统一维护在 [additional_information.md](additional_information.md#algorithm-alternatives)。状态必须区分**完整编译期候选 / 已启用 / 已验证 / 待数据或接口**，不把静态审核、历史通过或默认禁用构建当作候选的运行验收。
