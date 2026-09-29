# XTYF-AutoAim 构建与验证记录

本文件集中保存从 README 迁出的构建日志、测试矩阵和逐批修复记录。
构建、运行与测试命令见 [README](README.md#2-编译并跑通第一个示例)；
模块详解、技术契约和实施历史见 [详细手册](additional_information.md)。

以下内容按阶段保留。测试数字只对应记录中的源码阶段、工具链和构建选项，
不代表当前工作区已重新验证；文档整理本身不产生新的构建或测试通过记录。
离线验证、SDK 编译与实机验收的边界不变，未验收项仍按原记录保留。

## 阅读导航

- [瘦身第 1、2 批（2026-09-30）](#slimming-20260930-v2)
- [并行复核与专项验证（2026-09-30）](#并行复核与专项验证2026-09-30)
- [测量闭环验收（2026-09-30）](#测量闭环验收2026-09-30)
- [分批修复进度与续修矩阵（2026-09-30）](#分批修复进度2026-09-30)
- [历史完整矩阵（2026-09-29）](#历史完整矩阵2026-09-29)
- [返回 README 测试说明](README.md#8-测试已验证范围与待完成项)

## 阶段记录

<a id="slimming-20260930-v2"></a>

### 瘦身第 1、2 批（2026-09-30）

基线提交 `ebb046ffe233621b21f1cc44cee268096b272768`，开始时工作区干净。
本轮仅删除两个空测试占位、共享两处文本 YAML 写出及四处 FNV 字节累计；
保留 Frame 公共别名、OpenCV→SE3 转换、标注的独立写出语义及其他空预留。
两名 GPT-6 Astra / Ultra 分别负责 YAML、FNV 和交叉复核；逐批落盘、验收。

独立目录为 `out/slimming-20260930-v2/`，WSL Ubuntu 22.04、GCC 11.4、
CMake 3.22.1、C++17 Debug，海康、ASan/UBSan、TSan 均 OFF。
ON 配置使用 OpenVINO 2026.3.1 和已有 YOLOv5/YOLO11 XML/BIN；
[模型摘要](out/slimming-20260930-v2/baseline/models.json)固定本次输入身份。
两配置均在修改生产代码和测试前重新配置、构建并全量运行，不复用旧目录的通过记录。

| 阶段 | 专项 | OFF 全量 / 编译单元 | ON 全量 / 编译单元 | 结果材料 |
| --- | --- | --- | --- | --- |
| 修改前基线 | — | 103/103；172 | 110/110；174 | [OFF](out/slimming-20260930-v2/baseline/off/ctest.log)、[ON](out/slimming-20260930-v2/baseline/on/ctest.log) |
| 第 1 批 | 回放与故障相关 5/5 | 103/103；172 | 本批未重跑 | [专项](out/slimming-20260930-v2/batch1/off/targeted.log)、[全量](out/slimming-20260930-v2/batch1/off/ctest.log)、[差异](out/slimming-20260930-v2/batch1/changes.diff) |
| 第 2a 批 | YAML/标定 CLI/标注/评测 4/4 | 103/103；172 | 本批未重跑 | [专项](out/slimming-20260930-v2/batch2a/off/targeted.log)、[全量](out/slimming-20260930-v2/batch2a/off/ctest.log)、[差异](out/slimming-20260930-v2/batch2a/changes.diff) |
| 第 2b 批 / 最终状态 | FNV/标定/标注/评测 7/7 | 103/103；172 | 110/110；174 | [专项](out/slimming-20260930-v2/batch2b/off/targeted.log)、[OFF](out/slimming-20260930-v2/batch2b/off/ctest.log)、[ON](out/slimming-20260930-v2/batch2b/on/ctest.log)、[差异](out/slimming-20260930-v2/batch2b/changes.diff) |

第 1 批再次确认 `tests/replay/test_replay_pipeline.cpp` 和
`tests/fault_injection/test_fault_injection.cpp` 均为零字节且未注册后删除，
仅移除随后为空的两个目录。生产来源摘要仍为
`517753954cdad1e1e42f87303af6d1e9e16c379685c0ffae4273a5b63f173ba5`，
测试文件清单仅少这两项，注册名称和公共头数量 79 均不变。
固定产物全文与基线一致，见 [比较结果](out/slimming-20260930-v2/batch1/off/compatibility.json)。

第 2a 批将标定与评测的文本 YAML 写出从两份合并为私有内联头一份，
保留标定文件存在检查、两类错误原文和写出/flush/检查顺序；标注和 HAL 写出未动。
现有测试增加实际 double 的 17 位固定字节、截断覆盖、缺父目录、目录目标和
Linux `/dev/full` 失败断言，以及标定/评测 CLI 拒绝覆盖时的退出码、诊断与原文件保护。
交叉复核修正了新测试的 Windows CRLF 预期，生产代码未因此改变；
[首轮 Linux 记录](out/slimming-20260930-v2/batch2a/first-linux-off/ctest.log)保留，
修正后重新通过上表专项和全量。Windows 分支仅静态核对，未在 Windows 运行测试。
[私有头独立包含检查](out/slimming-20260930-v2/batch2a/first-linux-off/yaml-include.json)成功。
[固定产物比较](out/slimming-20260930-v2/batch2a/off/compatibility.json)通过；
报告仅两个生产来源标量变化，其余字节、两份标注 YAML、命令、UART14 和 CLI help 不变。

第 2b 批将四份 FNV 字节循环合并到 core 头文件，文件读取、灰度逐行范围、
点坐标小端编码/正负零归一和会话集合长度前缀/排序/字符串格式保留。
现有测试增加空输入、已知向量、高位字节、分段和字节计数，
65535/65536/65537 字节文件边界、固定集合摘要与精确点摘要；未改原有 golden。
另一名同配置子智能体独立核算全部新常量并交叉复核，见[复核记录](out/slimming-20260930-v2/review_notes.md)。
[公共头独立包含检查](out/slimming-20260930-v2/batch2b/off/core-include.json)通过。
两配置注册名称、数量及编译单元数与各自基线一致；公共头由 79 增至 80，
新私有头和公共头均进入生产清单，CMake 未修改。逐批来源/测试文件变更集合、
当前文件与模型摘要均已核对，见[清单核验](out/slimming-20260930-v2/final-inventory-verification.json)。
最终生产来源摘要为 `f71a361256bae4e67a4f8d0382e0b3ae8308ad8af00df45208f721e4846dae35`。
最终 [OFF 产物](out/slimming-20260930-v2/batch2b/off/compatibility.json)与
[ON 产物](out/slimming-20260930-v2/batch2b/on/compatibility.json)均满足上述字节兼容判据。
本轮结果仅说明重复实现减少和所列兼容性验证通过，不作为速度、RSS 或二进制体积收益。

固定样例在 WSL 原生 `/tmp/autoaim-slimming-20260930-v2/{off,on}` 运行，
每次归档后只清理本轮创建的位置。每配置同一程序路径、工作目录、参数和输出路径
重复两次；原始 YAML、回放 TSV、UART14 文本和墙钟 `timing.yaml` 均保留。
`index.yaml`、`annotations.yaml`、命令与 UART 字节要求全文一致；
`report.yaml` 同构建全文一致，跨生产改动仅允许两个 build 来源标量变化。
比较副本只遮蔽 `source_inventory`、`source_sha256` 的原文区间，保留原始文件和完整差异。
墙钟值单独归档，不用作性能收益结论。

每阶段的 `before/after` 保存非忽略项目文件副本与 SHA-256，`changed.json` 和
`changes.diff` 包含新增/删除文件；配置缓存、编译命令、CTest 注册、来源清单和
各命令的退出码均在对应阶段目录。回退说明要求先核对当前摘要，保留后续编辑，
本轮未执行回退、暂存或提交。材料在被忽略的 `out/` 下，不是新克隆的运行依赖。
初次第 1 批测试启动时 WSL 将选择表达式误解析为管道，未启动 CMake；
改为脚本内选择键后成功，见 [启动记录](out/slimming-20260930-v2/batch1/launch-error.json)。
收尾发现本机驱动把 `manifest.json` 清单副本覆盖为同名命令日志；修正命名后，
用已归档的原始报告 build 字段、源码快照、缓存和注册名单，通过仓库原核验器重建
六份验收清单并逐项复核摘要，原命令另存 `manifest-command.json`。
过程见[清单恢复记录](out/slimming-20260930-v2/manifest-recovery.json)；报告原件未重写，
构建与测试通过记录仍来自实际执行日志。

### 并行复核与专项验证（2026-09-30）

取证基线为 `ea4982c`，改动归因范围为 `d217004 → 59ee160 → ea4982c`。
两名 GPT-6 Astra / Ultra 子智能体分别核查既有改动与观察项，前者随后核查模型资料；
主线程执行离线验证并汇总。在本次检查范围内未发现可确证的新错误，未形成生产代码
修复包。取证阶段未修改生产源码、正式测试或运行配置，也未新增持久回归覆盖。

WSL Ubuntu 22.04，C++17 Debug、OpenVINO OFF、海康后端 OFF；复用
`out/repair-20260930-v1/debug`，先核对源码与测试指纹，再执行增量构建和专项测试：

| 检查 | 本阶段结果 | 证据 |
| --- | --- | --- |
| 构建来源 | 生产源码及测试/夹具指纹与上一轮验收一致；CTest 注册数仍为 103 | [指纹摘要](out/audit-parallel-20260930-v1/source-verification.json) |
| Debug 增量构建 | 退出码 0 | [命令](out/audit-parallel-20260930-v1/debug-incremental-build.json)、[输出](out/audit-parallel-20260930-v1/debug-incremental-build.log) |
| 既有针对性测试 | **30/30 通过**，覆盖 PnP/标定、标注评测、跟踪决策、队列及配置等相关路径 | [命令与测试选择](out/audit-parallel-20260930-v1/targeted-tests.json)、[输出](out/audit-parallel-20260930-v1/targeted-tests.log) |
| 临时审计探针 | **9/9 检查通过**；未注册进 CTest | [探针源码](out/audit-parallel-20260930-v1/observation_probe.cpp)、[编译命令](out/audit-parallel-20260930-v1/probe-compile.json)、[运行命令](out/audit-parallel-20260930-v1/probe-run.json)、[输出](out/audit-parallel-20260930-v1/probe-run.log) |
| TSan 单项复验 | `test_frame_queue` 退出码 66，`unexpected memory mapping`；**未验收** | [命令](out/audit-parallel-20260930-v1/tsan-frame-queue.json)、[错误](out/audit-parallel-20260930-v1/tsan-frame-queue.log) |
| 多配置环境 | 当前 WSL 无 Ninja，未进行多配置验收，未安装依赖 | [环境记录](out/audit-parallel-20260930-v1/environment.json) |

临时探针确认：容量恰为 8 时接受并建立 8 个假设，容量为 7 时拒绝，同帧四条观测
不再乘入初始化容量；预测拒绝后的重建分别可返回 `failure`、`success(false)` 与
`success(true)`；指定额外拼错键被忽略，缺少正确必需键则拒绝；指定三灯图可产生
三种配对。上述结果限定于夹具输入，不证明有效融合、真实精度或多配对构成缺陷。

复核同时更正了观察项表述：生产路径已有事件与诊断输出；精修失败的
`corners_reliable=false` 仍传入 PnP；相机接口明确要求同一采集/生命周期线程串行调用，
不能因没有内部锁就认定同步缺陷。HAL 生产装配仍未完成。

历史两套 XML/BIN 的文件身份已核对，历史 YOLO11 的 38 类与当前字典一致，包含
类别 24/28 的紫色大/小基地。拟部署版本、预处理资料差异及网络关键点物理端点仍待核验，
见 [历史模型清单](out/audit-parallel-20260930-v1/model-inventory.json)。本阶段未运行模型、
设备或完整多构建矩阵，不作 NUC FPS、延迟、RSS 或真实精度结论；历史五项 TSan
失败与本阶段单项复验分别记录，普通并发测试通过不替代线程检查。

完整定级、资料缺口与验收条件见 [并行取证报告](out/audit-parallel-20260930-v1/REPORT.md)。
上述 `out/` 材料是本机证据，不作为新克隆仓库的运行依赖。本次追加日志仅修改本文档，
未为文档更新重新运行构建或测试，未暂存或提交 Git。

### 测量闭环验收（2026-09-30）

WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / OpenCV 4.5.4 / Eigen 3.4 /
yaml-cpp 0.7，C++17、OpenVINO OFF、海康后端 OFF：

| 配置或检查 | 结果 |
| --- | --- |
| 修改前 Debug / Release 基线 | 各 99/99 |
| 最终 Debug / Release | 各 102/102 |
| 最终 ASan/UBSan | 102/102；开启泄漏检测和 UB 遇错停止 |
| 合成标注端到端 | 导出、检查、应用、评测通过；原始会话全部文件 SHA-256 不变 |
| 报告复现 | 同一工作目录、构建、输入、参数和输出路径，完整 report.yaml 逐字节一致 |

新增三项为标注单测、工具自检及 golden 回归；模块边界与负例检查包含在全量测试中。
本阶段未重跑 OpenVINO ON、C++20 或 SDK 矩阵，下面保留其历史记录，不混作本次验证。
命令、日志、快照及回滚依据见本机 [验证记录](out/measurement-20260930-implementation/VALIDATION.md)。
该目录是本机验收证据，不是新克隆仓库的运行依赖。

### 分批修复进度（2026-09-30）

B0 已建立独立基线：C++17、OpenVINO OFF、海康 OFF 的 Debug / Release 全量各
102/102 通过；固定六帧回放和 UART 输出已保存，相同参数的完整评测报告重复一致。
这不是后续修复的验收结果。B1 已将 Tracker 的可靠性绑定到最终候选；
改选其他 PnP 候选仍可用于估计，但不能继承原选解的可靠性。新增失败用例修复后通过，
Debug 全量 102/102；Release 和扩展矩阵留待最终回归。
B2 标定相关专项 5/5 通过：去重覆盖改名图片及重复点集，内参增加消除逐图位姿影响后的数值
可观测性检查。内参报告新增 `intrinsic_quality`；旧报告可读取参数，但缺少该质量
依据时不再授予内参能力，需用独立样本重新生成报告。此数值检查不等于设备精度验收。
B3 已统一标注读取和直接评测的有限性、重复点及零面积检查，相关测试 4/4 通过；
保留合法的图外不可见角点，不重排物理角点。B4 位姿门限完整解析及批量评测测试通过：
拒绝尾随垃圾/非有限/负数，仍允许零门限。B5 专项 6/6 通过：新决策可以
复用原观测，不刷新曝光时间或支持帧数；无新观测的指令不请求开火，且到期不越过原
观测的控制时效。B6 补充 `calibration_reports`：分别记录内外参报告的文件标识、
已装载证据等级和 replay 域资格；来源结构版本为 `run_metadata_schema_version: 2`，
不改指标字段或总报告版本。旧报告缺质量数据仍记录为 missing，文件存在不等于通过。
B6 专项 3/3 通过。B7 将标注导出的逐帧全事件扫描改为一次反馈索引和二分查询；
同时间取首条、精确姿态需显式时间且有效的规则不变，专项 2/2 通过。
256 帧/256 反馈合成导出预热后五次中位数 2.083→0.664 秒，完整清单指纹不变；
这是本机工具耗时，不是 NUC 性能结论。
B8 初次验收未通过：新增独立针孔参照发现 OpenCV 4.5.4 的精确半周
旋转案例中 IPPE 返回 0.460504/0.536219 px 残差；只链接 OpenCV 的复现相同，
同输入迭代 PnP 为约 7.58e-7 px。已保留失败的 `test_pnp`，未放宽门限或替换算法。
当时专项结果为 3/4，原始失败记录保留在
[B8 证据](out/repair-20260930-v1/B8/FINDING.md)。
B8N 已按批准方案用等价求解板系避开半周数值异常，再恢复原板系计算质量；
仍是单次 IPPE 双候选，没有 LM、单解替换或自动回退。原 B8 专项现为 4/4，
门限及语义证据要求未变。B8V 扩展专项 4/4：360 组大小板、滚转、正视/倾斜、
半周附近扰动、畸变与噪声输入，双候选未合并；无噪声最大 RMS 0.000174012 px，
平移误差 1.61e-6 m、旋转误差 1.17e-5 rad。
B9 数值契约与定向检查 5/5 通过：不同检测器的分数不是统一的校准概率，
PnP 的 m/rad 信息矩阵比与标定的列归一化信息比不能混用；默认配置未调整。
B10 定向回归 10/10 通过：发送接纳之前撤销不得写出旧命令，已经接纳的在途写允许
完成后停止；同时检查位置收敛、超时新身份、CV/CA 交叉协方差与默认 declared
证据门禁。生产行为保持不变。B11 基础设施专项 3/3，新增独立验收清单自测；
本次最终矩阵如下（不是下面的历史结果）：

| 配置 / 检查 | 本次结果 |
| --- | --- |
| C++17 Debug / Release，OpenVINO OFF | 各 103/103 |
| C++20 Debug，OpenVINO OFF | 103/103 |
| ASan/UBSan，OpenVINO OFF | 103/103；泄漏检测开启、UB 立即失败 |
| C++17 Debug，OpenVINO ON，现有 YOLOv5 + YOLO11 | 110/110；含同步、异步、整链及批量条件测试 |
| 海康 SDK | autoaim_hal 编译通过，未运行硬件 |
| TSan 离线并发 | 编译通过；运行时 unexpected memory mapping，五项未验收 |
| 多配置生成器 | 环境无 Ninja，未验证，不安装依赖 |
| 固定输入重复回放 | 完整报告、命令、UART、原始事件重复一致；墙钟耗时单列 |

所有配置使用相同生产源码与测试内容标识。TSan 首轮有一项启动段错误，单次诊断
同样报告内存映射错误；不据普通 CTest 通过宣称无数据竞争。Debug 最终增量构建
无重新编译及先前的时钟偏差警告。冻结 golden、三个打符预留文件未变。

不要混用旧构建目录的结果：`build-debug` 是较早的 OpenVINO ON / C++17 Debug；
`out/measurement-20260930-implementation/debug` 是测量闭环阶段的 OpenVINO OFF；
`out/repair-20260930-v1/debug` 是本次续修的 OpenVINO OFF / C++17 Debug。
三个目录均为海康 OFF，但源码阶段不同；历史通过数字只属于其当时源码与选项。
传统角点是灯条中心线端点；YOLO 的物理端点定义仍需训练标签资料核验，不据张量
形状猜外边缘或添加固定尺寸偏移。C 类仍需单独确认。
本机快照、逐批 diff、命令和回滚说明见
[修复执行记录](out/repair-20260930-v1/STATUS.md)，不作为新克隆仓库的运行依赖。
逐批命令、指标、差异与逆序回滚入口见
[续修交付清单](out/repair-20260930-v1/Final/DELIVERY.md)。

### 历史完整矩阵（2026-09-29）

2026-09-29，WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / OpenCV 4.5.4 /
Eigen 3.4 / yaml-cpp 0.7 / OpenVINO 2026.3.1：

| 配置 | 记录结果 |
| --- | --- |
| C++17 Debug / Release，OpenVINO ON | 各 106/106 |
| C++20 Debug，OpenVINO ON | 106/106 |
| ASan/UBSan，OpenVINO OFF | 99/99 |
| 海康 SDK | autoaim_hal 静态目标编译通过，未运行设备 |
| 公共头独立编译 | 79/79 |

106 项包含提供两份外部权重后启用的 7 项模型/批量测试；未提供权重时测试数量不同。
上述为历史回归记录，不表示 2026-09-30 的修改重新通过了这套扩展矩阵。
辅助构建目录曾在验收后 clean；若只有缓存没有程序，先用 `cmake --build` 重建。
历史实施记录见 [详细手册 §11](additional_information.md#implementation-history)；
本机逐批证据在 `out/slimming-20260929-224127/`，不作为新克隆仓库必需文件。
