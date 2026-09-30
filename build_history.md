# XTYF-AutoAim 构建与验证记录

## 项目内 MVS 包适配（2026-09-30）

安装器在没有显式相机包参数、也没有完整已安装 SDK 时，选择脚本同目录的
`MVS-5.1.0_Linux_x86_64_20260909.zip`。保留显式参数优先、SDK 复用和
`--skip-camera`；从其它工作目录调用也可使用默认包。Docker 仍跳过相机安装，
不将 ZIP 加入默认镜像构建内容。

只读检查实际 ZIP：包含同名版本的 DEB 与 TAR；选择 DEB，包名 `mvs`、架构 `amd64`、
DEB Version `2025-07-11`。DEB 内还有 `MVS.tar.gz`，SDK 头文件位于 `include/`，
64 位控制库在 `lib/64/`，库文件后缀 `4.8.2.1`。未用文件名替代实际元数据。
ZIP SHA256：`257f94ffe9fc86e6bc57bedbff41e6c24673484f8f38e22475aced2431116c8a`。
厂商 postinst/setup.sh 只读取、未运行；README 和脚本帮助说明其替换 SDK、权限及
驱动/日志自启动操作，不再笼统宣称实际 DEB 安装没有系统副作用。

验证环境为已有 WSL Ubuntu 22.04：

```bash
bash -n install_dependence.sh
python3 tests/unit/test_install_script.py
ctest --test-dir build-debug -R '^test_install_script$' --output-on-failure
```

无安装基线 18/18；修改后 22/22，CTest 选定项 1/1。另从 `/tmp` 通过绝对路径运行
`--check-camera-only --mvs-root /tmp/xtyf-no-installed-mvs-20260930`，确认选择项目内真实包，
DEB 结构及数据归档检查通过。缺少 unzip 时使用已有 Python 标准库只读查看内嵌归档，
未安装额外依赖。未运行 APT、sudo、厂商安装脚本或设备程序，未声称实机或驱动安装验收通过。

<a id="docker-20260930"></a>

## Docker 环境打包（2026-09-30，矩阵验证中）

已生成开发、运行、SDK 编译检查镜像，增加 `Runtime` 安装组件、Compose 离线入口和验证脚本；
不改算法或硬件权限，不提交 Git。并行的 fast_choose/算法选择修改原样保留。
最终矩阵采用 [冻结源码](out/docker-20260930-v1/acceptance-source)与
[逐文件标识](out/docker-20260930-v1/acceptance-inventory.json)，避免验证中宿主编辑改变输入。
生产源码标识为 `95272dfca4bc351bca852bccb8bd2eaf5de28bce71b8375d2d5525a78118b128`。

环境：Docker Desktop 4.85.0 / Engine 29.6.2 / Compose 5.3.1，Linux amd64；
容器为 Ubuntu 22.04、GCC 11.4.0、CMake 4.4.3、OpenCV 4.5.4、Eigen 3.4.0、
yaml-cpp 0.7.0、OpenVINO 2026.3.1.22476。CMake 来自现有安装脚本的 Kitware 源，
不把它误写成本机 WSL 的 3.22.1。完整包版本见 [清单](out/docker-20260930-v1/environment)。
基础镜像固定为 `ubuntu:22.04@sha256:b8b6ee6aa931ecd9d0d952abc34dc0e5f7c6a30c6bb71b079fe399fde0329c02`。

| 镜像 | 镜像 ID 前缀 | Docker 内容大小（字节） | Docker 报告磁盘占用 |
| --- | --- | --- | --- |
| `xtyf-autoaim:dev` | `f7ed08ef2ca2` | 574277673 | 2.35 GB |
| `xtyf-autoaim:runtime` | `f29f62bbf443` | 146221411 | 562 MB |
| `xtyf-autoaim:sdk-check` | `fac34b304c55` | 579285601 | 2.38 GB |

完整 ID 见 [inspect 结果](out/docker-20260930-v1/final-image-sizes.txt)，
[磁盘占用](out/docker-20260930-v1/final-image-list.txt)包含共享层，不能直接相加或视为导出 tar 大小。
运行镜像冻结构建仅改变证明清单，程序/文件系统配置摘要仍为
`226d571d11f780cae16da34590c91e4a4ac85237c51dcc6019e16b5e6ae30506`，与冒烟镜像一致。

| 验收 | 结果与证据 |
| --- | --- |
| C++17 Debug / OpenVINO ON / 双模型 | 112/112，[日志](<out/docker-20260930-v1/container output/matrix-frozen/debug-ctest.log>) |
| C++17 Release / ON / 双模型 | 待完成 |
| C++20 Debug / ON / 双模型 | 待完成 |
| ASan/UBSan / C++17 / OpenVINO OFF | 待完成；要求 `detect_leaks=1`、`halt_on_error=1` |
| SDK 编译 | `autoaim_hal` 通过，[日志](out/docker-20260930-v1/sdk-check.log)；未枚举或打开设备 |
| 运行镜像无网络冒烟 | 合成、回放、可视化、标注自检/导出、两种 YOLO CPU 推理、硬件拒绝通过，[日志](out/docker-20260930-v1/runtime-posix-retry.log) |
| 完整报告确定性 | 同路径相同参数两次报告 SHA256 均为 `ba548190e2ac20049dc5b30a5a256b830fe70f90167a89dba1854cbc6a9e83eb`；源会话校验不变 |
| 挂载与用户 | 中文/空格路径、只读输入、输出持久化、UID 1000/1234 通过；[权限](out/docker-20260930-v1/permissions.log)、[自定义 UID](out/docker-20260930-v1/custom-uid.log) |
| 错误输入 | 缺模型、缺事件清单明确失败；[模型](out/docker-20260930-v1/missing-model.log)、[输入](out/docker-20260930-v1/invalid-input.log) |

### 本阶段发现并处理的问题

- 开发容器 `/tmp` 默认 noexec，使两个测试失败；只为 dev 添加 exec，保留 nosuid/nodev。
  [定向重测 2/2](out/docker-20260930-v1/tmp-exec-fix.log)，runtime 仍 noexec。
- Windows 绑定目录不支持标注工具所需的原子无覆盖发布；保留工具错误，改用 Linux 卷验收后
  显式 `docker cp` 导出产物，[结果](out/docker-20260930-v1/runtime-posix-results)。
  普通复制不充当工具的原子发布；使用方法见 README §2.4。
- 本机 Desktop 会自动创建缺失的 Windows 绑定源，即使 Compose 设置了 `create_host_path: false`。
  [原始检查](out/docker-20260930-v1/invalid-mount.log)保留；README 要求先在宿主解析/检查路径。
- 外部 Dockerfile 前端认证失败后改用引擎内置 BuildKit；没有更换官方源或升级 OpenVINO。
  依赖下载使用 APT 缓存；开发镜像最终 [缓存构建成功](out/docker-20260930-v1/dev-confirm.log)。
- Docker/WSL 多次 RPC EOF、一次 `0xc00000fd` 启动错误中断矩阵；没有重启或重配服务。
  引擎自行恢复后发现 0 字节对象文件，清理本任务的损坏产物，再从冻结源码构建。
  首次失败、续跑和重建日志均保留，没有把中断或链接失败记作通过。
- 初始本机 Debug 在 E 盘空间不足时失败，只清理本次目录；原阶段本机 Release 111/111
  及增量复测通过，见 [日志](out/docker-20260930-v1/native-release-refresh.log)。不代替容器结果。

构建和验证命令见 README §2.4；本次日志、修改前文件及缓存位于
`out/docker-20260930-v1/`。完成的容器配置通过后执行本次目录的 `clean`，保留参数和验收清单。
本阶段未运行相机、串口或设备控制，不声称真实精度、150fps 或目标 NUC 性能已验收。

本文件集中保存从 README 迁出的构建日志、测试矩阵和逐批修复记录。
构建、运行与测试命令见 [README](README.md#2-编译并跑通第一个示例)；
模块详解、技术契约和实施历史见 [详细手册](additional_information.md)。

以下内容按阶段保留。测试数字只对应记录中的源码阶段、工具链和构建选项，
不代表当前工作区已重新验证；文档整理本身不产生新的构建或测试通过记录。
离线验证、SDK 编译与实机验收的边界不变，未验收项仍按原记录保留。

## 阅读导航

- [快调配置与启动快照（2026-09-30）](#fast-choose-20260930)
- [CMake 集中算法选择（2026-09-30）](#central-selection-20260930)
- [剩余方案同接口候选（2026-09-30）](#remaining-alternatives-20260930)
- [算法注释候选与快速替换（2026-09-30）](#algorithm-alternatives-20260930)
- [ESO 兼容接口与注释实现（2026-09-30）](#eso-compat-20260930)
- [根 CMake 整理（2026-09-30）](#cmake-cleanup-20260930)
- [瘦身第 1、2 批（2026-09-30）](#slimming-20260930-v2)
- [并行复核与专项验证（2026-09-30）](#并行复核与专项验证2026-09-30)
- [测量闭环验收（2026-09-30）](#测量闭环验收2026-09-30)
- [分批修复进度与续修矩阵（2026-09-30）](#分批修复进度2026-09-30)
- [历史完整矩阵（2026-09-29）](#历史完整矩阵2026-09-29)
- [返回 README 测试说明](README.md#8-测试已验证范围与待完成项)

## 阶段记录

<a id="fast-choose-20260930"></a>

### 快调配置与启动快照（2026-09-30）

新增 `config/fast_choose.yaml`，按后续指令默认选择 `yolov5`；三个原入口固定选择自己的
检测器。108 个公共调参叶子、检测器参数及 ESO 参数集中声明，默认数值与原配置逐项
核对；身份、标定、几何、角点、板型、坐标及重力契约留在基础文件。配置注释说明单位、
范围与调整影响；修改后重启生效。未知键、重复所有权、缺项及非法值明确拒绝。

加载器保留旧完整配置，按声明文件解析相对路径，并新增批量预加载快照。运行来源格式
升级为版本 3，保存入口、基础配置、快调原始字节指纹、有效配置及实际估计器；CLI 输入
覆盖和原标定报告记录语义保持不变。`EsoOptions` 移到共享类型并通过编译期构造函数
接线，本阶段冻结快照默认仍为 EKF，ESO 算法以原块注释方式保存。同步适配临时配置、
安装清单及候选测试。随后并行任务将算法选择迁至 CMake；该改动保留，最新启用方式
以手册开头为准，不把本节快照的通过结果套用到后续 CMake 切换版本。

本轮软件验收使用独立源码快照、C++17 Debug / 海康 OFF，不改 golden 或门限：

| 配置 / 检查 | 实际结果 |
| --- | --- |
| 修改前 OFF 基线 | 103/103；172 个编译单元 |
| 默认 EKF / OFF，最终默认选择 YOLOv5 | 104/104；172 个编译单元 |
| 默认 EKF / ON，现有 YOLOv5 + YOLO11 | 111/111；174 个编译单元 |
| 独立 ESO / OFF | 105/105；173 个编译单元；仅 `test_eso.cpp` 使用 `-g0` |
| 60 帧传统链新旧入口与 ESO 参数变化对比 | 命令及 UART 逐字节一致，评测指标一致 |
| 真实 YOLOv5：统一默认入口对固定入口 | 命令及 UART 逐字节一致，评测指标一致 |

基线到最终 OFF 的新增注册项仅为并行 Docker 工作引入的 `test_install_layout`。
本轮补装 `fast_choose.yaml`，安装后搬移测试固定传统入口以免依赖外部权重；默认 YOLOv5
的真实权重对比使用临时配置提供模型路径，不将本机绝对路径写回运行配置。
默认 EKF 对比中 ESO 参数从 `10/10/1` 改为 `7/11/0.4` 后输出不变。
快调注释修改、重新加载、批量共享快照、非法输入及声明目录路径均由本轮测试覆盖。

WSL 三次在构建中退出，首轮 `/tmp` 构建丢失，随后改用 `/var/tmp` 保留进度并单任务
续建。首次续建链接失败确认由 17 个零字节 `.o` 引起；下一次中断另留下 3 个零字节对象，
均记录后定向重新生成，没有改算法。ESO 专项最后使用仅针对 `test_eso.cpp` 的 `-g0`
降低编译资源占用，其它 Debug 编译选项、断言与门限不变；中断不作为通过或算法失败结论。
首次中断日志保存在
[interrupted](out/fast-choose-20260930/interrupted)，最终配置快照与摘要在
[final-v3](out/fast-choose-20260930/final-v3)，默认算法的日志、缓存、命令、退出码及清单在
[OFF](out/fast-choose-20260930/off)和[双模型 ON](out/fast-choose-20260930/on)。
完整输入、原报告、字节与指标比较见
[兼容性记录](out/fast-choose-20260930/compatibility/comparison.json)。
ESO 的实际命令、失败及续建过程、快照不变性和最后通过结果见
[ESO 摘要](out/fast-choose-20260930/eso/summary.json)与
[完整测试日志](out/fast-choose-20260930/eso/ctest.log)。最终记录汇总见
[验收摘要](out/fast-choose-20260930/summary.json)；其中单列了当前并行工作区相对快照的差异，
不把这些后续改动写成本轮重新运行通过。
保留工作区已有及并行修改，未暂存、提交或重置 Git；不据这些软件结果作 NUC 性能或实机效果结论。

收尾（2026-09-30）：快调配置已纳入后续 [CMake 集中选择验收](#central-selection-20260930)，
其 [验收汇总](out/central-selection-20260930-v1/acceptance_summary.json) 已完成，
不再保留“等待集中选择整合验证”的软件待办。默认检测器为 YOLOv5，默认估计器为 EKF；
当前 ESO 启用入口为 `AUTOAIM_USE_ESO=ON`，参数继续读取快调文件。
本次续作沿用上述有效记录，仅核对当前实现与已测版本及文档衔接，不重复构建、测试或模型运行，
不新增测试通过记录。NUC 性能、真实模型标注资料与设备验收仍按各自原记录保留。

<a id="central-selection-20260930"></a>

### CMake 集中算法选择（2026-09-30）

按用户后续要求，六个算法选择统一在根 `CMakeLists.txt` 的“快速替换”区域，
默认全部 OFF；cpp/hpp 不再保存手工解除注释的候选窗口。构建定义选择同接口实现，
各开关值写入构建来源报告。I3-LINEAR-CA 与 ESO 的模型噪声选择冲突时配置拒绝；
THROUGHPUT 要求 OpenVINO ON。没有新增 YAML 算法键或运行时算法框架。

验证脚本复制源码后只传 CMake 开关，不改副本；CMake 按所选候选注册专项测试，
测试基线固定在 `tests/support`，不成为生产公共接口。Windows 脚本自测 36 项通过。
冻结当前合并源码（包含 fast_choose 与安装组件）后，在 WSL 新目录以
C++17 / Release / 海康 OFF 构建；默认与 I1 从新目录开始，其余逐项在隔离目录重新配置、
构建并全量测试，每次显式关闭其它五项。回退也重新构建和测试。

源码、指纹、全部命令与结果在 [本轮验收目录](out/central-selection-20260930-v1)。
首次 `/tmp` 构建遇 WSL 重启，未形成测试结果，现改用工作区持久验收目录；
[中断记录](out/central-selection-20260930-v1/interruption.json)保留实际观测，不推断重启原因。

| 集中选择配置 | 实际全量结果 | 记录 |
| --- | --- | --- |
| 默认 / OpenVINO OFF / 新目录 | 105/105 | [摘要](out/central-selection-20260930-v1/fresh/DEFAULT/summary.json) |
| I1-CONTRAST-IRLS / OFF / 新目录 | 106/106 | [摘要](out/central-selection-20260930-v1/fresh/I1-CONTRAST-IRLS/summary.json) |
| I2-LM / OFF | 106/106 | [摘要](out/central-selection-20260930-v1/incremental/vision-runtime/I2-LM/summary.json) |
| I3-LINEAR-CA / OFF | 106/106 | [摘要](out/central-selection-20260930-v1/incremental/estimation/I3-LINEAR-CA/summary.json) |
| ESO / OFF | 106/106 | [摘要](out/central-selection-20260930-v1/incremental/estimation/ESO/summary.json) |
| I9-PREALLOC / ON / 双模型 | 113/113 | [摘要](out/central-selection-20260930-v1/incremental/vision-runtime/I9-PREALLOC/summary.json) |
| I9-THROUGHPUT / ON / 双模型 | 112/112 | [摘要](out/central-selection-20260930-v1/incremental/vision-runtime/I9-THROUGHPUT/summary.json) |
| ESO 后全部关闭 / OFF | 105/105 | [摘要](out/central-selection-20260930-v1/incremental/estimation/DEFAULT/summary.json) |

六个候选均“可替换且已验证（软件）”。所有配置包含模块边界检查，两个 I9 均实际注册并通过
七项 YOLOv5/YOLO11 条件测试；两个 XML/BIN 对的指纹在测试前后相同。
配置拒绝测试 2/2、Windows 验证脚本自测 36/36 通过，ESO 兼容入口参数转交检查通过。
实际记录见[配置拒绝检查](out/central-selection-20260930-v1/invalid-configurations/summary.json)；
默认构建没有注册候选专项，各候选只注册自己对应的专项。

中途 WSL 再次重启，ESO 与 PREALLOC 构建日志已保留为 `.interrupted-1`，使用持久缓存续跑后
取得上述结果；[第二次中断记录](out/central-selection-20260930-v1/interruption-2.json)不归因于算法。
构建中出现过约 0.05 秒的生成依赖文件时间戳偏差警告，构建、专项、全量测试及清单实际均成功。
算法源码、CMake、验证脚本与运行配置仍与冻结快照一致；并行 Docker 任务的 `compose.yaml`
增加 dev 容器 `/tmp` 可执行挂载，该部署变动保留，不由本 WSL 矩阵验收。

视觉/队列链全部关闭并切回 OpenVINO OFF 后也通过 **105/105**，见
[最后回退摘要](out/central-selection-20260930-v1/incremental/vision-runtime/DEFAULT/summary.json)。
[最终验收汇总](out/central-selection-20260930-v1/acceptance_summary.json)核对了九次配置的实际开关、
测试注册数和源码未变；[当前源码核对](out/central-selection-20260930-v1/current_source_consistency.json)
记录文档及并行 Compose 变动，算法源码、CMake、脚本与运行配置均匹配已测快照。
目标 NUC 的吞吐/延迟、真实精度与候选组合继续单列待实测。

<a id="remaining-alternatives-20260930"></a>

### 剩余方案同接口候选（2026-09-30）

以下保留当时的块注释阶段记录；当前启用入口已迁移至上述 CMake 集中选择。

本轮按严格同接口要求新增 I1-CONTRAST-IRLS、I3-LINEAR-CA 和 I9-PREALLOC 完整注释候选。
原算法、公共类型、配置与 CMake 注册保持不变，前轮候选和开工时所有未提交修改保留；
没有暂存、提交或重置 Git。候选仅在独立副本解除注释，主工作区始终关闭。
开工快照及去注释后的有效代码对照见
[验收目录](out/remaining-alternatives-20260930-v1)与
[默认代码对照](out/remaining-alternatives-20260930-v1/default_code_comparison.json)。

I1 保持中心线端点语义，使用对比度加权 Huber IRLS；I3 复用平移 CA，实验白 jerk PSD 为
`1.0 m²/s⁵`；I9 仅预分配 pending 容器，保留原有锁、积压和租约语义。
其它 I0–I10/A–C 的缺口、后续步骤和验证入口见
[详细替换手册](additional_information.md#algorithm-alternatives)，快速命令置于手册首页。

验证使用冻结的 [源码](out/remaining-alternatives-20260930-v1/acceptance-source)和
[摘要](out/remaining-alternatives-20260930-v1/acceptance_inventory.json)。WSL 新构建目录位于
本任务获准的 C 盘持久工作目录 `remaining-alternatives-acceptance`；最多两个构建并行，
各 `--parallel 2`。配置为 C++17 Debug / 海康 OFF / sanitizers OFF；I9 使用 OpenVINO ON
和已有 YOLOv5、YOLO11 XML/BIN，其余使用 OFF。Windows 与 WSL 验证脚本自测各 62 项通过，
包含既有 I2/THROUGHPUT/ESO 激活方式和失败隔离。专项只在候选副本注册。

| 验证配置 | 实际结果 | 记录 |
| --- | --- | --- |
| 默认版本 / OpenVINO OFF | 103/103 | [摘要](out/remaining-alternatives-20260930-v1/default-off/summary.json)、[全量日志](out/remaining-alternatives-20260930-v1/default-off/ctest.log) |
| I1-CONTRAST-IRLS / OFF | 首次 103/104，修正后 104/104 | [首次日志](out/remaining-alternatives-20260930-v1/i1-contrast-off/ctest.log)、[续跑摘要](out/remaining-alternatives-20260930-v1/i1-contrast-off/retry-resume/summary.json) |
| I3-LINEAR-CA / OFF | 104/104，含新增专项 | [摘要](out/remaining-alternatives-20260930-v1/i3-linear-off/summary.json)、[全量日志](out/remaining-alternatives-20260930-v1/i3-linear-off/ctest.log) |
| I9-PREALLOC / ON / 双模型 | 111/111，含新增队列基线对照及七项模型条件测试 | [摘要](out/remaining-alternatives-20260930-v1/i9-prealloc-on/summary.json)、[全量日志](out/remaining-alternatives-20260930-v1/i9-prealloc-on/ctest.log) |

I1 专项提取原函数作为对照；I9 从原类声明和原方法区生成测试专用 BaselineFrameQueue，
不从候选反造基线。该冻结版本三个新增候选均可替换且已验证（软件）；
后续集中入口版本另见本页新记录，不挪用此处结果。
目标 NUC 性能、真实端点精度、设备效果和候选组合仍待各自验收。

I1 首次专项通过，但 evaluation-golden 的精确 RMS 比较失败：原值
`2.0466006582283107 px`，候选为 `2.0466161579665623 px`，P/R 与计数一致。
夹具通过阈值的像素均为同一红色，新增等权重算造成数值变化。
修正将像素筛选保持原 float 边界，并在对比度全等时保留原 Huber/端点结果；
非等权仍使用 IRLS，同时新增倾斜等权的精确基线比较。不修改 golden 或放宽原断言。
修正后的首次增量构建因 WSL 重启中断，未形成测试摘要；保留
[中断记录](out/remaining-alternatives-20260930-v1/i1-contrast-off/retry/interruption.json)。
源码和构建目录位于持久目录，续跑单独记录，不把中断当作算法通过或编译错误。

构建启动后，工作区出现本轮之外的配置快照接口并行修改。已保留这些内容；
上述测试只证明冻结快照与注明修正的候选，不代表后续合并工作区已整体复验。

<a id="algorithm-alternatives-20260930"></a>

### 算法注释候选与快速替换（2026-09-30）

保留开工时已有的 ESO、12 维状态、配置和构建修改，没有暂存、提交或重置 Git。
本轮仅在 PnP、OpenVINO 的原实现旁增加完整注释候选与原实现边界标记；
默认仍为原 IPPE、`LATENCY` 与 EKF，没有新运行配置或公共类型。
默认有效代码与开工快照一致，见
[对照记录](out/algorithm-alternatives-20260930-v1/default_code_comparison.json)。

I2-LM 保留 IPPE 双初值与等价板系，以 20 次 / `1e-6` 的 LM 做角点重投影精修，
优化失效或误差上升保留原候选，双初值合并时保留原组。
I9-THROUGHPUT 只改变 OpenVINO 性能提示。既有 ESO 未改写。
替换、回退和命令见[手册首页快速替换](additional_information.md#quick-algorithm-swap)，
全部提案审核见 [possible_method.md](possible_method.md)。

验证使用本轮冻结的
[源码快照](out/algorithm-alternatives-20260930-v1/acceptance-source-v2) 与
[文件摘要](out/algorithm-alternatives-20260930-v1/acceptance_inventory_v2.json)，
分别在独立副本启用候选。验证脚本仅在 I2 副本提取原函数对照并注册专项；
I9 必须提供 YOLOv5 / YOLO11 两组 XML/BIN 并确认七项模型测试已注册。
脚本标记、隔离与失败原子性自测 15/15 通过。

工具链为 WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / OpenCV 4.5.4；
所有配置使用 C++17 Debug，海康与 sanitizers 均 OFF，ON 配置使用 OpenVINO 2026.3.1。

首轮四个 WSL 构建在编译中意外中断，退出码均为 1，`/tmp` 源码和构建目录随后消失，
没有最终测试摘要，不能据编译进度认定通过。现场 `uptime` 表明 WSL 已重新启动；
没有足够证据确定重启原因。首轮 console、命令和中断记录保留在
[验收目录](out/algorithm-alternatives-20260930-v1)。重试降低并行度，使用获准 C 盘持久目录
`algorithm-acceptance`，不清理已有项目构建或设备数据。

| 本轮验证 | 实际结果 | 记录 |
| --- | --- | --- |
| 默认版本，C++17 Debug / OpenVINO OFF | 103/103 | [摘要](out/algorithm-alternatives-20260930-v1/default-off-v2/summary.json)、[全量日志](out/algorithm-alternatives-20260930-v1/default-off-v2/ctest.log) |
| I2-LM，C++17 Debug / OpenVINO OFF | 104/104，含新增专项 | [修正后摘要](out/algorithm-alternatives-20260930-v1/i2-lm-off-v2/retry/summary.json)、[全量日志](out/algorithm-alternatives-20260930-v1/i2-lm-off-v2/retry/ctest.log) |
| I9-THROUGHPUT，OpenVINO ON / 两组模型 | 110/110，含七项模型条件测试 | [摘要](out/algorithm-alternatives-20260930-v1/i9-throughput-on-v2/summary.json)、[全量日志](out/algorithm-alternatives-20260930-v1/i9-throughput-on-v2/ctest.log) |
| ESO 独立副本，OpenVINO OFF | 104/104，含现有 ESO 专项 | [摘要](out/algorithm-alternatives-20260930-v1/eso-off-v2/summary.json)、[全量日志](out/algorithm-alternatives-20260930-v1/eso-off-v2/ctest.log) |

I2 首轮 103/104，新增共线用例错误地要求底层 `ippe_candidates` 一定返回空。
现有接口将几何退化门控放在 `solve_pose`；把该断言移到对应接口后增量构建，104/104 通过。
没有修改候选算法、生产门限或原有回归断言；改善、最大单角误差恶化回退、双初值合并回退
均通过。首轮失败[日志](out/algorithm-alternatives-20260930-v1/i2-lm-off-v2/ctest.log)保留。
该专项只在 I2 副本注册，其他配置没有编译它；冻结快照与最终专项的差异另存于重试记录。

本轮不作目标 NUC 性能、真实端点/尺寸、实机效果或硬件上线结论；
各候选单独验证，不代表组合启用已验收。
默认源码与已有未提交修改的保留情况见
[范围与完整性记录](out/algorithm-alternatives-20260930-v1/scope_and_integrity.json)。

<a id="eso-compat-20260930"></a>

### ESO 兼容接口与注释实现（2026-09-30）

基线提交 `a255825b59d3b9510e9c9d0aef14763131f0a0ca`。开始时已有的
`CMakeLists.txt` 版本号修改完整保留，本轮未改根构建文件，未暂存、提交或重置 Git。
GPT-6 Astra / Ultra 子智能体分别负责 ESO 实现、独立验证及只读交叉复核。

公共状态扩展为 `x,y,z,vx,vy,vz,phase,omega,alpha,ax,ay,az`；默认 EKF 保持平移 CV，
新增失活加速度状态与协方差行列清零。运动模型增加平移 CA、完整白 jerk 过程噪声及
快照外推；配置兼容原 9 项初始方差并追加三个 1，自带三份配置显式使用 12 项。
Tracker 与关联通过 `StateEstimator` 选择算法，默认仍为 `Ekf`，专项 EKF 测试保持直接引用。
共用创新/更新报告的字段与原接口一致。

`eso.hpp` 的参数、包含和完整仅头文件实现均在唯一指定 `/* ... */` 块内。
独立副本解除注释、改别名并注册 `test_eso`，主工作区始终未启用 ESO。
实现包含离散极点增益、同曝光多板信息累计、批先验 NIS、匹配实际增益的 Joseph 协方差，
以及拒绝/失败不改变已有融合结果的语义。实验参数和限制见
[手册 §5.6](additional_information.md#experimental-eso)，不作为设备调参结论。

本次工具链为 WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / C++17 Debug，
海康、ASan/UBSan 和 TSan 均 OFF；ON 使用 OpenVINO 2026.3.1 与现有两份历史模型。

| 验收 | 实际结果 | 记录 |
| --- | --- | --- |
| 修改前独立 OFF 基线 | 103/103 | [全量日志](out/eso-compat-20260930/baseline-off/2.log) |
| 默认 EKF，OpenVINO OFF | 103/103 | [全量日志](out/eso-compat-20260930/default-off/ctest.log) |
| 默认 EKF，OpenVINO ON，YOLOv5 + YOLO11 | 110/110 | [全量日志](out/eso-compat-20260930/default-on/retry3-ctest.log) |
| 启用 ESO 的独立 OFF 副本 | 104/104，含新增专项 | [最终全量日志](out/eso-compat-20260930/eso-off-v2/integrity-supplement/ctest_all.log)、[完整性补验](out/eso-compat-20260930/eso-off-v2/integrity-supplement/summary.json) |
| 三个新增公共头独立包含 | 3/3，默认别名为 EKF，块注释无嵌套 | [编译与结构检查](out/eso-compat-20260930/headers/compile-results.json) |
| 验证脚本隔离及标记检查 | 11/11 | [脚本自检](out/eso-compat-20260930/eso-off-v2/script_selftest.json) |
| 固定 60 帧命令兼容性 | 修改前、修改后旧 9 项配置、修改后新 12 项配置的命令 TSV 与 UART 十六进制逐字节一致 | [比较结果](out/eso-compat-20260930/command-compatibility/comparison.json) |

ESO 专项覆盖接口与复制、二/三阶离散极点、恒速度/恒加速度、角 CV/CA 与角度跨界、
变步长与丢帧、零间隔初始化、多板联合协方差、退化量测与拒绝原子性、模型切换及未来外推。
补充断言确认：已经校正的 ESO 在零时间切换模型后拒绝再次更新，正时间预测后恢复。
没有更改 golden 或放宽既有门限。默认 OFF/ON 的生产来源摘要均为
`7cd9b90d745b8a9f1cc4671d846e9742a9ed578659a60b68c9a8f36dd23067bd`；
最终测试内容摘要均为 `02e638ac955913438b7bc9ba7c63e0d0c6baea6e113435b244744052f2fd5c74`。
OFF 通过后仅补充未注册的 ESO 测试断言，已刷新其验收清单并保留刷新前版本。

环境与取证异常单独保留：ON 初次构建出现链接 I/O 错误，随后两次增量构建因 E 盘空间
不足退出；将本轮新建的基线及默认 OFF 完整构建产物移至获准 C 盘工作目录后，最终
增量构建、110 项测试及来源校验均退出 0。没有清理项目原有目录或删除产物；迁移位置见
[基线记录](out/eso-compat-20260930/baseline-off/relocation.json)与
[OFF 记录](out/eso-compat-20260930/default-off/relocation.json)，日志和清单保留在原验收位置。
各次 ON 命令与失败日志仍保留，最终命令见
[retry3-commands.json](out/eso-compat-20260930/default-on/retry3-commands.json)。

首轮 ESO 的工具输出显示 104/104，但随后 `/tmp` 材料消失，未将其作为最终验收依据。
使用最终断言版本重新构建并在同一 WSL 进程结束前归档。该轮构建和测试均成功，但
外部工作同期新增 `possible_method.md`，导致脚本源码不变检查按设计返回失败；原失败
摘要未覆盖，该文档未被本线程修改或删除。仅同步这份文档后补验，主源与副本在补验期间
均未变化，副本恰好只有 ESO 解除注释、选择别名、实验测试注册三项预期差异，104 项再次
通过，归档摘要逐项匹配。

最终文件清单、包含新增文件的差异、配置、命令退出码和模型摘要见本机
[验收汇总](out/eso-compat-20260930/final/acceptance-summary.json)。这些本机材料不是新克隆
仓库的运行依赖。本轮没有 NUC 性能、实机效果、TSan 或硬件联调验收结论。

<a id="cmake-cleanup-20260930"></a>

### 根 CMake 整理（2026-09-30）

基线提交 `4d90163539f08571afc515c0d9f3e97cf7b09a05`，开始时工作区干净。
将 48 处零散 `target_sources` 并入九个模块的显式清单，73 个实现文件的集合与
各目标内顺序不变；普通测试和两个条件异步测试复用 `autoaim_test_target` 创建程序。
普通测试仍直接链接 `autoaim_options`，异步测试保留原直接链接模块，注册条件、
名称、命令及超时不变。全部构建逻辑仍在根 CMake，内嵌检查器未修改。
GPT-6 Astra / Ultra 子智能体完成修改前审阅和修改后只读复核。

复用 `out/slimming-20260930-v2/build-{off,on}` 前，先逐项确认已有生成清单与当前源码
一致，再保存本轮缓存、File API、编译命令和 CTest 基线；本轮重新配置、构建并运行测试。
WSL Ubuntu 22.04 / GCC 11.4 / CMake 3.22.1 / C++17 Debug，海康和 sanitizer 均 OFF。

| 配置 | 本轮结果 | 编译单元 | 构建等价比较 |
| --- | --- | --- | --- |
| OpenVINO OFF | [103/103 通过](out/cmake-cleanup-20260930/after/off/ctest.log) | 172 | [目标属性、编译命令、测试命令与属性一致](out/cmake-cleanup-20260930/after/off/comparison.json) |
| OpenVINO ON，已有两份历史模型 | [110/110 通过](out/cmake-cleanup-20260930/after/on/ctest.log) | 174 | [目标属性、编译命令、测试命令与属性一致](out/cmake-cleanup-20260930/after/on/comparison.json) |
| BUILD_TESTING OFF、OpenVINO OFF | 仅配置比较，未构建或运行测试；20 个目标、0 项测试，未生成检查器 | 84 | [目标属性与编译命令一致](out/cmake-cleanup-20260930/after/no-tests/comparison.json) |

目标比较只去除 CMake File API 的诊断回溯字段，保留实际源码顺序、依赖、编译与链接属性。
初次比较遗漏了 `languageStandard.backtraces`，因函数层级改变而失败；核对本机 CMake
手册确认其为回溯索引后，将其纳入位置字段处理，两套比较通过，原始 File API 副本保留。
生产来源清单仅 `CMakeLists.txt` 的摘要变化；重建程序通过固定样例输出的
[实际来源标识](out/cmake-cleanup-20260930/report-source-check.json)与新生成清单一致。
增量构建日志含 Windows 挂载上约 0.02 秒的文件时间偏差警告，构建退出码和全量测试均为 0。

README 和手册同步集中清单及测试 helper 的维护方式。本轮未改运行代码、默认选项、
依赖权限、模型或测试预期，不作构建耗时收益结论。命令及退出码、缓存、前后图与
验收清单保存在 `out/cmake-cleanup-20260930/`，未暂存、提交或重置 Git。

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
