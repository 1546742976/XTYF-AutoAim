# XTYF-AutoAim

面向 RoboMaster 的 C++ 装甲板视觉项目。它把“图片里的装甲板”逐步转换成“目标在哪里、接下来往哪里运动、应当给出什么控制请求”，再经过许可检查输出命令。

**当前可用的是离线软件链路，不是接上设备就能运行的整机程序。** 可以生成测试图片、回放图片与姿态、检查检测/跟踪/预测结果、保存命令和评测报告。相机与通信后端已有代码，但公共运行入口仍拒绝硬件模式；打符只保留文件，尚未实现。

本页负责入门、简明文件导航和使用方法；九模块逐文件说明、算法约束、数据契约和工程约定见
[additional_information.md](additional_information.md)；按阶段保存的构建日志、测试矩阵和修复记录见
[build_history.md](build_history.md)。

## 阅读导航

- [快速替换算法：精修 / LM / 平移 CA / 推理提示 / 队列预分配 / ESO](additional_information.md#quick-algorithm-swap)
- [先理解处理流程](#1-先理解处理流程)
- [编译并跑通第一个示例](#2-编译并跑通第一个示例)
- [浏览器调参与离线实验工作台](workbench/README.md)
- [目录和文件怎么看](#3-目录和文件怎么看)
- [九模块速查与详细手册入口](#4-九个模块逐个说明)
- [想改某项功能该找哪里](#5-想改某项功能该找哪里)
- [配置与常用工具](#6-配置与常用工具)
- [控制模式与通信接口](#7-控制模式与通信接口)
- [测试、已验证范围与待完成项](#8-测试已验证范围与待完成项)
- [构建日志与历史验证记录](build_history.md)

## 1. 先理解处理流程

一次离线处理大致经过以下步骤；`pipeline` 负责把这些模块连接起来：

```text
本地图片 + 姿态/操作事件（hal）
  → 对齐曝光时刻的姿态、管理图像队列（pipeline）
  → 找装甲板四角、解算三维位置与朝向（vision）
  → 判断是否为同一目标，估计位置和运动状态（estimation）
  → 预测未来位置，求飞行时间并选择装甲板（decision）
  → 根据步兵/哨兵和操作手输入提出控制请求（mission）
  → 检查时效、权限与故障，形成最终命令（control）
  → 唯一发布线程写出 TSV / UART 十六进制记录
```

几个后文会反复出现的词：

| 名称 | 可以怎样理解 |
| --- | --- |
| 检测 | 在二维图片里找装甲板、颜色、类别和角点；还没有三维位置 |
| PnP / IPPE | 根据角点、板的真实尺寸和相机标定，计算板相对相机的位置与朝向；平面板可能有多个解 |
| EKF | 扩展卡尔曼滤波器：结合上一时刻的运动估计和这一帧的观测，降低噪声并更新状态 |
| profile | 一份几何描述，记录一辆目标的板数、板偏移、朝向和旋转轴，不是另一套算法 |
| 快照 | 一份不能被下游改写的目标状态；预测使用它，不把未来预测写回跟踪器 |
| 世代号 generation | 模式切换等操作后的“版本号”，用于拒绝旧模式留下的结果和指令 |
| 请求与许可 | 上游可以请求控制/开火，但只有控制层检查通过后才形成最终许可 |

## 2. 编译并跑通第一个示例

不想在宿主安装 C++ 依赖时，可用 [§2.4 Docker 环境](#24-docker-环境)；
下面的本机构建方式继续保留。容器验收状态见 [构建历史](build_history.md#docker-20260930)。

以下命令在 **Ubuntu / WSL 的 Bash 终端、仓库根目录**执行，不是在 Windows PowerShell 中直接执行。默认 C++17，可通过 CMake 选择 C++20。

### 2.1 准备依赖

依赖用途：OpenCV 处理图片与标定，Eigen 做矩阵计算，yaml-cpp 读取配置，OpenVINO 运行 YOLO；CMake 负责构建，Python 3 用于部分测试。当前项目不要求 ROS2。

[install_dependence.sh](install_dependence.sh) 是人工安装入口。先查看计划，再决定是否安装：

```bash
# 只预览，不联网、不安装；跳过相机 SDK
bash install_dependence.sh --dry-run --skip-camera

# 确认后由你执行：安装软件依赖和 OpenVINO，跳过相机 SDK
bash install_dependence.sh --yes --skip-camera
```

脚本声明支持 Ubuntu 22.04/24.04 amd64；本项目已验证环境是 Ubuntu 22.04。默认 OpenVINO 版本为脚本中的 2026.3.1，安装脚本可能升级 CMake；项目自身的最低 CMake 版本是 3.22。脚本不会自动构建或启动设备。

相机 SDK 默认使用项目根目录现有的 `MVS-5.1.0_Linux_x86_64_20260909.zip`，不再要求填写示例下载包路径。
该压缩包通过 Git LFS 保存；克隆仓库前请安装 Git LFS 并运行 `git lfs install`。
已克隆但未取得完整压缩包时，在仓库根目录运行 `git lfs pull`。
该 ZIP 内含 `MVS-5.1.0_x86_64_20260909.deb` 和同版本
`.tar.gz`；脚本只按 DEB 元数据选择唯一 amd64 MVS 包，不执行 TAR 中的安装脚本。
实查 DEB 的包名为 `mvs`、架构为 `amd64`、`Version` 字段为 `2025-07-11`，
与文件名的 `5.1.0` 标识不同；脚本如实显示元数据，不据文件名判断版本或包来源。
DEB 内嵌 SDK 归档，头文件/库安装路径仍为 `/opt/MVS/include`、`/opt/MVS/lib/64`；
库文件名为 `libMvCameraControl.so.4.8.2.1`，不能把这几个版本标识混为一谈。
未指定包时先复用已安装 SDK，否则按**脚本所在目录**查找这份 ZIP，不依赖当前工作目录。
`--mvs-deb`、`--mvs-archive`、`--mvs-url` 可显式覆盖；`--skip-camera` 完全跳过相机包。
先检查本地包，再由你决定是否安装：

```bash
bash install_dependence.sh --check-camera-only \
  --mvs-archive ./MVS-5.1.0_Linux_x86_64_20260909.zip
bash install_dependence.sh --dry-run
# 确认来源和许可后由你执行；无完整 SDK 时自动选择上述本地 ZIP
bash install_dependence.sh
```

本地 DEB 使用 `--mvs-deb FILE`；其它选项见 `bash install_dependence.sh --help`。检查成功只说明包结构等检查通过，不证明相机可用；安装会运行厂商包维护脚本，需先核实来源和许可。仅做传统检测时可用 `--skip-openvino`，构建时也必须关闭 OpenVINO。

这份 DEB 的 `postinst` 会执行厂商 `setup.sh`，包括替换 `/opt/MVS`、配置 USB/虚拟串口
权限及驱动/日志服务自启动等操作；`--yes` 不会跳过它们。`--check-camera-only` 和
`--dry-run` 均不执行厂商脚本。本次只做结构检查与无安装测试，未实际安装驱动。

### 2.2 构建

```bash
# OpenVINO_DIR 换成你的实际安装目录；已能自动找到时可省略该参数
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug \
  -DAUTOAIM_OPENVINO=ON \
  -DOpenVINO_DIR=/usr/lib/cmake/openvino2026.3.1
cmake --build build-debug --parallel 2
ctest --test-dir build-debug --output-on-failure
```

没有 OpenVINO、暂时只想跑合成示例：把上面的配置命令换成
`cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -DAUTOAIM_OPENVINO=OFF`。
此时仍可运行传统检测，但选择 YOLO 会明确报错，不会自动换算法。

`-S .` 指当前源码目录，`-B build-debug` 指生成文件存放目录；可执行程序也在该目录。
Release 构建将目录改为 `build-release`、构建类型改为 `Release`。
需要 C++20 时增加 `-DCMAKE_CXX_STANDARD=20`，并使用独立的 `build-cxx20` 目录。

CMake 配置阶段会将完整 `config/` 复制到构建目录，保留 `fast_choose.yaml`、
`offline/` 和 `hardware/` 的目录结构；不必等到编译完成。进入构建目录后也可运行：

```bash
cd build-debug
./autoaim_node --config config/fast_choose.yaml
```

模型、输入数据等相对路径以各 YAML 所在目录为基准；使用这份副本时，
`fast_choose.yaml` 中的 `../models/` 和 `../out/` 分别指向构建目录下的 `models/` 和 `out/`。
日常推荐修改源码目录的配置；再次运行 CMake 时会按复制规则更新副本，普通编译不会主动同步。
继续从仓库根运行 `build-debug/autoaim_node --config config/fast_choose.yaml` 则使用源码配置。

| CMake 选项 | 默认值与用途 |
| --- | --- |
| `BUILD_TESTING` | ON；构建测试并注册到 CTest |
| `AUTOAIM_OPENVINO` | ON；编译 YOLOv5/YOLO11 推理后端 |
| `AUTOAIM_I1_CONTRAST_IRLS` | OFF；角点中心轴对比度精修 |
| `AUTOAIM_I2_LM` | OFF；IPPE 候选角点重投影 LM 精修 |
| `AUTOAIM_I3_LINEAR_CA` | OFF；EKF 平移恒加速度对照，与 ESO 互斥 |
| `AUTOAIM_I9_THROUGHPUT` | OFF；OpenVINO THROUGHPUT 提示，要求 OpenVINO ON |
| `AUTOAIM_I9_PREALLOC` | OFF；pending 容器预分配，保留队列和租约语义 |
| `AUTOAIM_USE_ESO` | OFF；选择 ESO 并注册对应专项，参数仍来自快调文件 |
| `AUTOAIM_TEST_YOLOV5_MODEL`、`AUTOAIM_TEST_YOLO11_MODEL` | 空；提供模型 XML 的绝对路径后增加真实权重 CPU 冒烟测试，配套 BIN 与 XML 相邻 |
| `AUTOAIM_SANITIZERS` | OFF；开启 GCC/Clang 的地址与未定义行为检查 |
| `AUTOAIM_HIKROBOT` | OFF；编译海康后端，不开放硬件运行入口 |
| `AUTOAIM_HIKROBOT_INCLUDE`、`AUTOAIM_HIKROBOT_LIBRARY` | 开启海康后端时显式提供 SDK 头目录和库文件 |

修改根 CMake 的 `option()` 默认值只影响新构建目录；已有目录必须用 `-D选项=ON/OFF`
覆盖缓存，或先清除该缓存项。算法验证脚本默认 Debug，可用 `--build-type Release` 选择 Release。

### 2.3 不需要相机或模型的完整示例

统一入口 `config/fast_choose.yaml` 默认选择 YOLOv5，需要 OpenVINO 和对应模型。
下面的无模型示例先将其中 `active_detector` 改为 `traditional`；也可在三条命令中
使用固定传统入口 `--config config/offline/armor.yaml`。调参后重启生效，不需要重新编译。
输出目录/文件必须是新路径；重复执行时换一组名字，不覆盖已有结果。

```bash
# 1. 生成 30 帧灯条图片、事件和真值
build-debug/synthetic_sim --config config/fast_choose.yaml \
  --output out/quickstart-data --frames 30

# 2. 走完整处理链，输出命令、UART14 字节记录和录制会话
build-debug/offline_replay --config config/fast_choose.yaml \
  --input out/quickstart-data/events.yaml \
  --output out/quickstart-data/commands.tsv \
  --uart-output out/quickstart-data/uart.hex \
  --record-session out/quickstart-recording

# 3. 在图片上画出检测和位姿结果，不弹出相机窗口
build-debug/replay_visualizer --config config/fast_choose.yaml \
  --input out/quickstart-data/events.yaml --output out/quickstart-view

# 4. 统计命令记录中的源数据年龄
build-debug/pipeline_metrics out/quickstart-data/commands.tsv --fire-age-s 0.08
```

运行后查看：

| 输出 | 内容 |
| --- | --- |
| `out/quickstart-data/*.png`、`events.yaml`、`truth.tsv` | 输入图片、按时间排列的事件、独立几何真值 |
| `commands.tsv` | 最终命令及来源时间，可用文本编辑器或表格工具查看 |
| `uart.hex` | 离线 UART14 编码，不会打开串口 |
| `out/quickstart-recording/` | PNG、原始事件清单及独立的派生记录 `derived.yaml` |
| `out/quickstart-view/*.png` | 灰色检测候选、橙色有效但未可靠的位姿、绿色可靠位姿 |

示例缺少真实设备证据，最终 `control=0、shoot=0` 是预期结果，不代表程序没有处理图片。
`NA/null` 的首次有效控制写入指标表示没有这样的写入，不是零延迟。合成图只用于测试流程，不能衡量真实识别精度。

### 2.4 Docker 环境

先确认 `docker info` 能连接 **Linux/amd64** 引擎；Docker 客户端存在不代表引擎可用。
Windows 可直接使用 PowerShell 中的 Docker Desktop。WSL Bash 还需要已有的 Docker 集成，
不能把 Windows 端可用当成 WSL 端也已配置。本项目不会自动启动、安装或重配 Docker。

[Dockerfile](Dockerfile) 固定 Ubuntu 22.04 镜像摘要，默认 OpenVINO 2026.3.1、CPU。
首次构建联网下载官方依赖，只安装到镜像；启动容器不安装依赖，也不自动启动程序。
开发依赖复用 [安装脚本](install_dependence.sh)，不把宿主现有 `/usr`、旧构建缓存打包进去。
BuildKit 的 APT 下载缓存用于中断后复用已下载包，不进入镜像层；它仍会占用 Docker 存储。

| 构建目标 | 装了什么 / 用来做什么 |
| --- | --- |
| `dev` | 编译器、CMake、Python 与开发依赖；宿主编辑源码，容器编译和测试 |
| `build` | 内部 Release 构建阶段，生成 `Runtime` 安装组件 |
| `runtime` | 12 个程序（含打符诊断占位入口）、示例配置及运行库；不装源码、测试、编译器或 CMake |
| `sdk-check` | 可选海康 SDK 编译检查；默认构建不需要 SDK，不运行设备 |

在仓库根目录构建两个常用镜像（PowerShell / Bash 相同）：

```text
docker compose build dev runtime
```

Compose 默认非 root、断网、只读根文件系统，无设备映射/特权权限；只允许临时目录、
独立编译卷和输出目录写入。直接 `docker run` 时需显式带上相同限制，镜像本身不能强制断网。
源码默认只读挂载，宿主编辑器仍可编辑原文件，容器下一次编译会读到变化。
开发容器的 `/tmp` 允许执行 CTest 的临时程序，仍禁用 suid 和设备文件；
运行容器的 `/tmp` 保持不可执行，不通过放宽运行容器权限来通过开发测试。

首次演示可准备空模型、数据目录和输出目录。在 **PowerShell** 执行：

```powershell
New-Item -ItemType Directory -Force out/docker-models,out/docker-data,out/docker | Out-Null
$env:AUTOAIM_MODELS_DIR = (Resolve-Path out/docker-models -ErrorAction Stop).Path
$env:AUTOAIM_DATA_DIR = (Resolve-Path out/docker-data -ErrorAction Stop).Path
$env:AUTOAIM_OUTPUT_DIR = (Resolve-Path out/docker -ErrorAction Stop).Path
```

在 **Linux / 已集成 Docker 的 WSL Bash** 则执行：

```bash
mkdir -p out/docker-models out/docker-data out/docker
export AUTOAIM_MODELS_DIR="$PWD/out/docker-models"
export AUTOAIM_DATA_DIR="$PWD/out/docker-data"
export AUTOAIM_OUTPUT_DIR="$PWD/out/docker"
export AUTOAIM_UID="$(id -u)" AUTOAIM_GID="$(id -g)"
```

UID/GID 默认 1000，须为非零。改变后重新构建镜像，并使用新的 Compose 项目名创建编译卷，
例如 `docker compose -p xtyf-myuser build dev runtime`；所有后续命令保持相同 `-p`。
已有卷不会自动改属主，不执行递归 chown。输出目录需事先允许该用户写入。

| 宿主输入 | 容器位置 | 权限 |
| --- | --- | --- |
| 仓库根目录（仅 dev） | `/workspace/src` | 只读 |
| Compose 编译卷（仅 dev） | `/workspace/build` | 可写；不复用宿主 CMakeCache |
| `AUTOAIM_MODELS_DIR` | `/models` | 只读；YOLO XML/BIN 放在同一目录 |
| `AUTOAIM_DATA_DIR` | `/data` | 只读；录像转换后的 PNG/YAML 会话 |
| `AUTOAIM_CONFIG_DIR`，默认仓库 `config/` | `/config` | 只读；自有标定和配置可另行指定 |
| `AUTOAIM_OUTPUT_DIR` | `/output` | 可写；容器移除后保留结果 |

挂载源目录必须事先存在。Compose 已设置 `create_host_path: false`，但本机 Docker Desktop
仍曾自动创建不存在的 Windows 源目录；不能仅依靠该选项检查拼写。PowerShell 设置自有路径
时沿用上面的 `Resolve-Path -ErrorAction Stop`，Bash 设置后用 `test -d` 检查目录，再运行
Compose。`--help` 不读取数据，不能用它验证挂载内容是否正确。
配置中的模型/输入路径须使用容器路径，
例如 `/models/yolov5.xml`、`/data/events.yaml`，不填写 `E:\...`，工具不改写原配置。
镜像内合成示例在 `/opt/autoaim/share/autoaim/config/offline/`，不是设备标定。
安装时同时保留其上级 `config/fast_choose.yaml`；外置配置也要保留该引用关系。

进入开发容器后，按下列方式构建（首行在宿主执行，其余三行在容器内执行）：

```bash
docker compose run --rm dev
cmake -S /workspace/src -B /workspace/build/debug -DCMAKE_BUILD_TYPE=Debug
cmake --build /workspace/build/debug --parallel 2
ctest --test-dir /workspace/build/debug --output-on-failure
```

完整容器矩阵有显式脚本，运行较久且占用编译卷；输出名必须未使用。
按 `/models/yolov5.xml`、`/models/yolo11.xml` 查找模型，配套 BIN 必须存在。
无模型时明确记录相应测试未验收，不会下载模型。脚本不启用默认关闭的 CMake 算法候选。

```text
docker compose run --rm dev bash tools/container/verify.sh /output/validation-01
```

空间有限时可在 `run` 后加 `-e KEEP_BUILD_ARTIFACTS=0`：每个配置通过后只对该次新建的
构建目录执行 `clean`，保留缓存、命令和验收清单；再次使用程序需要重建。不清理历史卷或目录。

只运行已打包程序，不挂载源码或编译卷：

```text
docker compose run --rm runtime offline_replay --help
docker compose run --rm runtime annotate_session --self-test
docker compose run --rm runtime synthetic_sim --config /opt/autoaim/share/autoaim/config/offline/armor.yaml --output /output/demo-01 --frames 6
docker compose run --rm runtime bench_detector --dataset /output/demo-01/events.yaml --config /opt/autoaim/share/autoaim/config/offline/armor.yaml --iou 0.5 --output /output/report-01
```

独立的 [运行镜像检查脚本](tools/container/runtime-smoke.sh) 可只读挂载后显式运行；
它检查回放、标注自检、报告重复一致与硬件拒绝，不会作为镜像启动动作。
若 `/models` 有配套 YOLO XML/BIN，还使用 `/config/offline/yolov5.yaml`、`yolo11.yaml`
执行实际 CPU 推理，验证运行库、CPU 插件与 IR 前端；配置须指向相应挂载模型。
缺模型时明确记录该项未验收，传统合成示例不替代 YOLO 检查。

Windows 注意：Docker Desktop 的 Windows 绑定输出目录可能不支持标注工具要求的
原子且不覆盖目录发布，表现为 `Atomic no-overwrite publish failed: Invalid argument`。
这不是标注校验失败。不要绕过工具检查；导出、检查、应用可改用 Linux 命名卷：

```text
docker compose run --name xtyf-annotation-export --volume xtyf-annotation-work:/output runtime annotate_session --export /output/draft-01 --session /data
docker cp xtyf-annotation-export:/output/draft-01 ./out/draft-01
docker rm xtyf-annotation-export
```

这三步分别是工具原子发布、人工导出已完成产物、删除本次停止的容器；第二步不是工具的
原子发布替代实现。命名卷仍保留结果，重复运行须换新输出名，宿主复制目标也须事先不存在。
Linux 用户可继续使用支持该操作的宿主绑定目录。普通回放和评测仍可写 Windows 输出目录。

可选 SDK：只接受用户已取得的 Linux amd64 SDK 目录，作为 BuildKit 命名输入。
将下列路径换成实际目录；`MVS_INCLUDE`、`MVS_LIBRARY` 均相对于该目录。
不执行 SDK 安装程序，默认镜像不包含 SDK，不映射相机/串口。

```text
docker build --platform linux/amd64 --target sdk-check --build-context mvs=/absolute/path/to/sdk --build-arg MVS_INCLUDE=include --build-arg MVS_LIBRARY=lib/amd64/libMvCameraControl.so -t xtyf-autoaim:sdk-check .
```

程序及配置安装也可在已有本机构建中使用，不需要 Docker：

```bash
cmake --install build-release --prefix "$PWD/out/install-release" --component Runtime
```

离线转移已验收镜像（文件名必须未使用）；模型、数据及 Docker 引擎不包含在镜像归档中：

```text
docker image save -o xtyf-autoaim-images.tar xtyf-autoaim:dev xtyf-autoaim:runtime
docker image load -i xtyf-autoaim-images.tar
docker image inspect xtyf-autoaim:dev xtyf-autoaim:runtime
```

镜像内 `/opt/autoaim-environment/` 保存软件包版本，运行程序来源在
`/opt/autoaim/share/autoaim/autoaim_build_information.json`。镜像 ID、实际大小和日志另记入
构建历史。固定摘要和指定 OpenVINO 版本不保证未来 APT 仍提供全部包；缺版本时应修正
来源或另行批准升级，不能静默改用最新版。SDK 编译和离线镜像均不证明设备可用。

### 2.5 浏览器调参与离线实验工作台

新增独立的 [本地工作台](workbench/README.md)，在 Linux / WSL 启动服务后通过浏览器
编辑参数、另存配置包、选择算法预设或自定义组合、执行构建测试及离线回放，并对比报告。
计算仍由现有 C++ 工具完成；实验目录位于源码树外，每次任务固定配置快照。
参数修改对新任务生效，编译方案切换需要独立构建和测试。

页面使用 Vue 3 + TypeScript，服务使用 Python FastAPI；依赖与启动命令见工作台文档。
首版仅覆盖离线实验，不提供实时设备入口。

## 3. 目录和文件怎么看

```text
XTYF-AutoAim/
├── apps/                 各程序的 main 入口，通常只调用公共流程
├── include/autoaim/       对外可用的 .hpp：类型、接口、单位、线程/所有权约定
│   └── 九个模块目录/
├── src/                  .cpp：各接口背后的算法和处理逻辑
│   └── 九个模块目录/
├── config/
│   ├── fast_choose.yaml  日常调参与检测器选择；启动读取，修改后重启生效
│   ├── offline/          合成场景、YOLO、标定数据等离线配置
│   └── hardware/         待填写的设备参数模板；不是可运行设备配置
├── tools/                合成数据、协议、可视化、指标工具的入口
├── tests/                单元测试、契约测试、合成场景及测试数据
├── CMakeLists.txt        全部构建目标、源文件清单、依赖规则和测试注册
├── Dockerfile           开发/构建/运行镜像及可选 SDK 编译目标
├── compose.yaml         只读输入、独立编译卷、可写输出和离线运行约定
├── .dockerignore        限定发送给镜像构建器的文件，不带旧输出和外部资产
├── install_dependence.sh 人工依赖安装脚本
├── .clang-format         C++ 排版约定
├── additional_information.md  九模块文件详解、技术规格、协作约定与实施历史
├── build_history.md      按阶段保存的构建日志、测试矩阵、修复进度与证据入口
├── build-*/              构建生成目录，不应在这里修改源码
└── out/                  回放/评测/验证输出，不纳入 Git
```

例如要理解队列，先读 [queue.hpp](include/autoaim/pipeline/queue.hpp) 看接口，再读
[queue.cpp](src/pipeline/queue.cpp) 看实现，最后看
[test_frame_queue.cpp](tests/contract/test_frame_queue.cpp) 了解满队列、丢帧等情况。

[详细手册的模块表](additional_information.md#module-guide)中列出的 `.cpp` 一般在
`include/autoaim/<模块>/` 有同名 `.hpp`；
没有独立同名头的实现会单独说明。只有头文件不等于“没实现”：小类型和简单方法可以直接定义在头里。
空预留文件则会明确标出。

## 4. 九个模块逐个说明

完整文件说明已迁至 [详细手册的九模块与文件详解](additional_information.md#module-guide)。
九个模块是由 pipeline 组合的静态库，不是九个独立程序或运行时插件。

| 模块 | 主要职责 | 详细说明 |
| --- | --- | --- |
| core | 基础类型、单位、时间、配置、日志和图像内存池 | [M1：文件与职责](additional_information.md#module-core) |
| math | 角度、数值检查及带坐标系的刚体变换 | [M2：文件与职责](additional_information.md#module-math) |
| hal | 图片、时钟、反馈和字节传输的输入输出接口 | [M3：文件与职责](additional_information.md#module-hal) |
| vision | 装甲板检测、角点、PnP、标定与视觉评测 | [M4：文件与职责](additional_information.md#module-vision) |
| estimation | 关联观测、确定板身份、EKF 跟踪并发布不可变快照 | [M5：文件与职责](additional_information.md#module-estimation) |
| decision | 未来外推、弹道拦截、选板和瞄准质量评估 | [M6：文件与职责](additional_information.md#module-decision) |
| mission | 角色与模式、人工接管、目标选择和控制请求 | [M7：文件与职责](additional_information.md#module-mission) |
| control | 最终许可、协议编码、唯一发布线程及写失败处理 | [M8：文件与职责](additional_information.md#module-control) |
| pipeline | 配置装配、姿态同步、队列与整条处理链的启停 | [M9：文件与职责](additional_information.md#module-pipeline) |

当前默认状态估计器仍为 **EKF**。公共状态为 12 维，平移加速度在默认 CV 模型中失活。
实验 ESO 与其它五个同接口候选统一由根 [CMakeLists.txt](CMakeLists.txt) 的六个布尔开关选择，
默认全部 OFF；启用设 ON，回退显式设 OFF，再重新配置、编译和测试。
无需修改 cpp/hpp 注释、函数或 `StateEstimator` 别名。ESO 仅支持当前完整位姿观测，
参数与批量融合约定见[手册](additional_information.md#experimental-eso)。

可先在 WSL/Linux 的独立副本验证，源文件保持不变：

```bash
# 输出目录必须尚不存在，且位于项目目录之外。
python3 tests/verify_alternatives.py --source . --method ESO --output /tmp/autoaim-eso-check --jobs 2
```

`verify_eso.py` 保留兼容入口；统一脚本显式关闭全部六项，再选择单项候选，由 CMake 注册专项。
角点 I1-CONTRAST-IRLS、PnP I2-LM、EKF 平移 I3-LINEAR-CA、OpenVINO I9-THROUGHPUT、
队列 I9-PREALLOC 和 ESO 的全部开关、回退与独立构建命令见
[补充手册最前面的快速替换表](additional_information.md#quick-algorithm-swap)。
THROUGHPUT 与 OpenVINO OFF、I3-LINEAR-CA 与 ESO 两组无效组合在配置阶段被拒绝；
其它组合仍需独立验证。构建报告保存六个开关的实际值。
当前集中入口的实际验收状态见[构建历史](build_history.md#central-selection-20260930)，
此前手动注释版本的通过记录不代替本次验证；[方案审核](possible_method.md) 保留全部尚缺接口、
数据或硬件的部分。`config/fast_choose.yaml` 的运行参数和 Docker 使用方式保持各自职责。

## 5. 想改某项功能该找哪里

先改配置能表达的参数；需要改算法时再进入对应实现，同时补相应测试。

| 你的目的 | 先看配置/接口 | 主要实现 / 测试线索 |
| --- | --- | --- |
| 换 YOLO 模型、阈值或敌方颜色 | detector、corners 配置；detector.hpp | detector_openvino.cpp、detection_labels.cpp；test_yolov5 / test_yolo11 |
| 调灯条或角点精修 | detector、refinement 配置 | detector_traditional.cpp、corner_refine.cpp；test_traditional_detector / test_corner_refine |
| 距离/方向不对、大小板用错 | calibration.yaml、plate_sizes、角点顺序 | vision/pnp.cpp、pipeline/bootstrap.cpp；test_pnp / test_pose_quality |
| 改板数、三种高度布局或倾斜轴 | geometry.yaml | estimation/geometry_model.cpp；test_geometry / test_geometry_selector_synthetic |
| 跟踪跳目标、运动估计不稳 | tracker、motion、targets 配置 | association.cpp、tracker.cpp、motion_model.cpp；test_tracker_set / test_motion_selection |
| 改目标锁定或选板策略 | targets、armor 配置 | mission/target_selector.cpp、decision/armor_selector.cpp |
| 切换辅助/自动、改按键方式 | operator_input 配置 | mission/button_policy.cpp、infantry_mission.cpp；test_button_mode / test_manual_takeover |
| 图像积压、乱序或内存生命周期 | queue 配置 | pipeline/queue.cpp、core/buffer_pool.cpp；test_frame_queue / test_out_of_order_completion |
| 理解为什么 control/shoot 为 0 | 命令来源、证据及 inhibit 原因，不先调低门限 | control/command_guard.cpp、publisher.cpp；时效/模式/发布故障契约测试 |
| 改协议或反馈坐标映射 | command.hpp、uart-mapping.yaml | control/protocol.cpp、pipeline/uart_adapter.cpp、uart_writer.cpp |
| 加源文件或测试 | CMakeLists.txt 的显式清单 | 对应 autoaim_* target / autoaim_test；不要修改 build 中生成的文件 |

使用编辑器全局搜索上述文件名或测试名即可定位。外部图形化编辑器可以直接打开仓库：
例如 Windows 编辑文件、WSL 编译；语义补全可使用构建生成的 `compile_commands.json`。
不要在 Windows 和 WSL 之间混用同一个构建目录的工具链。

新增模块实现时，将文件加入对应 `autoaim_module(...)` 的集中清单。
普通测试用 `autoaim_test(...)` 创建并注册；需要按模型条件注册的测试程序用
`autoaim_test_target(...)` 创建，再由各条件分支注册名称、参数和超时。

如需在其它 CMake 项目中复用，可通过 `add_subdirectory` 引入后链接 `autoaim_vision` 等目标；
这些目标带有向下的传递依赖，不是“只复制一个 cpp 就能用”。当前没有独立安装/导出的 SDK 包，
也没有保证任意单模块脱离根 CMake 配置；只构建某个目标仍需满足配置阶段的依赖查找。

## 6. 配置与常用工具

### 6.1 配置文件

日常只改 [fast_choose.yaml](config/fast_choose.yaml)：`active_detector` 选择
`traditional/yolov5/yolo11`（默认 `yolov5`），`common` 保存公共参数，`detectors` 保存各自模型路径与阈值，
`eso` 保存编译启用 ESO 后使用的实验参数。每项的单位、范围与作用直接写在配置注释中。
启动示例：`build-debug/autoaim_node --config config/fast_choose.yaml`。

原三份入口保留模型与物理契约；直接用 `--config config/offline/yolo11.yaml` 等旧命令时，
检测器仍固定为该入口类型，忽略 `active_detector`，但调参值仍取自同一快调文件。
不要在两处重复定义迁移参数；装载会明确报错。独立的旧完整配置无需迁移即可继续读取。
完整加载和来源规则见[手册快速调参](additional_information.md#fast-choose)。
本轮修改与实际回归结果见[快调构建记录](build_history.md#fast-choose-20260930)。

| 文件 | 用途与是否能直接作为运行配置 |
| --- | --- |
| [fast_choose.yaml](config/fast_choose.yaml) | 统一离线入口、检测器选择及日常参数；修改后重启生效 |
| [armor.yaml](config/offline/armor.yaml) | 传统检测基础契约及固定检测器入口，引用快调参数；不是实机参数 |
| [yolov5.yaml](config/offline/yolov5.yaml)、[yolo11.yaml](config/offline/yolo11.yaml) | YOLO 基础契约及固定检测器入口；模型路径在快调文件中配置 |
| [calibration.yaml](config/offline/calibration.yaml) | 640×480 合成相机内外参；不是设备实测标定 |
| [geometry.yaml](config/offline/geometry.yaml) | 合成四板等高竖直轴 profile；其它布局由同一结构表达 |
| [corners.yaml](config/offline/corners.yaml) | 旧 class_id 格式的角点样例；配合当前 armor.yaml 前需补 plate_type，不能直接当作现成可运行样例 |
| [calibration-dataset.yaml](config/offline/calibration-dataset.yaml) | 标定数据清单模板；null 和空 samples 必须补齐，不能直接求解 |
| [camera.yaml](config/hardware/camera.yaml) | 海康彩色 MV-CS016-10UC 参数契约模板，不是入口配置；150fps 是目标 |
| [uart-mapping.yaml](config/hardware/uart-mapping.yaml) | 反馈轴向/符号及按键输入配置片段，不是完整运行配置；无配置继承/自动合并功能 |
| [disabled.yaml](config/hardware/disabled.yaml) | 明确拒绝设备模式的示例，运行失败是预期行为 |

配置内引用文件相对该配置所在目录；事件图片路径相对事件清单；
CLI 的 `--input/--output` 相对当前工作目录。复制配置到别的目录时，需要同步修正相对路径。

YOLO 的 XML/BIN 权重由使用者配套提供。历史模型测试路径可从对应构建目录的 `CMakeCache.txt`
中查找 `AUTOAIM_TEST_YOLOV5_MODEL`、`AUTOAIM_TEST_YOLO11_MODEL`；缓存中的路径不代表拟部署版本。
更换模型需分别核对类别字典、预处理约定和关键点标注定义，核验边界见[手册 §5.1](additional_information.md)。

单位统一为：距离 m、角度 rad、时长 s、时间戳 ns，四元数顺序 w,x,y,z。
相机系为 x右/y下/z前，板系为 x物理右/y物理上/z外法向；云台/世界轴向须明确配置。
`small/big` 板型决定 PnP 使用的物理尺寸，未知板型保留二维检测，不猜尺寸。

YOLOv5 模板的 `detector.plate_type_by_class` 默认为空，必须按实际模型语义填写才能为相应类别选尺寸。
YOLO11 使用自己的 38 类映射，不能套用 YOLOv5 的裸整数。v5 的 `corners.indices` 作用于旧兼容预排
`[0,3,2,1]` 之后，v11 保留原角点顺序；两者都不能仅靠图像坐标排序证明物理四角可靠。

当前 YAML 装配使用 `Evidence::declared()` 作为角点映射证据，因此不授予角点可靠资格。
`mapping_id` 只是证据绑定标识，修改其名称不会提升证据等级。

### 6.2 程序入口

| 程序及入口文件 | 功能 |
| --- | --- |
| [apps/autoaim_node.cpp](apps/autoaim_node.cpp)、[apps/offline_replay.cpp](apps/offline_replay.cpp) | 同一个公共离线运行流程；支持 --config、--input、--output、--record-session、--uart-output |
| [apps/autoaim_infantry.cpp](apps/autoaim_infantry.cpp)、[apps/autoaim_sentry.cpp](apps/autoaim_sentry.cpp) | 多做一次角色校验的入口，不是两份独立算法 |
| [apps/autoaim_rune.cpp](apps/autoaim_rune.cpp) | 可构建、安装的打符诊断占位入口；`--help` 返回 0，其余调用提示未实现并返回 2，不启动处理链或设备 |
| [apps/calibration_tool.cpp](apps/calibration_tool.cpp) | PnP 检查、内参求解、手眼外参求解 |
| [apps/bench_detector.cpp](apps/bench_detector.cpp) | 单图检测计时或同一数据集上的多配置评测 |
| [apps/annotate_session.cpp](apps/annotate_session.cpp) | 离线人工标注的导出、审核回看、独立会话副本应用；支持 --self-test，不运行模型或设备 |
| [tools/synthetic_sim/main.cpp](tools/synthetic_sim/main.cpp) | 生成合成灯条图像、事件和真值 |
| [tools/replay_visualizer/main.cpp](tools/replay_visualizer/main.cpp) | 复用 Pipeline 的只读结果画 PNG，不是另一个算法或实时 GUI |
| [tools/protocol_tester/main.cpp](tools/protocol_tester/main.cpp) | 离线协议自测、停止包编码、AB 反馈解码，不打开串口 |
| [tools/pipeline_metrics/main.cpp](tools/pipeline_metrics/main.cpp) | 读取命令 TSV 计算源年龄/过期源写入比例，不运行视觉链 |

各工具可执行 `--help`。以下使用 `/path/...` 的命令是待填写路径的模板：

标定与批量评测共用文本 YAML 写出，保留 double 精度 17、末尾换行和写出失败诊断；
标注工具仍使用自身的 float 精度 9 与二进制写出，详见[手册中的工具职责](additional_information.md#module-pipeline)。

```bash
# PnP 自检：程序生成已知姿态的角点，不需要外部角点文件
build-debug/calibration_tool --config config/fast_choose.yaml --self-test

# 内参与手眼求解：使用自己采集并划分的数据
build-debug/calibration_tool --solve-intrinsics /path/intrinsic-data.yaml \
  --output /path/new-intrinsics
build-debug/calibration_tool --solve-hand-eye /path/hand-eye-data.yaml \
  --intrinsics /path/new-intrinsics/intrinsics.yaml --output /path/new-calibration

# 单图计时：包含预热，输出 P50/P95，不等同于整条链的 FPS
build-debug/bench_detector --config config/fast_choose.yaml \
  --image out/quickstart-data/frame_000001.png --iterations 100

# 同一数据、同一敌方颜色，多套配置比较
build-debug/bench_detector --dataset /path/events.yaml \
  --config /path/yolov5.yaml --config /path/yolo11.yaml \
  --config /path/traditional.yaml --iou 0.5 --output /path/new-comparison

# 默认 UART14 停止包，纯字节输出
build-debug/protocol_tester --encode-stop
```

检查自己的角点时，使用 `--corners /path/corners.yaml` 替换 `--self-test`。当前板型配置对应的角点文件写法如下；数字仅为测试像素：

```yaml
plate_type: small
corners_tl_tr_br_bl: [[302, 232.7], [338, 232.7], [338, 247.3], [302, 247.3]]
```

标定支持棋盘格（内角点行列数）和对称圆点阵。实际间距、分辨率、ROI 和设备标识必须实填；
至少三份 fit 样本才能求解，形成通过证据还需至少两份独立 validation 样本。
内参首版求解为针孔加五参数畸变；手眼用 Park，要求同步的完整 `gimbal_to_reference` 位姿、
平移及时间，不能把缺失平移填零，也不能只给 yaw/pitch。没有验证集可以生成候选参数，但不代表实测通过。
报告导入会重查参数和残差；完整数据字段及样例注释见标定清单模板。

### 6.3 回放与评测数据

输入不是 MP4/相机编号，而是本地图片加 YAML 事件清单。建议先查看 synthetic_sim 生成的 `events.yaml`，
也可用 [tests/fixtures/session_v2.yaml](tests/fixtures/session_v2.yaml) 了解字段，但它是读取器测试夹具，缺完整姿态和角点标注，不是可直接运行整链评测的数据集。

- v2 使用 `schema_version: 2`、`domain: replay`、`complete: true`；中断未完成会话拒绝读取，旧无版本清单仍兼容。
- 支持 image、feedback、原始 uart 和独立 button 事件；`at_ns` 非递减，同时间保留文件顺序，图片帧号严格递增。
- 三维处理需要曝光时刻的有效姿态；二维 Precision/Recall 可以在无姿态时评测，不能因此声称整链可用。接收时间、设备 tick 和曝光时间不是一回事。
- `annotations: []` 是已标注无目标；缺少 annotations 是未标注，二者统计含义不同。
- 标注包含 category、color、plate_type、物理 TL/TR/BR/BL 四角、visible，可选 track_id；可见性和角点格式见 [annotation.hpp](include/autoaim/vision/annotation.hpp) 与测试样例。
- 原始记录与 `derived.yaml` 分开，后者包含运行信息、最终命令/撤销原因、丢帧和耗时。内容标识用于复现定位，不是安全认证。
- 批量 `report.yaml` 输出匹配、Precision/Recall（查准率/查全率）、分类/角点误差和连续漏检；`timing.yaml` 单独保存墙钟耗时。匹配按四角的轴对齐包围框 IoU 排序，确定性一对一分配。
- 默认不报位姿误差；只有合格 pose_truth，并提供 `--pose-reference`、`--pose-position-limit-m`、`--pose-rotation-limit-rad`，才使用该真值评估。
- 批量模式 `--iou` 必须显式提供有限的 `(0,1]` 数值，拒绝尾随垃圾；未提供位姿参数时报告 `pose_metrics: not_produced`。
- 报告新增原始参数、工作目录、数据指纹、审核绑定和构建来源；固定六帧 golden 已进入 CTest，阶段验证记录见 [构建与验证记录](build_history.md#测量闭环验收2026-09-30)。

### 6.4 人工标注与可信评测

`annotate_session` 是离线命令行工具，不是 GUI，也不会调用模型生成标签。
先用图片查看器读像素坐标，再编辑导出的 YAML；标注四角必须按物理 TL/TR/BR/BL 顺序，
不能只按画面里的上下左右排序。不可见角点仍要填写几何位置，并将对应 visible 设为 false。

```bash
# SOURCE 可为会话目录或 events.yaml；输出均须是源会话外、父目录已存在的新目录。
build-debug/annotate_session --export /path/new-label-draft --session /path/source
# 人工编辑 new-label-draft/annotations.yaml，核对后将 reviewed 改为 true。
build-debug/annotate_session --check --session /path/source \
  --annotations /path/new-label-draft/annotations.yaml --output /path/new-review
# 查看 check.yaml 及 frames/ 回看图；任何错误均返回非零，不能跳过检查直接信任文件。
build-debug/annotate_session --apply --session /path/source \
  --annotations /path/new-label-draft/annotations.yaml --output /path/new-labeled-session
build-debug/bench_detector --dataset /path/new-labeled-session/events.yaml \
  --config /path/config.yaml --iou 0.5 --output /path/new-report
```

首版只接收完整 v2、每个事件一行的 flow 清单，允许前导注释和既有可选字段缺省。
图片须为会话内相对 PNG 路径，不含 `..`，也不允许符号链接逃逸；旧回放器不受此限制。
图片始终独立复制，编辑副本不会改变原图。工具只在 Linux/WSL 发布输出：
在目标旁创建临时目录，完整校验后无覆盖原子重命名；不支持时失败，不回退为覆盖写入。
本机 `/mnt/e` 的 Windows 挂载不支持该发布操作；输出请放在 WSL 原生文件系统
（例如 `/tmp` 下自己新建的工作目录），源会话仍可从 `/mnt/e` 只读加载。

导出内容：`index.yaml` 记录帧、尺寸、时间及原始反馈参考；缺失原字段保持未知，
不伪造同步姿态；`frames/` 保存 PNG 副本；`annotations.yaml` 默认未审核。
模板含 `schema_version: 1`、`source_fingerprint`、`reviewed` 和逐帧 `frames` 列表。

| 提交状态 | 含义 |
| --- | --- |
| 未列入 frames | 未标注，不参与 P/R |
| 有帧但缺 annotations | 格式错误 |
| annotations: null | 未完成草稿，检查和应用拒绝；请完成或移除该帧条目 |
| annotations: [] | 人工确认没有目标；不能拿它占位 |
| 非空列表 | category、color、plate_type、corners、visible；可选 track_id |

`--apply` **整体替换标注集**：源会话中即使已有标签，未列入本次清单的帧也会在新副本中
变为未标注；原会话保持不变。非 image 行含换行按字节保留，image 行只改变标注，
不复制旧 `derived.yaml`。输出附提交的 `annotations.yaml` 与 `annotation_provenance.yaml`。

审核绑定同时核验输出事件/PNG、提交文件及嵌入标注。数据指纹采用 FNV-1a64 与字节数，
集合采用长度前缀编码；它能定位内容变化，**不是签名，也不证明人工标签正确**。
文件摘要字段 `fnv1a64` 和集合算法 `fnv1a64-length-prefixed-v1` 保持既有格式；
共享字节累计的范围与各用途编码见[手册 core 职责](additional_information.md#module-core)。
没有来源 sidecar 的旧数据仍可评测，报告审核字段为 null；存在但不匹配的 sidecar 会拒绝评测。
`reviewed: true` 只是人工审核声明，不会授予控制能力或影响置信度、阈值和最终 shoot。

报告保留现有指标，新增 schema、原始参数数组、工作目录、数据/标注指纹和构建来源。
构建信息包括源码清单 SHA-256、编译器、标准、选项及库版本，无生成时间；未知版本明确标出。
同一构建和输入、同一工作目录与完整参数下：先移走第一次输出，再以同一路径复跑，
完整 `report.yaml` 应逐字节一致，不排除任何字段；墙钟变化只在 `timing.yaml`。
不同路径/构建的来源信息可能不同，不能要求全文相同。

固定回归素材见 [evaluation-golden](tests/fixtures/evaluation-golden/README.md)。
合成标签、合成指标与本工具的通过结果都不能代替真实标注质量、位姿精度或 NUC 实测。

## 7. 控制模式与通信接口

| 模式 | 行为 |
| --- | --- |
| 步兵辅助（默认） | 人工为主，程序提出相对修正；程序 shoot 始终为 false |
| 步兵自动 | 必须明确启用；可申请绝对指向和开火，仍需全部许可条件通过 |
| 人工接管、输入失效或故障 | 撤销自动模式和旧指令；松手或恢复输入不自动重新启用 |
| 哨兵自动 | 提出自动指向/开火请求，是否开火取最终 shoot，而不是上游请求值 |

独立按键使用 `operator_input.kind: button`：
`button_mode: toggle` 每次有效按下切换；`hold` 按住自动、松开辅助。
两种实现都编译。启动时按住或接管后继续按住都不自动启用，需先释放再按下。
`legacy_enable_event` 保留旧回放的显式启用语义，不从 UART 未定义字段猜按键。

`kind` / `button_mode` 只选择输入策略。独立按键启用自动模式还需要合格的 `button_evidence`
与 `control_channel`，且证据的时钟域、设备及配置绑定必须匹配。默认装配的这两项均为 `declared`，
现有 YAML/CLI 未提供它们的合格证据装载入口；已有正向回归由 C++ 调用方显式注入仅在 replay 域
合格的模拟证据。仅修改按键策略不会使默认示例进入自动模式。

对外 [Command](include/autoaim/control/command.hpp) 固定五个字段：

```cpp
struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};
```

yaw/pitch 是弧度，horizon_distance 是米。停止值用 `Command{}`，不要使用未初始化的 `Command command;`。
来源、世代、指令空间和前馈另存 CommandMetadata，不塞进这五个字段；不能直接发送结构体内存。

发送默认 **CBoard UART v2，14 字节**；接收是 **AB 帧头的 43 字节反馈**，收发长度不同。

| 发送偏移 | 内容 |
| --- | --- |
| 0–1 | `A5 0E`：帧头与总长度 |
| 2–3 | control、shoot 各一字节 |
| 4–9 | yaw、pitch、水平距离，各 int16 小端，值乘 10000 后向零截断 |
| 10–11 | 前 10 字节 CRC16，小端 |
| 12–13 | 帧尾 `91 78` |

距离编码饱和到 0–3.2767 m，主机内部距离不截断；启用控制时超范围角度拒绝编码。
CRC 初值 0xFFFF、反射多项式 0x8408、无末尾异或。默认停止包：
`a5 0e 00 00 00 00 00 00 00 00 aa d5 91 78`。
其它既有 UART/AB/CAN 发送格式仅保留显式兼容选项，不自动探测或回退。

相对修正的固件映射尚未验证，编码器拒绝启用的相对控制包。
最终许可由 command_guard 形成，publisher 发送前只能保留或撤销；完整写入成功也不等于设备已执行或停止。

## 8. 测试、已验证范围与待完成项

### 测试文件怎么看

| 位置 | 作用 |
| --- | --- |
| [tests/unit/](tests/unit/) | 单功能算法、配置解析、协议和工具测试 |
| [tests/contract/](tests/contract/) | 跨模块约定：时间/世代、缓冲生命周期、乱序、模式切换、写失败与关闭 |
| [tests/synthetic/](tests/synthetic/) | 已知真值下的跟踪、板身份、几何选择和落点指标检查 |
| [tests/fixtures/](tests/fixtures/)、[tests/support/](tests/support/) | 固定输入文件与测试替身/数据构造，不是生产设备参数 |
| [tests/test_support.hpp](tests/test_support.hpp) | 轻量测试检查工具，Release 不会因 assert 被关闭而跳过检查 |

`tests/synthetic/test_ekf_synthetic.cpp` 仍是空预留；实际相关测试分布在上表目录。
**是否参与测试以 CMake 的注册为准，不以文件名或文件存在为准。**

```bash
# 查看已注册测试 / 只跑队列与乱序测试
ctest --test-dir build-debug -N
ctest --test-dir build-debug --output-on-failure \
  -R 'test_frame_queue|test_out_of_order_completion'

# 本项目离线代码的内存检查；不包含第三方 OpenVINO 运行时
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
  -DAUTOAIM_OPENVINO=OFF -DAUTOAIM_SANITIZERS=ON
cmake --build build-sanitize --parallel 2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-sanitize --output-on-failure
```

模块边界检查与其负例自测同样由 CTest 执行。检查器源码在根 CMakeLists.txt，
生成到构建目录的 check_dependencies.py 不应手工修改。
检查包含 tools/，按各工具实际链接的模块分别授权，并核对 CMake 白名单是否漂移。

构建和测试后可保存独立验收清单（此命令只列测试，不证明测试已通过）：

```bash
cmake --build build-debug --target verification_manifest
# build-debug/verification_manifest.json：测试/夹具 SHA-256、实际 CTest 注册项、
# CMake 缓存、生产源码及构建标识；测试内容不加入运行报告的生产源码指纹。
```

可选线程检查使用单独构建目录，与 ASan/UBSan 互斥；只运行离线并发测试：

```bash
cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DAUTOAIM_OPENVINO=OFF \
  -DAUTOAIM_HIKROBOT=OFF -DAUTOAIM_THREAD_SANITIZER=ON
cmake --build build-tsan --parallel 2 --target test_publish_failure_shutdown \
  test_async_buffer_lifetime test_frame_queue test_worker_exception test_out_of_order_completion
TSAN_OPTIONS=halt_on_error=1 ctest --test-dir build-tsan --output-on-failure \
  -R '^test_(publish_failure_shutdown|async_buffer_lifetime|frame_queue|worker_exception|out_of_order_completion)$'
```

TSan 必须能在宿主环境运行才算验收；普通并发测试通过不能替代线程检查。
代码采用两空格缩进，类型 UpperCamelCase、函数/变量 snake_case；按 .clang-format 排版，return 独占一行。

### 构建与验证记录

构建日志、测试矩阵和逐批修复记录已迁至 [build_history.md](build_history.md)。
其中保留各阶段的环境、结果、未验收项和本机证据链接；历史通过数字只对应当时的源码与构建选项，
不能代替当前工作区的重新验证。构建与测试命令仍以本页 §2 和 §8 为入口。

### 尚不能从现有测试得出的结论

- 已确认相机型号为海康 MV-CS016-10UC 彩色款；序列号、镜头、实际 ROI/曝光/增益、白平衡和时钟映射仍需实测。150fps 是采集目标，不是整链实测帧率。
- 真实录像、角点标签、设备标定和 NUC 运行指标尚未完成验收；CPU 模型冒烟不证明 YOLO11 比 YOLOv5 更准确。
- 固件坐标/权限、人工接管、相对通道、设备超时清零和回执仍需核实。不能把离线输出直接替换为实机闭环。
- 打符、MPC、CKF、空气阻力弹道不属于当前已实现功能；打符预留文件不会因此删除。
