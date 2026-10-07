# 实验工作台

工作台在本机浏览器中管理参数、编译方案和实验。Vue 页面由 FastAPI 服务提供；离线计算仍由项目原有 C++ 工具完成。离线构建和回放使用同一个 Linux / WSL 环境。Windows 服务还提供 SSH 目标设置与 NUC 任务入口，调用 NUC 上已经部署的实时程序。

## 前后端启动方式

本机项目目录为 `E:\大学\巡天御风\XTYF-AutoAim`。下文命令均先进入仓库根目录，
不要从 `workbench/backend` 目录执行 `python -m workbench.backend`。

| 使用方式 | 后端 | 前端 | 浏览器地址 |
| --- | --- | --- | --- |
| 日常使用（推荐） | FastAPI，端口 `8765` | 先构建一次，再由后端提供 `frontend/dist`；无需单独启动前端 | <http://127.0.0.1:8765> |
| 前端开发 | FastAPI，端口 `8765`，终端 A | Vite，端口 `5173`，终端 B；修改页面自动更新 | <http://127.0.0.1:5173> |
| 生产页面预览 | FastAPI，端口 `8765` | Vite preview，端口 `4173`；读取已构建页面 | <http://127.0.0.1:4173> |

Windows 服务用于参数方案、目标设置和 SSH 到 NUC 的任务。本地 C++ 构建、配置检查
和离线回放使用 Linux / WSL 服务，相关 C++ 依赖按[项目 README](../README.md)安装。
只打开网页和管理参数不需要先编译 C++，实际执行离线任务时才需要对应工具和依赖。

首次准备需要 Python 3.10 以上（含 `venv` 和 `pip`）及 Node.js 22.12 以上，
或 Node.js 20.19 以上的 20.x 版本。本机 Windows Python 路径为 `D:\python\python.exe`；
换机时将命令里的解释器路径替换为实际路径。已有 `frontend/dist/index.html` 时，
日常打开网页不需要 Node.js；修改前端后才需要重新构建。

实验目录与源码目录必须相互独立，不能互相包含。同一个实验目录只启动一个后端实例。
方案、独立构建副本、任务、日志和结果均保存在实验目录，仓库配置和输入数据保持只读。
下文 Windows 和 Linux 示例使用不同实验目录，各自显示自己的历史任务。

### Windows：首次准备与日常打开

打开 PowerShell，首次执行以下命令安装服务依赖并构建前端。已有虚拟环境仍需确认已安装
`requirements.txt` 中的依赖；不需要执行虚拟环境激活脚本。

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
& 'D:\python\python.exe' -m venv .venv-workbench
& '.\.venv-workbench\Scripts\python.exe' -m pip install -r workbench/requirements.txt
node --version
npm.cmd --version
npm.cmd --prefix workbench/frontend ci
npm.cmd --prefix workbench/frontend run build
```

使用 `npm.cmd` 可以直接运行 npm，不需要修改 PowerShell 执行策略。`run build` 会先检查
TypeScript，再生成 `workbench/frontend/dist/index.html` 和静态资源。命令成功后，在同一个
PowerShell 中启动后端：

```powershell
& '.\.venv-workbench\Scripts\python.exe' -B -m workbench.backend `
  --source 'E:\大学\巡天御风\XTYF-AutoAim' `
  --workspace "$env:LOCALAPPDATA\xtyf-autoaim-workbench" `
  --host 127.0.0.1 --port 8765
```

看到 `Uvicorn running on http://127.0.0.1:8765` 后，保持该终端打开，在浏览器访问
<http://127.0.0.1:8765>。无需再运行 `npm run dev`。后端 API 文档为
<http://127.0.0.1:8765/docs>，环境信息为 <http://127.0.0.1:8765/api/context>。

以后重新打开时，只需重新进入项目目录并启动后端：

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
& '.\.venv-workbench\Scripts\python.exe' -B -m workbench.backend `
  --source 'E:\大学\巡天御风\XTYF-AutoAim' `
  --workspace "$env:LOCALAPPDATA\xtyf-autoaim-workbench" `
  --host 127.0.0.1 --port 8765
```

若继续使用本机已安装依赖的全局 Python，可以把上述后端启动命令的解释器替换为
`D:\python\python.exe`，其余参数相同。2026-10-07 的临时验收实例使用该解释器和端口
`8766`，访问地址为 <http://127.0.0.1:8766>。若该实例仍在运行，可以直接使用；
不要再为同一实验目录启动第二个实例。改用下文默认端口或开发模式前，先停止旧实例。

### Linux / WSL：首次准备与日常打开

打开 Ubuntu 或 WSL Ubuntu 的 Bash 终端。WSL 中本机 E 盘项目对应
`/mnt/e/大学/巡天御风/XTYF-AutoAim`；在原生 Ubuntu 上将 `cd` 路径替换为实际项目目录。
Ubuntu 缺少虚拟环境支持时先安装 `python3-venv`，缺少 pip 时安装 `python3-pip`：

```bash
sudo apt update
sudo apt install python3-venv python3-pip
```

安装适用版本的 Node.js 后，在 Bash 中执行首次准备：

```bash
cd '/mnt/e/大学/巡天御风/XTYF-AutoAim'
python3 --version
node --version
npm --version
python3 -m venv "$HOME/.venvs/xtyf-autoaim-workbench"
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" -m pip install -r workbench/requirements.txt
npm --prefix workbench/frontend ci
npm --prefix workbench/frontend run build
```

Linux 虚拟环境位于 `$HOME/.venvs/xtyf-autoaim-workbench`，与 Windows 项目中的
`.venv-workbench` 分开；不同系统的虚拟环境不可互用。若 `node_modules` 是在 Windows 安装的，Linux 中也先执行
`npm ci` 再构建；回到 Windows 开发时同样重新执行 Windows 的 `npm.cmd ... ci`。

前端构建完成后，启动后端：

```bash
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" -B -m workbench.backend \
  --source "$PWD" \
  --workspace "$HOME/.local/share/xtyf-autoaim-workbench" \
  --host 127.0.0.1 --port 8765
```

保持终端打开，在浏览器访问 <http://127.0.0.1:8765>；WSL 服务可从 Windows 浏览器访问
该地址。以后只需执行 `cd` 和后端启动命令，无需重复创建虚拟环境、安装依赖或构建页面。
WSL 页面中填写 Linux 路径，例如 `/mnt/e/datasets/events.yaml`；实验输出使用上面的
Linux 用户目录。

### 开发模式：分别打开后端和前端

先完成所用系统的首次依赖准备。开发模式需要两个终端，两个终端都保持打开。
后端启动方式与日常使用相同，必须使用端口 `8765`，因为
`workbench/frontend/vite.config.ts` 将 `/api` 代理到 `http://127.0.0.1:8765`。

Windows 终端 A（PowerShell，后端）：

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
& '.\.venv-workbench\Scripts\python.exe' -B -m workbench.backend `
  --source 'E:\大学\巡天御风\XTYF-AutoAim' `
  --workspace "$env:LOCALAPPDATA\xtyf-autoaim-workbench" `
  --host 127.0.0.1 --port 8765
```

Windows 终端 B（另一个 PowerShell，前端）：

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
npm.cmd --prefix workbench/frontend run dev -- --port 5173 --strictPort
```

Linux / WSL 终端 A（Bash，后端）：

```bash
cd '/mnt/e/大学/巡天御风/XTYF-AutoAim'
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" -B -m workbench.backend \
  --source "$PWD" \
  --workspace "$HOME/.local/share/xtyf-autoaim-workbench" \
  --host 127.0.0.1 --port 8765
```

Linux / WSL 终端 B（另一个 Bash，前端）：

```bash
cd '/mnt/e/大学/巡天御风/XTYF-AutoAim'
npm --prefix workbench/frontend run dev -- --port 5173 --strictPort
```

浏览器打开 <http://127.0.0.1:5173>。页面修改后由 Vite 自动更新；后端 Python 文件修改后
需在终端 A 按 `Ctrl+C`，再重新执行启动命令。前端开发服务不代替后端，单独启动前端
无法读取配置、保存方案或提交任务。

如果只想预览生产页面，先执行对应系统的 `npm ... run build`，保持后端在 `8765` 运行，
在终端 B 用以下命令替换 `run dev`：

```powershell
# Windows PowerShell
npm.cmd --prefix workbench/frontend run preview -- --port 4173 --strictPort
```

```bash
# Linux / WSL Bash
npm --prefix workbench/frontend run preview -- --port 4173 --strictPort
```

打开 <http://127.0.0.1:4173>。当前 Vite preview 同样将 `/api` 代理到 `8765`；
它读取 `dist`，修改源码后要重新 `run build` 才能看到更新。日常使用仍推荐由后端直接
提供生产页面。

### 停止、更新和启动问题

- 停止开发前端：在终端 B 按 `Ctrl+C`。关闭浏览器标签页不会停止前后端进程。
- 停止后端：先在任务中心取消或停止活动任务，查看结果，再在终端 A 按 `Ctrl+C`。
  NUC 停止须取得远端确认；SSH 断开或服务显示中断不证明设备已经停止。
- 更新前端后用于日常使用：在所用系统执行 `npm ... run build`，刷新浏览器。
  首次生成 `dist` 时若后端已经启动，需要重启后端以挂载页面。
- `ModuleNotFoundError: fastapi`：使用启动后端的同一个 Python 执行
  `-m pip install -r workbench/requirements.txt`，不要将全局和虚拟环境解释器混用。
- `No module named workbench`：先回到仓库根目录，再执行 `-m workbench.backend`。
- `npm` 找不到或 Node.js 版本不满足：安装上面要求的版本并重新打开终端，再运行版本检查。
- 根页面 `503` 或 `404`：确认 `workbench/frontend/dist/index.html` 已生成，然后重启后端。
  先访问 `/api/context` 可以区分后端未启动和静态页面未构建。
- 端口占用：复用已有服务，或停止自己的旧实例。日常一体化模式可将后端 `--port` 改为
  `8766` 并访问相应地址；开发或 preview 模式若改后端端口，还需将
  `vite.config.ts` 的代理目标改为同一端口并重启前端。

工作台不会自动安装 C++ 依赖或下载模型。离线任务不启动设备；NUC 运行须由用户提交
任务并通过目标程序预检。启动网页本身不会连接 SSH 或启动相机。

## NUC 目标配置与运行

打开“算法方案”页顶部的“在 NUC 上运行方案”，展开“NUC 连接与程序设置”填写
以下信息。SSH 登录目标为 `xtyf@10.141.143.124:22`；
目录、程序和模型字段使用 NUC 上的实际 Linux 路径，不填写 Windows 或 WSL 的映射路径。

| 设置 | 含义 |
| --- | --- |
| `host`、`user`、`port` | NUC 地址、SSH 用户与端口；本机目标为 `10.141.143.124`、`xtyf`、`22` |
| `identity_file` | Windows 本地私钥路径；使用 SSH agent 时可留空 |
| `project_dir` | NUC 上的项目绝对路径 |
| `build_dir` | NUC 上已有构建的绝对路径，包含实际构建元信息 |
| `device_config` | NUC 上实际设备配置的绝对路径，含标定、几何和设备字段 |
| `workspace_dir` | NUC 上的任务输出绝对路径；与 `project_dir`、`build_dir` 独立，彼此不可包含 |
| `executable` | 实时程序；默认 `autoaim_node`，也可指定绝对路径或相对构建目录的路径；解析后的真实程序必须位于所选 `build_dir` 内 |
| `yolov5_model`、`yolo11_model` | 所需检测器在 NUC 上的模型 XML 绝对路径，配套 BIN 必须相邻 |

目标设置不保存密码。Windows OpenSSH 必须已能通过密钥或 SSH agent 无交互登录，
目标主机密钥须已核实并写入当前用户的 `known_hosts`。工作台不会绕过主机密钥检查，
也不弹出 SSH 密码或首次信任确认。目标地址可先保存；运行前补全实际项目、构建、
设备配置和任务目录。

点击“保存连接设置”后，可用“检查 NUC”执行只读检查。选择“已保存参数方案”、
“要求的构建类型”、OpenVINO 状态及六个算法开关，再点击“在 NUC 上运行”。
参数页另存方案后也可用“到 NUC 运行已保存方案 →”跳到该面板。程序必须位于所选
构建目录内，才能将实际程序与该目录的构建元信息对应；目录外的已安装程序不满足
本入口的构建身份检查。

NUC 检查任务 `nuc_probe` 只读取系统信息、构建元信息、文件存在性和设备配置中的
`execution` 字段；它不调用设备程序的配置检查或启动设备。运行任务 `nuc_run`
使用所选参数方案和编译选项，先核对 NUC 构建元信息中的六个算法开关、OpenVINO
状态和构建类型是否与网页选择完全相符。它固定任务参数，将日常参数与 NUC 的
设备配置组合到任务包中，随后由所选实时程序执行：

```bash
/actual/nuc/program --check-config --config /actual/nuc/task/fast_choose.yaml
```

程序必须支持这个参数，返回有效配置，并明确报告
`effective_configuration.execution: hardware`。通过后才执行同一个程序的
`--config` 运行入口。设备配置必须来自 NUC 的实际部署，离线模板不能替代设备标定、
几何或设备通道声明；选中 YOLO 时也必须使用 NUC 上实际存在的对应模型。

任务包保留设备配置中的相机、通信通道和物理证据字段，将标定与几何引用按远端
设备配置文件解析为 NUC 绝对路径。输入资源来自远端设备配置或 NUC 项目模板；
模型路径以目标设置为准，未指定时使用 NUC 项目模板中的路径。Windows 参数方案
里的本机资源和模型路径不作为 NUC 路径使用。

当前仓库的 `autoaim_node` 仅支持 replay，并明确拒绝 hardware。NUC 任务入口不会
删除这个限制或把配置检查失败改成成功；需要 NUC 上已有可用的实时程序及设备配置，
才能进行实机运行。现在尚未核验这些路径，也未完成实机验收。

活动任务可用“查看运行日志”查看日志，点击“停止 NUC 运行”请求远端停止整组
子进程，并检查进程组已停止。远端停止请求失败时 API 返回 422，任务仍保持
`running`，页面保留停止未确认的原因；不能仅凭本地 SSH 进程退出显示设备已经停止。
服务重启后未完成任务记为 `interrupted`，这也不等于远端已停止。任务日志和已有产物保留。

2026-10-07 对上述 NUC 的两次只读 SSH 连接均建立了 TCP 连接，随后远端在 SSH
握手前关闭，报 `kex_exchange_identification`。目前没有成功登录、设备配置检查或
实时程序运行记录；详见 [验证记录](VALIDATION.md)。
用户随后已断开该 SSH 连接；本轮没有继续连接 NUC。

本轮新增软件测试共 23 项通过：WSL Linux 桥接与远程 helper 18 项，Windows NUC
任务与 API 5 项。原有后端在 Windows 复验为 18 通过、1 个 POSIX 测试跳过，Vue
TypeScript 检查与 Vite 构建通过。远程 helper 使用临时 Python 程序夹具；这些结果
不代表目标 NUC 的硬件程序或停止确认已经验收。

## 无模型演示

1. 在“算法方案”选择基线，关闭 OpenVINO，执行构建。默认 Release、C++17、两路编译并行；构建流程同时运行 CTest。
2. 在“参数配置”将 `active_detector` 明确设为 `traditional`，另存方案。原模板仍保持 YOLOv5 默认值。
3. 使用该构建与参数方案执行配置检查，然后生成合成数据。
4. 选择生成的 `events.yaml` 进行离线回放；任务完成后查看命令、记录会话和逐帧叠加图。
5. 另存第二份参数方案，用同一个数据清单进行批量比较。

有真实模型时，注册模型所在目录，填写 XML 路径，确保同目录有配套 BIN，选择 OpenVINO 构建。模型未提供或实际推理未验证会如实显示，不会自动改用传统检测。

## 使用已有 YOLO 资源

模型无需重复下载。本机现有 `RM2026-AutoAim/assets` 已包含匹配当前解码器的
`yolov5.xml/.bin` 与 `yolo11.xml/.bin`。在本机 WSL 服务中可注册
`/mnt/e/大学/巡天御风/RM2026-AutoAim/assets`，在“算法方案”启用 OpenVINO 并选择
两份 XML；本机已有 OpenVINO CMake 包目录为 `/usr/lib/cmake/openvino2026.3.1`。
显式填写包目录时也要将其注册为服务端目录，能自动找到时可留空。

再分别另存 YOLOv5 和 YOLO11 参数方案：将 `active_detector` 设为相应检测器，
将 `detectors.yolov5.model_path` 或 `detectors.yolo11.model_path` 指向所选 XML，
设备可选 `CPU`。构建验收模型路径与运行参数中的模型路径分别用于构建测试和实验，
页面不会替用户切换检测器或修改仓库默认配置。以上目录属于本机已有资源，换机时
使用实际路径；Git 工作副本不会自动带入其它目录中的模型或系统运行库。

“构建与验收记录”显示实际模型验收状态。仅找到 XML/BIN 或通过配置检查不能确认
推理成功；提供两套模型的完整 CTest 包含七项真实模型测试。

## 参数与方案

参数表单覆盖 `config/fast_choose.yaml` 的叶节点。字段说明来自模板注释，数组按整项编辑。基础标定、几何和角点文件支持查看和选择，首版不提供标定求解编辑器或人工标注画板。

另存会生成独立配置包，同时修正三份基础配置的 `fast_choose_file` 回指及资源路径。
所选标定、几何以及标定引用的内外参报告一并复制，使用包内相对引用。任务提交时再固定
配置快照，之后保存新方案或修改原契约文件不影响已提交任务。模型和数据保持原路径，
其内容标识由实际运行报告记录。配置检查调用所选构建中的：

```bash
autoaim_node --check-config --config /path/to/fast_choose.yaml
```

成功输出包含 `check_schema_version`、`valid` 和 `effective_configuration` 的 YAML；失败返回非零并保留诊断。此模式不运行处理链。配置检查通过不等于模型已经成功推理。

运行参数在新任务启动时生效。I1 精修、I2 LM、I3 平移 CA、I9 推理提示、I9 队列预分配和 ESO 属于编译方案，需要独立构建与测试。I3 与 ESO 互斥，THROUGHPUT 要求 OpenVINO；工作台对两种 I9 候选沿用现有双模型验收要求。

验证脚本保留原来的 `--method`，组合模式使用 `--selection-file`，文件格式为：

```json
{
  "schema_version": 1,
  "options": {
    "AUTOAIM_I1_CONTRAST_IRLS": true,
    "AUTOAIM_I2_LM": true,
    "AUTOAIM_I3_LINEAR_CA": false,
    "AUTOAIM_I9_THROUGHPUT": false,
    "AUTOAIM_I9_PREALLOC": false,
    "AUTOAIM_USE_ESO": false
  },
  "openvino": false
}
```

组合不会继承某个单项候选的验收结论。全部开关显式传给 CMake，完整 CTest 和候选专项注册情况一起保留。

## 任务和结果

任务按队列串行运行，避免同时运行的实验互相干扰耗时。离线取消操作终止整组子进程；NUC 取消还须确认远端进程组停止。保留已经生成的日志和未完成产物。服务重启将未完成任务标记为中断，不自动重跑，也不覆盖历史输出；中断状态本身不证明远端设备已经停止。

工作台复用以下工具：

| 操作 | 工具与产物 |
| --- | --- |
| 合成数据 | `synthetic_sim`：图片、事件、真值 |
| 回放 | `offline_replay`：命令 TSV、UART 记录、录制会话 |
| 逐帧预览 | `replay_visualizer`：独立回放产生的 PNG |
| 命令源年龄 | `pipeline_metrics`：读取命令 TSV |
| 单图计时 | `bench_detector --image`：P50/P95 |
| 多方案比较 | `bench_detector --dataset`：`report.yaml`、`timing.yaml` |

同一构建内使用多配置评测；跨构建分别运行并检查数据指纹、敌方颜色、IoU 和位姿评测条件是否一致。逻辑报告与耗时报告分开展示，不能把单图计时当成整链 FPS。逐帧预览来自独立回放，不冒充批量评测时的逐帧记录。

无标签帧不当作负样本；缺失指标显示“未产生”，而不是零。没有显式合格位姿真值时保留 `pose_metrics: not_produced`。合成数据用于验证流程，本机或 WSL 的测量不替代目标设备验收。

## 开发与验证

本次实际验证范围见 [验证记录](VALIDATION.md)。

前端使用 `npm --prefix workbench/frontend run build` 完成 TypeScript 检查与生产构建。后端测试使用：

```bash
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" -m unittest discover -s workbench/backend/tests -v
python3 tests/verify_alternatives.py --self-test
```

启动服务后，在同一个 Linux / WSL 环境执行真实工具的端到端验收：

```bash
# 经 HTTP 接口构建并跑 CTest，生成数据、回放、比较，并复核直接 CLI 的逻辑报告。
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" workbench/verify_workbench.py --with-combination
# 已有通过验证的基线构建时，可以复用页面中的构建任务 ID。
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" workbench/verify_workbench.py --build-id BUILD_TASK_ID
# 使用已有模型；省略 --build-id 会创建新的 OpenVINO 基线并跑完整 CTest。
"$HOME/.venvs/xtyf-autoaim-workbench/bin/python" workbench/verify_workbench.py --build-id OPENVINO_BUILD_TASK_ID \
  --yolov5-model /path/to/yolov5.xml --yolo11-model /path/to/yolo11.xml \
  --openvino-dir /path/to/openvino/cmake
```

该脚本创建新的方案和实验，不覆盖已有记录。它使用真实 C++ 程序；后端单元测试中的
子进程替身只检验任务编排，不能替代这项验收。
带模型参数时逐路执行配置检查、单图推理、六帧回放及 API/CLI 报告比对，并核对
复用构建的模型指纹和 OpenVINO 目录。证据保存至实验目录的 `acceptance_checks/`，
包含配置与模型前后指纹、任务索引和直接 CLI 产物。

本地 HTTP 接口由服务的 `/docs` 页面列出，主要资源如下：

| 资源 | 接口 |
| --- | --- |
| 环境与表单 | `GET /api/context`、`GET /api/config/template` |
| 参数方案 | `GET /api/profiles`、`POST /api/profiles`（不可变另存） |
| 已有构建 | `GET /api/builds` |
| NUC 目标设置 | `GET/POST /api/nuc` |
| 任务 | `GET/POST /api/jobs`、`GET /api/jobs/{id}`、`POST /api/jobs/{id}/cancel` |
| 日志与结果 | `GET /api/jobs/{id}/log`、`GET /api/jobs/{id}/results`、`GET /api/jobs/{id}/artifacts` |
| 文件与数据目录 | `GET /api/files`、`POST /api/roots` |
| 任务产物 | `GET /api/artifacts/{job_id}/{path}` |

任务种类为 `build`、`check_config`、`synthetic`、`replay`、`benchmark`、`single_image`、
`ctest`、`nuc_probe`、`nuc_run`。任务记录保存状态、固定配置、实际命令、工作目录和退出码。状态包括等待、
运行、成功、失败、取消和中断；成功构建的注册信息与实际 CMake 参数一起展示。

工作台是独立工具层，不改变九个 C++ 模块的职责、现有 YAML 格式或离线入口的设备限制。接口、实际验证范围和已知限制应与项目主文档一起更新。
