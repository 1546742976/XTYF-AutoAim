# 工作台验证记录

## 2026-10-07：两批瘦身删除

执行目录为 `E:\大学\巡天御风\XTYF-AutoAim`，分支 `codex/slimming-deletions`。
初始提交为 `1efa4952dac28623b9516d2ffae37cedcebb5ff2`；开始时工作区及暂存区均为空。
只实施批准的 Z1 / L1 范围，原有验证记录保留在下方。

| 批次 | 实际 diff 与指标 | 结果 / 回滚点 |
| --- | --- | --- |
| Z1 | `App.vue` 删除 `buildName/profileLabel`；`remote_runner.py` 删除未用 `sys`；`test_backend.py` 删除未用 `write_json` 导入符号。3 文件，1 行增加 / 4 行删除；私有函数 2→0，导入符号 2→0 | 提交 `a0f65a6897475e4dffaccb3d255d8e23097cd6bb`，相关验证通过；其他 358 个跟踪文件字节一致 |
| L1 | 仅删除 `assets/index-caNWWbLH.js`（134,396 B）和 `assets/index-CvYuxZNt.css`（24,070 B）。dist 从 5 文件 / 332,170 B 变为 3 文件 / 173,704 B，减少 158,466 B（47.7%） | 完整 dist 及 SHA256 清单已备份；当前 HTML、JS、CSS 字节未变。Git 只提交本节记录，忽略产物不入库 |

Z1 不承诺速度或包体积收益；L1 只减少本地过期资源占用，当前页面加载的资源和传输量未变。

### 引用与接口保护

删除前重新检查全部跟踪文件、当前 HTML 和资源依赖图，没有两个旧资源名的引用；
可观察的浏览器清单中没有其他工作台页面，验证页使用当前 JS/CSS，未识别到引用旧资源的页面。
完整 5 文件备份及待删文件 SHA256 均与删除前产物一致；没有清空 dist 目录。

打符入口、构建/安装注册、`Task::rune`、两个 `rune_mission` 预留文件未改；
`autoaim_rune --help` 返回 0、其他调用返回 2 的诊断源码及输出文本未改。
六个候选 CMake 开关仍默认 OFF，OpenVINO CMake 默认 ON；I3/ESO 互斥、THROUGHPUT
需要 OpenVINO 的约束未改。三检测器选择、默认 YOLOv5、配置回指、估计器接口、
`--method` / `--selection-file` 和 `verify_eso.py` 均未改；HTTP 接口和请求字段未改。

浏览器检查了基线、六预设、自定义组合、模型及构建选项、创建构建按钮和独立 NUC 运行按钮。
选择 I1 后两个表单同步勾选 I1；I3+ESO 及未启用 OpenVINO 的 THROUGHPUT 都显示原有提示、
禁用构建提交。最后恢复原表单选项，没有保存配置、提交任务或连接 NUC。
NUC 按钮仍因既有远端路径与方案未配置完整而禁用，此轮没有硬件运行验收。

### 验证命令与本轮结果

以下 PowerShell 命令在项目根目录执行；Python 使用已存在的本地依赖，不安装依赖。
前端输出使用独立暂存目录，没有覆盖在线 dist。

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
$slimRun = 'C:\Users\Lenovo\AppData\Local\xtyf-autoaim-workbench\slimming\20261007-2894c6ff'
npm.cmd --prefix workbench/frontend run build -- --outDir "$slimRun\dist-Z1"
& 'D:\python\python.exe' -B -m unittest workbench.backend.tests.test_backend -v
wsl.exe -d Ubuntu-22.04 --cd /mnt/e/大学/巡天御风/XTYF-AutoAim -- python3 -B -m unittest workbench.backend.tests.test_nuc -v
& 'D:\python\python.exe' -B tests/verify_alternatives.py --self-test
git diff --check
```

| 验证 | 实际结果 |
| --- | --- |
| Vue TypeScript / Vite 生产构建 | 通过；暂存的 3 个产物与在线当前 HTML/JS/CSS SHA256 完全一致 |
| Windows 后端既有测试 | 19 项：18 通过，1 项 POSIX 进程组测试按平台跳过 |
| WSL Linux NUC helper 既有测试 | 18/18 通过；仅使用临时进程夹具，没有连接 NUC |
| 算法验证脚本自测 | 278 项通过；保留旧入口与组合约束；没有真实 C++ 构建 |
| 删除前后 HTTP | `/`、当前 JS/CSS、`/api/context` 均 200，资源响应与磁盘字节一致；删除后的两个旧 URL 为 404 |
| 删除后浏览器 | 参数、算法方案、任务及日志区域正常；默认检测器仍为 `yolov5`；没有创建任务 |
| Diff / 接口保护 | 空白检查通过；Z1 之外的源码字节一致，L1 另增加本节记录 |

复查 HTTP 的命令（服务此轮已运行在 8766，不另启动服务）：

```powershell
@('/', '/assets/index-B6t77Qy8.js', '/assets/index-CP64yvY-.css', '/api/context') | ForEach-Object {
  (Invoke-WebRequest -UseBasicParsing -Uri ('http://127.0.0.1:8766' + $_)).StatusCode
}
```

本轮是清理及软件接口回归，不重跑未变化的 C++ 矩阵，也不提供 NUC 性能或硬件通过结论。

### Diff、备份与回滚

证据目录：`C:\Users\Lenovo\AppData\Local\xtyf-autoaim-workbench\slimming\20261007-2894c6ff`。
其中 `Z1.diff` 是三个源码文件的实际 diff，`L1-artifact.diff` 是两个忽略产物的删除清单，
`L1.diff` 是本验证文档的实际 diff；`Z1-*.log` 保留各验证输出。
`dist-before/` 保留完整原 dist，`dist_inventory_before.json` / `dist_inventory_after.json`
记录 SHA256 和字节数，`metrics_before.json` / `metrics_after.json` 记录前后指标。
`UI-*.txt` / `UI-assets-*.json` 与 `slimming-interfaces-preserved.png` 保留页面检查证据。

Z1 用独立提交回滚；L1 必须恢复下面两个产物，不能只用 Git。
如需撤销 L1 的文档记录，另对相应文档提交执行 `git revert <文档提交号>`。

```powershell
Set-Location -LiteralPath 'E:\大学\巡天御风\XTYF-AutoAim'
git revert a0f65a6897475e4dffaccb3d255d8e23097cd6bb
$slimBackup = 'C:\Users\Lenovo\AppData\Local\xtyf-autoaim-workbench\slimming\20261007-2894c6ff\dist-before\assets'
Copy-Item -LiteralPath "$slimBackup\index-caNWWbLH.js" -Destination 'E:\大学\巡天御风\XTYF-AutoAim\workbench\frontend\dist\assets\index-caNWWbLH.js'
Copy-Item -LiteralPath "$slimBackup\index-CvYuxZNt.css" -Destination 'E:\大学\巡天御风\XTYF-AutoAim\workbench\frontend\dist\assets\index-CvYuxZNt.css'
```

## 2026-10-07：Windows 工作台与 NUC SSH 入口

新增范围是 Windows 本地工作台到 NUC 的 SSH 目标设置、只读检查和实时程序任务入口。
Windows 本机 Python 服务依赖已经准备好；这不代表 NUC 设备程序已具备运行条件。

| 检查 | 当前证据与状态 |
| --- | --- |
| SSH 目标 | `xtyf@10.141.143.124:22` |
| 实际 SSH 连接 | 两次只读连接显示 TCP connection established，随后远端在 SSH 握手前关闭，报 `kex_exchange_identification`；未成功登录 |
| NUC 项目、构建、实时程序与设备配置路径 | 尚未取得和核验 |
| NUC 构建选项一致性 | 尚未实机读取构建元信息；六个算法开关、OpenVINO 与构建类型须和所选方案相符 |
| NUC hardware 配置检查与设备运行 | 未验收；当前仓库 `autoaim_node` 仅支持 replay，明确拒绝 hardware |
| 远程取消和停止确认 | 尚未在目标 NUC 验收；停止失败或连接丢失须显示未确认 |
| NUC 桥接与远程 helper 测试 | WSL Linux 18/18 通过；使用真实临时 Python 进程作为程序夹具，没有 NUC 硬件 |
| NUC 任务、API 快照固定与取消确认测试 | Windows 5/5 通过 |
| 原有后端测试复验 | Windows 18 通过、1 个 POSIX 进程组测试跳过 |
| 前端检查与生产构建 | Vue TypeScript 检查及 Vite 构建通过 |

NUC 只读检查不得启动设备。实时运行必须先由所选 NUC 程序执行
`--check-config --config`，确认配置有效且 `execution: hardware`，再启动同一程序。
设备配置必须包含实际标定、几何和设备字段；配置任务包中的日常参数不能补造这些
设备条件。SSH 使用 `known_hosts` 与密钥或 agent 无交互认证，工作台不保存密码。
远端工作目录须与项目、构建目录独立；所选程序解析后须位于所选构建目录内，
将程序绑定到已核验的构建元信息。设备标定、几何与模型使用 NUC 上的资源路径，
不使用 Windows 参数方案中的本机路径。

本轮新增 23 项测试分平台通过，只证明 SSH 桥接、任务编排、配置组合与停止确认逻辑，
不能替代 NUC 登录、实际设备配置检查、相机/通信装配、实时运行或硬件停止确认。
测试覆盖远端停止确认失败时 API 返回 422、任务保留 `running`，以及服务重启将
未完成任务标记为 `interrupted`。这些状态不宣称远端设备已停止。用户随后已断开
SSH，本轮没有继续连接 NUC。
下面的 2026-10-01 离线与已有模型记录保留其原有范围，不作为本轮实机通过证据。

## 2026-10-01：离线工作台与已有模型

2026-10-01 完成了离线工作台代码和浏览器联调。以下结果属于该轮修改，不继承主项目历史
C++ 构建的通过记录。

| 检查 | 本次结果 |
| --- | --- |
| FastAPI 后端 unittest | WSL 19/19，包括进程组取消；Windows 18 通过、1 跳过 |
| `verify_alternatives.py --self-test` | 278 项检查通过，覆盖旧 `--method` 与组合选择 |
| 前端生产构建 | `vue-tsc --noEmit` 与 Vite 构建通过 |
| C++ 模块 include 边界 | 检查通过 |
| 浏览器与真实本地 HTTP 服务 | 五个页面、135 个字段、文件选择、方案另存和互斥提示已检查 |
| 仓库配置只读 | 浏览器另存前后 `config/` 全部文件 SHA-256 相同 |
| 当前改动空白检查 | `git diff --check` 通过 |
| C++17 Release 基线，OpenVINO OFF | 完整 CTest 106/106 |
| C++17 Release I1+I2，OpenVINO OFF | 完整 CTest 108/108，含两项候选专项注册检查 |
| C++17 Release 基线，OpenVINO ON，已有双模型 | 完整 CTest 113/113，七项真实模型测试全部注册并通过 |
| 真实 HTTP 离线闭环 | 配置检查、六帧合成/回放、双配置与跨构建比较通过 |
| API / 直接 CLI 一致性 | 同构建、数据、配置快照下逻辑报告一致；耗时单独保留 |
| 单图计时 | 真实传统检测 10 次，正确读取 detector-only P50/P95 |
| 已有 YOLOv5 / YOLO11 工作台闭环 | 两路分别通过配置检查、3 次单图推理、六帧回放及批量报告 |
| YOLO API / 直接 CLI 一致性 | 两路逻辑报告一致，推理各 6 个样本，模型及完整 `config/` 指纹前后相同 |

浏览器实际将检测器选为 `traditional`，把 `common.pnp.maximum_rms_px` 改为
`2.5` 并另存。随后读取服务生成的配置包，确认数值保留、三份基础配置回指正确，
标定与几何文件均在包内。I3/ESO 同时开启，以及 I9 缺 OpenVINO 或双模型时，
页面显示原因并禁止提交。

后端测试使用真实 Python 子进程检验串行、失败和持久化状态，并使用临时输入夹具
检验配置资源快照、原资源修改隔离、取消/重启和部分结果对齐。工具命令编排测试中的
占位程序与报告夹具不是 C++ 算法验收结果。

随后 WSL 执行权限恢复，使用现有 Ubuntu 22.04、GCC 11.4.0、CMake/CTest 4.4.3、
OpenCV 4.5.4、Eigen 3.4.0、yaml-cpp 0.7.0 完成上述真实 C++ 验收。Python 3.10.12
及服务依赖置于独立 `/tmp/xtyf-workbench-venv-20261001`，未改系统 Python。
首轮两份构建均使用 C++17、Release、两路并行，OpenVINO 与相机后端关闭；
后续双模型构建保持同样语言标准、构建类型与并行数，启用 OpenVINO，相机后端仍关闭。
新增 `test_check_config` 已随完整 CTest 通过，包括非法数值、字段缺失、联动约束
以及不打开输入/输出或创建推理请求的检查。

本次证据目录为 WSL 的 `/tmp/xtyf-workbench-acceptance-20261001`，其中任务 ID：

| 任务 | ID |
| --- | --- |
| 基线 | `47ac717f65ef4269af1d4b7190bf53b2` |
| I1+I2 | `068903c3fe34411d9a67797119eaf2a9` |
| OpenVINO 基线及现有双模型 | `eb34786280604b4da2fdb3d03198ce5c` |
| 配置检查 | `592b33a32514467fb08be4b673b34fb9` |
| 合成六帧 | `c49dcb6752f04b84b5062a6c1a656f99` |
| 回放与逐帧预览 | `6806d9d35e1e4afda5821a086a614840` |
| 同构建双配置比较 | `fb7c32d66a394375be31808268d568f8` |
| 跨构建比较 | `0df35366710348ac83da0aea8e853b27` |
| 单图计时 | `08da41ec5837415f9881715b6d174640` |

完整命令、退出码、指纹、CTest 日志及报告由对应任务保留。这些临时验收材料不是
新克隆仓库的运行依赖。随后使用本机已有双模型完成 OpenVINO 构建与模型验收，
未重复下载或安装推理运行库。未连接设备，没有目标 NUC 的性能结论。

本机资源位于 `E:\大学\巡天御风\RM2026-AutoAim\assets`，WSL 路径为
`/mnt/e/大学/巡天御风/RM2026-AutoAim/assets`。当前 XTYF 工作副本中的默认
`../models/*.xml` 并不存在，因此在独立参数方案中显式选择现存路径。既有模型只读，
OpenVINO CMake 包使用现装 `/usr/lib/cmake/openvino2026.3.1`（2026.3.1.22476）。

| 模型 | 输入 | 输出 | XML / BIN 字节数 |
| --- | --- | --- | --- |
| YOLOv5 | `1×3×640×640` | `1×25200×22` | 362164 / 2836140 |
| YOLO11 | `1×3×640×640` | `1×50×8400` | 732967 / 2922484 |

两份模型布局符合当前解码器；七项真实模型测试覆盖同步、异步、整链异步及双模型
批量评测。源码原件和独立副本在构建验收前后均未改变，模型 SHA-256、实际命令和
注册检查保存在该构建任务的 `acceptance/summary.json`。角点物理顺序仍未标定；
合成样例的检出率不作为真实模型精度结论。

后续 HTTP 验收复用这份已通过构建，分别另存明确选择 YOLOv5 / YOLO11 的参数方案。
每路进行了配置检查、3 次真实单图推理、六帧回放（原图及独立叠加图 HTTP 读取）、
批量报告和同参数直接 CLI 比对。两路均记录 6 个推理样本；此组合成图上预测数为 0，
查准率、角点误差与位姿指标保持缺失值，未人为补板型、调整角点或降低置信度。

| 现有模型 | 配置检查任务 | 单图推理任务 | 六帧回放任务 | 批量报告任务 |
| --- | --- | --- | --- | --- |
| YOLOv5 | `0310a3187bdc4cb6a8694e5913909568` | `f015d25647c94afe9b52767e044ee29a` | `a5e80457cb0645e9b6434bef249fee48` | `2646968375b7489eb09fb465a5dfb891` |
| YOLO11 | `925a7e99e5204b039b5df787af4af968` | `a831c27f290e47e88479ae61aff2bc84` | `274c92ea06fc496381c6d979f86dc535` | `674e2907aa4949fca17a24fbc426c53e` |

这次六帧数据任务为 `31e9a63c83ca413c82396a1a3da229d4`。完整任务索引、逻辑比对、
直接 CLI 产物、完整配置与模型前后指纹保存在实验目录
`acceptance_checks/20261001-184404-c356722c/evidence.json`，结果为 `passed`。
页面直接展示“已通过实际模型验收”、两份模型名称和 7 项注册模型测试。
同一构建内双模型对照任务 `e4d65813a5ec4ad0a364cfde5b84635f` 也已完成，
结果页读取两套实际报告列并确认评测条件兼容，缺失指标显示“未产生”。

复验时，按 [启动说明](README.md)启动服务，再执行：

```bash
python3 -B -m unittest discover -s workbench/backend/tests -v
python3 -B tests/verify_alternatives.py --self-test
python3 -B workbench/verify_workbench.py --with-combination
```

最后一条通过 HTTP 创建新的配置、基线和 I1+I2 构建，运行完整 CTest、合成六帧、
离线回放及比较，再复核同样输入的直接 CLI 逻辑报告。配置、输出和任务均保留独立
目录，不覆盖历史实验。复用已有 OpenVINO 构建完成本机模型闭环时可执行：

```bash
python3 -B workbench/verify_workbench.py --url http://127.0.0.1:8766 \
  --build-id eb34786280604b4da2fdb3d03198ce5c \
  --yolov5-model /mnt/e/大学/巡天御风/RM2026-AutoAim/assets/yolov5.xml \
  --yolo11-model /mnt/e/大学/巡天御风/RM2026-AutoAim/assets/yolo11.xml \
  --openvino-dir /usr/lib/cmake/openvino2026.3.1
```

任务 ID 及目录属于本机临时证据，换机使用实际已有模型与成功构建，或省略
`--build-id` 创建新的 OpenVINO 验收构建。
