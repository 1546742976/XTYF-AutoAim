# 工作台验证记录（2026-10-01）

本次完成了工作台代码和浏览器联调。以下结果属于本次修改，不继承主项目历史
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
