# 离线实验工作台

工作台在本机浏览器中管理参数、编译方案和离线实验。Vue 页面由 FastAPI 服务提供；计算仍由项目原有 C++ 工具完成。服务与 C++ 工具在同一个 Linux / WSL 环境运行，浏览器可以在 Windows 中打开。

## 启动

先按项目 README 安装 C++ 依赖，并准备 Python 3.10 以上（含 `venv` 支持；Ubuntu 对应
`python3-venv` 包）及 Node.js 22.12 以上
（Node.js 20.19 以上的 20.x 版本也可）。下面的命令在仓库根目录的 Linux / WSL Bash 中执行：

```bash
python3 -m venv .venv-workbench
.venv-workbench/bin/python -m pip install -r workbench/requirements.txt
npm --prefix workbench/frontend ci
npm --prefix workbench/frontend run build
.venv-workbench/bin/python -m workbench.backend \
  --source "$PWD" \
  --workspace "$HOME/.local/share/xtyf-autoaim-workbench" \
  --host 127.0.0.1 --port 8765
```

打开 <http://127.0.0.1:8765>。Node.js 只用于构建页面；运行服务时不需要前端开发服务器。工作台不会安装 C++ 依赖、自动下载模型或启动设备。

实验目录必须在源码树外。配置方案、独立构建副本、任务状态、日志和结果均保存在该目录，仓库配置和输入数据保持只读。WSL 用户在页面填写 Linux 路径，例如 `/mnt/d/datasets/events.yaml`；输出目录建议使用 WSL 的 Linux 文件系统。

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

任务按队列串行运行，避免同时运行的实验互相干扰耗时。取消操作终止整组子进程；保留已经生成的日志和未完成产物。服务重启将未完成任务标记为中断，不自动重跑，也不覆盖历史输出。

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
.venv-workbench/bin/python -m unittest discover -s workbench/backend/tests -v
python3 tests/verify_alternatives.py --self-test
```

启动服务后，在同一个 Linux / WSL 环境执行真实工具的端到端验收：

```bash
# 经 HTTP 接口构建并跑 CTest，生成数据、回放、比较，并复核直接 CLI 的逻辑报告。
.venv-workbench/bin/python workbench/verify_workbench.py --with-combination
# 已有通过验证的基线构建时，可以复用页面中的构建任务 ID。
.venv-workbench/bin/python workbench/verify_workbench.py --build-id BUILD_TASK_ID
# 使用已有模型；省略 --build-id 会创建新的 OpenVINO 基线并跑完整 CTest。
.venv-workbench/bin/python workbench/verify_workbench.py --build-id OPENVINO_BUILD_TASK_ID \
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
| 任务 | `GET/POST /api/jobs`、`GET /api/jobs/{id}`、`POST /api/jobs/{id}/cancel` |
| 日志与结果 | `GET /api/jobs/{id}/log`、`GET /api/jobs/{id}/results`、`GET /api/jobs/{id}/artifacts` |
| 文件与数据目录 | `GET /api/files`、`POST /api/roots` |
| 任务产物 | `GET /api/artifacts/{job_id}/{path}` |

任务种类为 `build`、`check_config`、`synthetic`、`replay`、`benchmark`、`single_image`、
`ctest`。任务记录保存状态、固定配置、实际命令、工作目录和退出码。状态包括等待、
运行、成功、失败、取消和中断；成功构建的注册信息与实际 CMake 参数一起展示。

工作台是独立工具层，不改变九个 C++ 模块的职责、现有 YAML 格式或离线入口的设备限制。接口、实际验证范围和已知限制应与项目主文档一起更新。
