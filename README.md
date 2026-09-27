
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
│       └── pipeline/
├── src/
│   └── ...                 # 与 include 对应
├── apps/
├── tests/
└── tools/
```

## 目录说明

| 目录 | 说明 |
|---|---|
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
|---|---|
| `core` | 时间、结果、配置、日志、基础类型 |
| `math` | 角度、变换、SE3、数值方法 |
| `hal` | 相机、云台反馈、串口、时钟抽象 |
| `vision` | 帧、检测、PnP、标定 |
| `estimation` | 观测、关联、Tracker、EKF、状态机 |
| `decision` | 预测、选板、弹道、火控 |
| `control` | 指令、平滑、协议、CRC、看门狗 |
| `pipeline` | 主循环、队列、调度、命令槽 |

## 依赖方向

```text
apps → pipeline → decision → estimation → vision → math → core
                ↘ control → hal
```

依赖只能向下，同层通过接口通信。

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