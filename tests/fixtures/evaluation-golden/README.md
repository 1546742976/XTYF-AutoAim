# 固定评测回归素材（仅合成）

2026-09-30 使用未修改的 `synthetic_sim` 一次生成 6 帧并归档。
对应命令：`synthetic_sim --config config/offline/armor.yaml --frames 6 --output NEW_DIR`。
本目录的 armor/calibration/geometry 配置为生成时快照；input_manifest 改为本地 events.yaml。
CTest 只读取夹具，不生成、覆盖或更新 expected.yaml。

真值来自独立投影，保存在 events.yaml 和 truth.tsv；不是检测器输出。
每帧两块板，共 12 个真值。固定传统检测链产生 23 个候选，匹配 6 个，
因此 Precision=6/23，Recall=6/12；每帧都有一个匹配，最大连续零匹配长度为 0。
24 个可见角点的 RMS 冻结为 2.0466006582283107 px。
这些是当前软件的回归基线，不是应当达成的精度目标，更不是真实识别精度。

冻结环境：WSL Ubuntu 22.04、GCC 11.4、OpenCV 4.5.4、Eigen 3.4、yaml-cpp 0.7，
C++17、OpenVINO OFF。`test_evaluation_golden` 另有可手算的匹配、1 px 角点误差、
未标注/负样本及连续漏检断言，防止只有输出自比。
浮点值按此环境精确比对；变化时先调查原因，禁止自动重录或放宽阈值。
标注中的 synthetic-projection 位姿仅属合成来源；本回归不启用位姿评测参数。
