# 启动学习可行性取证（未交付）

版本：基线 `723a0fc255a1b9874af7041d0d1cb14c233df168`，`fix/degraded-drive-speed`，仅新增计划和离线探针，生产行为未变。设计及阻塞见 [实施计划](../superpowers/plans/2026-11-15-startup-self-return-learning.md)。

命令（共享既有虚拟环境，无安装、无服务）：

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_startup_learning_feasibility.py tests/unit/test_self_filter.py -q --tb=short
```

结果 **10 passed, 1 failed**；失败为原有 `test_uncalibrated_mask_is_unusable`，它要求缺旧 calibrated 标志抛错，但当前生产 validator 已显式忽略这个历史标志。未改该旧测试或兼容链；与新增探针无关，不报告整组通过。

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_startup_learning_feasibility.py -q --tb=short
git diff --check
```

结果 **3 passed**，diff检查通过。探针覆盖基线内部 hit 的实际 ConstraintEngine 阻塞、直接用 polygon 替代旧 mask 导致外部障碍消失、静止重复观测不增加归属证据。测试是合成输入调用现有纯模块，不是ROS、实车或自动学习验收；未修改生产代码，也没有RED/GREEN功能实现。

未覆盖/未实现：启动实链、急停/中断、一次性及重复启动恢复、实际学习分类、平行墙/跟随物/空转运动证据、设备变化撤销、GPS精度门及接线。前方障碍的旧mask反例只证明迁移风险，不能声称新路径安全。完整ROS、SCF3/823及ScanMotion双capture同版重建、实车仍Draft，未重复运行前序测试。独立兼容链清理、1米前障碍保守扫掠等继续排队。

SHA256：

- `tests/unit/test_startup_learning_feasibility.py`: `89f0cd14d7ba214be739f47ee25601c8f4defcc440024d785e6f25a2f81e379d`
- `docs/superpowers/plans/2026-11-15-startup-self-return-learning.md`: `9dca9de2df8bd317fbe72dc6ef6a0305082bcaabb9a9b5ef0137b3ff05198e5e`

结论：本项未完成，不能接入无依据首次前移；向负责人以 failed 回报具体首次运动证据缺口，不把测试通过冒充功能交付。
