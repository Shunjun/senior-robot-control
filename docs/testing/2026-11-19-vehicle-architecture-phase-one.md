# 第一阶段定向验证

基线 `main@cfbc5e4`，功能分支 `feat/vehicle-architecture-refactor`。记录对应本文件所在提交，未启动服务、未部署、未推送。

环境：macOS arm64，CPython 3.12.15，pytest 8.4.2，ruff 0.17.3。工作区原无虚拟环境，uv 尝试因沙箱缓存权限和网络 DNS 失败，最终复用父仓库 `.venv/bin/python` 和 `.venv/bin/ruff`；未改依赖文件。

以下以 `PY='/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python'` 表示实际解释器。

```sh
"$PY" -m pytest -q tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_native_output_protocol.py tests/unit/test_motion_core_perception.py tests/unit/test_vehicle_capabilities.py tests/unit/test_task_terminal_release.py::test_terminal_identity_does_not_revoke_current_manual_or_touch_output_frontiers --disable-warnings
```

结果：42 passed, 6 skipped（4.21 秒）。六项为现有 native 生产接收协议 Linux 限定用例，macOS 未执行；native 无 ROS 头文件的编译/确定时钟 harness 与 Python 包字段测试在本次通过集合内。最终能力适配器装配参数调整后单独重跑能力入口测试，结果见提交交付记录。

选定场景覆盖：

- 生产 MotionCore 实际组装能力门面，不可用入口无运动；新增 1 场景。
- 未确认地图不能导航；新增 1 场景。
- 同物理帧原始/补点分离、手动接管后同 run 不恢复、原净距与速度行为、40 字节原字段和停止协议：复用原学习链、感知及 native 协议用例，不新增场景。仅将 monkeypatch 目标迁至职责所有模块，未更改断言。
- 旧 terminal release 不撤新手动权限：迁移同名旧用例到当前学习链真实 core fixture，保留目标身份、代次、输出前沿、输出次数及控制状态断言；用现有 `_output` 与 sequence 替代已经不存在的 controller/scheduler。

新增/实质迁移合计 3 场景；其余是既有场景的调用位置迁移。无额外 bug 场景。

已观察的旧失败：`test_learning_core_surface_review.py` 收集时导入已不存在的 `test_full_core_cycle_with_delayed_and_reordered_raw_scans`；`test_motion_core_chain.py` 和未迁移的 `test_task_terminal_release.py` fixture 引用基线已不存在的 `MotionCoreOutput.ProcessWorker`。这些失败没有通过删除保护或填造属性掩盖，登记留待旧用例迁移；所选 terminal 场景单独迁移并已通过。

未验证：完整旧测试集、真实 ROS DDS 图、实际传感器/实车、Linux 六项协议场景、GPS 硬件、地图/定位/导航真实提供者完整装配、跨进程隔离、阶段二算法和阶段三整链。通过本地回放不代表车辆部署验收。

静态检查：对所有改动 Python 文件运行 ruff check；`git diff --check`。没有新增测试矩阵或其他功能场景。
