# 控制隔离新增类型错误修复与冻结

基线HEAD：`c00bc7f00a619bc8e15dccb75da9240f56b6dea0`，仍在 `fix/degraded-drive-speed`。本次仅修改六份自有产品文件及本报告，未修改测试、mypy配置、停用native文件，未提交或推送。应用/传输接口拆分已与负责人及通信worker协调，connection.py由通信worker修改，不计入自有diff。

## 实际类型契约

- `ControlCadence` 的周期/时刻为float、槽序号为int，`observe/finished -> None`、`due -> bool`；首次时钟样本建立origin的内部不变量显式检查，不改变正常槽位调度规则。
- `DiagnosticPump` 明确消费 `RuntimeDiagnostics`，线程入口及close均返回None。
- `LatestSnapshot[SnapshotT]` 的put/take保持成对的同一payload类型；仅在私有pickle反序列化边界使用 `cast(SnapshotT, ...)`，没有扩散Any或忽略检查。RawArray/RawValue改用ctypes明确类型，同Python标准库原有B/I/Q映射完全相同；Python3.10已检查三组class identity。
- `ProcessWorker[ResultT]` 将任务 `Callable[[], ResultT]`、响应槽、`_ProcessFuture[ResultT]` 及 `submit -> Future[ResultT]` 关联。响应仍是原tuple：`tuple[Literal[True], ResultT] | tuple[Literal[False], str]`，保留原序列化和错误语义，不新增报文、缓冲或生命周期路径。Event/BaseProcess/BaseContext使用标准库明确类型。
- controller的 `_still` 与 `reset_at_stillness` 明确轮速/IMU可缺失，时间为float；没有更改静止判断。
- `filter_snapshot -> tuple[FilteredObservation, SelfReturnRevocation]`，`filter -> FilteredObservation`，共享 `interior(Point2D) -> bool`。为新方法建立完整契约时同时清除了四条相关既有错误，没有改分类、阈值或撤销逻辑。
- `TaskAwareMotion.send_intent` 保持应用层三参数契约，读取车辆published_snapshot后调用 `IpcMotion.send_intent_frame`。通信worker将低层方法改为该名字且仍要求driving/driving_version必填；不再通过不兼容override混用配置选择和IPC发送，不使用可选配置假兼容。

## mypy：新增清零，既有仍保留

同环境运行命令为 `.venv/bin/mypy src/senior_control --no-incremental`；mypy 2.4.0、执行器Python3.12.15，项目配置目标Python3.10、strict未变。完整输出 `/tmp/control-typing-mypy-final.log`：**68 errors / 9 files / 159 source files**，所以完整mypy仍是失败状态，不宣称CI通过。

负责人/统一worker的ab24106基线 `/tmp/startup-baseline-mypy.log` 为72条既有错误；提交c00bc7f检查日志 `/tmp/startup-final-mypy.log` 为104条。按“路径＋AST函数/方法限定名＋错误码＋消息”逐条多重集比对，当前相对ab24106 **新增0条**，四条相关既有错误已清除；c00bc7f相对基线新增33条、移除1条（净增32），这33条新增错误均已消除。比对明细在 `/tmp/control-typing-baseline-comparison.json`，脚本 `/tmp/compare_control_mypy.py`；不按漂移后的行号简单扣减。

c00bc7f新增错误分布：cadence 5、diagnostic_pump 3、process_resources 17、controller 5、filter_snapshot 2、ports override 1，合计33；同时原SelfReturnModel.filter中的1条no-untyped-call消失，因此104-72为净增32而非新增32。剩余分布为：

| 文件 | 剩余既有错误 |
| --- | --- |
| `src/senior_control/localization/gps_course.py` | 21 |
| `src/senior_control/motion/constraints.py` | 2 |
| `src/senior_control/motion/controller.py` | 2 |
| `src/senior_control/motion/learning_pose_history.py` | 2 |
| `src/senior_control/motion/startup_learning.py` | 26 |
| `src/senior_control/safety/external_motion_features.py` | 4 |
| `src/senior_control/safety/self_learning.py` | 9 |
| `src/senior_control/tasks/lifecycle.py` | 1 |
| `src/senior_control/tasks/lifecycle_store.py` | 1 |

混合修改文件的函数级证据：`MotionController._candidate` 两条validate_output的可空ConstraintResult错误与ab24106旧625/627行完全相同；`Footprint.physical_vertices` 的返回坐标tuple形状及join_style=2两条错误来自ab24106旧183/185行，本次未改这两个函数。self_learning剩余9条均在原有constructor/clear/observe/retain_recent/validate/_observed_free，新增filter_snapshot没有错误；只补其共享interior及filter的契约，移除了原interior无注解、validate/filter调用interior无注解、filter缺返回类型四条既有错误。GPS、tasks、启动学习等既有类型债务没有顺手重构。

## 验证及冻结

自有六文件Python3.10 AST通过；`ruff check`及`ruff format --check`通过（ruff 0.17.3，6 files already formatted）；自有diff空白检查通过。没有运行pytest或全套；受影响定向测试与构建由统一验证worker负责，负责人已回报其受影响检查通过，本任务不重复全套。

自有源码清单 `/tmp/control-typing-owned.sha256` 的SHA256为 `86ea6f029e50699ac81eb8ebc437215336e6b146e54494326969677dc2b66fb6`；实际增量diff `/tmp/control-typing-owned.patch` 的SHA256为 `fdb72dff05320d3b2f5b1295db8652aa80536fef10fea8f8232762871a6fe814`。接口依赖connection.py在检查时的SHA256为 `ec71bfb1bc696c0dbf454659f90f137bb41a1abc2b31273a55ca1a5220896213`（通信worker所有）；完整mypy日志SHA256为 `a28ed312d10818e098f111637e51782018bf7568afe497ec96b08a24c44430da`。产品已冻结，后续仅补文档，由负责人审diff后统一worker锁定最终受影响验证，不继续扩大类型清理。

```text
4d84332f3a5ad058a6bc0cc5cedaa6d0703ca5c809a6616d43a34d03b56a4ba4  src/senior_control/agent/ports.py
cba369892c8b7c8ca062ca198f4aee0986245ce0ff097ceaefc91b168a7ddf6d  src/senior_control/motion/cadence.py
1bbef72443f7bb7ed28cad6aefd5473ab0b5dbec5aba58d44b24ed6f5eb5c2a3  src/senior_control/motion/controller.py
70ad6f4cc279410cc19c315111c36295909c5c0fa042d8dc2ba43ffa1da0ca18  src/senior_control/motion/diagnostic_pump.py
4c5c61c312626f5553cfa6f8d2eb52fcc543e07926fde3a3f58122d3346e603e  src/senior_control/motion/process_resources.py
c7054c96f83939007b67f9570db5f2c29cb99a19d7aa1a011e7c41af098a0e1e  src/senior_control/safety/self_learning.py
```

实际自有diff规模：

```text
 src/senior_control/agent/ports.py              |  3 +-
 src/senior_control/motion/cadence.py           | 15 +++---
 src/senior_control/motion/controller.py        |  4 +-
 src/senior_control/motion/diagnostic_pump.py   |  8 +--
 src/senior_control/motion/process_resources.py | 73 +++++++++++++++++---------
 src/senior_control/safety/self_learning.py     | 10 ++--
 6 files changed, 71 insertions(+), 42 deletions(-)
```

最终Ruff检查/格式检查原始日志为 `/tmp/control-typing-ruff-final.log`，SHA256 `2e0f68b5c9e6b4a4130b48f310e9cb308af2bac4f6456f506e5edb9ac2f75ada`；生成日志前逐个复核六文件及connection依赖hash，均与冻结值一致。
