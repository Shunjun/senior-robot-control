# 固定随车回波返修证据

本轮任务 `task_8a48448b5365 / ctx_0a993da4d662`，功能分支 `fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。产品修改及本地集中验证完成，待负责人亲审；未暂存、提交、推送，未启动开发服务、ROS、仿真或车辆。

## 最新批准行为与实际实现

用户最终明确：固定0.05m/s、host steady3秒采集；逐帧实际角度+距离转换的相对位置在1cm内持续得到支持，可由邻近方向回波支持，不要求同数组序号或精确坐标相等。相邻有效命中点连续段包含车内种子即可延伸车外，不增加欧氏连接距离、角度阈值或亚束实体证明；不计算车辆位移，不用世界固定反假设。

SelfReturnModel在采集阶段以首次实际候选位置为固定参考，逐帧求1cm支持交集；连接规则在各帧独立检查。保留原2048点预算及最多128个结果，64帧采集身份窗口滚动后也不能遗忘更早不稳定证据。动作实际首次提交时重新建立采集段，避免把等待静止帧当行进证据；原采集来源、host标签/物理轴、重放与迟到检查保留。动作后的5个新停车帧只确认生命周期，不再改变3秒内候选。无运动采集证据不伪造完成，现有动作和安全输出期限不变。

完成后模型通过原单写owner原子保存，仅持久完成目录可交付；Attach完成模型只返回只读结果，不发写会话。运行对每个实际命中点直接做固定1cm匹配，遮挡/缺帧/无匹配后回波回归仍过滤；同一束其他距离和未匹配外部点保留普通扫描处理。不检查物体身份，不因车身/扫描几何报告撤销已完成模型。全局sensor_error/orientation_error_rad用途未改。

## 实际删除与保留

删除 `runtime/learning_mailbox.py` 全模块、撤销pipe、SCM_RIGHTS传fd、49字节memfd、IN_FRAME/IDLE/FAILED/CLOSED逐帧状态、pipe容量/PIPE_BUF探测、owner drain、EOF/PID撤销会话交接、永久撤销集合、accepted/durable撤销前沿、撤销同步与持久化、writer_state及异常退出清空模型链。删除SelfReturnRevocation、apply_revocation、filter_snapshot和LearningFilterView.deleted/frontier/producer；没有兼容开关或空壳。删除仅供学习的 `motion/learning_pose_history.py` 及StartupLearning位移/误差积分、世界固定反假设与自由支持证明；其他估计消费者的MotionPath/原始分段积分保留。

删除退役测试 `test_learning_revocation_mailbox.py`、`test_self_return_process_revocation.py`、`test_learning_recorded_path.py`；迁移dual-clock中的位姿积分专用用例，保留采集时间轴、ROS标签、迟到/重放、来源/世界重置和host3秒证据。删除MotionFeed等学习专用积分测试脚手架；原估计产品未因此删改。新增 `test_learning_frozen_reference.py`，迁移owner/client/Linux IPC、1cm、core、latch测试和当前设计/根方案。历史shared-state、recorded-path及旧测试报告均未改写为新行为通过。

保留原LearningRecordOwner唯一写入职责、RecordWriterLock稳定inode、专用ProcessWorker原子保存、LearningService/Client线程、有界队列、Linux abstract SOCK_SEQPACKET与SO_PEERCRED校验；这些实际服务结果交付和动作防重放，未误删。保留已消费动作原子预消耗、首次私有提交后host3秒、0.05m/s、控制期限裁剪、现有成功启动一次清理和退出资源回收；无每帧磁盘写入或新增模型资格门。

未新增学习接管/等待超时。学习嵌在控制进程，记录owner在supervise内；完成保存不会退出控制或监督进程。受管实车入口持 `.agent-service.lock`，仿真入口持ROS domain及根目录锁；owner文件锁只保护记录单写。直接绕过受管入口启动控制节点没有在所查Python入口发现等价进程互斥，本轮仅记录，不扩展控制单实例实现。

## 集中验证

环境：macOS Darwin arm64，CPython3.12.15，仓库 `.venv`。最终同一版本单次集中组 `/tmp/frozen-learning-final.log`、`/tmp/frozen-learning-final.xml`：**136 passed、1 skipped，50.12秒**。

```sh
.venv/bin/pytest -q tests/unit/test_learning_frozen_reference.py tests/unit/test_learning_record_owner.py tests/unit/test_learning_client.py tests/unit/test_learning_service_linux.py tests/unit/test_learning_dual_clock.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_timed.py tests/unit/test_startup_learning_review.py tests/unit/test_body_return_geometry_only.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_latch.py tests/unit/test_learning_cleanup.py tests/unit/test_motion_core_handoff.py tests/unit/test_motion_core_chain.py --junitxml=/tmp/frozen-learning-final.xml
```

覆盖：稳定车内及连续车外段、无效束断连/无车内种子、相邻方向/改变束序号支持、单帧不稳定与100帧早期证据不遗忘、1cm包含边界及外侧、未匹配外部点保留；完成→保存→同run只读恢复、1000次缺帧/遮挡/不匹配后回归、不提交运行记录写入（替换write_record为一调用即失败探针）、owner修订号与磁盘字节不变；完成后旧写会话不能替换模型、owner重新创建仍恢复固定结果、动作预消耗需持久确认且不重放、原子替换故障与符号链接防护、客户端真实线程状态机；实际core当前扫描过滤/缓存、5个原完整周期、动作期限/锁存/清理、handoff与core chain。

原完整周期场景在动作期间没有车内种子，完成的是空模型；晚出现物品保持外部点。非空模型的学习→保存→恢复由真实owner/存储和客户端线程测试覆盖，实际core非空过滤链用真实core加完成目录覆盖，不能将其宣称为“非空设备场景完整core运动周期已通过”。

静态命令：

```sh
.venv/bin/mypy src/senior_control/motion/learning_record.py src/senior_control/motion/startup_learning.py src/senior_control/runtime/learning_owner.py src/senior_control/runtime/learning_client.py src/senior_control/runtime/learning_service.py src/senior_control/runtime/learning_store.py src/senior_control/runtime/learning_wire.py src/senior_control/runtime/learning_cleanup.py src/senior_control/safety/learning_view.py src/senior_control/safety/self_learning.py src/senior_control/safety/external_motion_features.py
.venv/bin/ruff check src/senior_control/safety/self_learning.py src/senior_control/safety/learning_view.py src/senior_control/motion/startup_learning.py src/senior_control/motion/learning_record.py src/senior_control/runtime/learning_client.py src/senior_control/runtime/learning_owner.py src/senior_control/runtime/learning_service.py ros2/src/senior_robot/senior_robot/motion_core_output.py tests/unit/learning_test_support.py tests/unit/test_learning*.py tests/unit/test_startup_learning*.py tests/unit/test_body_return_geometry_only.py
```

mypy：Success, 11 source files；Ruff全部通过；同范围ruff format --check：23 files already formatted。日志 `/tmp/frozen-learning-mypy.log`、`/tmp/frozen-learning-ruff.log`、`/tmp/frozen-learning-format.log`。未降低类型检查或放宽传感误差/安全输出断言。

## 失败记录与限制

首轮 `/tmp/frozen-learning-first.log/.xml`：83通过、1失败、1跳过；完成目录恢复状态问题已修复。扩展轮 `/tmp/frozen-learning-second.log/.xml`：111通过、11失败、1跳过，包含一次测试迁移误删MotionStateMachine.observe的now实参、旧帧结构/位姿断言及旧后置分类预期，均按新契约迁移并保留有效安全断言；`/tmp/frozen-learning-core-followup.log/.xml` 定向32通过。最终整理删除积分空脚手架及一个已无估计输入的无效旧测试，最终当前版本136通过；此前137通过不混入当前计数。一次命令误列不存在的test_startup_learning_cleanup.py导致无测试执行，已纠正命令，不能算验证。

额外加场景探针 `/tmp/frozen-learning-extra-raw-gap-probe.log/.xml`：134通过、3失败、1跳过（94.88秒）。在原完整周期额外将rear index0设0.15、相邻index1及末束设NaN时，三个原正例均停在moving且action_consumed=False，未实际发放学习动作；未把它冒充学习成功，未放宽未知空间或安全输出来造通过。该附加输入同时改变了provisional self和缺束空间证据，未进一步定位限制输出的具体原因；最终集中组恢复原完整周期原始输入与既有有效安全断言。该失败探针保留为非空设备完整core周期的实际未通过限制，需负责人判断后续范围。

唯一跳过为真实Linux LearningService+LearningClient+spawn I/O端到端测试：当前macOS不支持目标abstract SOCK_SEQPACKET/SO_PEERCRED组合。撤销专用memfd/SCM_RIGHTS已删除，不再需要旧memfd测试或macOS模拟fallback；客户端到真实owner适配测试不代替目标Linux内核IPC。未运行Linux/aarch64/Python3.10/ROS/仿真/实车，未实测物理执行ACK或实时上界。

## 启动快照、实际增量和保护

启动快照 `/tmp/frozen-learning-start`，793文件及sha256.json；本轮所有返修文件相对此快照的实际增量见 `2026-11-16-learning-frozen-reference-owned.diff`，不是整个工作区相对HEAD的大量历史/其他任务改动。逐文件启动/最终SHA、删除标记及保护校验见 `2026-11-16-learning-frozen-reference-fingerprints.json`。清单包含本报告，指纹文件本身不自引用；diff不包含自身或指纹文件。

motion_core_output.py相对启动快照仅学习初始化参数、模型过滤/签名、advance实参和summary小hunk；没有覆盖时钟接线或其他输出逻辑。三保护文件相对启动快照SHA均未变：native_receipt.py、tests/test_native_receipt_diagnostics.py、docs/reviews/2026-11-16-software-estop-owner.md；原工作区脏态继续保留，无暂存。
