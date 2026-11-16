# 软件急停单一 owner 与硬件解耦

起始实际 HEAD：`800d29f5de8da1c1bcb0ab769adc695def32c31a`，分支 `fix/degraded-drive-speed`。工作期间状态 Agent 提交 `91098d7`、学习 Agent 提交 `ea871ba`；本任务先单独提交共享文件的硬件删除 hunk `77f34e0`，其余修改随本报告提交。未推送。

用户后续明确覆盖旧设计：仅当前运行内存、完整启动未软件锁存、唯一软件急停 owner、删除退役链路。本次没有修改车辆物理急停、厂家代码或驱动保护，也没有发送运动命令。

## 最终职责

- `MotionStateMachine._software` 是唯一锁存，位于最终输出 `MotionCoreOutput` 内。显式、带请求身份的急停触发事件，重复同帧不推进版本；无身份状态回声不成为新事件。复位仍匹配版本与静止证据，故障单独恢复，旧手动/任务来源不重放。
- CommandNode 只保留请求缓存、在途原始帧和来源撤销。它等待新鲜 owner 状态确认交付，普通断连/清理屏障不得覆盖在途急停；该记录确认交付即消失，不等待软件复位。导航与手动操作读取 owner 的实时状态。`motion_mode` 同样读取 owner。
- Agent 只保留本进程请求内容去重及交付完成记录，不再保存急停真值、复位令牌或数据库安全意图。网页请求回执表示受理，实际锁存/复位由 owner 状态展示。

## 删除的链路

硬件 Bool 订阅及来源/超时处理、硬件事件和复位/控制周期门、硬件配置类与 schema/config 字段、`hardware_latched` 与硬件观察协议/UI；Arbiter 镜像锁存、CommandNode 反向状态锁存/过期报告锁存/复位镜像；PersistentEstop 类、线程、文件锁、文件读写、初始化工具、CLI/launch 状态文件参数及要求；Agent 数据库安全意图/回执存储、启动恢复、清除/修复线程任务与相关退役测试。

仍保留的真实消费者：请求身份/序列/来源隔离、FinalCommand 原始帧重传、运行态时效校验、任务/凭据数据库与旧任务审核机制、软件急停 owner 的版本匹配复位。旧急停文件及旧数据库表不读取、不删除、不清空；独立任务迁移测试仍检查历史数据原样保留。自定义配置需移除退役参数，不提供兼容空壳。

## 定向验证

环境：Python 3.12.15、pytest 8.4.2、Node v24.14.1、npm 11.11.0；本机无 ROS 进程。

```sh
.venv/bin/pytest -q tests/unit/test_software_estop_owner.py tests/unit/test_motion_state_machine.py tests/unit/test_motion_core_handoff.py tests/unit/test_runtime_guard_ready.py tests/unit/test_arbiter.py tests/unit/test_vehicle_state.py tests/unit/test_startup_learning_chain.py tests/integration/test_estop_process_lifetime.py tests/integration/test_manual_wire.py -k 'not real_authenticated'
# 88 passed, 1 deselected

.venv/bin/pytest -q tests/integration/test_vehicle_cli.py tests/integration/test_control_directory_cli.py tests/unit/test_rgbd_transport_config.py tests/unit/test_stopping_fused_input.py tests/integration/test_task_runner.py tests/unit/test_home.py -k 'schema_upgrade_preserves_legacy or domain_versions_upgrade or core_migration_failure or legacy_home_guard_migration or profile_schema or control_directory or profile or launch or paths'
# 48 passed, 200 deselected

.venv/bin/pytest -q tests/integration/test_manual_wire.py::test_real_authenticated_relay_agent_ipc_and_native_owner tests/integration/test_estop_recovery.py
# 3 passed；临时 Unix socket，模拟 ROS I/O

.venv/bin/pytest -q tests/integration/test_manual_wire.py::test_real_authenticated_relay_agent_ipc_and_native_owner tests/integration/test_estop_recovery.py tests/unit/test_docker_start_helper.py -k 'real_authenticated or received_estop or control_stop or handshake_failure'
# 5 passed, 10 deselected；子进程/ROS 均为测试替身

npm --prefix web test -- --run src/features/drive/drive.test.tsx src/api/vehicleState.test.ts
# 11 passed
npm --prefix web run typecheck
# passed
```

修改的 Python 文件通过 Ruff 与 compileall；`git -c core.whitespace=cr-at-eol diff --check` 通过（保留现有 CRLF 文件格式）。静态检索生产源码/配置中退役的 PersistentEstop、initialize_estop、safety_state_path、estop_state、HARDWARE_ESTOP/HARDWARE_RELEASE、confirm_local_estop 均无消费者。

初轮测试中暴露的退役方法残留调用、旧测试期望和测试导入已修正。沙箱内 Unix socket 绑定受限，必要 IPC 测试在批准的沙箱外执行；没有启动 ROS、Gazebo、服务或车辆。未跑全套、未做实际跨进程 DDS/ROS/实车验证，应用受理回执不等于物理停车证明。原有 `native_receipt.py` 和 `tests/test_native_receipt_diagnostics.py` 未修改、未暂存或提交。
