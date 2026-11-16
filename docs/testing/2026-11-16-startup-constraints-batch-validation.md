# 第1—6项整批测试迁移与统一验证

日期：2026-11-16。功能分支 `fix/degraded-drive-speed`，开始 HEAD `ab241065e9446e933155bff5b1b58badc6d5e2cd`。

**当前状态：第1—6项本地测试迁移和统一验证已完成，可执行范围内无剩余失败；环境跳过项与整车验收未验证。** 没有提交、暂存、推送、启动开发服务、ROS、仿真或车辆，也没有向真实车辆发送运动命令。第7—15项参数和分类行为没有扩大修改。

## 环境与版本

- macOS 26.4 / arm64；Python 3.12.15，pytest 8.4.2，pytest-asyncio 0.26.0。
- Pydantic 2.14.1，Shapely 2.2.0，NumPy 2.5.4，Ruff 0.17.3。
- Node v24.14.1，npm 11.11.0，Vitest 5.0.5；TypeScript 由项目锁定依赖运行。
- Apple clang 21.0.0，目标 `arm64-apple-darwin25.4.0`。没有 rclpy、OpenCV/cv2、aiortc；本机 AF_UNIX / SOCK_SEQPACKET 返回 `OSError(43, 'Protocol not supported')`。
- Python 3.10仅做语法 AST 检查，没有声称在Python 3.10运行测试。

## 测试边界和迁移

共享构造器迁移删除ResponseBounds/RiskEnvelope静态期限和ControllerConfig退役字段，保留传感器、感知、指令与输出证据的原期限。运行报告不再用150ms或旧healthy/allowed_actions快照拒绝命令；来源、版本、序号、显式急停交付、终端身份及障碍保护继续断言。

`control_process_test_support.py` 将ROS发布替换为记录transport，将接收侧独立MotionSensorRuntime的估计和位姿，经真实LatestSnapshot序列化及ControlInputs.update交接给真实core；没有恢复产品的ingress_factory/core_reference/_on_reference路径。确定性几何资源执行真实任务函数，不返回伪造许可；真实spawn、真实GEOS结果、完整接收快照序列化、忙锁、容量、故障与回收另有专门测试。历史JSON位姿缺少sequence/received_ros时，仅测试边界使用确定性采集时间身份和更晚的原快照ROS时钟保守补齐，不续期、不补造位姿；生产NativeObservations本身已带的字段保持原值。

新增覆盖：400ms独立请求未决时100ms意图继续发送且不增加intent pending/回执/重试；无报告的独立control.connection身份；Agent与IPC最新值槽的清理和原300ms截止；实际控制timer跨输入/预取/计算耗时跳槽与实际dt；真实spawn CPU占用期间父进程心跳/快照更新；10/20Hz持续扫描配80/120ms慢几何的非零verified输出、新障碍立即收紧、来源/坐标/过滤版本换代；主owner收到设备移除/新障碍撤销后的回波不能重新被遮罩；已经开始的学习不因geometry pending退出；只有新鲜原始轮速/IMU才静止，无独立body仍能真实core复位且旧意图不能复活。

持续扫描的慢执行资源采用可控时钟、真实几何/约束/控制计算；真实spawn资源另测。这些证据不等于实际ROS多源重载、最坏执行时延或硬实时验收。

## 已执行命令和结果

以下为实测输出，失败未计通过；失败子集与新改动才复测，没有逐项修改后例行重复整套。原始日志/JUnit位于同机`/tmp`，节点级对账在`/tmp/batch-merged-evidence.json`，所有记录均可由相应pytest节点重现。

| 阶段 | 命令/选择范围 | 实际结果 | 原始证据 |
| --- | --- | --- | --- |
| 首轮统一Python | `.venv/bin/python -m pytest tests/unit tests/integration -q --tb=short --junitxml=/tmp/batch-python-initial.xml` | 2121 passed, 183 failed, 12 skipped, 59 errors；错误含旧fixture teardown及沙箱本地socket拒绝 | `/tmp/batch-python-initial.log` / `.xml` |
| 集中迁移失败项 | pytest首轮187个失败nodeid，沙箱外仅允许本地IPC/子进程测试 | 139 passed, 48 failed | `/tmp/batch-python-rerun.log` / `.xml` |
| 再次迁移失败项及新进程/timer测试 | pytest剩余48项 + control_cadence_gate/process_resources | 53 passed, 8 failed | `/tmp/batch-python-rerun2.log` / `.xml` |
| 余下纯fixture/ROS边界 | pytest余下6项及3个相关ROS测试模块 | 3 passed, 3 failed, 15 skipped | `/tmp/batch-python-pure-final.log` / `.xml` |
| 纯fixture修正 | pytest上轮3项 | 2 passed, 1 failed；缺少接收位姿交接进一步定位后保留原正向断言 | `/tmp/batch-python-pure-final2.log` / `.xml` |
| 接收位姿共享fixture改变后的依赖复测 | 27个文件，列表`/tmp/batch-core-fixture-dependent-files.txt` | 298 passed, 4 failed；其中3个fixture身份/退役报告断言随后修正通过，持续扫描正向失败保留 | `/tmp/batch-core-fixture-final.log` / `.xml` |
| 最后3个fixture来源/所有权修正 | reference_clock_frontier的epoch、terminal_release_wire的实际来源保护 | 4 passed | `/tmp/batch-last-fixtures.log` / `.xml` |
| 急停返修冻结版 | `.venv/bin/python -m pytest -q --tb=short --junitxml=/tmp/batch-safety-final.xml tests/integration/test_shutdown.py tests/integration/test_estop_recovery.py tests/integration/test_estop_process_lifetime.py tests/integration/test_manual_wire.py tests/integration/test_agent.py` | 112 passed, 1 failed；唯一失败为新增取消用例首先撞上请求自身超时的日志措辞；下行单项验证取消分支 | `/tmp/batch-safety-final.log` / `.xml` |
| 阻塞IPC关闭取消分支 | test_shutdown.py::test_blocked_estop_transport_has_bounded_release_and_explicit_unconfirmed_log；在请求已进入0.3s等待后以0.03s关闭预算验证取消 | 1 passed，0.23s | `/tmp/batch-safety-cancel-final.log` / `.xml` |
| 无body真实复位及300ms排队截止 | software_estop_owner新增raw-only reset、manual_wire新增queued cutoff | 2 passed | `/tmp/batch-extra-crosslayer.log` / `.xml` |
| 主owner回波撤销 | self_return_process_revocation新增core真实_filtered_scan两场景 | 2 passed | `/tmp/batch-core-filter-owner.log` / `.xml` |
| 真实IPC通知与请求隔离 | `.venv/bin/python -m pytest -q tests/integration/test_intent_ipc_stream.py` | 1 passed | `/tmp/batch-ipc-stream-final.log` / `.xml` |
| 扫描scope换代与有界待办 | `.venv/bin/python -m pytest -q tests/unit/test_scan_geometry_scope.py` | 3 passed | `/tmp/batch-geometry-scope.log` / `.xml` |
| 控制器raw轮速/IMU复位命名/输入迁移 | motion_controller的raw reset与旧意图不恢复 | 2 passed | `/tmp/batch-controller-raw-reset.log` / `.xml` |
| 退役意图回执协议 | test_protocol.py::test_retired_intent_receipt_is_rejected | 1 passed | `/tmp/batch-protocol-retired-receipt.log` / `.xml` |
| 起步可用性冻结版定向 | `.venv/bin/python -m pytest -q --tb=short --junitxml=/tmp/batch-startup-availability-focused.xml tests/unit/test_motion_core_chain.py tests/unit/test_continuous_scan_control.py tests/unit/test_perception_window_union.py` | 13 passed, 4 failed；新增direct-ingress fixture漏带真实laser首/末采集时钟，补齐生产要求的原始时钟后下行四项通过 | `/tmp/batch-startup-availability-focused.log` / `.xml` |
| 真实位姿跨anchor四组合 | `.venv/bin/python -m pytest -q --tb=short tests/unit/test_continuous_scan_control.py -k real --junitxml=/tmp/batch-cross-anchor-final.xml`（选择real wheel-pose四参数节点；以下附JUnit完整节点可重建等价argv） | 4 passed，9.16s；未放宽误差、扫描、正向输出 | `/tmp/batch-cross-anchor-final.log` / `.xml` |
| 最终两处车端返修影响范围 | `python3 /tmp/run_batch_availability_affected.py`；内部以pytest -q --tb=short及下附43文件列表运行 | 584 passed, 1 failed, 2 warnings，102.61s；唯一失败是诊断测试硬编码4次尝试，prefetch修复后真实3次；改为观测真实调用次数并逐一精确计数 | `/tmp/batch-availability-affected.log` / `.xml` |
| 诊断精确计数及进程资源 | pytest计数修正节点 + test_control_process_resources.py，JUnit为batch-final-diagnostic-resource.xml | 11 passed，1.43s，无警告 | `/tmp/batch-final-diagnostic-resource.log` / `.xml` |
| fixture资源回收 | `.venv/bin/python -m pytest -q --tb=short -W error tests/unit/test_control_process_resources.py tests/unit/test_degraded_manual_speed.py --junitxml=/tmp/batch-resource-cleanup-final.xml` | 69 passed，2.97s；显式关闭mailbox，安全点回收已关闭Future循环引用，未过滤/抑制警告 | `/tmp/batch-resource-cleanup-final.log` / `.xml` |
| 首轮统一前端 | `npm --prefix web run test -- --run` | 157 passed, 12 failed；fixture缺control.connection等，另1个测试失败收尾导致的未处理拒绝 | `/tmp/batch-web-initial.log` |
| 集中修正前端失败文件 | `npm --prefix web run test -- --run src/api/client.test.ts src/features/drive/drive.test.tsx src/features/devices/devices.test.tsx` | 3 files / 96 passed，无未处理错误；加首轮其余文件证据，169项均已验证 | `/tmp/batch-web-rerun.log` |
| 前端静态 | `npm --prefix web run typecheck` | tsc --noEmit通过 | `/tmp/batch-web-typecheck-final.log` |
| 测试静态 | Ruff check（实际路径清单`/tmp/batch-test-python-files.txt`）、Python3.10 grammar AST、保留CRLF的git diff --check | 64个Python测试/支持文件通过；新急停main.py由开发worker另有静态核查 | `/tmp/batch-static-final.log` |

一次失败子集命令曾因已改名nodeid不匹配而在收集阶段退出，未执行测试，未计为通过；修正节点后才得到上表139/48结果。首轮沙箱拒绝本地Unix socket的用例在获准的本地测试环境重新执行，环境拒绝未被改写为skip或通过。

最终逐节点对账：2364项已收集，**2341 passed、23 skipped、0 failed、0 error、0 missing**，没有遗留旧名称节点。另4个依赖cv2的模块在收集时跳过，没有计作通过；数字为各迁移/返修阶段每个节点最新有效证据的合并，**不是同一命令一次全绿结果**。最终两处车端返修影响的43个文件已补跑，未受影响且产品同版的通信急停测试未例行重跑。

## 产品缺陷与原始失败证据

### 已修复：容量故障吞掉已接受急停

旧`_drain_safety`在runtime_failed且stopping时提前return，队列先接收64条急停、随后overflow触发关闭而尚无inflight时，唯一真实MotionStateMachine owner仍idle。原失败：

```text
FAILED test_newer_or_other_session_estop_is_not_swallowed[True-False-page-a]
FAILED test_newer_or_other_session_estop_is_not_swallowed[True-False-page-b]
assert rig.machine.state.mode.value == "estop"
AssertionError: assert 'idle' == 'estop'
```

原通信worker返修并由负责人亲审；测试没有增加第二owner或成功替身。返修后8参数组合及关闭回归通过，新增“满容量后立即_release”验证64条已接受急停均到达唯一owner，真实transport失败/取消有明确未确认日志且没有成功回执。

### 已修复：真实位姿交接下持续扫描输出归零（保留原失败）

```text
FAILED tests/unit/test_motion_core_chain.py::test_native_recovery_then_intent_and_local_release_are_actually_consumed
assert core.node.emitted[-1][0].vx > 0
AssertionError: assert 0 > 0
reason='fused_reference:sweep_unknown'
```

原场景保留：720条10m扫描，sensor_x=0.15699、sensor_y=0.00201，原profile误差、轮速/IMU/laser零速和逐帧原生位姿。位姿sequence/received_ros来自生产NativeObservations，没有测试补写或覆盖。`/tmp/debug_batch_chain.py`复现显示原selected几何能给0.2/0.15/0.1/0.05许可，最终空间跨wheel pose anchor后software_alignment_error_m=0.06并持续sweep_unknown；普通startup disabled路径还因为_learning_geometry为None永久跳过prefetch。已由原车端worker返修并由负责人亲审：普通驾驶prefetch不再依赖_learning_geometry；同一观测的窗口先union再以相同error_m侵蚀，消除人工窗口接缝。测试未改变原场景、误差或正向断言；原core链正向及新增真实轮速位姿跨anchor的10/20Hz×80/120ms四组均通过，跨anchor误差仍至少0.06。新增人工接缝、真实未知间隙、严格误差边界和no-return语义7项通过，disabled学习prefetch通过。

## 指纹与未验证项

- 最终车端冻结37产品文件清单：`/tmp/control-runtime-startup.sha256`，清单SHA256 `3cf341215cf81a9a2c53d1ecaf218727d839ca6aaa5dedfdcf8a481399acd5b2`，37项全部匹配；取代先前review基线。
- 车端37项与通信额外10项共47个文件最终内容组合SHA256为`802b67bc1707c9651ca328d37d741552389564796caddb8d7d281b8f66ff91e2`，算法为按路径排序的`path + NUL + bytes + NUL`；最终定向运行前后和交付前均匹配，逐文件摘要见末尾。
- 急停容量返修：`src/senior_control/agent/main.py` SHA256 `1af59786e41a733d00840f2f0d6d4efd9f4a747202afe91e80f56a9db8debd11`。
- 两份停用native诊断改动及既有急停报告的内容hash均与本worker开始时一致；没有编辑、暂存、复制其内容或夹带报告。
- 23项测试skip：6项Linux SOCK_SEQPACKET原生输出协议运行、1项Linux peer credentials、1项aiortc、15项相关ROS测试。另4个cv2依赖模块未收集。原生C++使用实际生产.cpp和记录型ROS头编译通过；Python实际pack/原期限/序号、缺可执行文件显式失败及传输失效通过。这不是真实ROS/Humble/aarch64构建或原生EOF/溢出运行通过。
- 未启动ROS、车辆、开发服务或真实运动链。没有实际ROS spawn/DDS发现/报告发布、真实多源负载耐久、超过一小时墙钟耐久、物理watchdog/制动距离/停车时限证据。静态模型跨7200s为受控时钟逻辑测试，动态观测和命令原期限继续过期。

## 本worker实际修改文件

以下仅测试/测试支持文件；本报告另位于`docs/testing/2026-11-16-startup-constraints-batch-validation.md`。产品修改由开发worker完成。

- `tests/fixtures/runtime_guard_ready.json`
- `tests/integration/agent_fakes.py`
- `tests/integration/test_agent_snapshots.py`
- `tests/integration/test_cloud.py`
- `tests/integration/test_estop_recovery.py`
- `tests/integration/test_intent_ipc_stream.py`
- `tests/integration/test_limited_manual_ingress.py`
- `tests/integration/test_manual_wire.py`
- `tests/integration/test_runtime_reporting.py`
- `tests/integration/test_settings_logs_review.py`
- `tests/integration/test_short_admission.py`
- `tests/integration/test_shutdown.py`
- `tests/integration/test_video.py`
- `tests/ros/test_motion_core_acceptance.py`
- `tests/ros/test_native_motion_executor.py`
- `tests/support/native_output/geometry_msgs/msg/twist.hpp`
- `tests/support/native_output/protocol_harness.cpp`
- `tests/support/native_output/rclcpp/rclcpp.hpp`
- `tests/unit/control_process_test_support.py`
- `tests/unit/test_acceptance_manual_exchange.py`
- `tests/unit/motion_reference_test_support.py`
- `tests/unit/test_agent_ports.py`
- `tests/unit/test_constraint_scheduling.py`
- `tests/unit/test_continuous_scan_control.py`
- `tests/unit/test_control_cadence_gate.py`
- `tests/unit/test_control_process_resources.py`
- `tests/unit/test_degraded_manual_speed.py`
- `tests/unit/test_directional_constraints.py`
- `tests/unit/test_drive_challenge.py`
- `tests/unit/test_estimation_review_contracts.py`
- `tests/unit/test_laser_reference_publication.py`
- `tests/unit/test_limited_manual_rearm.py`
- `tests/unit/test_limited_motion_core.py`
- `tests/unit/test_motion_contracts.py`
- `tests/unit/test_motion_controller.py`
- `tests/unit/test_motion_core_chain.py`
- `tests/unit/test_motion_core_event_time.py`
- `tests/unit/test_motion_core_exit_deadline.py`
- `tests/unit/test_motion_core_handoff.py`
- `tests/unit/test_motion_core_startup.py`
- `tests/unit/test_motion_core_stop_fallback.py`
- `tests/unit/test_motion_guard_to_owner.py`
- `tests/unit/test_motion_reference_backlog.py`
- `tests/unit/test_motion_ros_twist_output.py`
- `tests/unit/test_motion_trajectory.py`
- `tests/unit/test_native_output_protocol.py`
- `tests/unit/test_navigation_collision_qualification.py`
- `tests/unit/test_navigation_command_clock.py`
- `tests/unit/test_perception_window_union.py`
- `tests/unit/test_protocol.py`
- `tests/unit/test_raw_capture.py`
- `tests/unit/test_reference_clock_frontier.py`
- `tests/unit/test_reference_nanosecond_boundary.py`
- `tests/unit/test_relative_interval_regressions.py`
- `tests/unit/test_rgbd_parent_ledger.py`
- `tests/unit/test_runtime_guard_ready.py`
- `tests/unit/test_scan_geometry_scope.py`
- `tests/unit/test_self_return_process_revocation.py`
- `tests/unit/test_software_estop_owner.py`
- `tests/unit/test_startup_learning_chain.py`
- `tests/unit/test_startup_learning_cycle.py`
- `tests/unit/test_startup_learning_latch.py`
- `tests/unit/test_startup_learning_review.py`
- `tests/unit/test_stopping_fused_input.py`
- `tests/unit/test_terminal_native_channel.py`
- `tests/unit/test_terminal_release_wire.py`
- `tests/unit/test_tick_evaluation.py`
- `tests/unit/test_unified_motion_runtime.py`
- `tests/unit/test_wheel_imu_stillness.py`
- `web/src/api/client.test.ts`
- `web/src/api/vehicleState.test.ts`
- `web/src/api/vehicleState.testUtils.ts`
- `web/src/features/devices/devices.test.tsx`
- `web/src/features/drive/drive.test.tsx`

## 最终命令选择与静态核验

首轮交付收集命令：`.venv/bin/python -m pytest --collect-only -q tests/unit tests/integration tests/ros/test_motion_core_acceptance.py tests/ros/test_native_motion_executor.py tests/ros/test_agent_adapter_shutdown.py`，2359 tests collected，1.18s。`/tmp/merge_batch_evidence.py`读取上表各JUnit，以完整nodeid覆盖较早证据，最终没有缺失、失败或废弃节点；三处迁移命名映射仅为同一测试重命名。

静态命令为`.venv/bin/ruff check`加所有64个已修改Python测试/支持路径；同一列表各文件使用`ast.parse(text, feature_version=(3,10))`；`git -c core.whitespace=cr-at-eol diff --check -- tests`、五个web测试/支持路径及本报告。保护文件由`shasum -a 256 -c /tmp/batch-protected.sha256`校验三项一致；最终产品由`shasum -a 256 -c /tmp/control-runtime-startup.sha256`及47项逐文件SHA256比较校验。

43文件补跑的实际argv为`.venv/bin/python -m pytest -q --tb=short --junitxml=/tmp/batch-availability-affected.xml`后接以下每个路径：

```text
tests/unit/test_constraint_scheduling.py
tests/unit/test_continuous_scan_control.py
tests/unit/test_control_process_resources.py
tests/unit/test_degraded_manual_speed.py
tests/unit/test_directional_constraints.py
tests/unit/test_estimation_review_contracts.py
tests/unit/test_laser_reference_publication.py
tests/unit/test_limited_manual_rearm.py
tests/unit/test_limited_motion_core.py
tests/unit/test_motion_controller.py
tests/unit/test_motion_core_chain.py
tests/unit/test_motion_core_event_time.py
tests/unit/test_motion_core_exit_deadline.py
tests/unit/test_motion_core_handoff.py
tests/unit/test_motion_core_perception.py
tests/unit/test_motion_core_startup.py
tests/unit/test_motion_core_stop_fallback.py
tests/unit/test_motion_guard_to_owner.py
tests/unit/test_motion_reference_backlog.py
tests/unit/test_navigation_completion_handoff.py
tests/unit/test_perception_window_union.py
tests/unit/test_prepared_control.py
tests/unit/test_reference_clock_frontier.py
tests/unit/test_reference_nanosecond_boundary.py
tests/unit/test_relative_interval_regressions.py
tests/unit/test_scan_geometry_scope.py
tests/unit/test_self_return_process_revocation.py
tests/unit/test_software_estop_owner.py
tests/unit/test_startup_learning.py
tests/unit/test_startup_learning_chain.py
tests/unit/test_startup_learning_cycle.py
tests/unit/test_startup_learning_feasibility.py
tests/unit/test_startup_learning_latch.py
tests/unit/test_startup_learning_review.py
tests/unit/test_stopping_fused_input.py
tests/unit/test_task_terminal_release.py
tests/unit/test_terminal_collection_capacity.py
tests/unit/test_terminal_native_channel.py
tests/unit/test_terminal_release_collection.py
tests/unit/test_terminal_release_wire.py
tests/unit/test_tick_evaluation.py
tests/unit/test_unified_motion_runtime.py
tests/unit/test_wheel_imu_stillness.py
```

## 最终产品逐文件SHA256

```text
1b8c304d8165900b4ff7b7b6f6c4a2f599a23ea8d1a64c245579f14e0f1c2af5  config/hardware-motion.json
112b3ff30b0deb04924295db260ffae6ee112156643b9235b55f290d68b060ed  config/hardware-motion.schema.json
08fe48e7b77ae24c5c6f7a6afcd096cb67554cfcb15963f50a963e0657ffed7e  config/hardware-safety.yaml
71a8fdb5312e5cdc90c47c821556303c787c925c19db9cf8a998447c2fab3528  ros2/src/senior_motion_msgs/CMakeLists.txt
f139d856f58688fe2cfc056fc5628a4e0a7dcfe97abea0290a9192c672cae1a6  ros2/src/senior_motion_msgs/src/motion_output_guard.cpp
3a6c099c20b8ba48d5f7cc2458fa3873c9f1cc8765ff9fc4f058588858feec43  ros2/src/senior_robot/config/simulation_motion.yaml
3ee38bb2068843b3a1dfd379446b70901aa97439db7c553cce85e7284df9c755  ros2/src/senior_robot/launch/safety_motion.launch.py
b5f9359dab44be03b0a5579b17d9eb388332776fd4afcb32a20e61e06053aa53  ros2/src/senior_robot/launch/simulation_motion.launch.py
e2d07c5f8d29d1d2abe6bb798c456cc65bdc6ae6f646e8e07265a405d45b6dd9  ros2/src/senior_robot/senior_robot/agent_adapter.py
320bd2381e3681b2fbceeff9efe03224e39f1192a648b5cd9942397be940b277  ros2/src/senior_robot/senior_robot/command_node.py
adb44124bb1fd12f3050fe8c270767bd37ecdc94b8a9b49f8d0942bbc8ed2458  ros2/src/senior_robot/senior_robot/control_inputs.py
edd70f57bcba6eb84df71587edb29e4c5eb7a27fbc640c5f1cf455826f4eff01  ros2/src/senior_robot/senior_robot/motion_core_output.py
4d07cad83a1d946bdfb8e609aa68c36fd646d92d4d2ad00a9a94a8216a0c937b  ros2/src/senior_robot/senior_robot/motion_core_perception.py
95dc4f648218ab088dc80c596b3e629574ed96fcfb849f19343855676b3415b0  ros2/src/senior_robot/senior_robot/motion_output.py
8e98a5d55314ce2a63afef5f4785de46b88f9c0a1888f6bdb208494d45e33424  ros2/src/senior_robot/senior_robot/motion_sensor_runtime.py
7153e615f4a7e49d2d527163f18b962629fc981bf3e090c328483c7f01180460  ros2/src/senior_robot/senior_robot/runtime_reports.py
fb8e4ed3141610aade0ab6b532a60e1f3b3d1fed21d8a8f6dd809ae72149c21a  ros2/src/senior_robot/senior_robot/safety_node.py
ad78ab5fca33825f2fe7ff542c9d32b81f008d15f6d10eef2c6e5add48e6c346  ros2/src/senior_robot/senior_robot/simulation_motion_profile.py
4ca31643a426f523d5b1b1fec6c0ab301570b24bc304caada475b9fca1fab344  ros2/src/senior_robot/senior_robot/simulation_reference.py
a13a3a33a34450806bba8586d4c8026286df0fb4e97f7e9aaaca120eb4441d40  src/senior_control/agent/composition.py
828269df733896385a46e9fe3461db86db9224e01734db57e3d943be808244d8  src/senior_control/agent/connection.py
1af59786e41a733d00840f2f0d6d4efd9f4a747202afe91e80f56a9db8debd11  src/senior_control/agent/main.py
15efbfe6039c8f4c8645d8ddc4044126813a8f31da2b03de3cab02715ef5e6cb  src/senior_control/agent/manual_control.py
d28ba050ba2933bc2be267bafb28a1b2b4f5575548ad1691312f3ddad83cfaa4  src/senior_control/agent/ports.py
1b7c61aa9d072ea30409cb9e914718d898376c1cfa5d97dc47d70284b7ee9fae  src/senior_control/agent/settings_wire.py
96f2e5d4ce6fe6c26d381cfca8c26c7e136af5b727c0564da55aa56a65abd4d7  src/senior_control/cloud/relay.py
aecbf1fbb7efce864d4a77a302ce4795f423d9a6c6910d3c1f41c43927aa1973  src/senior_control/motion/cadence.py
a524a29de4488959be46f5a260d6913e9ec1e8c8f0c4c0e89c3cec7d3ed5cec0  src/senior_control/motion/constraint_worker.py
8c22558c333129bce844c3ee0bc88c421b7d97d695ce9398f76f58c59319cae1  src/senior_control/motion/constraints.py
426aab3acabe101a393bd167d45fb24c0b8c92b171f00a428a51700c828f27fb  src/senior_control/motion/contracts.py
54573d7b947fe1bdfc6a2bceb31afaf0eb2597c4c89e7d4853f4002115cd8cd1  src/senior_control/motion/controller.py
86c89d5c17a1d4a225cf4c59d29e0cde6e9bc6e158e680b71d3873b4592531e2  src/senior_control/motion/diagnostic_pump.py
1f03d81f11f21ce5aa7ace784daf5c70bff3c3b281de14539ddaae4af2b846c6  src/senior_control/motion/hardware_profile.py
66aa5f7ba5189858fd0f2ae559615ef27250b5bf4ace81382abb4ba8f2718dfe  src/senior_control/motion/process_resources.py
98660c75773e4a3c5420fb073196ffd5101aa6ff87ba47271dfbf77877ad5ac8  src/senior_control/motion/startup_learning.py
740fc6aac11e6dab61c954f721f5cc9d81cc6875a45d3367d5031ccf0354b76e  src/senior_control/motion/state_machine.py
7afb168b66ca5c8567bac3686327041f7702385ed43e309d518343b1a923146d  src/senior_control/motion/stillness.py
ba909e96ec97343b2af2299296e71068cffcdae4172230ef226dca0507862a72  src/senior_control/protocol.py
b64e6a7687128ccc699ea60d232c56bb22a0935b41e4531826073630fc9d7e96  src/senior_control/real_runtime.py
0515be5dc9248ab52d1d3b0962d0ee74c600a3f53e6485fadcff20e59c27a3cc  src/senior_control/runtime/ipc.py
76f2777ce7cf649283d61d62190a90423b804d4504e781c80d2a2d55a2da9d3d  src/senior_control/safety/self_learning.py
7e3904dbe9bf6c2b42cf2768ee3cba1b9f3aedaef80eb33e884e66f6940fa26d  web/src/api/client.ts
e9245bb2ae6880a04bba9ee454a87d46d8c827540850cbb69064f2e3dbe0b29b  web/src/api/protocol.ts
8fe049d3e85010c55eff8a122b65906e569020ff45cdf6061f0c196b5269bb3d  web/src/api/vehicleState.ts
b9f31d30a113f73cb5dcc795f083e7b2a7cb960aa6863b7ab4712fe728aafbd1  web/src/features/devices/VehicleStateView.tsx
6b6d8484d904c7c9ed2ac3530d855011cf439257309d9694e46f279be5af61dc  web/src/features/drive/DriveView.tsx
d1bbf2678f8d6774577fba522e29e03c5faa248ce0d26c6e8e8dd2a3621e14d3  web/src/features/drive/ManualController.ts
```

## 最后修正项的精确节点

下列为对应JUnit记录的实际节点；以pytest加这些节点可重建相同选择，避免将日志摘要误认为额外全套运行。

`batch-cross-anchor-final`：

```text
tests/unit/test_continuous_scan_control.py::test_slow_scan_with_real_wheel_pose_anchors_and_mailbox_eventually_drives[0.08-10]
tests/unit/test_continuous_scan_control.py::test_slow_scan_with_real_wheel_pose_anchors_and_mailbox_eventually_drives[0.08-20]
tests/unit/test_continuous_scan_control.py::test_slow_scan_with_real_wheel_pose_anchors_and_mailbox_eventually_drives[0.12-10]
tests/unit/test_continuous_scan_control.py::test_slow_scan_with_real_wheel_pose_anchors_and_mailbox_eventually_drives[0.12-20]
```

`batch-final-diagnostic-resource`：

```text
tests/unit/test_estimation_review_contracts.py::test_scan_reuse_has_one_raw_and_each_attempted_use_age
tests/unit/test_control_process_resources.py::test_absolute_slots_skip_50ms_after_80ms_work_and_skip_long_backlog
tests/unit/test_control_process_resources.py::test_cadence_rejects_nonfinite_and_backwards_clock[nan]
tests/unit/test_control_process_resources.py::test_cadence_rejects_nonfinite_and_backwards_clock[inf]
tests/unit/test_control_process_resources.py::test_cadence_rejects_nonfinite_and_backwards_clock[-inf]
tests/unit/test_control_process_resources.py::test_cadence_rejects_nonfinite_and_backwards_clock[-0.01]
tests/unit/test_control_process_resources.py::test_mailbox_latest_is_private_bounded_and_busy_lock_does_not_wait
tests/unit/test_control_process_resources.py::test_real_spawn_serializes_tasks_and_results_and_cleans_up
tests/unit/test_control_process_resources.py::test_process_death_fails_pending_future_and_close_reaps_it
tests/unit/test_control_process_resources.py::test_real_spawn_carries_geometry_and_complete_sensor_snapshot
tests/unit/test_control_process_resources.py::test_cpu_busy_spawn_does_not_hold_parent_snapshot_or_heartbeat
```

## 亲审后定向纠正（ctx_c763639df965）

负责人发现两处测试迁移错误，本worker已定向修正，产品没有修改：

- `tests/ros/test_motion_core_acceptance.py`的`control.release`仍为请求/响应，错误的`assert response is None`恢复为`assert response.get("ok") and response.get("accepted"), response`。抽取同文件`manual_exchange`作为明确的测试支持边界，嵌套request继续记录所有请求应答，仅`drive.intent`调用独立intents.send并返回None。
- 新增`tests/unit/test_acceptance_manual_exchange.py`，直接载入上述实际支持函数，无ROS、服务或运动输出。获取/释放×成功/拒绝四组合要求返回原始响应对象而不调用意图通道；意图单独验证仅发送、无请求等待、即使transport返回值也不能变成业务回执，并检查原接收/截止时钟。
- `tests/unit/test_drive_challenge.py`移除未接收任何消息的死Arbiter及idle断言，保留真实Hub队列的`safety.estop`先于`drive.stop`转发顺序。端到端锁存仍依赖前述真实MotionStateMachine owner集成证据，没有新增owner。

检查范围包括最终23个skip节点所属文件与4个cv2收集跳过模块，并对tests/ros整体搜索`drive.intent/manual_intent/control.acquire/control.release/response is None`。这批跳过文件中只有motion_core_acceptance有意图与获取/释放请求分支，其余没有同类误改；该文件剩余两处None断言均紧跟drive.intent调用。清理路径的release同样经请求通道取得响应，清理未确认的整车效果仍没有验证证据。

本轮准确命令：

```text
.venv/bin/python -m pytest -q --tb=short tests/unit/test_acceptance_manual_exchange.py tests/unit/test_drive_challenge.py tests/ros/test_motion_core_acceptance.py --junitxml=/tmp/batch-review-contract-fix.xml
.venv/bin/ruff check tests/ros/test_motion_core_acceptance.py tests/unit/test_drive_challenge.py tests/unit/test_acceptance_manual_exchange.py
.venv/bin/python -m pytest --collect-only -q tests/unit tests/integration tests/ros/test_motion_core_acceptance.py tests/ros/test_native_motion_executor.py tests/ros/test_agent_adapter_shutdown.py
```

实测定向**7 passed、4 skipped，0.30s**，日志`/tmp/batch-review-contract-fix.log`及同名JUnit；4项ROS整车验收依旧skip，不能据此声称其业务链已验证。Ruff三文件通过，三文件Python3.10 AST通过；产品47文件指纹及三份受保护文件hash均与原冻结版本一致，git保留CRLF的diff检查通过，无暂存或提交。

新增5个无ROS用例后最新收集2364项，合并最新节点证据**2341 passed、23 skipped、0 failed/error/missing**，另4个cv2模块收集跳过。修改Python测试/支持文件总数从64增至65，原64文件静态证据与此次三文件检查共同覆盖；未重跑整套，原环境/整车未验证项保持。
