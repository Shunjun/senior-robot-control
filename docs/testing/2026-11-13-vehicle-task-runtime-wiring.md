# T6c 车端任务运行器接线（待负责人审阅）

本批实现真实 Python 调用链与最小 ROS 接口，未提交、未 push，未操作 index，未启动开发服务、ROS 或 Gazebo。T6a 已审提交为 `76f260e2c34aa2a3f736bf5053cb73a733b3f2f2`；本批不修改其 lifecycle.py 或核心测试。T6b 获审版本保存于 `/tmp/t6b-reviewed-2026-11-13.patch` 和同名 `.sha256.json`；重建六文件后逐项 SHA256 校验一致。本批增量按 runtime、IPC/ROS、arbiter 分组保存于 `/tmp/t6c-runtime.patch`、`/tmp/t6c-ipc-ros.patch`、`/tmp/t6c-arbiter.patch`，当前逐文件摘要在 `/tmp/t6c-files.sha256.json`。

## 实际链路与唯一 owner

`run_agent -> build_task_runtime -> TaskRuntime.tick -> TaskExecutor -> LifecycleService -> LifecycleStore / TaskStore` 使用同一 SQLite、write_lock、agent_operation 请求权威与 execution_id。绑定实例的身份、epoch、进度、回执、outbox 由 LifecycleStore 唯一写；TaskStore 保留定义、计划扫描与粗状态投影，旧状态 writer 不得写已绑定实例。事务提交后 `NavigationEffects -> IpcNavigation -> AdapterCore task_start/status/cancel -> RouteService -> NavigationAdapter -> DedicatedController / CommandNode` 调用真实接收端类；没有 fake-port 生产许可分支。

定时任务接受时在同一事务保存定义、稳定 action IDs 和目标 Pose2D；编辑未来任务/航点不会改写已接受执行。直接导航由 `LifecycleNavigationOperations -> DirectNavigation -> _TaskRouteReceiver -> TaskExecutor.submit_route` 接受，复用规范 operation fingerprint，不先 AgentJournal.begin；内部一次性定义从调度列表隐藏。状态查询区分 ACCEPTED 和实际 starting/running/终态，重复请求返回原接受结果；未知结果只 query。

Context provider 在获审事务入口取得锁及检查版本后读取当前值。production provider 同时读取即时 agent.connected；本地 IPC 样本带 monotonic valid_until、位姿、地图/定位 nonce、task_lifecycle_version、task_evidence_ready。直接导航提交前刷新同一 IPC 状态转换。网页 RTT 不作为任务准入；数据库检查不能替代执行端实时本地许可，RouteService 和 Arbiter 继续检查本地来源/地图/健康/样本 deadline。

## 替换与兼容清单

- Runtime 不再因 webpage connection generation 改变调用 mark_active_cancel_pending；运行中实例离线仍 reconcile。离线新定时任务仍拒启/记录跳过，重启核心 restart 只撤销/暂停，不新发 plan。
- Runner 对活动实例保留来源/安全判断，移除远端 connected 和普通 admission_ready 对活动任务的取消语义；新接受仍检查这些准入条件。普通障碍不清任务，执行异常/急停保持暂停。
- Executor 移除已接受执行的 link/generation 取消检查。生产入口注入 LifecycleService；集中 `_UnmigratedExecutor` 标 `//Compatible`，仅无绑定旧调用使用，生产不走旧 runtime 连接取消路径。
- 旧执行首次接入保持 execution_id，先撤来源再暂停；不推断历史已完成动作，不补造旧目标几何。没有已接受目标快照的旧执行拒绝直接 resume，需明确重建确认路线。
- 显式 pause/cancel/takeover/resume 均进持久生命周期；松杆/通信重连没有 resume 调用。并发用户命令赢得 CAS 后 runtime 下轮重读，不覆盖用户优先级。一次 status 超时保留实例，下轮只查询，不重新发 plan。

## IPC/ROS 增量（与运行器分开审阅）

`RoutePlan` v2 使用 task_epoch、effect_id、稳定 action_ids、pose_ref；与 legacy connection_epoch 字段互斥。`RouteProgress` 返回真实 task_epoch/effect_id、completed_action_ids、fenced_through、controller_output_exited。既有 task_start/status/cancel 承载稳定效果查询，state.get 明确 task_lifecycle_version=2；旧 cancel ok 不能成为撤销证据。旧接口兼容集中标注，不新增通用能力框架/四套路由。

NavigationAdapter 仅在 Nav2 goal 实际成功结果产生 action completion；并发取消先排空已经到达的成功结果，避免丢已完成动作。RouteService 使用 task epoch，网页换代不取消路线；一个路线来源贯穿多个目标，最终退出 dedicated publisher 后等待来源退役证据。epoch fence 拒绝旧来源与旧 cancel 影响新执行。

CommandNode 撤销过程销毁旧订阅、退役 Arbiter 来源、递增输出 generation，发送随机 token 关联的零输出 barrier。必要直接调用点 `safety_node.py` 增加 14 行：仅在现有 FinalCommandFence 接受 barrier 后，发布零输出，再回传 `retired`；沿用现有 DriveExecutionFeedback 字段，无 .msg/schema 改动。CommandNode 校验唯一 safety_gate 来源、owner boot、robot、token、generation 后确认。此处是来源/旧输出退役，**不代表车体静止**；未改 T5 轨迹、制动算法或阈值。旧 Nav2 cancel RPC 返回也不代表边界完成。

`NavigationEffects` 所有外部 query/execute/resolve/progress 使用 0.3 秒可取消等待；未知发送结果留 sending，通过效果 ID 查询。服务每轮有界协调，background tick 0.2 秒；没有数据库锁等待网络。动作结果仍依赖 Nav2 和来源 fence 的真实承诺，不宣称物理 exactly-once。

## Arbiter 独立增量

begin_navigation 明确车端 task_epoch；task_navigation_allowed 与旧网页导航资格集中分开。`_clear_remote` 撤网页手动来源，保留已接受任务；`_clear_task_source` 退役任务 epoch/导航输出；`_clear_motion` 全撤。disconnect、heartbeat 换代/超时及手动 lease expiry 用前者；急停、shutdown、真实任务取消保留全撤/明确任务撤销。导航本地样本 deadline、任务 fence、显式暂停接管与急停仍有效。来源伪造、旧 epoch、手动断流回归保留。

手动链路为 `TaskAwareMotion.control.acquire -> durable takeover -> revoke proof -> manual_ready 持久消费 -> real IpcMotion acquire -> Arbiter`；未确认时返回 pending/unavailable，不能提前转发手动许可。nav.start/resume 使用持久显式继续，只携带未完成 action，并用当前位姿/地图规划。安全 halt/estop 继续先走直接运动安全端口，不等待数据库日志 IO。

## 验证证据

新增 `test_task_ownership.py` 九个正常链路/必要回归：真实 Runtime+Runner+Executor+Service+SQLite；其中一例使用真实 IpcNavigation、AdapterCore、RouteService、NavigationAdapter，与可控 Nav2 goal/controller transport，验证两目标完成/暂停/继续/新 epoch 只发剩余目标。它是 Python 集成测试，未运行 rclpy/Gazebo。另覆盖正常同 ID 查询、离线继续完成、新请求去重、接管边界、重启暂停、accepted 定义不被未来编辑替换、TaskAwareMotion 到真实 Arbiter 的串行接管。

定向 RED：新增 status_timeout 测试首先失败于 `TaskExecutor.reconcile -> NavigationEffects.progress -> TimeoutError`；修复后九例通过。旧任务 75 条中的冲突断网断言已迁移；失败/取消重试用真实 safety_clear=False 触发，保留原验证作用，没有扩大异常专项矩阵。

最终全量定向回归 **373 passed in 11.37s**；最后一次运行器/接线子集 **84 passed in 1.55s**。静态 ROS 入口编译及 CommandNode/SafetyGate 真实方法定向测试只能证明 Python 接线，不代替 ROS middleware、DDS 元数据、Nav2/controller 生命周期的实测。

## 首版正常验收与后续

首版优先负责人同版 Linux 环境安排：提交任务、开始、常规导航完成及同 ID 查询、显式暂停/继续、手动接管后不自动续跑。本批未启动环境；建议先同步本批获审代码并构建现有 senior_motion_msgs/senior_robot，再按现有 baseline 运行正常手动/导航测试，补持久任务正常路径观测。可复用命令入口：`SENIOR_SIM_NAV_SOCKET=<实际socket> python -m pytest tests/ros/test_simulation_manual.py tests/ros/test_simulation_navigation.py -vv`；这些旧场景本身还不是持久任务全部验收。

后续专项：断网/进程崩溃/未知效果/迟到 DDS 回执的 Linux 场景，交付首版后与用户细化，不作本批新增资格门。T5 admission_changed 尚需生产事件接线；本批仅用当前本地状态轮询驱动反馈恢复，不改 T5/settings RuntimeGate owner。真实 ROS 双端必须同版部署，退役 receipt 必须实际可达；旧端无版本/无证据时保持拒启或待确认。T7 请求权威与设置执行未改。T6c 待亲审、不提交；不能宣称整项 T6 或车辆实际运行已完成。

## 最终核验命令与修改清单

```sh
.venv/bin/pytest -q tests/unit/test_task_lifecycle.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py tests/integration/test_task_runner.py tests/integration/test_task_ownership.py tests/integration/test_storage.py tests/integration/test_agent.py tests/integration/test_safety_intent_store.py tests/unit/test_arbiter.py tests/unit/test_arbiter_recovery.py tests/unit/test_agent_ports.py tests/unit/test_direct_navigation.py tests/unit/test_navigation_command_clock.py tests/unit/test_navigation_collision_qualification.py
# 373 passed in 11.37s
.venv/bin/mypy --strict src/senior_control/tasks/{executor,models,runner,runtime,store,lifecycle_store,lifecycle_service,navigation_port}.py src/senior_control/agent/{composition,dispatcher,main,ports}.py src/senior_control/navigation.py src/senior_control/safety/arbiter.py tests/integration/test_task_ownership.py
# Success: no issues found in 15 source files
.venv/bin/python -m py_compile ros2/src/senior_robot/senior_robot/{agent_adapter,navigation_adapter,command_node,safety_node}.py
# exit 0
```

对以下 23 个代码/测试文件运行 `ruff format --check`、`ruff check`，分别为 23 files already formatted / All checks passed；限定路径 `git -c core.whitespace=cr-at-eol diff --check` exit 0。`git diff --cached --name-only` 输出空；T6a 两文件 diff 空。历史 TaskStore CRLF 保留。

- `src/senior_control/tasks/executor.py`
- `src/senior_control/tasks/models.py`
- `src/senior_control/tasks/runner.py`
- `src/senior_control/tasks/runtime.py`
- `src/senior_control/tasks/store.py`
- `src/senior_control/tasks/lifecycle_store.py`
- `src/senior_control/tasks/lifecycle_service.py`
- `src/senior_control/tasks/navigation_port.py`
- `src/senior_control/agent/composition.py`
- `src/senior_control/agent/dispatcher.py`
- `src/senior_control/agent/main.py`
- `tests/integration/test_task_ownership.py`
- `tests/integration/test_task_runner.py`
- `src/senior_control/agent/ports.py`
- `src/senior_control/navigation.py`
- `ros2/src/senior_robot/senior_robot/agent_adapter.py`
- `ros2/src/senior_robot/senior_robot/navigation_adapter.py`
- `ros2/src/senior_robot/senior_robot/command_node.py`
- `ros2/src/senior_robot/senior_robot/safety_node.py`
- `tests/unit/test_navigation_command_clock.py`
- `tests/unit/test_navigation_collision_qualification.py`
- `src/senior_control/safety/arbiter.py`
- `tests/unit/test_arbiter.py`

另新增本报告。T6b 两个测试及 durability 报告保持已审版本；未修改 protocol.py/.msg、T5 motion、settings、日志、plan 或其他 worker 文件。

## 本轮 task_77933bc61f94：回执阻断修复与前端契约只读对齐

只读检查 `/private/tmp/task_cc710bd1c2fb-t9b-report.md`、`web/src/api/vehicleState.ts`，没有修改 web。T6c 此前未获批提交，本轮仍无 index/commit/push 操作。开始时复制本批 6 个待改文件的基线到 `/tmp/task_77933bc61f94-baseline` 并保存 SHA；本轮完整增量 `/tmp/task_77933bc61f94-followup.patch`（20445 bytes），逐文件 SHA `/tmp/task_77933bc61f94-files.sha256.json`。原 T6b 获审基线及原三组 T6c patch 保留，原三组是上一轮快照，审阅最新实现需叠加本轮 patch。

### 负责人发现的正常回执阻断

`Agent._reply` 实际传入 TaskOperationResult.data，而旧 ResponsePayload 仅准许 workspace 数据。新增三个参数化用例首先真实复现 ValidationError；修复将 **navigation.goal / navigation.return_home / operation.query** 的数据限制为严格 TaskNavigationState（execution_id、phase、epoch、completed、remaining）。拒绝额外键、缺字段、非法 epoch、非法 phase、非字符串动作、重复/重叠动作身份；旧非任务 query 仍可省略 data，其他请求不能任意带 data，失败响应也不能携带成功数据。没有放松 workspace 原来的必需数据规则。

测试不只看 service 返回值：既有真实 SQLite / Dispatcher 接受与查询用例增加完整 `Agent._reply -> ResponseEnvelope -> serialize_envelope -> validate_envelope` 检查，三个请求类型均走实际回执方法，另检验旧非任务 query 无 data。四条不合法任务数据先 RED（旧宽松试修未拒绝），加严格结构校验后 GREEN。

### 正常任务命令实际入口

最小新增 TaskExecutionEnvelope 类型，接 `Agent.dispatch -> LifecycleNavigationOperations.execution_command -> LifecycleService.command -> LifecycleStore.command`；不经第二个 AgentJournal.begin，不改 cloud、认证或 settings/logs 路由。既有 relay 在认证连接上复用共享协议转发，无产品 relay 改动。该入口已经实际调用 SQLite 和效果协调，不是 unavailable 占位。

- 输入严格为 `{request_id, expected_version, task_id, action}`；action 仅 pause/resume/cancel/takeover。
- 此处 **task_id 是执行实例 execution_id**，不是可编辑/定时计划的 TaskDefinition.task_id；前端 task projection 必须采用同一身份。
- 输出严格为 `{request_id, task_id, accepted, reason, version}`，就是持久核心 Receipt；version 为受理事务版本，效果确认可能使当前状态继续增版，不能以 receipt 覆盖遥测。外层 ACK 代表请求处理成功，accepted/phase/撤销是否完成分别判断。
- 新增可选 operation_fingerprint 参数仅传到已有 service/store command；该入口以完整嵌套载荷（含 expected_version）计算指纹，传输重试改变外层 epoch/request ID 不会重新执行。同 ID 修改版本或动作冲突，旧版本新命令拒绝；重放先查已提交回执，避免重放 resume 依赖新的健康样本。
- 首次 resume 刷新本地位姿/地图状态，复用已有目标几何；暂停/取消/接管的受理不要求导航先返回成功。正常 runtime 独立协调直到撤销确认后结算，没有假造 cancel RPC 的物理完成。
- `execution_view(execution_id)` 提供与前端 TaskExecution 对齐的 T6 投影（task_id/version/phase/completed 数/total/stopped_for/allowed_actions）；仅从核心状态与当前 Context 派生，不复制任务状态机。该投影还没有塞进 vehicle_ui，见下节真实缺口。

两条参数化真实 Agent._answer 集成测试分别经过 pause 和 takeover：先返回稳定持久回执、保持 revoking；重复同载荷不重发、异版本同 ID 冲突；实际导航 ack 后 paused；显式 resume 用新位姿重新发 plan；显式 cancel 进入 canceled，终态无 allowed_actions。所有这些测试使用实际 Runtime/Executor/Service/SQLite 与可控导航端口；它们没有运行 ROS/Gazebo，也不声称新手动 UI 驾驶已接通。

### 真实尚缺的 T2/T5 连接（仍是产品集成工作）

1. **T2 新手动协议尚未接入。** Python SimpleEnvelope.control.acquire 仍为旧空载荷，IpcMotion.forward 只返回旧 command ok；没有生成/返回前端要求的 `{authority_id,state:'prepared'}`。control.release 还不接受 authority_id，drive.intent（authority_id/gesture/sequence/linear/turn/gear）尚无 Python parser/ROS 接收映射。现有 TaskAwareMotion 已真实串行执行任务撤销与旧 IpcMotion acquire，但不等于新版 T2 authority/intent 链；Agent 的旧手动 eligibility 仍含 legacy clock 门，必须在 T2 新路径替换，不能通过前端虚构许可绕过。本轮未借修任务回执修改整个手动/认证链。
2. **T5 生产状态与准入尚未接入。** 当前 Context 来自既有本地导航 IPC 资格与 TTL；没有把真实 MotionStateMachine 的 stationary/healthy、ESTOP/fault revision、admission_changed 事件和唯一运动输出装配进该 provider。特别是“旧输出退役”不能冒充 T5 的车体静止/恢复许可；T6 resume 仍需消费那条真实本地准入。所需串行接口是：最新 T5 可规划许可及 pose/map refs -> Context provider；准入变化 -> 生命周期 admission_changed；task started/paused/finished/manual handoff -> 可信 T5 事件流。任务 epoch/进度仍归 T6，T5 不写任务快照。
3. **vehicle_ui 尚未发布。** `agent/vehicle_state.py` 已有纯投影类型，但 composition/SnapshotPayload 未装配可信 T5 RuntimeSnapshot、T2 会话 authority 与本轮 T6 TaskExecution；因此还不能声明 runtime.v1/motion.intent.v1/task.execution.v1 给当前前端。需要复用现有 project_vehicle_state，提供可信序列、安全版本、独立车体平均速度来源/年龄和按会话隔离的手动 authority；不能用轮速、固定 true 或最后一次网页状态填补。这是明确的实际装配缺口，本轮没有给它造成功占位。settings/logs 认证路由仍留后续串行任务。

因此：本轮修复了已发现的正常导航回执阻断并接通正常任务命令后端入口；**整车新 UI 的正常手动/完整状态投影尚未完成，T6c 仍待亲审与上述 owner 接线**。不以本轮通过测试宣称整项 T6 或正常车辆实测完成。首版正常验收继续优先；断网/崩溃专项实际场景后置。

### 本轮最终核验

- 新增 9 个参数化场景；TaskOwnership 共 18 例。初始导航回执 3 RED、命令入口 2 RED，严格数据校验另 4 RED；最终均 GREEN。
- 任务/存储/agent/来源/协议及现有认证 relay 回归：**415 passed in 13.44s**。新增协议、TaskOwnership 和现有 cloud 子集 **51 passed in 3.33s**；cloud 测试为 in-process ASGI，没有开启开发服务。
- `.venv/bin/mypy --strict src/senior_control/tasks/{executor,models,runner,runtime,store,lifecycle_store,lifecycle_service,navigation_port}.py src/senior_control/agent/{composition,dispatcher,main,ports}.py src/senior_control/{protocol,navigation}.py src/senior_control/safety/arbiter.py tests/integration/test_task_ownership.py`：16 source files success。
- 本轮六文件 Ruff format --check / Ruff check 均通过；限定路径 `git -c core.whitespace=cr-at-eol diff --check` 通过；`git diff --cached --name-only` 空。
- 415 回归命令在上一轮完整命令后加 `tests/unit/test_protocol.py tests/integration/test_cloud.py`；其他路径完全相同。未重跑已知沙箱 bind E2E、未收集全仓 ROS、未启动服务。

本轮修改：`protocol.py`、`agent/{dispatcher,main}.py`、`tasks/{lifecycle_store,lifecycle_service}.py`、`tests/integration/test_task_ownership.py` 与本报告；其他已批 T6c 文件保留，T5/settings/logs/前端均只读。
