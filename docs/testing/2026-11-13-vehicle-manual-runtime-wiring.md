# T2 实际手动链路与 T5/T6 接线审阅

本批 task_aa944ec6cce1 / ctx_b09c24f02bb7，基于获审 T6c 接线继续实现。未操作 index、提交、push，未启动开发服务、ROS 或 Gazebo。T5 的 guard/解码、motion_fusion、safety_node、输出模块及 launch/config 由另一 worker 独占，本批没有修改这些文件。这里验证的是实际 Python 协议/运行器/SQLite/IPC 类接线，不是车辆运动或 Linux 实时验收。

## 正常主路径已落地

认证 Cloud Hub 为控制连接分配 controller_generation，拒绝浏览器自报该字段；control.acquire / drive.intent / control.release 通过真实 relay → Agent → IpcMotion → AdapterCore → CommandNode → 原 ManualAuthorityManager。prepared 不开短 TTL，也不产生运动目标；第一条新边沿 intent 才 active。Agent 单槽合并未发送意图，车端首次 received_steady 和固定 deadline 经排队、IPC、重复发布均保持原值。新链不以旧 challenge/clock/网页 RTT/网页墙钟资格放行；本地身份、来源、原样本时效仍检查。生产 native 模式拒绝旧 drive.velocity/challenge/reset 并行链，未迁移兼容入口集中且标注 //Compatible。

松杆 release、expiry、连接上下文 retirement 封闭原 authority；关闭来源后交 T5 唯一本机制动 owner，不发网页 ESTOP。旧释放不能关闭后继 authority；急停/故障后仍须新 acquire 与新边沿。T6 接管先走持久 takeover/revoke，消费真实来源 retired 回执，再由最新 T5 健康/静止状态允许第一条手动输入；任务身份与进度保留，松杆不续跑。T6 明确 resume 使用当前地图/位姿重规划剩余动作。现有已接受任务断网继续的车端所有权不回退。

任务 Context 增加实际 T5 halt reason 的事件桥；feedback revoke 尚未确认时到来的 fault/estop 也进入同一持久生命周期，不能被反馈恢复变成自动续跑。相应两例先 RED 后 GREEN。没有复制 T5 运动状态机。

## 唯一实际 T2 → T5 契约

`senior_motion_msgs/msg/FinalCommand`，话题 `/safety/final_command`。本批唯一负责消息与 CommandNode producer；T5 独占 guard 固定帧、Python decoder 及消费者，必须同版构建，不能靠旧消息默认值补来源。

新增五字段：source_kind(manual/navigation/idle)、source_id(真实 authority_id/execution_id/空)、source_epoch(认证 controller_generation / T6 task_epoch / 0)、received_steady(原车端收包或导航样本本地接收时间)、close_reason。原 output generation 是来源 fence，区别于 source_epoch；boot/session/sequence 和固定 steady_deadline 保留。

关闭原因：prepared、release、expiry、connection_retired、task_revoke、qualification_lost、shutdown 撤来源并移交本机制动；intent 是目标输入。estop/fault 不相互冒充；qualification_lost 不制造物理故障。estop_reset/fault_recover 是经最新状态验证的显式控制事件，T5 再验自身 revision/证据。

导航 retired：request_id 为原 retirement token、session_id/execution_id 原样、sequence=0、gesture=0、deadline=0，source_kind=navigation。只有 T5 接受 fence、使旧候选不可消费且原子移交本机制动 owner 成功才回 retired；失败不确认。这只证明来源退出，不证明输出零或车体静止。

显式用户 safety.estop：request_id=原 transport request_id，gesture=0、sequence=0、deadline=0、vx/wz=0，source_kind=manual（有当前手动目标）或 idle，received_steady=Agent 第一次收包时刻。同请求缓存完整原帧及 generation；重发不延期、不重新撤销后来的 authority，不把关闭回声当新用户急停。真正新 request 即使已经锁存也推进新 producer generation。reset 后新事件仍有效。应用 ACK 不是 T5 复位成功；CommandNode 仅在同 T5 boot 的新确认 clear 且原持久锁存清除完成后解除本地锁存。

完整字段与状态 schema 保存在 `/private/tmp/task_aa944ec6cce1-live-wire.md` 和 `/private/tmp/task_aa944ec6cce1-motion-runtime.schema.json`，已向协调者发送供 T5 转交。

## T5 状态 → 真实 IPC → 网页

`/motion/runtime` 使用 std_msgs/String strict schema=1。唯一根 safety_gate publisher/GID、boot、sequence、安全 revision 及原 evidence 时间检查位于 CommandNode；state.get 回传 motion_runtime/version、manual_protocol_version 与当前 owner。Agent 的 SnapshotState 复用 T7 project_vehicle_state 和 T6 execution_view 发布与 `web/src/api/vehicleState.ts` 一致的 vehicle_ui。只发布实际已接的 runtime.v1 / motion.intent.v1 / task.execution.v1；不发布本批未接的 settings/logs 能力。

body_reference 只展示 T5 的独立车体平均参考及原证据年龄/过期，轮速独立；没有样本不造 body 或 ready。authority 仅向认证 generation/agent epoch 相符的控制者投影，其他浏览器看 null，ACK 也不广播。网页安全版本是跨 T5 boot/Agent epoch 的单调投影 token；恢复命令先重新采样，再把当前 token 翻译为同一 T5 boot 的原 revision，避免重启后的旧 UI token 误恢复。

## 设置 owner 与启用方式

`SENIOR_SIMULATION_SETTINGS=1` 显式启用唯一 SettingsService 与 simulation-only SettingsProfile。默认低/高速 5/9、25/18 m/s，两档角速度 1 rad/s，倒车 0.3 m/s；线速度上限两档均 25/18、角速度上限两档均 2、倒车上限 0.3。全部只在 settings/models.py 定义。Agent/ROS 从 published_snapshot 消费，不另建默认值 owner。这是仿真请求策略，不是实车标定或物理能力。未显式选择 profile 不发布手动能力；实车不能自动选该 profile。

SettingsService 原 store/debug 生命周期保留；本批只消费驾驶快照。settings/logs 网页认证路由和完整 settings 写入 RuntimeGate 装配仍待后续串行任务，本批不暴露其能力。

## 验证证据

新增 `tests/integration/test_manual_wire.py` 10 场景：prepared 等待、新边沿前进/转向、原 deadline、release 后新 authority；真实认证 ASGI relay/Agent/IPC/CommandNode 类；网页 generation 伪造拒绝、authority 隐私；正常 ESTOP/确认复位/新输入与重复、新请求；真实 T6 持久接管边界；T5 缺样本不放行；显式仿真策略；fault/estop 抢占反馈恢复；native connector 不再协商旧 drive clock。IPC 用真实 socketpair 进行长度帧编码，导航及 T5 状态是明确可控替身，无 ROS/车辆运行声明。

最终相关回归命令：

```sh
.venv/bin/pytest -q tests/integration/test_manual_wire.py tests/integration/test_agent.py tests/integration/test_agent_pairing.py tests/integration/test_agent_snapshots.py tests/integration/test_cloud.py tests/integration/test_storage.py tests/integration/test_task_runner.py tests/integration/test_task_ownership.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py tests/integration/test_vehicle_settings.py tests/unit/test_task_lifecycle.py tests/unit/test_protocol.py tests/unit/test_agent_ports.py tests/unit/test_agent_config.py tests/unit/test_cloud_epoch.py tests/unit/test_arbiter.py tests/unit/test_arbiter_recovery.py tests/unit/test_direct_navigation.py tests/unit/test_navigation_command_clock.py tests/unit/test_navigation_collision_qualification.py tests/unit/test_settings.py tests/unit/test_websocket_link.py -k 'not test_connector_uses_authenticated_socket_and_epoch'
```

结果：540 passed、1 deselected，17.40 秒。被排除的是既有 loopback bind 测试，先前在本沙箱因 PermissionError 失败；已用新 native connector mock、真实 ASGI 与 socketpair 定向验证，不伪报该 bind 测试通过。新增 10 场景独立运行 10 passed in 1.37s。

Ruff check 与 format --check 覆盖本批 18 个 Python 文件均通过；严格 Mypy 覆盖 14 个 Python source/test 文件通过；两个 ROS producer 文件 py_compile 通过；限定路径 `git -c core.whitespace=cr-at-eol diff --check` 通过。ROS 类型动态导入测试使用明确 stub，不冒称 ROS strict typecheck。

## 审阅材料与剩余正常验收

获审 T6c 基线由原归档加获审 followup patch 重建，六文件 SHA 全部核对。仅本批 19 文件 delta：`/private/tmp/task_aa944ec6cce1-t2-wire.patch`；逐文件基线/当前 SHA：`/private/tmp/task_aa944ec6cce1-files.sha256.json`；重建基线：`/private/tmp/task_aa944ec6cce1-approved-baseline/`。原获审 patch 未覆盖。该 19 文件包括协议、七 Agent 文件、relay、设置 profile、arbiter、任务 models/executor、FinalCommand.msg、两个 ROS producer、三个针对性测试；本报告另列为新增文档。T5 文件不在 delta 内。

下一步负责人亲审本批与 T5 输出 diff 后，在同版 Linux ROS 环境构建消息/guard/consumer，使用明确仿真 profile 验证正常获控→前进/转向→松杆→重新获控、ESTOP→确认复位→新输入、任务开始/完成/暂停/继续/取消/接管。可先运行本报告 pytest 命令复核 Python 接线；本 worker 不启动 ROS/服务。断网/崩溃等专项实际场景后置，不阻塞首版正常验收，但已批准车端任务所有权语义保留。

T2/T6 到真实 Agent/IPC producer 已接，T5 消费/闭环与状态 producer 由其 worker 交付并由负责人合版验证；未实际运行 Linux ROS、Gazebo 或实车，不宣称整项 T6 或车辆正常运行已经完成。


## 本轮审阅修复：task_edef3f5e28ae / ctx_a8e8d7e5d7eb

R1 已先用真实 Hub/Outbox 复现 RED：第二条意图覆盖第一条后，manual_requests 已有两条；release/evict/close 丢弃也不结算。修复将 Outbox 的所有丢弃统一经过 `_drop` 回调，由 Hub 用原 controller_generation/session/request/seq 定位唯一 owner；实际车端回执也复用同一匹配函数。覆盖、显式清除、淘汰、关闭的未发送 drive.intent 返回 `ok=true`、无执行 status/data 的传输结算，不宣称车辆已执行，不作为拒权错误关闭 held 输入，也不重发旧目标。检查 64 条真正未决请求上限前先结算旧队列样本，不提高上限；已经从 outbox 取走的请求仍必须等待真实车端回执。连续 100 条正常短暂积压测试保持一个最新目标及一个待回执，逐条结算原 owner，其他控制者不收到回执。

Agent 用 `_settle_queued_intent` 统一覆盖、release、drive.stop、ESTOP 对未发送意图的结算；受控在途调用与单槽测试证明旧排队样本只回执、不执行。前端既有 intent Promise 成功即继续发送 latest 槽，失败才封闭；因此本批无需 web 协作改动，也没有修改 web。这个成功只表示这条传输请求已经处理，既有协议原本就不以 intent ACK 证明车辆运动。

R2 已把整链测试 helper 从 `_answer` 改成真实 `_receive_command`，经 `_accept`、意图槽或安全队列再调用端口；复位后旧请求实际复现 recovery_required=True 的 RED。修复在原 AgentJournal/SafetyIntentStore 同 SQLite 内增加小型 `agent_safety_receipt` 表，唯一键(robot_id,request_id)，保留事件指纹与完成位；不另造任务/请求数据库。新事件注册与 unresolved 安全义务同事务，只有实际本地 ESTOP port 确认后才持久 completed。指纹包含 type/session/payload，不包含可变 transport seq/expiry/connection；同 ID 不同内容冲突。接收入口先查该持久回执，完成的重放直接回同成功，不 mark、不调用运动端口、不清复位 token、不产生 Agent 假锁存；未完成仍保留 fail-closed 义务，不冒称成功。直接 dispatch 入口同样核对，避免另一条绕路重锁。

实际 T5 新确认复位后才安全清理 unresolved 标记，已完成事件身份保留；同会话不同急停请求在安全队列按 request/seq 独立保存，不能被后来的新事件覆盖。真实认证 ASGI→Agent `_receive_command`→真实 socketpair IPC→CommandNode 路径验证：急停、确认复位、重新 prepared、旧请求重放保持 clear/原权限、新 request 再正常锁存。另用真实 tmp SQLite 关闭重开验证完成回执、异载荷冲突、新事件及未完成义务，不依赖内存 cache。

最终验证：沿用上节完整相关 pytest 命令并加入 `tests/integration/test_safety_intent_store.py`，552 passed、1 既有沙箱 bind 测试 deselected，17.86 秒。新增/调整正常场景集含 manual_wire 18 场景及 safety_intent_store 4 场景；Ruff check/format --check 和 strict Mypy 对本轮五个 Python 文件均通过，限定 diff whitespace 检查通过。未修改 T5、ROS producer、protocol、settings 或 web；未操作 index/commit/push，未启动服务。

本轮仅 3 个生产文件（agent/main.py、agent/journal.py、cloud/relay.py）、2 个测试文件和本报告。相对任务开始工作区的准确增量：`/private/tmp/task_edef3f5e28ae-review-fixes.patch`；逐文件 baseline/current SHA：`/private/tmp/task_edef3f5e28ae-files.sha256.json`；原获审/待审归档均未覆盖。仍待负责人亲审与 T5 独占改动合版后的 Linux 正常操作验证，未把专项实测扩大为交付门。
