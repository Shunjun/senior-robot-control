# T6a 任务生命周期纯核心（2026-11-13）

本批仅新增 `src/senior_control/tasks/lifecycle.py`、`tests/unit/test_task_lifecycle.py` 和本报告。基于已批准新版 spec 与 plan Task 6；没有修改既有 runner/runtime/executor/store、protocol、agent、T5、settings、vehicle_state 或计划，没有操作 index、提交或 push，没有启动开发服务、ROS 或 Gazebo。T6 尚未完成；当前产品执行路径仍使用旧语义。

## 实现与接口所有权

`Lifecycle` 是一个执行实例的不可变快照，`version` 用于现有数据库中的乐观并发更新，`epoch` 是任务自己的执行代次。`task_id` 在该核心中指稳定的**任务执行实例身份**，接线时绑定 `TaskStore.ExecutionRecord.execution_id`，不能把周期任务定义 ID 直接作为所有 occurrence 共用的实例 ID。定义 ID、occurrence 与实例关系继续由既有 TaskStore 保存。`actions` 是有序且唯一的动作实例 ID，同一航点重复访问应分配不同动作 ID；它们不是速度采样，也不是 ROS goal ID。

- T6 唯一拥有实例身份、有序已完成进度、执行代次和命令回执。`command`、`event`、`acknowledge` 是纯转换，`query` 返回原请求的稳定回执；当前执行状态从同一快照查询。`query == None` 表示未知，不能作为失败或重发依据。
- T5 继续拥有运动模式、健康、静止、故障、ESTOP 和手动 authority。`Context.motion_allowed` 是其准入结果，不在 T6 重新计时或计算健康。`pose_ref/map_ref` 是车端采样并校验的当前位姿/地图引用，接线不得直接信任浏览器传值。恢复时不重放旧轨迹。
- `Command` 只包含命令载荷；连接及当前健康上下文不属于载荷指纹。同 request 同载荷返回原回执、无新效果；异载荷冲突。被拒绝请求也有稳定回执；情况改变后需新的显式请求。记录没有静默淘汰逻辑，持久层必须保留去重记录或满额拒收。
- 模块仅涵盖互斥运动任务的生命周期，不引入调度器、第二数据库、通用任务框架或网页状态模型。队列与非运动操作继续由已有领域组织，不在本批扩大并发。

## 转换与外部效果

| 输入/边界 | 状态与效果 |
| --- | --- |
| 新提交（包括定时 occurrence） | 离线拒绝；在线且 T5/当前参考准入后记录身份，`starting` 并发出 `plan`；确认后才 `running` |
| 运行中断连/连接换代、松杆、普通障碍 | 身份/代次/进度不变，不取消任务；障碍约束和 Nav2 普通重规划归原链 |
| 手动接管/用户暂停/取消 | 先 `revoking` + `revoke`；未确认前不能继续或交手动，失败/未知确认保留义务 |
| 撤销确认 | 暂停或取消落定；接管才发 `manual_ready`，这只是交给 T5 的许可通知，不声明手动已获授权/车辆已运动 |
| 用户明确继续 | 仅暂停且当前 T5 准入/参考齐全时生成新执行代次，从当前参考 `plan` 未完成动作 |
| 临时反馈丢失/恢复 | 先撤销并确认；反馈原因暂停允许 T5 恢复事件触发重规划，允许车端在网页离线时继续已接受任务；用户暂停/取消/接管、故障、急停、重启原因不会被反馈恢复覆盖 |
| 执行故障/急停 | 撤销后保持暂停，健康恢复不启动任务；T6 不执行急停复位 |
| 进程重启 | 提高执行代次、重建覆盖旧来源的撤销义务；原暂停继续暂停；不重发 plan，不自动续跑，不重给手动许可；保留显式取消义务 |
| 动作/完成回调 | 当前代次、有序动作才记录；重复/旧代次忽略，撤销中仍记录已提交的动作完成但不恢复输出；完成不能越过取消/接管的撤销边界 |
| 不可恢复任务失败 | 撤销确认后进入 `failed`；区别于可恢复执行故障暂停 |
| 终态 | succeeded/failed/canceled 均不能被回调、继续、重连或重启复活 |

`Effect.effect_id + epoch` 必须原样回执，错误 ID/代次、重复或迟到回执无效果。`plan` 不确认就不称运行；启动失败/超时可能已产生外部副作用，先撤销，不能直接宣称回滚完成。`completed` 是导航适配器确认任务完成并退役来源后的事实事件，不是浏览器状态猜测。

`revoke` 必须撤销该实例 **through epoch** 的所有旧来源（重启后包括旧执行代次）、确认 Nav2/控制器旧输出不能再被接纳，并在确认边界前交付已提交动作进度；仅发出 cancel RPC 或收到 cancel 接受响应不够。末动作与接管交错时，完成进度保留，撤销确认后可进入成功；显式取消则进入取消，不能由迟到完成复活。T5 再决定手动 authority 是否可准入。本模块没有伪造 ROS 撤销或车辆停稳确认。

调用者须串行化输入，在既有 SQLite 事务内 CAS 保存快照、命令结果及效果义务，提交后才调用外部端口。重复 command/ack 不重发效果；重启/超时从持久 `pending` 和端口查询协调，不盲目重发计划。`manual_ready` 等通知也需要同事务 outbox/已消费记录，不能只靠进程内 tuple；本批未实现该适配。物理动作必须以稳定 `(task instance, action ID)` 做外部幂等，边界 ack 前刷新动作账本，才能跨进程崩溃防止已做动作再执行；纯核心只能保证已持久确认的动作不会重新进入 plan，不能宣称物理“恰好一次”。

## 现有持久化与精确接线点

| 现有位置 | 可复用能力与后续变更 |
| --- | --- |
| `storage/database.py:Database.write_lock`、`tasks/store.py:TaskStore._transaction` | 现有单库 `BEGIN IMMEDIATE`/事务；扩展同一库保存 version、epoch、进度、待确认效果及请求结果，禁止另建数据库。现有公开方法各自提交，不能简单串联冒充一次原子事务 |
| `tasks/store.py:TaskStore.claim`（353 行）、`execution_by_id`（565 行） | occurrence 唯一约束、execution_id、任务定义/地图绑定检查可复用；claim 目前生成 UUID，应在原子接受事务绑定 request/result 与核心实例，而不是先发导航 |
| `tasks/store.py:TaskStore.prepare`（95 行） | 现有重启变 cancel_pending，不自动重发；接线迁移到持久核心快照的 restart 处理，不能恢复旧 running |
| `tasks/store.py:mark_running/finish/pause/mark_cancel_pending` | 当前只有粗粒度状态，无完成动作、CAS、效果 ack；需要同库演进并替换写路径，不能让两套状态并行相互覆盖 |
| `agent/journal.py:AgentJournal.get/begin/finish`（156/164/180 行）、`agent/dispatcher.py:Dispatcher.run/_replay`（108/129 行） | 已有 operation ID + fingerprint 去重/查询；复用同一身份命名空间并绑定稳定 receipt 与实例。`begin`/`finish` 当前各自提交，只有 status/error/revision，不能无改动承载完整生命周期原子接受结果 |
| `tasks/runtime.py:TaskRuntime.tick`（41 行） | 删除连接换代对已接受实例调用 `mark_active_cancel_pending`；断网仍 reconcile，分别保留新定时 occurrence 离线拒启与 cursor/skip 审计 |
| `tasks/runner.py:TaskRunner.tick/_block_reason`（32/172 行） | 区分 active 的车端所有权与新的 occurrence admission，移除 active offline pause；不暗中允许离线新启 |
| `tasks/executor.py:persist_and_dispatch/reconcile`（86/225 行） | 移除对已接受实例的 link/generation 取消；转换计划/撤销效果与 ack，持久进度后再推进，不再把 await cancel 返回直接等同完整输出边界 |
| `agent/ports.py:IpcNavigation.start_qualified/status`（92/142 行）、`navigation.py:RoutePlan/RouteProgress` | 连接 epoch 与执行 epoch 分离；补充稳定动作实例与完成账本，当前 active_waypoint_index 无法单独证明动作已提交 |
| `ros2/src/senior_robot/senior_robot/navigation_adapter.py:RouteService.start/_run/status/cancel`（201/225/310/320 行附近） | 当前 connection_epoch 参与 route 运行资格；后续用任务 epoch fencing 替代已接受任务的连接资格。执行恢复/接管必须回到当前位姿地图，不 replay 原 route 游标 |
| `motion/state_machine.py` 与 T2 authority、ROS dedicated_controller/command_node | T6 发撤销意图，适配层汇聚来源 fence 与控制器输出退役证据，再 ack；ack 之后 `manual_ready` 才进入 T2/T5 手动准入。T6 不新增健康或停车计时器 |

后续串行步骤：负责人亲审本批实际 diff → 同库 schema/事务及 journal 接线 → runner/runtime/executor 替换旧连接语义 → ROS 来源 fence/动作账本/真实撤销边界 → T2/T5 手动交接与当前参考重规划 → 迁移冲突旧测试并增加实际 ownership integration → Linux ROS/Gazebo 无 skip 验收。不得把现有历史回归通过当成新版 T6 产品完成。

## 验证记录

1. 先新增测试运行 `.venv/bin/python -m pytest tests/unit/test_task_lifecycle.py -q`，预期失败：`ModuleNotFoundError: senior_control.tasks.lifecycle`。随后实现；竞态扩展曾实际捕获“重启后 cancel 义务被重复取消短路吞掉”，修复后回归通过。
2. 最终 `.venv/bin/python -m pytest tests/unit/test_task_lifecycle.py tests/integration/test_task_runner.py -q`：**115 passed**（40 新核心场景 + 75 现有任务回归）。覆盖离线完成同 ID 查询、请求/回执重复、异载荷冲突、旧进度/回执 fence、接管/继续/完成/取消竞态、反馈与显式操作优先级、终态、重启、未知启动结果与撤销失败。
3. `.venv/bin/ruff check src/senior_control/tasks/lifecycle.py tests/unit/test_task_lifecycle.py`：All checks passed；`.venv/bin/mypy --strict src/senior_control/tasks/lifecycle.py tests/unit/test_task_lifecycle.py`：Success, 2 source files。格式经 Ruff format 校验。
4. 尝试包含 `tests/e2e/test_tasks.py` 的回归时，111 passed、1 fixture error：沙箱不允许 fixture 在 `127.0.0.1:0` bind，未执行该用例正文。该文件本身仍断言旧 offline pause，须后续串行迁移，不作为新版行为证据；未申请扩大权限或启动服务。
5. 未接 runner/真实 Nav2/ROS，未验证物理停止、传感恢复、跨崩溃事务/外部动作幂等或真实撤销确认。本报告只证明纯模块与现有可运行任务回归，不宣称 T6 全完成。

## 负责人复审修复 R1/R2（本次最新结果）

首轮提交虽然有 115 项通过，负责人实际复审复现了两处遗漏，暂未验收 T6a。本次仅修复原三文件，生产核心相对首轮为 **+61/-19 行**，测试 **+161/-4 行**，无共享接线、T5/settings/plan 修改，无 index/提交/push 操作。

**R1 根因与变更：** `feedback_recovered` 原先只在 paused 接受，撤销中的恢复边沿被直接忽略。原测试在 ack 后另发恢复事件，并未证明第一次恢复被保留；现将其改为真正的晚到恢复测试，另加完整早到恢复顺序测试。

`Lifecycle.recovery_requested` 只持久保存“允许尝试反馈恢复”的意图，不保存旧健康、位姿或地图。撤销中收到恢复事件后保持 revoking，不发 plan；`acknowledge(..., ok=..., context=...)` **必须**由适配层传入确认时最新的 T5 准入和当前位姿/地图引用，确认成功后才消费意图重规划。没有兼容默认 Context，也不复用恢复事件时的旧引用。恢复事件晚于确认则使用该事件携带的当前 Context；网页离线不阻止已接受任务恢复。

若确认时运动准入不足或当前位姿/地图缺失，义务仍持久保留为 paused；适配层在 T5 准入或当前参考变更时发可信 `Event('admission_changed')`，用该次最新 Context 再评估，不要求重新发送一次 `feedback_recovered`。这个事件只是消费待处理意图，本身不能创建恢复许可，不是定时器或新的健康状态机。计划发出即消费意图；再次 feedback_lost、明确暂停/接管/取消、fault/estop/restart 均清除它，终态也不遗留自动恢复义务。错误/失败/重复 ack 不提前消费义务，正确 ack 最多产出一次计划。

后续串行接线新增两点：撤销 ack 调用点须实时采样 Context；T5/参考更新流须将准入变化投递为 admission_changed，快照与 recovery_requested 仍纳入原数据库同一事务。未完成这些接线前，不能把纯模块测试称为产品恢复已验证。

**R2 根因与变更：** `_stop` 原先对 revoking 的 stopped_for 执行后到覆盖，导致不可恢复 failure 变成 pause/takeover/fault，确认后还能继续。现在所有停止原因经唯一的 `_settlement_reason` 与优先级表：

`cancel > failure > restart > fault = estop > pause = takeover > feedback`

这里的优先级只决定 T6 撤销后的任务结算，不代表 T5 的运动、急停或健康优先级；同级取最新明确决定。显式 cancel 保留其独立终态语义；failure 不能被普通暂停/接管、重复故障/急停、反馈或重启降级。failure 晚于既有暂停/接管/健康撤销也能升级结算，旧代次的 failure 回调仍受执行 fence 限制。重启建立新撤销义务时同样保留高优先级 cancel/failure；确认前仍是 revoking，不伪称外部执行结束，确认后进入 failed/canceled。当前代次的已提交动作进度继续保留，不产生 manual_ready 或后续 resume plan。

**RED → GREEN 证据：** 先写最小顺序测试，在旧实现看到早恢复最终 `paused != starting`、failure 后 pause/takeover 最终 `paused != failed`；扩展顺序测试进一步看到 failure 被 fault/estop/restart 降级，以及 pause 后 failure 被忽略。补充最新准入测试又先看到旧行为无法响应 admission_changed、终态遗留恢复意图，然后实现对应转换。新测试断言真实 phase、effect、当前参考和进度，不依赖重发恢复边沿。

本次新增 **30 个参数化场景**，覆盖早/晚恢复、在线/离线、恢复后再次失效、确认时最新准入/参考缺失及后续准入恢复、显式操作抢占、撤销确认失败和重复、failure 与控制/故障/急停/反馈/重启的顺序、进度保留和终态不可继续。

最终执行：

- `.venv/bin/python -m pytest tests/unit/test_task_lifecycle.py tests/integration/test_task_runner.py -q` → **145 passed**（70 生命周期场景 + 75 现有任务回归）。
- `.venv/bin/ruff check src/senior_control/tasks/lifecycle.py tests/unit/test_task_lifecycle.py` → All checks passed。
- `.venv/bin/mypy --strict src/senior_control/tasks/lifecycle.py tests/unit/test_task_lifecycle.py` → Success, 2 source files。
- Ruff format → 2 files left unchanged；本批按要求不再运行未修改的 E2E 沙箱 bind 用例，也未启动服务。

当前仍未接真实 runner/Nav2/ROS、持久事务或实体动作幂等；本次修复待负责人再次亲审，不宣称 T6 完成。
