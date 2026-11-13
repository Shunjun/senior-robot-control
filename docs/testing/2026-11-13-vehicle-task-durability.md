# T6b 同库任务持久化与效果恢复适配（待负责人审核）

本批依据获批 spec / plan Task 6 和 T6a 接线报告实施，没有重新定义产品需求。仅实现 SQLite 事务与可执行端口服务，没有接生产 runner/executor、agent dispatcher/composition、T5 admission_changed 或 ROS；不代表 T6 已完成。

## 获审 T6a 本地提交

开始时 index 为空，已重新运行负责人获审的 145 项核心/任务测试。限定本地提交：

- SHA：`76f260e2c34aa2a3f736bf5053cb73a733b3f2f2`
- 信息：`feat: add vehicle-owned task lifecycle and recovery ordering`
- 仅包含 `src/senior_control/tasks/lifecycle.py` 与 `tests/unit/test_task_lifecycle.py`，没有包含报告或其他脏文件。
- 提交后及本批结束前 index 均为空；没有 push。T6b 未提交，等待实际 diff 亲审。

## 本批准确文件边界与差异

新增：

1. `src/senior_control/tasks/lifecycle_store.py`：同库事务、版本 CAS、请求结果明细、快照/动作进度、效果 outbox 与恢复。
2. `src/senior_control/tasks/lifecycle_service.py`：启动恢复、command/event/ack 与提交后端口 execute/query。
3. `tests/integration/test_task_lifecycle_store.py`：真实临时 SQLite、事务失败、跨连接 CAS、进程退出与重启。
4. `tests/integration/test_task_lifecycle_service.py`：可控端口、提交/发送/确认边界与控制竞态。
5. 本报告。

最小修改 `src/senior_control/tasks/store.py`：**+85/-36 行**，主要将原 claim 校验及 INSERT 提取为 `claim_in_transaction`（376 行附近），增加执行实例绑定表与旧 writer 保护。保留该文件原 CRLF，避免全文件换行差异；增量空白检查使用 `git -c core.whitespace=cr-at-eol diff --check`。没有修改 storage 全局 schema/database、agent/journal/composition/protocol、motion、settings、日志或 plan，没有覆盖其他 worker 的脏文件。

## 请求权威与原子边界

继续使用现有 `Database` 连接、`Database.write_lock`、`TaskStore._transaction` 的 `BEGIN IMMEDIATE` 和原 WAL/FULL 设置。`LifecycleStore` 不创建另一个数据库；复用 `agent_operation` 的全局 operation_id 主键及 robot_id/fingerprint，使用既有 `AGENT_JOURNAL_STATEMENTS` 初始化同一张权威表，未修改 journal 文件。`task_lifecycle_receipt` 只是该权威记录的任务结果明细，外键绑定 operation_id，不能单独接受请求。

| 同库记录 | 职责 |
| --- | --- |
| `task_execution` / `task_execution_generation` | 沿用 TaskStore execution_id、定义与 occurrence 唯一约束；旧 generation 列保留旧语义，不冒充任务 epoch |
| `task_lifecycle_binding` | 已迁移实例的唯一 writer 归属；只有绑定后才创建新核心快照 |
| `task_lifecycle` | robot_id、version CAS、boot_id 与不可变核心快照；快照包括稳定动作实例 ID 和有序完成进度 |
| `agent_operation` + `task_lifecycle_receipt` | 唯一请求身份、完整载荷指纹、稳定接受/拒绝回执；submit 指纹包含定义与 occurrence，跨实例/机器人复用 ID 冲突 |
| `task_lifecycle_effect` | 稳定 effect_id、所属实例、完整效果（含 epoch/当前参考）、ready/sending/done/retired 与已知结果 |

接受事务原子写入 execution claim、binding、快照/进度、outbox、journal 权威与回执明细；任意一步失败全部回滚。外部端口只能在事务提交后被调用。事件与确认使用同一事务更新快照 CAS、动作进度、旧效果退役、新效果以及粗状态投影；实例没有“先存状态再补 outbox”的窗口。

请求查询 `query(request_id)` 返回持久原回执，当前状态从 `load(execution_id)` 查询。超时/取消不等于失败；同请求重试先查 journal，返回原回执及当前快照，不生成新效果。未知返回 None，不擅自重新执行。不同载荷、其他实例或其他领域已占用 operation_id 时拒绝冲突，不窃取其身份。

当前 journal 的 succeeded/failed 表达**命令接受处理结果**，任务运行/终态由 Lifecycle 快照表达；未来生产 dispatcher 必须统一其 fingerprint 与 receipt/status 展示映射，不能将接受请求的 succeeded 解读为导航任务已完成。本批未把新命令接到旧 dispatcher，也没有另设第二请求权威来绕过这一接线。

## 事务、取消和新旧 writer

`TaskStore.claim_in_transaction(connection, task, occurrence, execution_id=..., lifecycle=True)` 在调用者持有的原连接事务内复用原任务定义、地图、删除/失效与 busy 检查，绑定核心实例 ID；不自行 commit、不等待端口。原 claim 仍调用同一校验路径。

已绑定实例的旧 `mark_running/finish/mark_cancel_pending/pause/mark_needs_review` 明确拒绝写入；旧批量连接代次取消与 TaskStore.prepare 的旧重启更新排除绑定实例。未绑定实例保持旧路径，回归仍通过，迁移保留处标明 `//Compatible`。新核心是已绑定实例状态/进度的唯一 writer；它只向旧表投影供既有读接口核查，canceled 因旧枚举缺失投影为 paused，完整终态必须读新快照。旧 runner/runtime 对绑定实例的读取/调度还没有切换，不能混接生产后把拒写异常视为已完成接线。

SQLite 单写事务同时串行化同一实例；跨两个 Database 连接也由真实 SQLite 写锁及持久 version CAS 拒绝旧写入。回调另校验任务 epoch。新旧 effect 回执不能修改已退役边界；重复已完成确认不改结果、不产生新效果。恢复一个暂停实例时仍检查同机器人其他 active 实例，未扩大并发运动任务范围。

`_atomic` 对完整本地事务做 shield，收到一次或多次调用取消都等待已开始的数据库工作完成/回滚后才释放锁。取消可能发生在提交之后，此时调用方收到 CancelledError，但按 request_id 仍能查到唯一已提交结果；取消不冒充事务回滚。这里不 shield 外部导航等待，外部等待取消立即返回，留下 sending 供查询。

## 效果执行和消费协议

`LifecycleService.start()` 自动调用同库恢复；服务 API 的 submit/command/event/ack 会先确保启动恢复。真实应用须每次进程启动提供唯一 boot_id，同一进程所有包装器共享该值。新 boot 对旧实例执行核心 restart，保留身份与动作进度，提高执行 epoch、退役旧计划/旧手动通知，建立撤销义务或保持暂停；终态仍终态。没有自动发送 plan，也没有自动继续。历史未消费手动通知即使属于成功终态也退役。

`reconcile(execution_id)` 是一次有界 outbox pass，没有新后台调度器。ready 经数据库 CAS 变 sending **并提交**后才调用 `LifecyclePort.execute(effect)`；sending 只能调用 `query(effect)`，异常、超时或未知均不能盲重发物理动作。领取后尚未发送就崩溃同样是未知：查询/重启撤销协调，不能凭“可能没发送”重做。结果确认产生的新效果留给下一次 pass；长端口等待不占数据库锁，也不阻塞其他 command/event 提交。返回的 Transition.effects 用于观察，接线方不能再次遍历它们直接执行，唯一发送路径是持久 outbox。

端口协议：

- plan True 表示目标已保留，不表示任务完成；False 仍按可能产生副作用处理，先撤销。
- revoke True **必须证明该实例 through epoch 的全部来源 fence 和旧控制器输出退出**；cancel RPC 接受/返回不等于 True。False/None 保留 sending，后续只查证。
- manual_ready 是交给 T5 的通知，True/False 都持久记录为已消费，不再重发；None 为未知，只查询，query 不得重新授权。T5 必须在实际授权的串行决策点校验当前任务与 authority 准入，不能把网络通知本身当作无条件运动许可。
- 接管撤销确认、手动通知创建是一个事务。重复接管/被拒绝继续等仅新增回执的操作保留合法未消费通知；明确暂停/取消/故障、重规划或重启退役失效通知。已 consumed 通知和旧 boot 通知均不重新授权。

确认端口结果时，服务把**同步、无外部 I/O 的当前 Context provider**传给 store；store 在取得事务、校验当前版本后才调用 provider，因此不会复用等锁前或恢复边沿时的旧健康/位姿/地图。当前 T5 准入不足则保持恢复义务，后续 `admission_changed` 使用新投影消费它。T5 生产事件接线尚未实现。

稳定动作账本是同一持久快照中的 `(execution_id, action instance ID)` 及已完成前缀，不用新 goal ID 重置动作身份；重启和重规划不会重发已持久确认动作。但外部动作执行器仍必须以此键幂等，撤销边界前必须刷新已提交动作进度；本批不声称物理 exactly-once，也未验证真实来源 fence/控制器退出。

## RED/GREEN 与验证证据

先新增真实 SQLite / 可控端口测试运行，模块缺失时报 ModuleNotFoundError，随后实施。扩展测试先实际捕获并修复：终态旧 manual_ready 未在 restart 退役；确认等待数据库锁时使用旧 Context；无保留效果时 SQL NOT IN(NULL) 未退役手动通知。各反例都断言状态/效果/数据库结果，不通过重发恢复边沿或模拟 cancel 成功掩盖问题。

新增 **41 项集成场景**：接受事务每个写表点触发 SQLite ABORT 后完整回滚；端口观察到已提交身份/回执且数据库锁已释放；提交后未发送与发送后未知；发送异常/取消只查询；确认事务失败后通过查询协调而非重发；多次事务取消后的唯一结果；真实两个连接 CAS；并发实例 admission；跨实例/机器人请求冲突；旧回执/旧 epoch；进度与 failure 优先级跨库重开；手动通知消费/退役；最新 Context 与离线 reconcile。

其中三项参数化场景使用真实 Python 子进程 `os._exit`：接受 COMMIT 之前退出（无请求/实例残留）、接受提交后退出（ready 持久）、领取发送后退出（sending 持久）。父进程重开 SQLite 检查 WAL 结果、查询原请求，再执行 restart 只产生 revoke，原 plan 退役，不重复动作。它们没有启动服务/ROS/Gazebo。

最终命令与结果：

```text
.venv/bin/python -m pytest tests/unit/test_task_lifecycle.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py tests/integration/test_task_runner.py tests/integration/test_storage.py tests/integration/test_agent.py tests/integration/test_safety_intent_store.py -q
273 passed

.venv/bin/ruff format --check src/senior_control/tasks/store.py src/senior_control/tasks/lifecycle_store.py src/senior_control/tasks/lifecycle_service.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py
5 files already formatted

.venv/bin/ruff check <以上五文件>
All checks passed!

.venv/bin/mypy --strict <以上五文件>
Success: no issues found in 5 source files
```

273 包含 70 项 T6a 核心、41 项新增持久化/服务测试、75 项既有任务及 87 项存储/agent/journal 回归；不把旧任务测试中的断网取消当作新版规范。未运行禁止范围的 E2E/ROS/Gazebo，未启动开发服务。

## 后续串行接线

1. 负责人亲审本批实际 diff；T6b 未审不提交。
2. 将 agent dispatcher 的 operation fingerprint/稳定回执、任务实例查询与该同库事务入口统一；不能先 begin journal 再另行 claim 冒充原子接受，需要明确共享接线 patch，当前未改这些共享文件。
3. runner/runtime/executor 切换绑定实例为新服务唯一 writer：删除已接受任务的连接代次取消，离线仍安排本地 reconcile，保留离线新定时 occurrence 拒启及 scan 审计。
4. 接 T5 当前投影与 admission_changed；T2/T5 串行消费 manual_ready，并撤销退役/旧 boot authority。导航/控制器端口实现持久来源 fence、旧输出退出证据、效果查询与稳定动作幂等。
5. 从当前位姿/地图重规划未完成动作，迁移旧冲突测试，执行 Linux 实际 Nav2/ROS/Gazebo 撤销、断网完成、恢复与重启验收。本批 fake port 的 True 不作为真实撤销证明。

## T6b 负责人复审 R1/R2 修复（最新，待再审）

负责人复审指出上一版 273 项测试没有覆盖所有准入路径的排队时效，也没有证明协调入口的工作量有上限；该版暂未通过。本次仅修改 lifecycle_store/service、两份新增集成测试和本报告，TaskStore 本批未变更且保留 CRLF，T6a 提交的两文件保持不动，不操作 index、不提交/push、不接共享生产路径。

相对复审前版本的实际增量：

| 文件 | 增量 |
| --- | --- |
| `tasks/lifecycle_store.py` | +97/-17 行 |
| `tasks/lifecycle_service.py` | +58/-29 行 |
| `test_task_lifecycle_store.py` | +100/-33 行（含旧调用统一改用 provider） |
| `test_task_lifecycle_service.py` | +186/-2 行 |

### R1：所有准入/计划路径统一在事务内采样

原 service 的 submit/command/event 在等待数据库写锁前调用 context()，只有 ack 在事务内采样。先新增四条路径 × 两种排队期间变化的真实 SQLite 测试：submit、resume、feedback_recovered、admission_changed，分别在等锁时变为不可用/缺位姿或改变当前位姿地图，旧实现 **8 failed**，表现为仍接受并生成旧 pose/map 的 plan。

现在 store 所有接收 Context 的公共写 API（submit/command/event/acknowledge/recover）统一要求 `ContextProvider = Callable[[], Context]`，不再接受 Context 值或提供旧值兼容分支。service 只传同一个 provider。store 取得数据库锁后、检查实例/版本/回调代次后，在调用核心转换的位置采样一次；幂等回放不重新准入、不生成计划，无需采样。核心 Context 仍是原不可变值对象，未修改 T6a。

测试同时证明旧版本被拒绝前不会调用 provider、provider 调用时持有数据库锁、直接误传 Context 值会失败并回滚；原 ack 等锁后读取新参考/最新不许可的回归保留通过。纯测试的静态 Context 以 lambda provider 提供，生产必须提供同步、无外部 I/O、从当前 T5/参考投影构造值的 provider。

**数据库事务只界定这次准入决定，不能冻结之后的物理许可。** 提交后到端口接收/输出之间仍可能失效，真实执行端必须再次核验本地实时许可与 source fence；本修复不声称数据库锁解决了所有控制时序。

### R2：活动读取、确认预算与继续状态

原协调器用 effects() 全量读取历史，且确认 CAS 使用无限 while。先用 5000 条不可解码的 done/retired 历史记录证明旧协调器错误进入历史 decoder；再用每次读取后实际提交一条保留 pending plan 的拒绝命令回执，持续改变 SQLite version，证明旧循环超过 3 次仍不退出。两项均先 RED，再修改入口。

新的接口与边界：

- `active_effects(execution_id, limit=16)`：SQL 仅筛选 ready/sending，ORDER BY rowid + LIMIT，limit 限制为 1–100；使用同库部分索引 `task_lifecycle_effect_active_idx(execution_id) WHERE status IN ('ready','sending')`。实例存在性只读 SELECT 1，不为 outbox 读取解码整个核心快照。
- `effect_page(execution_id, after=0, limit=100)`：独立审计分页，返回 `EffectPage(effects, next_cursor)`，使用 rowid keyset，每次读取最多 limit+1、解码最多 limit 条；同库增加 execution_id 审计索引。页尾 next_cursor=None。
- 原 `effects()` 仅为既有诊断/测试保留，标明 `//Compatible`；它仍是全历史接口，生产协调不调用它，生产审计应使用 effect_page，不以该兼容入口宣称有界。
- `LifecycleService(..., effect_limit=16, ack_attempt_limit=3)`：每轮外部 execute/query 操作不超过 effect_limit（1–100），**整轮**本地确认尝试合计不超过 ack_attempt_limit（1–16），不是每个效果重新分配预算。确认逻辑用有限 range，无无限 CAS 重试。
- `reconcile()` 返回 `ReconcileResult(operations, ack_attempts, needs_continue)`；轮末再做 LIMIT 1 的活动查询，观察是否仍有工作，包括新生成效果、未知结果或预算耗尽。调用者安排下一轮即可，不增加后台调度器。
- CAS 预算耗尽时没有伪造确认、清 pending 或把 sending 改成 ready。已返回的外部结果仍以相同 effect_id 在接收端可查询；下轮只 query，不再次 execute。真实竞争测试证明第一次只 execute 一次、三次实际 CAS 冲突后返回继续，竞争结束的下一轮只 query 一次后落定 running。

预算约束活动输入条数、外部操作次数及本地确认次数，**不是墙钟时间保证**；启动恢复是独立的既有同库恢复阶段，数据库操作和核心快照本身仍有处理成本。端口等待截止与取消由调用方/本地适配器拥有，例如用 asyncio.wait_for 包住一次 reconcile 或由端口对 execute/query 设有限截止；无限等待的端口不能称为实时有界。外部等待取消仍传播并保留 sending，事务提交清理的 shield 语义保持不变。T5 输出时效/看门狗不由这里的批次预算替代。

新增 **14 项场景**：上述 8 条准入顺序、持续真实 CAS 竞争、历史不解码、SQL 活动数量上限/索引与审计 keyset、store 拒绝值快照、每轮端口预算及下一轮不重发、版本校验后才采样。原重启、动作进度、重复/旧回执、手动通知消费与真实进程退出测试全部保留。

最终结果（同前述 273 项选择，加本次 14 项）：

```text
.venv/bin/python -m pytest tests/unit/test_task_lifecycle.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py tests/integration/test_task_runner.py tests/integration/test_storage.py tests/integration/test_agent.py tests/integration/test_safety_intent_store.py -q
287 passed

.venv/bin/ruff format --check <TaskStore、lifecycle_store/service、两份集成测试>
5 files already formatted
.venv/bin/ruff check <以上五文件>
All checks passed!
.venv/bin/mypy --strict <以上五文件>
Success: no issues found in 5 source files
```

本次新集成场景总数为 55（原 41 + 新 14）。没有启动开发服务、ROS 或 Gazebo；T6b 仍待负责人亲审，真实 runner、dispatcher、T5 admission_changed 与 ROS source fence 接线仍未完成。
