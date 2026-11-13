# T7a 设置持久化与纯状态投影（2026-11-13）

本批仅交付独立可测试车端模块，不宣布整个 T7 完成。依据已批准的
`docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md` 和对应计划 Task 7/8/9。
未改 protocol/composition/dispatcher/main/ROS/client、T5 模块、T6 lifecycle、全局迁移或其他既有脏文件；
未启动服务、仿真或硬件，未操作 git index、提交或推送。负责人仍需亲审实际新增文件 diff。

## 实际新增文件与职责

- `src/senior_control/settings/__init__.py`：导出 SettingsService/RuntimeGate。
- `settings/models.py`：冻结且严格验证的五项驾驶配置、四项日志配置、profile、完整快照、回执和能力范围。
- `settings/store.py`：Database 写锁、BEGIN IMMEDIATE、原子配置与回执提交、完整持久图像校验。
- `settings/service.py`：唯一设置写入/发布 owner、驾驶运行门、版本/请求去重、临时 debug 恢复 owner。
- `src/senior_control/agent/vehicle_state.py`：消费 T5 已决模式和允许动作的纯投影，不执行 reducer、控制或任务动作。
- `tests/integration/test_vehicle_settings.py`、`tests/unit/test_vehicle_state.py`：针对性失败与成功用例。
- 本文记录接口、结果与未接线限制。

全部为新文件，普通 `git diff` 不显示未跟踪文件；逐一用 `git diff --no-index -- /dev/null <path>`
查看完整实际 diff，exit 1 表示正常存在差异。禁止把其他 worker 的脏文件混入本批限定提交。

## 设置接口与持久化语义

`SettingsService(store, profile, runtime, utc_now=..., steady_now=..., utc_trusted=...)`：

- `await prepare()`：创建独立幂等表并加载完整配置，启动前完成已到期/不可信时钟的 debug 恢复；
  无全局 user_version 或 cloud schema 变更。未知本地 schema、缺组、缺字段、缺主记录及非法值明确 unavailable，
  不默默补半组默认。只有两个设置表均不存在才初始化产品默认值。
- `await get() -> SettingsSnapshot`：刷新 debug 到期状态并查询；不能完成到期持久化时抛错，不回报恢复成功。
- `published_snapshot`：不等待、不做 IO 的最后已提交不可变图像，供控制 tick/日志热路径读取。
  初始化恢复完成前不可读；事务中仍为旧对象，提交后一次发布新对象，旧引用不变化。
- `await apply(SettingsRequest(...)) -> SettingsResult`：整组替换，固定 `group=driving|logging`、
  `request_id`、`expected_version`、完整 `values`；`restore_defaults=True` 使用 profile 默认并经过相同运行门/校验/事务。
- `await result(request_id)`：查询原回执，不存在返回 None。网络超时/取消应显示待确认并查询，不猜测成功或失败。
- `capabilities` 及每个回执的同名字段：提供 driving 的 defaults/minimum/maximum、logging defaults 与允许时长范围。
  `SettingsResult.snapshot` 是该回执当时的实际生效值及组版本；`reason` 为 applied、version_conflict、
  stop_required、invalid_values、request_conflict。重试历史回执后，要用 get 回读现在的配置。

五项驾驶字段只有 low_linear_mps/high_linear_mps/low_angular_rps/high_angular_rps/reverse_linear_mps，
内部 SI。所有字段拒绝布尔冒充数值、NaN/Inf、负值、profile 越界；低档线速不得超过高档。
`DrivingSettings.reverse_limit(high=...)` 返回 min(档位线速度, reverse_linear_mps)。profile 显式注入；
本批测试中的低档 5/9、高档 25/18 m/s、两档角速度 1 rad/s 是产品可配置值，测试上界为 synthetic，
均非物理能力或实际 5 km/h 证据。没有 PI、车体标定或磁盘上限输入。

驾驶和日志组版本独立递增。每个请求的 fingerprint 覆盖组、expected_version、restore_defaults 与 values；
同 ID 同内容返回完全相同历史回执，同 ID 不同内容冲突且不覆盖原回执。拒绝结果也持久化；
落盘异常则不产生回执，不改变组版本或已发布值。配置和成功回执同一个事务，防止崩溃后两种 owner 各认一份结果。
BaseException 路径 rollback；服务的提交操作等待取消结束，确保 SQLite 物理 commit 与图像发布之间不能被取消切断。
不同 SettingsService 意外成为第二写入 owner 时，事务内校验完整旧图像并拒绝覆盖，不能用过期 cache 覆写新值。

本数据库必须是单车本地数据库，只创建一个 SettingsService。T6/AgentJournal 未来仅路由此 request_id 和查询本回执；
不得再执行一次设置、再创建一份“设置已完成”的权威结果，或使用 running->needs_review 推翻本事务原子结果。
T6 的其他任务生命周期仍由 T6 拥有；本批没有重造任务调度框架。

## 运行门与串行接线契约

RuntimeGate 是可执行的排他协议：驾驶 apply 持有同一个 gate 锁，从读取运行状态一直到 durable commit 和发布。
`RuntimeGate.transition(async_operation)` 在同锁内完成运动准入/任务开始或继续/手动激活及状态发布；
操作失败或取消会留下 unconfirmed 的保守状态。初始状态不允许驾驶设置。
`RuntimeGate(observe=current_runtime_check)` 可以在该锁内同步读取最新 T5 静止证据（包含 steady 时效）、
T6 executing 状态和 manual owner，避免只检查调用方事前传来的过期 MotionState。
`update(...)` 用于观测/测试，不可在运动已经获准后才异步补记。

后续 composition 必须：

1. 唯一实例化这个 gate，所有允许新运动的路径使用 transition；不能还有绕过 gate 的 Nav2/手动输出激活路径。
2. observer 必须在调用时核验 T5 静止证据仍有效，不能仅返回很久以前的 stationary=True。
3. observer 返回 RuntimeCheck(stationary=..., executing_motion_task=..., manual_active=...)。
   只有确认静止、无 executing 运动任务、手动已释放时可应用驾驶配置；暂停任务可改，但不发起 resume。
4. 普通控制 tick 读 published_snapshot，不 await get，不等待日志或设置 IO；紧急安全输出保持原独立高优先链。
   gate 是运动准入与设置的串行边界，不应把所有控制/急停 tick 都接成等待数据库的 transition。
5. 锁顺序为设置 owner -> driving gate -> Database.write_lock；transition 内不得回调设置 apply，
   以免自锁；它应直接返回 RuntimeCheck，结束后再独立发配置请求。

logging apply 不获取 runtime gate，也不产生 MotionEvent；运行中、任务执行中和驾驶 gate 忙时仍可修改日志。
测试用暂停实际写入验证驾驶事务期间运动 transition 不可进入，并覆盖取消后仍能查询成功结果；
用注入 observer 验证提交时重新读取已过期/恢复的静止证据。
这些协议本批已实现并可验证，但既有生产 composition 尚未接入，不能声明当前运行链已消除设置/起步竞态。

## T8 唯一 debug 恢复 owner 与消费示例

本批明确由 SettingsService 拥有临时 debug 截止、首次原级别、恢复持久化和定时循环；T8 不另起计时器。
logging 固定字段为 level、retention_seconds、debug_enabled、debug_restore_seconds，保留 3600–2592000 秒，
默认 604800；debug 60–3600 秒，默认 300。完整四字段提交，缺字段或额外字段拒绝。

首次 debug 启用保存开启前常规 level 和 UTC 截止，并建立进程内 steady 截止；重复 ID 或新 ID 的持续启用均不续期，
也不覆盖首次原级别。debug 期间 logging.level 显示保留的常规级别，effective_log_level 为 DEBUG。
关闭/到期恢复首次原级别；显式 restore-defaults 恢复 profile 默认。日志其他字段可独立随整组更新。
到期恢复只递增 logging_version。重启使用原 UTC 截止及剩余期限，时钟不可信/非有限时立即恢复；
在线 steady 截止避免 UTC 回拨延期，UTC 前跳可提早恢复。时间可信性由车端注入，不能信任网页时钟。

composition 的生命周期调用顺序为 prepare -> start_debug_restore -> 运行 -> close -> 关闭 Database。
定时器每 250ms 检查一次，与网页连接完全无关；落盘失败保留旧值及版本、不假报恢复成功，
`debug_restore_error` 报告故障并由同一 owner 重试。get 也在过期落盘失败时明确抛错。
实际 timer 调度延迟和磁盘故障期间无法承诺按时完成持久化，不能用单测代替 Linux 负载验收。

T8 稳定消费示例（假设 composition 已构建并启动 service）：

```python
snapshot = service.published_snapshot  # logger 热路径只读，无 IO
logger.setLevel(snapshot.effective_log_level)
retention_seconds = snapshot.logging.retention_seconds
# retention worker 使用 retention_seconds；logger 仍只向 T8 有界队列 enqueue。
# 不自行保存/改写 debug_deadline_utc，不自行续期或恢复。
```

T9 提交/回读示例：

```python
current = await service.get()
receipt = await service.apply(SettingsRequest(
    request_id="settings-request-123", expected_version=current.logging_version,
    group="logging", values={
        "level": "INFO", "retention_seconds": 604800,
        "debug_enabled": True, "debug_restore_seconds": 300,
    },
))
# receipt.reason == "applied" 才展示本次成功；超时则 await service.result(id)。
# 展示 receipt.snapshot.logging、logging_version 和 receipt.capabilities；刷新用 get。
```

这里返回的是设置策略生效图像；实际 Python/ROS 日志级别、日志保留清理、授权查询和队列仍待 T8 接入，
本批没有声称实际 logger 已接好或日志洪峰控制优先通过。

## VehicleState/T9 消费契约

`project_vehicle_state(RuntimeSnapshot(...), ConnectionState(...), ...) -> VehicleState` 无副作用。
RuntimeSnapshot 由可信车端 composition 提供 sequence、T5 MotionState、真实 task_status/progress、BodyMotion、
body_source、wheel_linear_mps、received_steady。它不伪造任务状态，不重写 T5 优先级/reducer。

- 主字段 mode、reason 来自 T5 已决 mode 类别（estop 可区分 hardware/software）；末次 reducer 事件仅放 detail_reason。
  因此 fault 锁存后即便末次事件是 obstacle，主原因仍是 execution_fault。
- 原样消费 T5 allowed_actions，离线隐藏提交动作；hardware_estop 时额外移除 estop_reset。
  software_estop_pending 只产生 pending 标签，不把 TASK 等模式改成已急停。硬件标识必须与该车端快照一致。
- estop_revision、fault_revision 独立；拒绝 sequence 不新或任一安全版本倒退的快照，仍可刷新连接诊断和测量年龄。
  sequence 必须由可信 transport/composition 跨启动 epoch 正规化；本批不接协议，不自行猜测 epoch 顺序。
- connection.connected/reconnecting/rtt_ms 完全独立，RTT 不作为资格门。断线 last_known=True，
  原任务 running 保持 running 并 task_may_continue=True，不伪报停止或取消。
- 主速度 body_average 只来自合格 BodyMotion.average；没有 body 时保持 None，绝不从 wheel_linear_mps 填充。
  同时提供 body_source、body_age_seconds、body_stale、body_computed_steady/valid_until_steady，轮速独立诊断。
  过期保留最后实测并标 stale，不把最后测量当当前资格。源与统计/退化性质仍由 T3 提供。
- task_id/status/progress 是 T6 的投影字段；完整任务回执/操作查询仍由 T6 transport 适配，不复制 journal。

T9 只需消费归一化的 mode/reason、allowed_actions、body_*、task_*、estop_status 和独立连接区，
不要再次用 RTT、轮速或网页急停点击推导车辆资格/已停车。未执行 UI 或网页截图验收。

## RED/GREEN 与实际验证

先创建测试再执行，RED 为两个新模块缺失的 ModuleNotFoundError；实现后扩展事务、取消和损坏负例。
一次固定 300ms 的 timer 测试在整组运行受调度影响失败，已改为等待实际恢复写入事件，未放宽产品 deadline。
最终命令与结果：

```text
.venv/bin/python -m pytest tests/integration/test_vehicle_settings.py tests/unit/test_vehicle_state.py tests/integration/test_storage.py tests/integration/test_agent_snapshots.py tests/integration/test_workspace_data.py -q
89 passed in 1.56s

.venv/bin/ruff check src/senior_control/settings src/senior_control/agent/vehicle_state.py tests/integration/test_vehicle_settings.py tests/unit/test_vehicle_state.py
All checks passed!

.venv/bin/mypy --strict src/senior_control/settings src/senior_control/agent/vehicle_state.py
Success: no issues found in 5 source files
```

覆盖全部驾驶字段非法值与组合、完整日志校验、并发版本冲突、组版本独立、请求重试/冲突/查询、
事务失败回滚、物理 commit 前后取消、重启保持、损坏/部分缺失/未来 schema、恢复默认运行门、
静止/运动任务/手动运行门、guard 排他、日志不等 guard、不可变快照发布、第二写 owner 拒绝、
debug 原级别/不续期/离线 timer/重启不可信时钟/落盘故障重试、状态优先、断线继续、主速度/轮速分离及旧快照拒绝。
未验证全产品集成、权限接线、实际日志、真实控制输出、仿真/实车速度或 T8 负载预算；T7 集成与 T8/T9 仍待后续串行接线。

## 负责人审核修正：pending 显示层（2026-11-13）

新增 confirmed_estop_status 保存已接纳车端快照的确认状态，estop_status 每次由确认状态与当前
software_estop_pending 请求叠加计算；相同/旧遥测也可在请求失败、撤销、完成后清除 pending。
已确认软件或硬件锁存优先，不会被 pending=False 清除。hardware_estop 已放入不可变 RuntimeSnapshot，明确属于传入
RuntimeSnapshot.sequence，仅整份新快照获接纳时消费；相同/旧快照附带的硬件标记不能重标已确认状态。
不改变 T5 reducer 或 settings。RED 为 4 个 pending 清除断言失败；修复后 test_vehicle_state 16 passed，
覆盖同/旧快照、离线/重连、两种锁存以及相反的旧硬件标记。审核修复独立 patch 导出至
`/private/tmp/task_9ac2cd84d2d8-t7-review-fix.diff`，与原 T7a 全量 diff 分开。
