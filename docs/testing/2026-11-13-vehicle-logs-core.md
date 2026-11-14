# T8a 独立车端日志核心（2026-11-13）

依据已批准的 vehicle-motion-redesign spec/plan Task 8，落实独立日志存储、授权端口、
有界非阻塞写入、筛选/游标查询、续传导出和有界跟随。本批不宣布整个 T8 完成。
另按负责人审核修复 T7 pending 投影，已保留与本批分开的审核修复 patch。

## 实际文件及边界

新增 `src/senior_control/vehicle_logs/{__init__,models,store,service}.py`、
`tests/integration/test_vehicle_logs.py` 和本文。仅修改 T7 的
`src/senior_control/agent/vehicle_state.py`、`tests/unit/test_vehicle_state.py` 与原 T7 报告。
settings 保持只读；未修改共享 protocol、composition、dispatcher、agent main、cloud、ROS、UI、motion、tasks 或总 plan。
未启动开发服务、仿真、实车；未操作 git index、提交或推送。负责人仍须亲审实际 diff。

审核修复 patch：`/private/tmp/task_9ac2cd84d2d8-t7-review-fix.diff`。
T8a 完整新增 patch：`/private/tmp/task_9ac2cd84d2d8-t8-logs-core.diff`。
均为实际文件生成，非 diff 摘要；新增文件未跟踪，普通 git diff 不会显示它们。

T7 修复将 confirmed_estop_status 与当前软件请求 pending 叠加层分开：同/旧快照也可清 pending，
但不能清除确认的软件/硬件急停。硬件标记现归入不可变 RuntimeSnapshot.hardware_estop，
与 sequence 一同接纳，旧/同版本标记不能重标已确认状态；T5 仍是唯一运动状态 owner。
负责人提供的复现加离线/重连和两种锁存组合，RED 为 4 个清 pending 断言失败，修复后 16 项投影测试通过。

## 职责、复用与存储选择

- models：固定查询/响应值、UTF8 字节和结构预算、输入预处理与脱敏。
- store：独立 log-only SQLite 连接/IO 线程、批量原子写、保留/容量淘汰、至多 200 行扫描的查询。
- service：可信权限端口、1024 队列、单写入 worker、独立 retention worker、丢失汇总、查询资源配额及跟随 outbox。

已审 `observability.py`：现有 JsonFormatter 通过 allowlist 去掉任意消息，服务后台 Metrics 是闭集指标；
本批不放宽该 formatter，不把任意日志接入现有控制日志调用链。新增核心未来可接其结构化事件，
Python logging/ROS 日志源适配仍待接线。本批沿用现有 aiosqlite 与 SQLite 事务方式，但不复用控制/settings 的
Database 连接或 write_lock，也没有创建重复的任务/配置表。

可信 composition 注入一个私有日志目录，文件名固定为 vehicle-logs.sqlite3；外部请求没有路径或 shell 参数。
独立 SQLite 连接只有日志表 log_meta/log_entry，且它自己的锁只协调日志 IO。
MEMORY rollback journal、MEMORY 临时存储和 SQLite max_page_count 使默认磁盘文件硬上限为 64MiB，
无 WAL/磁盘 rollback journal 的额外增长。拒绝不支持的页大小、已超限文件和目标文件符号链接。
逻辑 payload 保留默认至多 16MiB（硬上限的 1/4），为 SQLite 页、索引及碎片留出空间；
不把 64MiB 误说成可用净日志内容。测试可由可信构造参数缩小磁盘上限，外部 API 不开放此参数。

这是尽力保留的日志数据库，MEMORY journal 不提供掉电安全 journal 的保证；损坏报 unavailable，不自动删除重建。
每次重启在现有流追加 restart_unknown gap，loss_count=None 表示重启前未刷队列的实际丢失数量未知。
持久 high_water/floor 保障正常重启不复用已提交序号。数据库被人工删除重建不属于连续流，transport 必须作为流重置处理；
不能在不同车辆之间复用数字游标。

## 核心 API 与可信授权边界

`VehicleLogStore(directory, disk_limit_bytes=...)` 是 composition 管理的 async context manager。
其 append/query/prune 为可信内部 IO 端口，不可直接映射为外部路由。

`VehicleLogService(store, vehicle_id, authorizer, settings, utc_now=...)`：

- settings 是同步回调，只返回 `SettingsService.published_snapshot`。本批没有 settings get/apply 或 RuntimeGate 依赖。
- authorizer 实现 `await require_vehicle(credential, vehicle_id, action)`，必须用真实认证会话/签名凭证验证
  当前身份、授权车辆和 action，失败抛 PermissionError；不是请求自己传 authorized=True。
- query/download/follow 都先拒绝非本车 vehicle_id，再调用该可信端口；IO 后也重新检查授权。
  跟随 next_page 在等待前后验证，权限撤销会关闭该订阅；凭证不进入日志记录、回执或导出。
- `await start()` 启动一个写入 worker 和每 30 秒运行的 retention worker；`await close()` 先封闭生产者，
  关闭跟随并限时 flush，再取消后台任务；持久 IO 持续失败时 flush/close 会超时并显式抛错，不能报告全量已落盘。
- `enqueue(LogInput(...)) -> bool` 为同步、线程安全、无文件 IO/await 的有限工作。
  True 仅表示进入内存队列，不是 durable 回执；低于配置 level 的数据被有意过滤，False 不一定是故障。
- `await flush()` 为可信生命周期/测试屏障，不是控制路径操作或外部 API。
- `await cleanup()` 根据当前 retention_seconds 清理，仅操作日志数据库，不产生 MotionEvent 或任务动作。
- `await query(credential, vehicle_id, LogQuery(...)) -> LogPage`。
- `await download(...) -> LogDownload`：NDJSON bytes + next_cursor/has_more/gaps。
- `await follow(...) -> LogSubscription`，`await subscription.next_page()`、`await subscription.close()`。

生产 Auth/WS/云用户车辆关系尚未接入。测试使用独立 session->vehicle 验证器，证明端口及拒绝行为，
不把测试 token 验证器声称为真实认证实现；后续 composition 必须注入现有认证链。

## 资源限制与明确缺口

写入队列至多 1024 条；单次批写至多 64 条，跨线程唤醒合并为至多一个待执行回调。
队列已满时先短路，避免持续洪峰还进行无必要编码。enqueue 不等待文件、SQLite、异步锁或消费者；
线程锁仅保护短内存操作。正则输入和结构化字段均有固定上限，不能按任意消息长度工作。

- detail 在脱敏前和脱敏后均至多 4096 UTF8 bytes，summary 512 bytes；额外结构化 fields 序列化至多 4096 bytes，
  至多 64 节点、4 层、每容器 16 项。超大单条不截断成伪完整事件，拒绝并记 oversize gap。
- 结构化 password/secret/token/credential/authorization/cookie/api key/device key/pairing key/code 等键递归脱敏；
  常见文本 key=value、带引号的密码、Bearer/Basic、Authorization/Cookie 头和 URL 用户密码脱敏。
  模块和关联 ID 不接受含此类 secret 形式的值；读取/导出再次脱敏且保持 fields_json 合法。
  这不等于任意秘密识别/DLP，日志源适配仍应避免原始凭证输入。
- 查询最多同时 2 个操作，额外请求立即 quota 拒绝；单页至多 200 条、完整 JSON 小于等于 256KiB。
  过滤最多扫描 200 个物理行；即使没有匹配也用 next_cursor 推进，不能全库扫描后伪报空结果完整。
- 导出最多 16 个扫描页（3200 物理行）且 payload + 预留元数据不超过 2MiB，达到任一预算都返回
  has_more=True 与精确续读位置；不会重复已返回记录或跳过未返回匹配。可取消，资源槽在 finally 释放。
- 跟随至多 32 个订阅，每个 outbox 至多 4 页。慢消费者溢出合并 slow_subscriber 游标区间，
  后续再溢出也保留最早缺口，不悄悄丢 gap。订阅关闭会唤醒等待者。
- 跟随回放和 live 发布在独立日志 publication 锁内交接，不能重复或漏掉竞态中提交的记录。
  首次回放限一页；若仍有历史，follow_backfill 明示 next_cursor..high_water 区间需另行 query 回补。
  live 页面 next_cursor 未到该批 high_water 时仍 has_more=True，不伪称整批完整。
- 保留/容量均删连续前缀并持久推进 floor；游标早于 floor 时返回 durable evicted gap，
  即使全部记录已被清理，也不会把空结果冒充完整历史。
- queue_full/oversize/invalid/storage_error 汇总为固定有限字典，恢复写入后作为 gap 记录进入同一序号流。
  磁盘仍不可用时 query.gaps 暴露 durable=False 的未落盘计数与闭集错误类别，不泄露原异常消息。
  写入故障和保留清理故障分开持有；写成功不能掩盖清理失败。重启只能给出未知 gap，不猜测精确计数。

has_more 描述当前已提交 high_water 内是否仍可续读；不表示所有生产者都已 flush。
尚在队列的正常记录不是丢失，也没有 durable 承诺。过滤不会隐藏 gap 记录。
retention 变化在下一次 cleanup（后台间隔 30s）消费；时钟回拨产生异常时间顺序时可能保守淘汰更长前缀，
同样以 floor/gap 表达。日志 IO 堵塞会导致日志丢失，而不会等待控制/设置锁。

## T7 debug owner 与 T9 消费示例

T8 没有 debug deadline、恢复计时器或第二份原级别。只读 effective_log_level 和 logging.retention_seconds；
临时 debug 首次原级别/截止、离线恢复与重启处理继续由 SettingsService 唯一拥有。
composition 仍需启动其 start_debug_restore，并在数据库关闭前关闭它。
本批集成测试直接消费真实 T7 published_snapshot，证明 INFO 过滤 DEBUG、开启后接受、T7 恢复后再次过滤；
还证明持有真实 settings write_lock/RuntimeGate 时日志 flush/query 仍可完成。

可信 composition 接线示意（本批未修改 composition）：

```python
async with VehicleLogStore(private_log_directory) as store:
    logs = VehicleLogService(
        store,
        local_vehicle_id,
        real_session_authorizer,
        lambda: settings_service.published_snapshot,
    )
    await logs.start()
    try:
        logs.enqueue(
            LogInput(
                level="INFO",
                module="navigation",
                summary="task accepted",
                source_utc=event_utc,
                task_id=task_id,
                request_id=request_id,
            )
        )
        # 日志源 handler 只调用 enqueue，不 await flush/query。
    finally:
        await logs.close()
```

T9/transport 查询与导出：

```python
page = await logs.query(
    session_credential,
    vehicle_id,
    LogQuery(
        after=last_cursor,
        levels=("WARNING", "ERROR"),
        task_id=task_id,
    ),
)
# 显示 entries、gaps；保存 (vehicle_id, page.next_cursor)，page.has_more 时继续。
# 即便 entries 为空，只要 has_more=True 也应续读，不能把过滤扫描预算当成没有更多日志。

part = await logs.download(session_credential, vehicle_id, LogQuery(after=last_cursor))
# part.data 为 NDJSON；part.has_more=True 时用 part.next_cursor 请求下一段。
# gaps 必须随下载元数据回传并显示，不得只丢给浏览器一个声称完整的文件。

subscription = await logs.follow(session_credential, vehicle_id, LogQuery(after=last_cursor))
try:
    live_page = await subscription.next_page()
    # follow_backfill / slow_subscriber 提示补读，按 sequence 去重，浏览器保留窗口仍须有界。
finally:
    await subscription.close()
```

固定 record 字段为 sequence/kind/level/module/summary/detail/fields_json/source_utc/received_utc/
request_id/task_id/loss_reason/loss_count；gap 字段 reason/count/after/through/durable。
字段不含文件路径、原凭证、运动命令或 UI 磁盘配额。

## RED/GREEN 与实际验证

先写新接口测试，初始 RED 为新模块尚不存在；之后分别观察 header token 泄露、重复溢出最早缺口丢失、
IO 期间撤权导出仍返回、脱敏扩张超字节上限、live 页过早宣称完整、清理错误被掩盖的断言失败再修复。
新增 19 项日志测试覆盖多字节边界、真实 SQLite full/恢复、持续洪峰、事件门控慢存储、查询取消/配额、
页/导出字节上限及无漏重续读、稀疏扫描预算、重启 gap、权限/跟随竞态、脱敏和 T7 策略消费。

T8a 首轮相关命令（R1/R2 审核修复前的历史结果）：

```text
.venv/bin/python -m pytest tests/integration/test_vehicle_logs.py tests/unit/test_vehicle_state.py tests/integration/test_vehicle_settings.py tests/integration/test_storage.py tests/integration/test_backend_observability.py tests/integration/test_cloud.py tests/integration/test_agent_snapshots.py tests/integration/test_workspace_data.py -q
132 passed in 5.07s

.venv/bin/ruff check src/senior_control/vehicle_logs src/senior_control/agent/vehicle_state.py tests/integration/test_vehicle_logs.py tests/unit/test_vehicle_state.py
All checks passed!

.venv/bin/mypy --strict src/senior_control/vehicle_logs src/senior_control/agent/vehicle_state.py
Success: no issues found in 5 source files
```

另外尝试了仓库根完整 `.venv/bin/python -m pytest -q`，结果 **12 skipped、5 errors during collection，exit 2**，
不是全量通过；未修改无关文件以掩盖收集问题：

1. ros2/src/senior_scan_matcher/test/test_replay_cli.py：独立 CLI 脚本把 pytest 的 `-q` 当可执行文件，FileNotFoundError。
2. ros2/src/senior_slam_bridge/test/test_candidate_preflight.py：缺少 candidate_preflight 模块导入路径。
3. tests/unit/test_home.py：与 tests/e2e/test_home.py 同名模块 import mismatch。
4. tests/unit/test_recovery.py：与 tests/e2e/test_recovery.py 同名模块 import mismatch。
5. tests/unit/test_velocity_uncertainty.py：与 tests/ros/test_velocity_uncertainty.py 同名模块 import mismatch。

慢存储测试持有 append 事件，连续 5000 次 enqueue 仍有界返回，独立伪控制事件能推进；
这只证明代码不等待日志 IO，不是 Linux 最坏 tick/急停实时预算证据。
剩余真实认证/WS/云路由、日志源 handler、UI 跟随/下载、retention 和 SettingsService 生命周期接线、
Linux 洪峰控制优先时序及实际运行验收以后串行推进；未声称整项 T8 完成。

补充工作区检查：本批两个 patch 的逐文件 whitespace check 无输出；全仓 git diff --check 在并行 T6 的
`src/senior_control/tasks/store.py` 新增 CRLF 行报告 trailing whitespace，本 worker 未修改该文件，需其 owner/负责人处理。


## 负责人 R1/R2 审核修复（2026-11-13）

本轮仅修改 vehicle_logs/service.py、store.py、test_vehicle_logs.py 和本文，
未改 settings/T5/T6/共享接线/plan/UI，未操作 index、提交、推送或启动服务。
独立小 diff：`/private/tmp/task_9fc6a2f377e9-t8-r1-r2.diff`。

R1：先复现空库 follow 后 64 条短事件被拆成 64 页，等待中的消费者只拿到第 61 条，属于错误的发布策略。
现在按 query.limit（最多 200）及实际序列化 UTF8 字节打包批次，计入 envelope 和最大可能的合并 gap 开销，
每页不超过 256KiB。64 条短事件完整放一页；必须多页时，每次非阻塞 offer 后让出一次调度，
快速消费者可继续接收，不等 outbox 空位/客户端确认，也不扩大四页 outbox。
真正连续慢消费者仍丢弃并合并缺口；gap 的 through 改为新页第一条序号之前，
不把同页已交付条目误标成丢失。末页 cursor 推进到该批 high_water，含过滤掉的尾部，has_more=False；
中间页 cursor 为该页最后序号，has_more=True。旧的三条事件每条占一页断言迁移为 limit=2 确需分页的行为测试。

R2：先复现 meta 缺失、计数/范围矛盾仍被打开。只有路径原本不存在的全新日志文件允许事务初始化；
既有空文件也明确 unavailable，既有数据库不再执行 CREATE IF NOT EXISTS / INSERT OR IGNORE。
启动时校验两表的列/主键、唯一 meta 行、整数/版本/非负计数和容量，以及保留记录连续范围、
count==high_water-floor、wire_bytes 与真实 UTF8 payload 字节统计。初始化扫描受 64MiB 文件上限约束，
仅在打开阶段执行，不给 enqueue/query/append 增加全表扫描。11 种损坏状态均拒绝打开且文件字节不变；
合法初始空库和全部清理后的 floor==high_water 正常重开，并从原 high_water+1 继续。
不自动重造高水位或修复损坏文件。

新增/迁移覆盖：等待中的快速消费者、64 条短批次、UTF8 和 JSON 转义膨胀导致的真实跨页、limit 跨页、
过滤后的 cursor/高水位、持续真实溢出、meta 缺行/缺表/错版本、非法计数、记录范围/中间缺号矛盾，
以及合法空库/清空后重开；日志定向测试现为 38 项。RED 为 11 个实际失败断言，另有新增 gap 区间断言
发现 through 曾涵盖已交付批次，修复后通过。权限、撤销、导出和既有资源保护回归保留。

本轮最终验证：

```text
.venv/bin/python -m pytest tests/integration/test_vehicle_logs.py tests/unit/test_vehicle_state.py tests/integration/test_vehicle_settings.py tests/integration/test_storage.py tests/integration/test_backend_observability.py tests/integration/test_cloud.py tests/integration/test_agent_snapshots.py tests/integration/test_workspace_data.py -q
151 passed in 5.58s

.venv/bin/ruff check src/senior_control/vehicle_logs tests/integration/test_vehicle_logs.py
All checks passed!

.venv/bin/mypy --strict src/senior_control/vehicle_logs
Success: no issues found in 4 source files
```

按本任务未重跑已知有 ROS 收集错误的全仓 pytest，也未修改其他 worker 的 CRLF。
以上不构成 Linux 最坏调度/急停预算或真实认证/WS/控制接线验收，仍待负责人亲审后安排限定提交与后续接线。
