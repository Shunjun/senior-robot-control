# T7/T8 实际设置与日志接线审阅材料

本批 `task_c2da3f03056b / ctx_2ae440b0059e` 在独立 worktree
`/Users/zhangshunjun/workspace/orca/senior-robot-control/vehicle-settings-logs-wire` 实施，
分支 `Shunjun/vehicle-settings-logs-wire`，基线 HEAD
`dae9f56884fbc9f3ea59c7e8e45a6fee6e71b824`。开始时该 worktree 干净。
原仓库仅用于读取未提交的权威 specs/plans、T7/T8 核心及手动接线说明和现有依赖；
未复制脏树，未操作任何 index/commit/push，未启动开发服务、ROS、Gazebo 或修改 VM。
当前交未提交 diff，待负责人亲审后再限定提交/推送/PR，不自动合并。

## 实际接口映射

| 现有网页调用 | 实际链路/权威 owner |
| --- | --- |
| `settingsRead()` → `settings.get {}` | 认证 `/ws/control` → relay 定向请求 → Agent → 唯一 `SettingsService.get()`；回 `{snapshot,capabilities,can_apply_driving}` |
| `settingsApply(request)` → `settings.apply` | payload 原样含 request_id、expected_version、group、完整 values、restore_defaults；直接调用同一 SettingsService，版本/幂等/事务与回执仍只归核心 |
| `settingsResult(id)` → `settings.result` | 同一 service 查询历史回执；不存在明确返回 `data:null`；不会走另一份 AgentJournal 设置回执 |
| `VehicleLogPort.query/download/follow` | 新默认 browser adapter → 独立认证 `/ws/logs?robot_id=...` → 独立 `/ws/agent/logs` → 原 `VehicleLogService`/私有 `VehicleLogStore` |
| 暂停/恢复跟随 | 关闭对应 follow socket；恢复带上已有 after 游标，新连接先补历史；源采集与持久化不随页面暂停 |
| 分段下载 | 固定结构化 query；返回 NDJSON data/next_cursor/has_more/gaps，沿原车端扫描和字节配额；没有路径参数 |

沿当前 web schema，不沿历史 ui-contracts 提案另造协议。未改组件、样式或驾驶数值模块。
唯一 profile 仍是已有显式 `SENIOR_SIMULATION_SETTINGS=1` 选择的 `simulation_settings_profile()`，
未增加默认 profile，也不会自动把仿真策略当作实车标定。未选择 profile 时不启用本批设置/日志能力。

## RuntimeGate 与装配

`SettingsRuntime` 只适配现有 `IpcState` 和现有 `LifecycleStore`，不复制任务状态算法。
驾驶写入持有同一 gate，刷新实际 IPC/持久生命周期，事务中再次按当前 steady 时间核验 runtime
采样窗口及独立证据截止；必须 stationary、无 prepared/active 手动 owner、无执行/撤销义务。
T6 paused 且无 pending 可以修改，写设置不恢复任务。未来时间、过期样本、读失败均不放行。
手动准入/意图、显式导航/任务命令和原任务 runtime tick 使用同一锁；急停、释放保持原链路，
没有添加新运动资格条件。锁顺序仍是 settings owner → gate → Database.write_lock。
设置仍只落原车端数据库，五项驾驶/四项日志参数和能力范围全部复用核心。

只在 settings service 已装配时投影 `settings.v1`；日志 service 已装配且独立认证连接建立后投影
`logs.v1`。真实 IPC runtime 不可用时仍沿原 vehicle_ui 行为，不伪造 runtime。

## 日志源、认证与生命周期

实际源是 Python Agent 的 `agent_log_source_started`、`agent_command_result` 以及原 Agent logger 事件，
带模块/源时间/request_id、可用的 task_id 和安全的事件字段；`response_ok` 仅表示响应正常，
不把传输回执解释成车辆已运动。handler 仅非阻塞 enqueue，沿原脱敏、1024 输入队列、64MiB
车端私有 store、200条/256KiB页面与2MiB下载上限；大帧 JSON 编解码在线程执行。
**ROS `/rosout` 源未接入**，本批不修改 ROS/guard/launch，也不声称已采集 ROS 日志。

日志浏览器连接每次请求与交付前检查真实 session/车辆访问；空闲也每秒检查撤销/过期。
Agent 日志连接独立验证实际 device token，不分配或改变控制 epoch。
车端只接受已认证 TLS 云连接颁发的随机、随连接/订阅关闭而撤销的临时 opaque grant，
没有 `authorized=true` 或网页自报 principal 的入口。viewer outbox 最多四帧，慢/断连接明确关闭，
核心 follow 的有界队列与 gap 保留；日志请求/IO/编码不进入控制 outbox。

日志策略、retention 消费同一个 SettingsService.published_snapshot；只由该 service 启停唯一
debug 恢复 timer，网页断开不影响到期恢复。日志目录打开失败不启用日志能力、控制继续；
清理日志失败也通过 finally 关闭设置 owner/timer。日志路径只由 composition 固定为数据库旁
`logs/vehicle-logs.sqlite3`，请求无法选路径。

## 验证

新增 `tests/integration/test_settings_logs_wire.py`：9 passed。
真实认证 ASGI relay → Agent 收包入口 → 原 SettingsService/SQLite，验证认证读改回读、原请求重放/
结果查询/缺结果、版本冲突、日志运行中改写和关闭重开持久；第二个控制者不接收设置回执。
另用真实 socketpair 长度帧 IPC/IpcState 与真实 LifecycleStore 验证新鲜/过期 runtime、running、
revoking 拒绝、paused 允许且不恢复；prepared/未停稳/未来样本也拒绝。
日志测试接实际 handler、原服务/store 与认证 ASGI 独立连接，覆盖产生/脱敏、查询过滤、跟随、
暂停期间继续采集、恢复补页、16+4条真实分段下载、无 cookie 拒绝、非法路径参数拒绝、session
撤销关闭，以及同一设置 owner 的过滤/debug timer 离线恢复。
IPC 中的 T5 状态是显式可控测试数据，没有运行真实 T5 节点。

相关回归命令（PYTHONPATH=src，PYTHONDONTWRITEBYTECODE=1，使用现有只读 venv）：

```sh
python -m pytest tests/integration/test_settings_logs_wire.py tests/integration/test_manual_wire.py tests/integration/test_agent.py tests/integration/test_cloud.py tests/integration/test_vehicle_settings.py tests/integration/test_vehicle_logs.py tests/integration/test_agent_snapshots.py tests/integration/test_task_lifecycle_store.py tests/unit/test_protocol.py -q -p no:cacheprovider
```

结果：263 passed in 15.74s。此后仅将日志大帧 JSON 处理移至线程并补 shutdown finally，定向9项
再次通过（2.21s）。本轮全回归发现的新 adapter 导入循环已修复，T6 真实子进程恢复测试通过。

网页相关 `logs.test.ts / client.test.ts / vehicleState.test.ts / settings.test.tsx / logs.test.tsx`：
5 files、25 tests passed。`tsc --noEmit` 通过；使用只读依赖链接与临时 config runner，全部 Vite/
Vitest cache 定向 `/private/tmp`，依赖链接/临时 config 已删除，没有写原依赖树。
Ruff check/format 和10个生产 Python 文件 strict Mypy 通过，`git diff --check` 通过。

这证明基本正常功能及必要的认证/版本/持久/门控边界；不证明 ROS 源接入、实际车辆运行、Linux
最坏调度/急停预算或持续洪峰表现。当前未启动任何服务，不做这些专项实测的成功声明。
负责人须审最终 diff，并与运动分支同版合并后完成实际 runtime 正常验收。
