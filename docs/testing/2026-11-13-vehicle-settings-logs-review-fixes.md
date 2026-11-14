# T7/T8 R1/R2 复审修复

任务 `task_fa7aeafb6f70 / ctx_5856e5458654`，工作目录仍为
`/Users/zhangshunjun/workspace/orca/senior-robot-control/vehicle-settings-logs-wire`，
分支 `Shunjun/vehicle-settings-logs-wire`，HEAD 仍为
`dae9f56884fbc9f3ea59c7e8e45a6fee6e71b824`。
本轮增量基于上一批未提交待审图像；该15文件图像先归档到
`/private/tmp/task_fa7aeafb6f70-baseline/`，逐文件比对确认其中仅 relay.py 被本轮修改。
原脏树只读，未改 index/提交/推送，未启动服务、ROS、Gazebo 或 VM。

## R1：只登记已入队请求，丢弃按原控制者失败结算

先用实际 Hub/Outbox 复现：protected 消息填满队列后，settings.get 的 put 返回 false，
原控制者已收到失败却仍留 pending；另两例证明 protected 挤出和关闭丢弃没有任何设置回执。
RED 日志：`/private/tmp/task_fa7aeafb6f70-red.txt`。

现在仅在 outbox.put 成功后登记 settings_requests，因此拒绝入队不占8个 pending 名额。
真实车端回执与排队丢弃共用 `_route_settings_reply`，以云分配 controller_generation 和原
request/session/seq/type 匹配；先清记录再向唯一 owner 交付，交付失败沿原 overload 关闭。
通用 discard callback 对尚未发送的 settings 返回 ok=false/conflict，不宣称车端执行。
已从 outbox 取走的请求仍等待真实车端回执或结果查询，不被此逻辑猜测失败。
原 drive.intent 合并成功结算分支保持原样。

测试连续10次实际拒绝入队后仍可正常读；两个控制者故意复用完全相同的浏览器标识，
挤出只能结算原 owner，关闭则分别结算两者。原手动合并/丢弃回归保留。

## R2：在 log store 打开边界归一化预期的 SQLite 打开失败

修正测试 IPC 路径长度后，用真实 SQLite 目录路径复现同一个 OperationalError：
`/private/tmp/task_fa7aeafb6f70-r2-red.txt`，堆栈从真实 composition 到 aiosqlite.connect。
失败确实先于 Agent 装配，且 debug restore timer 已启动而未进入正常清理。

本轮只在 VehicleLogStore.__aenter__ 的 connect 处捕获 sqlite3.OperationalError，转换为
该 store 已有 unavailable ValueError 契约；composition 既有的可选存储处理即可继续运行。
失败的 connect 没有可用连接，当前 aiosqlite._connect 在异常路径停止自己的 worker；
已成功取得连接后的 rollback/close 仍由原 store 边界负责。
没有扩大为 catch Exception/BaseException，也不把取消、RuntimeError、ProgrammingError 归一化。
本轮不增加日志 service、数据库或恢复 timer owner。

新装配测试保留真实 run_agent、Agent.run、supervisor、SettingsService 与 SQLite，
只替换外部 Cloud socket 与 ROS IPC peer；目录异常后 Agent 仍实际连接并产生 snapshot，
有 settings.v1、没有 logs.v1，仍能获控并发送心跳，shutdown 后设置 timer 已关闭且连接退出，
原目录保持不变。没有运行或伪称真实 ROS/车辆运动。

## task_loop/gate 的正常延迟边界

静态路径为 composition.task_loop 在 `gate.lock` 内 await 整个 TaskRuntime.tick，随后在锁外
sleep(0.2)。Agent.dispatch 的 acquire/导航/任务准入，以及 `_drain_intent` 的手动意图也用同一锁。
因此普通手动请求确实可能等待当前 tick 剩余时间与排在前面的持锁者；200ms 是 tick 间隔中的
锁外 sleep，**不是**最长锁占用或手动响应上界。

无执行任务的正常 idle tick 主要是 SQLite 查询/调度扫描；运行任务还会等待状态轮询和 effect。
NavigationEffects 的单次 resolve/execute/query/progress 默认超时为0.3秒，但一个 tick 可以包含
多次这样的操作及持久读写；LifecycleService.reconcile 明确只限制操作/确认次数，不限制总历时。
SQLite 的排队、事务/磁盘完成及 IpcClient 取得自身串行锁的等待也不构成整个 tick 的固定上界。
所以不能从单端口0.3秒、导航 client 3秒或数据库 busy timeout 1秒推导总 tick 延迟。
acquire 的5秒、intent 的0.5秒 IPC wait_for 位于取得 gate 之后，也不覆盖 gate 排队时间。

意图仍携带原 received/deadline，取得锁后若原0.3秒窗口已过就拒绝，不延长旧样本。
该共锁会影响手动输入的正常延迟，端口接近0.3秒等待时尤其可能耗尽样本窗口；本轮没有证据
证明 Linux 正常负载的最长延迟，也没有擅自缩锁、拆任务状态机或增加运动资格条件。

**心跳不直接取得 gate**：Agent._session 在接到 HeartbeatFrame 后调用
_motion.server_heartbeat；composition 给它独立 heartbeat_client，和任务 navigation_client、
普通 manual client 分开。_beat 发送 cloud heartbeat 也不取得 gate，急停/释放仍沿原独立链路。
新增装配测试持有实际 gate，让 acquire 在等待时，接收队列后继心跳仍经实际 Agent._session
到达替身 ROS 边界；解锁后 acquire 正常完成。这证明没有 gate 的直接互斥依赖，
不能证明共享事件循环 CPU、IPC/ROS 调度、磁盘压力下的实时预算。

## 验证与交付

新增 `tests/integration/test_settings_logs_review.py` 共4项（含 protected/close 参数例）通过；
与原 manual_wire 定向合跑22项通过。

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python -m pytest tests/integration/test_settings_logs_review.py tests/integration/test_settings_logs_wire.py tests/integration/test_manual_wire.py tests/integration/test_agent.py tests/integration/test_cloud.py tests/integration/test_vehicle_settings.py tests/integration/test_vehicle_logs.py tests/integration/test_agent_snapshots.py tests/integration/test_task_lifecycle_store.py tests/unit/test_protocol.py tests/unit/test_agent_config.py -q -p no:cacheprovider
```

最终结果277 passed in 17.80s；两生产文件与新增测试的 Ruff check/format、strict Mypy 均通过。
全仓 pytest 含 ROS/环境相关测试，本轮按请求仅验证正常主路径与上述必要边界，没有扩专项实测。
原前端未改，本轮未重复其上一批25项测试。

仅4文件增量：cloud/relay.py、vehicle_logs/store.py、本新增测试及本文。
准确增量 patch：`/private/tmp/task_fa7aeafb6f70-r1-r2.patch`。
本轮 baseline/current SHA 与完整路径清单：`/private/tmp/task_fa7aeafb6f70-files.sha256.json`。
上一批待审 patch/SHA 文件未覆盖；仍待负责人复审后再限定提交，不自动合并。
