# RuntimeGate 短提交边界修复（待亲审）

任务 `task_b7719c254510 / ctx_1624ef794cab`，基线保持
`c19ee761a5c349239ed12bf210c8eea7c3540423`，独立工作树与分支未变。
本轮没有提交、推送、操作 index、修改原脏树或 ROS 源码，没有启动服务/ROS/Gazebo。
负责人已批准扩大 tasks/lifecycle_store.py、tasks/executor.py 的必要锁接线范围。

## 复现与实际接口

`tests/integration/test_short_admission.py` 运行真实 run_agent、Agent.run、TaskRuntime、
LifecycleService/Store、NavigationEffects、IpcNavigation、TaskAwareMotion、SQLite；
Cloud socket 和 ROS IPC peer 是明确可控替身，没有抽取函数体执行，也没有另写任务 reducer。
任务 effect 回执使用实际 lifecycle_version/effect_id/task_epoch/source retirement 字段。

原版本在后台 tick 和显式 task.execution.command 两条路径上，将实际 task_cancel 与后续
status/progress 的各0.18秒等待串入全局锁；20Hz送来的原始意图出现 TIMEOUT，部分仅被合并结算，
不能以这些成功回执冒充已到达 manual 端口。RED 保存于
`/private/tmp/task_b7719c254510-red.txt`。
测试源的 motion/authority 由 IPC peer 控制，检验的是 Agent 排队、持久任务和端口调度；
它不是实际 T2/T5 对并发来源资格的完整车辆验收，不能拿它替代真实输出/连续运动证据。

## 最小职责调整

- composition 不再持 gate await TaskRuntime.tick；Agent.dispatch 不再用外层 gate 包住整个
  navigation/task 命令。路线解析、effect 执行、状态轮询和 takeover retirement 等端口等待留在锁外。
- LifecycleStore 的唯一 `_atomic` 边界接受可选 composition admission guard；所有生命周期/效果
  写入默认经过它，包括 submit/command/event/acknowledge/recover/bind_existing/claim_effect。
  纯读取与 schema prepare 明确不取 gate，避免 settings observer 在 gate 内读任务时重入。
  reducer、任务身份、来源退出和持久 owner 均未改；未配置 guard 的独立旧调用标注 //Compatible。
- task_commit 在 gate 外取新 IPC 样本；进入 gate 后比对同一个 RuntimeGate generation，变化则重新
  取样。请求局部 provider 用 ContextVar 传到该事务，复用 TaskExecutor.current_context 的原资格逻辑，
  不受其他协程覆写 executor.context 影响。样本失效/取样失败使用保守上下文，不沿缓存启动。
- generation 是同步 fence，**不是第二份任务状态**。手动物理准入、驾驶配置提交、任务写入和原
  RuntimeGate.transition/update 参与同一版本规则；安全/释放/复位/连接 retirement 非阻塞地
  invalidates，继续走原独立链路。Agent 即使未选 settings profile，也绑定同一个 task gate。
- gate 内重新核对后，原 SQLite claim 校验完整 task definition/revision/map link/在途执行，原
  lifecycle `_require`/ack 校验 expected_version、epoch、effect 状态。工作结束、实际 COMMIT 前，
  对产生 plan 的 Transition 再核验请求局部 generation/证据有效性，变化或过期即回滚；不留下半个
  execution/receipt。物理任务端口继续沿原 epoch/fence 和 T5 最新本地资格复核。
- 手动锁移至 TaskAwareMotion.manual 的实际 handoff；takeover 的任务 retirement 在此之前完成。
  锁内仍读取唯一 published_snapshot 并 await 实际 manual IPC。Agent ingress 的同一个 steady clock
  绑定到该端口，锁后再次检查原 deadline，不延期。急停/释放不等待此锁。

## 锁顺序

```text
任务 service.start_lock（仅初始化） → 锁外 IPC 采样 → gate → Database.write_lock → SQLite事务
设置 SettingsService._lock → gate → Database.write_lock → SQLite事务 → 发布快照
手动 takeover task reconciliation（锁外，内部短事务沿上述顺序）
  → gate → 无锁 published_snapshot 读取 → manual IPC（不取数据库/SettingsService._lock）
只读 lifecycle 查询 → Database.write_lock → SQLite事务（不再反向等待 gate）
```

异步任务事务沿原 shield/cancellation 规则，完成实际提交或回滚才释放 gate/DB 锁。
服务 start_lock 不在物理 manual gate 内获取；端口 effect 执行也不在数据库或全局 gate 内等待。
没有把大锁替换成 DB→gate 或 gate→service.start_lock 的反向获取。

## 定向验证

新增7项通过：

1. 后台真实 task tick 的两次受控端口等待期间，5条20Hz意图都实际到达 manual port，均在各自原截止前。
2. 显式 task.execution.command 的同类等待期间，同样5条都到达，不能靠合并回执凑成功数。
3. 任务短事务受控停在 publication 处时，驾驶写入和手动接管不能穿透；事务完成后配置按真实执行
   owner 拒绝，接管沿实际 retirement 正常完成。
4. 锁外旧采样与手动获控交错，generation 改变后重新取样，prepared/active owner 使新任务拒绝。
5. 事务期间原证据截止耗尽，COMMIT 前拒绝并回滚 execution 与 receipt。
6. 取样失败时即使 executor 原缓存 ready，也不能沿旧值提交新运动。
7. 同事务期间收到释放，非阻塞 fence 变化阻止旧 plan 提交，而释放本身不等待 gate。

`7 passed in 1.79s`。最终相关大回归 `567 passed, 3 deselected in 21.37s`，命令记录在
`/private/tmp/task_b7719c254510-final-regression.txt`；源6文件 strict Mypy 通过，7个改动Python文件
Ruff check/format 通过，CRLF-aware diff whitespace 检查通过，原 executor CRLF 未整体转换。
最后仅将相同的 generation 加一收敛为 invalidate 方法并更新注释，随后定向7项再次通过。
前端未改，未重复运行 web 测试。

第一次较大回归实际为565 passed、2 failed、1 deselected；失败为
`tests/unit/test_navigation_command_clock.py::test_gate_retires_generation_only_after_brake_owner_handoff[True/False]`，
IndexError，保存于 `/private/tmp/task_b7719c254510-regression.txt`。
在由纯 c19ee76 git archive 重建的临时基线中，从基线目录运行该文件仍为2 failed、9 passed，
同名同堆栈，见 `/private/tmp/task_b7719c254510-baseline-failures.txt`，本轮未修改其 ROS callback。
最终排除这两例及既有沙箱 loopback bind 例；没有把已知失败隐藏为全仓全绿，也不扩入 T5 修复。

## 剩余持锁时间与限制

测量 JSON：`/private/tmp/task_b7719c254510-tick-timing.json`、`...-explicit-timing.json`。
以下数值由最终定向测试实测填入；实际 manual IPC peer 的10ms等待是**注入边界延迟**，
不是 ROS/车辆实测。快照仍是生产 SettingsService 属性读取，端口前后为生产 handoff。

| 路径 | manual gate 最大持有 | 快照属性最大读取 | 到达IPC时最少原剩余期限 |
| --- | ---: | ---: | ---: |
| tick（5条） | 16.420ms | 7.875µs | 299.009ms |
| explicit（5条） | 13.336ms | 9.959µs | 299.383ms |

任务长 I/O 不再把这些意图排到原300ms截止后，不等于整个实时预算解决。
实际物理 manual IPC/编码仍占 gate；原端口超时与 SQLite 排队/磁盘完成可能更久，普通控制输入
仍可能被这些剩余段影响。新鲜采样持续被新的本地 admission generation 抢先时，任务会重采样，
不拿旧许可强行提交；没有保证所有任务调度历时上界。
T5 证据窗口、source fence 和最终物理资格仍沿原 owner；本轮不调整 TTL、不扩大许可、不把独立
心跳进展当输入连续性证明，也不能解释未运行该 Agent 的旧 r6 停顿根因。
后续由负责人亲审后同版合版，交运动诊断 owner 做真实正常输入/输出连续性及剩余持锁预算验证。

本轮交付：`/private/tmp/task_b7719c254510-short-lock.patch` 与
`/private/tmp/task_b7719c254510-files.sha256.json`（精确路径、HEAD/current SHA、patch SHA）。
保持未提交等待亲审。
