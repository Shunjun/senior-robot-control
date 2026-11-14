# 短锁复审：手动 handoff 的 owner/revision 重核

`task_6cddda85f7a7 / ctx_2bcd32b91ae4`，独立分支和 HEAD 仍为
`Shunjun/vehicle-settings-logs-wire / c19ee761a5c349239ed12bf210c8eea7c3540423`。
本轮在上一批8文件未提交图像上增量修复；基线保存于
`/private/tmp/task_6cddda85f7a7-baseline/`，其中仅 ports.py、executor.py 和新测试被改动。
未改原脏树、index、ROS、任务 reducer，未提交推送或启动服务。

## 实际 RED 与最小修复

新增 takeover-submit、takeover-resume 两例运行原完整 Agent/composition、真实 TaskExecutor、
LifecycleService/Store、SQLite 和任务端口 adapter，只控制 Cloud/ROS IPC peer。
在真实 control_current 完成返回之后，测试边界暂缓把该返回交还 TaskAwareMotion；此时分别
插入真实 submit/plan 与同一 execution 的 resume/plan，确认其已经持久到 running 后再放行 acquire。
两例都复现 **acquire 回执成功且 manual 端口调用时持久任务仍为 running**；没有把下游拒绝当正确接管。
RED：`/private/tmp/task_6cddda85f7a7-red.txt`（2 failed、7 deselected）。

修复只涉及两个生产文件：

- `TaskExecutor.control_current` 返回实际检查/退役过的 `(execution_id, lifecycle.version)`，
  无任务返回 None；该值只是一次权威读的观察，不保存新 owner、不构成另一份 reservation。
- TaskAwareMotion 在同一 runtime.admission 锁内调用 `manual_handoff_matches`，用当前 SQLite
  owner/revision、paused/takeover/pending/effect 条件和已有 coarse active projection 重核。
  只有相符且无执行来源义务才读取 settings 快照并发 native manual IPC；task mutation 也用此 gate，
  因而检查与物理 acquire ACK 之间不能插入新的任务提交。
- 不相符时先退出 gate，再用原 TaskExecutor.control_current/真实 effect reconciliation 重新退役。
  沿既有5次尝试上限，无法证明可交接则返回 unavailable，绝不假报获控。
- takeover 内部幂等键由外层 acquire request、目标 execution 与观察到的 task epoch 确定，仍走原
  agent_operation/lifecycle receipt。这样新 owner 或 resume 的新 epoch 不会误重放旧 takeover 回执。
  没有改变原 task.execution.command 请求键、状态算法、来源退出确认或持久数据库 owner。
- 新的 native handoff 查询在 gate 内只读现有数据库，不调用 service.start 或端口、不发 effect。
  legacy forward 入口保持原兼容范围并标注 //Compatible；生产 native Agent.acquire 经 manual()。

GREEN 两例证明端口真正收到 acquire 前，刚插入/恢复的任务已为 `paused/takeover` 且 pending=None。
原7例未删，包含相反顺序、20Hz持续意图、取样失败/过期、释放 fence 和提交互斥。
取样/authority/实际 motion 均由 IPC peer 控制，以上是持久接管与调用顺序证据，不是车辆运动验收。

## 锁顺序与仍存在的长等待

```text
锁外 control_current → service.start（如需）→ 单次短任务提交 gate → DB → SQLite
  → 锁外 task port/effect/retirement 等待 → 返回 owner/revision 观察
native handoff → gate → 只读 owner/revision/active projection（DB锁各自释放）
  → 无锁 published_snapshot 读取 → manual IPC → 释放 gate
owner/revision 不匹配 → 释放 gate → 锁外重做 control_current
```

本轮不增加 gate→service.start 的反向获取，不把 retirement 端口放回 gate。
**SettingsService 驾驶 apply 仍是 `_lock → gate → await runtime.refresh()`**，其中包括真实 IPC
state.read 与持久任务读取，随后 gate 内 await SQLite transaction/commit；这一段没有移出。
因此不能说所有外部等待已经锁外，也不能从快照属性读取的微秒级时间推导驾驶设置提交持锁上界。
手动物理 IPC、owner/revision 数据库重核、SQLite 排队/磁盘完成也仍会占用 handoff gate。
上一批 task-port 等待移出只解决该项耦合，不代表所有输入延迟或实时预算已解决。
原 intent received/deadline 检查与 steady 域保持，独立心跳通过仍不是连续驾驶证明。

## `_atomic` 提交前检查的准确覆盖

所有生命周期/effect mutation 仍经过统一的短 gate；纯读和 schema prepare 不取 gate。
提交前的额外 sample.motion_allowed 检查明确只针对返回的 Transition **新产生 plan effect**，
不能解释为每个写入、每个 claim 或每个 ACK 都重新授予/复核了物理运动权限。

| 路径 | 提交前 plan 检查与实际职责 |
| --- | --- |
| submit、resume command | 产生 plan，原 reducer 事务内使用请求局部 context；COMMIT 前再核对 sample，原 claim/expected_version 校验保留 |
| feedback-recovered/action-completed 等 event | 仅当实际产生新 plan 时做相同检查；停止/撤销/进度写入不因缺健康而被拦截 |
| revoke ACK 导致 feedback recovery 重新 plan | Transition 带新 plan，因此被覆盖；原 epoch/effect/status/pending 和 expected_version 仍在事务内重核 |
| plan ACK → running | plan 已在端口执行/接受，ACK 只结算原承诺；没有新 plan effect，因此不走该额外 predicate，仍核对版本/epoch/未 retired 的 sending/pending 身份 |
| manual_ready ACK | 仅消费通知，不授予手动 authority；本轮 native handoff 的锁内 owner/revision 重核才允许发送真正 acquire |
| claim_effect：ready→sending | 返回 StoredEffect，额外 plan predicate 不适用；只 claim 当前 ready 行，不做物理执行；之后才在锁外执行原稳定 effect |
| recover/bind_existing | 原 restart reducer 只撤销/迁移和 fenced source retirement，不生成新 plan；仍在 gate/DB 内提交，不因外部失效而跳过撤销 |

`LifecycleService.reconcile` 是 claim 之后的锁外端口执行 owner，`_confirm` 经当前 store.load/version
和 acknowledge 处理迟到结果；retired/旧 epoch ACK 不可恢复为 running。
`NavigationEffects`/`IpcNavigation` 传原 effect_id/task_epoch 和已接受的地图/pose/route；RPC 接受不是
来源退出。撤销结果必须满足 sources_fenced_through_epoch 与 controller_output_exited。

实际物理侧现有复核由 `navigation_adapter.RouteService.start/_run.qualified/cancel` 与
`CommandNode.task_motion_allowed/bind_navigation_source/_on_navigation_sample`、原 T5 source fence
和当前本地资格承担：start 检查撤销 epoch fence，cancel 推进 fence，路线/采样持续复核 map、
source binding、当前 motion runtime/手动来源。它们不是本轮新增的替代持久接管屏障；本轮已在
Agent 内消除错误 handoff 顺序，不靠物理侧最终拒绝来伪称持久任务暂停成功。
这些 ROS 路径本轮只读核对，未运行，因此不声称真实物理执行证明。

## 验证与交付

定向组合154 passed in 7.08s，保留全部既有7例和新增2例。
最终相关回归569 passed、3 deselected in 21.24s，记录
`/private/tmp/task_6cddda85f7a7-regression.txt`；6个改动生产文件的 strict Mypy、7个Python文件
Ruff check/format 均通过，CRLF-aware whitespace 检查通过，executor 原 CRLF 保持。
未扩大到 ROS/实车、洪峰或断网专项。

排除项继续单列：沙箱 loopback bind 例1项；
`test_gate_retires_generation_only_after_brake_owner_handoff[True]`、`[False]` 2项，之前已经由纯
c19ee76 archive 在基线目录复现相同 IndexError（2 failed、9 passed），证据
`/private/tmp/task_b7719c254510-baseline-failures.txt`。本轮没有将这两项说成通过，也未改相关 ROS 文件。

本轮4文件增量：ports.py、executor.py、test_short_admission.py 和本文，基于上一批待审8文件图像。
`/private/tmp/task_6cddda85f7a7-handoff-fix.patch` 与逐文件基线/当前 SHA 清单
`/private/tmp/task_6cddda85f7a7-files.sha256.json` 供复审；此前 patch/manifest 均未覆盖。
另给相对 c19ee76 的累计9文件 patch：`/private/tmp/task_6cddda85f7a7-short-lock-combined.patch`。
仍保持未提交，等待负责人亲审；SettingsService.refresh 与物理 IPC/数据库剩余持锁预算继续如实待验。
