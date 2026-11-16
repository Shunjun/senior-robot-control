# 第10项：心跳仅在线判断、统一5秒实施报告

本轮产品和相关测试修改完成，本地集中验证 **284 passed，16.13s**，待负责人亲审实际增量 diff 后沿原 Draft PR26 集成交付。分支仍为 `fix/degraded-drive-speed`，HEAD 仍为 `593e8f13f385892f0bd62cf75c593f7b07efd2da`；没有暂存、提交、推送、部署，没有启动开发服务、ROS、仿真或车辆。测试使用局部 ASGI/Hub、回环 WebSocket、Unix socket/socketpair 夹具，不属于整栈或公网验收。

## 启动快照与最终代码指纹

- 启动文件副本、逐文件 SHA256、工作树状态和 HEAD：`/tmp/heartbeat-round-20261116-start/`，清单 `sha256.json`。保留已有大量脏改动。`protocol.py` 启动时为干净文件，其初始副本从该 HEAD 补取，未用当前修改后的文件冒充原始副本；根方案修改前副本为同目录 `root-plan.md`。
- 本轮逐文件初始/最终 SHA、三保护文件核对和产品清单：[fingerprints](2026-11-16-heartbeat-fingerprints.json)。产品六文件的排序 `path SHA256\n` 清单聚合 SHA256：`6206ffe94a46bba373533cd886b52949c1785910816feab37737f34881925075`。
- 相对**本轮启动内容**的实际增量：[owned.diff](2026-11-16-heartbeat-owned.diff)。它不包含 `test_manual_wire.py` 等文件在启动前已有的脏改动，也不以整个分支 diff 冒充本轮改动。
- 三保护文件原件 SHA 均与启动快照一致，暂存区为空；没有修改当前学习实现、时间证明/驱动或 ROS 输出规则。

## 产品职责与变化

`protocol.py` 集中实际消费者共享的在线窗口5秒和发送周期1秒；没有恢复未消费的 AgentSettings。Agent 在线期限由 WebSocketLink 唯一维护：从当前连接建立或最近有效回复开始，到5秒过期。nonce 对应的原请求、robot、epoch、sent_at、有限服务器时间及单调延迟校验保留。定时器与报文入口均执行到点过期；有效回复刷新在线期限，不把发送时刻或独立 freshness 再作为运动期限。

心跳请求按周期发送，不等待此前回复。仅保留5秒内可供身份匹配的在途 nonce；旧请求到期不代替当前有效回复直接断链。Socket 发送资源预算单独命名为 `SOCKET_SEND_TIMEOUT_SECONDS`（5秒），Supervisor 不再用发送周期乘5制造第二份发送期限。发送等待、在线证据期限和发送周期职责分开。

删除 HeartbeatLedger、ROS eligibility property、连续性 generation、失联取消任务回调及心跳触发的直接 disable 链，不留兼容空壳。真实 session 关闭仍按旧 epoch 撤销手动上下文、丢弃旧排队输入和取消该连接未完成的请求；实际手动断流停车仍归原车端 TTL/唯一输出 owner。已接受自主任务的车端 lifecycle 不因心跳或云连接失联被取消，重连同步原身份状态，不重放旧手动输入。

云端当前连接有效心跳的年龄 `<5` 才在线，`>=5` 过期。未收到首个心跳的连接也从建立时计5秒。watcher 睡到证据期限并在新证据后重新计算；报文入口先做过期检查，避免 watcher 尚未调度时迟到请求续活旧 epoch。重连宽限、既有远程身份/准入政策、手动 TTL、任务准入、停车 owner 均未另加规则。当前协议说明已同步；历史报告没有改写。

## 新改行为验证与复用回归

| 类别 | 实际验证 |
| --- | --- |
| 云端边界（改写原位置） | `test_missing_heartbeat_expires_connection_even_if_socket_is_open` 从真实有效当前报文出发，可控时钟4.999秒在线、5秒离线关闭、5.001秒仍离线；旧 epoch/重连宽限相关原回归保留 |
| 云端到期竞态（新增定向） | `test_heartbeat_at_deadline_cannot_revive_epoch_before_watcher` 验证到5/5.001秒时先处理真实报文仍不能续活；`test_watcher_sleeps_until_current_evidence_deadline` 覆盖有效新证据刷新后的期限 |
| Agent 延迟回复（改写/扩展） | 原 WebSocketLink 报文测试接受3秒及4.999秒延迟；robot/epoch/nonce/sent_at 不匹配仍拒绝；新 `test_delayed_replies_preserve_send_cadence_and_connection` 经真实 Hub 请求/回复连续接受3秒延迟，跨越首请求5秒期限且连接保持 |
| Agent 无回复（改写原位置） | `test_missing_reply_times_out_even_if_receive_is_idle` 使用可控时钟及真实定时器回调，4.999秒不提前断开，5秒唤醒空闲 recv；任务链 heartbeat 场景验证实际 Agent session 关闭后进入重连 |
| 手动持续输入及重连（新增完整软件链） | `test_heartbeat_window_keeps_manual_delivery_then_reconnect_retires_old_context` 先接受一次有效回复，再丢后续回复；真实 Hub→WebSocketLink→Agent→TaskAwareMotion→长度分帧IPC→AdapterCore→CommandNode/ManualIngress，每0.1秒新鲜输入跨过旧1/2秒直到4.999秒仍交付且不 disable；5秒真实关闭才撤销旧 epoch。重连后旧 epoch、旧 authority 无输出，新显式授权及输入可以执行 |
| 已接受自主任务（实质改写原位置，两种失联） | `test_runtime_runs_offline_and_completes_same_identity[socket/heartbeat]` 以真实 Hub transport 关闭/WebSocketLink 超时和 Agent session 关闭为离线依据；车端真实 SQLite/lifecycle/runtime 继续至成功；重连真实 snapshot 同步原执行/任务身份及成功进度；仅一次 plan，原 schedule receipt 不变。导航端使用原 Navigation 测试端口，不声称实车执行 |
| 复用原有效回归 | `test_expiry_and_controller_retirement_do_not_reopen` 原车端 TTL 断流；原 authenticated relay/IPC/native owner 链；旧命令/重复操作结果、真实断线 epoch、慢 ROS/地图/快照不阻塞心跳、任务所有权与未知计划查询、云双 socket 旧 epoch、急停交付及关停等。少量旧准备步骤改为等待心跳被接收，不再等退役资格接口 |

直接删除退役 ledger/资格刷新/失联取消任务测试；旧急停测试中仅重复显式 disable 的 `expired_heartbeat` 参数分支合并，保留已接收急停在 epoch 撤销期间交付的有效测试。Supervisor 原 wait_for 取消竞态测试改为发送回调吞取消的真实关停场景；没有为旧结构逐条制造兼容测试。

## 命令与结果

最终集中验证命令（本地监听夹具在授权的提升权限环境运行）：

```sh
.venv/bin/pytest tests/unit/test_heartbeat_wire.py tests/unit/test_websocket_link.py tests/unit/test_cloud_epoch.py tests/unit/test_manual_authority.py tests/integration/test_agent.py tests/integration/test_manual_wire.py tests/integration/test_task_ownership.py tests/integration/test_shutdown.py tests/integration/test_estop_recovery.py tests/integration/test_settings_logs_review.py tests/integration/test_task_runner.py tests/integration/test_runtime.py tests/integration/test_cloud.py -q --junitxml=/tmp/heartbeat-round-final-v2.xml
```

结果：**284 passed in 16.13s**，无跳过、失败或收集错误。原输出：[log](2026-11-16-heartbeat-evidence/heartbeat-round-final-v2.log)、[XML](2026-11-16-heartbeat-evidence/heartbeat-round-final-v2.xml)。对产品六文件、全部本轮存活 Python 测试及新支持文件执行 `ruff check` 通过，`ruff format --check` 为17文件均格式正确。`git -c core.whitespace=cr-at-eol diff --check -- <本轮文件>` 通过，使用该设置识别既有 CRLF，而非迁移旧文件换行。

首轮曾为199通过、23失败、3错误（`/tmp/heartbeat-round-first.log/.xml`）：其中本地 bind 受沙箱限制，另有测试残留退役接口、手动夹具未接 TaskAwareMotion/任务交接接口、额外 tick 的 terminal_settled 断言与 intent 无逐条响应的夹具错误。后续修正夹具/过时断言，未放宽产品条件；282通过后发现发送节拍耦合，再完成最终284项同版验证。中间日志 `/tmp/heartbeat-round-fix.log`、`/tmp/heartbeat-manual-fix.log`、`/tmp/heartbeat-manual-final.log`、`/tmp/heartbeat-round-final.log`、`/tmp/heartbeat-cadence.log` 保留。

对照验证将本轮启动的六个产品文件覆盖到隔离 `/tmp/heartbeat-round-before/src` 副本，测试运行当前定向用例，不回滚或覆盖工作树：

```sh
PYTHONPATH=/tmp/heartbeat-round-before/src .venv/bin/pytest tests/integration/test_agent.py::test_delayed_replies_preserve_send_cadence_and_connection tests/unit/test_heartbeat_wire.py::test_heartbeat_at_deadline_cannot_revive_epoch_before_watcher -q --junitxml=/tmp/heartbeat-before.xml
```

实际 **3 failed**：旧 Agent 在无回执的下一秒发送时已抛超时；旧云端在 watcher 清理前接受迟到报文并恢复在线。这是实际报文与代码路径复现，不是编造异常。修正版本全部通过；对照证据：[log](2026-11-16-heartbeat-evidence/heartbeat-before.log)、[XML](2026-11-16-heartbeat-evidence/heartbeat-before.xml)。

## 根方案与未验证范围

工作树外根方案 `/Users/zhangshunjun/Wheel 机器人/车辆启动约束审查与处理方案.md` 只更新第10项实际实施/验证状态，保留用户原文与其它项；外部文档增量另存本轮 evidence。负责人尚未亲审本轮实际 diff，PR26尚未集成交付。

未执行完整仓库测试/完整mypy/全量CI、Linux/Humble/aarch64、实际DDS/ROS、仿真、车辆、真实公网抖动/拥塞或硬件 watchdog 验收。软件测试中的可控时钟和导航测试端口只证明本轮职责与协议/lifecycle行为，不能推导调度实时性或物理运动保证。毫秒配置迁移及第11项主Agent故障隔离未混入本轮。
