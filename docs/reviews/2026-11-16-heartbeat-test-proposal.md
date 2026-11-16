# 第10项：心跳链路只读盘点与测试候选

基线 HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`，基于当前含其他工作未提交改动的工作树阅读；未修改产品、配置或测试，未运行测试、服务、ROS 或车辆。依据根方案第10项、新版运动设计及 `heartbeat-online-only.md` 最新要求。

## 现状及涉及位置

- `src/senior_control/agent/websocket.py:45` 默认回执超时1秒、freshness 2秒；同一 `_timeout` 也用于所有发送的 wait_for。`send_heartbeat` 仅保留一个 pending，`recv` 以单调时间、nonce、robot、epoch、sent_at 校验回复；成功返回以发送时刻加 freshness 的 deadline。`WebSocketConnector` 使用这些默认值。窗口修改需同时审视发送超时与回执计时的不同用途，保留身份校验。
- `src/senior_control/agent/main.py:135,234-281,398-471` 默认 ledger 窗口2秒；过期经 `_expire_eligibility` → `_lose_eligibility` 撤销 gate、递增 continuity generation、标记 revoked link，`_beat` 随后 `motion.disable` 并关闭连接；回执发送失败也触发此路径。`_session` 实际关闭时同样执行撤销及 disable，必须保留真实连接关闭的身份隔离。`_admit`（1022行）直接检查 link、revoked link、epoch、命令有效期，并不直接调用 `eligible_for_motion`，但 revoked link 使它间接受影响；不能仅修改该 property 就认为解耦完成。`connection.py:49` 的 HeartbeatLedger 和相关 deadline/调用须按实际用途整理。
- `runtime/supervisor.py:114` 心跳周期由 Agent 默认1秒覆盖 Supervisor 独立默认0.2秒；`wait_for` 上限为周期5倍，属于发送调用超时，需随实际接线一并核对，不能把发送周期误作断线窗口。`composition.py` 正式构造未覆盖 Agent 心跳参数。
- `cloud/relay.py:56,213,273,375,482,649,865` 云端窗口2秒用于在线判定、未首个心跳时从连接开始计时的 detach、重连宽限起点及每0.2秒 watcher；控制转发前也调用 stale 检查，重连新连接首次心跳前对 authority/intent 有专门离线限制。须核对在线准入和真实连接代次隔离，避免把窗口当作独立停车控制。当前云端 `<=` 为在线、`>` 才过期，Agent 回执 `elapsed < timeout`、定时器到点触发；精确边界应统一明确并测试。
- `config.py:129,165` 的 `AgentSettings.heartbeat_seconds` 默认0.2，`load_settings` 在生产代码未见消费者；`hardware-safety.yaml:63` 心跳字段已被 `real_runtime.py:37` 列为退役参数。它们并非当前正式 Agent 的5秒窗口，不应误认已接线。`docs/protocol/http-ws.md:47,57` 仍描述2秒和1秒，后续相关交付需同步协议描述。
- 现有实际代码与获批要求的冲突由上述静态调用链可直接证明；本次未运行复现，不将其称为已实测复现的独立 bug，也无已修复 bug 定向回归。

## 候选测试（供负责人筛选）

1. **云端正常心跳及5秒边界**。依据双端5秒在线窗口；输入有效当前代次心跳后，在5秒前、恰到5秒和5秒后查询在线及处理到期；预期前段在线且不提前断开、到期后离线并断开，精确等号采用双方一致的约定。复用并实质改写 `tests/unit/test_heartbeat_wire.py::test_missing_heartbeat_expires_connection_even_if_socket_is_open`，同时利用 `test_authenticated_heartbeat_echoes_epoch_and_nonce` 的真实报文。明确边界值当前双方实现不一致。
2. **Agent 延迟回执**。依据不应被1秒旧门提前断开，输入当前连接合法回执，延迟超过1秒但少于5秒；预期接受、连接保持，下一次心跳正常；保留匹配 nonce/robot/epoch/sent_at 校验。复用并实质改写 `tests/unit/test_websocket_link.py::test_reply_must_match_outstanding_nonce_and_epoch` 或 `test_unqualified_command_clock_keeps_link_alive`。
3. **Agent 无回执5秒到期**。依据双端断线窗口，输入发送一次心跳后无回复、接收空闲；预期5秒前不以心跳超时关闭，达到约定到期边界才进入离线/重连，不产生额外心跳停车 owner。复用并实质改写 `tests/unit/test_websocket_link.py::test_missing_reply_times_out_even_if_receive_is_idle`，必要时结合 `tests/integration/test_agent.py::test_failed_heartbeat_closes_session_and_forces_reconnect` 检查完整生命周期。
4. **无回执但手动输入持续新鲜**。依据心跳不作为运动资格或停车依据；在连接仍开放且不到5秒窗口时延迟心跳回执、持续输入合法新鲜手动 intent，覆盖旧1秒/2秒门；预期已授权手动链仍交付当前输入，不能仅因缺回执撤销 authority、disable 或停车。复用 `tests/integration/test_manual_wire.py::test_real_authenticated_relay_agent_ipc_and_native_owner` 的 relay→Agent→IPC→native owner 夹具及 `tests/integration/test_agent.py::test_heartbeat_send_success_does_not_restore_eligibility`，需新增或实质改写链路场景；原测试旧运动资格断言需迁移。
5. **手动实际断流本地停车**。依据设计第122行及用户要求；输入已激活手动 intent 后停止输入，其他条件维持正常；预期原本地 TTL 到期关闭 authority、产生本地停车，迟到旧 intent 不重开。复用 `tests/integration/test_manual_wire.py::test_expiry_and_controller_retirement_do_not_reopen`；已有覆盖原则上作为不改动回归执行，若需要证明完整链路再实质扩展，不重复新增。
6. **已接受自主任务在失联后继续与重连同步**。依据设计第123、138—140行；输入已接受并运行的任务，随后真实连接关闭或心跳到期离线，任务在车端继续到进度/终态，再重连查询；预期身份一致、不重复提交或执行。复用 `tests/integration/test_task_ownership.py::test_runtime_runs_offline_and_completes_same_identity` 与 `test_runtime_queries_unknown_plan_without_second_submission`，并按现有链路覆盖情况新增或实质改写连接联动；单纯 runtime 的 connected=False 不能证明真实连接重连链路。
7. **真实关闭和重连旧手动意图不复活**。依据连接代次撤销和设计第122、152行；输入活动手动、实际 socket 关闭，重连后发旧 epoch/旧 authority 的迟到意图，再显式新授权并给出新输入；预期旧上下文撤销、旧意图不执行，新操作可按正常条件执行。复用 `tests/integration/test_agent.py::test_dropped_link_disables_its_epoch`、`test_reconnect_rejects_old_command_but_returns_committed_result` 与 `test_real_authenticated_relay_agent_ipc_and_native_owner`，需新增或实质改写以覆盖完整链路。
8. **云端重连在线准入与旧代次隔离**。依据离线不接受新远程任务和重连身份隔离；输入真实旧连接关闭、新连接建立，新连接首个合法心跳前后分别检查在线、控制/新远程请求；再送旧代次迟到帧。预期离线期间不伪称已接受，当前代次合法上线后正常处理，旧连接不能恢复在线或控制。复用 `tests/unit/test_heartbeat_wire.py::test_reconnect_grace_ends_without_a_fresh_epoch_heartbeat`、`tests/integration/test_cloud.py::test_two_sockets_old_epoch_dropped`；需实质改写或补充实际准入链路，注意现有 relay 对新连接首个心跳前的限制只显式覆盖 authority/intent，应据此核实其他新请求的现有行为。

现有 `test_heartbeat_eligibility.py`、`test_agent.py` 中以心跳刷新 ROS 资格为前提的断言与新要求冲突，按修改后真实职责迁移或删除废弃断言；保留无关安全、身份和执行隔离有效保护。身份校验、慢操作不阻塞心跳、重连宽限等仍有效现有回归可复用，不为重复覆盖另设用例。
