# 运行状态报告与动作许可解耦

基线：`fix/degraded-drive-speed` / `800d29f`（前序 `5d45dd9`）。本地实施，不推送，待负责人亲审更新 Draft PR 25。

## 实际变更

- ROS `motion_core_output` 尾部已在每周期发布模式、故障及急停，不需修改发布器，也未修改 150 ms 窗口。
- `MotionRuntimeReceiver.observation()` 从既有可信接收前沿返回 current/missing/expired/revoked 与最后原始采样时间；只读，既有 current/accept/invalidate 及来源隔离不变。
- CommandNode → agent_adapter → IpcState 增加上述元数据；motion_runtime 继续仅承载当前动作证据。
- SnapshotState 每次生成 vehicle_ui；IPC 不可用也报告 ipc_unavailable，保留最后观测时间。未知时 mode/急停 unknown、速度 null、无运动动作；复位 token 必须对应当前许可。当前 runtime 的运动证据过期时，报告继续但不暴露运动动作。
- 云端沿既有 snapshot 转发，无额外缓存/权限体系；网页分开呈现上报在线与状态可观测性，并显示最后观测年龄。拒绝旧帧不再恢复重连后的 available。
- 协议及上线顺序记录在 `docs/protocol/http-ws.md`。

## 定向验证

版本：Python 3.12.15；pytest 8.4.2；ruff 0.17.3；mypy 2.4.0；Node v24.14.1；TypeScript 5.9.3；Vitest 5.0.5。

以下命令均在本工作树执行；npm 命令工作目录为 web：

```sh
.venv/bin/python -m pytest tests/integration/test_manual_wire.py tests/unit/test_runtime_guard_ready.py -q
# 27 passed：既有手动链路、来源隔离、撤销、复位。
.venv/bin/python -m pytest tests/integration/test_runtime_reporting.py tests/integration/test_settings_logs_wire.py -q
# 11 passed（当时 reporting 含 2 项）：真实 socketpair IPC + ASGI 认证云转发，及设置/日志。
.venv/bin/python -m pytest tests/integration/test_runtime_reporting.py::test_report_heartbeat_does_not_extend_motion_evidence -q
# 1 passed：之后新增的独立证据过期用例。
npm test -- --run src/api/vehicleState.test.ts src/features/devices/vehicleState.test.tsx src/features/drive/drive.test.tsx
# 新增测试前 12 passed。
npm test -- --run src/api/vehicleState.test.ts src/features/devices/vehicleState.test.tsx
# 新增测试后 7 passed：unknown 解码/呈现、重连、旧帧拒绝、正常新状态恢复。
npm run typecheck
# passed。
.venv/bin/ruff check src/senior_control/agent/composition.py src/senior_control/agent/manual_control.py src/senior_control/agent/ports.py src/senior_control/protocol.py tests/integration/test_runtime_reporting.py
.venv/bin/mypy src/senior_control/agent/manual_control.py src/senior_control/agent/composition.py --follow-imports=silent
# 均通过。
git -c core.whitespace=cr-at-eol diff --check
# 通过；保留原 ROS 文件 CRLF，不做整文件换行改写。
```

新链路用例覆盖 missing → current → expired（重复心跳）→ revoked → 新可信样本恢复 → estop → execution_fault，逐帧验证云转发内容；拒绝旧样本并确认原 deadline 不变。另验证 IPC 失败持续报告。重连验证为前端 transport 层测试，未运行真实网络/浏览器 E2E。

## 限制与隔离

未启动 ROS、开发服务、车辆、VM，未跑全套，未改 DNS/现场配置。没有证明现场 DDS 能收到状态；即使没有收到，也会明确报告 missing，最后观测时间指可信接受样本而非任意拒绝包。历史速度不作为新实时状态填入 unknown 帧。

网页须先于新 Agent 上线：旧严格解码器不认识 observation；新版网页接受旧帧，兼容分支已标注。未修改或提交用户叫停诊断 `native_receipt.py`、`tests/test_native_receipt_diagnostics.py`。学习生命周期由另一个 worker 负责，本提交不修改 `motion_core_output.py` 或学习文件。
