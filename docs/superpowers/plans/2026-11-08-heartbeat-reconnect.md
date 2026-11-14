# Task 23 · 心跳、重连与远程运动验收计划

> **本轮改版优先级说明（2026-11-12）**：本文保留历史设计/计划。与[新版运动控制设计](../specs/2026-11-12-vehicle-motion-redesign.md)及用户最新要求冲突的内容不再作为本轮开发依据，尤其是非手动任务断网暂停、网页延迟资格与旧速度目标。未冲突部分仍供参考；新版设计已于 2026-11-12 获用户批准，后续修订按用户最新明确要求执行；批准不代表新功能已实现或验收通过。历史正文保持原样，当前阶段见[阶段索引](../../testing/2026-11-14-vehicle-stage-index.md)。


> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 Linux 仿真 Agent 在公网链路发生临时故障后自行重连，并保留 ROS 适配器和本地 socket。

**Architecture:** 分别跟踪 WebSocket 连接状态、设备展示状态和运动许可。连接失败时立即撤销旧代次的运动权限，展示状态允许重连宽限，恢复连接后重新取得控制租约。

**Tech Stack:** Python 3.10+、asyncio、WebSocket、ROS 2 Humble、Gazebo Fortress、pytest。

**Spec:** [剩余工作清单第 1 项](../../testing/2026-11-07-remaining-work.md)。

## 当前证据

Linux 仿真曾激活两个底盘控制器，Agent 通过本地身份检查并打开公网 WebSocket。心跳回复随后因时钟偏差或延迟超限被拒绝，Agent 退出 78，关停流程让适配器删除 socket。工作区已有一秒心跳、可重试错误和相关超时调整的代码草稿。相关测试曾通过 82 项，另有一项需要本地监听端口的测试未执行。草稿待审，Linux 和公网端到端未验收。

## 全局约束

- 心跳周期和回复超时各为 1 秒。连接故障至少尝试重连 5 次，后续重试使用有上限的退避。
- 断线立即撤销运动许可，旧代次、旧命令和旧租约失效。重连后需要重新取得控制权。
- 将在线探测与运动时序资格分离。500 ms RTT 下验证在线与状态上报，运动仍须满足命令有效期和停车约束。
- 显式凭据拒绝继续按认证失败处理。兼容性代码标注 `//Compatible`。仅在调试和验收时启动仿真或服务。

## 23A · 复审心跳与重连草稿

涉及 `src/senior_control/agent/websocket.py`、`agent/main.py`、`runtime/supervisor.py`、`tests/unit/test_websocket_link.py` 和 `tests/integration/test_runtime.py`。沿用 `WebSocketLink.recv()` 和 `Agent._session()`。

- [x] 补齐测试，覆盖回复丢失、时钟跳变、连续 5 次失败后成功重连、凭据撤销。断言临时错误不调用 ROS 适配器的 `exit()`。
- [x] 复审工作区草稿，将心跳回复期限、命令时钟有效性和连接退避分开验证。500 ms 回复可维持连接，时钟或命令有效期不足时撤销运动资格。
- [x] 使用不需要监听端口的单元和集成测试确认恢复时使用新代次，临时故障期间 ROS 适配器保留。
- [x] 在允许监听回环端口的环境补跑真实 WebSocket 握手与连接代次测试，并通过真实 Unix IPC 停车回归。

## 23B · 网页重连宽限与运动安全

涉及 `src/senior_control/cloud/relay.py`、`docs/protocol/http-ws.md`、`web/src/features/devices/DeviceList.tsx`、ROS `command_node.py` 和 `simulation_motion.yaml`。

- [x] 明确云端如何判断连续重连失败次数。云端无法直接看到未到达服务器的尝试，若采用时间宽限，先定义与重试退避的对应关系，并记录页面“重连中”转“离线”的条件。
- [x] 补齐断线期间禁用驾驶、宽限内恢复、持续失败显示离线、旧连接迟到消息和重连不自动续跑的回归测试。
- [x] 核对云端与 ROS 仿真的 2 秒心跳窗口及独立的 0.3 秒速度命令时限，并运行云端及 Arbiter 回归。
- [x] 网页回归通过 138 项，生产构建成功。

本地 Python 单元和集成回归通过 1139 项，跳过 4 项。心跳专项覆盖回复超时、模拟 500 ms RTT、时钟跳变、五次失败后新代次恢复、凭据撤销、重连宽限及租约重新获取。云端以 60 秒宽限表示重连中，计时依据和字段约定见 [HTTP 与 WebSocket 协议](../../protocol/http-ws.md)。

## 23C · Linux 与公网复验

Ubuntu 22.04 上已用冻结源码运行 160 项车端及 ROS 回归，并用现有 Gazebo 构建验证传感器、TF 和底盘超时停车。公网受保护 API 返回 401 且 TLS 验证通过。以上证据分别来自独立测试，联合链路的关闭条件如下。

Ubuntu 隔离同机 WSS 与已校准平地 Gazebo 已通过控制租约、0.119 m 前进、松手停车、Agent 暂停后约 0.115 s 归零、重连换代及 500 ms 心跳回复延迟下的在线与运动资格隔离。连续五次 503 握手失败后，设备经历重连中、离线、恢复在线，原 Agent PID 和 IPC 套接字保留。Chrome 在定位辅助模式下实测前进 0.0605 m，松手后稳定速度为零。公网复验单独跟踪，详见[剩余工作清单](../../testing/2026-11-07-remaining-work.md)。

- [ ] 在同一冻结版本更新云端、Agent 和 ROS 适配器，记录构建版本、虚拟机时间同步和实测 RTT。
- [ ] 分别验证正常网络、500 ms RTT、心跳超时、连续五次重连失败和网络恢复。记录网页状态、Agent PID、socket 是否持续监听及连接代次。
- [ ] 在符合运动时序的链路上验证网页短按前进、松手停车、运动中失联停车和恢复后重新取得租约。以仿真里程计及 ROS 日志作为证据。
- [ ] 更新剩余工作清单。公网高延迟遥控能力以运动时序和停车测试结果为准。
