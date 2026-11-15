# 手动意图最新值流（2026-11-16）

基线：`ab241065e9446e933155bff5b1b58badc6d5e2cd`，功能分支 `fix/degraded-drive-speed`。用户本轮第1—6项覆盖旧逐条回执和运行状态准入要求；本计划负责第1项和第3项网页配合。

## 实施与接口

1. 网页 ManualController 按100ms更新并发送最新摇杆意图，发送完成仅代表本地传输，不等待车端响应；socket拥塞丢弃当前样本，下个周期重采样，不积压动作。
2. relay独立发送 `control.connection` 连接身份事件，网页不再从运行snapshot建立手动epoch。RobotClient/VehicleTransport 为意图使用单向发送接口，不进入 pending/重试；获取与释放控制权、急停及复位保持独立请求/状态语义。网页入口不消费 allowed_actions、健康或报告新鲜度作为手动准入；连接epoch/来源隔离保留。
3. relay 只转发/合并 drive.intent，不登记 manual_requests，不生成合并成功假回执；仅获取/释放控制保留请求映射。删除基于状态能力副本的手动准入。
4. Agent 最新值槽只等待本地发送，不等待IPC回复或云端回报。接收时间和本地300ms断流期限原样交接；替换/停止/释放清槽，不延长期限，不重放旧值。
5. `MotionPort.send_intent(envelope, received_steady, deadline_steady)` 交付独立IPC客户端；`IpcClient.send` 的 `manual_intent` 帧不读回复，`IpcServer` 对该类型不写回复。车端worker维护 TaskAwareMotion/composition/ROS接收接口、来源序号校验和最终输出期限；通信worker独占 runtime/ipc.py。TaskAwareMotion读取冻结整组driving与driving_version，IpcMotion将其随意图透传。
6. 状态上报仍走独立快照。传输错误明确记录并撤销连接；不以成功回执或吞异常掩盖失败。身份认证、车辆ID、连接epoch和来源隔离不变。

## 文件归属

通信worker：web/src/api/{client,protocol,vehicleState}.ts、web/src/features/drive/{ManualController,DriveView}、cloud/relay.py、agent/{main,connection,websocket}.py（按需）、runtime/ipc.py、web/src/features/devices/VehicleStateView.tsx及docs/protocol/{v1,http-ws}.md。
车端worker：protocol.py、agent/{manual_control,composition,ports}.py、motion/、ROS节点及配置。跨边界通过 Orca message 协调，不同时编辑同文件。

## 最终统一测试（此阶段不迁移/运行）

- 400ms状态/控制回执延迟下100ms意图发送持续；意图无pending、无响应依赖、无重试，拥塞仅保留最新样本。
- 无/慢/历史运行报告、allowed_actions不允许、healthy=false均不阻止合法意图交接；最终控制循环仍可输出制动/零速。
- 原始received_steady期限跨排队/IPC不重置；停止发送到期撤销旧目标；释放/急停优先且不能被旧样本复活。
- 车辆ID、认证、连接epoch、controller_generation、authority/gesture/sequence隔离；重连不恢复旧会话。
- 获取/释放、急停、复位与任务控制响应/幂等回归；IPC单向流与请求应答互不串帧。
- 受影响测试：web/src/api/{client,vehicleState}.test.ts、web/src/features/drive/drive.test.tsx、devices.test.tsx，以及tests/integration/test_{cloud,agent,manual_wire,limited_manual_ingress,agent_snapshots,ipc_lifecycle}.py、tests/unit/test_{cloud_epoch,agent_ports,ipc_frames,manual_authority,manual_session,limited_manual_rearm,degraded_manual_speed}.py、tests/ros/test_agent_adapter_shutdown.py；仍有效保护测试保留。

本阶段仅语法/类型静态检查；不启动开发服务、ROS或车辆，不发送运动命令，不提交/推送。代码修改完成后由负责人审diff并统一迁移/补充测试及验收。
