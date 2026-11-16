# 手动发送链实施记录（2026-11-16）

代码修改完成，统一测试待执行。功能分支 `fix/degraded-drive-speed`，HEAD基线 `ab241065e9446e933155bff5b1b58badc6d5e2cd`，未提交、未暂存、未推送。

## 本worker实际修改

- ManualController删除busy/回执串行等待及冗余待发槽，按100ms持续发送当前摇杆值，松手/失焦/退出清理当前值；传输拥塞丢本次样本，下周期重采样。
- RobotClient意图直接写socket，不登记pending、不重试、不解析drive.intent回执。获取/释放控制、急停/复位仍有独立请求响应；延迟响应只更新RTT，不反向更改手动连接状态。
- 云端去掉意图manual_requests项、合并成功假回执及基于motion capability状态副本的手动准入；保留认证、车辆识别、epoch、controller_generation、session及sequence隔离。
- 新增relay-owned `control.connection` 独立连接身份通知，包含v/type/robot_id/connection_epoch，建立与退役手动epoch不再等待任何运行报告。状态缺失/历史/慢更新及allowed_actions不会关闭手动入口；报告仍按真实新鲜度显示。
- Agent最新值槽通过MotionPort.send_intent交接，删除逐条_reply及direct dispatch旧请求路径；释放/停止/断连清槽。received_steady与300ms deadline_steady不重置，原本地断流停车职责继续由车端执行，不延长TTL。
- IpcMotion使用必填独立intent_client，发送manual_intent单向帧并透传driving/driving_version。IPC server读入与交接分离，只保留一个最新待处理值，通知不写reply；独立请求client保留原应答语义。
- 网页移除expired运行状态枚举/文案，协议文档同步删除旧回执/150ms说明。

## 接口协作

已与车端worker dispatch:ctx_c0f46efce24e 协调：TaskAwareMotion冻结读取整组驾驶配置，composition提供intent_client，agent_adapter接收manual_intent但禁止旧manual意图请求；车端worker维护protocol及最终输出、来源退役和期限执行。软件急停owner及纯运行期内存语义不变；本worker未编辑native_receipt.py、tests/test_native_receipt_diagnostics.py及既有急停报告修改，也未编辑用户根目录方案文档。

## 静态检查与验证边界

- `npm --prefix web run typecheck`：通过（tsc --noEmit）。
- Python `ast.parse`：agent/main.py、agent/connection.py、cloud/relay.py、runtime/ipc.py全部通过。
- `git diff --check -- <本worker修改路径>`：通过。
- 按用户指定未迁移/新增测试，未跑测试套件，未启动开发服务、ROS或车辆，未发运动命令；不声明功能验收通过。

## 统一验证待办

负责人亲审实际diff后统一迁移/补充测试：400ms回执下100ms连续发送、无意图pending/回执/重试、socket及多层背压合并、IPC单向和请求通道隔离、无运行报告仍获取控制/发送、过期报告不更改epoch、旧epoch/车辆/来源/序号隔离、300ms本地断流及释放/急停不能被旧值复活、任务自主与急停/复位回归。受影响测试清单见实施计划；旧测试fixture要增加独立control.connection事件及MotionPort.send_intent接口，保留有效来源和急停保护。

同版网页/relay/Agent/ROS部署是本次协议切换要求，不保留旧回执兼容链；本阶段未执行真实网络时序、车辆输出或ROS运行验收。没有本worker范围内待写代码；整体功能需车端worker完成及负责人统一验证。

## 文件与指纹

下列文件按路径排序，逐项 `path + NUL + bytes + NUL` 拼接的SHA-256（不含本记录）：`e52a04bddf457ee8a432c70845e710d6a0200e98f59d5971e47404af9be3801b`。

- `docs/superpowers/plans/2026-11-16-manual-intent-stream.md`
- `docs/protocol/v1.md`
- `docs/protocol/http-ws.md`
- `src/senior_control/agent/connection.py`
- `src/senior_control/agent/main.py`
- `src/senior_control/cloud/relay.py`
- `src/senior_control/runtime/ipc.py`
- `web/src/api/client.ts`
- `web/src/api/protocol.ts`
- `web/src/api/vehicleState.ts`
- `web/src/features/drive/ManualController.ts`
- `web/src/features/drive/DriveView.tsx`
- `web/src/features/devices/VehicleStateView.tsx`
