# 运动路径收敛：迁移计划与证据

基线 `8858514a966843e5e5f70e51d629c7cf58b08f13`，功能分支 `fix/degraded-drive-speed`，开始时工作区干净。遵循上级 AGENTS、新版 vehicle-motion-redesign 设计及实施计划；不启动服务、ROS、VM、Docker 或车辆，不修改车端配置。

## 删除前消费者迁移清单

1. 网页现行 ManualController 生产 drive.intent，云 relay 的 controller_generation/manual_requests 隔离和 Agent 的手动 TTL/合并队列是现行消费者。删除同文件中旧 drive.challenge、drive.clock、drive.velocity、服务端期限换算及其独立反馈转发；心跳仍负责连接活性和身份，不为自主任务续租。
2. ROS command_node 的 ManualIngress 生产完整 FinalCommand，导航任务使用带 task_epoch 的本地样本。将 Arbiter 收敛为急停和任务导航源拥有者，删除无 task_epoch 的服务器租约导航、旧手动速度仲裁和恢复伪入口；保留 FinalCommandFence、PersistentEstop 和源身份/序号/TTL。
3. SafetyGate 的 MotionCoreOutput 已消费有原始来源围栏的扫描、深度准入与 profile。保留这些共享入口，将无核心时旧 worker/旧 MotionEvidence 安全门及输出分支删除；profile 必选，标准仿真默认显式选择现行仿真 profile，实车仍必须提供有效硬件 profile。
4. motion_fusion 的新版消费者是 NativeObservationRosBridge；旧 MotionEvidence 只服务旧安全门与旧进展资格服务。移除旧发布/进展恢复专用入口，标准运行只保留原生观测桥；同步 CLI、launch、配置、注册及当前操作说明。
5. 离线 DeviceProfile.self_mask 仍有 coverage、几何等价与离线回放消费者，不能删。self_mask_calibrated 已是输入迁移时忽略字段；迁移矛盾测试，保留配置迁移并说明，不恢复资格门。
6. 按实际消费者删专用测试，迁移共享认证、跨车/来源隔离、急停、任务离线继续和 TTL 回归。历史设计/历史结果保留时点事实，当前说明更新到唯一入口。

## 范围外与验收限制

不改 1 米前障碍角扫掠、启动学习分类容差；保留统一估计、默认 0.3m/s 可配比例限速、decel=.5、停车风险、15cm 启动和 GPS 学习成果。Linux ROS 消息生成/guard/scan matcher 链接、SCF3/823 与 ScanMotion 双 capture 同版重建、实车验收仍 Draft。

## 实施结果与有效入口

- 网页 `drive.intent` → relay 的原 socket/controller_generation → Agent 本地原始收包 TTL → `ManualIngress` → `FinalCommand` → `MotionCoreOutput`。删除 drive.challenge / drive.clock / drive.velocity、旧回复字段、x-drive-timing 协商、云端期限签发/换算、旧速度/反馈队列及 Supervisor 速度槽；连接心跳只验证 nonce、身份、超时和连接活性，不再向 ROS 发送运动心跳或续租。
- `Arbiter` 收敛为持久急停使用的结果/围栏和本地任务导航源；手动 authority 由 `ManualIngress` 独占。导航启动必须携带 task_epoch、effect_id、稳定 action_ids，删除连接租约命名空间及未围栏的旧 IPC 启停/取消出口，状态查询仍可读。车端导航样本仍保留真实 publisher/source timestamp、执行专用 topic、序号、原始 TTL 及终态回收证明。
- 删除未选择核心时的旧 SafetyGate worker/评估/输出分支；profile 必须有效，空选择报错，不返回“永远通过”的资格。标准仿真默认显式选用 `fortress-candidate-unvalidated-v1`，使用当前配置的本地 ROS 域与模拟时钟；模型仍 unvalidated。实车要求显式硬件 profile 与安全参数，不再用 Agent-only 旧启动分支兜底。
- 删除 MotionEvidence.msg 及生成注册、旧 MotionFusion 推进连续性/重核验服务、CLI requalify-motion、旧 gate/profile 的专用参数、两个无消费者的仿真 overlay 及安装注册。motion_fusion 仅保留原生观测诊断 facade 和真实 TF；控制估计 owner 仍是输出组合中的 MotionSensorRuntime，诊断 `/motion/core_reference` 不授予控制权限。采集工具已迁移到当前诊断生产者，保留原始 JSON、区间和来源身份。
- `FinalCommand` 是唯一指令入口，删除无消费者的 `/safety/command_in` Twist 镜像。每条指令仍经原生 guard、FinalCommandFence、控制器和方向约束。ROS DriveExecutionFeedback 消息仍承载实际消费的 retired/task_retired/task_collected 本地证明，旧网页逐速度回执协议已删；估计反馈与状态继续走 MotionRuntimeState。
- 共享 SourceEpochFence / ProducerTopology / DeferredSourceQueue 移到 motion/source_fences；rgbd_body_motion 的 SE(2) 工具移到 motion/kinematics；扫描原始收包结构独立为 raw_safety_observation。先迁移实际消费者，随后删除旧 async_guard、guard、motion_estimator、motion_runtime、slip、depth_geometry、velocity_uncertainty、drive_feedback、drive_timing 模块。
- 额外修复清理中发现的失效回调残留：原生反馈 writer 丢失时不再访问已删除旧队列，而是撤销原终态回执证明并要求重新观察当前 owner；保留/加强原始身份隔离，有专门回归。
- 前端 HomeControls 仍由 ControlSurface 消费，已保留 Home 设置、返航和地图约束；删除旧 navigation.start/resume/cancel 消费，继续/取消通过 VehiclePort.taskCommand 使用当前 task_id/version/allowed_actions。无当前任务运行状态时不能用旧 route_progress 缓存发控制。前端类型列表、协议、操作说明和验收矩阵同步更新。

## 明确保留的兼容与残留

| 内容 | 实际用途与边界 |
| --- | --- |
| 持久驾驶设置字段/版本、旧 SQLite 迁移、任务定义/调度数据模型 | 现有数据库/用户配置仍需读取；本轮未删除持久化格式或修改任务存储。通用任务库不是标准车端运动的另一个输出入口 |
| real_runtime 的 YAML 读入迁移、DeviceProfile 的旧 calibrated 标志忽略、旧 frame/统一年龄迁移 | 老设备文件仍可能包含退役 gate 参数，读入时忽略并标记 `# //Compatible`，不改现场文件，不以这些字段选择旧控制器。AgentSettings 的旧 disconnect_seconds 同样只读入丢弃 |
| 离线 self_mask / coverage / frozen geometry oracle | 真实找到的消费者是离线捕获 fixture 的几何/过滤等价回归与库函数；没有生产配置入口。保留纯几何 API，不恢复生产 self_mask_xy 或 calibrated gate；矛盾的 test_uncalibrated_mask_is_unusable 已迁移为“旧标志 True/False 均不授予或撤销资格” |
| EmptyPayload 的空可选字段形状 | 原生急停意图的持久 fingerprint 已使用该 JSON 形状，保留以免升级后同一 request_id 重试冲突；不会签发旧驾驶期限或授予旧速度权限 |
| RouteProgress 的可选旧任务字段、旧快照显示、旧日志/trace 解析 | 用于读缓存/历史数据，不参与新 RoutePlan 的接受；新计划必须具备任务代次与动作身份 |
| 仿真 topology-only 来源分支、真实原生 receipt、深度/扫描 ingress、急停状态和回收消息 | 是当前真实来源隔离、传感器根时间或安全状态消费，不是旧 gate。仿真 provenance 仍明确较弱，实车不能继承该资格 |
| 历史设计、计划、测试报告与 fixture 元数据 | 保留当时事实。当前运行源码/launch/注册/工具无已删除模块或 MotionEvidence 的运行依赖；负向协议测试有意保留退休协议字符串以证明拒绝 |

未修改 config 下车端参数文件，亦未改 controller、constraints、estimator、startup_learning、self_learning、gps_course、settings/models 的实现。默认 0.3m/s 可配比例限速、既有 decel=.5 配置、停车风险、15cm 启动学习和 GPS 学习成果保持原版；原始 CRLF 文件约定已保留。1 米前障碍角扫掠、启动分类容差及其他排队问题未处理，交负责人后续审查。

## 回归命令与结果

环境：独立 `.venv`，CPython 3.12.15，使用 uv.lock 离线安装 safety/agent/cloud 与 dev 依赖；Node v24.14.1。没有运行服务、ROS、VM、Docker、Gazebo 或车辆。临时 socket / 本机临时端口 / 原生帧假子进程测试经检查后按权限机制执行。

| 命令或证据 | 实际结果 |
| --- | --- |
| `.venv/bin/python -m pytest tests/unit --ignore=tests/unit/test_docker_start_helper.py --ignore=tests/unit/test_motion_guard_to_owner.py --ignore=tests/unit/test_websocket_link.py -q --tb=short` | 1576 passed / 13 failed / 4 skipped；其中两项是本轮退休 API 的旧断言，已迁移并在后续定向回归通过，其余 11 项在 8858514 独立归档基线同环境复现 |
| 基线 `.venv/bin/python -m pytest tests/unit/test_navigation_collision_qualification.py tests/unit/test_raw_capture.py tests/unit/test_rgbd_parent_ledger.py tests/unit/test_terminal_collection_capacity.py tests/unit/test_terminal_native_channel.py tests/unit/test_terminal_release_collection.py tests/unit/test_terminal_release_wire.py -q --tb=short` | 基线 11 failed / 66 passed；涉及旧夹具缺 odometry_topic、采集年龄、RGB-D ledger、终态回执，未借本次清理修改其算法 |
| `.venv/bin/python -m pytest tests/integration -q --tb=short` | 653 passed / 1 failed / 1 skipped；唯一失败是测试仍向 RouteService 传旧 connection_epoch，已迁移，其整组和其他受改边界在下列 158 项回归通过。未宣称该旧日志为最终整套全绿 |
| `.venv/bin/python -m pytest tests/integration/test_task_ownership.py tests/integration/test_agent.py tests/unit/test_navigation_command_clock.py tests/unit/test_settings.py tests/unit/test_task_terminal_release.py tests/unit/test_navigation_completion_handoff.py -q --tb=short` | 158 passed |
| `.venv/bin/python -m pytest tests/integration/test_agent.py tests/integration/test_manual_wire.py tests/integration/test_settings_logs_review.py tests/integration/test_short_admission.py tests/unit/test_navigation_command_clock.py tests/unit/test_settings.py -q --tb=short` | 133 passed；认证连接、手动原始 TTL、共享 task/settings gate 与原生本地接线 |
| `.venv/bin/python -m pytest tests/unit/test_heartbeat_wire.py tests/unit/test_protocol.py tests/unit/test_motion_launch_profile.py tests/unit/test_arbiter.py tests/unit/test_motion_path_convergence.py tests/unit/test_motion_core_chain.py tests/unit/test_degraded_manual_speed.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_review.py tests/unit/test_gps_course.py tests/unit/test_stopping_fused_input.py -q --tb=short` | 161 passed；保留前序运动/学习/停车成果及退休协议拒绝 |
| `.venv/bin/python -m pytest tests/unit/test_navigation_command_clock.py tests/integration/test_manual_wire.py tests/integration/test_limited_manual_ingress.py tests/unit/test_motion_launch_profile.py tests/unit/test_motion_path_convergence.py -q --tb=short` | 最后反馈失效回调修正后 43 passed，包含 writer 丢失撤销回执证明的新回归 |
| `.venv/bin/python -m pytest tests/integration/test_estop_recovery.py tests/integration/test_shutdown.py tests/unit/test_docker_start_helper.py tests/unit/test_motion_guard_to_owner.py tests/unit/test_websocket_link.py -q --tb=short` | 69 passed / 1 skipped；本机临时 IPC、急停持久化、假进程关闭、认证握手及原生帧往返。最终窄范围补验结果见下文 |
| `npm --prefix web run typecheck`、`npm --prefix web test -- --run`、`npm --prefix web run build` | Typecheck / build 通过；13 test files、166 tests 通过。没有启动 Vite/browser 服务 |
| 受改 Python 的 `.venv/bin/ruff check` | 通过；精确文件列表由 git diff 与三个新增 Python 文件组成 |
| 全范围 Ruff / Mypy | Ruff 唯一失败是未修改的 test_startup_learning_feasibility.py 原有 I001，归档基线单文件复现；Mypy 67 errors / 9 files，归档基线同为 67 / 9，分布一致。没有把既有问题改成伪全绿 |
| AST 删除引用与打包注册检查；`git -c core.whitespace=cr-at-eol diff --check` | 通过；所有 setup.py data_files 存在、console_scripts 的模块/main 存在，src/ros2/tools/tests 无已删除模块 import |

自动审批审查拒绝了宽范围 `tests/integration tests/e2e` 提权命令，理由是可能触发仿真/网络资源且超出禁止启动服务约束；没有绕过拒绝。此前普通权限的 e2e 尝试只到 fixture 临时端口 bind 即被沙箱拒绝，不能计为运行验收。采用已核查的窄范围测试完成必要回归。

完整 Linux ROS 消息生成、guard 与 scan matcher 链接、SCF3/823 和 ScanMotion 双 capture 同版重建、完整 UI/仿真链和实车仍 Draft。仅独立原生帧测试编译/往返通过，不等于 ROS guard/RMW 链接或车辆验收。最终由负责人亲审累计实际 diff 和原始证据；本工作仅本分支提交，不推送、不自动合并。

最终窄范围 `.venv/bin/python -m pytest tests/integration/test_estop_recovery.py tests/integration/test_shutdown.py tests/unit/test_motion_guard_to_owner.py -q --tb=short`：**47 passed / 4.73s**，已按更安全的明确范围获得执行许可；没有执行被拒绝的宽范围命令。

最终持久化/协议补验：`.venv/bin/python -m pytest tests/unit/test_protocol.py tests/unit/test_motion_path_convergence.py tests/integration/test_safety_intent_store.py -q --tb=short`，**33 passed / 0.27s**；急停 fingerprint 保留字段形状的说明与回归一致。

## 最终代码版本与原始证据

代码/测试规范化 LF SHA-256：`fc3f5fcc4b5d3a07bf52264d72083692ea8ea5d6633afdbae6463b071e7bf664`（按 src、ros2、tools、tests、web/src 下现存 636 个版本文件的路径与内容排序计算；逐文件原始 SHA-256 随证据保存）。文档不参与该指纹。

本地证据目录：`/private/tmp/vehicle-motion-convergence-8858514/`，含原始测试日志、基线对比、逐文件指纹、累计 patch 与提交元数据；本分支最终提交 SHA 见其中 commit.json 及 git log。

## 负责人审阅后的测试资产返修

仍有效的 DDS barrier/身份/guard 到 owner 的组合保护不应随旧夹具整文件丢弃。按后续收窄审阅，仅恢复小型参数化真实 DDS 组合资产；旧 progress、MotionEvidence、协方差资格门、逐速度回执和租约断网导航测试不恢复，已有等价保护不重复。精简逐场景去向、实际运行命令、版本及未执行边界见 [运动测试迁移审查](2026-11-15-motion-test-migration.md)。
