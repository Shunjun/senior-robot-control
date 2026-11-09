# 单页驾驶台、多航点任务与遥控时序 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在一个车辆页面完成驾驶、导航和多航点任务管理，并提供可实测的遥控时序和车端反馈。

**Architecture:** 先实现单屏工作台和真实地图、航点、任务详情读取，再交付路线编辑和生产执行。遥控时间契约及执行回执独立开发，最后接入驾驶工作台并在 Ubuntu 进行联合验收。

**Tech Stack:** React、TypeScript、CSS Modules、Python、FastAPI、SQLite、ROS 2 Humble、Nav2、Gazebo、Vitest、pytest、Playwright。

**Spec:** [单页驾驶台、多航点任务与遥控时序规格](../specs/2026-11-08-operator-workspace-design.md)。

**Status:** 待审阅。审阅通过后进入实现阶段。

## Global Constraints

- 网页速度周期 **100 ms**，云端基础有效期 **300 ms**，云端→车机传输年龄上界 **500 ms**，车端接纳后最低计划执行窗口 **200 ms**。
- 保留各层安全门、急停优先、引用和连接代次围栏。缺少真实来源时明确显示不可用。
- 组件超过三个 props 使用命名类型。场景样式放 CSS Module。兼容代码标注 `//Compatible`，Python 使用 `# //Compatible`。
- 排除 CI。实现按 GPT‑6.1 Sol low 开发，复用现有工作区成果并隔离本轮变更。未经调试或验收不启动服务。
- 每个开发任务先写指定失败测试并确认 RED，再实现并确认 GREEN。提交只包含该任务的差异。

## Review Focus

- 切换地图或视频时摇杆还在按住，组件保持且焦点丢失仍停车。任务 1。
- 地图引用变更时旧航点、旧图块、旧任务详情迟到，均丢弃。任务 2、3。
- 取消或断线后 Nav2 迟到结果不得启动下一点。任务 4。
- 迟到速度、时钟跳变、停车先到和旧代次均不能续时。任务 5。
- ROS 接纳了速度但安全门拦截或样本被替换，网页延迟不更新成已执行。任务 6。

### Task 1 · 单页工作台与场景切换

**文件** `web/src/features/devices/{ControlSurface,RobotPanel}.tsx`，新增 `OperatorWorkspace.module.css`，测试 `devices.test.tsx` 与 `web/e2e/control.spec.ts`。

**接口** `WorkspaceMode = "manual" | "navigation" | "tasks" | "mapping"`，`PrimaryView = "video" | "map"`。沿用 `robotId`、`link` 和 `snapshot`。

- [ ] 新增 `test_keeps_manual_controls_mounted_on_primary_view_switch`、`test_leaving_manual_revokes_gesture_without_rearm` 和 Playwright 单视口用例。断言视图切换摇杆实例保持，切出驾驶仅停止一次，返回后需重新获取；桌面及手机横竖屏无需滚动文档即可同时看视频和停车控件。
- [ ] 运行 `npm --prefix web test -- --run src/features/devices/devices.test.tsx`，确认新场景断言先失败。
- [ ] 实现场景导航、固定状态栏、地图/视频主视图和独立侧栏。复用现有驾驶和视频组件。设备切换清空手势及视图状态。
- [ ] 复跑定向测试和 `npm --prefix web run build`，记录通过。提交该工作台改动。

### Task 2 · 真实地图、航点与任务详情读取

**文件** `src/senior_control/{protocol.py,agent/{dispatcher,ports,snapshots,composition}.py,waypoints/store.py}`，`web/src/api/{client,protocol}.ts`，新增 `tests/integration/test_workspace_data.py`。

**接口** 拟新增 `task.get(task_id)`、`waypoint.list(map_ref, cursor)`、`map.manifest(map_ref)`、`map.tile(map_ref, tile_x, tile_y)`。成功回复回显车辆及地图引用。任务详情含 `waypoint_ids`、`schedule`、`timezone`、`enabled`、`revision`。地图清单含原点、分辨率、尺寸和图块大小。

- [ ] 写 `test_workspace_read_is_reference_fenced` 与 `test_tile_fits_ipc_frame`。旧引用拒绝，真图块可拼接且不超过现有 IPC 8 KiB、WS 64 KiB，缺源明确不可用。
- [ ] 运行 `.venv/bin/python -m pytest tests/integration/test_workspace_data.py -q` 验证 RED。
- [ ] 复用车端权威任务和航点存储，按引用分页读取。地图采用有界图块读取，首版 32×32 单元，控制帧优先；接入有引用代次的真实地图来源。后端无此来源时可交付明确的不可用状态，真实地图验收保持未勾选。
- [ ] 运行定向测试和前端类型检查。补跑旧协议输入，确认旧数据仍可读，提交数据契约改动。

### Task 3 · 地图上的多航点编辑

**文件** `web/src/features/tasks/{TaskList,ScheduleEditor,tasks.test}.tsx`，新增 `RouteEditor.tsx`，修改 `Tasks.module.css` 与 `web/src/features/map/MapCanvas.tsx`。若新增点图保存，接入 `src/senior_control/agent/dispatcher.py` 与现有 `WaypointStore.save_waypoint`。

**接口** `RouteDraft` 保存有序 `waypoint_ids: string[]`、`map_ref`、`expected_revision` 和调度。地图预览输入为同一引用的有序航点坐标。

- [ ] 写 `test_reorders_and_reopens_three_point_route`、`test_reference_change_rejects_late_route_read` 和 `test_single_point_compatibility`。精确断言保存顺序、冲突保留草稿、旧地图不能提交。
- [ ] 运行 `npm --prefix web test -- --run src/features/tasks/tasks.test.tsx` 验证 RED。
- [ ] 实现固定地图、路线编辑、编号连线及调度表单。点图新增必须先明确保存航点，再纳入草稿。任务新建使用现有 `task.create`，编辑使用 `task.update` 与该任务的 `expected_revision`。
- [ ] 复跑任务及地图测试和生产构建。用三点任务保存、重读并核对顺序后提交。

### Task 4 · 车端有序路线执行

**文件** `src/senior_control/{agent/ports.py,tasks/executor.py}`，`ros2/src/senior_robot/senior_robot/{agent_adapter,navigation_adapter}.py`，`tests/integration/test_task_runner.py` 与 `tests/ros/test_navigation.py`。

**接口** `start_qualified(execution_id, task, map_generation)` 消费 Task 2 的同源地图与航点数据。执行结果关联 `execution_id`、引用代次、活动航点索引。

- [ ] 写 `test_qualified_route_runs_in_order` 与 `test_cancel_or_map_change_drops_late_goal_result`。验证 A→B→C，B 失败或代次变化不启动 C，车机失联后不续跑，浏览器离线不终止后台计划任务。
- [ ] 运行 `.venv/bin/python -m pytest tests/integration/test_task_runner.py -q` 验证 RED。
- [ ] 接入真实 Nav2 动作与取消确认。每点开始和完成时验证当前执行与引用代次，旧结果不能更新进度或启动下一点。
- [ ] 复跑任务集成和 Ubuntu Nav2 仿真。验证三点、失败、取消和重连。缺少可信活动地图引用与导航接线时保持运行入口锁定。提交可验证的执行改动。

### Task 5 · 补偿截止与车端最小执行窗口

**文件** `src/senior_control/{protocol.py,cloud/relay.py,agent/{main,websocket,connection}.py,safety/arbiter.py}`，`ros2/src/senior_robot/senior_robot/{agent_adapter,command_node,safety_node}.py`，`web/src/{api/client.ts,features/drive/DriveView.tsx}`。测试 `tests/unit/{test_heartbeat_wire,test_drive_challenge,test_arbiter}.py` 与 `tests/integration/test_shutdown.py`。

**接口** 定义 `DriveSample` 的车辆、租约、连接代次、手势代次、序号、产生时间、云端签发时间和补偿截止。定义可信时钟偏差区间的提供接口。车端最终接纳后输出本地 `steady_deadline`，下游原样消费。

- [ ] 写 `test_remote_hold_floor_after_valid_admission`，断言截止 1650 ms 时在 1276/1600/1650 ms 到达分别得到 1650/1800/拒绝。补测 500 ms 上界、浏览器迟到输入、时钟跳变、少于三条可信样本、停止先到及重连旧包。
- [ ] 运行 `.venv/bin/python -m pytest tests/unit/test_heartbeat_wire.py tests/unit/test_drive_challenge.py tests/unit/test_arbiter.py -q` 验证 RED。
- [ ] 先交付四时间戳偏差区间测量，分别用于浏览器→云端和云端→车机。测试同机、非对称路径、时钟跳变和空交集。补偿使用 10 秒内至多 20 个可信样本的 P90，至少三样本。对无法证明时钟误差上界的链路保持驾驶未授权。
- [ ] 实现 100 ms 最新值采样，解除逐条 ACK 对发送节奏的限制，沿用短时租约，按手势代次隔离停车和新旧速度。ROS 仲裁器作为最终接纳点，在同一原子处理内校验和计算 `max(补偿截止, 接纳+200 ms)`，IPC 后续阶段不重新赋时。旧协议保留其现行约束，能力协商通过后才启用新规则。
- [ ] 复跑前述 Python、`tests/integration/test_shutdown.py` 和网页驾驶测试。测量断线至归零和松手至实际停稳，分别记录。提交通过的时序改动。

### Task 6 · 实际输出反馈与网页延迟

**文件** `ros2/src/senior_motion_msgs/msg/FinalCommand.msg`，新增 `ros2/src/senior_motion_msgs/msg/DriveExecutionFeedback.msg`，修改 `ros2/src/senior_motion_msgs/{CMakeLists.txt,src/final_command_guard.cpp}`、`ros2/src/senior_robot/senior_robot/{agent_adapter,safety_node}.py`、`src/senior_control/{protocol.py,cloud/relay.py,agent/main.py}`、网页 `client.ts` 和 `DriveView.tsx`。

**接口** `DriveExecutionFeedback` 关联车辆、当前连接代次、手势代次和指令标识，状态取 `applied | blocked | superseded | expired`，输出成功时含实际 `vx/wz` 和是否限幅，拦截可带拒绝原因。浏览器用自己的发送和回执接收时间计算延迟。

- [ ] 写 `test_reports_applied_only_after_safety_output`、`test_latency_ignores_old_or_unapplied_ack`。确认普通 IPC 接纳、旧连接回执及安全门拦截均不会更新成功延迟。2 秒无新样本标陈旧。
- [ ] 运行新增 ROS 回执测试和 `npm --prefix web test -- --run src/features/drive/drive.test.tsx` 验证 RED。
- [ ] 让现有命令围栏把稳定关联键传到最终发布点。安全门按实际输出生成回执，Agent 经云端仅向对应控制连接转发。浏览器用有界待确认表管理发出时间和回执。
- [ ] 在 Ubuntu 重建受影响消息和原生守卫，验证命令来源与 `/cmd_vel` 唯一发布者约束，再复跑网页和 Python。提交回执改动。

### Task 7 · 联合验收

**文件** `web/e2e/control.spec.ts`、`tests/e2e/test_drive.py`、`docs/testing/2026-11-07-remaining-work.md`。

- [ ] 将无滚动视频驾驶、切换场景、真实输出反馈、三点路线编辑与实际导航接到同一冻结版本的隔离环境。
- [ ] 在现有 Ubuntu 检查可用内存、磁盘和时钟，复用构建，串行执行。验证稳定高延迟、500 ms 上界、突发抖动、重复旧速度、丢停车包、过期回执、半开与重连。记录实测 RTT、单向估计及误差、速度输出、里程计和页面反馈。
- [ ] 运行 Python 单元/集成、前端 Vitest/生产构建及隔离 Playwright，确认退出码和具体通过范围。确认进程回收，直接在清单勾选有证据的子项。
- [ ] 隔离仿真与公网、实车分别记账。提交验收记录。

### Task 8 · 车端部署、配对与仿真 CLI

本任务可独立于网页任务开发。若在联合验收前完成，Task 7 可使用该工具启动隔离仿真。

**文件** 新增 `src/senior_control/cli.py` 与 `tests/integration/test_vehicle_cli.py`，修改 `pyproject.toml`、`docs/operations/{deploy,hardware,simulation}.md`。复用 `src/senior_control/agent/pairing.py`、`deploy/senior-agent.service` 与现有 ROS 仿真 launch。

**接口** 在 `[project.scripts]` 注册 `senior-robot = "senior_control.cli:main"`。入口为 `main(argv: list[str] | None = None) -> int`，提供 `pair`、`doctor`、`deploy`、`start`、`stop`、`status`。操作均选用显式的 `--profile`。

- [ ] 新增 `test_pair_preserves_existing_credentials` 与 `test_deploy_dry_run_leaves_files_and_services_untouched`。验证配对走 HTTPS，凭据权限为 0600，失败和日志不泄露秘密，预览无副作用。
- [ ] 新增 `test_simulation_waits_for_matching_adapter`、`test_failed_start_reaps_only_owned_processes` 和 `test_restart_preserves_estop_state`。覆盖 ROS 就绪后再启动 Agent、失败回收、重复启动、状态准确及急停状态保留。
- [ ] 运行 `.venv/bin/python -m pytest tests/integration/test_vehicle_cli.py -q`，确认新增用例 RED。
- [ ] 实现配对和只读检查，复用现有一次性码兑换和诊断。实车部署只写所选档的配置及 systemd 单元，显式 `start` 后运行。仿真复用构建，记录本次进程所有权，关停先走 Agent 再回收 ROS 与 Gazebo。
- [ ] 复跑 CLI 与现有配对、IPC 关停测试。在现有 Ubuntu 验证隔离 HTTPS 配对、仿真启停、进程回收和磁盘内存余量；另在目标 Linux 验证 systemd 路径、账号与服务生命周期。提交 CLI 改动并在剩余工作清单标记实测范围。
