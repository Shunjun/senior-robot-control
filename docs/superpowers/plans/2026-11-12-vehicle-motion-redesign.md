# Vehicle Motion Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 本轮已指定 Orca 委派，由负责人审核后派发；不再询问用户交付方式，不自行创建 helper。

**Goal:** 为 senior_4wd_dl 交付同版验证的 2/5 km/h 可驾驶仿真、车端闭环与方向避障、离线任务、网页状态/配置/日志。

**Architecture:** 保留认证、感知来源保护、地图与 Nav2 能力，局部替换运动链。纯 Python 组合模块集中运动状态、约束和反馈，ROS 节点负责时钟转换与唯一输出；持久任务、配置、日志分别拥有生命周期，通过类型化接口组合，不让大节点继续增长。

**Tech Stack:** Python ≥3.10 / Pydantic 2 / aiosqlite / pytest；ROS 2 Humble、C++ 原生 ingress/scan matcher、Gazebo Fortress、Nav2；React 19 / TypeScript / CSS Modules / Vitest / Playwright。

**Spec:** `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md`（用户已批准，2026-11-12）。证据：`docs/reviews/2026-11-12-vehicle-hardware-audit.md`、`2026-11-12-vehicle-control-audit.md`、`2026-11-12-vehicle-design-builder-check-v2.md`；报告中的旧“待确认”状态不覆盖最终设计。

## 首版验收优先级调整（用户明确，2026-11-13）

首版优先跑通基础功能与正常操作：常规手动两档、平滑启停、常规方向避障、正常导航、急停按钮与恢复、五项驾驶设置保存后回读，以及日志查看，作为首版实际场景主路径。

断网、极端时序、滑移、故障注入、洪峰负载等专项实测放在交付一版后，结合用户实际环境与现场反馈再细化测试并修复；不要求专项矩阵全部通过才能交第一版。本节仅调整验收分层、交付前置与实施优先级，不撤销已批准的任务自主、车端约束、请求幂等、急停确认/恢复等行为要求，也不重写既有失败或未验收证据。

必要定向代码回归与已知明确缺陷仍须修复；不因专项后置新增安全资格门，不把未运行专项报告为已通过。后文专项矩阵作为后续覆盖清单，其未完成项不整体阻塞正常主路径首版交付。用户现场正常使用反馈纳入后续修复。

## 当前接线与分支范围（2026-11-14）

功能主分支始终为 `fix/simulation-runtime-qualification`（Draft PR #14）。先审阅并收口其全部未提交改动，再合版；`vehicle-motion-integration` 仅作临时验证，之后通过 merge 回功能主分支，不再 cherry-pick。后续独立开发由 Orca 分配独立 worktree，不能继续在共享脏树追加产品修改；每阶段提交仍须负责人亲读实际 diff 授权；按 AGENTS 既有授权，功能完成并通过对应检查和验收后直接推送功能分支并创建或更新 PR，阶段未完成如实使用 Draft；不 push main、不自动 merge。当前收口阶段不推送。

已批准的例外包含纯模块开发，以及显式隔离、simulation-only/unvalidated candidate profile 的正常接线和有界 Linux 调试；29 项正式测量为 null 不整体阻塞这些工作。候选必须逐字段保留实测证据或 assumption、绑定同版来源和隔离上下文，不启用实车，不用 truth/命令/轮速冒充独立 body 反馈，也不把候选升格为已冻结能力。实际驾驶通过、定量安全声明和实车放行仍需各自适用证据；不能延长旧 authority/证据期限来造通过。下文“生产资格/冻结”指这类能力声明与放行，不能撤销本段获批仿真接线例外。

历史任务日志中的“不接输出/待审/未提交/未运行”保留其记录时点含义；当前源码提交与未验收范围见[阶段索引](../../testing/2026-11-14-vehicle-stage-index.md)，不能倒推旧失败已通过。

## Global Constraints

- 车型 senior_4wd_dl；低档 2 km/h = 5/9 m/s，高档 5 km/h = 25/18 m/s；两档角速度初始 1 rad/s，可配置。倒车默认请求上限为 0.3 m/s，核对唯一 `src/senior_control/settings/models.py::simulation_settings_profile()`：defaults 与 maximum 均为 0.3。它是显式选择的仿真 SettingsProfile 策略，用户可在该 profile 允许范围内配置；不是硬件倒车能力或实测安全上界，实际输出仍受 T5/轮限/方向约束。车端 SI、网页线速度 km/h。
- 激光、质量合格 GPS、IMU、轮速构成多源车体反馈；允许内部限速的轮速退化，轮速不冒充独立测速；DriveExecutionFeedback 只是执行回执。ground truth 仅作隔离验收真值，不进入运动估计、响应模型或反馈控制。
- 普通加减速闭环；反向切换先停稳；首版无反向制动实现/开关。5% 几何余量按长宽各 ×1.05，附件包含在足迹中；不和时间/测量误差重复记账。
- 网页无校时/挑战/延迟资格；保留 RTT、来源代次、车端手动活性、感知时效与任务幂等。非手动已接受任务断网继续；接管暂停，明确继续后重规划。
- 仅开放五项驾驶设置：低/高线速度、低/高角速度、统一倒车上限；另开放日志级别/保留时间/临时调试与自动恢复时长。无磁盘限额及其他驾驶参数输入项。
- 故障停车、障碍停车、软件/实体急停独立；软件急停持久化与版本复位，复位静止待命。测速退化按源质量与方向感知选择受限继续或受控停车；健康恢复至少 5 份连续合格估计且跨度 ≥0.5 s，T1 验证该初值。
- 不擅自启动开发服务；后续仿真启动仅为明确的有界调试/验收并回收。兼容代码标注 `//Compatible`（Python 注释写 `# //Compatible`）；组件 props 超过三项用命名类型，局部样式放 CSS Modules。
- 分支、worktree 与合版方式按前述当前范围执行；逐项审实际 diff、复用 Draft PR #14，不 push main、不自动合并，不把其他脏文件混入阶段提交。
- 架构/核心接口及架构 review：GPT-6.1 Sol / medium，不用 Cursor；常规非架构实现优先既定 Cursor 模型；盘点可 Terra / medium。负责人必须亲自读实现 diff。

## Review Focus

1. 断网后积压的同一按住动作或迟到 acquire 回执不能复活手动运动：T2 的封闭代次与新用户边沿测试。
2. 到达/取消/接管同时发生不能重复动作或复活终态任务：T6 的串行转换与持久进度测试。
3. 仿真暂停、倒退、观测换源或辅助量缺失不能把旧观测改时间继续行驶：T3/T5 双时钟测试。
4. 慢计算且持续更新时既不能饿死放宽，也不能让新障碍漏过：T4 的确定性调度与真值停车测试。
5. 配置落盘失败、日志洪峰/越权查询不能改变运动或阻塞急停：T7/T8 的故障注入与资源边界测试。

---

## 代码事实、替换边界与依赖

已阅读下列实际入口；新文件是计划产物，不假称已经存在。

| 现有文件/调用关系 | 实施责任 |
| --- | --- |
| `web/src/features/drive/DriveView.tsx` 的 `DriveController` 当前输出 vx/wz、满杆 0.5；`web/src/api/client.ts` 同时维护 timing/challenge 分支 | T2 改线协议；T9 改用户控件与状态，删除旧分支 |
| `src/senior_control/protocol.py` → `cloud/relay.py` → `agent/websocket.py` → `agent/main.py` → `agent/connection.py:IpcMotion` → ROS `agent_adapter.py`/`command_node.py` | T2 统一意图授权与消息边界；T6 分离任务/连接；T8 日志独立有界通道 |
| `safety/arbiter.py:Arbiter` 把导航和服务器心跳关联；`command_node.py` 输出 `/safety/command_in` 与 `FinalCommand`，有来源保护 | T2 拆手动租约；T5 接唯一运动 tick；T6 保留导航本地时效而去网络资格 |
| `motion_fusion.py` 对齐 wheel/imu/`/motion/scan`，发布 `/motion/evidence`；`safety/motion_estimator.py` 输出区间平均 body twist | T3 独占区间到当前上界，不给轮速换名 |
| `safety_node.py:SafetyGate._on_timer` 先过期/零命令就发布零，含 speed ==0.3 等历史资格；`safety/async_guard.py:SafetyTicket.permits` 要求各 generation 完全相同 | T4 替换许可与异步放宽；T5 把停车交给周期闭环，删除旧运行分支 |
| `agent/direct_navigation.py`、`tasks/{runner,runtime,executor,store}.py`、`agent/journal.py:AgentJournal`；runtime 当前连接 generation 改变就 cancel_pending | T6 使用现有 journal 幂等能力、单独任务所有权，保留调度的离线不新启动/重启不续跑 |
| ROS `navigation_adapter.py:RouteService`、`dedicated_controller.py`、`command_node.py` 的 `_NavigationSource` | T6 在接管时撤销旧输出并保留未完成航点；T5 所有 Nav2 输出共用新核心 |
| `agent/snapshots.py`、`agent/workspace.py`、`web/src/features/devices/ControlSurface.tsx`/`RobotPanel.tsx` | T7/T9 单一状态投影、配置入口；T8 日志 |
| `simulation_motion.launch.py` 动态改 gate rate、Nav2 vx_max=0.22；`simulation_reference.py` 用 ground truth 和 wheel 生成定位参考 | T1 记录默认实际配置；T5 改自洽能力配置；T10 验收隔离真值并验证定位路径 |

ROS 路径均以 `ros2/src/senior_robot/senior_robot/` 为前缀（launch/config 以其包目录为根）；Python 路径均在 `src/senior_control/`。下列文件边界引用这个约定。

**固定依赖与获批例外：** T1 测量可与 T2a 纯契约/manual、T3a 纯观测准入/模式及本轮获批 T3b 纯离线融合、T4a 纯方向约束/异步调度与T5a纯轨迹/闭环状态模块并行；T3a/T3b/T4a/T5a 依赖已有纯类型及前序纯接口，全部配置显式注入、上界可空、不接输出，不要求先填满29项测量。显式隔离 candidate 可按前述例外完成 T2/T3/T4/T5 仿真接线与正常调试；T1 适用边界冻结 → T3b 生产定量界与响应模型 → T4生产资格 → T5生产闭环资格；T2+T5→T6（任务实现接线），T2+T3+T5+T6→T7（配置/投影集成），T2+T7→T8（日志）；T5+T6+T7+T8→T9（UI）；全部→T10（同版验收）。T6/T7 可在完整依赖到齐前先做设计/纯模块，集成接线必须满足前述 T5/T6 依赖；T6/T7/T8 在各自前置完成后可并行读/实现独占新文件，共享 `main.py`、`protocol.py`、composition、ROS adapter、client.ts 的接线按 T6→T7→T8 串行，不能同文件抢写。

每个任务的文件白名单包含该任务列出的新测试；额外改动先回负责人说明具体依赖。每项提交前负责人读实际 diff、运行该项命令，拒绝仅凭报告/测试绿灯验收。以下提交步骤由负责人审后执行，不授权本计划 worker 提交。

## 冻结的接口、时钟和状态归属

T2 创建 `motion/contracts.py`，不可变 dataclass；已有 `models.py:Velocity(vx,wz)` 和 `BodyTwist(vx,vy,wz)` 继续使用。

```python
ManualIntent(authority_id: str, gesture: int, sequence: int,
             linear: float, turn: float, gear: Literal["low", "high"])
MotionTarget(velocity: Velocity, source: Literal["manual", "navigation", "idle"],
             source_id: str, deadline_steady: float)
BodyMotion(average: BodyTwist | None, upper: BodyTwist | None, interval_start_ros: float,
           interval_end_ros: float, computed_steady: float, valid_until_steady: float,
           source_epoch: str, qualified: bool, reason: str)
ResponseBounds(accel: float, decel: float, jerk: float, yaw_accel: float,
               yaw_decel: float, yaw_jerk: float, actuator_lag: float,
               valid_until_steady: float, profile_id: str)
ConstraintSnapshot(target: MotionTarget, motion: BodyMotion, mode: SourceMode,
                   response: ResponseBounds, footprint: Footprint, perception: Perception,
                   pose: Pose2D, frame_id: str, scope_id: str, config_version: str,
                   envelope: RiskEnvelope | None)
MotionPermit(velocities: tuple[Velocity, ...], created_steady: float,
             deadline_steady: float, source_identity: tuple[str, ...])
ConstraintResult(permit: MotionPermit | None, brake: BrakeRequest | None,
                 certificates: tuple[SweepCertificate, ...])
ControlOutput(velocity: Velocity, mode: str, reason: str, limited: bool)
MotionEvent(kind: str, source_id: str, revision: int, sequence: int)
MotionState(mode: str, reason: str, task_id: str | None, estop_revision: int,
            fault_revision: int, stationary: bool, allowed_actions: tuple[str, ...])
TaskEvent(kind: str, revision: int, completed_waypoint: int | None)
```

`BodyMotion.upper` 可为 None，qualified 仅指参考质量，不是运动许可。有依据的 upper 才能被当作模式适用域内的当前分量绝对上界。T3a 的 `MotionAssessment` 组合 source mode、原始质量/协方差/年龄、uncertainty_supported、内部目标 cap 与恢复阶段，避免向 BodyMotion 重复写入同一组状态；T5 负责 cap 恢复平滑。轮速退化不能声称具备独立真值上界。`StampedObservation` 沿用 `safety/motion_runtime.py`；多传感证据在 adapter 内合成不可变快照，未知扇区仍为未知。`MotionPermit` 许可离散的线/角速度组合及其停车轨迹；插值/新组合必须重新快速核验，不把两个轴各自限幅当安全证明。

- 车端 `time.monotonic()` 管手动期限、控制 dt、计算预算、许可期限、进程 watchdog；ROS stamp 管观测区间与对齐；显式桥接同时记录接收 steady/ROS，不拿浏览器时钟换算资格。ROS 暂停/倒退使估计失效，但 steady 制动/故障监督继续；控制 dt 跳变超过 T1 上界时不补算长轨迹。
- T3a `AdmissionOwner.ingest/assess` 拥有源时效、epoch与去重恢复计数，纯 `select_reference` 选择参考而非统计融合；T3b 已实现 `snapshot(after_cursor, now_ros, now_steady)` → `TemporalResolver.consume/resolve(a,b,snapshot)` → `NumericalFusion.update(window,snapshot,resolver)` 纯离线链，保留分量mask、原始keys与时间支持；统计CI不自动生成当前上界，后续有适用响应模型后才统一执行区间/年龄/协方差到当前界转换；T4 不再重复加年龄；响应模型不再膨胀速度。
- T4a实际接口：`ConstraintEngine.tighten(previous, snapshot, now)` 只收紧；`refine(snapshot, now)`在worker计算；`admit(result, latest, now)`复核最新几何/模式/来源/时效。结果均为ConstraintResult，制动请求明确stop_verified，不把输出零作为停车证明。`validate_output(result, latest, proposed, now)`对较小轨迹步进行独立有界几何证明，离散许可不能自动许可插值。`ConstraintScheduler.update/poll/stop`独占调度，普通scan更新不按generation撤销，来源/配置变化和cancel fence迟到放宽。
- T5 的 `MotionController.tick(target: MotionTarget, motion: BodyMotion, permit: MotionPermit, *, now_steady: float) -> ControlOutput` 持有轨迹、限幅 PI/抗饱和和静止判据。`MotionStateMachine.reduce(event: MotionEvent, *, now_steady: float) -> MotionState` 集中转换；事件 kind 在 T2 固定为 manual_start/manual_release/manual_expired/task_start/task_pause/task_resume/task_done/obstacle/obstacle_clear/feedback_lost/feedback_healthy/execution_fault/fault_recover/software_estop/estop_reset/hardware_estop/hardware_release；实现使用 enum/Literal，T5 实现转换表；节点不再各自推导模式。
- 运动状态：idle / manual / task / braking / obstacle_hold / feedback_stop / execution_fault / estop；任务另有 accepted / running / paused_takeover / paused_fault / succeeded / failed / canceled。终态不可恢复；反馈恢复允许任务重规划继续，但 paused_takeover/paused_fault 必须明确 resume；急停复位不清执行故障且暂停任务等待继续。显式取消、急停、接管优先于任务工作队列，单线程 reducer 排序，事件序号决定并发结果。
- T6 `TaskLifecycle.accept(request_id: str, fingerprint: str, payload: dict[str, object]) -> TaskReceipt`、`query(task_id: str) -> TaskReceipt`、`event(task_id: str, event: TaskEvent) -> TaskReceipt` 为 async；receipt 含 task_id、request_id、status、revision、completed_waypoints、remaining_goals、reason。TaskEvent.kind 固定为 accepted/start/waypoint_done/takeover/feedback_lost/feedback_recovered/execution_fault/recover/estop/resume/cancel/success/failure；SettingsSnapshot/SettingsResult/TaskReceipt/LogPage 在各自 owner 文件声明命名模型并由 protocol 引用。稳定 request_id 相同载荷回原结果，不同载荷冲突；journal 事务先持久接受再启动。
- T7 `VehicleSettingsService.get() -> SettingsSnapshot`、`apply(request: SettingsRequest, state: MotionState) -> SettingsResult` 为 async；请求含 request_id、expected_version、group（driving/logging）、values；响应含 version、effective_values、capabilities、reason。`project_vehicle_state(...) -> VehicleState` 是唯一状态投影，含测得 average 及其来源/年龄、mode/reason、task receipt、急停/故障版本、允许动作；UI 不从输出回执冒充车速。
- T8 `VehicleLogStore.query(query: LogQuery) -> LogPage` 为 async；cursor 是不透明车端游标，page 含 entries/next_cursor/gap。日志 event 含 vehicle boot_id、event_sequence、source_time、received_time、task_id/request_id、level/module/summary/detail，UTC 只用于显示/保留，车端 steady 管临时调试恢复，重启时根据持久 UTC 截止时间保守恢复常规级别。

### 手动输送机制（T2 决策，不保留延迟挑战）

沿用认证 WebSocket 与现有 control.acquire/release，不新增 WebRTC。acquire 由车端生成不可预测 authority_id，绑定车辆、连接代次、控制会话；本地短活性期限由 T1 冻结；drive.intent 只携 authority_id、递增 gesture/sequence、归一化力度和档位。浏览器与云各保留一个未发送最新样本、最多一个未确认样本，ROS ingress 只留最新样本；等待期间更新覆盖，不追加队列；紧急事件独立优先槽。一次活性断流、释放、故障或连接换代永久关闭该 authority_id，迟到样本即使 seq 更大也不能刷新/重开它；重连清空待发送动作。

acquire 回执本身不启动运动；需回执后新的用户松杆→按下边沿，不能把正在按住的动作或迟到 acquire 回复自动转成新 gesture。失效后必须重新 acquire 再有新边沿；旧 acquire request_id 的重试幂等回原已关闭授权，不能生成新授权。每段本地排队在转发前检查本段接收 steady 期限并带原接收时间到同机 IPC，绝不在出队时重置年龄。这保证断流后积压不复活；没有声称能测量首次样本的任意单向网络延迟，也不重新引入每样本往返挑战/浏览器时钟资格。TCP 已进入发送缓冲的样本在车端活性过期后由封闭 authority 拒绝。

## Task 1: 默认失败取证、校准与验收冻结（最初批次，现与 T2a/T3a/T3b/T4a/T5a 纯模块并行）

**Files:** Create `tools/vehicle_motion_capture.py`、`tools/vehicle_motion_analyze.py`、`tests/unit/test_vehicle_motion_analyze.py`、`tests/fixtures/vehicle_motion_trace.json`、`tools/vehicle_motion_calibrate.py`、`tests/unit/test_vehicle_motion_calibrate.py`、`docs/testing/2026-11-12-vehicle-motion-baseline.md`、`config/vehicle-motion-acceptance.yaml`。不改产品控制、模型、旧测试、旧文档或当前脏文件。

**Interfaces:** 消费现有 `/motion/scan`、`/motion/evidence`、`/wheel/odom`、`/imu/data`、`/cmd_vel`、`/safety/status`、`/ground_truth/odom`、`/clock` 和 IPC state/task status；产出带版本/配置摘要的 JSONL、rosbag、冻结 acceptance YAML，T3b/T4/T5 的生产定量能力声明只能消费其已验证响应/误差界；获批隔离仿真候选按前述范围运行，不得自行调通过线或将 assumption 称为已测；T3a/T3b/T4a/T5a 可显式注入合成配置测试纯逻辑、数值算法与方向约束，缺真实上界必须保留 None。

- [x] 写 analyzer 失败测试：固定 fixture 包含 ROS 停顿、258 ms 对齐等待、wheel 有速度而 truth/scan 静止、慢 worker 永远过期；断言分别报告 `clock_pause`、`alignment_wait`、`wheel_body_disagreement`、`permit_starvation`，且缺 truth 不输出“物理停车通过”。
- [x] 运行 `.venv/bin/python -m pytest tests/unit/test_vehicle_motion_analyze.py -q`，预期新 analyzer 未实现而失败；实现 `analyze_trace(path: Path) -> dict[str, object]`，保留原始样本，输出区间/接收时间/来源/原始状态与误差预算；同命令应通过。
- [x] capture 实现 CLI `--socket PATH --seconds N --output PATH`（只读，退出码缺必需流为非零），analyze 实现 `--input PATH --output PATH`；先在 fixture 验证确定性，不启动仿真。运动刺激由现有 ROS 测试施加，capture 不隐含发送速度。calibrate CLI 固定为 `--plan PATH --seconds N --output PATH`：plan 明列隔离 ROS 域、机器人 ID、场地边界、唯一执行器目标话题、速度/加减速段及硬退出期限；先写 `test_calibration_refuses_shared_domain_or_unbounded_plan`，不符合即拒绝，无默认运行行为。校准只在专用隔离测试图中驱动执行器，不能向运行产品图注入第二个 /cmd_vel 发布者；原始测试参数及每段刺激随输出保存。
- [x] 在下述 Linux 环境按“基线命令”运行默认手动/导航测试与同步采集；保留真实退出码，预期旧失败是待复现而非预定结论。复现不了也记录版本、原因与负对照，不编造 258 ms 是唯一根因。
- [ ] 默认 cap 下取得观测/调度基线；如需 5 km/h 开环响应，先由负责人审 `vehicle_motion_calibrate.py` 及其 plan 的有限刺激/场地/唯一发布者，再单独有界运行，不改产品运动链来临时绕过 gate。先量轮速上限是否执行、正反/转弯响应、激光速度界/可观测性、不同摩擦制动/执行滞后；失败场景不用于宣称通过。
- [ ] 在实际控制资格使用前冻结下表适用的待测数值及依据，由负责人实际读原始数据/配置 diff 后签字日期和 SHA；若无可用环境/数据，本任务报告取证阻塞，T2a/T3a/T3b/T4a/T5a 仍可实现获批不接输出的纯模块，T2/T3b–T5 可继续获批隔离仿真接线，但未冻结参数不得形成生产能力或实车放行。完成后仅提交白名单文件，消息 `test: freeze vehicle motion baseline and acceptance bounds`。

T1 工具/环境取证子阶段已实现，详见 `docs/testing/2026-11-12-vehicle-motion-baseline.md`；上述勾选仅指工具本地验证，不表示已采集 ROS 数据、校准或冻结。capture 原样保存所有收到的 ROS 消息和 IPC 快照并报告缺流；当前数值对齐等待/过期饥饿计数未导出，不能凭日志文本诊断。新核心的性能在 T4/T5 实测，不把 T1 扩大为证明未实现模块；T1 冻结数据边界/预算分配后，后续实现必须满足它们。

### 基线命令与环境（本计划未执行启动）

本机已核验 Darwin，`ros2`、`colcon`、`gz` 不在 PATH；`.venv` 与 `web/node_modules` 存在。`.venv/bin/python tools/doctor.py config/simulation.yaml` 返回静态 passed、所有实时项 not_run；相关现有测试 136 passed（命令见自审），不表示仿真通过。后续在 `docs/testing/linux-vm-layout.md` 指定 `/home/toby/senior-audit-20261031/` 的 Linux Humble/Fortress/Xvfb/systemd 用户会话执行；本子阶段通过 prlctl 只读确认 Ubuntu 22.04 VM 已运行且 Humble/Fortress 可用，私有 profile 存在、adapter 未运行；约定统一根当前不存在，源码/安装身份尚未冻结，具体事实和运行前置见 baseline 文档，不凭历史目录宣称验收可直接执行。

先只读：`command -v ros2 colcon ign xvfb-run`，`python -m senior_control.cli doctor --profile "$VEHICLE_PROFILE"`；`VEHICLE_PROFILE` 必须是现场私有 simulation.json（不是 `config/simulation.yaml`），credentials/db/socket/estop/ROS 域不能杜撰。冻结源码放该根目录的 `vehicle-motion-<sha>/src`，构建 `build/install/log` 和证据均在该目录；对源/安装包和 web 构建算 SHA，记录依赖/CPU/实时因子与未提交补丁摘要。现场私有 profile 的 ros_setup 必须指向本次冻结 install/setup.bash，运行 Python 也须加载同一份冻结 src；先核对实际进程命令/包路径，不能编译新版却启动旧版。用 `.vm-test.lock` 的 `flock` 对整个构建/仿真验收串行加锁。

```bash
# 在冻结 src 下，已加载 /opt/ros/humble/setup.bash 与现存 Python 环境；先持有 VM 锁。
colcon build --base-paths ros2/src --build-base ../build --install-base ../install
source ../install/setup.bash
export ROS_HOME="$RUN_ROOT/ros-home" ROS_LOG_DIR="$RUN_ROOT/ros-home/log"
# RUN_ROOT 必须由测试执行者设置为上述统一根下本次绝对目录；域选现场未占用值。
python -m senior_control.cli start --profile "$VEHICLE_PROFILE" --headless true
export SENIOR_SIM_NAV_SOCKET="$VEHICLE_SOCKET"
python tools/vehicle_motion_capture.py --socket "$VEHICLE_SOCKET" --seconds 180 --output "$RUN_ROOT/baseline.jsonl"
# 采集与以下测试在同一受控运行的独立终端同步执行，不能先采完再运动。
python -m pytest tests/ros/test_simulation_manual.py tests/ros/test_simulation_navigation.py -vv --junitxml="$RUN_ROOT/baseline.xml"
python tools/vehicle_motion_analyze.py --input "$RUN_ROOT/baseline.jsonl" --output "$RUN_ROOT/baseline-metrics.json"
python -m senior_control.cli stop --profile "$VEHICLE_PROFILE"
```

执行者先从 profile 读取实际 socket/ros_domain，`ROS_DOMAIN_ID` 与启动一致；采集对齐测试开始/结束事件。测试进程外层设 180 s watchdog，finally/EXIT trap 执行 stop；采集超时不等于测试通过，必要时分场景重跑并保留各次证据。额外原始录制：`timeout -s INT 180 ros2 bag record -o "$RUN_ROOT/rosbag" /clock /scan /motion/scan /motion/evidence /wheel/odom /imu/data /cmd_vel /safety/status /ground_truth/odom`；退出后核对本次进程组、域与唯一 `/cmd_vel` 发布者。已核对 manual 与 navigation 两个测试均读取 `SENIOR_SIM_NAV_SOCKET`；不得因变量错误 skip 也算通过。

### 阈值冻结表（适用定量能力声明与生产放行门；纯模块及获批隔离仿真接线例外）

| 字段 | 依据/冻结规则与验收用途 |
| --- | --- |
| speed low/high、几何、轮限 | 固定 5/9、25/18；实测声明 20 rad/s 与 r=.076 的有效性；理论高档每轮约 18.27 rad/s，仅直行，不提高电机上限容纳组合；足迹各轴 ×1.05 |
| 测速误差、静止 v/w、窗口 | 静止与定速正/反/弧线数据分别给 interval-average 误差、采样/插值误差；取观测最大残差加测量分辨率构建界，并用独立留出数据验证。窗口至少覆盖完整激光区间与执行滞后；静止须连续窗口满足 v/w 界，不能凭命令零判断 |
| 高低档稳态误差/建立时间 | 目标固定上述 SI；高档真值稳态窗口中位速度必须达到 25/18 m/s（只允许 T1 证实的仿真数值分辨率误差，不能把下述跟踪误差界从达速要求中扣掉）。冻结 nominal 跟踪误差界及连续保持窗口，误差来自独立测量预算，不能用失败控制器误差反定容差；误差必须小于 `(20*.076-25/18)/2 ≈ .06556 m/s`，否则不足以区分目标与轮限、先解决测量。建立时间按标定加速度/jerk/滞后积分计算上界，随后修改控制不能放宽 |
| accel/decel/jerk/yaw、lag、滑移/堵转 | 隔离校准原始曲线分方向/速度/摩擦区间，保守制动下界和滞后上界；正常空旷启动/转弯作为负对照，真滑移/锁轮/跟踪失败作为正刺激，检测窗口需兼顾分离能力与停车预算；无分离能力不冻结故障门槛 |
| 快速路径周期/预算、异步放宽上界 | 测 VM 负载下最坏耗时、激光频率；预算为单周期扣 ROS 接线/发布最坏耗时，快路径必须在预算内。异步期限在 T1 按实测传感周期、端到端停车预算及调度余量固定分配，不能依赖尚未实现的 refine 才定标准；T4 对最大输入规模实测 refine+调度必须符合该期限，持续输入在此限内至少得到一次可用放宽；做不到缩算法/降速，不能延长旧许可 |
| 手动断流、传感/许可期限、恢复 | 手动初始沿用现有 0.3 s 待测，按端到端停车预算核验后冻结；5 份且跨度≥0.5 s 的恢复条件保留。传感/许可期限由独立运动上界与制动可行性计算，不能只改 .8 s/.25 s 掩盖停顿 |
| 停车距离/场地与导航阈值 | 依据带 lag/jerk 的停止轨迹积分加各自测量/几何预算；所有真值轨迹不得碰撞扩张足迹。最小场地大于最高速度最坏停止范围；导航 goal/progress 容差读 Nav2 实际参数并由传感误差/地图分辨率支持，不能迁移旧 stall 误锁存 |
| 日志负载/资源与控制优先 | T8 初值固定容量，T1 记录无日志控制最坏调度；满日志配额和持续超额输入下，控制接收→下一 tick/输出仍≤冻结控制周期预算，急停从车端接收起至覆盖输出≤一个控制周期；不承诺公网 RTT 上界 |

此表允许测量待填，空值不阻塞获批纯模块与显式 simulation-only/unvalidated 隔离接线、正常调试；不得用空值或合成替代值宣称 T3b/T4/T5 定量能力已验证或放行实车。冻结后任何不满足均记失败；若模型/场地/误差界改变，先新版本复核冻结，旧验收作废，再改控制，禁止看完结果后改标准。

## Task 2: 意图契约和授权，删除延迟资格

**Files:** Create `motion/__init__.py`、`motion/contracts.py`、`motion/manual_session.py`；Modify `protocol.py`、`errors.py`、`safety/arbiter.py`、`agent/{websocket,connection,main}.py`、`cloud/relay.py`、ROS `agent_adapter.py`/`command_node.py`、`web/src/api/{protocol,client}.ts`；Test 新 `tests/unit/test_manual_session.py`、现有 `tests/unit/test_{protocol,arbiter,drive_timing}.py`、`tests/integration/test_{agent,cloud,safety_intent_store}.py` 及 Create `web/src/api/client.test.ts`（当前没有独立 client/protocol 测试文件，不假称已存在）。

**Interfaces:** 消费 T1 冻结活性参数；产出上述 contracts 与 `ManualSession.accept(intent: ManualIntent, *, now_steady: float) -> bool`、`expire(*, now_steady: float) -> None`、`close() -> None`，以及 drive.intent wire schema，所有后续任务只用此版本。control.acquire 返回 authority_id/关闭状态；RTT 独立 ping/pong，仅展示。

- [ ] 写失败测试 `test_expired_authority_rejects_larger_sequence`、`test_old_acquire_reply_cannot_restart_held_input`、`test_queue_keeps_only_latest_sample`：过期后 100 个递增旧样本仍拒绝、无回执后新边沿不发运动、队列≤1；保留跨车/来源代次/急停优先回归。`pytest tests/unit/test_manual_session.py -q` 预期未实现失败。
- [ ] 实现 ManualSession 与归一化 [-1,1]、非有限数拒绝，按“手动输送机制”贯穿现有链路；能力不匹配明确拒绝旧 velocity 协议，不静默同时支持两套。节点暂只做转换，T5 接完整轨迹。
- [ ] 盘点 `rg -n 'drive_timing|drive.clock|drive.challenge|ClockReceiver|DelayBudget|command_clock_valid|sampled_at|server_issued_at' src ros2 web/src tests`；逐项标注 RTT/手动/感知/来源/幂等归属，删除 `drive_timing.py` 及无剩余消费者的 timing 模型/分支和相应旧测试，保留独立 RTT 与感知时钟测试；不得改成永远 true。迁移 agent fakes，避免假接口继续掩盖旧依赖。
- [ ] 运行 `.venv/bin/python -m pytest tests/unit/test_manual_session.py tests/unit/test_protocol.py tests/unit/test_arbiter.py tests/integration/test_agent.py tests/integration/test_cloud.py tests/integration/test_safety_intent_store.py -q`；`cd web && npm run typecheck && npm test -- --run`。预期通过；上述 rg 只允许文档/删除资格回归的显式断言，不剩产品调用。
- [ ] 负责人审 contracts/跨链路实际 diff 后提交白名单，`refactor: replace drive timing eligibility with fenced manual intent`。

## Task 3a: 获批多源准入、适配与参考模式纯模块

**Files:** Create `motion/{observations,source_admission,source_modes,estimate_adapters}.py`、`tests/unit/test_motion_sources.py`；必要修改 `motion/contracts.py` 和 `test_motion_contracts.py`，不改 manual session/authority，不接 ROS/cloud/UI/T4/T5 或旧 safety 输出。

**Interfaces:** `AdmissionOwner(config).ingest(observation, *, now_ros, now_steady, heading=None) -> AdmissionDecision`；`assess(*, now_ros, now_steady) -> MotionAssessment`。所有源/恢复/cap/时钟配置必填注入；Observation 保留来源、frame、ROS区间、原始 steady 接收、epoch/seq、质量、covariance/basis/correlation。GPS ENU 转 body 依显式合格 Heading，姿态相关协方差未定义时不制造 body covariance。IMU 只接 body yaw rate，不接无重力/姿态/零偏补偿的车速积分。

- [x] 独立源准入、未来/过期/非有限/协方差/重放/重启/双时钟拒绝；连续样本及跨度恢复，不在重复 tick 累计。
- [x] 可解释 laser_reference、gps_imu_reference、wheel_imu_limited、recovering/unavailable；当前按明确优先级选参考，不宣称完成数值融合。无可信真速上界时 upper=None、uncertainty_supported=False。内部 cap 仅为目标约束，T5 负责平滑，不在这里授予运动许可。
- [x] 复用 MotionEstimate 的完整 body_twist，要求调用方明确提供速度协方差；不把位移对角协方差除 dt² 冒充完整 twist 模型。不同区间的 GPS/IMU 不拼成一个区间平均。
- [x] 本地 synthetic 单测与所改模块严格 Mypy/Ruff；这些不冻结 cap、不替代仿真精度/实车/5kmh。详见本轮 T3a 报告。

## Task 3b: 不接输出的异步数值融合（本轮）及后续生产界

**本轮 Files:** 新增 `motion/{sample_types,admission_events,kinematics,temporal,fusion}.py`；必要扩展 `motion/{observations,source_admission,source_modes}.py`。新增 `tests/unit/test_motion_{temporal,fusion,fusion_replay}.py`、`tools/motion_{fusion_replay,replay_records,fusion_synthetic}.py`。既有 manual/authority、safety/ROS、matcher/校准工具不改。

**本轮 Interfaces:** `PointObservation` 是点；`RateIntervalObservation` 兼容原 `Observation`；`DeltaObservation` 保留SE2位移、协方差及外参。AdmissionOwner 独占资格/恢复，以有界事件/游标提供全部接纳历史；TemporalResolver 仅处理已接纳数据的有限历史/因果ZOH/覆盖；NumericalFusion 对有统计模型依据的同窗分量执行固定w=.5 CI。FusionSnapshot 的完整average与带mask分量、原keys/协方差、source_health年龄/恢复状态、目标cap和CurrentReference分开。`upper=None`；无加速度模型不把当前参考改戳成当前估计。T4感知许可、T5限速恢复平滑未接。

- [x] 数据运行前登记解析容差、异步覆盖/缺口/去重/模式行为；无物理精度通过线，不看结果反调标准。
- [x] RED/GREEN：点准入、突发事件完整、30/50Hz异步、源换代/中断/时钟回退、有限缓存、GPS航向与SE2转弯/外参；未声明非完整运动假设时不产生轮式SE2均值。
- [x] 稳定数值CI、零方差不加epsilon、未知相关不虚假缩小、分量mask和d*diag统计包络（非停车保证）；保留原交叉信息；不递归重复融合/重复计健康。
- [x] 实际离线消费 observable-1/2、旧turn/弱走廊；真实无可信统计模型不进CI，保留退化参考/全部缺流、拒绝与同窗误差。合成GPS/统计模型运行与真实记录分列。
- [ ] 负责人亲读本轮实际diff/原始输出后接受T3b纯离线成果；尚不提交T3b，不宣称5km/h闭环。详见 `docs/testing/2026-11-13-vehicle-motion-t3b-offline.md`。

**后续生产界/接线（不属于本轮）:** 适用T1边界冻结后，新增 `motion/estimation.py`、`motion/response.py` 和 bounds 测试；才评审旧 `safety/motion_estimator.py`、`safety/velocity_uncertainty.py`、ROS `motion_fusion.py` 的迁移。`ResponseModel.bounds` 只提供已验证固定参数，禁止在线放宽；不能让本轮合成cap或统计n-sigma进入停车保证。保留原后续验证步骤：

- [ ] 写 `test_age_is_accounted_once`（同观测下游再计算不重复增大）、`test_missing_sources_selects_supported_mode_without_wait`、`test_laser_gps_absent_with_perception_allows_limited_wheel`、`test_source_recovery_has_no_speed_jump`（T5轨迹消费集成，T3只给目标约束）、`test_ros_pause_never_refreshes_old_motion`、`test_wheel_spin_is_not_body_speed`；运行 `.venv/bin/python -m pytest tests/unit/test_motion_bounds.py tests/unit/test_response_bounds.py -q`，预期缺实现失败。
- [ ] 实现上述两个接口；辅助量等待有界，不堵控制周期，各源原始区间、质量/协方差依据保留供诊断；GPS 缺失正常处理，轮速退化与不可用分别声明。参考 qualified 不代替界支持；未知/过期 upper 不能标 uncertainty_supported，也不能单独授予T4许可；获批轮速退化可用独立验证、模式/场地适用且当前有效的RiskEnvelope证明条件几何，不能以轮速或统计P替代它。最后可信上界仅在冻结故障监督范围内制动，绝不重新盖时间戳；超过边界升级执行故障停机。
- [ ] 按误差账本删除下游重复年龄膨胀入口，保留有效横向/旋转不确定性；数据源/协方差/区间对齐回归应继续通过。运行 `.venv/bin/python -m pytest tests/unit/test_motion_bounds.py tests/unit/test_response_bounds.py tests/unit/test_motion_estimator.py -q`；Linux 加 `python -m pytest tests/ros/test_velocity_uncertainty.py tests/ros/test_scan_exact_stamp.py -q`，不得 skip 算通过。
- [ ] 负责人核对每个上界项与 frozen 数据后提交 `feat: derive mode-aware body motion from qualified sources`。

## Task 4: 方向动态约束与不饥饿异步放宽

**T4a 获批限定例外（2026-11-13）：** T1尚未冻结时，实施不接ROS/车辆输出的约束与调度纯模块，显式注入synthetic响应、模式/场地风险包络、离散候选与计算预算；不以未验证统计P生成确定性真速界。优先复用BodyMotion/ResponseBounds，部分观测average允许None；独立风险包络补充模型能力，不给轮速换名。当前仅只读复用geometry，不改旧guard/输出；生产接线和物理停车资格仍需适用实测。

**Files:** Create `motion/constraints.py`、`motion/constraint_worker.py`、`tests/unit/test_directional_constraints.py`、`tests/unit/test_constraint_scheduling.py`；T4a仅只读复用 `safety/geometry.py`，不修改公共safety模块；后续生产adapter保留 depth/self-filter 能力；Test `tests/unit/test_guard.py`、`test_guard_hit_intersections.py`、`test_async_guard.py`。

**Interfaces:** 消费 T3 BodyMotion/ResponseBounds、T2 snapshot/permit；产出 tighten/refine/admit 三个既定接口。慢任务最多一个 in-flight、一个 latest pending；不可变快照含来源/配置/危险 revision，worker 不改状态机。

- [ ] 写 `test_front_blocked_reverse_requires_rear_observed`、`test_turning_sweeps_outer_corner_and_current_inertia`、`test_new_hazard_vetoes_inflight_relaxation`、`test_continuous_input_produces_permit_within_frozen_bound`、`test_unknown_fast_path_brakes`；对不同距离固定场景断言许可速度单调不增、未知后方无后退许可。运行 `.venv/bin/python -m pytest tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py -q`，预期未实现失败。
- [ ] 实现有界候选组合/停车扫掠，快速路径只收紧并使用预处理有界障碍摘要，不能在控制线程做全扫描复杂几何；异步放宽结果对最新危险核验，正常高频观测不因无关 generation 一律作废。过期、超预算、新方向超许可进入制动，控制 tick 始终能返回。
- [ ] 用确定性 fake clock/可控 worker 复现持续 10 Hz scan+慢 refine，再插入新障碍；断言冻结时间内有可用许可且新危险后无旧放宽。运行上述测试与 `.venv/bin/python -m pytest tests/unit/test_guard.py tests/unit/test_guard_hit_intersections.py tests/unit/test_async_guard.py -q`；Linux 捕获实际快路径最坏耗时，超过 T1 预算不放行。
- [x] T4a五代码测试文件经负责人亲审并独立330回归后提交 `c569658da88eeb111ee1e907a9c2fd3bf4e50332`；T4生产接线、物理模型及Linux资格仍未完成。

## Task 5: 轨迹、外环、恢复状态机与 ROS 唯一输出

**T5a获批限定例外（2026-11-13）：** 在适用实测边界未冻结时先实现不接ROS/执行器的trajectory/controller/state_machine，显式注入synthetic响应、cap、PI、轮限、时钟和plant参数；不要求29项全齐才开发纯模块。后续已批准显式隔离 candidate 的真实仿真输出调试；正常驾驶验收与生产能力声明仍需适用模型/感知/执行器及 Linux 验证。T4a已获审五代码测试文件独立提交`c569658da88eeb111ee1e907a9c2fd3bf4e50332`，其330回归通过不表示物理安全包络已通过。

**T5a实际接口：** Trajectory.propose/commit事务式推进v/a，Controller.tick(target,context,permission,body,wheel,now,estop=...)返回ControlDecision；None表示无可执行已验证候选而非安全零。状态机reduce消费可信单序列MotionEvent，observe接纳新的健康/静止证据；软件ESTOP、硬件ESTOP、执行故障分别持有状态。已接受任务feedback恢复只请求重规划，接管暂停必须显式resume；不造任务runner或复活旧authority。T5a报告及原始CSV见`docs/testing/2026-11-13-vehicle-motion-t5a-core.md`。

**Files:** T5a Create `motion/{trajectory,controller,state_machine}.py`、`tests/unit/test_motion_trajectory.py`、`tests/unit/test_motion_controller.py`、`tests/unit/test_motion_state_machine.py`；后续生产接线才创建 `tests/ros/test_motion_controller_integration.py` 并 Modify `safety/recovery.py`、ROS `safety_node.py`/`command_node.py`/`agent_adapter.py`/`motion_fusion.py`、`ros2/src/senior_robot/config/{simulation_motion.yaml,simulation_motion_flat_lateral.yaml,simulation_motion_rear_mask.yaml,nav2.yaml}`、`ros2/src/senior_robot/launch/{safety_motion,simulation_motion}.launch.py`。必要模型一致性仅改 `ros2/src/senior_sim/config/controllers.yaml`/`urdf/senior.urdf.xacro`，依据 T1 数据，不能提高电机能力造通过。

**Interfaces:** 消费 T2–T4；产出 MotionController.tick、MotionStateMachine.reduce，及 state.get 运动/故障原因/版本/允许动作，最终 `/cmd_vel` 仍由 SafetyGate 一个发布者输出，FinalCommand 的本机 steady 来源 fence 保留。

- [ ] 写 `test_zero_or_expiry_keeps_braking_ticks`、`test_reverse_waits_for_measured_stillness`、`test_integrator_does_not_wind_up_on_slip_or_block`、`test_feedback_recovery_needs_five_samples_and_half_second`、`test_fault_reset_is_distinct_from_estop_reset`；加入实体急停阻止软件复位、旧 reset revision 拒绝、复位不恢复旧手动/任务测试。运行 `.venv/bin/python -m pytest tests/unit/test_motion_controller.py tests/unit/test_motion_state_machine.py -q`，预期失败。
- [ ] 实现 trajectory 的 speed/accel/jerk 与限幅 P/PI（T1 响应决定是否用 I，首版不加 D）；正反跨零由状态机把关，饱和/停车/模式切换清理积分，轮限同比缩放保持曲率，再作组合扫掠最终核验。舒适减速和安全制动用不同已验证界；无默认负目标刹车。
- [ ] 用 steady timer 驱动核心；删除 SafetyGate 零输入/过期就退出闭环的旧分支及硬编码 0.3 资格，替换为冻结 profile 能力/一致性检查。fence/拓扑/进程 watchdog 仍保留；唯一最终快核验不能调慢 refine。缺反馈时在已验证范围制动监督，边界耗尽强制故障停机。
- [ ] 接独立 software estop reset 与 execution fault recover API，版本/权限/静止/健康条件必须车端验证；故障恢复后任务暂停待继续。T6 接任务事件，T9 接用户入口。
- [ ] 运行 `.venv/bin/python -m pytest tests/unit/test_motion_controller.py tests/unit/test_motion_state_machine.py tests/unit/test_arbiter_recovery.py tests/integration/test_estop_recovery.py -q`；Linux `python -m pytest tests/ros/test_motion_controller_integration.py tests/ros/test_final_command_dds.py tests/ros/test_motion_watchdog_latch.py tests/ros/test_invalid_source_receipt.py tests/ros/test_future_source_clock.py -q`；再用 T1 同采集命令取得实际启动/停车曲线，不以发布零代替停止。
- [ ] 负责人逐项审单一职责/旧代码删除和实测曲线后提交 `feat: close vehicle motion loop and separate recovery states`。

## Task 6: 车端任务所有权、接管暂停与 Nav2 重规划

**Files:** Create `tasks/lifecycle.py`、`tests/integration/test_task_ownership.py`；Modify `tasks/{models,runner,runtime,executor,store}.py`、`navigation.py`、`agent/{journal,direct_navigation,ports,main,composition}.py`、`safety/arbiter.py`、ROS `navigation_adapter.py`/`dedicated_controller.py`/`command_node.py`/`agent_adapter.py`；Test 现有 `tests/integration/test_task_runner.py`、`tests/ros/test_simulation_navigation.py`。

**Interfaces:** 消费 T2/T5 控制源 fence 和状态事件；产出 TaskLifecycle/TaskReceipt，operation.query/status 重连查询；Nav2 给本地新鲜 MotionTarget，不给远端手动 TTL，不绕过 T4/T5。

- [ ] 写 `test_accepted_task_continues_after_link_epoch_changes`、`test_unknown_submit_queries_without_reexecution`、`test_takeover_cancels_old_velocity_and_requires_resume`、`test_completion_wins_over_late_resume`、`test_offline_completion_reconnect_has_same_id`：重复 request 同载荷一次执行、异载荷冲突，断网仍 reconcile；完成动作数不增加。运行 `.venv/bin/python -m pytest tests/integration/test_task_ownership.py -q` 预期旧行为失败。
- [ ] 将已接受任务和新任务 admission、定时触发分别判定；移除 runtime 的连接换代取消已接受执行和 TaskRunner 对 active 的 offline pause，保留离线不新启定时任务、重启不自动续跑。TaskStore/journal 原子记录身份/进度；所有非手动操作（含配置/地图）超时为未知可查询，不借这次扩大并发/重启范围。
- [ ] 手动接管先撤 Nav2 source fence 和控制器旧输出再给予手动；保留未完成目标/进度，松杆/断流/重连仍 paused_takeover；resume 在静止且健康后由当前位姿/地图重规划，不重复完成航点。反馈临时恢复可继续原任务重规划；执行故障恢复、急停复位保留暂停。
- [ ] 导航普通遇障只收紧轨迹并让 Nav2 重规划；无路才等待/失败，本地速度过期仍停车。迁移 `test_gazebo_route_order_and_authority_loss` 的断网取消预期为继续，同保留来源撤销/本地样本失效回归。
- [ ] 运行 `.venv/bin/python -m pytest tests/integration/test_task_ownership.py tests/integration/test_task_runner.py tests/e2e/test_tasks.py -q`；Linux `SENIOR_SIM_NAV_SOCKET="$VEHICLE_SOCKET" python -m pytest tests/ros/test_simulation_navigation.py -vv`，需实际执行无 skip。
- [ ] 负责人审 task/connection 所有权与持久竞态 diff 后提交 `feat: retain accepted tasks offline and pause on takeover`。

## Task 7: 车端设置原子持久化与统一状态投影

**Files:** Create `settings/{__init__,models,store,service}.py`、`agent/vehicle_state.py`、`tests/integration/test_vehicle_settings.py`、`tests/unit/test_vehicle_state.py`；Modify `agent/{composition,dispatcher,snapshots,workspace,main}.py`、`protocol.py`、ROS `agent_adapter.py`；若需 schema 迁移只在 `storage/` 现有迁移体系按实际版本增加，不能把 cloud/agent 版本任意一起升（journal 已有幂等建表先例）。

**Interfaces:** 消费 T2/T3/T5 状态、T6 幂等 journal；产出 SettingsService、VehicleState。驾驶字段固定 `low_linear_mps, high_linear_mps, low_angular_rps, high_angular_rps, reverse_linear_mps`；logging 字段固定 `level, retention_seconds, debug_enabled, debug_restore_seconds`；版本分别按组递增，能力范围来自车端 profile，非任意 ROS 参数字典。

- [ ] 写失败测试：五项 NaN/越界/low>high 拒绝，倒车=min(档位线速,reverse)，驾驶运行中拒绝而日志成功；expected_version 冲突、落盘异常整组不变、重试幂等、损坏配置明确 unavailable、重启保持、restore-defaults 同校验；日志改动不发 MotionEvent。运行 `.venv/bin/python -m pytest tests/integration/test_vehicle_settings.py tests/unit/test_vehicle_state.py -q`，预期缺实现失败。
- [ ] 实现数据库事务+单一配置写锁；落盘完成后发布不可变新版本供控制下一 tick 读取，只允许静止且无 executing 运动任务应用 driving；task paused 可修改但不自动 resume。defaults/capabilities 与 T1 model profile 区分，不把未标定实车参数当仿真保证。
- [ ] 实现 projection：一个优先原因、独立连接状态与最后已知标签、真实 body speed、任务真实状态和允许动作、estop/fault 各自版本；确认前 software estop 为 pending，实体急停无网页解除动作。测试断网任务 running、障碍 hold 与故障/急停同时存在、旧快照版本拒绝回退。
- [ ] 同测试命令应通过；运行 `.venv/bin/python -m pytest tests/integration/test_agent_snapshots.py tests/integration/test_workspace_data.py -q`。负责人审事务失败与状态优先级后提交 `feat: persist bounded vehicle settings and project runtime state`。

## Task 8: 授权、有界车端日志与离线调试自动恢复

**Files:** Create `vehicle_logs/{__init__,models,store,service}.py`、`tests/integration/test_vehicle_logs.py`；Modify `agent/{composition,dispatcher,main,websocket}.py`、`cloud/{relay,app}.py`、`protocol.py`、`settings/service.py`。日志存储在车端数据库/轮转目录，只暴露结构化 query/download 接口。

**Interfaces:** 消费 T7 logging 设置/T6 request/task id；产出 VehicleLogStore.query 和日志订阅/下载，认证继续使用现有车端 Agent/云/用户车辆访问关系。

- [ ] 写 `test_cross_vehicle_log_query_denied`、`test_cursor_gap_after_retention`、`test_log_flood_does_not_delay_estop`、`test_debug_restores_original_level_without_browser`、`test_debug_restart_never_extends_deadline`；覆盖凭证/配对密钥脱敏和非法路径查询。运行 `.venv/bin/python -m pytest tests/integration/test_vehicle_logs.py -q` 预期失败。
- [ ] 实现单一有界写队列 1024 条、单条 detail≤4096 bytes、page≤200 条且≤256 KiB、单次下载≤2 MiB；内部落盘硬保护 64 MiB，容量不能放网页。保留时间允许 1 小时–30 天、默认 7 天；debug 恢复 60–3600 s、默认 300 s（资源产品初值，T1 负载核验，控制预算不随其改变）。超过窗口/容量必须 gap，不假称完整保留。
- [ ] 日志单独认证 WS 路由与独立 outbox（每连接≤4 pages），控制 receiver 不等待日志 IO/编码；跟随超额合并 gap，历史查询用车端资源配额拒绝/截断。logger 只能非阻塞 enqueue，磁盘慢/满丢日志并记 gap，不能堵控制；没有文件路径或 shell 参数入口。
- [ ] debug 首次启用持久保存原常规级别和 UTC 截止时间，在线用 steady deadline；重复幂等请求不续期，重启若时钟不可信立即恢复，网页离线不影响；保留/清理 worker 不改运动状态。
- [ ] 同测试命令应通过；在 Linux 采集满队列/持续超额日志时急停/取消/接管接收与输出 steady 时间，符合 T1 冻结周期；运行 `.venv/bin/python -m pytest tests/integration/test_backend_observability.py tests/integration/test_cloud.py -q`。负责人审权限、队列上界、控制通道优先后提交 `feat: add authorized bounded vehicle logs`。

## Task 9: UI 布局预览、归一化操纵、车辆设置与日志

**Files:** Create `web/src/features/devices/VehicleStateView.tsx`/`.module.css`、`web/src/features/settings/{VehicleSettings.tsx,Settings.module.css,settings.test.tsx}`、`web/src/features/logs/{VehicleLogs.tsx,Logs.module.css,logs.test.tsx}`、`web/src/api/vehicleState.ts`、`docs/reviews/2026-11-12-vehicle-layout-preview.html`；Modify `web/src/features/drive/{DriveView,EmergencyStop}.tsx`、`Drive.module.css`、`web/src/features/devices/{ControlSurface,RobotPanel}.tsx`、`web/src/api/{client,protocol,demo}.ts`、相关现有测试和 `web/e2e/control.spec.ts`。不得把局部样式加 theme.css。

**Interfaces:** 消费 T2 normalized intent、T6 task receipt、T7 projection/settings、T8 pages；只做展示/输入，不自己计算资格或车端故障恢复条件。

- [ ] 实施 UI 前创建沿用现有视觉体系的静态 HTML 布局预览，展示在线手动、断网任务继续、障碍受限、感知停车、急停 pending/锁存、settings 冲突、日志 gap 七种状态；通过 Orca browser 截图核对桌面/窄屏并交负责人审阅，预览审阅是实施门，不再请求用户确认已批准需求。
- [ ] 写失败测试：高/低档发送 [-1,1] 力度而非旧 0.5 m/s；主速度只有独立 body speed，轮速放诊断；断线显示最后已知/任务可能继续，恢复/接管状态只显示允许动作；急停未确认不能显示已生效。运行 `cd web && npm test -- --run` 预期新增测试失败。
- [ ] 实现 VehicleState selector/视图、高低档、独立 fault recover/estop reset；删除 timedMotion/challenge/timing UI 分支；保留 RTT。设置页五驾驶项和限定日志项、单位/默认/范围/回读、版本冲突/未知结果查询；运行中 driving 提示先停稳，logging 仍可编辑，无磁盘/其他参数输入。
- [ ] 实现日志过滤/跟随暂停/历史游标分页/有界下载、源时间/接收时间、关联操作定位；浏览器最多保留 1000 条，超额提示窗口裁剪/gap。日志面板打开/查询不能暂停任务。
- [ ] 运行 `cd web && npm run typecheck && npm test -- --run && npm run build`；连接 T10 有界测试栈后 `PLAYWRIGHT_BASE_URL="$WEB_TEST_URL" npm run test:e2e -- --project chromium`，以同版真实链路验证未知提交查询与无重复执行。负责人读实际组件/样式/state diff 后提交 `feat: unify vehicle driving settings status and logs UI`。

## Task 10: 同版端到端运动与导航验收、冲突测试迁移和文档

**Files:** Create `tests/ros/test_vehicle_motion_acceptance.py`、`tests/e2e/test_vehicle_task_offline.py`、`docs/testing/2026-11-12-vehicle-motion-results.md`；Modify `tests/ros/test_simulation_{manual,navigation}.py`、冲突的 `tests/e2e/test_drive.py`/`test_tasks.py`、`web/e2e/control.spec.ts`、`README.md`、`docs/protocol/{v1,http-ws,scheduling}.md`、`docs/operations/{simulation,hardware}.md`、`docs/testing/{matrix,real-robot-checklist}.md`。先与负责人处理这些文件的既有脏改归属，不覆盖；新验收场景必要时新增 `ros2/src/senior_sim/worlds/vehicle_motion_acceptance.sdf` 与专用 launch，不暗改老 fixture。

**Interfaces:** 消费全部实现、T1 冻结 YAML/采集器；产出同一 source/build/web SHA 的 rosbag/JSONL/截图/测试报告/物理曲线/需求矩阵。真值和轮速、激光平均值、当前上界与目标分别列出。

- [ ] 写矩阵失败测试并用未实现链路确认对应失败（缺环境记 blocked，不能 mock 代替）：两档稳态/启动/减速、方向切换、前堵后退/后方未知拒绝、转弯外角扫掠、滑移、感知与辅助量失效/恢复、慢计算持续输入+新障碍、软件/实体急停、执行故障恢复。真实轨迹满足冻结速度/停距/时间窗且扩张足迹不碰撞。
- [ ] 运行同版网页→云→Agent→ROS→Gazebo，验证手动断流制动后迟到 100 条不能复活；三点导航/绕障/无路停车、导航断网继续绕障直至离线完成再重连只查询、Nav2 本地速度过期停车、接管松杆不续跑/明确继续重规划不重做已完成动作、取消/完成并发、日志洪峰控制优先、配置重启与临时调试离线自动恢复。
- [ ] 核查 `/ground_truth/odom` 订阅拓扑：现有 `simulation_reference.py` 是已知 truth 定位 fixture，必须在报告单列；新版 motion controller/estimator/response 不得消费真值。若导航验收仍依赖该 fixture，明确只证明仿真定位给定条件下导航，不声明已验收真实定位；需要无真值导航的证据时，使用现有 SLAM 参考链路另跑并冻结地图/定位来源，不用真值误差零冒充激光定位通过。是否可用在 T1 查清，作为导航验收真实限制回报。
- [ ] 按需求映射迁移旧“断网取消、网页时间资格、固定低速/普通障碍锁存”断言；保留认证隔离、旧命令拒绝、感知来源/时效、急停持久化、shutdown/唯一出口回归，不能直接整文件删旧测试。用 `rg -n '5 m/s|drive.challenge|drive.clock|disconnect|offline|0\.3|0\.5' tests web/src docs/protocol docs/operations README.md` 逐项审查，允许有效物理/超时数值，不能盲替换。
- [ ] 本地静态/纯逻辑：`.venv/bin/python -m pytest tests/unit tests/integration tests/e2e -q`、`.venv/bin/python -m ruff check src tests tools`、`.venv/bin/python -m mypy src`；`cd web && npm run typecheck && npm test -- --run && npm run build`。若有与本任务无关失败，记录确切基线与负责人处理，不改无关代码造绿。
- [ ] Linux 同版：`colcon test --build-base ../build --install-base ../install`、`colcon test-result --test-result-base ../build --verbose`；`python -m pytest tests/ros/test_vehicle_motion_acceptance.py tests/ros/test_simulation_manual.py tests/ros/test_simulation_navigation.py -vv --junitxml="$RUN_ROOT/acceptance.xml"`；全部必需场景非 skip，配 T1 capture/analyze/bag 与 web E2E。每个场景设冻结最大时限并 finally stop，保留墙钟/ROS 时钟和失败原始证据。
- [ ] 负责人亲读所有实际 diff、原始曲线与需求映射，缺 UI/运动/导航证据保持 Draft PR #14。全部首版项通过才声明阶段5完成；在线自适应、实车、反向制动建立独立后续记录，STM32 教学 PI 已证而目标烧录未知写入硬件记录，均不阻塞仿真首版。负责人按已有授权提交/推送当前功能分支并更新 #14，不自动合并。

## 需求 → 任务 → 验收映射（逐项交付门）

| 设计要求 | 任务 | 必需证据 |
| --- | --- | --- |
| senior_4wd_dl、2/5 km/h、轮能力组合曲率 | T1/T3/T5/T9/T10 | frozen 车型/能力、18.27 rad/s 对照、两档真值稳态及组合限幅曲线 |
| 多源车身/GPS/IMU/轮速退化 vs 执行回执；区间到当前边界单一归属 | T3/T7/T9/T10 | wheel 空转负对照、区间/年龄账本、主界面速度来源 |
| 常规闭环/加减速/jerk/抗饱和，正反先停，无默认反向制动 | T1/T5/T10 | 方向切换和正常/滑移/制动曲线、无跨零反向目标 |
| 方向动态停车、5% 几何、前堵可退、后方未知、弧线/横向惯性 | T4/T5/T10 | 不同距离单调许可、真值扩张足迹无碰撞、扫掠外角测试 |
| 快收紧/异步放宽有界、不饥饿/危险撤销 | T1/T4/T10 | 连续观测慢 worker 有限放宽、新障碍否决、最坏 tick 耗时 |
| 障碍/反馈失效/真实执行故障/用户急停分离 | T3/T5/T6/T9/T10 | 测速退化可行驶/无方向感知受控停车、5份/0.5s与平滑恢复、独立恢复、版本拒绝、实体优先 |
| 手动断流本地减速、迟到积压不可复活 | T2/T5/T10 | 封闭 authority、100 条迟到样本、车体停车距离 |
| 全部非手动任务自主/幂等/未知/离线不新接收/重启不续跑 | T6/T7/T10 | 断网完成重连 ID/进度相同、配置/地图未知查询、调度与重启回归 |
| 手动接管暂停保进度、继续按现位姿重规划 | T5/T6/T9/T10 | 旧 Nav2 输出撤销、松杆停待、完成/接管竞态及动作恰好一次 |
| 删除浏览器校时/挑战/资格，保留 RTT/必要保护 | T2/T9/T10 | 产品调用盘点清零、RTT 展示、来源/传感时效回归 |
| 五项驾驶设置、原子持久、停稳门、版本冲突、回读/默认 | T7/T9/T10 | 越界/并发/断线/落盘故障/重启、倒车双限，页面无额外参数 |
| 日志级别/保留/debug 自动恢复，无磁盘 UI | T7/T8/T9/T10 | 离线/重启恢复、轮转 gap、界面字段清单 |
| 统一状态/下一动作/急停待确认/断线最后已知 | T7/T9/T10 | 七态布局预览、状态测试、真实链路截图 |
| 授权日志、筛选跟随分页下载、有界队列、脱敏、控制优先 | T8/T9/T10 | 跨车拒绝、游标缺口、洪峰时序/容量与浏览器窗口 |
| 共用 Nav2 约束、三点/绕行/无路、同版完整链路 | T5/T6/T10 | 同 SHA 路线轨迹/动态障碍/离线继续；truth fixture 限制单列 |
| 迁移冲突旧测试、保有效回归、文档一致/不伪称完成 | T2–T10 | rg 清单、回归结果、README/协议/矩阵、负责人 diff 审核 |
| 下位机教学 PI 非量产证明、真实硬件/在线自适应后续 | T1/T10 | 独立限制/后续记录；不让未知固件阻塞仿真 |

## 历史阶段记录（以下自审及进度保留各时点事实）

本节及后续阶段日志保留当时的冻结/未接线/待审描述；现行接线例外与分支规则以前文“当前接线与分支范围”为准，不据旧日志重新阻塞获批正常仿真工作。

## 自审与派发结论（计划编写阶段）

- 逐节对照最终设计与 builder-check-v2，全部要求已落入上表；无新增用户确认项。倒车/日志资源默认是明确实施选择，必须由负责人审计划，非伪称原文给定。
- 已核对消费/产出接口、字段/单位/时钟与任务顺序；控制/任务/配置/log 的状态拥有者独立，节点只接线；共享文件串行接线。每项均有针对失败断言、通过命令、审 diff/提交门；新测试/脚本命令标为待创建，不声称现有。
- 已运行现有基线 `.venv/bin/python -m pytest tests/unit/test_motion_estimator.py tests/unit/test_async_guard.py tests/unit/test_arbiter.py tests/integration/test_task_runner.py -q`：136 passed in 4.50s；doctor 静态通过、实时 not_run；本轮未启动服务/仿真，未改产品代码。
- 第一批派发 T1（有限取证/冻结，不改控制），建议 Sol medium 负责校准/验收决策，Terra medium 可执行确定的只读盘点；本任务不创建 worker。T1 环境无法提供时准确上报缺 Linux ROS/Fortress/私有 profile 与接入途径，不能用 136 单测替代；T1适用数据冻结后才允许生产输出阶段；获批T2a/T3a/T3b/T4a/T5a纯模块例外不受此阻塞。
- 未决技术风险：VM 当前可访问性与负载；满速激光质量/逐束时间；实际 actuator 限幅/制动；异步快路径预算；truth 定位 fixture 对无真值导航证据的限制；真实固件/watchdog/安装外参未知。前五项由 T1/T4/T10 实测并限制声明，硬件未知另列后续；在线学习与反向制动首版排除。

计划文档检查：`git diff --check` 无输出；两个新增/未跟踪文档额外用 `git diff --no-index --check /dev/null <path>` 检查均无空白错误输出（exit 1 表示文件有差异）。既有八个 tracked 脏文件的 diff 统计保持不变；本轮仅改本计划和设计标题/状态。


T1 后续进度：四条 reviewer 修复、专用 fixture 生成器/测试、r3 冻结默认栈实际手动与导航取证已完成，记录及源/安装/raw SHA 见 `docs/testing/2026-11-12-vehicle-motion-t1-review-baseline.md`；两场采集正常，旧产品测试均失败，不视为 T1 或首版通过。普通结束现用完整离散轨迹验证零速，Humble 缺 GID 明确null，ROS reset/无效数据不跨段拼接；校准 fixture 仅静态展开/编译待负责人审。响应/误差/预算尚未冻结，后续控制实现门保持不变。

T1 2026-11-13 子阶段：已获负责人授权的固定+.3m/s隔离fixture完成两次独立原点静止运行（repeat6/7），各100条regular正常归零、原生bag101条命令，真值确认零命令后才物理停下；标准capture缺产品链流仍明确exit2。现场受限recorder/plugin角色、ROS/transport并行发现及追踪时钟已补定向测试，52 passed、Ruff/Mypy通过；实际raw/工具/安装身份/清理SHA和r3 manual身份不匹配、navigation turn_stall同区间时间线见 `docs/testing/2026-11-13-vehicle-motion-low-calibration.md`。scan单帧速度误差、一次失效/epoch重置、采样/调度缺口及仅低速覆盖仍阻止通用边界冻结；29个实测字段保持null，T1整体未完成。下批低档5/9、高档25/18、倒车和turn计划/参数化fixture仅生成供审，未执行；先亲审实际diff、估计分辨率/来源失效问题，再选下一次有限取证，生产输出前适用边界冻结门保持不变；获批纯模块例外可并行。

T1 2026-11-13 扩展测量：低档5/9、倒车-.3、组合转向(.25,+.5)各两次正常固定计划已完成，身份与全新急停状态一致的旧manual真实3 passed；高档按已授权依赖停止条件未运行，走廊353帧实际信息矩阵不足而非调整gate/cap。分组实测候选已追加 `config/vehicle-motion-acceptance.yaml.candidates`，29个正式实测边界及freeze仍未批准；数据、原生缺流、环境负对照、SHA和静态场地修正提案见 `docs/testing/2026-11-13-vehicle-motion-expanded-calibration.md`。本轮明确授权的T2a只允许纯contracts/manual_session、显式注入TTL，可与T1测量并行；T2a与后续获批T3a/T3b/T4a/T5a纯模块例外均不允许接输出；T3b定量界/T4/T5接线仍需适用测量冻结，T2a完成状态以其独立回报为准。负责人须亲审实际diff和原始材料后确定下一批场地/估计验证与候选冻结；不以当前子阶段称首版完成。

## 2026-11-13 用户多源修订与分层验收增量

此节优先于前文激光唯一反馈/任一缺源全向停车的旧假设。T1 继续测量；T2a 仅纯 contracts/manual_session（显式 TTL、不接输出）可并行。源质量字段在 T2/T3 架构审阅时同步，未冻结适用测量和退化策略前，不宣称 T3b/T4/T5 生产能力或实车资格；显式隔离仿真接线按前述现行例外执行；T3a 纯准入/适配/模式与 T3b 纯离线数值融合、T4a 纯方向约束/调度及T5a纯闭环模块为已批准限定例外。T6 接线依赖 T5，T7 集成依赖 T5/T6 保持不变。

| 需求/场景 | 任务与边界 | 验收与证据 |
| --- | --- | --- |
| 多源 BodyMotion、质量/时效/协方差、源切换 | T1 核验 GPS 资料及独立误差；T2 定义模式和源状态；T3 独占融合、坐标/时钟转换和不确定性 | GPS 缺失是正常分支；拒绝过期/漂移与无依据协方差；保留 producer epoch/GID；用独立留出轨迹验证 |
| 受限轮速退化、平滑恢复 | T1 测摩擦/滑移，内部 cap 待测；T3 估计与退化策略，T4 感知方向约束，T5 轨迹恢复 | 激光/GPS 缺失但障碍感知健康时可限速行驶；不虚报真速上界，源恢复不跃升；无可用方向感知受控停车 |
| 丰富地标基线 | T1 当前获批 SDF，仅静态附加几何 | 固定低档两次，质量和场地成立后高档两次；区间 scan/truth 误差与原弱场并列，不替换旧负例 |
| 长直、重复、弱特征 | T1 已保留原走廊失效；T3/T10 重放与实景回归 | 保留 unobservable、错误匹配、重置首因；需要健康感知下的合法退化，不以改场地消除失败 |
| 狭窄、转弯、前后障碍/可退方向 | T4/T10 新有界物理场地 | 扫掠、前堵可退、Nav2 共用约束；不能分别限轴替代轨迹检验 |
| 滑移、摩擦差异、动态障碍 | T1 响应候选；T4/T5/T10 扩展物理场景 | 明列 mu、惯量与执行器假设；测车体/轮差，普通障碍不锁存急停；运动估计恢复与执行故障分别验收 |
| GPS 有效/失锁/漂移，激光测速失效，组合失效/恢复 | T3/T10 先可复现带标签注入；有真实模型后独立 Gazebo/硬件验证 | 注入不是硬件精度证据；检查降级 cap、方向感知、模式原因、平滑恢复和迟到数据不复活 |

当前产品 xacro 无 GPS 传感器；厂家代码具备 NMEA/ublox 速度接口，但实装型号/精度未知。T1 可执行场景为获批丰富地标物理仿真和旧弱特征 bag 离线对照；其余矩阵按所属任务实现，不能为一次测量宣称完成。下一轮估计器建议须针对已有 coarse+fine 的实际实现，独立 holdout 冻结后再实施，不重复已有 fine pass。

T1 本轮实测进度：见 `docs/testing/2026-11-13-vehicle-motion-observable-corridor.md`。获批新地标低档独立两次已采全原生诊断流，原产品 source/install 复核无差异；运动 scan 仅 3/78 合格，停止依赖高档。第一批 SQLite 活动读取引发 recorder 写锁的采集失败另存负例，修复后只通过实际消息就绪、停止后读库。新增候选分组不冻结阈值；下一有限任务是离线 matcher 失败子原因诊断及固定回归，独立未见 holdout 在实现前登记，不能重复已有 fine pass。

T1 matcher 诊断子阶段（2026-11-13）：新增独立结构化诊断和有界离线 replay，不更改有效性、阈值或 epoch/reset。新地标两次所有 match_unqualified 均仅 response 拒绝（各 33），弱场几何窗数拒绝；记录不足的旧 turn 首帧按缺少前缀解释，不能伪造热启动。详见 `docs/testing/2026-11-13-vehicle-motion-matcher-diagnostics.md`。激光已安装及厂家板载 IMU 串口/imu/data_raw 能力已确认，实装标定待核验；用户主退化为无可识别特征，继续按融合与 IMU+轮速限速模式设计。后续已批准并实施 T3a 多源质量/退化纯逻辑（显式注入配置、未知界None、不接运动输出），并保持接线/闭环之前的测量冻结门；这不授权 T3b 输出接线，也不宣布 T1 完成。不得因低特征激光失效而永久禁行，也不得据本轮 response 数据自动降低阈值或改评分。

T3a 本轮交付与审核修正：诊断构造/发布及失败日志均隔离异常，不改变 motion/epoch/reset；Linux 注入 bad_alloc/非标准异常有 RED/GREEN，5个CTest target通过。replay 对 body.source_epoch 与 owner epoch 不一致按实际 node 准入拒绝并 reset。Python 纯模块已实现，本轮合成配置只验证逻辑；`BodyMotion.upper` 消费点仅纯参考构造和契约测试，无运行链接入。本轮T3b已获准按明确点/区间/Delta模型实现纯离线数值融合；后续生产界仍需按适用数据验收，不能直接以参考选择或合成CI接管控制。

## T3b 本轮获批实施范围（2026-11-13）

实现不接输出的点/区间/SE2 Delta输入、AdmissionOwner单一资格状态及有界接纳事件/游标、因果ZOH有界TemporalResolver、固定w=.5分量CI和FusionSnapshot离线链。解析器不以T3a精确同窗组合成功为输入前置；复用源健康/过期判断，缺覆盖返回原因。原始Delta/外参/位移协方差保留，不将点样本伪造为区间；无侧向观测不以vy=0冒充真值，纯gyro不产生完整body delta。统计模型须有明确依据才进CI，未知相关保守处理；GPSheading一阶传播标近似，upper仍None，不用n-sigma当停车保证。区间平均和当前预测分别命名，重复原样本/tick不重复融合或累计恢复。仅离线真实bag与标明合成GPS/统计输入验证，不运行Gazebo、不接ROS/cloud/UI/T4/T5，不把实测未知变成拒绝实现数值融合的理由。


### T4a 实际进度（2026-11-13，已获审并独立提交）

已实现constraints/constraint_worker、不可变许可/制动契约及37项定向测试；复用既有ResponseBounds，只有一个调度owner，不改safety输出。314项相关回归、严格Mypy、Ruff通过。测试响应/模式风险包络/候选/预算均为synthetic，T1未冻结、真实CI仍0、物理停车及Linux最坏耗时未验收。详见 `docs/testing/2026-11-13-vehicle-motion-t4a-directional.md` 及其assumptions；T4生产接线/验收的原勾选项保持未完成。

T5下一步消费：由原始时效/来源构造Perception并保留depth、自滤和unknown；使用独立验证profile及当前状态适用RiskEnvelope；控制器持有加速度/jerk/反向停车状态，每个实际轨迹步调用validate_output（几何证明不替代控制可实现性）；按BrakeRequest执行持续制动及真实静止判断，普通障碍不触发ESTOP/清authority/取消任务；独占executor生命周期与唯一输出后续接线。禁止把固定profile参数误作固定零速状态包络，禁止用统计P或轮速任意误差替代独立物理能力。


T5a当前阶段：纯模块与合成闭环已实现，未改旧safety、ROS/UI/cloud、协议或T1参数；待负责人亲审实际diff后提交。正常加减速/停止、最终PI+轮限后T4核验、缺许可的独立停车证明、首次停车模型监督截止、模式cap平滑和独立复位已有定向测试与曲线。没有启动Gazebo/硬件或声明5km/h完成；T4几何拆分另做机械审阅，不混入本批。

### 2026-11-13 T3/T4/T5 simulation-only 实际输出接线待审

按本次明确授权，`motion_fusion` 原生观测适配、SafetyGate 唯一输出 owner、SCF2 同版 guard/decoder、T6 FinalCommand/MotionRuntimeState 实际 wire 已实现待审；默认参数空，候选仅域173/localhost/use_sim_time显式选择。普通退役迁移为 fence+本机制动接管后的 retired，不是硬零或车体静止。候选模型 unvalidated、T1 null不冻结。四正常起停/转向入口已实现；后续已获审并完成同版Linux六包构建及有界低档调试，低档未通过、其余三场依赖停止，尚无正常驾驶验收通过；本次实际结果见 `docs/testing/2026-11-13-vehicle-motion-normal-runtime.md`。原接线接口与命令见 `docs/testing/2026-11-13-vehicle-motion-output-wiring.md`。T6 producer/协议/Agent/UI由其owner独占，本轮只消费其实际类型；GPS、硬件急停实装、全程弱场初始静止证据与动态环境尚未接通，不据本轮单测宣称物理安全或5 km/h通过。

### 2026-11-13 T5 Linux正常路径实际调试（未通过）

新冻结目录同版编译msg/C++guard/ROS六包成功；限定C++原生frame回归通过，低档真实producer目标已接纳。修复内部Twist零的float边界、candidate无附件选择、release重采时钟假fault、ROS预测域与steady原证据时效混用；核实安装版本GPU lidar/bridge契约后，仅显式隔离candidate对+Inf产生NO_RETURN来源证据，shared filter/matcher不改。r6低档原生truth瞬时峰值约.149669m/s，但349ms控制tick间隙/手动断流导致失败，结束仿真时尚未证明车体停稳。20Hz候选调度没有解决该时序问题。高档/两转向均未运行，不宣称5kmh或网页/导航通过；全部证据、清理及最小下一步见上述正常路径报告。T1冻结null保持，下一步先定位低档调度缺口再新冻结验证，不能延长旧authority或证据deadline造通过。
