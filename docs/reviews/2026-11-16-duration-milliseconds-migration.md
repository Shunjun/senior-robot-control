# 小于 2 秒计时配置：毫秒迁移审阅清单

状态：**只读盘点，未实施，待负责人审阅**；以下证据补充表优先于原始表中简写。依据本工作树（存在并行未提交改动），实施前重新读取并与各 owner 合并；数值仅作等时换算。表中位置是入口和主要消费者，实施时须对旧标识全仓搜索，包含调用方、断言、生成 schema、示例及现行文档。无部署授权。

## 生效配置和可调默认值

| 旧接口（秒）→ 新接口（毫秒） | 旧值 → 新值；入口、消费者与冲突 |
|---|---|
| `SensorTiming.soft_age_s` → `soft_age_ms` | 0.3 → 300；`src/senior_control/sensors/defaults.json`、`sensors/policy.py`；age 分级、ROS `sensor_soft_age_s` → `sensor_soft_age_ms`，`ros_sensor_parameters` 及所有订阅/启动参数。owner：ctx_78d207f1bc18 全系统时钟。 |
| `SensorTiming.hard_age_s` → `hard_age_ms` | 1 → 1000；同上，ROS `sensor_hard_age_s` → `sensor_hard_age_ms`；影响 `real_runtime.py`、motion admission/temporal/pose、vision、ROS ingress、C++ `scan_motion_node.cpp`。在秒和纳秒运算入口分别 `/1000`、`*1000000`，不能同时重复转换。owner：ctx_78d207f1bc18，涉及学习 ctx_ebb465ebe313。 |
| `HardwareMotionProfile.max_age` → `max_age_ms` | 示例/现配 0.5 → 500；`config/hardware-motion*.json`、schema、`hardware_profile.py`。当前 validator 校验旧值后强制覆盖为策略 hard age（现 1 秒），因此 0.5 并非生效值；应与策略 owner 决定删除退役副本还是仅迁移被接收的契约，不能恢复 0.5 行为。`simulation_motion_profile.py` 同样覆盖。 |
| `HardwareMotionProfile.lateral_stop_s` → `lateral_stop_ms` | 实配 1 → 1000；`hardware-motion.json`、example null、schema；`hardware_profile.py` 上界与 `horizon_s`（12 秒）比较、`motion/constraints.py` 消费前转秒；保持 null 示例未标定。 |
| `SourceRule.max_interval_s` → `max_interval_ms` | 默认构造 0.4 → 400；`hardware_profile.py:185`、`motion/observations.py`；schema 的 SourceRule 必填字段及 `adapt` 区间比较。`max_age_s` 为策略派生 1 → 1000，名称改 `max_age_ms`；同版修改嵌套 profile 输入及验证。owner：时钟。 |
| `AdmissionConfig.heading_max_age_s` → `heading_max_age_ms` | 策略派生 1 → 1000；`hardware_profile.py`、`motion/observations.py` 和 schema；与 ROS/steady 秒比较边界转换。owner：时钟。 |
| `AdmissionConfig.alignment_max_skew_s` → `alignment_max_skew_ms` | 策略派生 1 → 1000；同上，两个 ROS 秒时间戳差在比较边界转换。owner：时钟。 |
| `AdmissionConfig.clock_pause_s` → `clock_pause_ms` | 策略派生 1 → 1000；schema、`source_admission.py`；owner：时钟。 |
| `TemporalConfig.max_hold_s` → `max_hold_ms` | 策略派生 1 → 1000；schema、`hardware_profile.py`、`temporal.py`；与 max gap 取最小值前保持同单位。owner：时钟。 |
| `TemporalConfig.max_gap_s` → `max_gap_ms` | 策略派生 1 → 1000；同上，现有双输入兼容注释须随契约审查。owner：时钟。 |
| `TemporalConfig.prediction_horizon_s` → `prediction_horizon_ms` | 策略派生 1 → 1000；schema、`temporal.py`、`fusion.py`；算法秒边界转换。owner：时钟。 |
| `scan_motion_node.max_interval` → `max_interval_ms` | 0.4 → 400，真实及仿真 YAML、`real_runtime.py`、`scan_motion_node.cpp`；C++ `Limits.max_interval` 默认 0.3 → 300，传入匹配器前转秒；`scan_matcher.cpp` 和 replay/测试需一致。owner：时钟。 |
| `command_ttl_seconds` → `command_ttl_ms` | 0.3 → 300；`src/senior_control/config.py` AgentSettings 仅解析未接线，`agent/main.py` 独立默认实际生效、`real_runtime.py`、ROS `command_node.py`、仿真 YAML；分别追踪 IPC/manual ingress 和校验，秒 deadline 边界 `/1000`。真实 `hardware-safety.yaml` 有过时副本，见下。owner：心跳 10 / 时钟。 |
| `ControlCadence.period_s` → `period_ms` | 默认 0.05 → 50；`motion/cadence.py` 与构造调用、cadence 测试；调度使用单调秒。owner：时钟。 |
| `ConstraintScheduler.worker_budget_s` → `worker_budget_ms` | `motion_core_output.py` 显式 0.25 → 250；`motion/constraint_worker.py` 及调用和测试；超时边界用秒。owner：时钟/故障隔离 11。 |
| `VideoLimits.frame_poll_s` → `frame_poll_ms` | 0.1 → 100；`vision/stream.py`、配置构造、异步等待和测试。 |
| `VideoLimits.push_timeout_s` → `push_timeout_ms` | 0.5 → 500；同上，`asyncio` 秒接口转换；`max_frame_age_s` 默认 2 秒不动。 |
| `TrackingConfig.max_detection_age_s` → `max_detection_age_ms` | 0.3 → 300；`vision/tracking.py` 仅校验；无行为消费者，按未生效项处理，勿复活。 |
| `TrackingConfig.motion_min_baseline_s` → `motion_min_baseline_ms` | 0.15 → 150；同上，仅校验；与 max baseline 比较须同单位，勿复活。 |
| `TrackingConfig.motion_max_baseline_s` → `motion_max_baseline_ms` | 0.6 → 600；同上，仅校验，勿恢复不存在的运动计算。 |
| `SpawnInferenceWorker.max_skew_s` → `max_skew_ms` | 0.03 → 30；`vision/spawn_detector.py`、synchronizer、调用方；校验上限 0.2 → 200，保留零的语义。owner：时钟/producer。 |
| `DepthAdmission.future_wait_s` → `future_wait_ms` | 0.02 → 20；`vision/runtime_evidence.py`，上限与 max age 比较、ROS 秒/纳秒消费。owner：时钟。 |
| `RawCaptureAdmission.hard_limit_s` → `hard_limit_ms` | 1 → 1000；`vision/raw_capture.py`，与输入 max age 的校验同单位，调用和测试。owner：时钟/producer。 |
| `EgressAdmission.hard_limit_s` → `hard_limit_ms` | 1 → 1000；`vision/egress_receipt.py`，边界与测试同上。owner：时钟/producer。 |
| `MapArchive.capture_timeout_seconds` → `capture_timeout_ms` | 1 → 1000；`mapping/archive.py` 构造参数、async 等待与测试；外部 asyncio 秒制。 |
| `NavigationEffects.timeout` → `timeout_ms` | 0.3 → 300；`tasks/navigation_port.py`、构造调用、超时测试。 |
| `IpcClient.timeout` → `timeout_ms` | 1 → 1000；`runtime/ipc.py`、调用方（`docker_start_helper.py` 显式 0.5 → 500）、socket/async 边界与测试。 |
| `Supervisor.retry_delay` → `retry_delay_ms` | 1 → 1000；`runtime/supervisor.py`、调用、sleep/测试。 |
| `Supervisor.shutdown_timeout` → `shutdown_timeout_ms` | 1 → 1000；`runtime/supervisor.py`、等待/测试。 |
| `Agent.retry_delay` → `retry_delay_ms` | 1 → 1000；`agent/main.py` 及构造和等待；与 supervisor 是不同接口。 |
| `WebSocketLink.heartbeat_timeout` → `heartbeat_timeout_ms` | 当前实际默认 1 秒；`agent/websocket.py:67,82,91,114` 会在回复超时后断链，和新要求 5 秒冲突。交心跳 10 owner 将断线窗口落实为 5 秒；不要按单位迁移把它锁定为 1000ms，也不要误认为现已实现 5 秒。 |
| `feedback_recovery_min_span_s` → `feedback_recovery_min_span_ms` | 0.5 → 500；`config/vehicle-motion-acceptance.yaml:14` 的模拟初始值；全仓无程序读取该名称，属待验收配置草稿，不能称为已生效；此文件同时包含历史测量，见下。 |

## ROS 草稿、策略覆盖及外部契约

| 字段 | 审阅/实施处置 |
|---|---|
| `scan_adapter.max_age_seconds` → `max_age_ms` | 声明 0.5 → 500；`self_filter.yaml` 0.5、仿真 `simulation_motion.yaml` 0.8、真实草稿 0.5 乘 1000；`scan_adapter.py` 实际使用统一 hard age，`real_runtime.py` 又覆盖该项。不要以这些旧值替换策略值。owner：时钟。 |
| `safety_scan_ingress.observation_max_age` → `observation_max_age_ms` | 仿真 0.8、真实草稿 0.5 分别 800、500；`safety_scan_ingress.py` 实际使用策略，`real_runtime.py` 覆盖。owner：时钟。 |
| `scan_motion_node.scan_max_age` | 真实/仿真草稿 0.5/0.8，但 C++ 声明旧名仅兼容且实际读取 `sensor_hard_age_s`。优先删除退役副本，勿新建独立年龄 owner；若确需兼容，标注 `//Compatible` 并说明理由。 |
| `depth_ingress.max_age_s`、`pair_wait_s`；`semantic_ingress.max_age_s` | YAML 分别 0.8、0.1、0.3；代码声明后实际分别使用统一 hard age（包括 pair wait），不能迁移旧值使其生效。与时钟 owner 核对后移除退役副本或以 `_ms` 明确仅输入契约；更新 `simulation_rgbd.yaml`、节点和测试。 |
| `hardware-safety.yaml` 的 `safety_gate` 年龄/TTL，以及 `command_node.heartbeat_seconds`、`disconnect_seconds` | `real_runtime.RETIRED_PARAMETERS` 已列退役；不要按 0.5、0.3、0.2 或 2 秒复活。当前心跳断线以用户要求 5 秒为准。清理草稿由对应 owner 排期。 |
| `nav2.yaml` `costmap_update_timeout` 0.3/1、`transform_tolerance` 0.1、`reset_period` 1 | Nav2 插件参数原名和秒单位是外部接口，保持原样；若引入仓内 ms 参数，生成传给 Nav2 的 YAML 时明确 `/1000`，并同时覆盖 overlay/launch。`bt_loop_duration`、`default_server_timeout`、`wait_for_service_timeout` 已是 Nav2 毫秒契约，禁止再乘 1000。owner：时钟。 |
| `slam.yaml` `transform_publish_period` 0.02、`minimum_time_interval` 0.5、`transform_timeout` 0.2 | slam_toolbox 外部参数秒制，保持原名；若仓内引入 ms 配置只在适配处转秒。 |
| `senior_sim/config/controllers.yaml` `cmd_vel_timeout` 0.3 | ROS diff drive controller 外部秒制；保持名称和值，内部配置若引入 ms 则边界转秒。README 与 `test_sim_contract.py` 保留其外部单位。 |

## 空值、零值、非配置与界限

- `config/vehicle-motion-acceptance.yaml` 中 `stationary_window_s`、`steady_window_s`、`settling_time_s`、`actuator_lag_s`、`slip_detection_window_s`、`stall_detection_window_s`、`manual_liveness_s`、`sensor_age_s`、`permit_age_s`、`control_period_s`、`control_wiring_budget_s`、`fast_path_budget_s`、`async_relaxation_budget_s`、`navigation_progress_window_s` 都是尚未标定/未给定，不等于零；没有数值无法依据小于 2 秒判定，保留秒名直到确定所属契约和标定值。`config.py` 的 `observation_max_age=None`、`hardware-navigation.schema.json` 的必填 `max_age_s` 同理核查实际调用/契约，不猜值。`spawn_detector.max_result_age_s=None` 表示采用统一 hard age，随该 owner 使用 ms 契约，而非赋 0。
- `recovery_span_s=0` 表示无需时间跨度，归属 admission 时间契约；若 owner 决定改 `recovery_span_ms`，零仍为零，验证保持 `>=0`。其他零默认的禁用或无偏差字段须逐项确认，不能机械迁移。
- 历史测量、原始日志、`vehicle-motion-acceptance.yaml` 的已记录观察值和绝对 ROS 时间保持原单位及哈希；网页 `age_seconds` 是测量输出，`rtt_ms` 已是毫秒；任务 `every_seconds` 是用户计划间隔且无小于 2 秒默认，不按某次输入变化改契约。
- 频率 Hz、速度、加速度、积分 dt、绝对时间戳及内部 `sleep`、`wait`、`join` 调用本身保持秒；固定 API 调用字面量不当成可调配置自动重命名。不要恢复被删除的 150ms 期限或 30s 学习预算。

## 实施顺序和集中验证

1. 等时钟、学习 owner 合并后重新冻结工作树，逐项检查实际默认值和未提交差异；心跳 10、故障隔离 11 与本表冲突项交对应 owner。先统一配置入口和 ROS 参数，再在内部算法、ROS C++、第三方库边界各换算一次；同版更新实车 JSON/YAML 和生成 schema，拒绝旧名，除确有已部署迁移必要并标记 `//Compatible` 外不保留别名。
2. 对所有旧字段执行 `rg`，分类为退役、历史证据、外部契约、实际遗漏；更新配置例子、校验上下界及生成文件、现行文档、Python/C++ 调用和测试。检查 `None`、0、边界等价、浮点精度、跨单位比较，确保 2 秒及以上字段不被改动。
3. 集中运行 profile/schema 与配置加载、时钟/采集、scan matcher、motion cadence/TTL、vision、IPC/任务/流媒体单测；构建 ROS 消息及 C++ matcher，验证仿真与真实参数生成和第三方秒制参数；执行仓库标准检查。不启动开发服务、不部署。逐项断言 300ms 等于旧 0.3 秒、1000ms 等于旧 1 秒、50ms 等于旧 0.05 秒，验证到期前后行为及旧名拒绝；保留历史证据不变。

本清单基于当前并行工作树；并行 owner 合并后应重新核对变动，不将后续复核替代本次盘点。这里不声称 ROS 构建、运行或实车验证已完成。

## 逐项定义与实际消费证据（复核版）

下列标记：生效＝当前组合或调用消费；接口＝可注入但没有证明当前产品组合采用；覆盖＝输入被统一策略覆盖；无消费＝仅声明或校验；外部＝外部参数契约。数值和迁移方案见上表，以下列出精确证据及补漏，状态以此表为准。

| 项目与状态 | 定义、入口和实际消费证据 |
|---|---|
| `soft_age_s` **生效** | `src/senior_control/sensors/defaults.json:1` 0.3；`sensors/policy.py:27,64,102` 分类和 ROS 注入。 |
| `hard_age_s` **生效** | `defaults.json:1` 1；`sensors/policy.py:29,64,78,103`；C++ 接收 `ros2/src/senior_scan_matcher/src/scan_motion_node.cpp:35,237`；producer/时钟 owner。 |
| `HardwareMotionProfile.max_age` **覆盖** | `config/hardware-motion.json:7` 0.5，`hardware_profile.py:37,41-54` 验证后重设 1；`:185-198` 生成各时序策略；不是 0.5 生效。 |
| `lateral_stop_s` **生效** | `config/hardware-motion.json:34` 1；`motion/hardware_profile.py:120,151` 校验，`ros2/src/senior_robot/senior_robot/motion_core_output.py:946` 传入约束。 |
| `lag` → `lag_ms` **生效** | `config/hardware-motion.json:14` 0.5 → 500；`motion/hardware_profile.py:92` 非负，`config/hardware-motion.schema.json:433`、`config/hardware-motion.example.json:13` null 同版更新；`motion_core_output.py:975` 传入响应，`motion/constraints.py:317,353,625` 停车计算秒边界 `/1000`，与 `motion/stopping_input.py:116,156` 一并迁移。仿真 `simulation_motion_profile.py:38` 同名 0.2 → 200，`simulation_motion_profile.py:130` 构造，消费同一 `motion_core_output.py:975`；不改实际反应时间。owner：时钟/学习。 |
| `SourceRule.max_age_s`、`max_interval_s` **生效** | `motion/observations.py:78-86,278,297-299`；`motion/hardware_profile.py:185` 分别派生 1、0.4；嵌套 schema `config/hardware-motion.schema.json:282,286`；规则配置年龄被 `hardware_profile.py:74-78` 策略覆盖。 |
| `heading_max_age_s`、`alignment_max_skew_s` **生效** | `motion/observations.py:92-101,354-356`；`hardware_profile.py:68-69` 使用策略 1；嵌套输入由 `hardware_profile.py:60-70` 覆盖。 |
| `clock_pause_s` **生效** | `motion/observations.py:101`、`hardware_profile.py:70` 策略 1；`motion/source_admission.py:77` 比较单调秒。 |
| `max_hold_s`、`max_gap_s` **生效** | `motion/temporal.py:46-58,160`；`hardware_profile.py:82,198` 均派生策略 1；嵌套输入被覆盖。 |
| `prediction_horizon_s` **生效** | `motion/temporal.py:52`、`hardware_profile.py:82,198` 派生 1；`motion/fusion.py:359` 消费。 |
| `scan_motion_node.max_interval` **生效** | `real_runtime.py:47,99`、`config/hardware-safety.yaml:26`、`ros2/src/senior_robot/config/simulation_motion.yaml:23` 0.4；`ros2/src/senior_scan_matcher/src/scan_motion_node.cpp:39` 读取、`src/scan_matcher.cpp:253` 比较；`include/senior_scan_matcher/scan_matcher.hpp:53` 独立结构默认 0.3。 |
| `command_node.command_ttl_seconds` **生效** | `real_runtime.py:69,137`、`simulation_motion.yaml:53`、`ros2/src/senior_robot/senior_robot/command_node.py:111`；实际消费者参照同文件 TTL 读取。`Agent.command_ttl_seconds` 为独立入口：`agent/main.py:136,173,616,924`；`AgentSettings.command_ttl_seconds` 只解析无生产连接，见下。 |
| `ControlCadence.period_s` **生效** | `motion/cadence.py:6-10` 默认 0.05，`ros2/src/senior_robot/senior_robot/safety_node.py:101` 实际按 `1/rate` 注入；20Hz 对应 50ms，Hz 参数保留。 |
| `ConstraintScheduler.worker_budget_s` **生效** | `motion/constraint_worker.py:58-63`，`motion_core_output.py:102` 显式 0.25，worker 截止计时。 |
| `VideoLimits.frame_poll_s`、`push_timeout_s` **生效** | `vision/stream.py:80,85-91,205,383,403`，分别 0.1、0.5；session 使用；换算到 `next_frame` 与 `asyncio.wait_for` 秒接口。 |
| `TrackingConfig.max_detection_age_s`、`motion_min_baseline_s`、`motion_max_baseline_s` **无消费** | `vision/tracking.py:22,26-27,35-49` 仅保存及校验；`:75-114,151-164` 不使用这些字段，`perception_node.py:129` 虽构造也不产生时长作用。分别 0.3/0.15/0.6，优先删除无效接口，勿把 300/150/600 接进算法。 |
| `SpawnInferenceWorker.max_skew_s` **生效** | `vision/spawn_detector.py:194,204,216,226` 默认 0.03 且转纳秒；ROS `perception_node.py:107,127` 是另一独立参数 `max_skew_s` 默认 0.03 → `max_skew_ms` 30，消费 `vision/synchronizer.py:58-66,74-79,110`。这两个入口都要迁移，纳秒乘 1,000,000。 |
| `DepthAdmission.future_wait_s` **生效** | `vision/runtime_evidence.py:35,42-48,148` 0.02；`safety_node.py:141` 显式 `min(0.02, scan_age)` 亦需 20ms 同单位比较。 |
| `RawCaptureAdmission.hard_limit_s` **接口** | `vision/raw_capture.py:66,74-79` 默认 1，仅作上限校验；`scan_adapter.py:47`、`safety_scan_ingress.py:41` 显式 5 秒，`vision/independent_motion.py:320` 走默认。保留上限语义。 |
| `EgressAdmission.hard_limit_s` **生效** | `vision/egress_receipt.py:150,153-158` 默认 1；`semantic_ingress.py:78` 默认调用，校验上限。 |
| `MapArchive.capture_timeout_seconds` **接口** | `mapping/archive.py:111,121,129,252` 默认 1，消费 `asyncio.wait`；仓内 `MapArchive` 构造仅见测试，不能称当前生产组合已接线。 |
| `NavigationEffects.timeout` **生效** | `tasks/navigation_port.py:20-27,39,52` 默认 0.3；`agent/composition.py:305` 构造，任务导航调用 wait_for。 |
| `IpcClient.timeout` **生效** | `runtime/ipc.py:284,287,307,329` 默认 1；`docker_start_helper.py:233` 显式 0.5，其余组合调用及单测同步。 |
| `Supervisor.retry_delay`、`shutdown_timeout` **生效** | `runtime/supervisor.py:21,29-31,49-50,83,144-164` 默认 1/1；`agent/main.py:285-294` 注入 retry；shutdown 用默认。 |
| `Agent.retry_delay` **生效** | `agent/main.py:134,170,292` 默认 1；`agent/composition.py:533-570` 使用默认。 |
| `WebSocketLink.heartbeat_timeout` **生效且与新要求冲突** | `agent/websocket.py:45,67,82,91-92,114,154,223`；connector 构造默认 1，计时器到期即断链。5 秒断线窗口需由心跳 10 owner 协同检查该路径及 cloud/ledger；当前工作树并未落实新要求。 |
| `feedback_recovery_min_span_s` **无消费草稿** | `config/vehicle-motion-acceptance.yaml:14` 0.5；仓内名称检索只有该定义，无程序读取；改名 `feedback_recovery_min_span_ms`、500 仅作为验收配置草稿，不宣称运行生效。 |
| `Agent.state_poll_seconds` → `state_poll_ms` **生效** | `agent/main.py:138,213-216,509` 默认 0.25 → 250，`agent/composition.py:533-570` 走默认；状态快照轮询 sleep 边界转秒。 |
| `Deadlines.safety` → `safety_ms` **生效** | `agent/dispatcher.py:37-47` 默认 0.5 → 500；`agent/main.py:161,359,378,727,740,976,1005,1010` 用于停止、清理和安全请求等待；校验和全部等待边界统一转换，涉及故障隔离 11。 |
| `Supervisor.heartbeat_seconds` → `heartbeat_ms` **接口及生效调用** | `runtime/supervisor.py:28,35,45,114-130` 默认 0.2 → 200，为发送周期；`agent/main.py:291` 正式 Agent 覆盖为其自身的默认 1 秒，不应声称 0.2 当前在该组合生效。 |
| `Agent.heartbeat_seconds` → `heartbeat_ms` **生效** | `agent/main.py:133,169,285-292` 默认 1 → 1000，为发送周期；`agent/composition.py:533-570` 未覆盖，`runtime/supervisor.py:114-130` 调度。此周期与 5 秒断线窗口不同，owner：心跳 10。 |
| `AgentSettings.heartbeat_seconds` **无消费** | `config.py:129-133,165-193` 0.2，可由 YAML、`SENIOR_HEARTBEAT_SECONDS`、CLI 合并并校验；全仓生产代码没有 `load_settings` 调用或字段读取，只有 `tests/unit/test_settings.py:8-16`。不得将其接入新发送周期或断线窗口；若保留外部输入契约则改 `heartbeat_ms` 200 与 env 同版更新，优先删除无效项并更新测试。 |
| `AgentSettings.perception_stale_seconds` **无消费** | `config.py:133,165-193` 0.5，可由 YAML、`SENIOR_PERCEPTION_STALE_SECONDS`、CLI 设置；全仓无读取该字段。优先删除无效项；如保留仅作为输入契约需 `perception_stale_ms` 500，同版更新 env/测试，绝不恢复独立过期策略。 |
| `AgentSettings.command_ttl_seconds` **无消费** | `config.py:132,165-193` 0.3，解析但未传给 `Agent`；不可与实际 `Agent` / ROS TTL 合并成已接线字段，优先删除无效项或按输入契约改 `command_ttl_ms` 300。 |
| `scan_adapter.max_age_seconds` **覆盖** | `scan_adapter.py:31,34,47` 声明 0.5 后直接采用统一策略；`self_filter.yaml:5`、`simulation_motion.yaml:11`、`hardware-safety.yaml:14` 是副本。 |
| `safety_scan_ingress.observation_max_age` **覆盖** | `safety_scan_ingress.py:34,37,41`；`simulation_motion.yaml:17`、`hardware-safety.yaml:20` 副本。 |
| `scan_motion_node.scan_max_age` **退役** | `scan_motion_node.cpp:34-35` 仅声明旧副本、实际读取统一参数；`simulation_motion.yaml:22`、`hardware-safety.yaml:25`。 |
| `depth_ingress.max_age_s`、`pair_wait_s` **覆盖** | `depth_ingress.py:67-68,82-84` 明确使用策略值；`simulation_rgbd.yaml:21-22` 旧 0.8、0.1 不生效。 |
| `semantic_ingress.max_age_s` **覆盖** | `semantic_ingress.py:66,76-81` 使用策略；`simulation_rgbd.yaml:38` 0.3 副本。 |
| 退役 ROS `safety_gate` 时长及 `command_node` 心跳 | `real_runtime.py:14-37,160-164` 忽略退役字段；`hardware-safety.yaml:46-50,63-64` 为旧草稿，删除而非复活。 |
| 第三方 Nav2/SLAM/controller | `ros2/src/senior_robot/config/nav2.yaml:12,45,51,150,226-228`、`slam.yaml:24,29-30`、`ros2/src/senior_sim/config/controllers.yaml:12`；外部名称及秒／毫秒约定不改。 |

### 零与 null 独立核查

| 定义 | 语义与处置 |
|---|---|
| `AdmissionConfig.recovery_span_s`，`motion/observations.py:95,117-118`；`hardware_profile.py:67` 0 | `source_admission.py:197` 恢复跨度门槛；0 是零跨度，不是开关。跟 admission 契约改 `recovery_span_ms` 0，原校验非负。 |
| `lateral_stop_s` null，`config/hardware-motion.example.json:27`、`simulation_motion_profile.py:55` | 前者示例未标定而模型要求值，后者表示无横向停车界；同所属 `lateral_stop_ms` 保持 null，绝不填值。 |
| `lag` null，`config/hardware-motion.example.json:13` | 未标定示例；跟 `lag_ms` 保持 null。 |
| `SpawnInferenceWorker.max_result_age_s=None`，`vision/spawn_detector.py:205,208-209` | None 继承策略 1 秒；契约改 ms 时继承策略 1000，不能转为零。 |
| `Supervisor.max_retry_delay=None`，`runtime/supervisor.py:30,49-50` | 根据 retry 的 30 倍计算；默认 30 秒，不按 0 值或 <2 秒规则改。若 retry 输入单位改毫秒，表达式输出与上限比较保持一致。 |
| `DeviceProfile.reaction_time`、`observation_max_age`，`config.py:45,47` | null 为未知、没有标定值；`config.py:103` 将其用于设备校验，默认无时长值，不猜单位范围或填 0。 |
| `MappingEvidence` 的 `confirm_span_s`、`clear_span_s`、`max_age_s`，`mapping/evidence.py:16,18,26` | None 表示未提供；没有固定亚 2 秒默认，保留契约待具体配置值确定。 |
| 验收 null 集合 | `config/vehicle-motion-acceptance.yaml:25-49` 未标定；保持 null、原有秒契约，直至值和用途确定；不修改同文件历史测量。 |
| `hardware_navigation.max_age_s` | `hardware_navigation.py:23` 必填且范围 `(0,5]`，非默认值；无固定亚 2 秒默认，不按运行时输入动态变单位。 |

### 搜索方法与边界

对 `src/`、`ros2/`、`tools/` 的 Python AST 检查带数字默认值的函数参数及注解字段（含 Pydantic `Field(default=...)`、`default_factory`），结合 JSON/YAML、ROS `declare_parameter`、C++ 参数、CLI/env 和全仓字段引用人工分类；频率、长度、速度、加速度、样本数、比率以及仅输出的时间戳或历史证据不列为时长配置。`config.py:182-193` 的通用环境映射是上述三个 `AgentSettings` 输入入口，运行 Agent 则从 `agent/composition.py` 自有 `AgentConfig` 组装。此审阅不读取或修改受保护文件，行号以盘点时工作树为准。

| C++ 补充 | 定义与消费 |
|---|---|
| `CaptureLedger.age_limit` → `age_limit_ms` **生效** | `ros2/src/senior_sensor_ingress/include/senior_sensor_ingress/capture_ledger.hpp:21-25` 构造默认 1 → 1000，上限 1 → 1000；`raw_receipt.hpp:90` 默认构造，`capture_ledger.hpp:58,105` 与 steady 秒作差。`test/test_capture_ledger.cpp:10,31` 注入 1 → 1000，秒比较处一次转换。owner：ctx_78d207f1bc18 producer。 |
| `scan_receipt_admission.max_age_seconds` **固定内部常量** | `ros2/src/senior_slam_bridge/src/scan_receipt_admission.hpp:20,53` 0.5 秒、直接参与单调秒比较，没有可调入口；本次配置迁移不把其单纯改数值，时钟 owner 应留意此独立固定阈值。 |
| Nav2 launch 覆盖 **外部毫秒契约** | `ros2/src/senior_robot/launch/simulation_motion.launch.py:53,55` 分别 250、4000；参数原本即毫秒，不重复转换。 |
