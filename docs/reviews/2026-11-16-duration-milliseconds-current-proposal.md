# 当前生效时长配置的毫秒迁移方案与用例

2026-11-16，只读盘点与方案；**未实施单位迁移、未运行套件**。盘点基于分支 `fix/degraded-drive-speed`、HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da` 及当前大量未提交改动，交付目标仍为 PR26。本文仅新增本文件，不修改、暂存其他文件，不启动服务、ROS、仿真或车辆，不提交推送。

已读取根方案 `/Users/zhangshunjun/Wheel 机器人/车辆启动约束审查与处理方案.md`，以及当前 `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md`，尤其末尾学习几何识别修订；它们及当前真实消费者优先于旧 `2026-11-16-duration-milliseconds-migration.md` 的历史判断。这里的值是仓内软件配置，不代表车端安装产物或实车已核验。

## 约定与当前关键结论

- 当前配置/default 中小于 2 秒的时长，入口值及对应配置变量改 `_ms`；实际时长和原比较符保持。固定接口的单位不随某次传入值是否达到 2 秒动态改变。2 秒及以上的配置保留秒。
- 单调/ROS/UTC 时间戳、测量区间、积分 dt、速度、Hz、日志年龄输出及历史记录维持原单位。毫秒进入秒制算法/asyncio/socket/第三方 API 时 `/1000` 一次，进入纳秒比较时 `*1_000_000` 一次；不先转秒再把同一配置当毫秒重复转换。
- 内部可保留有明确名称和用途的秒制算法投影，但不能保留接受旧配置键的空兼容入口。所有需保留的真正兼容代码按仓规标注 `//Compatible`；本方案不新增双键兼容分支。
- **心跳已经是 5 秒在线窗口**：`protocol.py` 的 `HEARTBEAT_WINDOW_SECONDS=5.0` 被 `agent/websocket.py` 和 `cloud/relay.py` 消费；旧表的 `WebSocketLink.heartbeat_timeout=1` 已不存在。发送周期另为 `HEARTBEAT_INTERVAL_SECONDS=1.0`，应迁移周期 1000ms，不能改在线窗口。
- `AdmissionConfig.clock_pause_s` 已不存在；`DepthAdmission.future_wait_s` 已不存在；学习动作 `StartupLearning.duration_s=3.0` 保持 3 秒。旧 150ms 状态/控制期限、30 秒学习预算均不迁移、不恢复。
- 当前静态发现一处真实接口残留：`safety_node.py:155` 仍调用 `DepthAdmission(max_age_s=scan_age, future_wait_s=min(0.02, scan_age))`，但 `vision/runtime_evidence.py:37` 构造函数只接收 `max_age_s`，走到此处会有未知关键字错误。应删除退役实参，不引入 `future_wait_ms=20`；本轮只记录，没有运行节点证明路径可达性。

## 生效入口与迁移清单

以下路径除特别说明均相对仓库根。每行列出实际入口及消费者；“派生”表示复用统一策略，不新增可独立调节的 TTL。配置和它的投影、调用点、校验、schema、现行文档及测试在实施时同版更新。

### 统一传感器、运动配置、ROS 与 C++

| 当前字段 → 目标字段 | 当前等时值 | 入口、真实消费者与转换边界 |
|---|---|---|
| `SensorTiming.soft_age_s` → `soft_age_ms`；ROS `sensor_soft_age_s` → `sensor_soft_age_ms` | 0.3s → 300ms | `sensors/defaults.json` → `sensors/policy.py` 的 `age()` 分级及 freshness。分级可直接比较 ns 与 `soft_age_ms*1e6`；返回 `Age.seconds` 仍秒。`ros_sensor_parameters()` 及 launch 注入同步；C++ `senior_sensor_ingress/src/rgbd_candidate_receipt.cpp:79` 接收新名。 |
| `SensorTiming.hard_age_s` → `hard_age_ms`；ROS `sensor_hard_age_s` → `sensor_hard_age_ms` | 1s → 1000ms | 同一 JSON/policy。`sensor_current()`、估计器、pose/space、自学习、raw/egress/depth/semantic/RGBD worker 均读取此策略。deadline 为绝对单调秒，构建时 `hard_age_ms/1000`。C++ `scan_motion_node.cpp:36` 接新参数；`sensor_timing.hpp::Policy` 当前构造 s/h 为秒并乘 1e9，改为明确 ms 输入乘 1e6，diagnostics period 仍秒。注意下文保护文件实际依赖。 |
| `HardwareMotionProfile.lag` → `lag_ms`；仿真 profile 同名字段 | 实车 JSON 0.5s → 500ms；仿真 0.2s → 200ms | `config/hardware-motion.json`、example 的 null、`hardware-motion.schema.json`、`motion/hardware_profile.py:91`、`simulation_motion_profile.py:38` → `motion_core_output.py:987` → `ResponseBounds.actuator_lag` / `constraints.stop_axis`。在构建秒制 response 算法模型处 `/1000` 一次，保持停车距离/时间；不可把 500 直接当秒传进 phase 积分。 |
| `HardwareMotionProfile.lateral_stop_s` → `lateral_stop_ms`；仿真同名字段 | JSON 1s → 1000ms；example null、仿真 None 保持 | `hardware_profile.py:121,151` → `motion_core_output.py:957` → `constraints.RiskEnvelope.lateral_stop_s`。profile 与 `horizon_s=12` 校验时 `/1000`；构建秒制误差/停车搜索模型处转换一次，后续 horizon、缓存和风险比较继续秒。`None` 仍代表缺失横向停车界，不替换成 0。 |
| 统一策略派生的 profile `max_age` → `max_age_ms`（仅派生读模型） | 实际 1s → 1000ms；JSON 的 0.5s **并非生效值** | `hardware_profile.py:40-79` 当前先验证保存的旧值，再强制覆盖为 hard age；仿真 `__post_init__` 也覆盖。删除 JSON/example/schema 接受的可编辑 `max_age` 副本及 legacy validator 对它的专门兼容；内部由统一 policy 派生，不能把旧 0.5 改成生效 500ms。 |
| `SourceRule.max_age_s` → `max_age_ms` | 派生 1s → 1000ms | `HardwareMotionProfile.admission()` 与仿真 admission → `observations.py::adapt/is_current` 及 deadline。比较原始捕获/接收单调秒时 `/1000`，不改观测时间戳。 |
| `SourceRule.max_interval_s` → `max_interval_ms` | profile 构造 0.4s → 400ms | 同上 → `physical_support(observation).duration`（秒）上界；在比较边界 `/1000`。`min_quality=0.1`、`max_variance=2.0` 不是时间。 |
| `AdmissionConfig.heading_max_age_s`、`alignment_max_skew_s` → 同前缀 `_ms` | 各派生 1s → 1000ms | `hardware_profile.py::admission` / 仿真 → `observations.py:372-375` 的 heading 单调年龄、测量轴差；秒差比较前转换。当前字段不是旧表所称 ROS-only 差，必须保留当前 time-proof/measurement 语义。 |
| `AdmissionConfig.recovery_span_s` → `recovery_span_ms` | 当前派生 0 → 0 | `hardware_profile.py` / 仿真 → `source_admission.py:197`；0 表示无需额外跨度，仍保留现有非负校验与 recovery_samples=1，不复活旧 5 份/0.5s 门。 |
| `TemporalConfig.max_hold_s`、`max_gap_s`、`prediction_horizon_s` → 同前缀 `_ms` | 各派生 1s → 1000ms | profile `temporal()` → `temporal.py::_check_span`、prediction 和 fusion；同单位求 min 后转秒供区间算法。`history_s=2.0`、`max_samples=512` 不迁移；既有两个 hold/gap 注入输入确实取交集，此次不合并架构。 |
| ROS `scan_motion_node.max_interval` → `max_interval_ms`；`MatcherLimits.max_interval` → `max_interval_ms` | 真实/仿真配置 0.4s → 400ms；C++结构默认 0.3s → 300ms | `real_runtime.py::real_parameter_defaults/REQUIRED_PARAMETERS`、`hardware-safety.yaml`、`simulation_motion.yaml` → `scan_motion_node.cpp:40` → `scan_matcher.hpp:53` / `scan_matcher.cpp:253`。保留扫描 stamp 秒，匹配器比较处使用 ms/1000。不能拿结构默认 300 替换组合已显式配置 400。 |
| `CaptureLedger.age_limit` → `age_limit_ms` | 默认及可调上界 1s → 1000ms | `senior_sensor_ingress/include/.../capture_ledger.hpp:21`，`raw_receipt.hpp` 默认构造 → `observe/expire`。capture_end_steady_ns 是绝对 ns，不迁移；可比较 ns 差与 ms*1e6，或既有秒差对 ms/1000，选一个边界。 |

Matcher replay 是**当前工具配置消费者**：`tools/scan_matcher_replay.py::LIMIT_FIELDS` 从参数读取 `max_interval`，`senior_scan_matcher/src/scan_matcher_replay.cpp:27` 读入 limits。新生成回放输入及两个工具同版改为 ms；已经存档的测量输入、日志和验收证据不重写，读取旧格式确需支持时采用独立明确格式版本适配并标注兼容理由，不能无版本把旧 0.4 当 0.4ms。也不为了本迁移新增不被调用的旧格式分支。

`hardware-motion.schema.json` 是 Pydantic 产物，需同步顶层 `lag/lateral_stop_s/max_age` 以及 `$defs` 中 SourceRule/AdmissionConfig/TemporalConfig 的 properties/required，不能只手改 JSON 值。嵌套 `admission_config/temporal_config` 目前是可选 legacy 声明、必须等于派生策略；优先删除这两个**输入副本**及覆盖兼容代码，保留真正使用的内部 dataclass 新契约及按需 schema 定义。不借清理删除非时间的 fusion/controller 行为。

### Agent、supervisor、IPC、任务与视频

| 当前字段 → 目标字段 | 当前等时值 | 入口、消费者和边界 |
|---|---|---|
| `HEARTBEAT_INTERVAL_SECONDS` → `HEARTBEAT_INTERVAL_MS`；`Agent.heartbeat_seconds`、`Supervisor.heartbeat_seconds` → `heartbeat_ms` | **当前共用默认均 1s → 1000ms** | `protocol.py:20` → `agent/main.py:134,254` → `runtime/supervisor.py:117`。发送 deadline 使用 loop.time 秒，interval=/1000；旧表 supervisor 默认 0.2s 已过时。5 秒在线窗口不动。 |
| `Agent.command_ttl_seconds`、ROS `command_ttl_seconds` → `command_ttl_ms` | 0.3s → 300ms | `agent/main.py:136,613,950` 排队意图/本地 deadline；`real_runtime.py`、真实/仿真 command YAML → `command_node.py:111,117,128,184` → `motion_profile.py` / manual TTL 消费。构建单调秒 deadline、timer 或底层 TTL 模型处 `/1000` 一次。IPC `received_steady/deadline_steady` 是绝对秒，线上字段不乘1000。 |
| `Agent.state_poll_seconds` → `state_poll_ms` | 0.25s → 250ms | `agent/main.py:138,216,505`，composition 默认构造；sleep 处 /1000。`snapshot_seconds=2.0` 不动；故障轮询失败依然只跳过本轮，不关闭 Agent。 |
| `Agent.retry_delay`、`Supervisor.retry_delay` → `retry_delay_ms` | 各默认 1s → 1000ms | Agent 将参数传给 supervisor，后者 uplink loop sleep 和指数退避；两个构造入口同版迁移。`max_retry_delay=None` 的现有推导为 retry*30，默认上限 **30s**，保留其秒制上限接口/语义，在统一秒制退避运算开始前换算 retry，显式上限照旧。retry=0 仍允许零延迟，None 上限推导仍0，不添新最小重试。 |
| `Supervisor.shutdown_timeout` → `shutdown_timeout_ms` | 1s → 1000ms | `runtime/supervisor.py:33,85`，asyncio.wait timeout /1000；保持 pending 时 shutdown_failed，不改故障隔离策略。 |
| `Deadlines.safety` → `safety_ms` | 0.5s → 500ms | `agent/dispatcher.py:43` → `agent/main.py` 多处安全请求、停止/取消等待；所有 safety 访问处转秒，`Deadlines.edit=5`、`map=120` 仍秒；不能把 min(edit,map,safety_ms) 当同单位校验。 |
| `IpcClient.timeout` → `timeout_ms` | 默认 1s → 1000ms；helper 探测0.5s → 500ms | `runtime/ipc.py:296,323,345` → request/send 的 wait_for；`agent/composition.py:344` 的 ipc 工厂同步。navigation client 显式 **3s**、docker helper service client **3s** 不改实际时长：调用同一个新 ms 接口时写3000ms；≥2s配置入口本身仍保持秒，可在工厂适配。不得传裸3造成3ms。 |
| `ControlCadence.period_s` → `period_ms` | 0.05s → 50ms | `motion/cadence.py`、SafetyGate 使用者；deadline/调度计算继续单调秒，构造算法周期时 /1000。固定20Hz、错过槽跳过行为不改。 |
| `ConstraintScheduler.worker_budget_s` → `worker_budget_ms` | `motion_core_output.py:103` 显式0.25s → 250ms | `motion/constraint_worker.py:58` 构造 → submitted_at 与 now 的单调秒差/预算比较边界 /1000；不将控制周期50ms当预算，不新增超时条件。 |
| `VideoLimits.frame_poll_s`、`push_timeout_s` → `_ms` | 0.1s →100ms；0.5s →500ms | `vision/stream.py:84-86` → source.next_frame(timeout秒) / asyncio.wait_for(peer.push)；调用前 /1000。`max_frame_age_s=2.0` 不动。 |
| `NavigationEffects.timeout` → `timeout_ms` | 0.3s →300ms | `tasks/navigation_port.py:21`，composition 实际创建；resolve/execute/query/progress 全部 wait_for 转秒，保持 effect 失败/unknown 语义。 |
| `MapArchive.capture_timeout_seconds` → `capture_timeout_ms`（可注入接口，未证生产组合启用） | 默认1s →1000ms | `mapping/archive.py:121,252`，仓内构造见测试；asyncio.wait 转秒。不得描述为当前Agent已使用归档超时。 |

### 感知其他可调入口与策略投影

| 当前字段 → 目标字段 | 等时值和入口/消费 |
|---|---|
| `SpawnInferenceWorker.max_skew_s` → `max_skew_ms` | `vision/spawn_detector.py:208,230` 默认0.03s→30ms，校验上界0.2s→200ms，零仍允许；存 `_max_skew_ns=ms*1e6`。`perception_node.py:107,127` 的独立 ROS 参数以及 `vision/synchronizer.py::RgbdSynchronizer(max_skew_s)` 同改新名。不能漏掉这两个不同组合入口。 |
| `SpawnInferenceWorker.max_result_age_s` → `max_result_age_ms` | None 仍继承统一策略1000ms，`spawn_detector.py:212,381` 与调用方上限取min后，和秒测量差比较时 /1000。timeout_s=5.0 保留。 |
| raw/egress `max_age_s` → `max_age_ms`、`hard_limit_s` → `hard_limit_ms` | `vision/raw_capture.py:88`、`egress_receipt.py:152`；max age 多由策略1s→1000ms，hard_limit默认1s→1000ms。部分当前节点显式 hard_limit=5s；它们调用统一新接口写5000ms以保持原校验上限，不能把5000ms误认为新增5秒配置或把5s误缩至5ms。外层独立5秒配置仍秒。 |
| depth/semantic/sensor-current 的策略上限接口 `max_age_s` → `max_age_ms` | `runtime_evidence.DepthAdmission`、`depth_runtime.DepthPairBuffer`、`semantic_runtime/semantic_coordinator`、`sensors.policy.sensor_current`、worker.take_latest 与 raw/egress proofs 等：当前动态观测采用策略1000ms，原始stamp/measurement不变；同源策略传递保持ms，到比较/算法时转换。某些构造只验证传入值后使用全局策略，删除无效输入副本比给它独立新TTL更简单。 |
| `DepthPairBuffer.pair_wait_s` → `pair_wait_ms` | `depth_ingress.py:85` 当前明确传 **统一hard age 1s→1000ms**，不是 YAML 旧0.1s。buffer 等待比较秒时 /1000。`DepthGeometrySettings.tf_max_age_ns` 已纳秒，继续由 hard_age_ms*1e6 推导，不重命名成ms后仍放ns。 |

策略覆盖图还包括 `simulation_camera.py`、`depth_runtime.py`、`semantic_runtime.py`、`rgbd_coordinator.py:150`、`rgbd_spawn_worker.py:433`、`independent_motion.py:322,335`、`motion/estimator.py:576`、`pose_estimator.py:721,860`。这些是转换**消费者**，不是增加一组独立可配置过期阈值的理由。raw/egress 证明函数内现有上界 1s/5s若对应新ms参数，校验数值也必须相应1000/5000。

## 退役、覆盖和无消费项的处理

| 项目 | 当前证据与处置 |
|---|---|
| `scan_adapter.max_age_seconds`；`safety_scan_ingress.observation_max_age`；`scan_motion_node.scan_max_age` | 声明仍在，但节点实际使用统一策略或 `sensor_hard_age_s`。删除声明和 `hardware-safety.yaml`、`simulation_motion.yaml`、`self_filter.yaml` 中副本；同步 `real_runtime` 默认、REQUIRED_PARAMETERS、owned校验和仅为覆盖副本服务的 legacy 循环。不能迁移旧0.5/0.8为新有效阈值。保留统一policy的真实注入。 |
| `depth_ingress.max_age_s/pair_wait_s`、`semantic_ingress.max_age_s` 的ROS/YAML副本 | `depth_ingress.py:67-85` 与 `semantic_ingress.py:66-77` 声明后实际用policy。删节点旧声明、`simulation_rgbd.yaml` 的0.8/0.1/0.3副本；内部 `DepthPairBuffer` 有实际消费的策略派生等待仍保留并迁移，二者不能混为一谈。 |
| `AgentSettings.heartbeat_seconds/command_ttl_seconds/perception_stale_seconds` | `config.py:129-133` 可被通用 YAML/env/CLI 合并并校验，但生产全仓未找到 `load_settings` 调用或这三个字段读取；实际Agent由 composition 自有配置组装。删除未消费字段、专用文档/测试输入，检查通用 env 解析不会继续承认无效键。不要把它们接到Agent、恢复0.2s发送周期或0.5s感知门。 |
| `TrackingConfig.max_detection_age_s/motion_min_baseline_s/motion_max_baseline_s` | `vision/tracking.py:22,26-27,35-49` 仅保存和校验，追踪执行未消费。删除三个无效字段和其校验/构造实参，不将300/150/600ms接进新算法。 |
| `AdmissionConfig.clock_pause_s`、`DepthAdmission.future_wait_s` | 当前类已无字段；schema/现行说明若残留则删除。`safety_node.py:155` 实参按前述实际接口残留处理，不迁移成新配置。 |
| 旧 safety_gate TTL/年龄、command heartbeat/disconnect、运行状态150ms与学习30s预算 | `real_runtime.RETIRED_PARAMETERS` 明确退役的字段及 YAML 草稿应移除；不把旧 disconnect_seconds=2 改5或2000ms来恢复另一条断线链。运行状态/学习预算无新同义名。 |
| `vehicle-motion-acceptance.yaml::feedback_recovery_min_span_s=0.5` | 只有草稿定义、无程序读取；不当作当前生效500ms配置。旧设计5份/0.5s恢复记录留作历史，现行验收草稿若维护则明确标注退役并删除该无消费项，不能联到admission。 |

## 外部秒制、较长配置、未知值与历史材料

- `nav2.yaml` 的 `costmap_update_timeout` 0.3/1、`transform_tolerance` 0.1、`reset_period` 1，`slam.yaml` 的 `transform_publish_period` 0.02、`minimum_time_interval` 0.5、`transform_timeout` 0.2，`senior_sim/config/controllers.yaml::cmd_vel_timeout=0.3`：这些名称及秒值是第三方原生API契约，保持。若后续仓内配置提供ms，明确在仓内生成第三方参数的适配函数 `/1000` 一次，输出仍原名原秒值；本次不为迁移凭空新增第二份输入。Nav2 `bt_loop_duration=10`、`default_server_timeout=20`、`wait_for_service_timeout=1000` 已是毫秒，launch 的同类覆盖也不重复转换；导航overlay继续遵守外部接口。
- 保留 `HEARTBEAT_WINDOW_SECONDS=5`、socket send资源预算5、`snapshot_seconds=2`、`VideoLimits.max_frame_age_s=2`、`TemporalConfig.history_s=2`、`horizon_s=12`、诊断period/repeat5、scan ledger3、学习duration3、spawn timeout5、TURN TTL600、Deadlines.edit5/map120、默认retry上限30；不能因某接口允许传入小于2的值，就动态换单位。
- `_MAX_TRANSPORT_AGE` 不是另一个新配置；它读统一policy并用于秒差。C++ `scan_receipt_admission.hpp` 内部固定 `max_age_seconds=0.5`、TF查询固定0.02、线程join/queue.get/sleep中的固定等待字面量不是本次可调配置入口，不机械改名；不得借迁移扩大成新控制门或新参数。
- `DeviceProfile.reaction_time/observation_max_age=None`、`MappingEvidence.confirm_span_s/clear_span_s/max_age_s=None`、验收文件的未标定窗口和预算 null：未知没有小于2秒的已知配置值，保留原契约及None，不猜默认。`hardware_navigation.max_age_s` 必填、允许(0,5]但无默认，不能只凭范围给出一个新1000ms值。`GeoLocalizer.max_age_s` 必填、未发现本次固定亚2秒生产默认，按实际调用契约保留，不从测试样例推断部署配置。
- `lag_ms` example null 仍未标定，Pydantic硬件模型仍可以拒绝未标定示例；不是要求加载示例成功。`lateral_stop_ms=None` 的仿真消费者仍使用其原来缺界语义。恢复跨度0、max_skew0、lag0和retry0各按原消费者语义，不统称禁用。
- `MeasurementTime.seconds`、`MeasurementInterval.duration`、RawCaptureHeader的stamp/ns、单调deadline、ROS stamp、UTC事件时间、`time_increment`、`missing_linear_s/missing_yaw_s`、`interval_half_s`、`Age.seconds`、`age_seconds`/`body_age_seconds`输出、物理积分dt和速度/Hz保持。历史日志、测试证据、原始测量和已存档replay不批量改值或哈希；现行生成工具与活动配置适配另处理。

## 保护文件依赖与实施范围

本轮三保护文件均未修改、未暂存。实际读取依赖为 `ros2/src/senior_robot/senior_robot/native_receipt.py:25`：`_MAX_TRANSPORT_AGE = default_policy().hard_age_s`，用于176行的silence limit、225行日志、248行年龄比较及374行delivery等待，均为秒。因此若直接移除 `SensorTiming.hard_age_s` 将导致此保护模块导入失败；不能声称仅改JSON/ROS新名就已完成全链迁移。

建议在新policy中区分毫秒**配置字段**与明确的秒制算法只读投影；保留此确实被消费的 `hard_age_s` 只读投影（由 `hard_age_ms/1000` 派生、不可独立输入），使保护文件当前引用有实际用途。它不是接受旧JSON键的兼容别名，不新增第二份配置；实施者应明确文档化这个秒边界。如果交付要求连只读算法投影也不能保留，则负责人需另外安排保护文件接口迁移，本轮不得触碰，不能以新默认属性静默漏迁。保护测试和急停review不需要为本文改动；本轮未发现它们需要新增计时配置键，接口实际迁移时仍须复核。

## 具体测试用例提案（本轮均未执行）

用fake clock、捕获调用实参和现有几何fixture验证等时，不依靠真实sleep撞边界。以下只覆盖用户需求、现有比较规则和上文确认的接口残留；不要增加不在现有校验中的拒绝条件，也不做所有参数的笛卡尔积。

| 用例、具体输入 | 预期与依据 | 可复用位置 |
|---|---|---|
| 策略 defaults 300/1000ms，measured=20s，received=20s，now依次20.299/20.300/20.999/21.000 | normal/low/low/invalid，deadline始终21s；300ms处freshness仍原值，不能以300秒或0.0003秒判断。依据现有 `age()` 的 `>=` 规则及等时需求。 | `tests/unit/test_sensor_timing.py::test_common_boundary`、`test_later_consumption_preserves_original_deadline`；C++ sensor timing消费者测试 |
| 相同原始采集20s，接收20.1s，分别20.2/20.9消费，后续重打包 | deadline均21s，不以消费时刻重新续1000ms；已有时钟契约，不改年龄/绝对时间单位。 | `test_sensor_timing.py`、`test_reference_nanosecond_boundary.py`、`test_native_sensor_age_boundary.py` |
| 实车profile `lag_ms=500,lateral_stop_ms=1000,horizon_s=12`；仿真lag_ms=200 | 加载schema与模型一致；传入停车算法的lag仍0.5/0.2s、lateral仍1s，同一现有运动fixture输出的停车距离/时间与迁移前等价。依据实际配置和物理量纲。 | `test_motion_launch_profile.py`、`test_directional_constraints.py`、`test_stopping_fused_input.py`、现有profile加载测试 |
| profile `lateral_stop_ms=13000,horizon_s=12`；lag_ms=0；example lag/lateral=null | 第一项按原上界拒绝；lag零保持原允许语义；example null保持未标定且硬件模型原本不接受就继续不接受，仿真None仍None。依据已有coherent及ge=0规则。 | 同上及 `test_sim_contract.py` |
| 配置生成器输出300/1000ms，ROS matcher max_interval_ms=400；C++结构默认300 | 节点收到新参数名和值，source策略不被旧0.5/0.8副本覆盖；同样scan时间差0.4s允许、0.400001s按原 `>` 拒绝。默认结构单独仍0.3s，不和ROS组合混淆。 | `test_motion_launch_profile.py`、`test_scan_matcher_replay.py`、`ros2/src/senior_scan_matcher/test/test_scan_matcher.cpp` |
| 同一原始scan测量记录，经新工具配置max_interval_ms=400写入replay | C++读取限值和节点一致，raw stamp秒/纳秒不变，同一记录匹配结果等价；归档源文件哈希不变。依据两个replay当前有真实配置消费。 | `tools/scan_matcher_replay.py` 的现有 `test_scan_matcher_replay.py`、ROS matcher `test/test_replay_cli.py` |
| SourceRule interval400ms，span=0.4/0.400001s；Temporal hold/gap1000ms，跨度1/1.000001s；recovery_span_ms=0 | 等号沿当前规则允许、超过拒绝；零恢复不添加等待，history_s仍2。用现有合法观测fixture，避免编造缺轴异常。 | `test_motion_sources.py`、`test_motion_temporal.py`、`test_relative_interval_regressions.py` |
| CaptureLedger age_limit_ms=1000，同代同seq合法header，capture=20s，now=21/21.000001 | 沿原 `>` 规则到1s仍可匹配、超过后retired；捕获ns不变。此处与公共policy的 `>=` 失效边界不同，单位迁移不得擅自统一规则。 | `ros2/src/senior_sensor_ingress/test/test_capture_ledger.cpp` |
| fake loop从0开始，period_ms=50，一轮工作80ms，回调时刻0.080/0.099/0.100 | 过期50ms槽跳过，100ms仅新一轮，不补算、不新增150ms失效门；保持已确认调度行为。 | `test_control_cadence_gate.py` |
| worker_budget_ms=250，合法flight submitted_at=20s，now=20.25/20.250001s | 秒制预算仍0.25，沿当前 `>` 规则等号未超预算、超过判worker_budget_exceeded；结果不穿透新快照；不新增每周期预算。 | `test_motion_core_handoff.py`、`test_motion_core_chain.py` 及现有constraint worker测试 |
| Agent/ROS command_ttl_ms=300，收到意图10s | IPC received=10、deadline=10.3秒；到期按现有owner比较规则停车，连接身份未改变，5秒在线窗口不续手动TTL。 | `tests/integration/test_manual_wire.py`、`test_intent_ipc_stream.py`、`tests/unit/test_degraded_manual_speed.py` |
| heartbeat_ms=1000；最后有效heartbeat在0s，fake now4.999/5/5.001；持续fresh manual/已接受任务fixture | 发送周期仍1s，online前真后假（5到点过期），延迟3s回复不在1s误断；自主任务和手动断流仍按现有独立语义。依据已实施心跳10需求。 | `test_heartbeat_wire.py`、`test_websocket_link.py`、`tests/integration/heartbeat_test_support.py`及心跳现有集成用例 |
| Agent state_poll_ms=250、Deadlines.safety_ms=500、supervisor shutdown_timeout_ms=1000；捕获sleep/wait timeout | 调用实参0.25/0.5/1秒；snapshot仍2、edit仍5、map仍120。状态读取一次IpcError后下一轮继续、后台局部故障只上报，依据故障隔离11当前实现。 | `tests/integration/test_agent_snapshots.py`、`test_runtime_fault_isolation.py`、`test_shutdown.py` |
| retry_delay_ms=1000,max_retry_delay=None，fake connect连续不健康；另一次retry_delay_ms=0 | 等待仍1,2,4,8,16,30秒；零延迟仍零；健康恢复后回到1秒。依据当前supervisor `_uplink_loop` 和None推导，不新增重试。 | `tests/integration/test_runtime.py`、`test_agent.py` |
| IpcClient timeout_ms=1000；helper500；navigation原3秒调用 | 捕获wait_for分别1/0.5/3秒；超时保持原IpcError/未知结果语义，不重放intent。确认所有调用改新接口时3秒为3000ms。 | `tests/unit/test_ipc_frames.py`、`tests/integration/test_ipc_lifecycle.py`、`test_runtime_fault_isolation.py` |
| NavigationEffects timeout_ms=300；MapArchive capture_timeout_ms=1000 | 捕获各resolve/execute/query/progress为0.3秒、archive wait为1秒；沿原未完成/取消结果断言，不造新超时恢复规则。 | `tests/unit/test_direct_navigation.py`、`tests/integration/test_task_runner.py`、`tests/unit/test_map_archive.py` |
| Video frame_poll_ms=100,push_timeout_ms=500,max_frame_age_s=2 | source收到0.1s timeout，push收到0.5s，frame年龄2秒的原比较结果保持；不将max age变2000秒。 | `tests/integration/test_video.py`、`tests/unit/test_simulation_video.py` |
| 两RGBD帧stamp相差30,000,000ns/30,000,001ns，max_skew_ms=30；再max_skew_ms=0的同stamp | 沿当前同步器边界配对/拒绝，零允许精确配对；spawn与ROS synchronizer同等时。None result age继承1000ms，不能变0。 | `test_rgbd_sync_ordering.py`、`test_spawn_detector.py`、`test_rgbd_runtime.py` |
| DepthPairBuffer pair_wait_ms=1000，深度采集20s，无info，fake age沿现有buffer到期点推进；DepthAdmission只传max_age_ms=1000 | 等待仍policy1秒而不是YAML旧0.1秒；真实SafetyGate构造路径不再传已删除future_wait关键字。依据当前接口残留及策略覆盖。 | `test_depth_producer.py`、`test_rgbd_runtime.py`；在SafetyGate现有构造替身中捕获真实DepthAdmission调用 |
| 生成第三方参数/overlay读取当前Nav2/SLAM/controller配置 | 外部名称及秒值逐项保持0.3/1/0.1/1、0.02/0.5/0.2、0.3；Nav2既有毫秒10/20/1000不再乘1000，Hz与stamp不变。 | `test_navigation_overlay.py`、`test_sim_contract.py`、`test_motion_launch_profile.py` |
| 迁移后新JSON入口给旧键lag/lateral_stop_s/可编辑max_age；旧无消费ROS/settings/tracking键 | 按相应严格加载器的既有未知字段策略拒绝或不再输出，不存在旧键和新键冲突优先级；无效副本不通过新名复活。检查产品配置、schema、生成器、现行文档一致，不对历史证据做“零旧名”要求。 | profile/schema加载测试、`test_settings.py`、`test_motion_launch_profile.py` |
| policy新JSON仅300/1000ms，保留秒制只读算法投影，导入native receipt | `_MAX_TRANSPORT_AGE`仍1s、既有1秒等待/日志值不变，保护文件字节不变；如果不保留投影则本用例明确阻塞待独立接口任务。依据实际保护依赖。 | 后续允许的 `test_native_sensor_age_boundary.py` /现有导入边界；本轮不编辑、不执行保护测试 |

## 实施建议与本轮验证限制

先确定新配置名与唯一单位，更新policy/schema/真实及仿真入口，再沿上表实际消费者补边界；清理无消费/覆盖副本同时完成，不保留一个可输入但无用的旧值。最后全仓搜索旧名，区分允许的秒制算法投影、外部接口、历史证据和遗漏；以具体消费者判定完成，不能仅靠改名数量。

实施后集中执行涉及的Python定向测试、schema再生一致性、C++ matcher/ledger与安装头消费者构建，ROS实际参数生成和依赖环境可用时再验证构建；未运行或环境跳过必须逐项如实记录。本轮只读取源码/配置/设计与测试位置，未执行上述用例，未宣称ROS、Linux或车辆验收通过。
