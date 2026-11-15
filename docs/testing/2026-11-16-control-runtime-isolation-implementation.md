# 车端控制隔离实现记录

这是初版历史记录；exact-scan-key、pending学习及rate/output硬门已由[审阅返修记录](2026-11-16-control-runtime-review-fixes.md)覆盖，以该记录的锁定指纹和同版测试交接为准。

状态：**代码修改完成，统一测试待执行**。工作分支仍为 `fix/degraded-drive-speed`，HEAD／开始基线为 `ab241065e9446e933155bff5b1b58badc6d5e2cd`；未提交、暂存或推送。

## 实现与边界

- 删除 `model_seconds`、启动 `model_deadline`、ResponseBounds／RiskEnvelope 的静态截止及所有对应消费者。配置／schema 同步删除，不使用无穷期限或周期续租；内部 idle／启动生命周期目标显式没有采样指令期限，外部运动指令仍必须有原始有限截止。
- 删除 MotionRuntimeState.valid_until、RuntimeObservation.expired、runtime_healthy 及命令／任务入口的报告缺失、报告健康、报告 allowed_actions／stationary／manual mode 副本准入。恢复请求只做身份与版本翻译，最终 MotionStateMachine 检查当前轮速／IMU证据；显式急停交付、身份隔离、序号与幂等仍保留。接上通信 worker 的 manual_intent 单向 IPC 和独立 intent_client，驾驶配置来自原子 published_snapshot。
- 删除 ControllerConfig.max_tick_s 及固定 150ms 拒绝条件。ControlCadence 使用单调时钟绝对50ms槽位：一次计算0→80ms只让100ms槽开始下一轮，不补算50ms。单线程控制 owner 不允许并发 tick，控制器仍拒绝非有限、非正 dt／倒退时钟，并使用实际 dt。
- 接收与估计、扫描过滤/patch/union、方向细化各用独立 spawn 进程，绕开跨任务 GIL 串行。接收快照和每个几何任务/结果使用固定8MiB槽；报告槽128KiB，诊断状态槽32KiB，忙锁读取/写入不等待；字节快照复制后才反序列化，本地副本不与生产者共享可变对象。每个几何 worker 仅一个在途，待办只覆盖最新；空间/来源/扫描/完整过滤模型版本不同即丢弃，不刷新原证据时间。
- 原始观测统一0.3s软降可信、1s硬失效及命令TTL不被本次静态租约删除改变。5ms轻量唤醒在控制槽之间预取扫描几何，避免必然多等一个控制周期；pending 时归零，但不将计算pending冒充反馈丢失去永久撤销尚有效目标。学习运动已经撤回时仍不重放。
- `safety_gate` 内 MotionStateMachine 是唯一软件急停内存 owner；新增原生 `motion_output_guard` 只做私有传输和原决策证据截止检查，无急停锁存。它独占 `/cmd_vel`，父控制线程停顿、IPC EOF／溢出或原截止失效时归零；EOF 后退出。完整启动不恢复旧指令或旧软件急停文件，硬件急停解耦保持。
- 独立 `motion_reporter` 进程持续发布 `/motion/runtime` 和 `/safety/status`，保留原控制采样时间；接收/控制 executor 不执行报告 DDS 发布或日志 pump。命令接收只核对 `motion_reporter` 来源与序号；报告不反向撤销手动授权。
- 硬件/仿真 launch 删除无人消费的 motion_fusion 重复节点，wheel_frame 迁入 safety_gate，硬件 `/odom`、`/imu/data_raw` remap 传给接收进程。旧 YAML 只迁移实际 wheel frame，代码明确标注 `//Compatible`；控制核心旧本地接收/重新估计、core_reference回放及空消费入口直接删除，唯一输入为接收进程快照。

## 静止唯一规则与实际信号

`motion/stillness.py` 唯一判断新鲜、真实观测的轮速纵向速度、轮速角速度和 IMU yaw rate 是否在现有 profile 阈值内；缺失、无观测、非法及到期均不是零。当前硬件草稿实际阈值均为 `0.05 - 0.02 = 0.03`（m/s、rad/s），仿真复用现有 `0.02 m/s`、`0.03 rad/s`。IMU 在现有运动适配接口中提供角速度，不新增未标定的加速度阈值，也不把其占位 vx=0 当平移观测。

急停复位、故障恢复、状态机手动/任务转换、控制器换向与停车完成、学习前后以及驾驶设置的静止消费者共用此证据；独立激光/GPS/视觉/融合body健康不参与。删除控制器旧 limited_rearm／累计独立静止路径及其配置字段，学习初始化不再另设0.01/0.02阈值或连续5次门。原故障恢复的连续新健康样本条件仍计数真实轮速/IMU，学习的分类前后5帧要求属于未填写第7项，保留不改。

## 实际执行资源与等待位置

```text
safety_gate（Python，唯一控制决策/软件急停内存 owner）
  ├─ MotionExecutor：5ms取最新输入/预取，50ms绝对槽单个tick
  ├─ motion-diagnostics thread：独立日志pump
  ├─ spawn motion-reception：独立MotionExecutor，原始接收＋估计＋快照
  │    ├─ native_receipt：沿用现有接口，不修改该文件
  │    └─ motion-diagnostics thread
  ├─ spawn motion-scan-geometry：射线过滤／patch／union
  ├─ spawn motion-geometry：方向候选细化
  ├─ spawn motion-reporting / ROS motion_reporter：状态与诊断发布
  ├─ native final_command_guard：原命令身份/连接来源边界
  └─ native motion_output_guard / ROS motion_output：唯一速度发布与有限截止兜底
```

正常路径不等 Future、不等云网络回执、不排队补算；共享槽 try-lock 冲突跳过。复制/序列化、控制槽最终空间误差收紧、粗停车与输出证书 GEOS 核验仍在控制进程，有界输入不等于已测得最坏耗时；日志元数据短锁、DDS发现/命令图检查亦可能花时间。未固定核编号、未要求root、未宣称20Hz硬实时；长耗时跳槽与独立输出截止保护同时存在。

ExitStack 处理部分初始化失败及逆序清理；正常关闭先撤销/发送零，再停止接收、报告、几何、命令guard与输出传输。子进程按1s join、terminate、kill并join，Linux使用无需root的parent-death signal及竞态检查；native输出通过EOF归零，私有通道不复活旧速度。

## 静态结果及未验证限制

- Python3.10 AST：自有源码通过；另对全部 `src/senior_control` 和 ROS Python包做过AST解析。
- 自有Python文件 Ruff check：通过；配置JSON解析及schema生成通过。
- 保留 command_node.py、agent_adapter.py 原CRLF；`git -c core.whitespace=cr-at-eol diff --check` 通过。普通 diff --check 会将这些保留的CRLF新增行视作尾随空白，未为消除该提示重写整文件换行。
- 未运行测试套件、ROS、开发服务、仿真或车辆，没有运动命令。新增C++ target 尚未在ROS Humble/aarch64构建；需要重新构建/安装 senior_motion_msgs 后才有 motion_output_guard，缺失即启动失败，不回退旧输出路径。
- spawn ROS/消息可序列化性、退出信号、DDS发现及真实负载调度未实测。严格扫描版本准入在慢计算/更高输入频率下可能持续归零，必须验证持续正常输入的可用性、丢弃率和有限时间内有效输出；不能以放宽旧数据截止掩盖。
- 最终同步核验最坏耗时未测；原生输出进程自身被杀、DDS/驱动断流及物理固件watchdog仍需部署验收，profile.watchdog_mode保持unknown，不声称物理停止时限已验证。
- native_receipt.py、tests/test_native_receipt_diagnostics.py 两份叫停改动及 docs/reviews/2026-11-16-software-estop-owner.md 既有改动未编辑、暂存或覆盖；通信worker文件不列入本报告自有范围。

## 统一测试迁移清单

1. contracts/controller/trajectory/directional_constraints/tick_evaluation/stopping_fused_input：移除 ResponseBounds／RiskEnvelope静态期限、ControllerConfig.max_tick_s/still_samples/still_span、PreparedControl.limited_complete 构造；真实观测、感知、意图期限断言保留。内部无期限目标只用于idle/受控启动生命周期。
2. runtime_reporting/runtime_guard_ready/manual_wire/motion_contracts、相关JSONfixtures：移除runtime.valid_until/expired/150ms准入及drive.intent逐条回执断言；增加无报告仍能接收合法目标、报告旧healthy/allowed_actions不阻塞入口、独立来源/序号和显式急停交付边界。
3. motion_controller/limited_manual_rearm/limited_motion_core/software_estop_owner/motion_core_startup：用显式原始still_wheel和IMU取代body静止；无激光/GPS/视觉仍可静止/复位/故障恢复；任一轮速或IMU运动、缺失、非有限、到期时不静止。保留唯一owner/硬件解耦/不重放旧动作回归。
4. motion_core_event_time/exit_deadline/reference_backlog/sources/constraint_scheduling：迁移异步资源注入与publisher身份，补0→80→100ms、连续过载只单个在途、实际dt、非有限/倒退时钟、最新快照一致性、忙锁/容量上限、扫描/过滤/来源版本换代、过时结果不可放宽及收到新障碍即撤销。
5. motion_core_acceptance 与部署preflight：验证motion_fusion退役、wheel_frame/remap、motion_reporter身份、唯一motion_output writer；本地输出原证据截止/父进程退出/IPC断开或溢出归零，不新增急停锁存或旧运动恢复。
6. 负载与耐久：接收洪峰＋慢几何＋日志/报告/视频/任务负载，多核并行证据与每条执行资源阻塞位置；大于1小时运行无静态模型失效，动态数据仍到期；高频持续正常扫描必须能有限时间产出有效许可。
7. 启动学习/静止/模式切换：现有噪声阈值内初始化、学习后停稳、急停复位、故障恢复、手动/任务转换、设置静止检查一致；学习已撤回不重新前移、完整重启不恢复旧目标，分类参数（第7—15未填写范围）不修改。

具体待迁移文件包括 `tests/unit/test_motion_controller.py`、`test_directional_constraints.py`、`test_tick_evaluation.py`、`test_stopping_fused_input.py`、`test_limited_manual_rearm.py`、`test_limited_motion_core.py`、`test_motion_core_startup.py`、`test_motion_core_event_time.py`、`test_motion_core_exit_deadline.py`、`test_motion_reference_backlog.py`、`test_constraint_scheduling.py`、`test_software_estop_owner.py`、`test_runtime_guard_ready.py`、`tests/integration/test_manual_wire.py`、`test_runtime_reporting.py` 和 `tests/ros/test_motion_core_acceptance.py`。本阶段均未修改或执行。

## 自有文件及指纹

产品文件清单共36项，另有本实施记录和实施计划两份自有文档。完整实际diff保存在 `/tmp/control-runtime-owned.patch`（包括新增文件），SHA256 `72c2b6597ba961dcc2418e1df09588fa5bd21f69d5de09df27d546eb0e663909`；自有文件内容清单 `/tmp/control-owned.sha256` 的SHA256为 `21f44f262fa35531b17575a5da9eb24a1441843581a5af7654a8f054fb9d018c`，用于标识本轮未提交产品源码。

```text
1b8c304d8165900b4ff7b7b6f6c4a2f599a23ea8d1a64c245579f14e0f1c2af5  config/hardware-motion.json
112b3ff30b0deb04924295db260ffae6ee112156643b9235b55f290d68b060ed  config/hardware-motion.schema.json
08fe48e7b77ae24c5c6f7a6afcd096cb67554cfcb15963f50a963e0657ffed7e  config/hardware-safety.yaml
71a8fdb5312e5cdc90c47c821556303c787c925c19db9cf8a998447c2fab3528  ros2/src/senior_motion_msgs/CMakeLists.txt
17191d9ed0be7368c69407df03cece6413caf3827ec4a201d49190a14c5f3982  ros2/src/senior_motion_msgs/src/motion_output_guard.cpp
3a6c099c20b8ba48d5f7cc2458fa3873c9f1cc8765ff9fc4f058588858feec43  ros2/src/senior_robot/config/simulation_motion.yaml
3ee38bb2068843b3a1dfd379446b70901aa97439db7c553cce85e7284df9c755  ros2/src/senior_robot/launch/safety_motion.launch.py
b5f9359dab44be03b0a5579b17d9eb388332776fd4afcb32a20e61e06053aa53  ros2/src/senior_robot/launch/simulation_motion.launch.py
e2d07c5f8d29d1d2abe6bb798c456cc65bdc6ae6f646e8e07265a405d45b6dd9  ros2/src/senior_robot/senior_robot/agent_adapter.py
320bd2381e3681b2fbceeff9efe03224e39f1192a648b5cd9942397be940b277  ros2/src/senior_robot/senior_robot/command_node.py
3a10e11adca4d4cd4df1c9249890daef57567c87ecafc80a9b4712bd931499ce  ros2/src/senior_robot/senior_robot/control_inputs.py
0ec5b7f9561ac05391f43e64fdaca014131719f26edafccc047ab6d1162bc07d  ros2/src/senior_robot/senior_robot/motion_core_output.py
d77d42ccec4584900f1b35136b0e79f9ae20fae1e3c5b9a19fb60bc1c50319e1  ros2/src/senior_robot/senior_robot/motion_core_perception.py
b0aebc985722697c47d8276f4067d8671886d92883ac8dc8a14e5f9bf72f7f46  ros2/src/senior_robot/senior_robot/motion_output.py
8e98a5d55314ce2a63afef5f4785de46b88f9c0a1888f6bdb208494d45e33424  ros2/src/senior_robot/senior_robot/motion_sensor_runtime.py
191b6946da84f33e6cf09f9b21b104a01bda8d69456490b814ba05a119ecaa94  ros2/src/senior_robot/senior_robot/runtime_reports.py
ce25121fd9caeda0f896c70029c32f0b7487745a2af564c017e8352c46e3aac0  ros2/src/senior_robot/senior_robot/safety_node.py
ad78ab5fca33825f2fe7ff542c9d32b81f008d15f6d10eef2c6e5add48e6c346  ros2/src/senior_robot/senior_robot/simulation_motion_profile.py
4ca31643a426f523d5b1b1fec6c0ab301570b24bc304caada475b9fca1fab344  ros2/src/senior_robot/senior_robot/simulation_reference.py
a13a3a33a34450806bba8586d4c8026286df0fb4e97f7e9aaaca120eb4441d40  src/senior_control/agent/composition.py
15efbfe6039c8f4c8645d8ddc4044126813a8f31da2b03de3cab02715ef5e6cb  src/senior_control/agent/manual_control.py
d28ba050ba2933bc2be267bafb28a1b2b4f5575548ad1691312f3ddad83cfaa4  src/senior_control/agent/ports.py
1b7c61aa9d072ea30409cb9e914718d898376c1cfa5d97dc47d70284b7ee9fae  src/senior_control/agent/settings_wire.py
b6931b7e52a66015b89802648f951555b60afb7b175f8b5b4b235fe0b4c75dc5  src/senior_control/motion/cadence.py
a524a29de4488959be46f5a260d6913e9ec1e8c8f0c4c0e89c3cec7d3ed5cec0  src/senior_control/motion/constraint_worker.py
76fd8f1ef62223d8a4d0b33397b16334e3f150cc301c5cca0b13da1e948b1dab  src/senior_control/motion/constraints.py
426aab3acabe101a393bd167d45fb24c0b8c92b171f00a428a51700c828f27fb  src/senior_control/motion/contracts.py
54573d7b947fe1bdfc6a2bceb31afaf0eb2597c4c89e7d4853f4002115cd8cd1  src/senior_control/motion/controller.py
86c89d5c17a1d4a225cf4c59d29e0cde6e9bc6e158e680b71d3873b4592531e2  src/senior_control/motion/diagnostic_pump.py
1f03d81f11f21ce5aa7ace784daf5c70bff3c3b281de14539ddaae4af2b846c6  src/senior_control/motion/hardware_profile.py
7533a81757a01c2333a1a08b56b1b2a8e280af8f9f05d2f0b4e7eb3aafc63f24  src/senior_control/motion/process_resources.py
98660c75773e4a3c5420fb073196ffd5101aa6ff87ba47271dfbf77877ad5ac8  src/senior_control/motion/startup_learning.py
740fc6aac11e6dab61c954f721f5cc9d81cc6875a45d3367d5031ccf0354b76e  src/senior_control/motion/state_machine.py
7afb168b66ca5c8567bac3686327041f7702385ed43e309d518343b1a923146d  src/senior_control/motion/stillness.py
ba909e96ec97343b2af2299296e71068cffcdae4172230ef226dca0507862a72  src/senior_control/protocol.py
b64e6a7687128ccc699ea60d232c56bb22a0935b41e4531826073630fc9d7e96  src/senior_control/real_runtime.py
```
