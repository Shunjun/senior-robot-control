# Senior ROS 2 远程控制系统实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付可在Linux仿真中通过网页控制、并可适配senior_4wd_dl实车的多车远程控制系统。

**Architecture:** 厂商驱动保持独立，ROS节点通过适配器连接可单测的车端业务核心。云端负责单管理员认证和中转，车端拥有地图、Home与任务权威数据；所有运动经过唯一安全输出。先建立仿真和安全闭环，再完成导航地图、远程服务、网页视觉，最后做端到端验收。

**Tech Stack:** Ubuntu 22.04 / ROS 2 Humble（部署前检测）；Gazebo Fortress与对应ros_gz桥作为首个仿真目标；Python 3.10+、FastAPI、Pydantic、SQLite、pytest；React、TypeScript、Vite、CSS Modules、Vitest、Playwright；SLAM Toolbox、Nav2、WebRTC/aiortc、coturn、可选YOLO推理。解析可用依赖后生成锁文件，不将未验证版本写成已支持。

**Spec:** ../specs/2026-10-27-robot-control-design.md（包含最新自车遮挡要求）。用户已授权据功能清单编写本计划，尚未执行产品实现。

## Global Constraints

- 保留厂商底盘与传感器驱动；原目录不修改。工作目录为senior-robot-control。
- 单管理员、多台小车；不实现注册、角色、组织或多租户。
- 任务和Home在车端持久化；服务器读取。断线停车，不启动任务；重连不自动续跑或补跑。
- Home默认首次上电位置，跨重启保留；无可信初始位姿不能写入假Home。
- 单天线GNSS航向只利用正常前进被动校准，模块不得产生运动指令。
- 稳定障碍确认后写入；可靠自由空间确认后清除；移动对象与自车回波不写环境地图；遮挡不能视为空闲。
- 后方局部遮挡允许符合设计第4.2节的历史地图辅助受限慢退；不得把无回波当空闲。运动判断不单靠轮速odom，需独立雷达/IMU等观测与打滑/受阻检测。
- 未经要求不启动开发服务。Linux仿真是交付要求，目前不连接或操作用户虚拟机。
- 兼容代码明确标注//Compatible，Python使用# //Compatible；超过三个props定义独立类型；局部样式使用CSS Modules。
- 真实设备与模拟状态分开；macOS测试不得宣称ROS仿真或实车已通过。

## Review Focus

1. 半开网络连接、重连与旧页面竞态：认证心跳失效必须撤销运动资格，旧连接不能恢复控制（任务5、13）。
2. 自车附件遮挡与无效测距：不得生成穿透附件的空闲空间，也不得误放行倒车（任务3、4）。
3. 回环、栅格扩展与地图包来源：地图/任务/Home不能跨参考误用，错误导入不能破坏旧数据（任务8、9、10）。
4. 任务执行落盘与崩溃竞态：不能重复发导航，结果未知需人工核查（任务12）。
5. GPS假精度与视觉负载异常：不误纠正朝向，不将缺失测量视为空闲，不阻塞安全停车（任务11、15、16）。

## 文件与模块边界

所有下列路径相对项目根目录。Python统一包src/senior_control，按功能拆模块；ROS包装位于ros2/src/senior_robot，仿真位于ros2/src/senior_sim；网页位于web。tests/unit测试纯业务，tests/integration测试本进程ASGI/数据库与适配器，tests/ros测试Linux ROS，tests/e2e测试真实运行的仿真闭环。

- src/senior_control/{protocol,config,clock}.py：消息、设备配置与时钟。
- src/senior_control/safety/：几何、覆盖、自车过滤、仲裁、保护与脱困。
- src/senior_control/mapping/：长期占据、观测关联、参考版本和地图包。
- src/senior_control/localization/：地理配准、重定位和被动航向观测。
- src/senior_control/{waypoints,tasks}/：Home、任务、数据库和调度。
- src/senior_control/{cloud,agent,vision}/：云端、车端连接和视觉。
- ros2/src/senior_robot/{launch,config,behavior_trees,senior_robot}/：ROS适配、唯一速度链、后端桥和启动。
- ros2/src/senior_sim/{urdf,worlds,launch,config}/：仿真模型、场景与桥。
- web/src/{api,components,features}/：协议客户端及设备、驾驶、地图、任务、视频交互。
- deploy/：TLS反向代理、服务容器、TURN与车端systemd配置。
- docs/{protocol,operations,testing,training}/：协议、操作、验证和训练资料。

## 共享契约与规划决定

任务1在protocol.py中集中定义以下模型，后续任务只扩展其所属内部实现：Velocity(vx,wz)、MapGoal(pose,map_ref)、QualifiedFix(latitude,longitude,altitude,covariance,stamp,fix_type)、Waypoint(id,kind,pose,map_ref,anchor_id,source_robot_id,status)、RobotSnapshot(robot_id,connection,localization,map_ref,home,tasks_revision,captured_at)。二维底盘禁止非零横向速度。

扫描与地图证据类型定义在safety/self_filter.py与mapping/evidence.py：ScanObservation包含量程、角度、时间、frame和外参；FilteredObservation显式区分命中、真实自由射线、遮挡和无效区域；MappingObservation包含关联关键帧与动态掩码。Localization、Navigation、Mapping、Task适配接口使用可注入时钟和可替换ROS客户端，测试不能依赖真实网络或真实sleep。

命令类型固定为control.acquire/release、drive.velocity/stop、safety.estop/reset、navigation.start/cancel/resume、localization.initialize、waypoints.list/upsert/delete、home.get/set/return、tasks.list/get/upsert/delete/enable、maps.status/export/import/clear、operations.get、video.offer/ice。修改请求带expected_revision；上传文件通过鉴权HTTP分片上传接口传递，WS只传文件引用与校验和。大包不进入遥控WS队列。请求和响应均包含robot_id、request_id及明确错误码，长操作返回operation_id。

跨主机expires_at需要云车时钟偏差估计和有界误差校验；不能直接将浏览器墙钟与车端单调时间比较。设备依据认证连接代次、序号、经验证的发送时间和本地接收单调时钟计算最迟执行期限；时钟误差超出配置限额则拒绝驾驶并提示同步。测试覆盖旧命令延迟到达但仍处于同一连接的情况。

任务中断只影响该执行实例，不自动补跑或恢复。以后按计划正常到达的新触发时刻，在所有条件满足时可创建新的执行实例；显式停用任务则所有后续触发均禁止。间隔/cron调度与执行时钟分离，时钟倒退不可产生重复实例。

首次目标平台为Humble/Fortress；其他已有仿真只通过显式适配接入，不预先承诺Classic或其他ROS版本完全兼容。新增根目录config/simulation.yaml、config/hardware.example.yaml保存车端/测试配置，任务2创建并由任务20文档引用。

## 执行与验证约定

每个任务按“失败测试→确认失败原因→最小实现→验证→提交”执行；不得用无关依赖错误代替预期行为失败。测试示例中的类型和夹具由该任务创建，并在测试文件中定义，不要求预先存在。API返回模型与错误码在任务1集中定义，后续不得自行改名。

Python测试命令统一`.venv/bin/python -m pytest`；任务1创建环境并安装开发依赖。ROS测试在Linux执行，先source ROS与colcon工作空间；缺少Linux环境时记录未执行，不将静态检查替代运行验证。网页单元/构建不启动服务；需要浏览器或仿真进程的调试测试遵守禁止擅自启动开发服务的要求，实际启动前确认已具备相应执行授权。

项目目前不是Git仓库。实施开始先读取using-git-worktrees，确认新目录独立后初始化本地Git并提交已确认设计与计划，不创建远程、不推送。各任务提交仅包含明确列出的相关文件，失败不得提交为完成。

以下签名是接口契约；数据模型字段详见任务1。外部服务操作均异步且有超时、operation_id和结果状态。实现不得把NotImplemented、假数据或永远成功的适配器当成交付。

---
## 阶段一：仿真与安全基础

### Task 1: 协议、配置与可测试工程

**Files:**
- Create: `pyproject.toml`
- Create: `src/senior_control/{__init__,protocol,config,clock}.py`
- Create: `tests/unit/test_protocol.py`
- Create: `docs/protocol/v1.md`
- Create: `.gitignore`

**Interfaces:** 定义Envelope(v,robot_id,session_id,connection_epoch,request_id,seq,expires_at,payload)、Pose2D(x,y,yaw)、MapRef(map_id,reference_revision)、SensorHealth、OperationResult；配置统一使用米/弧度/秒。validate_envelope(raw: dict) -> Envelope；Clock.monotonic() -> float；DeviceProfile包含footprint、话题、TF与传感器覆盖配置。

- [ ] **Step 1 — 写失败测试：** test_reject_nonfinite_and_wrong_robot：NaN/Inf、非法枚举、未知协议版本、错误robot_id和超大消息被拒绝；test_clock_is_injected：使用假单调时钟无真实sleep。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 建立可编辑安装与dev依赖锁；写严格输入模型和配置加载。将心跳周期、失联阈值、命令TTL、感知过期阈值集中配置并验证约束，先作为仿真参数，不能标称实车制动保证。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_protocol.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: define robot protocol and typed configuration"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 2: Linux仿真底盘与驱动适配

**Files:**
- Create: `ros2/src/senior_sim/{package.xml,CMakeLists.txt,urdf/senior.urdf.xacro,worlds/baseline.sdf,launch/simulation.launch.py,config/bridge.yaml}`
- Create: `ros2/src/senior_robot/{package.xml,setup.py,resource/senior_robot,senior_robot/__init__.py,config/hardware.yaml}`
- Create: `tools/detect_environment.py`
- Create: `config/simulation.yaml`
- Create: `config/hardware.example.yaml`
- Create: `tests/ros/test_sim_topics.py`

**Interfaces:** simulation.launch.py(use_sim_time=true, world, headless)启动差速模型、雷达、RGB-D、IMU、轮里程计与独立world truth；detect_environment() -> dict报告ROS/Gazebo/设备/话题，不启动硬件。输出/scan、/camera/color/image_raw、/camera/depth/image_raw、/camera/*/camera_info、/wheel/odom、/imu/data、/ground_truth/odom、/clock。

- [ ] **Step 1 — 写失败测试：** test_sim_sensor_contract：时间戳、frame与消息类型正确，/ground_truth/odom独立于轮里程计且不发布truth TF；test_watchdog_stops：命令停发后仿真底盘速度归零。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 检测Humble/Fortress包可用性并固定桥消息映射，保留既有仿真接入launch参数。模型采用厂商轮位与外形，碰撞几何可简化但不得小于已确认车体。底盘看门狗与车端安全门独立。输出版本矩阵。
- [ ] **Step 4 — 验证：** `colcon build --packages-select senior_sim senior_robot && python3 -m pytest tests/ros/test_sim_topics.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add Linux senior robot simulation and sensor contracts"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 3: 自车过滤与可观测覆盖

**Files:**
- Create: `src/senior_control/safety/{__init__,self_filter,coverage}.py`
- Create: `ros2/src/senior_robot/{senior_robot/scan_adapter.py,config/self_filter.yaml}`
- Create: `tests/unit/test_self_filter.py`
- Create: `tests/unit/test_coverage.py`
- Create: `ros2/src/senior_sim/worlds/rear_occlusion.sdf`

**Interfaces:** filter_self(scan: ScanObservation, profile: DeviceProfile) -> FilteredObservation；结果分别包含external_hits、free_rays、occluded_regions和invalid_regions，不用range=Inf编码被过滤点。coverage.supports(sweep: SweptRegion, observations: list[FilteredObservation], now: float) -> CoverageResult。

- [ ] **Step 1 — 写失败测试：** test_attached_occluder_never_creates_free_ray：附件回波被排除但其后方保持未知；test_nearby_wall_not_self：附件外近墙仍保留；test_reverse_requires_realtime_or_explicit_degraded_policy：仅前向雷达不能直接获得后方实时空闲资格；满足历史地图降级策略时才允许受限慢退。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 基于URDF/实测附件体积与固定扇区建立车体系遮挡；显示原始/过滤扫描和覆盖。不能按长期跟随回波自动删除；外参无效即拒绝相关保护资格。深度传感器补盲按同一覆盖接口融合。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_self_filter.py tests/unit/test_coverage.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: model self occlusion without inventing free space"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 4: 矩形扫掠碰撞与独立安全门

**Files:**
- Create: `src/senior_control/safety/{geometry,guard,motion_estimator,slip}.py`
- Create: `ros2/src/senior_robot/senior_robot/safety_node.py`
- Create: `tests/unit/test_guard.py`
- Create: `tests/unit/test_geometry.py`
- Create: `tests/unit/test_motion_estimator.py`
- Create: `tests/unit/test_slip.py`

**Interfaces:** evaluate_motion(command: Velocity, measured: Velocity, observations: list[FilteredObservation], profile: DeviceProfile, now: float) -> SafetyDecision(allowed,velocity,reason)；同时检查预测扫掠、制动与CoverageResult；MotionEstimator.update(wheel_odom, scan_motion, imu, visual_motion, now) -> MotionEstimate(pose_delta,velocity,covariance,quality)；SlipMonitor.update(command, wheel_odom, actual: MotionEstimate) -> MotionAnomaly。scan_motion必须来自独立扫描匹配，不能重用轮速融合odom冒充独立信息。

- [ ] **Step 1 — 写失败测试：** test_warning_overlap_allows_safe_reverse：前方0.60m障碍、矩形车体与经测量配置下，后方已观测为空闲的安全倒车可通过；test_corner_rotation_blocked；test_invalid_depth_is_unknown；test_stale_sensor_stops；test_measured_velocity_extends_braking；test_wheels_spin_without_body_motion_stops；test_scan_degeneracy_disables_degraded_reverse；test_loop_jump_not_motion；test_startup_latency_not_slip；test_lateral_slip_and_unexpected_yaw。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 分离硬边界和速度相关预警区。扫掠离散精度与栅格/障碍误差关联并保守加界，测试步间角点碰撞。硬边界起始碰撞不尝试自动解锁，安全进程20Hz仿真目标独立运行。增加时间对齐、质量门控的独立局部运动估计与打滑/受阻检测；异常停车不加力。历史地图倒车资格在任务6接入，当前任务先单测质量和停止输出，距离预算使用实际位移而非轮速积分。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_geometry.py tests/unit/test_guard.py tests/unit/test_motion_estimator.py tests/unit/test_slip.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add footprint sweep and independent safety gate"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 5: 控制仲裁、心跳与断线停车

**Files:**
- Create: `src/senior_control/safety/arbiter.py`
- Create: `ros2/src/senior_robot/senior_robot/command_node.py`
- Create: `tests/unit/test_arbiter.py`
- Create: `tests/ros/test_velocity_chain.py`

**Interfaces:** Arbiter.accept(command: Envelope, now: float) -> CommandResult；tick(now: float) -> Velocity；disconnect(connection_epoch: str) -> None。优先级为锁定急停、手动、脱困、导航；自动模式互斥且不自动续跑。

- [ ] **Step 1 — 写失败测试：** test_half_open_connection_expires；test_old_epoch_rejected_after_reconnect；test_estop_latches；test_manual_takeover_stops_nav；test_sim_pause_expires_command：假单调时钟推进、ROS时间冻结仍输出零；test_delayed_same_epoch_command_expired：同一连接的延迟旧指令也不能被重新赋予有效期。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 实现每车控制租约和心跳门控，设备/会话/序号严格校验。重连增加连接代次，旧命令不排队。ROS输出侧仅安全门发布/cmd_vel；Nav2和恢复统一上游重映射。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_arbiter.py -q；Linux: python3 -m pytest tests/ros/test_velocity_chain.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: enforce command ownership and connection safety"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

## 阶段二：导航、地图与定位

### Task 6: Nav2导航与有界脱困

**Files:**
- Create: `src/senior_control/safety/recovery.py`
- Create: `ros2/src/senior_robot/{senior_robot/navigation_adapter.py,launch/navigation.launch.py,config/nav2.yaml,behavior_trees/navigate.xml}`
- Create: `tests/unit/test_recovery.py`
- Create: `tests/ros/test_navigation.py`

**Interfaces:** RecoveryController.step(context: RecoveryContext, now: float) -> RecoveryDecision；NavigationAdapter.navigate(goal: MapGoal) -> execution_id；cancel(execution_id) -> OperationResult。RecoveryContext含候选扫掠评估、进展与覆盖。

- [ ] **Step 1 — 写失败测试：** test_front_blocked_selects_safe_reverse；test_occluded_rear_uses_bounded_map_assisted_reverse_when_eligible；test_all_blocked_finishes_bounded；test_cancel_rejected_does_not_start_new_goal；test_expired_tf_blocks_recovery。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 验证安装版本Smac Lattice控制集与MPPI接口，使用差速原语和footprint，移除强制旋转前置。实现单次及累计时间/距离/次数上限和进展检测；候选复用任务4安全评估与MotionEstimate。后部遮挡时以MapRef匹配的已知空闲地图、近期观测年龄、定位质量、有效独立运动观测及距离/时间预算启用显式降级慢退；无资格则拒绝。调用者通过只读MapEvidenceProvider注入地图证据，地图功能未实现前仅用测试夹具验证，完整集成待任务8完成后验收。实时障碍可否决历史空闲；受阻、打滑或观测退化立即停止，不自动重试顶撞。记录规划失败、保护拦截与盲区阻塞的不同原因。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_recovery.py -q；Linux: python3 -m pytest tests/ros/test_navigation.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: implement collision checked bounded navigation recovery"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 7: 稳定障碍维护与自由空间清除

**Files:**
- Create: `src/senior_control/mapping/{__init__,evidence,occupancy}.py`
- Create: `tests/unit/test_occupancy.py`

**Interfaces:** OccupancyMaintainer.update(observation: MappingObservation) -> MapDelta；snapshot() -> OccupancySnapshot。MappingObservation含map参考、anchor_id、原始有效射线、非动态命中、动态/遮挡掩码与质量；MapDelta保留证据来源。

- [ ] **Step 1 — 写失败测试：** test_stable_box_added_after_confirmation；test_removed_box_cleared_after_free_evidence；test_person_standing_excluded；test_occlusion_does_not_clear；test_single_miss_preserves_wall；test_unknown_dynamic_tracks_excluded。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 采用带上下界的占据证据与独立新增/清除滞回；稳定时间、次数、距离一致性阈值显式配置。移动掩码不产生穿透自由射线；无效量测不清图。保存按关键帧关联的证据，便于回环重投影；不能只修改局部costmap。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_occupancy.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: maintain stable occupancy with observed free space removal"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 8: SLAM后端接入、回环关联与持续建图

**Files:**
- Create: `ros2/src/senior_robot/{senior_robot/slam_adapter.py,launch/mapping.launch.py,config/slam.yaml}`
- Create: `src/senior_control/mapping/{references,store}.py`
- Create: `tests/unit/test_map_references.py`
- Create: `tests/ros/test_mapping.py`
- Create: `docs/operations/slam-backend.md`

**Interfaces:** SlamAdapter.serialize(destination: Path) -> SlamSnapshot；load(snapshot: SlamSnapshot, initial_pose: Pose2D | None) -> OperationResult；anchors() -> AnchorSnapshot；MapReferenceStore.apply_anchor_update(update: AnchorSnapshot) -> ReferenceChange。

- [ ] **Step 1 — 写失败测试：** test_grid_growth_keeps_reference；test_loop_reprojects_box_and_waypoint；test_unavailable_anchor_marks_review_required；test_reload_preserves_removed_box；test_single_map_odom_owner。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 先做后端能力验证，输出真实Humble位姿图/关键帧关联接口及版本；若无所需接口，在项目内提供窄接口后端桥（必要时独立ament_cmake包senior_slam_bridge），不修改厂商目录。位姿图用于定位，可维护长期占据层用于导航/显示；两者必须保存、回环重投影并一致恢复。验证移除旧箱子后定位后端不会持续把已删除几何重新灌回导航地图。无法实现时本任务阻塞地图完成声明，不能以清costmap替代。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_map_references.py -q；Linux: python3 -m pytest tests/ros/test_mapping.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: persist SLAM and reconcile anchored map updates"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 9: 地图包导入导出与清除事务

**Files:**
- Create: `src/senior_control/mapping/{archive,operations}.py`
- Create: `tests/unit/test_map_archive.py`
- Create: `tests/integration/test_map_operations.py`
- Create: `docs/protocol/map-bundle.md`

**Interfaces:** export_bundle(map_ref: MapRef, destination: Path) -> BundleManifest；import_bundle(path: Path, robot_id: str) -> OperationResult；clear_map(expected: MapRef) -> OperationResult。manifest明确format_version、文件hash、后端版本、地图参考、稳定层、anchors、georeference和waypoints。

- [ ] **Step 1 — 写失败测试：** test_path_traversal_and_symlink_rejected；test_corrupt_bundle_preserves_active_map；test_mixed_revision_export_rejected；test_clear_invalidates_bound_tasks；test_cross_robot_home_requires_confirmation；test_unknown_operation_after_disconnect_queryable。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 统一地图写锁，停车并阻止新导航；先快照、校验再切换，失败回退且不恢复行驶。导入不覆盖凭据/硬件配置，不迁移任务；导出地理信息可空但必须明确unreferenced。清图生成新map_id，保留历史数据与失效任务定义。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_map_archive.py tests/integration/test_map_operations.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add transactional SLAM import export and reset"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 10: 导航点与Home持久化

**Files:**
- Create: `src/senior_control/waypoints/{__init__,store,home}.py`
- Create: `tests/unit/test_home.py`
- Create: `tests/integration/test_waypoint_store.py`

**Interfaces:** HomeService.initialize(start: StartupReference, localized: LocalizedPose) -> HomeResult；set_current(pose: LocalizedPose) -> Waypoint；return_home(expected: MapRef) -> execution_id。Waypoint含id、kind、pose、map_ref、anchor_id、source_robot_id和status。

- [ ] **Step 1 — 写失败测试：** test_home_persists_restart；test_moved_before_localization_not_silently_home；test_trusted_start_transform_recovers_start；test_obstacle_does_not_move_home；test_reference_mismatch_rejected。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 在SQLite中每车唯一Home；初始参考不连续则保持未初始化；复用导航动作/取消和地图参考变更。跨车地图Home为待确认，GPS仅附加展示信息。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_home.py tests/integration/test_waypoint_store.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: persist map bound waypoints and home navigation"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 11: GPS配准、重定位与被动航向校准

**Files:**
- Create: `src/senior_control/localization/{__init__,geo,relocalization,heading}.py`
- Create: `ros2/src/senior_robot/senior_robot/localization_node.py`
- Create: `tests/unit/test_localization.py`
- Create: `tests/ros/test_relocalization.py`

**Interfaces:** GeoAlignment.to_map(fix: QualifiedFix) -> PositionEstimate；Relocalizer.begin(map_ref: MapRef, hint: PoseHint | None) -> OperationResult；HeadingObserver.observe(sample: MotionSample) -> HeadingObservation | None。HeadingObserver不接收速度发布器，观测由融合估计器校验后使用。

- [ ] **Step 1 — 写失败测试：** test_unreferenced_map_rejects_geo_conversion；test_single_fix_no_heading；test_forward_baseline_corrects_wrapped_angle；test_reverse_curve_slip_jump_rejected；test_heading_observer_never_commands_motion；test_no_gps_manual_hint_scan_match；test_bad_gps_does_not_kill_healthy_local_navigation。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 记录datum、高度基准、ENU旋转/平移和不确定度；GPS需质量/新鲜度门控，单点不确定全变换。支持GPS辅助自动初值、静态扫描匹配、上次位姿初值与网页手动初值。正常前进足够基线后被动观测，区分配准/航向冲突再有界融合；无可靠匹配不进入ready。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_localization.py -q；Linux: python3 -m pytest tests/ros/test_relocalization.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add qualified georeferencing and passive heading calibration"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 12: 本地任务数据库与调度

**Files:**
- Create: `src/senior_control/tasks/{__init__,models,store,schedule,runner}.py`
- Create: `tests/unit/test_schedule.py`
- Create: `tests/integration/test_task_runner.py`
- Create: `docs/protocol/scheduling.md`

**Interfaces:** Schedule.next_after(utc: datetime) -> datetime；TaskStore.update(task: TaskDefinition, expected_revision: int) -> TaskDefinition；TaskRunner.tick(now_utc: datetime, now_mono: float, context: ExecutionContext) -> list[TaskEvent]。TaskDefinition含robot_id、map_ref、waypoint_ids、schedule、timezone、enabled和revision。

- [ ] **Step 1 — 写失败测试：** test_interval_anchored_to_created_at；test_cron_dom_dow_or；test_dst_gap_skips_and_fold_runs_once；test_offline_skips_no_catchup；test_crash_after_dispatch_never_redispatches_unknown；test_manual_takeover_requires_resume；test_device_reboot_preserves_tasks；test_next_scheduled_occurrence_runs_but_interrupted_instance_does_not_resume。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 五字段cron不含秒，日期/星期均受限时OR语义；间隔基准为创建时间UTC，日/周按配置时区，默认Asia/Shanghai；DST不存在时跳过、重复墙钟只执行一次。调度资格仅当前扫描周期内的有效触发，不补跑已错过时刻；发生时钟跳变记录跳过区间。SQLite事务claim执行实例，导航请求带持久化execution_id；崩溃未知进入needs_review。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_schedule.py tests/integration/test_task_runner.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: persist schedules and gate execution on server connectivity"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

## 阶段三：云端与车端连接

### Task 13: 单管理员认证、配对与多车中转

**Files:**
- Create: `src/senior_control/cloud/{__init__,app,auth,pairing,registry,relay}.py`
- Create: `tests/integration/test_cloud.py`
- Create: `docs/protocol/http-ws.md`

**Interfaces:** create_app(settings: CloudSettings) -> FastAPI；POST /api/session、DELETE /api/session；POST /api/pairing-codes、POST /api/devices/pair、GET /api/robots、DELETE /api/robots/{id}；/ws/agent与/ws/control使用版本化Envelope。

- [ ] **Step 1 — 写失败测试：** test_pair_code_atomic_single_use；test_rate_limit_invalid_login；test_revoked_device_cannot_reconnect；test_browser_cannot_impersonate_agent；test_two_sockets_old_epoch_dropped；test_wrong_robot_command_rejected。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 配置管理员密码hash，不引入注册；浏览器用Secure/HttpOnly/SameSite会话Cookie与写操作CSRF校验，WS校验Origin。设备使用单独凭据并哈希存储；心跳与控制转发有界队列，速度不积压，状态快照可合并。消息大小、鉴权与上传限额显式配置。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/integration/test_cloud.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add single admin pairing and authenticated robot relay"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 14: 车端代理与任务地图同步

**Files:**
- Create: `src/senior_control/agent/{__init__,connection,dispatcher,snapshots,main}.py`
- Create: `tests/integration/test_agent.py`
- Create: `ros2/src/senior_robot/senior_robot/agent_adapter.py`

**Interfaces:** Agent.run() -> None；dispatch(envelope: Envelope) -> OperationResult；publish_snapshot(robot_id: str) -> RobotSnapshot。RobotSnapshot包含连接、定位、map_ref、home、任务revision、执行状态与采集时间；云端缓存加last_synced_at/stale。

- [ ] **Step 1 — 写失败测试：** test_reconnect_fetches_full_snapshot；test_edit_ack_only_after_device_commit；test_offline_edit_fails_not_queued；test_other_robot_heartbeat_not_eligible；test_shutdown_disables_motion_before_ros_exit。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 任务/Home/地图操作在车端执行，以operation_id去重和expected_revision防覆盖；重连拒绝旧命令但允许查询已提交操作结果。长时间地图导出不阻塞心跳与停车，工作队列分离并有超时。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/integration/test_agent.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: connect robot agent and synchronize device owned data"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

## 阶段四：视觉与网页

### Task 15: RGB-D几何和YOLO语义管线

**Files:**
- Create: `src/senior_control/vision/{__init__,synchronizer,depth,detector,tracking}.py`
- Create: `ros2/src/senior_robot/senior_robot/perception_node.py`
- Create: `tests/unit/test_perception.py`
- Create: `docs/training/yolo.md`

**Interfaces:** project_depth(frame: AlignedRgbdFrame, calibration: CameraCalibration) -> GeometryObservation；Detector.infer(rgb: ImageFrame) -> list[Detection]；track(detections: list[Detection], geometry: GeometryObservation) -> DynamicMask。输出同时供实时安全与长期地图候选。

- [ ] **Step 1 — 写失败测试：** test_rgb_depth_time_mismatch_rejected；test_invalid_depth_not_free；test_missing_weights_explicit_disabled；test_person_mask_does_not_erase_occluded_wall；test_slow_detector_does_not_block_geometry。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 参数化相机topic/深度单位/内参与外参，拒绝未标定数据。YOLO可选依赖隔离进程或有界推理队列；几何保护独立频率。训练文档给出数据结构、按场景时间切分、训练/导出命令、阈值与漏检/负载验收，不虚构已训练权重。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit/test_perception.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add synchronized RGBD geometry and optional YOLO semantics"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 16: WebRTC视频与TURN

**Files:**
- Create: `src/senior_control/vision/{stream,turn}.py`
- Create: `tests/integration/test_video.py`
- Create: `deploy/coturn/turnserver.conf.template`
- Create: `docs/operations/video.md`

**Interfaces:** VideoSession.offer(robot_id: str, offer: SessionDescription) -> SessionDescription；issue_turn_credentials(session_id: str, now: int) -> IceCredentials；信令复用已认证设备/控制连接，不混入速度队列。

- [ ] **Step 1 — 写失败测试：** test_unselected_robot_stream_denied；test_turn_credentials_expire；test_congested_video_keeps_heartbeat_and_stop；test_no_camera_returns_unavailable；test_session_close_releases_encoder。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** aiortc接入车端RGB轨道，协商客户端实际可用编解码，支持ICE trickle与TURN relay。限制观看会话和码率，提供真实帧龄与统计；中继故障显示状态不伪造视频。文档给出relay-only跨NAT验收。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/integration/test_video.py -q`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: stream authenticated robot video through WebRTC"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 17: 网页骨架、身份验证与设备选择

**Files:**
- Create: `web/{package.json,index.html,tsconfig.json,vite.config.ts}`
- Create: `web/src/{main.tsx,App.tsx,App.module.css,theme.css}`
- Create: `web/src/api/{protocol,client}.ts`
- Create: `web/src/features/devices/{DeviceList,Login}.tsx`
- Create: `web/src/features/devices/Devices.module.css`
- Create: `web/src/features/devices/devices.test.tsx`

**Interfaces:** RobotClient.connect(robotId: string): Promise<void>；disconnect(): void；request<T>(type: string,payload: unknown): Promise<T>；subscribe(listener: (snapshot: RobotSnapshot) => void): () => void；类型从任务1协议生成并做schema一致性检查。

- [ ] **Step 1 — 写失败测试：** test_switch_robot_drops_previous_state；test_stale_cache_label；test_session_expired_returns_login；test_command_error_never_shown_success；test_demo_label_always_visible。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 按frontend-design工业仪表台方向实现真实路由和空/错/离线状态，使用有辨识度且可自托管字体。登录后设备列表显示真实状态，不引入多用户设置；组件类型与CSS遵守项目规范。
- [ ] **Step 4 — 验证：** `npm --prefix web test -- --run src/features/devices/devices.test.tsx && npm --prefix web run build`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: build authenticated multi robot control shell"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 18: 地图、驾驶与重新定位界面

**Files:**
- Create: `web/src/features/drive/{DriveView,Joystick,EmergencyStop}.tsx`
- Create: `web/src/features/drive/Drive.module.css`
- Create: `web/src/features/map/{MapCanvas,PosePicker,HomeControls}.tsx`
- Create: `web/src/features/map/Map.module.css`
- Create: `web/src/features/drive/drive.test.tsx`
- Create: `web/src/features/map/map.test.tsx`

**Interfaces:** DriveController.start(input: DriveInput): void；stop(reason: StopReason): void；MapCanvas将OccupancySnapshot的origin/resolution/yaw转换到屏幕；PosePicker输出MapGoal或PoseHint，始终附带MapRef。

- [ ] **Step 1 — 写失败测试：** test_pointer_cancel_and_blur_stop；test_switch_robot_stops_old_robot；test_stale_map_blocks_goal；test_rotated_map_coordinates_roundtrip；test_home_unset_and_localization_pending_disable_navigation；test_landscape_controls_accessible。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 实现地图缩放/平移、目标朝向、轨迹、安全区、原始/过滤覆盖诊断、Home和重新定位状态。横屏双拇指、固定急停、键盘可访问，按住发送短TTL命令；失焦/隐藏/组件卸载清理，不自动抢回租约。受限定位辅助遥控需明确进入。
- [ ] **Step 4 — 验证：** `npm --prefix web test -- --run src/features/drive/drive.test.tsx src/features/map/map.test.tsx && npm --prefix web run build`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add responsive robot driving and map localization controls"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 19: 任务、地图管理与视频界面

**Files:**
- Create: `web/src/features/tasks/{TaskList,ScheduleEditor}.tsx`
- Create: `web/src/features/tasks/Tasks.module.css`
- Create: `web/src/features/mapping/{MapManager,ImportDialog}.tsx`
- Create: `web/src/features/mapping/Mapping.module.css`
- Create: `web/src/features/video/{VideoPanel,DetectionOverlay}.tsx`
- Create: `web/src/features/video/Video.module.css`
- Create: `web/src/features/tasks/tasks.test.tsx`
- Create: `web/src/features/mapping/mapping.test.tsx`
- Create: `web/src/features/video/video.test.tsx`

**Interfaces:** ScheduleEditor产出任务12的Schedule模型；地图操作使用operation_id轮询/订阅；VideoPanel使用任务16信令，叠加检测需匹配帧时间戳，过期结果不画在新帧上。

- [ ] **Step 1 — 写失败测试：** test_daily_interval_weekly_cron_roundtrip；test_offline_edits_disabled；test_clear_requires_named_robot_confirmation；test_import_waits_for_localization；test_inflight_operation_recovered_after_reconnect；test_detection_frame_mismatch_hidden。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 显示任务权威来源、时区、下次触发、跳过/暂停原因；保存必须等车端ack。地图上传/下载、进度、回退状态和影响确认齐全；Home跨车导入待确认。视频显示帧龄/延迟/不可用与推理未启用。
- [ ] **Step 4 — 验证：** `npm --prefix web test -- --run src/features/tasks/tasks.test.tsx src/features/mapping/mapping.test.tsx src/features/video/video.test.tsx && npm --prefix web run build`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "feat: add schedule map lifecycle and video management UI"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

## 阶段五：部署与完整验收

### Task 20: 自托管部署与硬件检查

**Files:**
- Create: `deploy/{compose.yaml,Dockerfile.cloud,Caddyfile,env.example,senior-agent.service}`
- Create: `tools/{doctor.py,run_checks.sh}`
- Create: `docs/operations/{deploy,hardware,simulation}.md`
- Create: `tests/integration/test_deployment_config.py`

**Interfaces:** doctor(profile_path: Path) -> DiagnosticReport；报告版本、话题、TF、时钟、发布者数量、底盘watchdog、覆盖与模型状态；只检查不自动启动设备。compose提供云端、已构建网页与coturn，ROS代理在Linux宿主运行。

- [ ] **Step 1 — 写失败测试：** test_missing_secrets_fail_closed；test_deploy_never_exposes_dds；test_example_has_no_real_credentials；test_doctor_detects_duplicate_cmd_vel_and_tf；test_sim_real_profiles_cannot_mix。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 提供TLS/域名/端口、TURN认证、存储卷、备份、凭据撤销和升级回退说明；依赖锁与构建镜像固定已验证版本。明确宿主机/虚拟机地址与图形加速要求；不自动下载权重或开放无鉴权接口。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/integration/test_deployment_config.py -q；可用时docker compose -f deploy/compose.yaml config --quiet`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "chore: package self hosted deployment and hardware diagnostics"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

### Task 21: Linux端到端仿真验收与交付

**Files:**
- Create: `ros2/src/senior_sim/worlds/{wall_escape,narrow_corridor,dynamic_box,ambiguous_localization}.sdf`
- Create: `tests/e2e/{conftest,test_drive,test_recovery,test_maps,test_tasks,test_gps,test_occlusion}.py`
- Create: `web/e2e/control.spec.ts`
- Create: `docs/testing/{matrix,results,real-robot-checklist}.md`
- Create: `README.md`

**Interfaces:** pytest --simulation-profile <path> 使用隔离数据库、唯一端口和同一已记录场景seed；Playwright连接显式提供的base_url，不隐式启动开发服务。每个结果附代码版本、环境版本、配置hash、日志与录屏/截图路径。

- [ ] **Step 1 — 写失败测试：** test_browser_controls_simulated_robot；test_rear_occlusion_requires_bounded_map_assisted_reverse；test_box_added_then_removed_survives_reload；test_person_not_persisted；test_task_disconnect_stops_no_resume；test_home_returns_after_restart；test_gps_passive_heading_never_moves_robot；test_map_import_with_and_without_gps。
- [ ] **Step 2 — 确认失败：** 运行下列验证命令中本任务测试；应因尚未实现的指定行为失败，记录失败断言。
- [ ] **Step 3 — 实现：** 完成全链路故障注入：半开连接、进程重启、时钟暂停/回退、上传损坏、GPS跳点、感知过期和视频拥塞。桌面及手机横/竖屏截图与交互验收；确认没有假结果和绕过安全链。Linux/硬件不可用时逐项标记未执行并交付复现命令，不能标称全部完成。实车验收另列，先低速受控场地。
- [ ] **Step 4 — 验证：** `.venv/bin/python -m pytest tests/unit tests/integration -q；npm --prefix web test -- --run；npm --prefix web run build；Linux: python3 -m pytest tests/ros tests/e2e --simulation-profile config/simulation.yaml -q；npm --prefix web run test:e2e -- --project=chromium`。预期本任务测试全部PASS；包含构建时须退出码0。需要Linux而未运行的检查保留未完成状态。
- [ ] **Step 5 — 提交：** 显式暂存本任务Files中的实际文件，执行 `git commit -m "test: verify full robot simulation control and document evidence"`，记录验证证据；不要使用全目录暂存夹带凭据或生成数据。

## 阶段退出条件与需求覆盖

| 阶段 | 可独立检查的交付 | 对应任务 |
| --- | --- | --- |
| 一 | Linux底盘传感器运行，矩形保护、自车遮挡与断线停车可验证 | 1—5 |
| 二 | 导航/脱困、长期地图增删与续建、地图管理、Home、定位和本地任务通过行为测试 | 6—12 |
| 三 | 单管理员完成配对、选择多车，读取车端任务与地图，真实命令有车端确认 | 13—14 |
| 四 | 桌面与移动横屏控制、视频与YOLO、任务和地图管理完整可用 | 15—19 |
| 五 | 自托管部署可复现，Linux全链路仿真有证据，实车未验证项明确 | 20—21 |

本计划尚未运行任何产品测试。完成标记只在步骤真实执行后填写。实施中每次更改协议或上游签名须同步其所有消费者及协议文档，禁止以接口兼容名义保留无注释的分支。

## 自查记录

- 已覆盖统一设计第1—14节及最新雷达后部自车遮挡要求。
- 地图稳定新增/清除不是简单调SLAM参数：任务7—8包含证据层、关键帧关联和后端能力验证，并设不能降级为仅清costmap的完成门槛。
- 已明确GNSS被动校准无速度输出、无GPS重新定位、Home默认起点与地图参考隔离。
- 已区分网页离线和车端服务器断线，明确任务暂停、不补跑、车端权威存储与云端读取。
- 任务接口按protocol统一模型引用；Review Focus五类问题均在对应任务中有具名测试。
- 尚未确定的实测安全阈值不伪造为已验收值；由配置、Linux仿真和实车制动标定共同确定。

## 计划修订：轮速odom与独立运动观测

按用户补充更新任务3、4、6、21：自车遮挡不再一概禁倒车；满足地图与质量条件才执行受限慢退。增加独立雷达运动估计、IMU/可选视觉交叉检查、打滑和受阻检测。轮速odom不作为唯一位移依据，盲区内运动异常检测不等于接触前避障。Linux验收必须模拟轮子打滑和后方新增障碍，并记录检测与停车延迟。
