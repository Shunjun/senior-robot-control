# senior-robot-control 只读核查与开发建议

核查基准：2026-11-12，本地 fix/simulation-runtime-qualification @ 4dd9da0；工作区干净；读取上级 AGENTS.md。未修改仓库、启动服务或执行运动测试。以下测试数字均为仓库历史记录，不是本次复跑；远端引用未 fetch，GitHub 查询受网络限制，所有比较均针对本地缓存 origin/main d6696a1。

## 结论与实现路线

推荐局部重构车端运动链，保留认证、租约、时效、感知可信、唯一最终发布、急停持久化、地图/导航及其测试；不以移除保护或放大阈值恢复驾驶。用户最新目标是至少 5 km/h（25/18 ≈ 1.389 m/s），原报告的 5 m/s 目标已撤销。当前没有该速度的全链验证，也没有同版一般绕障通过；现有软限速是有限制或停止，不是距离连续限速。软件/仿真能力和物理车辆安全资格必须分开：5 km/h 应有足够观测距离、停距、硬件转速和实测制动曲线作为不同层的验收前提。

## 实际链路和现状（行号为仓库相对路径）

- 网页 `web/src/features/drive/DriveView.tsx:9` 定义直线上限 0.5 m/s、角速度 1 rad/s、辅助比例 0.3，`:323` 将摇杆映射成实际 vx/wz，`:16` 每 100ms 重发，`:126` 发 drive.velocity。无车端百分比/两档协议；网页直接计算 m/s。协议 `src/senior_control/protocol.py:62,139` 传 vx/wz 和时序信息，`agent/dispatcher.py` / `runtime/ipc.py:237` 经 Agent IPC 转命令；`ros2/src/senior_robot/senior_robot/command_node.py:139` 经仲裁发布带时效的请求，不发布最终 /cmd_vel。
- `src/senior_control/safety/arbiter.py:401,424,493,539,600` 仲裁 lease、连接代次、手动/导航及到期；导航必须带运行身份与采样时效，裸速度被拒绝（`:512`）。`safety_node.py:1246-1444` 消费异步安全结论并最终发布；ROS README 说明唯一安全节点发布 `/cmd_vel`。`motion_fusion.py:156-158,1006-1103` 接收 wheel/odom、IMU、scan 和命令对齐计算运动/滑移。反馈 `command_node.py:327-368` 是安全输出关联回执，不等于轮速闭环控制证明。
- 仿真 `ros2/src/senior_sim/config/controllers.yaml:8-20` diff_drive position_feedback=true、open_loop=false、100Hz 控制器、0.3s cmd_vel 超时；这是反馈里程计，不足以单凭配置证明车轮速度 PID。URDF `ros2/src/senior_sim/urdf/senior.urdf.xacro:20-21` 半径 0.076m、关节速度限 20rad/s，简单换算约 1.52m/s（仿真插件是否强制限速需测），理论轮周速度高于 5 km/h，但不证明驱动、负载下车体或仿真能达到该速度。仓库未发现 STM32 固件或驱动实现，无法核验串口、PID、实际制动与反馈闭环；需接入相应仓库/接口后验证。
- `ros2/src/senior_robot/config/simulation_motion.yaml:28,74-87` 扫描匹配速度可信上界 0.5m/s，安全门 0.3m/s、0.5rad/s，反应 0.2s，减速度 0.3m/s²，激光/测量时效 0.8s，指令 TTL 0.3s，心跳 0.2s，断连 2s。`safety_node.py:1194-1209` 不仅校验上限，而且特定几何资格写死 speed == 0.3 与制动阈值；直接调 YAML 不能完成 5 km/h 资格。`guard.py:85-117` 停距 v*t+v²/(2a)，以现有示例参数计算 25/18 m/s 停距约 3.49m（不含足迹和余量、非实测保证），所以不能简单沿用旧场景和资格。Nav2 配置及控制器实际能力需专项标定，不将扫描匹配可信阈值误认成电机限速。
- 斜坡：当前审阅网页即时改变 vx，安全门输出 `guard.py:417-419` 在 warning_overlap 时仍原样允许，硬扫掠冲突在 `:357` 全零；未找到车端按反馈的加减速器。Nav2 自身控制参数可能影响导航，不能代替车端斜坡；断网时导航继续或停车仍待用户决定；仿真 controller 配置无加减速参数。应在授权目标和最终输出之间建立可测车端速率规划，并确保安全紧急停车路径有独立、更保守策略。
- 显式急停：`arbiter.py:66-162,341-360,690-715` 持久锁存、清运动；网络 safety.reset 因缺少 estop fence 返回 unavailable，内部 `_apply_safety` 可清锁不等于网页可用。`web/src/features/drive/EmergencyStop.tsx` 是发送与确认 UI；需要新协议按当前 latch 版本复位、校验权限和静止条件，并清空旧意图/租约，复位不自动运动。普通障碍/过期由安全判决或仲裁归零，不能笼统称为持久用户急停；`slip.py:7-8,93-100,353-360` 另有运动异常锁存，`motion_fusion.py:129,470-472` 当前不会简单解除真实异常。网络超时保证输出过期归零不等于已实现物理受控减速，需故障注入测量。
- 感知失效：`safety_node.py:1152-1229,1253-1337` 检查新鲜度、测量、深度和配置等，失格不能发布先前授权的运动；`guard.py:242-265` 缺制动等参数停车。方向性由命令/测量的正反向扫掠组成（`guard.py:91-117,330-357`），但同一几何包括实际惯性与旋转；不能从此断言前方障碍时任意后退必可用，仍需后向覆盖及真实移动方向测试。
- 导航：`navigation_adapter.py:413-439` 向 Nav2 NavigateToPose 发目标；`simulation_reference.py:139,422` 将正在执行路线的 hard_sweep_collision 与资格失效分离，允许重新规划，最终门仍停车。实际障碍绕行是否完成依赖 Nav2 地图、成本图、可行空间与运行行为树；没有一般绕障验收证据。


### 激光运动反馈、地面能力与几何建议（用户最新约束）

- 主反馈是激光对静态场景的独立车体运动估计，轮速和 IMU 只作辅助一致性、执行器诊断及打滑检测，绝不将轮速直接当车速。`src/senior_control/safety/motion_estimator.py:46-61,155-215` 可复用 `MotionEstimate` 的 body_twist、质量、协方差、时间、退化和对齐，以及 SE(2) 车体坐标转换；其实现当前要求 scan、轮里程计、IMU 同时存在并对齐（`:162-193`），辅助传感器缺失时不能宣称已有可用降级运行，需明确资格与降级策略。估计是区间平均值而非瞬时速度，`safety/async_guard.py:210-223` 已明确此局限，必须把估计年龄、区间与加速度不确定度纳入瞬时上界；不能把平均速度或低质量数据当真实瞬时速度。
- `ros2/src/senior_scan_matcher/src/scan_motion_node.cpp:250-295` 可复用独立局部扫描匹配、源时戳、车体坐标外参变换、质量和协方差输出；`:269-271` 限制区间平均运动，当前配置 `simulation_motion.yaml:28` 仅 0.5 m/s，须在 5 km/h 条件下重新验证扫描频率、特征可观测性、运动畸变及界限，不只上调数字。`motion_fusion.py:156-158,1006-1103` 可复用时间对齐与独立运动/轮速/IMU 的异常检测，`tests/unit/test_motion_estimator.py:88-137` 和 `ros2/src/senior_scan_matcher/test/test_scan_matcher.cpp` 是现有测试入口；本次只读，未复跑。
- 可考虑在线估计地面加减速能力，但普通缓刹仅证明施加该次控制时的已观测减速度，不证明最大可用制动力，不能据此上调可保证制动能力。以经过验证、适用当前车辆与地面条件的保守初始下界和制动延迟建立动态停距；弱制动或滑移证据立即收紧可用下界并限速/停车，变好必须有充分、可信、可重复且适用的证据方能缓慢放宽。估计陈旧、传感退化、地面突变或场景不可观测时退回更保守的下界并降速/停车；不可把已驶过路面的强制动估计用于保障未知前方地面。停车距离使用上界车速、反应时间、保守减速度、坡度/载荷及误差边界计算，未知条件不能仅凭历史良好结果放行。
- 约 5% 的车体碰撞几何膨胀是待明确的设计输入，并非安全余量的全部：先定义沿各轴缩放还是等距外扩、坐标原点、附件与轮廓、弧线和实际动态偏差；与定位/时间/制动不确定度分项计算、避免重复叠加，验证扫掠与方向性覆盖，不以固定 5% 取代停距或感知余量。
- 增补验收：激光与轮速不一致/打滑时证明控制以合格独立车体运动为主；退化、扫描失配、时间错位或速度上界无法可靠建立时停止或按明确降级处理；湿滑或突变路面弱制动时及时收紧，普通温和制动不提升安全能力，陈旧/未知前方表面不能继承旧高摩擦估计；分别验证 5% 几何定义、障碍前后和转弯扫掠；5 km/h 使用车体独立真值核对。断网后导航继续还是停车保持待用户回答，不能在本报告预设产品选择。

## 用户指出问题：已证事实与推断

1. `ros2/src/senior_robot/launch/simulation_motion.launch.py:95` 默认 qualified_reference=false，`:171` 向仿真资格参数传值；`src/senior_control/simulation_cli.py:426` 显式传 true，`docs/operations/simulation.md:77` 也要求显式 true。故直接默认 launch 与 CLI 入口行为不同；是否应改默认取决于仅仿真且资格来源确实存在，不能对实车隐式放行。
2. `safety/async_guard.py:40-66` ticket 要求 command/scan/motion generation 精确相等；`safety_node.py:1246-1280,1313-1347` 两次复核并丢弃变化的结果，scan/motion 在 `:645-787` 等处更新。推断：输入更新频率高于计算周期时可能持续丢弃造成饥饿；代码结构支持该风险，但本次未运行复现，不能断言 11 月 12 日故障由此导致。设计以有界年龄、保守包络和危险变更即时失效代替所有更新均绝对相等；证明旧结果不能穿透更新后的障碍或刹车变化。
3. `docs/testing/results.md:119-120` 记录 ea0ebb2-1d4e 手动准备期扫描区间 5.0→5.1 秒等待 258ms、未开始驾驶；导航首点 turn/wheel/command stall 锁存，4.30 秒失败。`:96-98` 另一次独立真值显示轮转和车身转角明显不一致，历史锁存有物理依据；`:120` 后续物理探针 stimulus 无效。不得将本次所有 stall 归因为误报，也不得直接取消锁存。
4. `docs/testing/results.md:95` 同期 uncertainty=0.86680m，安全扫掠确与场景 feature 重叠、原终点净空不足；这是所用模型下的合理停车，不证明余量已正确标定或障碍真实撞击。`guard.py` 中停距、足迹、时间/位姿不确定度累加均需逐项留痕，避免重复计算；逐场景审查误差来源与方向性，保留保守包络。

## 保留、替换与明确文件范围

保留：`src/senior_control/safety/arbiter.py` 中租约/epoch/单调截止及持久急停结构，`safety/freshness.py`, `coverage.py`, `geometry.py`, `self_filter.py`, `motion_runtime.py`, `slip.py` 的可信/几何与异常能力；`ros2/src/senior_robot/senior_robot/command_node.py`, `navigation_adapter.py` 的来源隔离；传感器来源和最终唯一发布；`tests/` 相关回归。

替换/拆分：`web/src/features/drive/DriveView.tsx`, `web/src/api/{client,protocol}.ts`, `src/senior_control/{protocol,models}.py` 的速度表示和用户档位；`agent/{dispatcher,composition}.py`, `runtime/ipc.py`, `safety/arbiter.py`, `ros2/src/senior_robot/senior_robot/{command_node,safety_node,motion_fusion}.py` 的链路与生命周期；`safety/{guard,async_guard,drive_feedback,recovery,slip}.py` 的方向约束、限速/资格与故障状态；`ros2/src/senior_robot/config/{simulation_motion,nav2}.yaml`, `ros2/src/senior_robot/launch/simulation_motion.launch.py`, `ros2/src/senior_sim/config/controllers.yaml`, `ros2/src/senior_sim/urdf/senior.urdf.xacro` 的仿真动力学/限值；`simulation_reference.py` 与规划联动。新增文件仅在职责明确后定名：运动目标解析、受测速度规划器、方向性安全包络、状态转换；设备适配层/STM32 在其实际仓库设计，不假设已存在。

建议接口：DriveIntent(source, direction, normalized_throttle, gear, epoch, sequence, deadline) → arbitration；SafetyEnvelope(direction-specific max_v/max_w, stop_reason, observation_stamp, valid_until, version)；VehicleMotionController(intent, envelope, measured_body_motion, wheel_feedback, imu, dt) → bounded wheel target；measured_body_motion 含 scan_match 源、车体坐标系、区间起止与源/接收时间戳、质量、协方差、退化状态及有效期，失效明确输出受控停车模式；HardwarePort.send(target, deadline)/feedback(stamp, wheel speeds, faults)。显式 estop 独立状态和持久序号；普通障碍、断链、感知失效、故障各自状态与恢复条件（断网后导航继续还是停车待用户决定），不共用用户急停。速度限制依测得距离与制动能力单调变化并考虑测得速度、延迟和转弯扫掠；失去有效观测不扩大允许空间。异步工作用不可变快照加 deadline、事故版本即时撤销，边界处再验证；不要以松开版本校验来制造活性。

删除目标是被新接口覆盖的重复网页 m/s 换算、与单一车端速度规划重复的硬编码/分支、无法证明正确的无限等待或过时兼容路径；先定位调用和迁移测试，不建议预先整文件删除。兼容代码按上级要求标记 `//Compatible`；总负责人须亲审 diff。

## 分步验收

1. 固定构建和隔离场景，采集单调时间、源时间、队列等待、实际车体真值、轮速、最终输出、每一项几何余量；重现历史扫描 258ms 和 stall，设置有效刺激及负对照，不修改阈值掩盖失败。
2. 契约单测：双档/摇杆、越界与非有限值、sequence/epoch/TTL、手动导航优先级、断网行为按用户最终决策测试、重连不重放旧手动意图、显式急停持久化及 fence 复位/重放拒绝；重启后持续锁存。
3. 可控时钟和执行器仿真：从静止/高速、正反转、转向、骤变验证加减速度、以激光车体运动为主的反馈闭环及制动超时；断网导航分支待用户决定后验收，轮速丢失/伪造/滞后应按辅助反馈失效策略处理，不可用轮速代替车速；测量物理停止距离，不只断言发布零值。
4. 感知矩阵：不同距离障碍输出单调递减、硬极限停车、前阻后退（后向可观测）、后阻前进、弧线/转向/移动惯性、遮挡和 stale scan/depth；故障停与用户急停分离并符合可恢复策略。持续高频输入 + 慢 worker 保证有限时间产出，同时插入新障碍证明旧结果不放行。
5. Gazebo 改造动力学和可观测距离，在足够长、隔离场地逐速阶测试直至实测车体速度 ≥5 km/h，测刹车、独立车体真值、轮速、指令超时和暂停恢复；5 km/h 下合理停距必须受场地与制动实测支撑。复跑同版手动、三点、动态障碍绕行与不可绕行停车；验证地图/规划重算和最终门同时有效，再做真实网页→云→Agent→ROS→Gazebo 联合链路。
6. STM32 硬件接口、低速台架与轮空、再封闭场地分阶段测超时、急停、轮速反馈和制动，不将仿真资格迁移为实车资格。

成本比较：补丁式短期改动小但目前跨前端、资格硬编码、异步、动力学和状态语义耦合，难以证明目标；局部车端运动链重构为中等成本，保留现有证据与可审查边界，推荐；整体重写成本最高，丢失安全/认证/来源保护和大量回归证据，迁移及复验风险高，缺少必要性。具体工期需先获取硬件控制协议、可靠动力学数据和复现条件。

## Git 审计

本次 `git worktree list --porcelain` 共 17 个，均无已跟踪改动；未跟踪仅 `simulation-long-route/.simulation-write-check` 与 `operator-web/web/node_modules`，先保留待归属确认（不能因名称推断可删除）。主工作区和 Draft PR #14 明确保留；同 HEAD 的 `feat/web-drive-percent`, `feat/simulation-long-route`, `feat/vehicle-drive-core` 及 detached high-speed-navigation 仅作为未来候选，必须确认活动、未跟踪文件及 PR 后才可清理。其他包含独有提交的工作区、所有不确定 PR 或活动一律保留。

下表数值为 `git rev-list --left-right --count A...branch` 的 A 独有/branch 独有，列出全部本地分支；这不是补丁等价结论。`git log --cherry-pick --right-only HEAD...branch` 显示 operator-data 真正未找到补丁等价项 2afcc07，operator-timing 258122f/c5150ca，operator-web a8269e4，simulation-map 360c269；其他分支若拓扑有独有提交但该命令为空，表示其非 merge patch 与 HEAD 存在 patch-id 等价，仅是 Git 补丁级证据，不能保证相同运行语义。main 的独有 merge 8a17b50/cd47e47 不在 cherry-pick 等价比较内。

| 本地分支 | HEAD 独有/分支独有 | origin/main 独有/分支独有 |
|---|---:|---:|
| feat/agent-reconnect-and-workspace-plan | 1 / 0 | 4 / 30 |
| feat/control-foundation | 133 / 0 | 106 / 0 |
| feat/operator-data | 116 / 3 | 89 / 3 |
| feat/operator-live-actions | 86 / 3 | 59 / 3 |
| feat/operator-nav-runtime | 71 / 2 | 44 / 2 |
| feat/operator-timing | 116 / 6 | 89 / 6 |
| feat/operator-web | 116 / 6 | 89 / 6 |
| feat/simulation-direct | 77 / 2 | 50 / 2 |
| feat/simulation-long-route | 0 / 0 | 4 / 31 |
| feat/simulation-map | 86 / 1 | 59 / 1 |
| feat/simulation-video | 62 / 1 | 35 / 1 |
| feat/task-6-recovery | 238 / 0 | 211 / 0 |
| feat/task-7-occupancy | 238 / 0 | 211 / 0 |
| feat/vehicle-drive-core | 0 / 0 | 4 / 31 |
| feat/web-drive-percent | 0 / 0 | 4 / 31 |
| fix/configurable-deployment-ports | 120 / 0 | 93 / 0 |
| fix/jenkins-agent-any | 156 / 1 | 129 / 1 |
| fix/jenkins-deploy-config | 126 / 0 | 99 / 0 |
| fix/jenkins-github-webhook | 154 / 0 | 127 / 0 |
| fix/live-navigation-readiness | 62 / 1 | 35 / 1 |
| fix/operator-viewport | 90 / 1 | 63 / 1 |
| fix/simulation-runtime-qualification | 0 / 0 | 4 / 31 |
| fix/unify-deployment-env | 124 / 0 | 97 / 0 |
| main | 36 / 2 | 7 / 0 |
| test/operator-drive-faults | 62 / 1 | 35 / 1 |

### 工作区逐项

| 路径 | HEAD | 分支 | 状态 |
|---|---|---|---|
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control | 4dd9da0 | fix/simulation-runtime-qualification | 干净 |
| /Users/zhangshunjun/.codex/worktrees/drive-web-percent/senior-robot-control | 4dd9da0 | feat/web-drive-percent | 干净 |
| /Users/zhangshunjun/.codex/worktrees/high-speed-navigation/senior-robot-control | 4dd9da0 | detached | 干净 |
| /Users/zhangshunjun/.codex/worktrees/simulation-long-route/senior-robot-control | 4dd9da0 | feat/simulation-long-route | ?? .simulation-write-check |
| /Users/zhangshunjun/.codex/worktrees/vehicle-drive-core/senior-robot-control | 4dd9da0 | feat/vehicle-drive-core | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/jenkins-agent-any | c9bddc6 | fix/jenkins-agent-any | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/jenkins-github-webhook | 7bd5bdc | fix/jenkins-github-webhook | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/operator-data | 6a5a8cb | feat/operator-data | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/operator-faults | 31022ad | test/operator-drive-faults | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/operator-nav-runtime | 29629a8 | feat/operator-nav-runtime | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/operator-timing | 53b654d | feat/operator-timing | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/operator-web | 6cae9fb | fix/live-navigation-readiness | ?? web/node_modules |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/simulation-direct | f1b5e66 | feat/simulation-direct | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/simulation-map | 360c269 | feat/simulation-map | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/simulation-video | f332cab | feat/simulation-video | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/task-6-recovery | c0fd481 | feat/task-6-recovery | 干净 |
| /Users/zhangshunjun/Wheel 机器人/senior-robot-control/.worktrees/task-7-occupancy | bc79f68 | feat/task-7-occupancy | 干净 |

### 远端跟踪分支（本地缓存）

| 引用 | HEAD 独有/引用独有 | origin/main 独有/引用独有 |
|---|---:|---:|
| origin | 31 / 4 | 0 / 0 |
| origin/feat/agent-reconnect-and-workspace-plan | 31 / 0 | 4 / 0 |
| origin/feat/control-foundation | 133 / 0 | 106 / 0 |
| origin/fix/configurable-deployment-ports | 120 / 0 | 93 / 0 |
| origin/fix/jenkins-agent-any | 156 / 0 | 129 / 0 |
| origin/fix/jenkins-deploy-config | 126 / 0 | 99 / 0 |
| origin/fix/jenkins-github-webhook | 154 / 0 | 127 / 0 |
| origin/fix/simulation-runtime-qualification | 0 / 0 | 4 / 31 |
| origin/fix/unify-deployment-env | 124 / 0 | 97 / 0 |
| origin/main | 31 / 4 | 0 / 0 |

Orca `worktree ps` 经授权查询：本仓库列出的主工作区状态 working，包含本 worker 与另一个“真车与仿真配置核查”工作中的 structured agent；链接 PR #14 为 draft，四项检查成功。其他 Git worktree 未出现在该次本地 Orca 返回中，不能推断无活动，其他主机也未查询。GitHub API 首次连接失败；后续权限请求被新请求覆盖而取消，因此其他 PR 关联未知；应在后续可用网络重新查询后再决策。没有删除、修改、关闭任何 worktree 或分支。
