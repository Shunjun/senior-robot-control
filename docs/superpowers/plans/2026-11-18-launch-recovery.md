# Launch 节点异常恢复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 本文已由负责人亲审并授权按收口范围分阶段实施；部署与现场验收仍由负责人负责，不另派子 Agent。

**Goal:** 本轮纳入的长驻 launch 节点异常退出后自动恢复；正常完成、主动关闭和已取消的导航执行不被重新启动。用户已取消 `senior_sim/simulation.launch.py` 全部专用恢复，仿真环境人工处理。

**Architecture:** 保持现有 launch、厂家 include 和模块 owner，优先使用 ROS launch 的进程、退出事件、停机与延迟机制；仅以一个小型进程前缀适配模块补足厂家 include 内部的异常退出筛选。Nav2 生命周期组按现有 manager owner 恢复；Gazebo server/world/GUI 及仿真专用 bridge/状态发布恢复已由用户取消，控制侧仅补启动代次交接，不扩展到内部子进程监督。

**Tech Stack:** Python 3.10+、ROS 2 Humble launch/launch_ros、现有 Nav2 lifecycle manager、Fortress/ros_gz_sim；不新增依赖、公开 CLI 或平台。

**Spec:** 根目录 `../CURRENT_TASK.md`、`docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md` 及本次用户“所有 launch 管理节点自动恢复”的明确要求，冲突处以根任务和最新指令为准。

## 状态与约束

- 本文收口于 `fix/ros-typesupport-export-order@e97720f781d8aa18d5697768b3d7166dfab860b4`；该提交包含原盘点基线 `4bde9a6` 及负责人后续文档/回归，未覆盖后续成果。
- 原计划与直接测试候选已获负责人批准；后续批准 state_machine 恢复入口和既有 RGBD 测试迁移，最新又取消整个 senior_sim/simulation 专用恢复。用户随后已获预报并批准 DedicatedController 必要父死清理，当前准确范围为产品 13、测试 4、文档 1，共 18 文件；配置/构建/工具均为 0。
- 不修改厂家源码、协议、厂家 launch，不复制一套驱动 launcher，不在车端改源；部署由负责人负责。
- 不重启整车服务、Agent 或 LearningService，不创建新学习 run，不重放学习动作、旧手动指令或旧 goal；不改控制算法、TTL、控制频率或运动资格，不增加第二个 Twist owner。
- 后续实现使用负责人分配的隔离工作区，从固定功能主分支分叉并正常 merge 合回；本次不创建 worktree，不运行测试/服务，不访问车。

## 已核实的机制与选择

Humble `ExecuteLocal` 的重启判断不检查返回码是否为 0，因此不能把所有 Node 直接设为 `respawn=True`；`IncludeLaunchDescription` 也没有自动向内部 Node 注入 respawn 的能力。项目的 `navigation.launch.py` 自建 Node，并未调用标准 nav2_bringup 的 `use_respawn` 入口，直接传该参数不会生效。

选用现有 `launch-prefix` 作为原厂嵌套节点的公开接入点：项目入口在 scoped GroupAction 中设置前缀和精确的长驻可执行文件过滤器，厂家 Python/XML/OpaqueFunction 仍按原样解析。直接拥有的长驻 Node 可显式指定同一前缀；同一进程只能有一处重启循环，不能再叠加 respawn。正常返回 0 的单次工具没有前缀。纯原生 respawn 仅在安装版本确实提供符合要求的异常退出语义时替代适配器，不为统一写法依赖私有属性或 monkeypatch。

上游核对来源：[Humble ExecuteLocal](https://raw.githubusercontent.com/ros2/launch/humble/launch/launch/actions/execute_local.py)、[Humble Executable 的 launch-prefix](https://raw.githubusercontent.com/ros2/launch/humble/launch/launch/descriptions/executable.py)；显式 `prefix` 会覆盖继承值，所以上线前须由负责人一次性只读确认实际安装的递归 include 和可执行文件，不以本地厂家资料冒充实机安装版本。

### 单一小模块及环境闭合

新增 `src/senior_control/runtime/launch_recovery.py`：

- `restart_prefix() -> str`：使用当前 launch 解释器的绝对 `sys.executable`，经 `shlex.join` 生成 `python -m senior_control.runtime.launch_recovery -- <原节点命令>` 前缀；不调用 PATH 中不确定的 `python` 或 `ros2`。
- `run_node(command: list[str]) -> int`：只接管传入的一个长驻进程及其退出清理；子进程异常非零时记录实际 PID/返回码/尝试次数，回收旧子进程组后固定等待 2 秒再启动；返回 0 则结束，不再复活。stderr/stdout 继承 ROS 日志，不能吞掉原崩溃信息。
- 私有组运行入口仅供 navigation 与 simulation_motion 定位分支自身调用：使用 `LaunchService`、`IncludeLaunchDescription`、`OnProcessExit`、`Shutdown` 管理既有组，并把指定 owner 的异常退出转换成组异常结果；无通用服务注册、持久调度器或配置平台。
- 包含停止标志和单一重试等待：收到父 launch 的 SIGINT/SIGTERM 先取消等待，再转发/回收，停止标志置位后不启动替代进程；组恢复同样先停止、等待旧组完全退出，再延迟启动新组。取消发生在延迟结束/创建进程之间时，再检查停止标志，并回收已创建但尚未交付的进程。
- 私有子进程组的退出上限须短于现有 helper 的 10 秒清理期限；不能只等 leader 退出就认为旧组已消失。此处清理是 launch 节点替换所必需的资源释放，不是对子进程另加故障监督。
- 只有该节点的第二次及以后启动带内部环境标记 `SENIOR_LAUNCH_RECOVERED=1`，初次启动不带；不更换 `SENIOR_STARTUP_LEARNING_CLAIM`。

真实服务现由 `.ros-venv/bin/python` 执行 helper；`_ros_launch_command` 已通过该 `sys.executable` 调用 ros2cli，并 source 厂家和项目 overlay。该新模块由现有 senior-control Python 包安装流程自动包含，无新 entry point、setup 或 CMake 修改；前缀在同一 sourced 环境中用同一解释器导入。负责人上线前仅需一次核对该解释器的模块导入与实际生成的 argv，不改系统 Python。

独立 ros2 launch 入口若解释器不能导入现有 senior_control，则当前安全 launch 本来就无法完整导入；不为此另造兼容环境。实现时明确报告该入口不可用，不能悄悄换解释器或假称覆盖完成。

## 节点覆盖与恢复 owner

以下为条件化清单，每一条件只启动原有分支，不额外启动候选硬件；已存在的源 GID/epoch admission 继续工作，不增加资格条件。

| 项目父入口 | 节点或原厂递归分支 | 恢复 owner / 方式 |
| --- | --- | --- |
| helper 独立 camera launch | astra/dabai/gemini XML 内 `astra_camera_node`，或所选 `usb_cam`；`republish_color_compressed`、`republish_depth_compressed` | scoped 精确长驻前缀，分别恢复实际崩溃节点；camera 外层本身异常退出时 helper 只重开 camera，先清理旧 camera 组 |
| safety_motion → turn_on_wheeltec_robot | `wheeltec_robot_node`；所选 H30/N300pro IMU include（stm32 无额外进程）；`ekf_filter_node` 或 carto/nav 参数对应的唯一 EKF；`robot_state_publisher`、`joint_state_publisher`；配置存在的 `base_to_laser/camera/link/gyro/radar` 静态发布者；启用时 ultrasonic include | 原厂启动图不变；每个长驻可执行进程使用前缀，不因一台设备崩溃重开整个底盘 include |
| safety_motion → wheeltec_lidar | 所选 LS/LD/RPLidar 分支；双雷达分支的两台驱动和融合；lscx 分支的点云转扫描 | 同上；本地默认 `/x10/lslidar_driver_node` 的实际 C++ 是普通 rclcpp::Node，不能因 launch 使用 LifecycleNode 就猜测需生命周期转换；现场若不同则先收口真实 owner |
| safety_motion | `scan_receipt_node`、`scan_adapter`、`safety_scan_ingress`、`scan_motion_node` | 分别恢复新进程，保留健康的上下游；真实新帧沿既有换代接纳路径恢复 |
| safety_motion / simulation_motion | `safety_node` | 仅恢复该 launch 节点，旧私有输出组清理完再创建替代者；先完成下述 critical 交接，不能先开放重启开关 |
| navigation，external_controller=true | `planner_server`、`lifecycle_manager_navigation` | 仅 planner 生命周期组，不接管 DedicatedController |
| navigation，external_controller=false | `controller_server`、`planner_server`、`bt_navigator`、`lifecycle_manager_navigation`、`navigation_adapter` | 本 navigation 组；server 进程恢复与现有 manager bond 重连，manager 自身异常按整组边界重建；不重发目标 |
| mapping 默认分支 | `async_slam_toolbox_node`、`slam_adapter` | 长驻节点分别恢复；新内存图不能冒充原建图会话延续 |
| mapping capture/verified 分支 | `slam_graph_node`（名 slam_toolbox），verified 时另有 `slam_static_lidar_tf_receipt` | 长驻进程恢复；capture 目录重开是否被现有节点接受需实证，不自动创建新 capture 目录/新任务绕过 |
| rgbd_diagnostic | `static_tf_receipt_node`、`rgbd_candidate_receipt_node`、`rgbd_candidate_producer` | 长驻前缀 |
| simulation_rgbd | 原 simulation_motion + `raw_receipt_node`、`depth_ingress`、`semantic_ingress` | 各自长驻前缀；嵌套入口不重复叠前缀 |
| simulation_motion | `scan_adapter`、`scan_motion_node`、`scan_receipt_node`、`safety_scan_ingress`；可选 `simulation_camera`；`agent_adapter`（名 command_node） | 长驻前缀；command 重启不保留旧 lease/RouteService，不重建旧 goal；safety 和 command 不作为一个组重启 |
| simulation_motion 可选定位 | `map_server`、`amcl`、`lifecycle_manager_localization` | 与 navigation 同样的本定位生命周期组，恢复到 ACTIVE 才算完成 |
| senior_sim/simulation | ros_gz_sim server/可选 GUI | 用户取消热恢复；原启动退出逻辑保持 |
| senior_sim/simulation | `robot_state_publisher`、`senior_sim_bridge` | 用户确认也取消该仿真专用恢复，保留原逻辑；实车状态发布恢复不受影响 |
| senior_sim/simulation 单次工具 | `create`、`joint_state_broadcaster` spawner、`drive` spawner；xacro/world_reference 生成 | 保留原单次启动与退出；不新增循环或环境重建 |

### Nav2 的组边界与 Gazebo 范围取消

1. Nav2 server 异常：前缀恢复真实 server，使用现有 manager 的 `attempt_respawn_reconnection=true`、bond reset/startup 完成 configure/activate；采用上游已有窗口（本地源码默认 10 秒），不能据进程起来就宣称恢复。超出窗口时由同一 navigation/定位组进行一次有序重建，不另造 lifecycle 协调器。
2. Nav2 manager 自身异常：在所属 launch 建立的独立组运行边界内结束该组，等待所有旧 server/adapter 退出，再创建全新的 manager 与成员；仍活跃的 ACTIVE server 不可直接配一个新 manager。组运行调用同一项目 launch 的内部成员分支，成员分支不再创建组运行入口，避免递归；只传原参数、不复制节点定义。正常组关闭不触发恢复。外层保留 Agent、safety、其他 launch。
3. **用户取消整个 senior_sim/simulation 专用自动恢复**：server/world/GUI、robot_state_publisher、senior_sim_bridge 均人工处理，保留原有 Gazebo include、启动退出及 create/spawner 单次初始化逻辑；不实现环境组、不新增对应故障场景。此前仅只读研究上游，尚未写入任何 Gazebo 专属产品或测试改动，没有需要回退的代码或提交。Nav2 重连判定仍使用既有 lifecycle state/service，不增加运动许可接口。

## critical 交接与端点清理：仅保留三个必要产品文件

`command_node.py`：新增集中方法 `_on_output_incarnation_changed(previous, current) -> None`，在已验证的 `/motion/runtime` boot/GID 更换处调用；退休旧 manual authority、撤销当前 navigation source/arbiter execution，提升现有 generation，发布原协议的零速 barrier。现有 RouteService 对 source_active 的检查将结束旧执行，不能在恢复回调里调用 start/发 goal。保存“最后已接受、尚未被明确复位的急停请求”，不能在收到一次 ACK 后丢掉锁存恢复所需的事实；新输出 boot 下用原请求身份重建现有 estop barrier，禁止把它变成 estop_reset。source 清理不能覆盖待确认急停。

已核对 `FinalCommandFence.accept` 要求新发布者/boot/generation 首帧为零速 barrier，现有非零旧 intent 不能直接建立新输出代次；这可复用，但单独不足以恢复锁存。输出换代处理必须在将新 runtime 当作当前 clear 状态前保存并接续上一代的已锁 estop，清掉旧目标再发布 barrier，防止取消窗口中的迟到 intent 复活。command 自身重启时 safety 不随之重启，原输出锁存保持，command 初始无旧 lease/目标；不能由 prepared/普通 stop 清除输出急停。

`motion_core_output.py`：恢复标记只禁止 `_startup_target` 发起学习动作，继续消费 LearningClient 已完成的只读 view；不得进入新的 reserve_action，不删除完成模型、不改变 LearningService 的 run/已消费事实。初次明确程序启动仍遵循原学习规则。这个变更放在恢复接线处，不重构 StartupLearning 或控制算法。

`agent_adapter.py`：仅处理 simulation_motion 中由 launch 管理的 command 节点的 IPC 端点重开。现有 `ipc_replace_existing_socket=false` 加上 IpcServer 的 bind 行为，意味着崩溃遗留的 socket 会阻止替代进程启动；不可通过改为无条件 replace 来修复。该 launch 节点首次成功绑定后记录既有 socket 身份格式，恢复启动复用 `_prepare_socket` 的检查语义：同一所属 launch/run、同一 inode/uid、连接明确拒绝且身份复核未变化，才删除 stale socket，再绑定并更新身份；活 socket、陌生身份或不确定探测均不删除。记录与清理只对 launch 明确传入的端点所有权上下文启用；实车 helper 直接启动路径保持现有 PID/所有权流程，不能两边同时清理。这是 launch 节点恢复必需的端点交接，不含 RouteService/DedicatedController 监督或新业务协议。

以上只是最小接口设计，不是已验证闭环：必须实测新输出 boot 下旧 intent 被拒、零 barrier 重新建立、已锁 estop 仍锁住、旧任务不重发。sim command 进程恢复后，既有任务 outbox 的 unknown 查询不得转为重发 goal；当前 `LifecyclePort` 已声明 unknown 不授权重发，需用既有实现验证这一行为，若不成立先报具体缺口和最少文件再改，不能在本计划暗加任务持久化框架。

## 明确剥离的非 launch 范围

`ControlInputs/motion-reception`、`RuntimeReports/motion_reporter`、`final_command_guard`、`motion_output_guard` 是活 owner 内部创建的进程；`DedicatedController` 直接 subprocess 创建的执行作用域 controller/BT 也不是 launch 节点。本轮不为其新增检测/重启，不修改 `control_inputs.py`、`runtime_reports.py`、`safety_node.py` 或 `motion_output.py`；`dedicated_controller.py` 后续唯一获准例外为原 owner 退出后的必要父死清理，不监督内部故障；`agent_adapter.py` 仅作上一节的 launch 端点交接，launch 节点真正退出时清理其残留资源属于必要退出清理，不能改写成对子进程独立监督的授权。

实车 helper 直接启动的 agent_adapter 也不属于 launch，本轮不加其自动重启、更不套 wrapper 破坏真实 PID 的 SO_PEERCRED。helper 对 camera 的改变仅是传入相同前缀配置，以及 camera 外层明确异常时独立恢复；不重启 motion 外层来同时重建 safety/command 状态。

已有扫描换代路径位于 scan_receipt、scan_adapter、safety_scan_ingress、matcher 与 EgressAdmission；允许新 GID/epoch 进入、拒绝旧身份，不需为恢复更改 admission。真实话题/TF 是否恢复以及有限历史容量不能凭参数设置宣称通过。

## 准确候选文件清单与数量

产品代码 13 文件（新增 1、修改 12、删除 0）：

1. 新增 `src/senior_control/runtime/launch_recovery.py`：前缀、单进程异常恢复、两个既有组的 launch 运行边界。
2. 修改 `src/senior_control/docker_start_helper.py`：camera 启动接线/独立外层异常恢复。
3. 修改 `ros2/src/senior_robot/launch/safety_motion.launch.py`。
4. 修改 `ros2/src/senior_robot/launch/simulation_motion.launch.py`。
5. 修改 `ros2/src/senior_robot/launch/navigation.launch.py`。
6. 修改 `ros2/src/senior_robot/launch/mapping.launch.py`。
7. 修改 `ros2/src/senior_robot/launch/rgbd_diagnostic.launch.py`。
8. 修改 `ros2/src/senior_robot/launch/simulation_rgbd.launch.py`。
9. 修改 `src/senior_control/motion/state_machine.py`：显式恢复已确认的原急停请求与 revision，不隐式清锁。
10. 修改 `ros2/src/senior_robot/senior_robot/command_node.py`：现有输出代次与锁存交接。
11. 修改 `ros2/src/senior_robot/senior_robot/motion_core_output.py`：恢复不发起学习动作。
12. 修改 `ros2/src/senior_robot/senior_robot/agent_adapter.py`：仿真 launch 所有 IPC 端点身份记录与 stale socket 清理。
13. 修改 `ros2/src/senior_robot/senior_robot/dedicated_controller.py`：仅原 owner 强制退出后的既有私有 controller/BT 父死清理，不加检测或重启循环。

测试 4 文件（新增 2、修改 2、删除 0）：`tests/unit/test_launch_recovery.py`（新增）、`tests/ros/test_launch_recovery_live.py`（新增）、`tests/unit/test_docker_start_helper.py`（修改）、`tests/ros/test_rgbd_diagnostic_launch.py`（仅迁移既有进程遍历/PGID 断言，不新增场景）。优先复用既有真实进程设施。

配置/构建 0；工具 0；文档仅本文件新增 1，后续在本文更新结果，不另写报告。用户取消 simulation.launch.py 后为产品 12、总 17；随后负责人已提前报告并批准 DedicatedController 必要退出清理，使最新范围为产品 **12 → 13**、总 **17 → 18**，即 **18 文件：新增 4、修改 14、删除 0**。simulation.launch.py 从未改动，不回退 simulation_motion 的 Nav2/定位与 command IPC 成果；任何新范围仍须先报告。

## 顺序实施及验收

### 第一步：相机与现有设备源

- [ ] 负责人一次性只读核对实机所选 wheeltec YAML、相机 XML/递归 include、已有 respawn/显式 prefix、实际可执行文件、`.ros-venv` 导入与 sourced 环境，不重复采集已定位的相机崩溃证据。
- [ ] 实现单一前缀模块与 helper camera 接线；先只开放相机/设备长驻分支，不开放 critical safety 自动重启。
- [ ] 在已确认的 astra -11 路径验证：只让实际 camera node 异常退出，外层保留，图像发布者/发送帧恢复；日志留真实崩溃码，Agent/学习 run 不变，不产生运动。
- [ ] 覆盖原厂底盘/IMU/EKF/静态发布者/所选雷达的长驻分支，再验证一个真实 scan receipt/adapter 异常恢复后新 GID/epoch 接纳、旧 epoch 拒绝；复用 `tests/ros/test_scan_epoch_live.py` 的设施，不把它当前的局部 matcher 测试冒充全链恢复。

### 第二步：全部其余 launch 节点与必要组

- [ ] 先完成三个 critical/端点文件的交接并验证，再开放 safety_node 与仿真 command_node 恢复；内部子进程监督保持范围外。
- [ ] 接齐表中 mapping、RGBD、simulation_motion 所有条件分支；相同策略只接一次，单次工具无前缀。
- [ ] 接齐 navigation/定位组：真实 server 崩溃后 native manager 恢复 ACTIVE；manager 崩溃/重连窗口耗尽时仅本组有序恢复，确认旧 goal 不重发；复用 `tests/ros/test_navigation.py` 的真实 Nav2 启动设施。
- [x] 用户取消 Gazebo server/world/GUI 热恢复及候选 10：未实现该恢复与新测试，保留原启动退出逻辑。
- [ ] 负责人亲阅实际 diff 与版本对应证据后合回；本任务不执行这些步骤或自动部署。

## 最少验证候选与 Review Focus

### 实施记录（2026-11-18）

- 视频隔离工作区从 `d670323` 正常 fast-forward merge 到冻结基线 `cdbeeea`，无独有提交或冲突，保留视频三项提交及 `7b5a034` 补点。负责人已批准按本计划实施与选定直接测试；本文件前述“等待审阅”是计划创建时的状态。
- 首个阶段仅新增独立 `launch_recovery` 前缀模块和获批的三个真实子进程测试：SIGSEGV 后固定 2 秒恢复/第二轮正常退出不复活、退避取消、旧进程组完整清理及停止回收。模块不导入 ROS、不改学习 claim，不开放任何产品 launch 恢复开关；相机接线等待负责人提供 installed prefix/递归 include 证据。
- 本地 macOS Python 3.12 的独立非运动子进程验证不代表 Humble installed launch 或相机现场通过；Nav2 ACTIVE、critical 交接、Gazebo 组及实机图像恢复均尚未实现/验收。
- 阶段命令使用现有仓库 `.venv/bin/python` 并设置 `PYTHONPATH=src`：`-m pytest -q tests/unit/test_launch_recovery.py` → 3 passed（9.37 秒）；两个新文件 `ruff check`、`ruff format --check`、AST/import 检查通过，产品模块 `mypy --follow-imports=skip` 通过。只运行获批的三个新增场景，无实机/ROS/服务操作；尚无需迁移现有回归。

### 相机接线阶段

- 负责人提供的实际 installed 证据已核对：Humble `Executable` 通过公开 `launch-prefix` 与 `launch-prefix-filter` 对可执行文件 basename 做 `re.match`，显式 prefix 会覆盖继承；实际相机选择 astra，厂家 XML 的 `astra_camera_node`、两个 `republish` 和 USB `usb_cam_node_exe` 均未另设 prefix/respawn。helper 仍调用原 `wheeltec_camera.launch.py`，只给该独立 launch 传入上述三个 basename 的锚定过滤器；单次工具和 motion/safety 不受该配置影响。
- 经负责人认可，暂不开放 camera 外层重开：父 PGID 消失不能证明叶 prefix 的独立 session 均已清理，也不能把叶清理失败误作成功。本阶段仅修复实际相机叶进程异常恢复；Nav2/Gazebo 后续使用公开 LaunchService 正常 Shutdown 管理各自组，不重复套叶节点循环。
- 叶 prefix 复用 `child_lifetime` 的 Linux 父退出 SIGTERM 保障；这不是所有后代已退出的完成凭证，亦尚无 Linux 现场验证。helper 相机清理为叶节点五秒上限留六秒调度余量，总预算仍为十秒；未修改 helper 直接 adapter 的身份验证或启停策略。
- 新增一个 helper 真实非运动进程场景，使用本机替代 launch/ROS readiness，验证叶进程 SIGSEGV 后新 PID、至少两秒退避、正常结束不复活、父 camera launch/Agent/motion/adapter 不重建、学习 claim 不变。它不证明 installed XML 实际启动或 Linux peer credentials；原相机外层异常恢复场景随上述范围暂缓，不能算通过。实际相机图像和发送帧恢复由负责人在获批窗口验收。
- 模块复用父生存保障后，两个相关文件定向测试 `pytest -q tests/unit/test_docker_start_helper.py tests/unit/test_launch_recovery.py` 为 12 passed、1 skipped（14.06 秒）；随后 helper 清理余量调整的文件回归为 9 passed、1 skipped（5.19 秒）。skip 为本机无 Linux SO_PEERCRED，不冒充 Linux/ROS 通过；ruff/AST/import、模块 mypy 通过，无整套回归或实机操作。
- 最后补齐 USB 过滤器后，仅重跑 `pytest -q tests/unit/test_docker_start_helper.py -k camera_leaf` → 1 passed、9 deselected（4.86 秒）；三项改动 Python 文件的 ruff check/format 通过。提交前已亲查全部产品 diff，motion_core_output 与补点范围未改。

### 设备源与非 critical 分支接线阶段

- `safety_motion` 仅在 scoped GroupAction 内给厂家底盘/IMU/EKF/状态/静态 TF/超声波/雷达 include 设置精确 basename 前缀；项目四个扫描节点显式接同一前缀。已核对当前 installed 为 stm32、N10Plus，不以 LifecycleNode 类名推断设备需要生命周期激活；H30/LD/RPLidar 以补充 installed 可执行名接入。lscx 的转换 include 实际安装文本未在两份证据内，未将该条件分支视为实测覆盖。
- mapping 的默认 SLAM/adapter 与 capture/verified 图/receipt、RGBD diagnostic 三节点、simulation_rgbd 三节点、simulation_motion 的相机及四个扫描节点分别接一次显式前缀。嵌套 include 未继承一层额外循环；safety、command、Nav2、Gazebo 环境恢复此时仍关闭。
- 新增 live 文件中的获批 scan 场景复用 `test_scan_epoch_live._room_scan` 与既有 native receipt 可执行定位方式：隔离 ROS 域内真实 receipt SIGKILL 后接纳新 GID/boot/epoch、健康 adapter 继续产生真实 fenced 帧，并向该 adapter 回调旧 capture 确认不改变当前接纳。Linux `/proc` 残留（含 zombie）不算恢复；本机无 ROS，尚未运行该真实链。
- 定向 `pytest -q tests/ros/test_launch_recovery_live.py tests/ros/test_mapping.py tests/unit/test_rgbd_transport_config.py` → 30 passed、2 skipped（0.12 秒），新 live 场景因无 ROS 跳过，另一项 mapping ROS 检查跳过；五个 launch 与新 live 文件 ruff check、format/AST 通过。mapping 保持原 CRLF，差异空白使用 `git -c core.whitespace=cr-at-eol diff --check` 校验，无持久 Git 配置变更。
- 未验证 SLAM capture 目录重开、厂家条件分支话题/TF、RGBD 真实 worker 清理或扫描端到端现场恢复；不将前缀配置存在说成语义恢复完成。既有 `test_rgbd_diagnostic_launch.py` 的父 PGID/直接子进程结构断言与新前缀结构冲突，当前严格保持三个获批测试文件范围，待负责人明确如何迁移该既有断言，未隐去或冒称其通过。

### critical 代次交接接口阶段（尚未开放恢复开关）

- 负责人已向用户预报并批准最小范围调整：产品 12 → **13**（增加修改既有 `src/senior_control/motion/state_machine.py`）；测试 3 → **4**（增加仅迁移既有 `tests/ros/test_rgbd_diagnostic_launch.py` 遍历/PGID/退出 PID 断言，不新增场景，独立提交 `cb1932f`）。完整累计范围 **18 文件**，配置/构建/工具仍为 0，文档仍仅本文。capture/verified `slam_graph_node` 前缀已由独立提交 `705bf53` 撤下，非空目录无覆盖重开仍未完成。
- **明确的既有字段语义调整**：普通 manual/navigation 的 `source_epoch` 不变；仅 `idle` 零 barrier 以 `source_id == 当前 output boot` 精确回显已观察到的代次。无锁存时 prepared 的 request_id 为该 boot、source_epoch 为 0；有锁存时 estop 的 request_id 保持操作者原请求，source_epoch 表示原急停 revision，gesture 仍为 0。真实 v3 guard 要求普通 estop barrier 的 gesture 为 0，现有 C++ barrier/codec 已接受并原样编码上述 idle 字段，不修改 guard、消息字段、端点或协议版本；不能把 wire 布局未变说成完全没有语义调整。
- `CommandNode` 在接受真实 runtime GID/boot 后使用既有 revoke 关闭旧 manual authority，保留连接 context 以允许新的 acquire；旧导航经既有 source 清理退休，不调用 RouteService.start 或发送 goal。输出代次提升后稳定重发因果零 barrier，直至对应 boot/generation/锁存 revision 确认；交接中沿既有 receiver.invalidate 撤销 current，不能将新 Core 的初始 clear 冒充锁存已清。急停请求在 ACK 后仍保留，只在明确 reset 的 clear/generation 确认后删除。
- 只有带恢复标记的 Core 要求因果 boot echo 完成后才接 intent，时间新旧本身不够。旧 cached prepared、恢复后旧源新发的零屏障和旧 intent 均不完成交接；非匹配 estop 仍可先停车锁存。匹配 idle-estop 不将 boot 当新操作者事件，而调用 `restore_software_estop` 保留原 revision；该显式入口只接受正值及不回退已知 revision，退休源、保持软件锁存，不改变普通 SOFTWARE_ESTOP 递增或显式 reset 匹配。原 request_id 通过 C++ v3 编解码进入 Core barrier payload，不被 source_id 的 boot 回显替换。
- 恢复 Core 的 LearningClient 使用现有 read_only 模式，StartupLearning 不绑定写会话且 `_startup_target` 不发起动作；继续消费只读完成 view。首次普通 Core 仍使用原学习客户端、原 idle prepared heartbeat 和一次学习规则；7b5a034 原始学习/派生避障隔离段未改。
- 获批场景 7 的同一生产链回归复用现有 Command 生产方法提取、LearningRig、LearningClient 线程/真实 record owner 与既有 v3 C++ header；动态命令穿过实际 C++ barrier/codec → Python decoder → FinalCommandFence → MotionCore。覆盖未开始/已消费学习不发 Attach/Reserve、原启动学习不变、旧 manual/nav 退休、新 acquire 恢复、旧零/intent 拒绝、非匹配 estop 先锁不交接、原 request 与 revision=3 保持、普通 stop 不清锁、错误 revision reset 无效/正确 reset 有效。未将同一场景拆成参数矩阵或增加独立故障场景。
- 定向新生产链加两项现有直接回归：`pytest -q tests/unit/test_launch_recovery.py tests/unit/test_manual_command_generation.py tests/unit/test_startup_learning_chain.py -k 'recovery_handoff or manual_intents_use_prepared or real_tick_collects_once'` → 3 passed、8 deselected（4.93 秒）；最后只针对核心接线调整重跑 `-k recovery_handoff` 通过。没有重跑已有 Mac/Linux 前缀三项。真实 installed Command → RMW guard → Core 的隔离域用例已写入既有获批 live 文件，本机无 ROS → 1 skipped、1 deselected，不冒充实际 RMW/单一 Twist owner 通过；ruff、格式、差异检查通过。
- 目前 safety/仿真 command 恢复开关仍关闭，IPC 端点与 Nav2/Gazebo 组仍待实施。若 command 自身先重启而失去原 estop 请求身份，随后又发生 output 换代，不能捏造原身份；当前只保留停止且不发 clear 交接，需新的显式急停才能建立可确认原请求。这种复合故障的原身份延续尚未证明，不把局部生产链通过写成全部 critical 完成。

### 仿真 launch IPC 端点交接阶段（command 恢复开关仍关闭）

- `simulation_motion` 仅给自己创建的 command 节点传入随机、同次 launch 不变的私有 `SENIOR_LAUNCH_SOCKET_OWNER`；叶节点替换仍携带同一上下文，不改变学习 claim，不新增用户 CLI 或业务端点。
- `agent_adapter._launch_endpoint` 仅在明确上下文存在时持有既有端点旁的同 UID/私有常规文件锁，覆盖准备、bind、记录、服务关闭整个生命周期。初次启动不删 socket；带恢复标记时复用 helper `_prepare_socket(expected_run_id=...)`，要求同一 owner/run、同一 inode/uid、连接明确拒绝且身份复核不变才清理。bind 成功后复用原 `_record` 格式更新身份；活/陌生/不确定 probe 不清理，replace_existing=true 的 launch 上下文被拒绝。锁文件不在退出时删除，进程退出由内核释放锁。
- 实车 helper 直接 adapter 不传该上下文，继续原 PID/SO_PEERCRED 与已有停机流程，不新增其自动重启。helper 新增可选 expected_run_id 只约束 launch 恢复；原调用未传参数时语义保持。
- 获批场景 11 使用既有短路径 socket fixture 和真实本机 Unix socket 验证同 owner refused stale 可清、活 socket 原 inode 保留、陌生 owner stale 原 inode 保留；复用已有 active/unrecognized 直接回归，两项定向 `pytest -q tests/unit/test_launch_recovery.py tests/unit/test_docker_start_helper.py -k 'launch_endpoint or active_or_unrecognized'` → 2 passed、13 deselected（0.32 秒）。这不是实际 installed agent_adapter 进程重启通过；本机未运行仿真 command。
- 只读核对 `LifecycleService.reconcile`：已 sending 的 outbox 只走 query，unknown=None 不执行 execute；`AdapterCore.navigation.task_status` 未知 effect 返回 accepted=None，不调用 routes.start。沿用这些已有行为，不加入任务持久化新框架；对应现有 unknown-send 直接回归定向执行，无修改该测试文件。
- 现有 `pytest -q tests/integration/test_task_lifecycle_service.py -k unknown_send_is_only_queried` → 1 passed、29 deselected（0.36 秒）；四个改动 Python 文件的 ruff/format/AST 通过，CRLF-aware diff check 通过。已亲查本阶段产品 diff，未改变 Core、启动学习算法或任何 critical 恢复开关。

仅沿已列真实恢复路径验证，不追加 benchmark、全量回归或宽泛异常矩阵：

1. 已确认相机崩溃路径：新 PID/原始图像/发送帧恢复、崩溃日志保留、原 Agent 与学习 run 不变。
2. 真实 scan receipt/adapter 崩溃：新身份恢复真实新帧，旧身份拒绝；不重起健康设备。
3. Nav2 真实 server 与 manager 退出各自路径：最终 ACTIVE、无重复 owner、旧 goal 不重发；正常取消/完成期间没有复活执行。
4. safety launch 节点真实崩溃：仅一个 Twist owner、新代次初始零、旧手动/导航退休、已有急停仍锁、已用及尚未开始的学习均不因恢复发起动作。
5. 已由用户取消：Gazebo server 真实崩溃场景（对应选定候选 10），本轮不编写或执行。

这些是待负责人筛选的候选场景，不代表本次运行过或通过；不同独立输入需按实际场景计数。重点人工复核前缀解释器/日志、停止与重试竞态、Nav2 ACTIVE、estop/旧目标交接四处；任何未通过处报告具体缺口，不能用局部 mock 替代实际链路，也不能以缺口为理由扩成内部子进程监督。


### Nav2 navigation / localization 组阶段

- navigation 与 simulation_motion 定位分支通过当前解释器的私有入口，使用公开 LaunchService/IncludeLaunchDescription/Shutdown 重入同一项目 launch；成员定义只保留一份，仍仅传入原参数，不新增 launch 参数、ROS 端点或依赖。原 Node 参数/条件/remapping 已按 AST 逐项比对，差异仅为恢复前缀、退出预算和显式原生重连默认值。
- server/adapter 使用原叶前缀；manager 仅通过 exec 引导建立自身清理 PGID，不设叶重试循环。真实 manager 非零退出触发公开 Shutdown，LaunchService 等待所有旧叶退出和 manager 组清理完成，再固定退避两秒；任何叶清理失败、旧 manager PGID 残留均拒绝替代。正常零退出、外层停止、退避取消不重开；组运行复用已有 Linux 父死 SIGTERM 保障。叶清理五秒，launch 给六秒余量；外层九秒，保持 helper 十秒预算。
- 只读标准 GetState 与 manager is_active 同时确认 ACTIVE。只有原叶模块明确记录真实非零退出后才关注 native manager 的重连窗口；窗口复用原 10 秒及原生时钟，另留原生首轮一秒。仅 pause/reset 或服务探测不确定不触发重建。崩溃前发出的查询结果不能确认本次恢复；不解析厂家警告、不接管 configure/activate。
- 公开 API 与时钟核对来源：[Humble LifecycleManager](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_lifecycle_manager/src/lifecycle_manager.cpp)、[Humble LaunchService](https://raw.githubusercontent.com/ros2/launch/humble/launch/launch/launch_service.py)、[Humble LaunchContext](https://raw.githubusercontent.com/ros2/launch/humble/launch/launch/launch_context.py)。SIGTERM 转正常 Shutdown，退出清理以公开 add_completion_future 等待；没有跨厂家私有字段或 monkeypatch。
- 获批候选 8/9 写入同一 live 文件：复用既有 navigation 的合成 map/TF/scan 和 RGBD 进程遍历，真实 planner SIGKILL 后新 PID + ACTIVE 且 manager/健康 controller 不换；真实 manager SIGKILL 后旧节点全部消失、新组 ACTIVE，旧 goal UUID 不由新 BT writer 重发。无 /cmd_vel owner、无实车设备；Linux /proc 中 zombie 不算退出。未增加重连超时故障场景。
- 本机没有 ROS，真实 Nav2 与定位 ACTIVE、Linux manager/叶组清理仍未验证，不能把本地收集/语法检查或进程存在当作恢复成功。此前定向 pytest navigation 配置三项通过，两项真实 Nav2 用例明确 skip；不重跑已充分的叶前缀 Mac/Linux、critical/IPC 检查。
- Gazebo 候选 10 从本轮删除，新增故障场景减少一项；Gazebo 专属产品和测试均未写入，`senior_sim/simulation.launch.py` 目前未改。Nav2 定位与 command IPC 不属于 Gazebo world 恢复，保留。


### 用户最终缩减仿真环境范围

- 负责人确认按用户“仿真环境人工处理”方向，取消 `ros2/src/senior_sim/launch/simulation.launch.py` 内 server/world/GUI、robot_state_publisher 和 senior_sim_bridge 的专用恢复；文件未修改，相关新产品代码与测试均未写入，无回退或撤销提交。
- 产品 13 → **12**，总数 18 → **17**（产品 12、测试 4、文档 1）；Gazebo 候选 10 取消，新增故障场景减少一项。create/spawner 保留原单次启动退出，实车状态发布恢复、simulation_motion 的 Nav2/定位与 command IPC 成果保留。
- Nav2 代码阶段提交为 `87b63d1ab0aae3c6d30e0fbab55f0b668a574cfc`，负责人正在亲审；真实 ROS Nav2/定位 ACTIVE 验收未完成，safety/command 恢复开关继续关闭。本次仅更新同一计划，不新增或重复测试，不自行部署/启动服务。

### 续接：真实 Linux 隔离验收入口与待开放差异

此处是负责人串行执行的最小入口，worker 不 SSH、不运行 ROS 节点或服务。待验收源码须另行放在 Linux 独立目录 `RECOVERY_SRC`，内容为本 worker 待验收提交；不得用车端正在运行的 editable 源目录，也不将本隔离版本混入现车 61555f8。沿用现有 Humble、厂家 underlay、项目原生消息/guard/receipt 和 `.ros-venv`；不安装任何新依赖。Nav2 包/插件缺失是验收阻塞，不能以 skip 或自行安装代替通过。

在新的 bash 中设置 `RECOVERY_SRC` 为该独立源码的绝对路径；确认 87 为负责人选定的空闲 ROS 域（若占用，先改为另一空闲域），执行以下准备。临时构建只写新目录，不触碰生产 build/install 或服务：

```bash
set -e
: "${RECOVERY_SRC:?set an independent Linux source directory first}"
export RECOVERY_PY=/home/wheeltec/ros2_shared_dir/senior-robot-control/.ros-venv/bin/python
export RECOVERY_PRODUCTION=/home/wheeltec/ros2_shared_dir/senior-robot-control
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
source "$RECOVERY_PRODUCTION/ros2/install/setup.bash"
test "$(realpath "$RECOVERY_SRC")" != "$(realpath "$RECOVERY_PRODUCTION")"
export RECOVERY_BUILD="$(mktemp -d /tmp/senior-launch-recovery.XXXXXX)"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export SENIOR_LAUNCH_RECOVERY_LIVE=1
unset SENIOR_STARTUP_LEARNING_CLAIM SENIOR_LAUNCH_RECOVERED SENIOR_AGENT_SOCKET SENIOR_LAUNCH_SOCKET_OWNER
cd "$RECOVERY_SRC"
"$RECOVERY_PY" -m colcon --log-base "$RECOVERY_BUILD/log" build \
  --base-paths "$RECOVERY_SRC/ros2/src" --packages-select senior_robot \
  --build-base "$RECOVERY_BUILD/build" --install-base "$RECOVERY_BUILD/install"
source "$RECOVERY_BUILD/install/setup.bash"
export PYTHONPATH="$RECOVERY_SRC/src:$RECOVERY_SRC/ros2/src/senior_robot${PYTHONPATH:+:$PYTHONPATH}"
```

该构建方式复用项目既有 `python -m colcon`，使入口解释器与 `.ros-venv` 一致；仅 overlay 新 senior_robot，原生包继续由现有项目 underlay 提供。本任务未修改 C++/消息/guard，不能混用不兼容的既有原生包。先只读核对依赖与源码/launch 版本，不启动任何节点：

```bash
"$RECOVERY_PY" - <<'PY'
import importlib
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rosidl_generator_py import import_type_support
import senior_control.runtime.launch_recovery as recovery
import senior_robot.command_node as command
root = Path(os.environ['RECOVERY_SRC']).resolve()
assert Path(recovery.__file__).resolve().is_relative_to(root)
assert Path(command.__file__).resolve().is_relative_to(root)
for module in ('pytest', 'rclpy', 'launch', 'launch_ros', 'nav2_common',
               'lifecycle_msgs.srv', 'std_srvs.srv', 'nav2_msgs.action',
               'action_msgs.msg', 'tf2_ros'):
    importlib.import_module(module)
import_type_support('senior_motion_msgs')
for package in ('nav2_controller', 'nav2_planner', 'nav2_bt_navigator',
                'nav2_lifecycle_manager', 'nav2_costmap_2d', 'nav2_smac_planner',
                'nav2_mppi_controller', 'nav2_behavior_tree'):
    print(package, get_package_prefix(package))
for package, executable in (('senior_motion_msgs', 'final_command_guard'),
                            ('senior_motion_msgs', 'motion_output_guard'),
                            ('senior_motion_msgs', 'native_receipt'),
                            ('senior_sensor_ingress', 'scan_receipt_node'),
                            ('senior_robot', 'safety_node'),
                            ('nav2_controller', 'controller_server'),
                            ('nav2_bt_navigator', 'bt_navigator')):
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    assert os.access(path, os.X_OK), path
share = Path(get_package_share_directory('senior_robot'))
for name in ('navigation.launch.py', 'simulation_motion.launch.py'):
    assert (share / 'launch' / name).read_bytes() == (
        root / 'ros2/src/senior_robot/launch' / name).read_bytes(), name
print('imports, native executables and candidate launch files verified')
PY
```

先执行现有已批准 critical 真实链一项，再执行 Nav2 两项；都必须实际 passed，skip 不通过。现有 live fixture 在创建被测节点前验证 ROS 域内仅有自己的探测节点，发现其它节点立即失败。critical 使用现有 `output_topic` 参数将原生输出放在 `/launch_recovery/cmd_vel`，断言该测试话题唯一输出且全零，物理 `/cmd_vel` 无发布者/订阅者；没有 scan 供给、没有学习 claim/service。Nav2 合成 map/TF/scan，仅私有输出，无物理 `/cmd_vel` owner，目标只是已批准的旧 goal 不重发检验。

```bash
"$RECOVERY_PY" -m pytest -q -p no:cacheprovider tests/ros/test_launch_recovery_live.py \
  -k installed_command_guard_core_recovery
"$RECOVERY_PY" -m pytest -q -p no:cacheprovider tests/ros/test_launch_recovery_live.py \
  -k native_nav2
```

获批 scan receipt 新 GID/epoch 用例若尚未在 Linux 验收，单独串行执行 `"$RECOVERY_PY" -m pytest -q -p no:cacheprovider tests/ros/test_launch_recovery_live.py -k scan_receipt_crash`；不在此重跑已有充分的三项前缀、critical 非 ROS 生产链与 socket 检查。以上命令不启动 helper/Agent/LearningService、厂家底盘或 Gazebo，不调用项目 start/stop，不改变当前服务。

**尚未应用的准确接线补丁**：只在两份已批准 launch 中给三个现有 Node 加下列三行，无新配置/参数/公开 CLI。每处 `prefix=restart_prefix()` 只拥有该节点一次异常重试，正常 0/取消不复活；六秒 launch 余量允许叶五秒清理完成。safety_motion 的 helper 直接 command 不套前缀、不破坏 SO_PEERCRED。

```diff
--- ros2/src/senior_robot/launch/safety_motion.launch.py
+++ ros2/src/senior_robot/launch/safety_motion.launch.py
@@
             executable="safety_node",
+            prefix=restart_prefix(),
+            sigterm_timeout="6.0",
+            sigkill_timeout="1.0",
             parameters=[runtime["safety_gate"], selected, {"robot_id": robot_id}],
--- ros2/src/senior_robot/launch/simulation_motion.launch.py
+++ ros2/src/senior_robot/launch/simulation_motion.launch.py
@@
             executable="safety_node",
+            prefix=restart_prefix(),
+            sigterm_timeout="6.0",
+            sigkill_timeout="1.0",
             parameters=[
@@
             executable="agent_adapter",
+            prefix=restart_prefix(),
+            sigterm_timeout="6.0",
+            sigkill_timeout="1.0",
             name="command_node",
```

这不是验收命令一通过就无条件开放的承诺：真实 ROS 交接/清组未证实前上述补丁不落产品；复合 command/output 故障的原 estop identity 丢失仍是未完成项，不能以新显式 estop 建立新身份冒称原身份延续。另只读发现 DedicatedController 以 `start_new_session=True` 启动其私有 controller/BT，现有异常父死路径未见 parent-death 绑定；这两组不在 command 的叶 PGID 内。虽然对子进程独立监督明确不在范围，command 真退出后的既有私有 owner 清理仍须先证明。不能仅凭 launch IPC stale 清理通过就开放带导航的 command 恢复，也不能为通过验收私自把 DedicatedController 纳入新监督层或扩大文件。

本轮仅修正已批准 live 场景的真实接线与隔离：Command 与 Nav2 状态探测复用生产 MotionExecutor/with_dds_info，按 Humble 现有 MessageInfo dict 取真实 publisher_gid，避免普通 executor 丢失 metadata。未新增测试场景或产品文件。本机定向这三项为 3 skipped、1 deselected（无 ROS），不表示 Linux 行为通过；本轮剩余工作明确依赖负责人运行上述隔离用例并返回实际结果，以及收口上述 command 私有退出资源/复合急停身份缺口。

### 真实 Linux 首轮反馈：修正同一 critical 用例前提

- 负责人已从 `837a0c2` 的独立源码完成临时 senior_robot 构建（7.55 秒）及全部 Nav2 包、原生 exe/import、候选 launch 一致性核对。critical 首轮实际 1 failed / 3 deselected（22.85 秒），停在旧 authority 的第二个 drive.intent 返回 ok=False，尚未到后续急停断言；不能记录为交接/急停通过。Nav2 两项由负责人独立执行中。
- 具体原因链：`CommandNode.manual_request` 的 ingress bind/view 调用 `ManualAuthorityManager._observe` → `ManualSession.expire/_live`；300ms active 断流 TTL 在固定两秒重启退避中已到期，authority 变 CLOSED/expired。随后 `ManualAuthorityManager.accept` 拒绝 CLOSED，`ManualIngress.accept` 返回 None，manual_request 返回 `error=conflict`。本地直接调用同一生产 ManualIngress 验证：active 两秒后 closed/expired 且拒收，prepared 两秒后仍 prepared 且首次新鲜 intent 可接受；没有更改 TTL、clock 或资格。
- 经负责人确认，仅修正同一个真实场景：故障前 acquire 后保留真实 prepared authority 与生产 cached barrier，不先激活再断流；新 Core 已起、Command runtime 回调仍排队时，由真实 manual_request 首次生成该旧 authority 的合法 intent。通过实际 DDS 订阅取得生产 FinalCommand，再以原身份、原 received_steady、原 deadline 原样重发；不镜像协议、不复活 CLOSED authority。要求重发时仍在原期限内，并观察随后四个不同新 Core sequence 的 idle，再继续已批准的 boot 交接退休与 revision=3/request/reset 断言。
- 本机仅对这一受影响用例收集：1 skipped / 3 deselected（无 ROS）；ruff/格式通过。实际 Linux 是否通过待负责人只重跑这一此前失败项，已有前缀/critical 单元/IPC 与运行中的 Nav2 不重复跑。产品代码、恢复开关均未因此修改。


### 必要私有 owner 父死清理（新增既有产品文件已获预报批准）

- 原问题为 DedicatedController 用 ros2 run 包裹独立 session，Command 被 SIGKILL 无机会执行 AsyncExitStack，私有 controller/BT 不在 Command 叶 PGID 内。最小修复仅 `_start`：ament 解析同一个包/原生可执行，沿用原 argv、参数、私有 cmd_vel remapping、session、configure/activate 顺序与 `_stop`；启动命令前加已有模块的私有 exec-owner 引导，exec 保留原生 PID/PGID，没有额外长驻监督进程或内部重启循环。
- exec-owner 的预期 parent PID 在创建子进程之前由真实 spawner 捕获并传入；child_lifetime 先登记 Linux PR_SET_PDEATHSIG(SIGTERM) 再复核 PID，拒绝已经换父的启动，不把 PID 1 当原 owner。Nav2 manager 复用同一引导并同步使用精确 parent PID；没有改动已经验收的 run_node 叶重试路径。
- 复用已批准 unit 文件/真实进程设施作两个直接检查：Linux 上原 owner SIGKILL 后两台非运动原生替身接收 SIGTERM、退出并被独立测试 reaper 实际 waitpid 回收（zombie 不算成功）；已有 startup 取消路径在生命周期等待中取消后原组消失，并确认预期 parent 不匹配时不 exec。生命周期等待由替身阻塞，仅验证进程退出资源，不冒称 Nav2 ACTIVE 或实际原生控制器通过。本机 1 passed、1 skipped、5 deselected（Linux 父死项明确 skip）；Linux 真实父死项仍需负责人运行。
- 专用最小 Linux 命令：在上述独立候选源码与现有解释器中执行 `PYTHONPATH=src "$RECOVERY_PY" -m pytest -q -p no:cacheprovider tests/unit/test_launch_recovery.py -k dedicated`。这两项不启动 ROS 节点、学习、物理设备或服务，不重跑已有三项叶恢复/critical/IPC 测试。只能以 Linux 实际结果确认父死清理，不能把 macOS startup 取消结果代替它。
- 原复合 command/output 急停 identity 延续缺口仍未闭合，command 恢复开关继续关闭。真实原生 controller/BT 若拒绝 SIGTERM 或清理失败，不能假称此父死信号等同所有残留已回收；此处未新增强杀监督或外层重开授权。

### 真实 Nav2 配置前提失败（只读定位）

- 负责人从 837a0c2 运行两项实际 2 errors / 2 deselected（70.21 秒）：都在首次 configure controller_server 时因 installed pluginlib 无 `mppi::critics::CostCritic` 失败，未到崩溃注入。所有进程已退出清理；未把初始化错误记成恢复失败，也未宣称 ACTIVE。
- `config/nav2.yaml` 的 critics 列表/CostCritic 设置从初始 11393ee 已存在；b3664cd 的 blame 主要来自当时全文件行尾归一化，并非本次恢复引入。现有 test_navigation.py 的真实启动也直接用默认 launch，未提供可复用的兼容参数覆盖；仓库无另一份已获准 MPPI/Humble 参数 fixture，静态 CostCritic 断言只证明字段存在。
- 这是当前 installed 与默认产品配置不匹配的实际启动缺陷，具体版本/overlay 归因仍需其实际 package.xml/critics.xml/导出路径。上游固定 [Nav2 1.1.20 critics.xml](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/critics.xml) 本来导出 CostCritic，不能泛称 Humble 全部没有该类；[同版本 MPPI 文档](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/README.md) 分别说明 CostCritic 直接 costmap 评分与 ObstaclesCritic 距离/权重评分，不能改名声称行为等价。
- 本轮不改配置/安装依赖、不删除 critic，不重复失败两项；若后续用获准的特定参数验证恢复，只能报告该参数下的恢复行为，默认生产导航不可启动仍是缺口。生产配置整改至少涉及既有 nav2.yaml 和 test_navigation.py 对应断言，当前配置 0 范围外，须负责人另行收口预报。

### Linux 实际反馈与 Python 3.10 信号文本修正

- 负责人已对 `8da12b9` 独立源码重跑此前失败的 critical 场景：**1 passed / 3 deselected，30.47 秒**。真实 Command → DDS → native guard/Core、prepared/新鲜旧源拒绝、boot 退休及急停 revision/reset 通过；不因后续父死补丁重复跑，不将这一场景说成复合 command/output 原急停 identity 延续已闭合。
- 负责人亲审 `f424770` 后在 Linux 用现有同步函数和 asyncio.run（未安装 pytest async 插件、未改测试）运行新定向检查。首项的两个 child 已实际 waitpid exit=0、/proc 消失、从待回收集删除，随后仅失败于信号文本比较；第二个 async 取消函数因首项 raise 尚未运行，不能说失败或通过。
- 已在本机**实际解释器**核对：Python 3.10.22 的 str(SIGTERM) 为 `Signals.SIGTERM`、str(int(SIGTERM)) 为 `15`；Python 3.12.15 两者均为 `15`。handler 参数为 int，fixture 原本写 `15`，所以只将期望改为 str(int(SIGTERM)) 并标注 //Compatible，不改产品信号/父死策略/进程清理。依据还包括 [Python 3.11 IntEnum 变更说明](https://docs.python.org/3.11/library/enum.html#enum.IntEnum)。本地 ruff/差异检查通过；负责人只重跑受影响两函数，不重复 critical。
- actual MPPI 已确认是 `/home/wheeltec/wheeltec_ros2/install/nav2_mppi_controller` 的 **1.1.9**，导出 XML 无 CostCritic、MPPIController 本身存在。负责人不批准正式 nav2.yaml/算法整改或升级，范围仍 18 文件；仅批准在现有 live fixture 临时生成明确标为隔离恢复用的 1.1.9 参数，仍经原 parameters 入口验证同两项场景，不能称默认生产导航通过。默认定义须按该实际旧实现核对，不能把较新 1.1.20 的默认值冒充现场 1.1.9。

### Linux 父死用例二轮反馈：保留原始退出证据

- `819f2ec` 的 Linux 独立两函数检查首项在 waitpid 状态非正常零退出处再次失败，finally 又对已回收 PID 发 SIGKILL 引发 ProcessLookupError，掩盖了原始 status；async 取消函数仍未执行。本轮不能宣称父死路径完整通过，也不能在缺少 status 时断定重复信号或产品故障。
- 最小修正仅同一 unit 用例：waitpid 一旦 collected 即移出待清理集合；严格退出零断言保留，消息加入 PID、raw status、exit/signal 和两个已有 stopped 标记；失败清理先确认仍是本测试未回收 child，再核对 /proc birth 身份，忽略已失去 child 所有权/已消失的 PID，不重复信号已回收或复用 PID，以便继续尽力回收其它已知 child，保留原始失败。未改产品或扩大场景。
- ruff/格式/差异检查通过；本机无 Linux，不冒称真实父死结果已修复。负责人只重跑这两个受影响函数取得原始状态，已通过 critical 不重跑。

### 获准的临时 MPPI 1.1.9 恢复 fixture

- 负责人已只读获取实际对应源码 `/home/wheeltec/wheeltec_ros2/src/navigation2-humble/nav2_mppi_controller/src/critics/obstacles_critic.cpp`：评分默认 repulsion_weight=1.5、critical_weight=20、collision_cost=10000、collision_margin_distance=0.10、near_goal_distance=0.5、cost_power=1；原生 consider_footprint 默认为 false。inflation 分支默认 scaling=10/radius=.55，与项目 .35/3 不同，不能直接套新版或原生 inflation 默认。
- 仅在既有 native_navigation live fixture 中读取生产参数副本，明确命名临时 `test-only-nav2-mppi-1.1.9.yaml` 并带“仅隔离恢复测试”注释，经既有 parameters 入口使用。只将不支持的 CostCritic 选择/参数块替换为已安装 ObstaclesCritic 的上述真实评分默认；不映射 CostCritic 权重。保留项目原有 consider_footprint=true 的碰撞模式，inflation 从同一原参数 local_costmap 读取为 .35/3；其余速度、footprint、TF、传感话题、controller/planner/BT 及 launch 参数不变。
- 0.10 来自厂家旧 critic 评分默认，仅用于无底盘合成场景；不是新安全框、停车资格或产品避障修改。正式 nav2.yaml 未改、未升级依赖，当前默认生产导航仍因缺少 CostCritic 不可启动；此配置未来通过也只验证该临时参数下两个既有恢复场景，不表示算法等价或生产导航通过。
- 本机仅对两项受影响 live 场景收集为 2 skipped / 2 deselected（无 ROS，0.03 秒）。另外直接执行同一 fixture 的参数生成代码，YAML roundtrip 成功；恢复原 CostCritic 选择/参数后整个结构与生产原 YAML 完全一致，确认只有该 critic 区域不同。ruff/格式/差异检查通过；不重复已通过 critical 或原前缀/IPC 检查，不新增用例或跟踪配置文件，范围仍 18 文件。负责人亲审后仅运行此前两项 Nav2 初始化失败场景。

### Linux 定向退出回收当前结果

- 负责人亲阅 `a0b8309` 的实际诊断 diff 后，在独立 Linux 源码、各自 TemporaryDirectory 中直接调用现有同步父死函数及 asyncio.run 现有 async 取消函数：**DIRECT_RESULTS passed2 / failed[]，exit=0**。没有安装 pytest async 插件、没有改现场测试或生产服务；两台替身进程已回收，本次严格 exit=0 与 SIGTERM 标记、启动取消/parent 不匹配检查通过。
- 保留 `819f2ec` 那次未知非正常 wait 状态失败：后续一次通过没有解释此前原因，诊断修正也不是产品信号策略修复，不据此宣称产品退出路径稳定或所有真实 Nav2 native 退出已验收。不会为该未知失败重复跑循环或扩大矩阵。
- 临时 MPPI 1.1.9 fixture 代码提交 `6b9340468847f26bca5f24cb851b78adf1fb3ad7` 已交负责人，待仅重跑原 Nav2 两项；上述退出回收与 `8da12b9` critical 已有充分结果，不因文档记录或临时参数变更重复跑。safety/仿真 command 前缀仍未开放，复合急停 identity 延续缺口保持。
