# Launch 节点异常恢复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 本文仅供负责人审阅；尚未授权执行产品或测试修改，不另派子 Agent。

**Goal:** 项目入口启动的所有长驻 launch 节点异常退出后自动恢复；正常完成、主动关闭和已取消的导航执行不被重新启动。

**Architecture:** 保持现有 launch、厂家 include 和模块 owner，优先使用 ROS launch 的进程、退出事件、停机与延迟机制；仅以一个小型进程前缀适配模块补足厂家 include 内部的异常退出筛选。Nav2 生命周期组和 Gazebo 环境组按各自 owner 恢复，控制侧仅补启动代次交接，不扩展到内部子进程监督。

**Tech Stack:** Python 3.10+、ROS 2 Humble launch/launch_ros、现有 Nav2 lifecycle manager、Fortress/ros_gz_sim；不新增依赖、公开 CLI 或平台。

**Spec:** 根目录 `../CURRENT_TASK.md`、`docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md` 及本次用户“所有 launch 管理节点自动恢复”的明确要求，冲突处以根任务和最新指令为准。

## 状态与约束

- 本文收口于 `fix/ros-typesupport-export-order@e97720f781d8aa18d5697768b3d7166dfab860b4`；该提交包含原盘点基线 `4bde9a6` 及负责人后续文档/回归，未覆盖后续成果。
- 已批准的是上述行为与本次新增这一份文档；下列实现方式、文件清单和验证候选等待负责人审阅，不能将先前 19 文件候选当成已批准范围。
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
- 私有组运行入口仅供下述两个 launch 自身调用：使用 `LaunchService`、`IncludeLaunchDescription`、`OnProcessExit`、`Shutdown` 管理既有组，并把指定 owner 的异常退出转换成组异常结果；无通用服务注册、持久调度器或配置平台。
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
| senior_sim/simulation | ros_gz_sim 的 server/可选 GUI、`robot_state_publisher`、`senior_sim_bridge` | server 属环境组；桥、状态发布者独立恢复；GUI 独立退出不复制 server |
| senior_sim/simulation 单次工具 | `create`、`joint_state_broadcaster` spawner、`drive` spawner；xacro/world_reference 生成 | 正常成功不复活；仅在新环境组初始化时各执行一次，异常恢复限于仍有效的该次初始化，关闭/取消后不重试 |

### Nav2 与 Gazebo 的组边界

1. Nav2 server 异常：前缀恢复真实 server，使用现有 manager 的 `attempt_respawn_reconnection=true`、bond reset/startup 完成 configure/activate；采用上游已有窗口（本地源码默认 10 秒），不能据进程起来就宣称恢复。超出窗口时由同一 navigation/定位组进行一次有序重建，不另造 lifecycle 协调器。
2. Nav2 manager 自身异常：在所属 launch 建立的独立组运行边界内结束该组，等待所有旧 server/adapter 退出，再创建全新的 manager 与成员；仍活跃的 ACTIVE server 不可直接配一个新 manager。组运行调用同一项目 launch 的内部成员分支，成员分支不再创建组运行入口，避免递归；只传原参数、不复制节点定义。正常组关闭不触发恢复。外层保留 Agent、safety、其他 launch。
3. Gazebo server 异常：同一方式重建仅 senior_sim 环境组，等待旧 world/插件 controller_manager/drive 退出，重开 server，再按创建成功顺序执行 create → joint_state_broadcaster → drive；原来的三个单次工具无常驻重试前缀。bridge/状态发布者继续各自拥有恢复，不把健康控制链并入环境组。
4. 实机 ros_gz_sim 安装 launch 的 server/GUI 实际进程及退出行为尚未现场核对；实施前由负责人一次性只读确定精确的 owner 匹配，不能以“任意子进程退出”重开 world。Nav2 重连超时判定复用既有 lifecycle state/service，不增加运动许可接口。

## critical 交接与端点清理：仅保留三个必要产品文件

`command_node.py`：新增集中方法 `_on_output_incarnation_changed(previous, current) -> None`，在已验证的 `/motion/runtime` boot/GID 更换处调用；退休旧 manual authority、撤销当前 navigation source/arbiter execution，提升现有 generation，发布原协议的零速 barrier。现有 RouteService 对 source_active 的检查将结束旧执行，不能在恢复回调里调用 start/发 goal。保存“最后已接受、尚未被明确复位的急停请求”，不能在收到一次 ACK 后丢掉锁存恢复所需的事实；新输出 boot 下用原请求身份重建现有 estop barrier，禁止把它变成 estop_reset。source 清理不能覆盖待确认急停。

已核对 `FinalCommandFence.accept` 要求新发布者/boot/generation 首帧为零速 barrier，现有非零旧 intent 不能直接建立新输出代次；这可复用，但单独不足以恢复锁存。输出换代处理必须在将新 runtime 当作当前 clear 状态前保存并接续上一代的已锁 estop，清掉旧目标再发布 barrier，防止取消窗口中的迟到 intent 复活。command 自身重启时 safety 不随之重启，原输出锁存保持，command 初始无旧 lease/目标；不能由 prepared/普通 stop 清除输出急停。

`motion_core_output.py`：恢复标记只禁止 `_startup_target` 发起学习动作，继续消费 LearningClient 已完成的只读 view；不得进入新的 reserve_action，不删除完成模型、不改变 LearningService 的 run/已消费事实。初次明确程序启动仍遵循原学习规则。这个变更放在恢复接线处，不重构 StartupLearning 或控制算法。

`agent_adapter.py`：仅处理 simulation_motion 中由 launch 管理的 command 节点的 IPC 端点重开。现有 `ipc_replace_existing_socket=false` 加上 IpcServer 的 bind 行为，意味着崩溃遗留的 socket 会阻止替代进程启动；不可通过改为无条件 replace 来修复。该 launch 节点首次成功绑定后记录既有 socket 身份格式，恢复启动复用 `_prepare_socket` 的检查语义：同一所属 launch/run、同一 inode/uid、连接明确拒绝且身份复核未变化，才删除 stale socket，再绑定并更新身份；活 socket、陌生身份或不确定探测均不删除。记录与清理只对 launch 明确传入的端点所有权上下文启用；实车 helper 直接启动路径保持现有 PID/所有权流程，不能两边同时清理。这是 launch 节点恢复必需的端点交接，不含 RouteService/DedicatedController 监督或新业务协议。

以上只是最小接口设计，不是已验证闭环：必须实测新输出 boot 下旧 intent 被拒、零 barrier 重新建立、已锁 estop 仍锁住、旧任务不重发。sim command 进程恢复后，既有任务 outbox 的 unknown 查询不得转为重发 goal；当前 `LifecyclePort` 已声明 unknown 不授权重发，需用既有实现验证这一行为，若不成立先报具体缺口和最少文件再改，不能在本计划暗加任务持久化框架。

## 明确剥离的非 launch 范围

`ControlInputs/motion-reception`、`RuntimeReports/motion_reporter`、`final_command_guard`、`motion_output_guard` 是活 owner 内部创建的进程；`DedicatedController` 直接 subprocess 创建的执行作用域 controller/BT 也不是 launch 节点。本轮不为其新增检测/重启，不修改 `control_inputs.py`、`runtime_reports.py`、`safety_node.py`、`motion_output.py` 或 `dedicated_controller.py`；`agent_adapter.py` 仅作上一节的 launch 端点交接，launch 节点真正退出时清理其残留资源属于必要退出清理，不能改写成对子进程独立监督的授权。

实车 helper 直接启动的 agent_adapter 也不属于 launch，本轮不加其自动重启、更不套 wrapper 破坏真实 PID 的 SO_PEERCRED。helper 对 camera 的改变仅是传入相同前缀配置，以及 camera 外层明确异常时独立恢复；不重启 motion 外层来同时重建 safety/command 状态。

已有扫描换代路径位于 scan_receipt、scan_adapter、safety_scan_ingress、matcher 与 EgressAdmission；允许新 GID/epoch 进入、拒绝旧身份，不需为恢复更改 admission。真实话题/TF 是否恢复以及有限历史容量不能凭参数设置宣称通过。

## 准确候选文件清单与数量

产品代码 12 文件（新增 1、修改 11、删除 0）：

1. 新增 `src/senior_control/runtime/launch_recovery.py`：前缀、单进程异常恢复、两个既有组的 launch 运行边界。
2. 修改 `src/senior_control/docker_start_helper.py`：camera 启动接线/独立外层异常恢复。
3. 修改 `ros2/src/senior_robot/launch/safety_motion.launch.py`。
4. 修改 `ros2/src/senior_robot/launch/simulation_motion.launch.py`。
5. 修改 `ros2/src/senior_robot/launch/navigation.launch.py`。
6. 修改 `ros2/src/senior_robot/launch/mapping.launch.py`。
7. 修改 `ros2/src/senior_robot/launch/rgbd_diagnostic.launch.py`。
8. 修改 `ros2/src/senior_robot/launch/simulation_rgbd.launch.py`。
9. 修改 `ros2/src/senior_sim/launch/simulation.launch.py`。
10. 修改 `ros2/src/senior_robot/senior_robot/command_node.py`：现有输出代次与锁存交接。
11. 修改 `ros2/src/senior_robot/senior_robot/motion_core_output.py`：恢复不发起学习动作。
12. 修改 `ros2/src/senior_robot/senior_robot/agent_adapter.py`：仿真 launch 所有 IPC 端点身份记录与 stale socket 清理。

测试 3 文件（新增 2、修改 1、删除 0）：`tests/unit/test_launch_recovery.py`（新增）、`tests/ros/test_launch_recovery_live.py`（新增）、`tests/unit/test_docker_start_helper.py`（修改）。优先复用既有真实进程设施，不实质重写其它测试。

配置/构建 0；工具 0；文档仅本文件新增 1，后续在本文更新结果，不另写报告。完整候选共 **16 文件：新增 4、修改 12、删除 0**；本次实际只新增本文 1 个文件。增加 adapter 端点这一项已在写入前报告具体 bind 缺口；精确厂家分支、组运行边界或 critical 交接如需再扩大，须负责人先报告新范围，不静默恢复成原 19 文件方案。

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
- [ ] 接齐 Gazebo 环境组，验证 server 崩溃后只生成一个 world/实体与唯一 drive，create/spawner 各正常结束一次，不循环；GUI/桥独立恢复不复制 world。
- [ ] 负责人亲阅实际 diff 与版本对应证据后合回；本任务不执行这些步骤或自动部署。

## 最少验证候选与 Review Focus

### 实施记录（2026-11-18）

- 视频隔离工作区从 `d670323` 正常 fast-forward merge 到冻结基线 `cdbeeea`，无独有提交或冲突，保留视频三项提交及 `7b5a034` 补点。负责人已批准按本计划实施与选定直接测试；本文件前述“等待审阅”是计划创建时的状态。
- 首个阶段仅新增独立 `launch_recovery` 前缀模块和获批的三个真实子进程测试：SIGSEGV 后固定 2 秒恢复/第二轮正常退出不复活、退避取消、旧进程组完整清理及停止回收。模块不导入 ROS、不改学习 claim，不开放任何产品 launch 恢复开关；相机接线等待负责人提供 installed prefix/递归 include 证据。
- 本地 macOS Python 3.12 的独立非运动子进程验证不代表 Humble installed launch 或相机现场通过；Nav2 ACTIVE、critical 交接、Gazebo 组及实机图像恢复均尚未实现/验收。
- 阶段命令使用现有仓库 `.venv/bin/python` 并设置 `PYTHONPATH=src`：`-m pytest -q tests/unit/test_launch_recovery.py` → 3 passed（9.37 秒）；两个新文件 `ruff check`、`ruff format --check`、AST/import 检查通过，产品模块 `mypy --follow-imports=skip` 通过。只运行获批的三个新增场景，无实机/ROS/服务操作；尚无需迁移现有回归。

仅沿已列真实恢复路径验证，不追加 benchmark、全量回归或宽泛异常矩阵：

1. 已确认相机崩溃路径：新 PID/原始图像/发送帧恢复、崩溃日志保留、原 Agent 与学习 run 不变。
2. 真实 scan receipt/adapter 崩溃：新身份恢复真实新帧，旧身份拒绝；不重起健康设备。
3. Nav2 真实 server 与 manager 退出各自路径：最终 ACTIVE、无重复 owner、旧 goal 不重发；正常取消/完成期间没有复活执行。
4. safety launch 节点真实崩溃：仅一个 Twist owner、新代次初始零、旧手动/导航退休、已有急停仍锁、已用及尚未开始的学习均不因恢复发起动作。
5. Gazebo server 真实崩溃：环境组恢复与初始化单次工具不循环；正常停止和退避窗口中的取消均不新启进程。

这些是待负责人筛选的候选场景，不代表本次运行过或通过；不同独立输入需按实际场景计数。重点人工复核前缀解释器/日志、停止与重试竞态、Nav2 ACTIVE、estop/旧目标交接、world/单次工具 owner 五处；任何未通过处报告具体缺口，不能用局部 mock 替代实际链路，也不能以缺口为理由扩成内部子进程监督。
