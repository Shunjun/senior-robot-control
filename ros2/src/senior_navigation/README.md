> **2026-11-22 当前状态：** 导航 launch、配置和行为树只由本包安装；senior_robot 的重复资产与转发 launch 已删除。 下文 B 阶段记录中的旧包名、转发路径和保留承诺属于当时历史，不作为当前安装说明。当前清单及迁移步骤见 [ROS 包归属](../../PACKAGE_OWNERSHIP.md)。

# 导航功能包

B7：既有导航行为已迁入，真实 ROS/ament/Nav2 运行待验；新探索算法未实现。
基线 `4b8f31f`，固定功能分支直接交付。保留既有建图、定位、控制和任务 owner。

## 实际目录、来源和入口

```text
senior_navigation/
  CMakeLists.txt, package.xml, pyproject.toml
  senior_navigation/
    __init__.py                 # 无 ROS 或业务加载副作用
    route.py                    # 原 Root navigation.py；RoutePlan/QualifiedRoute/RouteRejected
    navigation_adapter.py       # 原 robot 同名实现；RouteService/Nav2 action/旧独立节点
    dedicated_controller.py     # 原 robot 同名实现；每执行独占 controller 生命周期
  scripts/navigation_adapter    # ament lib/senior_navigation/navigation_adapter
  launch/navigation.launch.py  # 原两 external_controller 分支，canonical share
  config/nav2.yaml, lattice_diff.json
  behavior_trees/navigate.xml, navigate_through_poses.xml
```

Root `senior_control.navigation` 和 robot 两个旧 Python 模块显式 Compatible 转发同一对象。
`RoutePlan/QualifiedRoute/RouteRejected` 的 `__module__` 是新 canonical 位置，旧 import 身份一致；
原 MapRef/Pose2D/OperationResult/OperationStatus/DomainError/ErrorCode 均继续来自原公共 owner。
RouteRejected 原继承、code、message 与调用/catch 不变；没有复制值、异常、锁或管理状态。
`hardware_navigation` 已转发定位实现，本片不搬。

ROS 新入口为 `ros2 run senior_navigation navigation_adapter`；旧
`ros2 run senior_robot navigation_adapter` 的安装声明仍保留，指向同一个 canonical main。
旧 robot 导航 launch 只 Include 新 launch，三个原参数的默认、覆盖和上下文/namespace 透传保持。
canonical 默认参数/树/lattice 均读 `senior_navigation` share；旧 launch 的 parameters 默认仍指旧 share。
原四份 robot share 资产作为冻结 Compatible 数据保留，全部与新资产逐字节一致，setup.py 标注原因。
旧显式 `navigation_parameters` 文件覆盖继续接受，不重写用户配置路径。

实车 safety_motion 和 senior_sim/simulation_motion 的真实导航 Include 均改 canonical；
仿真 controller 参数重写及原 AMCL 参数读取也指 canonical 资产，map_server/AMCL 仍属原定位装配。
实车/仿真生产仍使用 external_controller=true，由 agent_adapter 构造同一 RouteService 和
DedicatedController；独立 launch 默认 false 的 controller/bt/navigator/adapter 组合保持，不新增节点。

## 公开契约和状态 owner

- RoutePlan 保留原 strict/extra/finite 校验、execution/task/effect/action 身份、map_id/revision、
  map_generation、task_epoch 与 poses；poses 是原 map 坐标系下米/弧度三元组，action 与 pose 一一对应且唯一。
  map_generation 是原不透明代次，不改为新时间；MapRef 属性构造仍是原类型。
- QualifiedRoute 按原顺序执行，目标前后均复核原资格；失败、切图、取消不进下一个目标。
- RouteService 拥有单 current、64条终端记录、256条 collection scope 与原 fence；
  prepare/qualified/bind/retire/release/collect/query 接原真实回调，控制/定位资格和输出权限仍由原 owner 判定。
- Nav2 取消 false/未知不撤换已接受 goal；迟到接受只取消对应 handle。原服务就绪2秒、接受/取消/结果0.8秒、
  planner观测及 DedicatedController 生命周期/停止预算不改，不实现新阈值或新导航算法。
- 私有速度仍为 `/control/navigation/run_<nonce>/cmd_vel`，独立旧节点只发 `/control/recovery_cmd`；
  底盘 `/cmd_vel` 唯一输出与原终端/新手动权限隔离仍由控制模块持有。
- 原日志名 `senior_robot.navigation_adapter` 保留。跨云车 RouteProgress/RouteStatus 仍是原 contracts 对象，
  没有把 RoutePlan 或内部 DTO 塞入 WS。

导航不导入 robot/manager 实现。原 `senior_control.models/errors`、`runtime.launch_recovery` 与其
process_resources 继续为实际公共 owner，安装仍使用原 SDK 平台闭包；管理 IPC/诊断/Clock/帧公共归属
收敛不是本片前置，也未冒充已完成。缺失地图/Home/定位协作与前沿探索留对应模块实现。

## 实际35逻辑文件

| 类别 | 新增 | 修改 | 迁移 | 删除 | 文件 |
|---|---:|---:|---:|---:|---|
| 产品 | 6 | 7 | 0 | 0 | 上述四Python模块、script、canonical launch；Root navigation、robot两兼容模块/agent_adapter/旧导航launch/safety launch、sim motion launch |
| 测试 | 0 | 3 | 2 | 0 | 原 tests/ros/test_navigation、tests/unit/test_navigation_collision_qualification 归 ros2/tests；原 launch_recovery、manager test_home、tests/ros live恢复只更新真实路径/patch owner |
| 配置构建/资产 | 5 | 8 | 0 | 0 | 新 pyproject/四资产；nav CMake/package、robot setup/package、sim package、根 pyproject/uv.lock、backend CI |
| 工具文档 | 0 | 4 | 0 | 0 | 本 README、ros2 README、原计划、verify_batch_one_install |

测试路径迁移在物理 diff 可为2R或2A/2D，逻辑单计，无删用例/新增场景。
五份原用例的 assert AST 全部一致；原其他控制/任务/终端测试仍通过兼容路径验证。

## 构建和安装边界

CMake 通过标准 ament_cmake_python 安装 Python 包、script 到 `lib/senior_navigation`，
launch/config/behavior_trees 到 `share/senior_navigation`；没有新 C++ 或消息/typesupport 目标。
robot/sim 单向依赖导航；导航声明实际 ROS/launch/Nav2插件依赖，不反依赖 robot/manager。
根 SDK/uv/CI 纳入 senior-navigation wheel，pytest 发现 canonical 源和迁入用例。
Python wheel 提供 Python 实现；sdist 含完整 ROS 构建/入口/资产源。SDK wheel 不制造 ament index 或代替 ROS 安装。

`tools/verify_batch_one_install.py --navigation-only` 在完整 SDK 中检查真实旧新身份/错误继承，
有 ament 时再核对新旧 executable、launch 与两份 share 四资产一致；没有 ament 明确 UNVERIFIED。
它不启动节点，也不是 Nav2/DDS/设备运行验收。Route/DedicatedController 仍需原公共值/恢复环境，
没有宣称脱离旧 SDK 的独立生产部署。

## 验证与已知缺口

环境：macOS26.4 arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
下述 python/ruff/mypy 使用原主仓库 `.venv/bin`，uv 使用 `UV_CACHE_DIR=/tmp/vehicle-uv-cache`。

- 原配置用例改 canonical 路径先因缺少新资产失败，迁入后原用例通过。
- 主命令：`python -m pytest ros2/tests/test_navigation.py ros2/tests/test_navigation_collision_qualification.py ros2/tests/test_navigation_reference.py ros2/tests/test_navigation_command_clock.py tests/unit/test_navigation_completion_handoff.py tests/unit/test_task_terminal_release.py tests/unit/test_terminal_collection_capacity.py ros2/tests/test_terminal_release_collection.py ros2/tests/test_launch_recovery.py -q --tb=short`：58通过、3跳过、1失败、37错误，10.72秒。
- `git archive 4b8f31f` 到 `/tmp/b7-baseline-source`，同解释器/同原文件清单（前两项用旧路径）：
  58通过、3跳过、1失败、37错误，11.25秒；全部失败/错误ID逐条相同。
  旧控制夹具仍访问已退役的 MotionCoreOutput.controller，旧恢复夹具patch不存在的 vehicle_learning.LearningClient；
  未补假字段/弱化断言。资格时钟、原路线/取消、切图/旧位姿、collision原用例与可用恢复用例保持原结果。
- 实际消费者命令：`python -m pytest senior_control/tests/test_home.py::test_navigation_adapter_cancels_route_when_reference_changes tests/integration/test_task_ownership.py::test_existing_ipc_receiver_records_actions_and_task_epoch tests/integration/test_task_ownership.py::test_runtime_runs_offline_and_completes_same_identity tests/integration/test_task_ownership.py::test_runtime_queries_unknown_plan_without_second_submission tests/integration/test_task_ownership.py::test_accepted_targets_are_immutable_when_future_schedule_is_edited -q --tb=short`：6通过，0.85秒。
- MYPYPATH 取根 pytest pythonpath：`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  当前和4b8f31f归档均269源码/39旧错误，归一行号后的诊断相等；首次兼容导出出现4项隐式reexport错误，
  改为直接显式转发原公共值 owner 后恢复基线，没有类型ignore或新公共类型。
- 原 Route AST完全一致、DedicatedController字节一致；Nav2 adapter仅归一导入与旧logger名后AST一致；
  canonical launch仅换包/share后AST一致，整个AgentAdapter和sim组装仅归一实际import/share位置后AST一致。
  四资产与旧share/4b8f31f字节一致，所有registry锁记录相同，uv仅增加本地导航包/根依赖。
- `uv lock --check --offline`、三个项目 `uv build ros2/src/senior_navigation --offline --out-dir /tmp/b7-wheels`、
  `uv build ros2/src/senior_robot --offline --out-dir /tmp/b7-wheels` 和根 `uv build --offline --out-dir /tmp/b7-wheels`：三个sdist/wheel均通过，导航sdist含全部CMake/接口声明/script/launch/四资产源。
- 仓库外 `/tmp/b7-sdk` 新建完整11个wheel/26包闭包，从 `/tmp` 运行
  `python <仓库>/tools/verify_batch_one_install.py` 和 `--navigation-only`：原完整检查及canonical/旧SDK同身份通过，导航ROS可执行/share、原native/SLAM均标UNVERIFIED；uv pip check通过。
  初次完整工具仍要求RoutePlan在Root，已更新为canonical归属/旧新同身份，其余原公共owner断言保留。
- 19份变更Python的ruff check/format --check、标准XML/安装源声明/脚本权限检查通过；原AgentAdapter/test_home保留CRLF，默认diff检查会将新增行CR报尾空白，使用原约定 `git -c core.whitespace=cr-at-eol diff --check` 通过。

本机没有colcon/cmake/rclpy/ament，真实 ROS包发现、ament安装/可执行/share、两个launch分支实际节点图、
Nav2/Humble private速度通道/定位断链/真进程恢复、厂商/设备与Linux/Python3.10及CI均未验，
原native/SLAM安装仍UNVERIFIED，静态/SDK/替身不替代这些实测。
原 `NAV2_TEST_INSTALLED_LAUNCH=1` 与 `SENIOR_LAUNCH_RECOVERY_LIVE=1` 探针未启用；
它们需要Linux、localhost-only和隔离空ROS域。原live夹具的临时MPPI1.1.9参数不代表生产评分验证。
没有启动开发、实车/仿真服务，没有部署或推送。


## 2026-11-26：图内 unknown 准入（仓库实现，未部署）

管理端导航目标现在独立校验真实当前栅格：仅准入有限栅格内的 free（0）和 unknown（-1），
occupied、图外、旧 G/MapRef、不可用或仅供显示的历史图仍拒绝；航点创建继续仅准入 free。
栅格身份 G 使用 state.map_grid_id，定位参考代次 R 使用 state.map_generation；定位匹配、
就绪、安全/手动权限及 Home adopted/confirmed/map 检查保持，缺 G/R 不得用另一身份替代。
导航仍经原持久任务接纳、effect 和 IPC 下传 R，不由管理端直接发送 Nav2 goal。

canonical nav2.yaml 的 planner_server / GridBased（SmacPlannerLattice）启用 allow_unknown=true。
navigation.launch.py 通过 parameters/RewrittenYaml 将该文件交 planner；模拟 planner 也使用该
canonical 默认文件，模拟 controller 使用其原 overlay。硬件显式 navigation_parameters 会交给
planner 和 DedicatedController，必须核对实际选中文件及安装态参数；此仓库修改不证明部署已消费。
两 costmap 的 track_unknown_space、原 footprint/padding、MPPI 及静态/live 装配未在本段修改。
目标点准入不证明整个足迹或路径安全，也不授权任意图外导航。

本段离线验证只覆盖管理 reader 和原 SQLite task→IPC 调用链（S10/S11 的管理部分）；IPC 接收端
为观察替身，未启动 ROS/Nav2。全量 Q 插件、真实 global/local 旧格及膨胀撤销、固定墙保留、同版
Smac/MPPI 的 unknown 通道和 unknown/lethal 混合足迹，以及插件安装态 S12–S14 均未验。
精确部署版/ABI 与实际物理 profile 证据尚缺；整个 unknown 导航功能交付继续阻塞，不能仅凭此
配置和准入通过宣布 Nav2 已安全交付。未实现自研 frontier、临时等待、通道 ban 或第二运动规则。

## 2026-11-26：全量 current Q 插件与原 owner 覆盖契约（待联验）

`senior_navigation::CurrentScanLayer` 是薄 Nav2 Layer；live global/local 顺序均为
StaticLayer→CurrentScanLayer→InflationLayer。它消费 map-frame 的全部当前 hit（最多2048），
不使用 UI 的96点预览、不作物体分类、预测、历史续占据或写第二张固定地图。local 保持 odom
滚动窗口，每周期将 owner 已补偿的 map 点转入本 costmap frame；窗口外点仍保留在完整 Q，
由 global 覆盖。global 同时检查实际 master 范围和全部 hit，未扩域时 current=false。

旧实际渲染范围与本周期新点范围共同进入 updateBounds；Nav2 原 LayeredCostmap 重置该范围后，
StaticLayer 重铺真实固定 O/F/U，Q 只写当次 lethal，InflationLayer 按原机制重算。Q 移走、
有效空、失格、过期、reset 或 disable 均不直接把 master 写 free；旧点及膨胀撤去后，固定墙
仍参与规划，原 unknown 不会因没有当前 hit 被证明 free。reset/deactivate/resize 保留一次清旧域
机会，没有私有 dense grid、额外永久线程或跨周期历史点表。

Q 不能给自身授权。原 startup 必须给两个 costmap 注入同一私有0700运行目录、run nonce、
真实 host boot 及各自 role permit；插件严格读取原 bridge 的 `reference.json` schema1，
检查原 selected 授权、MapRef/R/backend/full source、floor、期限，再检查 Q 的原整数首收时间、
单调同源 sequence、map frame 与 finite hit。授权换代、失格或绝对期限到期不能靠重复 Q、
文件 revision、bridge ACK 或 timer 续期。Q 使用可靠 KEEP_LAST=1/volatile；逐消息 DDS GID
还须匹配唯一 Q writer 与同 node 的唯一 `/map` writer，但节点名称/GID 本身不是选图授权。
该契约沿原同机同uid受信进程边界，不声称提供 DDS 消息签名或跨主机 steady 时钟协议。

插件把完成周期的实际 master geometry、处理的原 ScanKey/截止、清理及全量覆盖结果写到
`global-coverage.json` / `local-coverage.json`。健康新周期保留上一份同授权/domain且未到期的
完成回执；明确失格立即排负回执。回执期限不超过原 Q、授权 lease 和本周期开始后0.5秒。
文件读写由现有 costmap ROS executor 的独立 callback group 定时处理，单 writer 合并待写槽，
不持 master/插件状态锁作文件 I/O。原 LiveMappingReference 的 selected、scan 和两端 coverage
合取、许可换代及 static/live 生命周期交接由原 owner 独立实现，不能把本插件 current 位当作
该跨进程接线已经完成；回执也不冒充 stock StaticLayer 已装入某个内容 G 的证明。

canonical nav2.yaml 的 live 两端启用 Q，原 `/scan` ObstacleLayer 参数作为 inactive 配置保留，
供原 startup 生成静态 fallback overlay；live 插件列表不重复选择 `/scan`。本 child 不再修改
simulation_motion.launch.py，临时 overlay、planner/DedicatedController 同路径消费和静态
map_server/AMCL 交接归原 startup owner。缺运行目录/许可时插件拒绝配置，不能直接拿 canonical
live 文件替代尚未交接的静态部署。global update_frequency 从1 Hz调整至5 Hz，与 local一致，
配合原0.5秒 Q 期限；未加长期限，这不是 VM/实机时延或 CPU 达标承诺。footprint/padding、
速度、MPPI critic 参数、Smac allow_unknown 及原任务取消保护没有放宽；
允许 unknown 的 CostCritic 评分修复边界见下文。

已读取参考 VM Ubuntu22.04 ARM64 的 Humble安装头文件：costmap
1.1.20-1jammy.20260916.124031、MPPI .130539、Smac .131156；严格 JSON 依赖为
`nlohmann_json >=3.10.5`，该 VM 已安装 nlohmann-json3-dev 3.10.5-2。这不是车端部署版本证明。
VM 对冻结候选 `5d929e71a4afdaa6873d8d9a7399c4bfe27c5991a22febe61bc2ecff7ddcbffe`
已完成实际构建和 S12；S14 三种 cyclic 起点在原 core 分别为255/254/255，原库确实存在
mixed unknown 掩盖已知 lethal 的失败。经单文件 lethal-first 修复后，三序均由真实共享 checker、
Smac、MPPI collision 检查得到254/true/true，插件也会拒绝原不安全 core 的配置。
这是该参考 VM/该冻结候选的结果，不是本次新增评分测试已编译或车辆部署验收。

S12 C++ 使用真实 LayeredCostmap/StaticLayer/InflationLayer 和冻结授权文件 fixture，
DDS metadata 由测试 peer 提供，不冒充生产 catalog/IPC 或实际桥 ACK；原 owner 联验另行覆盖。
S13 安装态 synthetic-map probe 使用真实 Smac action、MPPI private Twist 及原 SE2 函数的
20 Hz理想离线跟随，有限 unknown 通道和已知 lethal 分隔重规划，不代表实车。
**原 S13 仍失败：Smac 规划 status4 后，MPPI 跟随以 status6/Failed to make progress 终止，
尚未到已知 lethal 分隔阶段。** 一次获批的外部单周期观测保留全部2000×56候选、8 critic 和
实际 master：753条中心触达 unknown 的候选均受到 CostCritic 正 critical 惩罚，其余1247条
该项增量为零；PathFollow 等四项可见增量为零。观测未覆盖 gamma/softmax/滤波选择，不能据此
认定唯一根因或称修复/性能 GREEN。诊断代码和其专用库不属于产品。

下述最小评分补丁与新增真实 score 断言尚未在 ROS 环境构建/执行；Bot1 无 ROS/colcon。
最终原 S13、S12/S14评分回归、完整授权/coverage/静态-live 生产接线、车端 ABI/安装和
四核 CPU/RSS/锁等待仍须各自验证；缺 ROS、源码、skip 或规划 status4 都不能当实际跟随通过。


### 同版共享 footprint checker 的固定补丁与运行时边界

依赖补丁是 [nav2_costmap_2d-1.1.20-lethal-first.patch](../../patches/nav2_costmap_2d-1.1.20-lethal-first.patch)，
仅修改官方 navigation2 `1.1.20` / commit
`a097086719c88f781aa59788eca29ac6ca5e56db` 的
`nav2_costmap_2d/src/footprint_collision_checker.cpp`。官方 codeload 该 commit 的 tar.gz
SHA256 为 `8b0fdd2aa41947d48d543ad24a6e5dc7fe885f1014a2bd43e5410985c5ff31af`；
原文件 SHA256 为
`dedc7441a1c6404ae99f17632d2f0770c08677a37cf80c04316441d9f2d70c59`，
补丁 SHA256 为 `d6bd06dfd4da553afd9d9f3046d3889b832fd3393e0aba97e8abd8f9cdbb6ecb`，
预期补后文件 SHA256 为
`3d220699b3e23c7dacdce8ab06957b201e4333c54e52106f67709b6f72e03104`。
循环中的每条原始 edge cost、最后 closing edge 的原始 cost 都优先判断 lethal，随后才保留
原 unknown 最大值语义；不改 lineCost、公共头/签名、足迹、padding 或 Smac/MPPI 配置。
S14 保留原两个物理输入/断言，并明确增加第三个相同几何的 cyclic 起点，使 lethal 位于
closing edge；这是三个实际输入，须先在旧库记录第三输入 RED，再在固定 overlay 验证 GREEN。
纯 unknown 可通行仍须原 S13 实际路径与控制输出联验；当前跟随尚未通过。

Layer 在配置时还会用独立80×80、.01m分辨率的小 costmap 与同一 mixed 足迹调用**实际链接的**
FootprintCollisionChecker，验证循环/closing 三种起点。该临时网格约6.4KiB，立即销毁，不改
真实 master、真实传感输入或机器人 footprint，不读版本字符串代替行为检查；旧不安全库
会明确拒绝配置，不能因漏装补丁仍发 ready。没有默认绕过开关。该能力检查只证明所调用
共享 checker 的这一语义，不代替 S14 的真实 Smac/MPPI 调用和各进程实际动态库路径证据。

构建只在任务私有 prefix 使用该固定源的原 `nav2_costmap_2d` 包；先核 source archive/commit、
上述原文件 hash、实际 underlay 公共头与编译器/依赖版本，再零 fuzz 应用补丁并复核补后 hash。
不把整个 Nav2 源码放进产品仓库，不覆盖 `/opt/ros/humble`、系统 `.so` 或只遮蔽一个头文件。
原 cpp 的两个显式模板实例位于 `libnav2_costmap_2d_core.so`，必须实际重建该库。
示例中各目录均由验证 owner 选择的绝对私有路径，命令不是已执行证据：

```bash
source /opt/ros/humble/setup.bash
patch --directory "$T5_NAV2_SOURCE" --strip=1 --fuzz=0 < "$T5_PATCH"
colcon build --base-paths "$T5_NAV2_SOURCE/nav2_costmap_2d" \
  --build-base "$T5_OVERLAY/build" --install-base "$T5_OVERLAY/install" \
  --packages-select nav2_costmap_2d \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source "$T5_OVERLAY/install/setup.bash"
# 在上述环境重新构建并 source 当前项目的安装 prefix，不能先后混入旧 core。
ros2 pkg prefix nav2_costmap_2d
```

保留 patch 后 core 的 ELF hash/build ID、ament package/plugin XML、prefix 环境、readelf/ldd；
在实际 S14 test 与后续 planner/controller 运行时读取 `/proc/<pid>/maps`，确认所有映射的
`libnav2_costmap_2d_core.so` 均来自该固定 overlay，且 hash 一致，没有同时映射原 core。
ldd 只是辅助，不能代替运行进程 maps；S14 未绿时仍不运行 S13。
部署入口复用现有显式 `profile.ros_setup`：原 platform_runtime 的 adapter/motion 两命令
均先 source vendor setup 再 source 该项目 setup，DedicatedController 继承相同环境。
原部署 owner 可选择经过 prefix/hash 拒错的 composed setup，不新增 profile 字段或运动来源分支；
本 child 不修改实际车辆 profile 或 parent 启动代码。未取得实际安装与 S12–S14 GREEN 前，
上述补丁、能力检查和构建指令都不是 unknown 功能已通过或已部署的声明。


### 同版 CostCritic 对允许 unknown 的最小评分修复

依赖补丁是 [nav2_mppi_controller-1.1.20-unknown-cost.patch](../../patches/nav2_mppi_controller-1.1.20-unknown-cost.patch)，
同一官方 `a097086719c88f781aa59788eca29ac6ca5e56db` / Humble1.1.20，只改
`nav2_mppi_controller/src/critics/cost_critic.cpp`。原文件 SHA256：
`0630dc3f8302d146d7f5317ee2c3fc940cc6f6ad63f26b19bf0aa630accc7fa4`；
补丁 SHA256：`fa2fad32ddb1f38650dd571e4ec58c8b1044ec6c034b14943785ce7f671d05ab`；
补后文件 SHA256：`600c88698e3d6eb32b6cb638910f47293db0b16a10b2d2243217ec8cbcb73fa4`。
使用前节同一官方归档/hash，原公共头、API、critic 参数与算法其余部分均不变。

原评分先用真实 `inCollision` 判断，再把中心 cost>=253 都加 critical300，导致既有策略
已允许的 unknown255 也被当作已知近障碍。补丁只在原碰撞检查已通过后，对 `worldToMap`
成功、实际中心255且原 `isTrackingUnknown()` 为真的点跳过该项 repulsion。混合足迹中的
已知 lethal 仍先触发原 collision_cost/fail；known253的 critical300、其它 known 成本、
图外255和原不允许 unknown 的处理原样保留。没有把 U 写成 F，没有新增地图 writer、
source/资格判断、等待逻辑或永久线程；评分仍为 O(候选数×步数)，unknown分支多一次有界格索引。
这里选择的是允许 unknown 的中性评分，不证明 unknown 物理为空，也不改原 free-only 航点。

该补丁必须与前节共享 checker 的 lethal-first 修复共同验证。只改评分、漏装旧 core 的
known-lethal 修复不可验收。Layer 的配置能力检查只检测共享 footprint 语义，不检测
CostCritic 评分是否正确安装；不能拿 current/ready 或版本号代替真实 score/S13 与实际库身份。

S14 明确为五个实际语义输入：原三种 mixed 足迹起点保留 checker/Smac/MPPI collision
断言，并追加实际 `CostCritic::score` 的 collision_cost/fail 断言；另两个输入分别是图内纯
unknown（原评分期望 RED=4.5、修后期望0）和无 lethal 的 known253（原/修后都应保留4.5）。
一候选一步的数值独立由固定参数3.81/254、critical300、collision1e6得出，混合碰撞应15000。
测试通过同版真实 `ParametersHandler` 和 `CriticFunction::on_configure` 初始化，不手设 critic
内部状态，不复制碰撞/评分算法；无 inflation 的小型语义 fixture 会按原库发优化提示，
使用原 full-footprint 路径。它验证实际共享库语义，不能冒充完整 MPPI optimizer 或闭环运动。
测试翻译单元在所有头文件之前对齐实际 MPPI 编译行的 `XTENSOR_ENABLE_XSIMD` /
`XTENSOR_USE_XSIMD`；同 VM 的 xtensor0.23.10 会按后者选择公开张量的 allocator 类型，
见[官方固定版配置](https://github.com/xtensor-stack/xtensor/blob/0.23.10/include/xtensor/xtensor_config.hpp)。
这只影响既有测试 CPP，没有修改公共头或 build 路径；实际安装头与测试/critic 的类型、
对齐和链接兼容性仍须在原生构建中核验，不能用本机文本核对代替 ABI 证据。

后续原生验证使用**新私有 prefix**，从官方 pristine MPPI package 加本评分补丁，绝不把
临时观测 prefix 或被改过的 `critic_manager.cpp` 带进产品。其 pristine 文件 SHA256 必须是
`339587bccec06962453893eba577aa3ad9b65aa804ea12999dec5be8f73fce3b`。
先在原 uninstrumented MPPI + 已修 core 上运行同一评分测试，明确纯 unknown score RED；
再仅换评分补丁验证同输入 GREEN，并运行原 S13 与 S12。测试、map、goal、随机/控制参数、
原足迹/速度/任务取消/资格和 assertion 不变；原 S13 仍失败则如实报告，不能自行调参或重试。
示例是供原生验证 owner 审阅的命令形状，Bot1 未执行：

```bash
source /opt/ros/humble/setup.bash
source "$T5_COSTMAP_OVERLAY/install/setup.bash"
# T5_MPPI_SOURCE 必须是已核官方归档的新 MPPI 源，critic_manager.cpp 为上述 pristine hash。
patch --directory "$T5_MPPI_SOURCE" --strip=1 --fuzz=0 < "$T5_MPPI_PATCH"
colcon build --base-paths "$T5_MPPI_SOURCE/nav2_mppi_controller" \
  --build-base "$T5_MPPI_OVERLAY/build" --install-base "$T5_MPPI_OVERLAY/install" \
  --packages-select nav2_mppi_controller \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
source "$T5_MPPI_OVERLAY/install/setup.bash"
# 在该环境重新构建/source 本项目的新候选；原产品验证 owner 保存完整命令及环境。
```

保留 cost_critic baseline/after/patch、实际 controller/critics/core `.so` 的 ELF SHA/build ID、
原安装头比对、consumer参数文件及测试/planner/controller实际 `/proc/<pid>/maps`，不能仅用
ldd、源码或包版本签通过。原核心 `.so`、系统安装和车端 profile 均不在本 child 修改范围。
本次新评分补丁、两个评分对照和追加断言的 ROS 编译/加载/RED/GREEN/S13 结果仍未取得，
也不构成车辆安装、ABI 或 VM 模拟树莓派性能验收。

### Task7 生产链验证入口（2026-11-26，尚待 VM 执行）

`ros2/tests/shared_layers_native_fixture.py` 复用 simulation_motion 的原 action factory，
以真实 LaunchService/Event、原 planner/localization ExecuteProcess、agent_adapter /
DedicatedController 和真实 RSP 组合。`test_shared_layers_process.py` 用普通 raw/odom/IMU
经原资格与共用运动链，覆盖 static→live、双 coverage/state.get ready、S05/S07/S09 和活动
route 正常退出的 R4 PID 结算断言；无 AST 抽类/fake event/Process/killpg 或自签许可。
`motion_stack_actions` 仅拆原 actions 供复用，原 `_simulation_nodes` 默认动作执行顺序、
MPPI overlay（400、.22/-.15/.4）及状态机保留；实际参数由真实 consumer 服务与 argv 记录。

完整命令、十包构建/依赖顺序、冻结输入与 schema 见
[SLAM 操作文档](../../../docs/operations/slam-backend.md)。
`tools/occupancy_replay_benchmark.py --system-trace PATH --output PATH` 是新增实际解析入口；
原 `--baseline` 纯算法模式仍保留。完整 capture key 配对修复了原 current[-1] 对另一帧 slot
的 deadline 比较，global/local 可各用同源的不同仍有效 sequence，不会续原500ms截止。

Bot1 只有 portable/helper/语法检查，不能据此声称真实 Launch/Nav2/整链/性能已通过。
PublishFiles 与 capture RPC 日志不能证明 native 慢 SaveToFile 已重叠；global 插件内部重建
仍需 new startup run，harness 不代发 permit。冻结零 odom 对照保留原 route 的失败，不调
progress checker 求通过；UI、物理几何、实车、原始历史对定位/loop影响及完整 S16 待后续
准确集成版本验证。异常/未知结算保留 run 证据，不能当垃圾清除。


Task7八项审核返修后，`test_frozen_stationary_system_reference`明确只作stationary
sensor/SLAM参考，零odom不满足原progress .2m/10s，终态后的MPPI指标N/A，不能签持续
60s控制或完整S16。单execution由原task_status按原epoch观察，accepted不作active；
私有controller输出和最终cmd_vel只作新鲜running/ready窗口关联。取消须原effect查询、
输出退出与fenced-through确认，未确认留trace。pump持续只读slot/双coverage，完整key
各回原Q/TF/slot；trace envelope/contract schema独立且payload完整保留。summary分开
bootstrap/主stationary窗/save/restore，缺角色/配对明确N/A；资源阶段跨界CPU不误归属，
cgroup逐字段保留最近值/时刻。实际Humble/Launch、PID现场和T6 UI仍未验。
