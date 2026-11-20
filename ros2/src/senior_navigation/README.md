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
