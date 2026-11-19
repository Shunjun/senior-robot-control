# 运动估计功能包

B3 以 `155e2b9` 为基线，迁移既有轮速/雷达运动/IMU 适配、时序准入、融合及连续位姿估计。
只搬既有行为；现有可选 GPS 代码和默认配置原样保留，不启新 GPS、算法、控制或导航功能。
实际 49 个逻辑文件（迁移单计）：产品 28、测试/辅助 11、配置构建 8、工具文档 2。

## 实际目录、源与状态归属

```text
senior_motion_estimation/
  CMakeLists.txt, package.xml, pyproject.toml
  senior_motion_estimation/
    __init__.py, py.typed
    observations.py                  # 原测量/来源/坐标/协方差值与校验
    sample_types.py                  # 原点观测/差分观测及原始物理时间支持
    admission_events.py              # 原有界准入事件与前沿快照
    source_admission.py              # 原来源/代次/回退/恢复 owner
    temporal.py                      # 原时间窗口与支持对齐
    fusion.py                        # 原分量融合与可用性/不确定性
    estimator.py                     # 原当前运动与连续路径 owner
    pose_estimator.py                # 原连续位姿/误差与历史 owner
    motion_observation_adapter.py    # 原真实 ROS 轮速/雷达运动/IMU 接入桥
    motion_sensor_runtime.py         # 原估计组装 + 新公开只读决策快照接口
  test/
    test_motion_sources.py, test_motion_temporal.py
    test_motion_fusion.py, test_motion_fusion_replay.py
    test_unified_estimator.py, test_pose_space_estimation.py
ros2/tests/
  test_native_motion_observation_adapter.py
  test_control_process_resources.py
  test_laser_reference_publication.py
```

八份算法/值模块原位于 `src/senior_control/motion/` 同名文件，两份 adapter/runtime 原位于
`senior_robot/senior_robot/` 同名文件。旧路径显式 Compatible 转发同一类/函数，保持值类型身份。
数值融合、姿态/路径、时钟/来源、恢复及准入算法未重写；支持字段与缺失值/误差语义沿用原实现。
source_modes、kinematics、时间策略、公共值/诊断等继续复用既有公开接口，不为它们再建顶层项目。

## 公开输入输出与生产接线

| 接口/owner | 原输入与输出 | 时间、坐标、来源与状态 |
|---|---|---|
| NativeObservationRosBridge / NativeObservations | 原轮速 Odometry、ScanMotion、Imu → 已准入点/差分观测与原报告 | 原始测量/接收 ns、DDS GID/producer epoch、frame/TF 和实际协方差；无 IMU 时不捏造 IMU/可用轴 |
| AdmissionOwner / TemporalResolver / NumericalFusion | 原样本/中断事件 → 原来源窗口与融合分量 | owner 内保留代次、退役集合、序号/时间前沿、恢复与有界历史，不用快照刷新它们 |
| MotionEstimation / PoseEstimation | 原已知/未知误差观测 → 当前值、可用轴、不确定性、连续路径/位姿 | 连续历史与估计状态只在接收 owner 内推进，算法/期限沿用原规则 |
| MotionDecisionSnapshot | latest（按原规则去掉 path）、independent、raw_wheel、raw_imu | frozen 固定四字段；成员沿用不可变值/tuple，保留原时间、roots、代次、轴与误差；不包含 profile、诊断、可写 estimator 或位姿历史 |
| decision_snapshot / apply_decision_snapshot | owner 导出决策视图；控制 reader 应用该视图与原报告 reference | 不重新 ingest/estimate、无新时间戳和代次；IPC 仍由 LatestSnapshot 原 8 MiB 单槽限额约束 |

原生产进程边界不变：ControlInputs 用 spawn 启动 SafetyGate 的 input_channel 模式，
该子进程接收/估计后仍传 `(决策视图, 最新一帧扫描, ready, scan_generation, 原精简 reference)`。
父进程通过运行时的 apply_decision_snapshot 应用字段，不再写 runtime.__dict__。
原报告 reference 的 average/mode 精简与时间不变，tracking/历史仍留原 owner；没有引入全状态复制。
ControlInputs 的私有单槽、参数、传感器 remap、父死亡/有界关停及恢复行为保留。

实车/仿真原可执行入口和 launch 仍由 senior_robot 组装。
MotionFusion、SafetyGate 接收模式及 MotionCore 直接使用功能包类；原 senior_scan_matcher 与
senior_sensor_ingress 接收/安装路径、话题、消息、TF 和建图/导航 launch 均保持。
Bridge 直接使用接入包的 native_receipt；原 with_dds_info 标记函数迁到该既有接口，
motion_lifecycle 标注 Compatible 重导出同一函数，executor/信号/关闭实现未迁移或复制。
本功能包不反向导入 senior_robot，保持技术接口单一实现，不形成包循环。

## 构建、安装与测试落点

ament_cmake_python 安装功能包；ament_cmake_pytest 的 estimation_algorithms 目标收集包内原用例。
senior_robot 声明本包运行依赖。根 pyproject/uv.lock、云 Docker COPY 与 CI wheel 列表补齐该已有
ROS 功能包的 SDK 安装闭包；不更换根项目运行入口。默认 SDK 导入不加载 ROS，runtime/ROS 部分仍需原 safety/ROS 依赖。

六份纯算法原用例迁到包内 test/，三份原接入/进程/激光回归迁到 ros2/tests。
原 control_process_test_support 仍供未迁控制用例使用，本片按真实生产调用改为只读四字段视图、
最新一帧和原精简 reference，移除对已不存在 ProcessWorker 的补丁，不添假属性。
现有真实 spawn 用例保留几何进程与私有单槽断言，并用单轮/缺 IMU 输入核对重复应用不刷新
computed_steady、generation、原始 SampleTime，且原 1 秒截止到期仍不可用。
原拒收诊断用例仅更新 canonical 模块的真实补丁位置，未弱化断言。

## B3 验证记录与限制

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、Pydantic 2.14.1、
NumPy 2.5.4、Shapely 2.2.0、uv 0.12.23、ruff 0.17.3；使用原主仓库 .venv/bin/python。

- RED：先迁原 test_unified_estimator 的导入，在功能包尚未实现时得到 ModuleNotFoundError。
- `python -m pytest ros2/src/senior_motion_estimation/test ros2/tests/test_native_motion_observation_adapter.py ros2/tests/test_control_process_resources.py tests/unit/test_estimation_review_contracts.py::test_rejected_native_receipt_uses_original_identity_without_live_state_mutation tests/unit/test_motion_cli_lifecycle.py tests/unit/test_motion_launch_profile.py tests/unit/test_sim_contract.py ros2/tests/test_native_motion_executor.py -q`：127 passed、2 skipped；通过项包含原时序/来源、差分/单轮/无 IMU、误差、真实 spawn 与公开快照、原 executor 关闭及入口契约；两个实际 ROS 入口 DDS 用例因无 rclpy 跳过。
- 本片主验证中，原估计 review 文件只执行上列真实拒收身份用例；其他旧控制回放组合未宣称通过。
- 激光发布全文件与进程用例一起执行：12 passed、2 failed；两个旧用例仍调用已退役的 MotionCoreOutput._inputs。完整 155e2b9 archive 在 /tmp/b3-baseline，仅去掉两处已退役 ProcessWorker 补丁后，单跑原激光文件复现 4 passed、同名 2 failed。原始 archive 的该夹具早已在 B1/B2 记录 setup 错误，本片不补假 _inputs 或弱化其断言。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline`：解析 64 包，仅加 senior-motion-estimation；离线 lock --check 核对安装闭包。
- 同缓存 `uv build ros2/src/senior_motion_estimation --offline --out-dir /tmp/b3-wheels` 与根 uv build：sdist/wheel 构建通过；用 uv pip 向 /tmp/vehicle-batch-one-venv 安装两个 wheel，从 /tmp 实际导入 estimator/pose/Origin/Source 与旧入口身份相同，未加载 ROS；uv pip check 的 14 个已安装包依赖兼容。该验证不代表安装了/运行了 ROS 节点。
- 定向 ruff check/format、git diff --check，算法迁移/兼容身份和实际原生/launch 保持核对通过；没有新增 GPS、补偿、守卫或控制算法。

未验证：本机无 colcon/CMake/rclpy/ament，实际 ROS 构建/安装、DDS/TF、接收子进程、
实车/仿真及建图/导航运行均未执行；缺 Linux 目标的验证也不冒充通过。
实际接收子进程不是普通 spawn 单元测试可替代，两个旧激光组合用例的 retired 控制夹具另待所属批次处理。
ROS Humble 环境需安装原根 safety/ROS 依赖后 colcon build/test 本功能包及 senior_robot，
再执行保留的入口/扫描/恢复原回归。没有启动开发/实车服务、部署或推送。
