# Standard ROS Time Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完整撤销新增时间协议，让全部车端链路直接消费标准 ROS 消息的 `header.stamp`，保留有效算法、任务生命周期及单调时间保护。

**Architecture:** 协调者先完成公共时间模块、内部消息及共享夹具契约，在同一冻结提交上创建四个隔离 Orca worktree 并行迁移。原始接入只映射一次时间；派生结果保留原始测量端点和接收记录，控制期限使用本地单调时间，唯一原生输出传输发布标准 `/cmd_vel`。生产代码没有仿真时间域、倍率补偿或真值许可；环境设施只提供标准输入、配置与生命周期。

**Tech Stack:** Python >=3.10、C++17、ROS 2 Humble、rclcpp/rclpy、rosidl/ament/colcon、pytest、现有 Nav2 与 ros_gz/Gazebo/ros2_control 标准组件、Orca 隔离 worktree。

**Spec:** `docs/superpowers/specs/2026-11-17-standard-ros-time-design.md`，评审基线 `d32936c`；实际启动/导出修复 `3ceadf8` 必须保留。

## Global Constraints

- 用户已授权按方案实施及 Orca 协作；执行方式不再等待二次确认。本次计划编写本身只编辑此文件，不实施产品改动。
- 集成分支为 `fix/ros-typesupport-export-order`；不整批回滚 `dc9db96`，不使用整文件 ours/theirs 掩盖冲突，不自动合并 PR 或推送 main，完成后本地切回 `main`，保留未跟踪成果。
- 厂家驱动不修改、不另写替代版；不保留旧协议解析、wrapper、桥接回退或第三方时间补丁。
- `sample_steady = receipt_steady - (receipt_ros - header_stamp)`；数据对齐、差分用原始 ROS 纳秒，TTL、计算预算、队列等待用单调时间；保留 0.3 秒软退化/1 秒硬失效。
- 生产控制不识别仿真、不推算倍率、不消费 `sim_step`、真值许可或引擎 reset 证明。倍率、世界管理、可控测试时钟只在环境/测试设施。
- 网页仍能连接 1 倍仿真的通用 Agent/车端接口；例行仿真测试只测车端能力，不新增网页操作仿真测试，完整仿真仅少量必要的 1 倍场景。
- 车端仅 Git 拉代码、构建、项目 CLI 启停和读取日志/状态；不 SSH 手改源码、不发送调试运动指令、不清除急停、不放宽许可。实现阶段不擅自启动开发服务。
- 无新增全车资格门槛、不要求精确硬件采集证明；独立速度不得使用命令目标或轮速伪造。
- 用户 AGENTS.md 继续生效：兼容代码须标记 `//Compatible`，但本迁移不新增旧时间协议兼容；前端 props 和样式规则不变，本计划无前端改动。

## Review Focus

以下是确定验证契约与现有有效回归的迁移核对项；优先复用原位置，不创建重复测试，不要求产品实现前新建 TDD 测试。

1. 队列积压及重收样本仍保留旧测量年龄：Task 1/3/4 复用 `test_sensor_timing.py`、`test_motion_reference_backlog.py`、原生 ledger 测试。
2. 原始 stamp 的纳秒边界、乱序、来源换代不被浮点转换或本地新序号掩盖：Task 3/4 复用 `test_reference_nanosecond_boundary.py`、`test_reference_clock_frontier.py`、`test_native_motion_observation_adapter.py`。
3. RGB-D 四路原始端点、静态 TF、派生候选经过 worker 后仍指向原始观测：Task 3/4 复用 `test_rgbd_message_contract.py`、`test_rgbd_candidate_codec.py`、`test_rgbd_parent_ledger.py` 和原生 egress 测试。
4. 最终输出排队、EOF、过期、急停及导航取消/迟到结果仍撤销旧目标：Task 5 复用 `test_native_output_protocol.py`、`test_navigation_completion_handoff.py`、任务生命周期回归。
5. 删除 overlay 后不靠安装残留或仿真特许“通过”：Task 6/8 复用 installed-header consumer、`test_common_sensor_composition.py`、部署/CLI 回归及真实干净构建。

---

## 文件结构与单一所有权

执行 DAG：`Task 1 公共契约 → Task 2 冻结/分叉 → {Task 3 A, Task 4 B, Task 5 C, Task 6 D} 并行 → Task 7 顺序合并清理 → Task 8 构建/部署/交付`。

| Owner | 独占文件范围 | 明确排除 |
| --- | --- | --- |
| F：协调者，Task 1/2/7/8 | 新建 `src/senior_control/sensors/sample_time.py`、`ros2/src/senior_motion_msgs/include/senior_motion_msgs/sample_time.hpp`；所有 `ros2/src/*/CMakeLists.txt`、`package.xml`、`senior_robot/setup.py`；所有 ROS `.msg`；`sensor_timing.hpp`；退役公共时间文件；`senior_motion_msgs/test/installed_header_consumer/*`；全部共享 support、conftest、fixtures；最终集成文档和验收 | 并行阶段不得修改 A/B/C/D 的实现文件 |
| A：Task 3，C++ 原始接入/扫描/SLAM | `senior_motion_msgs/src/native_receipt.cpp`；`senior_sensor_ingress/include/`、`src/`、`test/`；`senior_scan_matcher/src/`；`senior_slam_bridge/src/`、`scripts/`、`test/` | 不改消息、CMake、package.xml、公共头、共享 fixtures |
| B：Task 4，Python 估计/感知 | `src/senior_control/motion/`（除 `navigation_clock.py`）、`safety/self_learning.py`、`localization/gps_course.py`、`vision/`（除环境 `simulation_video.py`）；`senior_robot` 下 `native_receipt.py`、`control_inputs.py`、`motion_observation_adapter.py`、`optional_motion_inputs.py`、`motion_sensor_runtime.py`、`motion_core_output.py`、`motion_core_perception.py`、`safety_node.py`、`motion_profile.py`、`motion_fusion.py`、`scan_adapter.py`、`safety_scan_ingress.py`、`raw_safety_observation.py`、`perception_node.py`、`depth_ingress.py`、`semantic_ingress.py`、`rgbd_*.py`；`tools/motion_fusion_replay.py`、`motion_fusion_synthetic.py`、`motion_replay_records.py` | 不改 `sample_time.py`、共享消息/夹具；`safety_node.py` 的输出默认值也由 B 改，C 不触碰该文件 |
| C：Task 5，导航/定位/最终输出 | `senior_motion_msgs/src/motion_output_guard.cpp`；`senior_robot` 下 `motion_output.py`、`command_node.py`、`dedicated_controller.py`、`navigation_adapter.py`、`hardware_reference.py`、`localization_node.py`、`slam_adapter.py`、`agent_adapter.py`、删除 `navigation_overlay.py`；`launch/navigation.launch.py`、`launch/safety_motion.launch.py`；`src/senior_control/safety/arbiter.py`、`motion/navigation_clock.py`、`hardware_navigation.py` | 不改 `safety_node.py`、`motion_core_output.py`、native receipt 或环境 launch |
| D：Task 6，环境/配置/工具 | `ros2/src/senior_sim/` 除共享构建文件；`senior_robot/launch/` 除 C 的两个 launch；`senior_robot/config/`；删除/搬迁 `simulation_reference.py`、`simulation_motion_profile.py`、环境 `simulation_camera.py`；`src/senior_control/real_runtime.py`、`vehicle_deploy.py`、`simulation_cli.py`、`cli.py`、`mapping/simulation.py`、`vision/simulation_video.py`；`config/`、`tools/` 除 B 的三个 motion replay/synthetic 文件 | 不改 B 的 profile 加载器；使用 Task 1 冻结的通用 profile 文件契约，不在环境 launch 中新增控制实现 |

路径中的 `senior_robot` 均指 `ros2/src/senior_robot/senior_robot`，其 `launch`、`config` 指包内同名目录。未列出的生产引用在 Task 1 的全仓搜索中归入明确 owner，并写入同一计划后才分叉；执行中发现新增引用，由协调者分配整文件所有权，不做跨分支小块并改。

**测试/文档所有权优先级：** F 独占 `tests/support/`、所有 `*test_support.py`、`tests/fixtures/`、`tests/**/conftest.py`、`tests/integration/agent_fakes.py` 以及 `tests/ros/motion_*.py` 共享设施；A 独占包内 C++/SLAM 测试及 `tests/unit/test_shared_sensor_cpp.py`、`test_laser_reference_publication.py`、`test_scan_matcher_replay.py`，并拥有 `tests/ros/test_scan_*.py`、`test_receipt_process_shutdown.py`；C 独占 `tests/unit/test_navigation*.py`、`test_direct_navigation.py`、`test_native_output_protocol.py`、`test_motion_ros_twist_output.py`、`test_motion_core_exit_deadline.py`、`test_terminal_native_channel.py`，以及 `tests/ros/test_navigation.py`、`test_relocalization.py`、`test_mapping.py`、`test_agent_adapter_shutdown.py`、`test_final_command_dds.py`、`tests/integration/test_manual_wire.py`、`test_task_*.py`；D 独占 `tests/unit/test_simulation*.py`、`test_sim_contract.py`、`test_common_sensor_composition.py`、`test_motion_launch_profile.py`、`test_motion_cli_lifecycle.py`、`test_vehicle_motion_*.py`，以及 `tests/ros/test_sim_topics.py`、`test_rgbd_diagnostic_launch.py`、`tests/integration/test_vehicle_*.py`、`test_deployment_config.py`；B 拥有其余本迁移涉及的估计、感知、学习、时钟、运行隔离单元/集成/ROS 测试（包括 `test_motion_fusion_replay.py`、`test_control_clock_domains.py` 和纳秒/参考系列）。此优先级先于按文件名前缀推断职责，F 在 Task 1 调整公共 fixture 所需的测试调用后将测试文件交还相应 owner，分叉后只由该 owner 写入。

F 拥有此设计/计划的状态更新及 `docs/testing/` 最终证据；D 拥有 `docs/operations/`、`docs/training/`、README 中的部署/环境/工具操作说明；历史 `docs/reviews/` 和其他旧 `docs/superpowers/` 状态标记由 F 统一处理，不能删除历史证据。

## 分叉前冻结的公共接口

### 1. 通用时间数据：仅项目内部记录

新模块 `src/senior_control/sensors/sample_time.py` 与 C++ `sample_time.hpp` 的字段语义一致；C++ namespace 为 `senior_motion_msgs::time`。不得从旧 proof 转换出该类型，不提供旧类型别名或 optional fallback。

```python
@dataclass(frozen=True)
class SampleTime:
    stamp_ns: int
    received_ros_ns: int
    received_steady_ns: int
    sample_steady_ns: int
    clock_generation: int

def map_sample(*, stamp_ns: int, received_ros_ns: int,
               received_steady_ns: int, clock_generation: int) -> SampleTime: ...

@dataclass(frozen=True)
class StampInterval:
    start_ns: int
    end_ns: int

def interval_ns(first: SampleTime, last: SampleTime) -> StampInterval: ...

@dataclass(frozen=True)
class ClockObservation:
    generation: int
    discontinuity: bool

class ClockTracker:
    def observe(self, *, ros_ns: int, steady_ns: int) -> ClockObservation: ...
```

C++ 同名 `SampleTime`/`StampInterval`/`ClockObservation` struct 使用 `int64_t stamp_ns, received_ros_ns`，其余时间/代次为 `uint64_t`；提供 `SampleTime map_sample(int64_t stamp_ns, int64_t received_ros_ns, uint64_t received_steady_ns, uint64_t clock_generation)`、`StampInterval interval_ns(const SampleTime &, const SampleTime &)`、`ClockObservation ClockTracker::observe(int64_t ros_ns, uint64_t steady_ns)`，以及 `uint64_t monotonic_ns()`。Python 错误用 `ValueError`，C++ 用 `std::invalid_argument`，调用者保留原因并撤销受影响窗口。

- 全部字段必填；`sample_steady_ns = received_steady_ns - (received_ros_ns - stamp_ns)`，用整数计算，禁止浮点 epoch 相减或把 receipt 替代 stamp。检查整数范围、负 ROS 坐标、未来测量及映射到负 steady 的输入；zero ROS stamp 合法，不新增启动等待门槛。
- `StampInterval` 只表示原始 ROS 纳秒端点且 `end_ns >= start_ns`，秒数只在算法计算差分后转换；点样本的 interval 两端相等。`interval_ns` 用于已经被来源/窗口 owner 接纳的端点；同一原始源跨 generation 的两端拒绝。不同入口的 generation 是局部代次，不能直接当成全局时钟证明比较相等。
- `ClockTracker` 初始 generation=1；ROS 或 steady 倒退撤销当前 owner 的窗口并递增 generation。没有倍率推断、引擎事件或同值 reset 识别；正向时间戳异常仍由既有年龄/区间/来源检查拒绝，不能擅自加全车冷启动等待。收到局部换代时窗口 owner 清空依赖该源的历史、候选和去重状态；自身时钟倒退时清空自身全部受影响窗口与已排队正向证据。
- 重复定义锁定为同一实际 publisher GID、接入生命周期/clock_generation、相同原始 stamp；去重在首次映射前执行。乱序保持拒绝/撤销原因，不通过给接收样本重新编号使其看起来更新。缓存只需保留现有有界窗口及最高已接纳 stamp；不建立无界 stamp 历史。
- 原始来源身份继续由真实 MessageInfo GID、接入进程 boot id、来源 generation、接入序号表达；这些是接收侧身份，不声称厂家提供测量序号或启动证明。`source_epoch` 根据这些接收身份产生稳定有界标识，与时间类型分开。
- 删除 `MeasurementTime`、`MeasurementInterval`、`SourceTime`、`SourceFrontier` 及 `measurement_time/measurement_interval/source_time` 解析器；`StampInterval` 不能再承载 host/sim 两种可选坐标。观测 `PointObservation.time: SampleTime`、`DeltaObservation.first_time/last_time: SampleTime`、归一化 `Observation.first_time/last_time: SampleTime` 全部强制提供，删掉旧 `measurement*` 可选字段和用 steady 伪造测量区间的路径。保持 source/frame/epoch/sequence/quality/covariance/model/几何字段及原有效算法。
- `Origin.time: SampleTime`、`Heading.time: SampleTime`；估计结果的对齐目标使用 `stamp_ns: int`，积分/运动窗口使用 `StampInterval`，有效期使用内部单调 ns 或现有明确秒单位的 deadline。允许在已有 TTL/控制器数值 API 边界做明确 ns→秒换算，不能保留旧 proof/measurement wrapper。

### 2. 有界消息与原生接收 IPC

`RawCaptureHeader.msg` 保留 `stamp`、`received_ros`、frame/GID/source_epoch/bridge_boot_id/capture_generation/sequence/valid/reason；删除 `SourceTime timing` 和 `float64 received_steady`，改成必填 `uint64 received_steady_ns`、`uint64 sample_steady_ns`、`uint64 clock_generation`。stamp/received_ros 仍为 `builtin_interfaces/Time`，反序列化统一转成整数 ns。其他包含 RawCaptureHeader 的消息自动携带同一记录，不能在 ledger/worker/UI 重新映射。

Python `vision/raw_capture.py` 的 `RawCaptureHeader` 使用 `time: SampleTime` 加现有身份/有效性字段，`capture_from_message(message) -> RawCaptureHeader` 只解码新字段。`BoundedStaticTf` 保留其原有首收/接纳生命周期；静态 TF stamp 是配置坐标，不作为必须与每帧相等的动态测量时间，也不靠重发 TF 延长图像 TTL。

`ScanMotion.first_capture/second_capture` 保留；interval_start 和 header.stamp 必须分别等于原始两个端点，不能拿 scan matcher 发布时刻当端点。RGB-D 的八个原始 capture、TF、标定、身份与 egress fencing 保留，`time_basis` 和“驱动证明”检查全部删除。

原生接收子进程仍使用私有继承 socket，无外部新话题。冻结帧为 SDR2，网络字节序，字段顺序：magic[4]、kind:u8、input:u8、generation:u64、sequence:u64、callback_steady_ns:u64、rmw_source_timestamp:i64、rmw_received_timestamp:i64、gid[24]、time_mode:u8、stamp_ns:i64、received_ros_ns:i64、original_received_steady_ns:u64、sample_steady_ns:u64、clock_generation:u64、payload_size:u32，随后原有 CDR payload；Python struct 为 `!4sBBQQQqq24sBqqQQQI`。

`time_mode=0` 无测量记录（心跳、撤销、无 header 的状态），`1` 首次原始接入映射，`2` 派生消息直接保留已有原始 capture；不支持 SDR1，也不将 RMW transport source_timestamp 当作 header.stamp。派生 `ScanMotion` 以 second_capture 提供 endpoint time，完整两个端点仍在 payload。原始时间在 native callback 序列化前记录；该子进程须从父节点显式接收标准 `use_sim_time` ROS 参数以使用正确的当前 ROS 时钟，不能自行检测环境。

B 的 `native_receipt.py` 冻结 `received_sample(info: dict[str, Any]) -> SampleTime`，只返回已解析的新 metadata，缺失即报错；保留 `received_steady(info) -> float` 表示本次 native callback/IPC 运输年龄，无普通 Python receipt 回退。`NativeReceiptGroup` 将 `info['sample_time']` 交给原始消费者；已携带 capture 的派生消息不二次映射。A 实现 SDR2 发送，B 实现接收；C 的定位接入消费同一 `received_sample`。

### 3. 标准命令与环境交界

- Nav2 控制器使用安装版本原有 `geometry_msgs/Twist`，每次运行仍 remap 到 `/control/navigation/run_<32hex>/cmd_vel`；`NavigationSample` 和自定义 `navigation_run_id` 参数退役。`CommandNode._on_navigation_sample(topic, msg: Twist, info)` 在回调第一步记录本地单调接收时间，在来源/生命周期检查完成后把该首收时间传给现有 `accept_navigation_sample(execution_id, sequence, velocity, sampled, now)`；sequence 为本次绑定的本地接收序号，不读取不存在的厂家 produced time。原有 0.16 秒非零 supersedence 策略及原配置 command TTL 保留。
- `MotionOutput(output_topic='/cmd_vel')`、`publish(velocity, deadline, *, computed_steady)`、`close()` 的内部 API 不变；`=ddddQ` 私有包维持 vx/wz/原 deadline/计算入口/序号。原生 `motion_output_guard` 唯一发布 `geometry_msgs/Twist`，实际 publish 前再次检查原 deadline，过期、EOF、关闭归零；停止值不伪造测量时间。
- B 负责将 `safety_node.py` 默认及参数解析改为 `/cmd_vel`，删除禁止 `/cmd_vel` 的 typed-output 检查；C 提供上述传输实现；D 修改 real_runtime/配置校验。三者使用同一字面值，不能保留 `/control/actuator_target` 隐藏别名。
- 通用 motion profile 仍以现有硬件字段/单位为唯一输入格式；B 删除 `simulation_only/unvalidated` 控制决策及生产 `SimulationMotionProfile` 的调用，`select_motion_profile(*, hardware_path: str) -> HardwareMotionProfile` 只加载显式文件，`configure_startup_learning(profile: HardwareMotionProfile, *, front_stop: bool, rear_stop: bool) -> HardwareMotionProfile` 保留数值覆盖。ROS 参数继续用 `hardware_motion_profile` 指定文件，不再消费 `motion_core_profile` 环境候选名；D 新建 `ros2/src/senior_sim/config/motion_profile.json`，以相同格式写环境候选数值，删除 `simulation_only` 字段，不复制 profile 类或放宽硬件许可。C 的 `agent_adapter` 仅使用通用地图/定位 reference，参数仍为 `hardware_navigation_reference`，不再构造 SimulationReference；环境需要定位时 D 生成 `senior_sim/config/navigation_reference.json` 并配置标准 map/AMCL 输入，真值只作环境诊断。
- 世界/桥接/相机/驱动/Agent 可在环境 launch 编排；底盘或仿真 drive 只启动一次，C 的导航 launch 只启动 Nav2 节点，不 include 厂家整套 bringup；D 的模拟 drive 配置用原包支持的 unstamped Twist 接口并 remap `/cmd_vel` 到该标准订阅，不编写转回 ActuatorTarget 的桥。

## Task 1: 公共时间契约、消息、构建与共享夹具（F，串行）

**Files:** Create 两个 `sample_time` 模块及 `tests/unit/sensor_time_test_support.py`；Modify 所有上表 F 拥有的消息/构建/support；Delete 下文退役公共文件。

**Interfaces:** 产出上节全部函数、字段、SDR2 格式及共享 fixture API；后续四个 worktree 只消费，不修改。`sensor_time_test_support.sample_time(*, stamp_ns, received_ros_ns, received_steady_ns, clock_generation=1) -> SampleTime` 调用真实 map_sample，`capture_fields(*, time: SampleTime) -> dict[str, Any]` 返回新 RawCaptureHeader 时间字段，不含旧证明。

- [ ] 读取最新设计、AGENTS.md 和基线状态；`rg -l 'SourceTime|Timed[A-Z]|ActuatorTarget|NavigationSample|time_proof|sim_step|simulation_only|source_timing|source_clock|simulation_timing|navigation_timing' src ros2 tools tests config`，为结果逐个登记 A/B/C/D/F，核对本计划文件路径实际存在。保存原有效回归名称与结果，不启动服务。
- [ ] 实现公共时间类型/纯映射、时钟倒退通知、原有年龄策略接线；在原有 `test_sensor_timing.py`、`test_control_clock_domains.py` 所覆盖行为上使用新必填类型，并按下文确定契约复用验证，不扩大验证目标。
- [ ] 修改 RawCaptureHeader 及所有直接时间字段契约；保留 FinalCommand、DriveExecutionFeedback、ScanMotion、有界原始/派生/TF/egress 消息的有效职责。删除 `SourceTime.msg`、`ActuatorTarget.msg`、`NavigationSample.msg`、`TimedCameraInfo.msg`、`TimedImage.msg`、`TimedImu.msg`、`TimedLaserScan.msg`、`TimedNavSatFix.msg`、`TimedOdometry.msg`、`TimedPose.msg`、`TimedTwist.msg`。
- [ ] 删除 `sensors/time_proof.py`、`senior_motion_msgs/python/source_clock.py`、`include/senior_motion_msgs/{source_timing,source_transport,actuator_timing,simulation_timing,navigation_timing}.hpp`、`src/simulation_timing.cpp`；普通 monotonic clock 工具迁到 sample_time.hpp，不留下 old namespace 转发。`motion/navigation_clock.py` 中仍用于内部命令身份/截止的工具由 C 后续保留清理。
- [ ] 一次性更新全部包 CMake/package.xml/setup.py 的生成、安装、导出和资源列表；`senior_motion_msgs` 版本升为 `0.3.0` 表达内部接口不兼容，移除旧 EXACT patched SDK 约束及无用 ament_cmake_python 安装，移除 simulation_timing target。保留 `sensor_timing` target 中有效年龄/队列策略及 installed-header consumer 检查，生成 message target 必须先加载再供手写导出引用，不能丢失 3ceadf8 的导出修复。冻结 D 的环境脚本为 `ros2/src/senior_sim/scripts/simulation_camera.py`、`world_reference.py`，由 senior_sim CMake `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME} RENAME simulation_camera)` 和对应 `world_reference` 安装项安装；senior_robot/setup.py 删除原 simulation_camera entry point，config 目录安装覆盖上述两个新 JSON。脚本实现在 D 分支完成前，只构建公共 rosidl 目标，不通过 placeholder/wrapper 补齐环境实现。
- [ ] 删除 `tests/unit/source_time_test_support.py`；迁移共享 `control_process_test_support.py`、`motion_reference_test_support.py`、`learning_test_support.py`、ROS/integration 共享设施及 fixtures 为新格式。`tests/support/native_output/` 删除 source_time/actuator_target 替身、`source_clock_harness.cpp`、仅测厂家 admission 的 `actuator_admission_harness.cpp`；有效 `protocol_harness.cpp`、`protocol_state_harness.cpp`、rclcpp shim 改录制标准 Twist、保留期限/EOF/序号断言，供 C 使用，不保留假旧消息头。
- [ ] 跑公共模块定向检查及可用 Linux ROS 环境中的 rosidl 生成目标；此冻结点的下游尚未迁移，完整包构建与 installed-header consumer 放到 Task 8，不为阶段性编译引入兼容层。若本机无 ROS，记录生成接口未验证并安排 Task 8，不能用 stub 编译宣称 ROS 构建通过。
- [ ] 提交 F 独占文件，记录冻结 SHA、schema/函数签名与 fixture 清单；所有 consumer 原有旧 API 的报错是迁移待办，不能留下新旧双轨。冻结后的公共改动必须由 F 串行提交并同步每个 worktree 后再继续，禁止 worker 私自改消息/CMake/共享夹具。

## Task 2: 隔离工作区并行派发（F）

**Files:** 只维护协调记录；不改业务代码。**Interfaces:** 四个 worker 的输入都是同一个公共冻结 SHA，输出是可合并提交、拥有文件列表、执行证据及未验证项。

- [ ] 使用已授权 Orca orchestration 为 A/B/C/D 分别创建独立 child worktree，从修复分支的冻结 SHA 出发；不采用 `--worktree current` 共享写代码，不用手工 raw git worktree 代替 Orca 管理。
- [ ] 每份 dispatch 带完整设计/本计划路径、冻结 SHA、独占文件与禁止范围、接口段、确定验证契约、清理清单；要求不 SSH、不启车/开发服务、不改厂家源码。复用原有效回归时机械迁移 fixture/字段，只有下文确定契约涉及的真实行为才修改预期。
- [ ] 同时启动四个独立包；某工作区缺文件/接口时通过协调者解决，不让一个 worker 编辑另一个 worker 的文件。工作包完成前不得把部分源码同步车端。

## Task 3: 标准 C++ 接入、扫描与 SLAM（A，并行）

**Files:** A 所有范围，尤其 `native_receipt.cpp`、`raw_receipt.{hpp,cpp}`、`scan_receipt.{hpp,cpp}`、capture/scan/rgbd ledger、`static_tf_receipt.cpp`、`scan_motion_node.cpp`、`bounded_scan_ingress.cpp`、`scan_receipt_admission.hpp`、native_capture。

**Interfaces:** 消费 SampleTime/RawCaptureHeader/SDR2；产出标准原始订阅、有界 capture 及保留双原始端点的 ScanMotion/SLAM。`native_receipt` 的标准 allow-list 覆盖 Odometry、Imu、PoseWithCovarianceStamped、NavSatFix 及既有非时间控制/地图消息；可选源实际使用的标准类型在冻结清单核对，不新增 Timed 包装。

- [ ] 把 native 原始输入在 callback 序列化之前记录 ROS/steady、核验实际 GID、去重/来源代次并 map_sample；派生 ScanMotion 透传原始端点，发送 SDR2，不使用 RMW source_timestamp 推导测量时间。保留 negative transition、有界队列、parent-death/shutdown 行为。
- [ ] `scan_receipt` 订阅配置中的标准 LaserScan，`raw_receipt` 订阅标准 Image/CameraInfo；移除 `/timed` 拼接、SourceOrder、timing/time_basis 校验。保留数据尺寸、几何、来源、队列/ledger/原始身份及失效原因；零 stamp 按公共规则处理。
- [ ] 迁移全部 capture/egress/rgbd/静态 TF 比对为新字段与有效生命周期。扫描过滤/匹配保留原始 stamp，速度由 ROS 端点差分，计算预算用 steady；扫描单帧自身 time_increment/scan_time 的原有几何语义不改。
- [ ] SLAM 采集、图提交及定位输入只消费原始测量记录，迁移公共 monotonic helper include；保留图/地图世代、防陈旧及质量判断，不修改导航算法。
- [ ] 迁移 A 拥有的现有原生 ledger/receipt/egress、scan exact-stamp/epoch/end-to-end、SLAM capture/graph 测试；重复/乱序/来源换代/纳秒/退出行为保留，退役驱动证明断言删除并写原因。跑 `uv run pytest tests/unit/test_shared_sensor_cpp.py tests/unit/test_laser_reference_publication.py tests/unit/test_scan_matcher_replay.py -q`；ROS 包定向测试在具备环境时执行，否则列为未验证。
- [ ] 自查 A 路径无 Timed/SourceTime/旧公共头引用，提交仅 A 拥有文件，报告原始→派生记录链和未验证环境；不修改 F 的 CMake 来临时恢复旧接口。

## Task 4: Python 估计、感知、学习与接收进程（B，并行）

**Files:** B 范围，重点 `native_receipt.py`、`raw_capture.py`、`sample_types.py`、`observations.py`、`estimator.py`、`pose_estimator.py`、`source_admission.py`、`temporal.py`、`motion_observation_adapter.py`、`motion_core_output.py`、`safety_node.py`。

**Interfaces:** SDR2 接收与 `received_sample`；所有点/区间观测必填时间端点；向控制/报告传递保留原时间的快照；消费 C 的 MotionOutput 稳定 API。

- [ ] 只解析 SDR2，保留 native callback steady 与 original measurement receipt 的区别；父节点标准 use_sim_time 显式传给 native 子进程。ControlInputs spawn 的 remap 使用配置标准轮速/IMU 话题，保留原有进程隔离、负向失效、队列边界和退出处理。
- [ ] 轮速/IMU、GPS/AHRS/RGB-D 等 optional 入口改标准消息并使用首次映射；ScanMotion/深度/语义已有 capture 不重新映射。删除 topology-only 仿真来源特许；硬件与环境都使用实际 GID/配置来源。
- [ ] 将 PointObservation/DeltaObservation/Observation/Origin/Heading 及序列化改为冻结必填时间结构，删除 MeasurementTime optional、host/sim 双坐标推断、所有 `sim_step` 分支；融合、对齐、Pose/GPS、运动区间采用原 ROS stamp 差。仍有效运动/误差/几何算法、模型身份、独立速度来源保留。
- [ ] 迁移 RGB-D joiner/frame builder/body motion/候选 codec、深度与语义、SpaceEstimation、启动学习/自返回；原始八路 capture 和首次时间贯穿 spawn/ledger/IPC，禁止重收/worker 完成刷新 TTL。移除 static simulation domain 的生产许可特许，保留未知场景不能默认空 exclusion、无效候选撤销及资源关闭行为。`safety_node.py` 使用 `/cmd_vel` 默认并删除旧 typed-output 禁令，保持其他控制 owner 行为。
- [ ] 同步三个 B 所属 motion replay/synthetic 工具为新时间结构，保留原协方差、frame、原 stamp 与算法语义；与 D 的车辆录制工具共同使用相同五字段 time 记录格式，不新增旧格式 reader。
- [ ] 移除 profile 的 simulation_only/unvalidated 决策与生产侧 SimulationMotionProfile 调用；同一个通用 profile 加载器读取硬件字段，不保留旧构造 wrapper。D 提供候选环境配置而非另一套生产类；独立证据不能退化成命令或真值兜底。
- [ ] 迁移 B 拥有的现有估计、纳秒、backlog、RGB-D、学习、控制隔离测试；用 F 的新 fixture，删除专为旧 proof/time_basis 的断言，保留年龄、乱序、来源换代、队列等待、时钟倒退、急停与 TTL 的有效行为。按所属位置跑 `uv run pytest tests/unit/test_sensor_timing.py tests/unit/test_native_motion_observation_adapter.py tests/unit/test_motion_core_event_time.py tests/unit/test_reference_nanosecond_boundary.py tests/unit/test_motion_reference_backlog.py tests/unit/test_rgbd_candidate_codec.py tests/unit/test_learning_dual_clock.py -q`，其他现有相关回归由整合清单选择。
- [ ] 搜索 B 路径确认旧证明、可选旧时间语义及仿真决策分支为零，提交 B 独占文件，报告快照/候选/学习链保留情况；公共夹具不足由 F 修正，不复制 helper。

## Task 5: 标准导航、定位与唯一最终输出（C，并行）

**Files:** C 范围，重点 `command_node.py`、`dedicated_controller.py`、`navigation_adapter.py`、`hardware_reference.py`、`agent_adapter.py`、`motion_output_guard.cpp`、`motion_output.py`、两个生产 launch。

**Interfaces:** 消费原包 Twist/PoseWithCovarianceStamped、B 的 received_sample、私有命令通道稳定格式；输出唯一标准 `/cmd_vel`。生产导航输入/任务结构维持现有 action/MapRef/grid/task epoch/retirement API。

- [ ] DedicatedController 删除 overlay marker/ABI 强制同版补丁检查及自定义 producer 参数，保留每次 run 的私有 remap、controller/navigator 生命周期、取消失败处理、迟到结果丢弃、run 退出/来源收集；不以常驻公共 `/cmd_vel` 代替私有来源绑定。
- [ ] CommandNode 改标准 Twist，第一步捕获首收 steady；来源身份、run topic、图拓扑/真实 GID可用信息核验后，用本地接收序号和首收时刻接现有 arbiter。移除 NavigationSample 字段校验而保留有效任务截止/取消/急停/手动接管；不宣称知道第三方计算入口。
- [ ] HardwareReference/可选定位/AMCL 改标准 PoseWithCovarianceStamped，消费 native 首次时间；保留地图内容匹配、坐标和任务世代、来源变化、防陈旧及定位方差判断。AgentAdapter 删除 SimulationReference 构造与相关产品参数分支，通过通用 reference 接环境标准输入；地图缺失继续按原需求不可用，不制造真值许可。
- [ ] 原生输出改发布 Twist、默认 `/cmd_vel`，保留私有包原计算入口/deadline、队列/序号/超时/EOF/关闭时归零及实际 publish 前检查。删除 SourceTime producer 和 ActuatorTarget，而非多加一个转换节点；普通 monotonic helper 使用新公共模块。
- [ ] 核对安全 launch/导航 launch 的进程图：底盘/IMU/EKF/TF/雷达只有原顶层 bringup 启动一次；导航只起 Nav2/适配器，不包含第二套厂家 bringup，controller/recovery 都只能到项目输入，底盘执行话题仅 motion_output 发布。算法插件/BT/路线动作语义不借此次迁移重写。
- [ ] 迁移 C 所属现有 `test_navigation_command_clock.py`、`test_navigation_completion_handoff.py`、`test_navigation_core_output.py`、`test_native_output_protocol.py`、`test_motion_ros_twist_output.py`、任务生命周期与 ROS 导航回归；`test_navigation_overlay.py` 仅 patched SDK/producer 的测试删除，仍有效的 run/TTL 行为迁回已有导航测试位置。运行这些定向单元测试；macOS/Linux/ROS 限制如实记录，不能用跳过输出协议替代证据。
- [ ] 自查无第二个最终速度 owner、没有 overlay 调用/自定义 Nav2 参数，提交 C 文件并给出 launch/话题连接证据；不编辑 B 的 safety_node 或 D 的配置。

## Task 6: 环境、部署配置、工具、退役资产（D，并行）

**Files:** D 范围及下列完整 overlay 目录；部署/环境文档按上表归属。

**Interfaces:** 环境生成通用 profile/reference 配置和标准传感器话题；消费生产 launch 及公共字段，不增加环境专属控制 API。工具仅读写当前时间记录，旧捕获数据不自动兼容。

- [ ] 删除 `tools/prepare_sensor_overlay.py`、`prepare_nav2_overlay.py`、`prepare_simulation_overlay.py` 及 `tools/{sensor_overlay,nav2_overlay,simulation_overlay}/` 全目录（patch/manifest/CMake/自建 core 全部退役）；不把其中生产者时间证明搬回项目。独立有效项目生命周期/截止职责已由 C/生产 owner 承担。
- [ ] 将 `senior_sim/config/bridge.yaml` 只保留标准 Clock/LaserScan/Imu/Image/CameraInfo/Odometry，配置原装 ros2_control 的标准 Twist 接口及正确 remap，URDF/插件清除项目时钟/ActuatorTarget 依赖；不修改已安装组件。
- [ ] 环境 `simulation_motion.launch.py`、RGB-D/相机 launch 只编排世界、1 倍标准输入、通用控制/Agent；移除仿真专属 reference/profile/特殊放行参数。删除 `simulation_reference.py`、`simulation_motion_profile.py`；将相机脚本移至冻结的 `senior_sim/scripts/simulation_camera.py`，其入口不参与控制许可。旧 SimulationReference 中仍有效世界栅格/坐标变换/scan 投影搬至 `senior_sim/scripts/world_reference.py` 作为环境生成工具，输出冻结的 navigation_reference.json，不被生产控制 import，不保留真值资格类。环境候选参数用通用配置，标准 map/AMCL 或项目既有通用定位链提供网页状态；F 在分叉前安装该脚本至 senior_sim 的 lib 路径。
- [ ] 更新 real_runtime/CLI/vehicle_deploy 以及 config 中的标准话题、`/cmd_vel`、新包资源，保留联合启停、身份握手、私有目录、退出和单次 bringup；导航可选配置缺失维持原行为。环境的 use_sim_time 由启动参数传递，控制层无环境分支。
- [ ] 更新 `scan_matcher_replay.py` 和全部 `vehicle_motion_{capture,analyze,calibrate,fixture}.py` 的记录/解析/生成，当前捕获行使用 `time` 对象的五个必填 ns/generation 字段；区间为 first_time/last_time，无旧 key 回退。B 同步其三个 replay/synthetic 工具；需要更新共用 JSON fixtures 时仅向 F 提交确切内容清单，不直接写共享文件。
- [ ] 退役 `test_simulation_clock_overlay.py` 中引擎 hook、倍率、同值 reset 专门保证及 `test_simulation_reference*.py` 的真值资格专属断言；保留仍有效的标准环境启动/话题/退出/车端行为，迁入已有适当测试文件。`test_simulation_reference.py` 中一般地图/任务行为如有则交 C 的既有通用 reference 测试，不整文件无审计删除。
- [ ] 运行现有 `test_common_sensor_composition.py`、`test_motion_launch_profile.py`、`test_motion_cli_lifecycle.py`、`test_vehicle_motion_analyze.py`、`test_vehicle_motion_fixture.py`、`tests/integration/test_deployment_config.py`；环境能力保留不等于新增网页浏览器测试。正式完整仿真只在少量确有必要的 1 倍场景中运行，不新增仅环境连通、倍率矩阵或依靠真实等待的慢测试。
- [ ] 更新 operations/training/README 中活跃使用说明，旧 overlay 操作页标明退役并指向本设计及标准部署说明，删除活跃命令入口；提交 D 独占文件、退役行为和需 F 同步的构建/fixture 清单。

## 确定验证契约与任务归属

此处列出本轮确定行为；表内测试均复用现有位置，必要时改 fixture/接口及明确改变的预期。其他有效回归保留，不能因引用旧时间类型整批删除；不要求实现前新建测试。`u/` 为 `tests/unit/`，`r/` 为 `tests/ros/`，原生包内位置写全路径。

| # | Owner / 阶段 | 确定输入与可观察结果 | 复用位置 |
| --- | --- | --- | --- |
| 1 | A / Task 3，F 最终真实链 | 标准 `/scan` 经真实接入、过滤、matcher；stamp/首次接收不变，不等待 `/timed` | `r/test_scan_native_end_to_end.py`；`ros2/src/senior_sensor_ingress/test/test_raw_scan_receipt.cpp` |
| 2 | B / Task 4 | 配置标准 Odometry 保留速度、协方差、frame，仅为轮速，不能冒充独立车体速度 | `u/test_motion_fusion_replay.py::test_native_wheel_retains_relevant_cross_covariance_without_certifying_it` |
| 3 | B / Task 4 | 标准 Imu 进入角速度来源，不制造积分区间或统计模型 | 同文件 `test_native_imu_has_no_fabricated_interval_or_statistical_model` |
| 4 | F Python / Task 1，A C++ / Task 3 | stamp=99.8、receipt_ros=100、receipt_steady=20 → sample=19.8；20.15 消费 age=0.35，不是0.15；两语言同值 | `u/test_sensor_timing.py`；`u/test_shared_sensor_cpp.py` |
| 5 | B/A / 各自路径 | 原整数纳秒保留，同秒相差1ns可区分，不因浮点 ULP 误判未来/重复 | `u/test_motion_fusion_replay.py::test_native_stamp_matches_integer_bag_clock_without_false_future_ulp`；`r/test_scan_exact_stamp.py` |
| 6 | B / Task 4 | age=0.300秒为 low，原 deadline 不刷新 | `u/test_sensor_timing.py::test_common_boundary[0.3-low]` |
| 7 | B / Task 4 | age=1.000秒为 invalid，原 deadline 不刷新；纳秒边界一致 | 同文件 `test_common_boundary[1-invalid]`；现有 `test_reference_nanosecond_boundary.py` |
| 8 | B / Task 4，A 端点传递 | 两帧扫描经处理延迟仍保留两次原始时间及较早依赖 deadline，到期失效 | `u/test_unified_motion_runtime.py::test_laser_delta_retains_both_raw_capture_receipts_through_processing_delay` |
| 9 | B / Task 4 | 原始样本重收、快照重发不续命、不重复积分或增加有效恢复计数 | `u/test_reference_clock_frontier.py::test_republishing_cached_raw_rows_does_not_refresh_original_deadline`；`u/test_motion_reference_backlog.py` |
| 10 | B / Task 4，A 接入撤销 | 普通 ROS 回退撤销受影响窗口，随后建立新段，旧段迟到不能接续；保留其他来源隔离 | 改 `u/test_reference_clock_frontier.py::test_local_ros_rollback_preserves_estimates_and_source_generation` 旧预期为本行要求 |
| 11 | B/A / 各自来源 owner | GID A→B，B 可建立；A 排队消息不能恢复或撤销 B，原时间不重盖 | `u/test_raw_capture.py::test_bridge_and_raw_restart_retire_old_data_without_restamping`；现有真实 native receipt 来源替换测试 |
| 12 | B / Task 4 | 30/50Hz 轮速/IMU 按原 stamp 因果对齐及 SE2 预测，不用处理间隔、不虚构观测侧向速度 | `u/test_motion_temporal.py::test_30_50_hz_causal_zoh_turning_prediction_is_complete_but_not_observed_lateral` |
| 13 | B / Task 4，A 原始 capture | 标准 Image/CameraInfo 四路按原 stamp/body 与静态 TF 绑定，不跨来源重启混帧、不重新盖时间 | `u/test_rgbd_joiner.py::test_exact_four_stream_join_and_newer_frame_keeps_inflight_generation`；`r/test_rgbd_generated_boundaries.py` |
| 14 | B / Task 4，C 标准定位接入 | 标准绝对定位与相对运动对齐，map/frame generation 改变不混世代、不产生陈旧任务位置 | `u/test_pose_space_estimation.py::test_space_aligns_sensor_poses_before_union_and_keeps_frame_epochs_separate` |
| 15 | C / Task 5 | 私有导航标准无 header Twist 首回调记录 steady，资格检查耗0.35秒不能续命，不声称 Nav2 计算时刻 | 改 `u/test_navigation_command_clock.py::test_freshness_uses_current_time_after_all_qualification_work[qualification]` 的旧输入/预期 |
| 16 | C / Task 5 | 实际发布恰等于原 deadline 不发非零并归零；最终类型为 Twist | `u/test_motion_core_exit_deadline.py::test_actual_exit_must_precede_original_aggregate_deadline[0.0]`；原生输出 harness |
| 17 | C / Task 5 | 有效命令后 IPC EOF/owner 关闭发布标准零 Twist，不重试旧非零 | `u/test_native_output_protocol.py::test_production_native_receiver_protocol[eof]`；现有关闭传输保护 |
| 18 | F / Task 8 | 干净安装全部当前 typesupport 可加载；独立安装 C++ consumer 编译成功；退役消息/库/patch 依赖无残留 | `ros2/src/senior_motion_msgs/test/check_installed_headers.cmake`；现有 type support 路径加载检查 |
| 19 | C / Task 5，F 最终只读现场 | 标准 Nav2 组合只有一个底盘/串口 owner；导航只发私有命令，底盘 `/cmd_vel` 唯一项目最终 publisher | `r/test_navigation.py` 真实图检查；车端仅现有静态观察，不发速度/任务 |

Python/C++ 替身通过不替代真实 ROS 生成/构建、DDS 及现场静态生产链证据。行为测试只在开发机或隔离 Linux ROS 域执行，含任何非零速度的 harness/ROS 用例绝不运行于实车域。源码/导出/配置引用搜索属于清理审查，不创建“字符串不存在”的行为用例。

### 混合测试与资产的具体退役动作

| Owner | 退役项 | 必须迁移保留的有效部分 |
| --- | --- | --- |
| C | `test_navigation_overlay.py` 整文件的 patched version/marker/checksum、提取厂家 publishVelocity、NavigationSample Producer | 私有 run、首收 TTL、零/非零转发在现有通用导航/输出测试保留 |
| D/F | `test_simulation_clock_overlay.py` 全部3项、source_clock_harness、共享 epoch 动态库测试 | 原 stamp 差分/steady TTL 由确定契约与已有算法测试保留；同值 reset 自动识别不再保证 |
| C/F | `test_native_output_protocol.py::test_actual_actuator_admission_preserves_origin_boot_order_deadline_and_two_axis_contract`、actuator_admission_harness、旧消息替身头 | 保留该测试文件其余有效私有 IPC、非有限/序号、两轴、期限、EOF/关闭行为；真实生产 receiver 改标准 Twist |
| B/F | source_time_test_support、synthetic_domain/synthetic_capture_schedule 的 boot/sim_step 证明与旧数据“补证明”fixture | 显式标准 stamp/ROS/steady 新 fixture；`reference_clock_frontier.json`、`motion_reference_backlog.json`、`reference_nanosecond_boundary.json`、`laser_publication_phase.json` 的原有效回放不整删，缺原 receipt 的材料仅标合成几何，不证明采集年龄 |
| B | `test_control_clock_domains.py` 的 foreign boot/缺 tick、9项倍率×修正和3项rate矩阵、跨world/世界轴证明 | 非有限运动拒绝、SE2 路径、稀疏/密集消费、绝对锚分段预测、不重复积分、steady 到期、来源重启；改同段标准 stamp，不把该文件整删 |
| D/C | `test_simulation_reference_admission.py` 中 truth/timed 配对、任意ROS标签有效、world reset/真值许可；删除旧生产 reference 测试结构 | D 将世界栅格/坐标变换/scan 投影保留在原 `test_simulation_reference.py` 对环境模块验证；C 将来源替换、frame/quaternion、到期/重复等通用保护迁入已有 reference/定位测试 |
| B | `test_rgbd_frame_builder.py::test_empty_exclusion_requires_explicit_static_sim_domain` 与 ROS candidate producer 中静态仿真domain专属许可断言 | 未知场景不能默认空 exclusion、invalid 候选、资源关闭原行为继续保留 |
| D | `test_motion_launch_profile.py` 的旧候选名/qualified_reference 绑定；`test_sim_contract.py` 中 ActuatorTarget、odom/timed remap 及禁止 cmd_vel 预期 | 通用控制编排、stock 标准路由及仍有效几何/配置检查；不添加仅环境连通的新例行用例 |
| B | `test_reference_clock_frontier.py` 回退仍保留原窗口的旧预期；`test_startup_learning_timed.py::test_ros_labels_do_not_order_or_deduplicate_learning_frames` 的任意ROS标签/sim_step前提 | 回退按契约10撤窗；学习原始帧去重、换段中断、固定3秒deadline、消费窗口不重播继续保留 |
| D/F | `vehicle_motion_trace.json` 及工具中 time_proof/world reset 假设 | 故障窗、interval 平均、非有限语义、录制失败可见；reset 后不混旧帧仅在可观察时钟换段/来源重启条件下验证 |
| F | installed consumer 的退役 includes/sizeof；外部 overlay 缓存/同版 patched SDK 前提 | 独立安装导出/typesupport依赖顺序及内部 FinalCommand frame 测试保留，不删除 CTest |

这些是退役行为说明，不扩大确定验证契约；其他有效算法、控制、学习、急停、来源和资源管理回归仅机械迁移所需字段/fixture。`tests/ros/test_motion_lifecycle_restoration.py`、receipt/matcher shutdown、IPC 身份与退出、独立状态报告保护必须继续保留；内部 Fenced* 和 FinalCommand 私有格式不因外部 Timed 退役而自动删除。

## Task 7: 顺序整合与全仓清理（F）

**Files:** F 文件、最终集成必要改动及历史状态说明；合并后文件统一由协调者接管。**Interfaces:** 四包全部已结束且给出提交/测试证据；没有运行中的 worker 与集成者同时写同一文件。

- [ ] 逐包审查实际 diff、删除理由、回归/未验证证据；按 A→B→C→D 正常 merge 回修复分支，解决冲突保留双方有效职责，不覆盖整文件。每次 merge 后做接口/import/生成字段检查；完整回归在全部合并后运行。
- [ ] 核对共享 schema/CMake/fixtures 仅 F 修改，落实 worker 报出的剩余安装/资源变更；必要修正串行进行并单独提交。审计 `NavigationSample` 及 source/nav/simulation 头的全部使用，保留 FinalCommand/DriveExecutionFeedback 私有计算时间/期限，不误删内部 IPC。
- [ ] 全仓搜索生产、构建、配置、工具和测试：`rg -n 'SourceTime|TimeProof|Timed[A-Z]|ActuatorTarget|NavigationSample|time_proof|source_timing|source_transport|source_clock|actuator_timing|simulation_timing|navigation_timing|driver_read|sim_step|simulation_only|simulation_qualified_reference|/timed|/control/actuator_target' src ros2 tools tests config`；每个命中必须删除/迁移或有明确非执行历史原因，最终活跃路径为零。另查 `MeasurementTime|MeasurementInterval|measurement_time|measurement_interval|time_basis|measurement_epoch|measurement_tick_ns|simulation_topology_only|static_sim_domain`，不能换名保留可选双坐标/证明解析或仿真来源特许。
- [ ] 审计启动图及生产 import：生产不 import 环境实现，导航不二次启动底盘；标准原始输入、相机/RGB-D、GPS/AHRS、派生 scan/估计/学习、map/AMCL/SLAM、Nav2、输出、实车与环境配置逐项对应 owner 和验证证据。
- [ ] 为删除测试逐项登记“仅旧协议/补丁/特殊许可/reset 保证”或“有效行为迁移到现有位置”；共享录制数据/替身/fixtures 不遗留旧格式。逐项核对下文确定验证契约的归属与证据，其余原有效算法、急停、TTL、任务、来源和资源管理回归只作所需 fixture/字段迁移，保持原输入及断言语义。
- [ ] 更新活跃旧设计/计划/硬件能力说明，历史证据仅标版本已被本设计取代；生成最终改动和验证清单，提交集成清理。

## Task 8: 真实构建、受限车端验证与交付（F）

**Files:** 仅必要构建/集成修正和 `docs/testing/` 真实结果；无车端手改。**Interfaces:** 完整合并提交通过源码审查后，才进入车端 Git 更新。

- [ ] 跑 CI 同等本地检查：`uv run --no-sync ruff check .`、`uv run --no-sync ruff format --check .`、`uv run --no-sync mypy src/senior_control`、`uv run --no-sync pytest tests/unit tests/integration`；预先确认依赖存在，安装环境变化单独记录。已有不相关失败与本迁移失败区分，不能把 skip 写成已验证。
- [ ] 在 Linux ROS 干净构建环境仅加载官方 ROS 和现有厂家 underlay；核对项目 `build/install` 中六个包 `senior_motion_msgs/senior_sensor_ingress/senior_scan_matcher/senior_slam_bridge/senior_robot/senior_sim` 的生成目录。仅删除已确认的项目生成物，merged-install 采用新的干净输出目录而不盲删混合安装；保留源码、配置、凭据、未跟踪文件和厂家 install。
- [ ] 在隔离 Linux ROS 构建环境的 `ros2/` 执行 `colcon build --symlink-install --packages-select senior_motion_msgs senior_sensor_ingress senior_scan_matcher senior_slam_bridge senior_robot senior_sim --cmake-args -DBUILD_TESTING=ON`，然后 `colcon test --packages-select senior_motion_msgs senior_sensor_ingress senior_scan_matcher senior_slam_bridge senior_robot senior_sim`、`colcon test-result --verbose`；真实生成消息和 installed-header consumer 必须通过，旧 SourceTime/Timed/ActuatorTarget/NavigationSample Python/C++ 安装入口不再存在。不得加载已退役外部 overlay 掩盖链接问题；该测试命令不在实车 ROS 域执行。
- [ ] 执行协调者确定的现有 ROS 标准输入回归与快速可控时钟/回放车端测试；少量必要完整仿真按 1 倍，不追加网页操作测试。缺少 Gazebo/Nav2/RGB-D 环境的能力逐项标未验证，不能用静态实车观测代替。
- [ ] 由协调者使用用户允许的车端操作入口：项目 CLI stop → Git 拉取完整修复分支/核对 SHA → 清理确认的本项目生成物并构建 → 项目 CLI start/status，并读取已确认服务的日志；使用现有 `senior-robot` 命令与实际 control-dir，不虚构 `logs` 子命令。车端不手改、不运行临时补丁；读取状态/日志验证标准 `/scan`、`/odom`、`/imu/data_raw` 真实进入生产链且状态持续更新、IPC/来源/数据年龄正确，不只看 mock。仅观测，不发运动指令、不触发学习或导航。
- [ ] 根据生产日志/项目状态核对唯一最终输出 owner 和底盘只启动一次；若项目现有 CLI 不暴露所需只读证据，先在本地补充到允许范围的 CLI/日志后 Git 部署，不在车端绕过约束运行临时源码。扫描几何或独立速度现场退化如实报告证据，不能篡改阈值或清除急停。
- [ ] 更新实际 PR 的标题/说明，覆盖完整标准消息迁移、删除清单、保留算法/生命周期、验证及未验证能力；正常提交/推送修复分支，不自动合并。确认未跟踪文件保留，完成后本地切回 main。

## 可逆实施者判断与剩余事项

没有阻塞实施计划的用户问题；不再次询问执行方式。以下决定由实施者在上述接口内完成并在交付中记录：环境通用 profile/reference 的具体数值沿用当前候选配置，文件路径采用上述冻结位置且不声称实车标定；现有测试中混合旧证明与有效算法的拆分按行为逐项迁移；真实构建/完整仿真缺失的环境按未验证记录，不添加新许可或硬件证明要求。

确定验证契约已按上节纳入计划，不把更大候选矩阵扩成此次目标。公共接口若实施准备时确需调整，须在 Task 2 分叉前由 F 一次锁定，不给并行 worker 留临时适配责任。
