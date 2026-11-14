# T3/T4/T5 实际输出接线（待代码审核及 Linux 同版运行）

2026-11-13。当前阶段是 simulation-only/unvalidated 的实际 ROS 适配代码，不是 5 km/h 验收完成。未启动 ROS、Gazebo、开发服务或硬件，未操作 index、提交或推送；T1 acceptance 的 null 未修改。T6 的 producer/msg/protocol/Agent/UI 文件只读消费。

## 输出与责任

`motion_fusion.NativeObservationRosBridge` 把真实 `/wheel/odom`、`/imu/data`、`/motion/scan` 接入同一个 T3 AdmissionOwner→TemporalResolver→NumericalFusion。保留原 ROS 点/区间、接收 steady、DDS GID 与 producer epoch；IMU 只旋转三轴角速度到 body，不积分加速度。ROS 协方差不自动获得统计模型身份，可信 registry 为空，真实 CI 未因此变成有效。独立 body 区间平均与 wheel 点分别输出；truth 没有产品订阅。

内部 `/motion/core_reference` 为有界 JSON，仅接受根命名空间唯一 motion_fusion 的对应 DDS GID。SafetyGate 复用原受界扫描入站/epoch/源时效检查。新增 core 从该观测构造 T4 Context，消费单 flight/latest pending scheduler 与 T5 controller/state machine；原 SafetyGate 的同一个 Twist publisher 是唯一出口。选择新链后旧 `_publish` 不再输出，未选择时旧链保留。最后轨迹/PI/轮组合后的候选仍经 T4，None 不保留旧非零命令；不可证明时的零输出明确是未验证异常停止请求，不是安全/静止证据。

普通 release/expiry/retirement 先 fence 旧输入与异步许可，保持轨迹 v/a，原子切换为本机制动 owner，再回 navigation token 的 `retired`；它不代表硬 Twist0 或车体静止。没有成功接管不回 ACK。普通 intent 保留原 received/deadline，重发不续命。本机制动不使用旧用户 authority；新目标仍由当前状态/约束核验。实际发布后复用 OutputFeedback 回传 manual applied/limited/blocked，不用目标冒充输出。

关闭 identity=(boot,generation,sequence) 的全字段相等重发只应用一次，可重复确认；内容偷换拒绝，未完成接管可按同一内容重试。已锁存 estop/fault 的来源回声不重复增 revision。按协调者确认的契约，非 navigation 的新非空 transport request_id 表示显式软件急停事件，空 ID/navigation close 仅确保锁存；request_id 不推断硬件事件。estop_reset/fault_recover 仅 idle、空 source_id/epoch0、零序号/截止/目标，revision 可为0，最终仍检查本地新鲜独立静止证据。硬件输入并无本轮新增接线，不能声称实车硬急停已接通。

`/motion/runtime` 严格复用 backend 的 MotionRuntimeState 序列化，包括 schema=1、原证据期限、body_reference 或 null、独立 wheel、版本/动作/输出 generation。缺独立 body 不用 wheel 填 body/静止。反馈暂挂只 fence 输出，不再次改成用户暂停；保留 T5 已定义的反馈恢复后 replan 动作，显式暂停/接管不自动恢复。

## 有界帧迁移

实际 final_command_guard 和 Python decoder 成对改为 SCF2/v2、固定799字节、网络序、显式长度；传递五个新来源字段与原 GID/boot/generation/session/deadline。字符串先查上界再写，无旧帧/截断补默认。保留非阻塞 private seqpacket、唯一 command_node 源、原准入与完整重复一致性。受限 token retirement/恢复分支不会全局放松 gesture/request 规则。C++17 原生 sender 使用生产 codec，真实 socket 发送后由生产 decoder 和实际 SafetyGate `_poll_command_guard/_on_command`、FinalCommandFence、本机 owner 消费；没有 mock 掉 `_on_command`。macOS 测试 transport 用 AF_UNIX datagram（该主机不支持 seqpacket）；Linux seqpacket/RMW 和完整 ROS C++ guard 尚未编译运行。

## 候选 profile 的出处与假设

只能显式选择 `fortress-candidate-unvalidated-v1`，同时要求 use_sim_time、LOCALHOST_ONLY=1、域173；默认参数为空，硬件默认不会启用。Gazebo partition 和本次独占 unit 由外部受审运行器隔离。profile 非持久化驾驶设置，驾驶设置的请求范围仍归 T6 SettingsService。

| 字段组 | 数值与依据（不是冻结保证） |
|---|---|
| 车体/轮声明 | .674×.552 m、track .481154 m、轮组合1.52 m/s，来自当前 URDF 声明；T4 1.05 几何膨胀一次，不是量产实测 |
| 响应模型 | A/D .3 m/s²、yaw A/D .5 rad/s²、lag .2 s 为旧仿真 nominal 配置/候选；jerk1/2、P .2、I0 为本轮明确假设，不是测得最大能力 |
| 请求/模式 cap | 正向25/18 m/s、角2 rad/s、wheel退化 .3 m/s 为仿真候选策略；实际轮组合/方向证明可进一步收紧 |
| 误差/驱动关系 | body/yaw error .1、sensor .02 m、position .03 m、drive overshoot .1/.1，均 unvalidated 假设；不能用未知 P 推导确定上界 |
| wheel-only 风险域 | 独立假设被动平地车体速界1.52及驱动方向，不是 wheel±常数/真速；完整抓地恢复与停车响应关系待 Linux 核验 |
| 时域 | profile 本次启动175 s，观测 .8 s、当前投影≤.15 s、静态场景 forecast12 s；body仍保留原区间/键/截止，误差包络计入年龄与区间半宽 |

选择候选时 launch 仅把 matcher 运动域从 .5/.8 提到1.52/2，允许测正常高档/转向；没有改 response/residual/inlier/observability/covariance 阈值或评分算法。`motion_core_world` 可显式引用已审丰富地标长场，默认空不改变旧场景。它不代表弱特征问题已解决。插值 free 窗口只使用相邻有效测距、显式插值契约；近盲区/无回波/缺束保持未知。完整无缺束一圈首尾拼接消除人工接缝，非完整扫描不跨空缺。

## 验证入口与结果

详见证据目录 `/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5b-output-wiring` 的 scoped patch、前后 SHA、完整日志。先用失败测试复现恢复事件角色缺口、完整扫描接缝、关闭重复与反馈暂停因果，再修复；正常/缺流/缺束负对照保留。

```sh
PYTHONPATH=.:src:ros2/src/senior_robot .venv/bin/pytest -q \
  tests/unit/test_motion_{core_handoff,core_chain,core_perception,guard_to_owner,trajectory,controller,state_machine}.py \
  tests/unit/test_native_motion_observation_adapter.py \
  tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py \
  tests/ros/test_motion_core_acceptance.py
.venv/bin/python -m unittest discover -s ros2/src/senior_motion_msgs/test -p test_final_command_frame.py
MYPYPATH=src:ros2/src/senior_robot .venv/bin/mypy --strict --follow-imports=silent \
  --ignore-missing-imports --explicit-package-bases \
  ros2/src/senior_robot/senior_robot/{motion_core_output,motion_core_perception,motion_observation_adapter,simulation_motion_profile,final_command_frame}.py
```

限定回归 **107 passed / 4 skipped**（四项真实ROS入口未运行）；原生 C++ 编解码 **5 passed**；Ruff限定文件通过，严格 Mypy五个新纯适配模块通过（ROS动态依赖 follow-imports=silent/ignore-missing-imports，非完整ROS类型/编译证明）。真实 C++ 帧→实际 gate/fence→owner 的测试还覆盖两次显式软件急停、同请求重发和锁存回声，revision分别1/1/2/2/2。

初次回归少了 `PYTHONPATH=.`，两个既有 tools 导入失败；保留失败日志，修正环境后重跑，不改测试。纯组合测试是真实 T3/T4/T5 数学链，ROS I/O 用 fixture，观测标为合成；它只证明接线逻辑，不是物理 plant 或 Linux 时延验证。

## 四个正常 Linux 入口（已接线、未实跑）

由负责人审本 diff 后，在新的冻结 source/build/install 下构建并核验实际导入路径，沿用 `.vm-test.lock`、独占 unit/cgroup、外 watchdog/finally stop。每个 case 新原点静止，不能一个持续运行的世界跑四次。以下是 unit 内命令，非本轮已执行记录：

```sh
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="senior-motion-core-$RUN_ID"
export GZ_PARTITION="$IGN_PARTITION"
export SENIOR_SIMULATION_SETTINGS=1
# source 同版 ROS Humble、冻结 venv 与 install/setup.bash；先核验路径/SHA。
# CORE_WORLD 引用已审长直/转向丰富地标 world（源 SHA 与本次 fixture manifest 一起登记）。
timeout --signal=INT --kill-after=5s 175s ros2 launch senior_robot simulation_motion.launch.py \
  motion_core_profile:=fortress-candidate-unvalidated-v1 qualified_reference:=true \
  motion_core_world:="$CORE_WORLD" robot_id:=route-sim agent_socket:="$RUN_SOCKET" \
  safety_state_path:="$RUN_ESTOP" headless:=true
# ready后另一个同unit进程执行单case；unit退出finally stop并确认只回收本次cgroup。
SENIOR_MOTION_CORE_ACCEPTANCE=1 SENIOR_SIM_NAV_SOCKET="$RUN_SOCKET" \
  SENIOR_MOTION_CORE_EVIDENCE="$RUN_EVIDENCE" \
  timeout --signal=INT --kill-after=5s 95s python -m pytest -q \
  'tests/ros/test_motion_core_acceptance.py::test_motion_core_normal[low_start_stop]'
```

其余 case 是 `high_start_stop`、`low_turn`、`high_turn`。straight 请求5/9、25/18 m/s持续12 s；turn 请求各档25%线速/.35 rad/s持续8 s；ready≤25 s、停车≤15 s、case≤90 s、仿真175+5 s。按候选 A/J/lag，high直行最坏测试路径预算需约25 m可观测自由距离，turn需要原点半径至少7 m；不要在默认短场景直接跑高档。外部运行器必须先验证实际场地，并同步 bag `/cmd_vel /motion/runtime /motion/core_reference /motion/scan /wheel/odom /imu/data /ground_truth/odom /safety/raw/scan` 及扫描诊断。真值只由测试 observer评分。

测试直接走本机可信 manual IPC→T2 prepared/neutral/intent/release→guard→输出，用显式五项仿真设置；不覆盖 cloud 登录/网络 relay 验收。输出 curve.csv 与 case.json、实际 body达到请求≥80%、转向≥.25 rad/s、末.5 s真值|vx|<.02/|wz|<.03、release连续非零减速等为运行前固定候选目标，失败照录，不回调标准。当前状态四项均为“已接线未跑”，本地收集为4 skip，无任何 Gazebo通过声明。

## 仍需核验的最小集合

1. 同版完整 guard ROS编译/seqpacket、RMW GID、T6实际显式 ESTOP request_id 透传与 runtime 回程；手动新的press/同请求重发/锁存回声须同版复核。
2. 普通运行 scan有效性、原区间 lag/误差与 wheel/IMU 质量；当前真实 CI仍0、移动扫描误差大，不能证明候选 .1误差成立。新链 wheel-only可受限预测，但缺独立body时不会声称静止/复位证据；初始全程弱场时新acquire缺静止证据仍是实际未闭合项。
3. body低于drive、残余加速度/抓地恢复的实际停车超调/时间模型；当前profile仅假设，不能据逻辑proof宣称物理安全。热重启时 trajectory初始0也未建立移动状态重建，正常验证限新原点静止。
4. 原始scan/轮pose对齐使用当前轮里程近场运动模型（未滑移条件），静态12 s环境假设不适用动态障碍；当前仅 laser_only、无 GPS/native depth 组合接线。本轮不以这些未测矩阵阻止四个正常候选验证，也不宣称实车可启用。

下一步是亲审 scoped diff 后编译并运行上述四个正常入口，按真曲线修正候选/必要局部适配，不再扩纯模块。机械拆分约束几何仍不混本批。
