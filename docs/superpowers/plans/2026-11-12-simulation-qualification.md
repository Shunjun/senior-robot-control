# 仿真运行与操作闭环修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复已复现的仿真时序和导航问题，以同一冻结版本实际验证手动驾驶、三点路线、失联停车、实时地图、视频与撤销配对。

**Architecture:** 保留现有唯一速度链、独立运动证据、原始接收时间及代次隔离。软件资格误判、运行环境阻塞、真实轮地滑移分别用独立证据定位。物理模型或控制器的改变必须另行验证，不能借旧安全校准直接启用。

**Tech Stack:** Python 3.10/3.12、ROS 2 Humble、Gazebo Fortress、Nav2、React、uv、systemd。

**Spec:** [操作工作台设计](../specs/2026-11-08-operator-workspace-design.md)、[机器人控制设计](../specs/2026-10-27-robot-control-design.md)。实测见 [验收结果](../../testing/results.md)。

## Global Constraints

- 本阶段只做仿真验收；实车验证按用户要求跳过。
- 500 ms 是速度样本最终接纳的传输年龄上限；300 ms 许可与接纳后 200 ms 最低计划窗口保持独立。
- 不延长传感器时效、不刷新排队样本接收时间、不放宽锁存或已建立的代次隔离。
- 不把 ROS 接纳、云端在线或单位测试通过当作车体运动证据。
- 调试服务必须在独立 ROS 域和 Gazebo partition 中有界运行，退出后恢复原仿真服务。
- 改轮地接触、摩擦或控制动力学时，保留旧证据；新模型未经重测不能沿用 `fortress-flat-v1`。
- 兼容逻辑标注 `//Compatible`；前端使用具名多属性 props 和局部样式。

## Review Focus

1. 两路样本乱序或时钟暂时领先：只使用原有效期内的同源匹配数据；真实过期仍应停止。
2. 当前轨迹扫掠受阻：停车并允许既有导航重规划，不能向新任务或网页误报安全。
3. 四轮转向时轮与车体差异：使用同窗真值核验，不能通过轮估计补出运动或降低滑移阈值。
4. 虚拟机调度或 DDS 查询阻塞：区分调度等待、原生函数等待和计算耗时，诊断负载不能冒充验收。
5. 视频订阅和设备删除竞态：重订阅收到新帧；删除后不能被旧刷新或旧连接恢复成可控设备。

## Task 1: 已复现的软件资格和生命周期缺口

**Files:** `src/senior_control/cli.py`、`simulation_cli.py`、`runtime/ipc.py`；ROS `agent_adapter.py`、`command_node.py`、`simulation_reference.py`；对应单元、IPC、网页设备测试。

**Interfaces:** `requalify-motion` 仅武装现有静止重资格；仿真 IPC 独占绑定；`navigation_evidence_ready()` 仅供运行中路线保留规划能力；`safety_ready()` 继续约束新任务。

- [x] 为定位未来样本、同一仿真时刻的输出切换、导航取时顺序、socket 替代路径和受阻资格分离建立失败用例。
- [x] 实现最小修复，保留原接收时间、真实回退和新鲜度失效路径。
- [x] 撤销配对覆盖确认、错误、并发刷新、当前设备移除和连接撤销。
- [x] 分批提交并完成独立复核；`f83e871` 完整 Python 回归 1340 passed / 4 skipped。
- [ ] 在最终冻结版本完成全部实际运行验收，不能由上述回归代替。

## Task 2: 定位的同窗配对

**Files:** `ros2/src/senior_robot/senior_robot/simulation_reference.py`、`tests/unit/test_simulation_reference_admission.py`。

**Interfaces:** `_observe(...)` 保留单流接纳 frontier；`_tick()` 从有界历史选择原有效期内的最新 truth 与最近 wheel；`pose()` 返回原始采样资格内的位姿。

- [ ] 测试 truth 4.46、最新 wheel 4.62、历史 wheel 4.46 与待接纳 truth 4.64 的错位情况，保持原 `.5s` 双年龄与 `.15s` 偏差限制。
- [ ] 测试选回旧 pair 后的倒序重放、重复样本续命、来源变更、时钟回退、排队过期、溢出以及 tick 之间读取过期。
- [ ] 最小实现、专项回归和独立复核后提交。

## Task 3: 运行阻塞归因

**Files:** 调试钩子和独立运行材料先保存在私有临时目录；产品只保留有界的 `adapter_loop_stall` 诊断。

**Interfaces:** `_spin_ready_callbacks(executor, clock=...) -> None` 逻辑、参数、异常和预算不变；钩子仅在明确启用的 adapter 进程中采样。

- [ ] 运行 45 秒静止快照/心跳负载，不取驾驶租约，不发送非零命令。
- [ ] 记录主线程 wall、CPU、runqueue wait 和 cProfile；标记诊断开销及 scheduler accounting 是否可用。
- [ ] 查明实际 RMW 与已装实现；只用证据决定查询或环境修正，不凭等待时长断言 DDS 锁。
- [ ] 无诊断钩子重跑，证明最终修正有效；不能以改长 `.25s` 排队期限解决。

## Task 4: 四轮物理响应诊断

**Files:** 独立、可审计的临时 prepare / physics-only launch / recorder；生产 `senior.urdf.xacro`、controllers 与安全上下文先保持不变。

**Interfaces:** 物理探针是独占测试域 108 中唯一 `/cmd_vel` 发布者；无 Agent、无云端，底盘 `.3s` watchdog 保持。

- [ ] 核对实际展开 SDF、四轮轴、碰撞和控制参数；记录引擎与插件版本和所有输入哈希。
- [ ] 比较连续 `(.099,-.264)` 和旧接收时间序列的短时脉冲，记录实际发布、ROS 源时间、轮、车体、关节及 IMU。
- [ ] 把调度迟到、观察器接收时间和模拟器应用时间分开；迟到试验判无效，不补发过期非零值。
- [ ] 根据数据选择最小控制或接触修正；不能用放大轮距、开环里程计或理想两轮替代掩盖差异。
- [ ] 若改变接触或动力学，先提出有依据的新模型和新上下文，再按下项验收；未验证前不启用。

## Task 5: 改动物理模型时的启用门槛

**Files:** 仅在 Task 4 给出实测结论后确定。旧 `tests/fixtures/fortress_lateral_impulses.json` 和历史校准必须保留，不修改其哈希冒充新证据。

- [ ] 重测六个正负侧向脉冲、正反向纵向停车、左右转向停车及角点扫掠，保存完整原始记录。
- [ ] 覆盖低速左右弧线、加减速、换向和原失败波形，使用原阈值验证正常运动与实际滑移。
- [ ] 固定车身空转、未标定低摩擦、传感器丢失、时钟暂停和底盘 watchdog 仍能停止或锁存；重资格不能清真实故障。
- [ ] 冻结模型、控制器、world、overlay 和依赖版本后才增加有独立证据的新仿真上下文。

## Task 6: 同版实际闭环与交付

**Files:** `tests/ros/test_simulation_manual.py`、`test_simulation_navigation.py`、`test_sim_topics.py`、网页运行端到端测试与验收记录。

- [ ] 实际手动前进、独立车体位移、松手零输出和持续停稳；报告实际测量而非预设 100 ms 完成指标。
- [ ] 净空三点路线顺序成功，最终 x≥0.95m、y≤−0.7m，失联撤销后不自动续跑。
- [ ] 保留旧直线路径被保守扫掠阻挡的记录；净空路线通过不能冒充该路径绕障通过。
- [x] `f83e871` RGB/Depth 传感器契约及断开后重订阅两项实际通过；最终版本仍需复核受影响部分。
- [ ] 浏览器、云端、Agent、ROS、Gazebo 同版验证地图车辆实时位置、驾驶反馈、视频与撤销配对。
- [ ] 最终完整回归、独立复核、分批提交、更新 PR 并等待 Actions；未通过项保持明确未关闭。
