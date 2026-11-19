# 第一阶段接口基准

分支 `feat/vehicle-architecture-refactor`，分叉 `main@cfbc5e4`。本阶段仅拆 Python 调用职责，不改变 ROS 图、C++ 发布者、速度算法或 40 字节包；禁止将本阶段直接部署到车辆。计划原稿见 `../superpowers/plans/2026-11-19-vehicle-architecture-refactor.md`。

## 实际目录和状态

以下路径相对仓库根目录。ROS 功能入口均位于 `ros2/src/senior_robot/senior_robot/`；它们当前仍属于同一个 ROS 包，不是假称已经拆成独立进程。

| 责任 | 入口、核心和接口 | 测试 | 状态 |
| --- | --- | --- | --- |
| 调度与生产组装 | `motion_core_output.py:MotionCoreOutput.tick`，由 `safety_node.py` 装配 | `tests/unit/test_startup_learning_chain.py` | 已实现 |
| 仲裁 | `vehicle_arbitration.py:CommandArbitration.command/revoke/release_terminal`；沿用 `MotionTarget` 和 `MotionEvent` | `tests/unit/test_task_terminal_release.py` | 已迁移 |
| 学习 | `vehicle_learning.py:LearningRuntime`；既有 `src/senior_control/motion/startup_learning.py` 和 `runtime/learning_client.py` | `tests/unit/test_startup_learning_chain.py`、`test_startup_learning_cycle.py` | 已迁移；模型和失败事实由学习所有 |
| 传感器 | `vehicle_sensor_decode.py:scan_observation/avoidance_observation`；`vehicle_sensor_access.py:SensorAccess`；既有 `motion_sensor_runtime.py` 提供运动观测 | 同上 | 已迁移；原始和补点视图分别缓存 |
| 碰撞 | `vehicle_collision.py:CollisionPrediction.clearance` | `tests/unit/test_motion_core_perception.py` | 已迁移原净距算法，无新增运动补偿 |
| 速度 | `vehicle_speed.py:SpeedCalculation.calculate` | 同上 | 已迁移原制动和斜坡算法 |
| 诊断 | `vehicle_diagnostics.py:CoreDiagnostics`；既有 `motion/diagnostic_pump.py` | 学习链和协议追踪用例 | 已迁移报告和序列化入口 |
| 输出 | `motion_output.py` → `senior_motion_msgs/src/motion_output_guard.cpp` | `tests/unit/test_native_output_protocol.py` | 原实现保留 |
| 地图/定位/导航/GPS | `vehicle_capabilities.py:VehicleCapabilities` | `tests/unit/test_vehicle_capabilities.py` | 生产组装已接入口，适配器未连接时结构化 unavailable；无运动出口 |

`vehicle_compat.py:OwnedState` 只供现有 ROS 调用者和回放使用，业务状态存放在各责任对象上。`motion_core_perception.py` 保留显式旧名称重导出。阶段三再移除兼容访问。当前 owner 的 context 是同进程调用端口，进程隔离不在阶段一完成。

## 公开契约

- `command(msg) -> bool` 消费通过既有认证/代次栅栏的 FinalCommand，保持字段、generation、source_epoch、sequence、request_id 的身份语义；`release_terminal` 仅消费已证明退场，不撤销新手动权限。
- 观测沿用 `ScanObservation`：ranges 为米，角度为弧度，sensor_to_body 为传感器到车体的二维变换；raw_capture/source_epoch/sequence、时间和变换共同确定缓存身份。学习仅消费真实端点，预测补点仅用于 avoidance 视图；合法历史数据的使用规则不在本阶段改写。
- `clearance(scans, now, filtered=...) -> DirectionalClearance | None` 返回前后净距（米）和原 deadline；None 表示不可计算，调度输出零。坐标和年龄检查仍由原算法负责。
- `calculate(requested, previous, measured, clearance, dt) -> Velocity` 输入米/秒、弧度/秒和秒，沿用已有 profile 加减速及传感器误差。
- `CapabilityResult(module, status, reason, value=None)` 是未接入能力的公开结果。生产缺省值为 `unavailable/adapter_not_connected`，导航为 `unavailable/map_pose_not_confirmed`。缺少可信地图位姿时没有可执行速度。测试固定结果不得作为生产默认值。
- 现有地图操作、`localization/Relocalizer`、`navigation/RoutePlan`、`hardware_navigation` 和 ROS Nav2 接入仍保留原调用路径；新能力门面不宣称这些旧路径已迁入，不生成虚假定位。GPS 硬件尚未接入，位置/速度融合属于阶段二。
- 输出保持既有 40 字节 ABI、原顺序、原合并绝对截止和停止屏障。本轮不提前实现 250ms 接收计时新协议。唯一标准 `/cmd_vel` 发布者仍为 native guard。

## 后续边界

本提交不实现阶段二新算法、GPS 驱动、地图 Home 交互、探索策略或新输出截止协议；不启动服务、不部署、不推送。尚未连接的地图/定位门面不能替代旧运行入口，独立进程契约及真实 ROS 图仍需阶段二、三验证。

实际新增入口树（其余既有实现位置见上表）：

```text
ros2/src/senior_robot/senior_robot/
├── motion_core_output.py       # 组装、tick、兼容端口
├── motion_core_perception.py   # 旧导入重导出
├── vehicle_arbitration.py      # 命令/退场状态所有者
├── vehicle_learning.py         # 客户端、学习及失败记录所有者
├── vehicle_sensor_access.py    # 帧身份与视图缓存所有者
├── vehicle_sensor_decode.py    # 原始/补点解码
├── vehicle_collision.py        # 净距入口
├── vehicle_speed.py            # 速度入口
├── vehicle_diagnostics.py      # 追踪、状态与报告
├── vehicle_capabilities.py     # 能力结果与既有实现适配器
└── vehicle_compat.py           # 迁移属性转发
```

既有能力可通过 `VehicleCapabilities(map_operations=..., relocalizer=..., route=...)` 装配现有 `MapOperations`、`Relocalizer`、`QualifiedRoute`。未提供实例的生产入口保持不可用；本阶段没有凭空创建这些实例所依赖的地图仓库、地图匹配器或导航执行器。`navigate` 是保守未接入入口，已有已装配路线通过 `navigation.run` 沿用原准入规则。
