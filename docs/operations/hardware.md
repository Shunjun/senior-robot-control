# 硬件部署与检查

机器人 Linux 宿主机运行 ROS 2、驱动和代理。先把
`config/hardware.example.yaml` 复制到受管配置目录，逐项校准 footprint、frame、topic
和传感器覆盖；不要把仿真与硬件配置合并。

安装 `deploy/senior-agent.service` 前创建专用 `senior-control` 用户，以及仅该用户可读的
`/etc/senior-control/agent.env`。云端 URL 必须使用云主机域名，不能填写机器人宿主机或
虚拟机的 `localhost`。设备令牌通过配对获得，撤销后必须重新配对。

运行只读检查：

```sh
uv run python tools/doctor.py /etc/senior-control/hardware.yaml
```

doctor 不启动设备、ROS 节点或下载任务，也不加载或获取模型权重。未运行 ROS 图时，
话题、TF、时钟、发布者、底盘 watchdog 和模型状态会明确显示 `not_run`。任何重复
`/cmd_vel` 发布者、非 `safety_node` 发布者，或同一 child frame 的重复 TF 发布者都必须
先修复。`command_node` 只能发 `/safety/command_in`。

上线前另行确认底盘固件 watchdog 能在命令超时后独立停车。视觉推理使用 GPU 时，宿主机
需安装与硬件匹配的驱动、容器/运行时支持和足够显存；无 GPU 时应明确采用已验证的 CPU
性能配置。模型权重由运维人员离线校验摘要后放入配置路径，镜像构建与 doctor 均不会下载。

## 升级兼容与实车运动配置

旧 `kind: real` 配置没有 `motion_profile` 和 `safety_parameters` 时，仍启动单个
ROS IPC adapter 和 Agent，保留云端连接、配对状态、设置及可用的非运动功能。
不会启动运动栈或静默换用其他控制器；IPC/state 和 CLI/status 明确报告
`real_motion_not_configured`，手动获取/驾驶和运动任务不获准。
因此升级后无需为了恢复 Agent 在线而自行创建 `hardware-motion.json`。
`kind: simulation` 仍走原 Gazebo 路径、原隔离 candidate 和时钟/domain 门。

显式配置任一运动字段后，视为选择实车控制：两字段必须配齐且文件有效；路径、JSON/YAML
或标定缺项在子进程创建前报错。helper 配置错误退出 78，已部署 service 的
`RestartPreventExitStatus=78` 阻止重启循环；Docker exec 原样传播该退出码。
ROS 子进程运行中退出/IPC 就绪超时仍属于运行故障，按 on-failure 策略处理。
空的可选导航参数不再传成无法解析的 ROS CLI 空 YAML 值。

恢复旧配置 Agent 的修复须经负责人审阅、安装到车端后，才执行常规 stop/start/status；
本次开发不远程操作用户实车服务。先前日志中的循环由新配置缺项返回 1 导致，已在代码修复，
不能把本地修复说成设备已更新。

### 文件来源和参数职责

`hardware-motion.json` 是本项目的硬件响应配置格式，由开发侧整理；厂商没有同名文件。
`config/hardware-motion.example.json` 现在只保留真正缺少物理依据的 null，不能直接驾驶。
算法/恢复/历史容量/P-only 初始软件调参已由 `real_runtime.py` 和硬件 profile 方法提供；
`config/hardware-motion.example.yaml` 只保留现场安装确认 override，不要求用户填写整份算法表。
软件阈值不作为测量误差、制动力或固件能力的证据；IMU 原始协方差不获授统计模型身份。

| 已填项目 | 来源及限制 |
| --- | --- |
| 0.674 × 0.552 m 名义包络 | 项目设计资料；沿用项目仿真模型，并非厂家规格表实车尺寸；收到实际含附件尺寸后直接填写 |
| 0.481154 m 名义轮距 | 厂家 senior_4wd_dl URDF 四轮中心坐标；有效滑移转向轮距仍需核对 |
| wheeltec_robot，/odom，/imu/data_raw，odom/base_footprint/gyro_link | 厂家 Humble 驱动声明；安装和命名空间需现场核对，原始消息不经过掩盖 writer 的 ROS relay |
| 请求最高 25/18 m/s、2 rad/s | 现有产品偏好上限；线速度会再取实测轮能力和车身包络的最小值 |
| 恢复 5 样本/0.5 s | 已批准 spec；其他 matcher/历史/调度阈值为现有软件算法策略，可调整但不冒充硬件测量 |

最少需要从真实资料或后续受控标定补齐的物理量：

| 字段组 | 单位 / 获取途径 |
| --- | --- |
| accel/decel、yaw_accel/yaw_decel、jerk/yaw_jerk、lag | m/s²、rad/s²、m/s³、rad/s³、s；对应载荷/地面条件下命令与独立车身观测的响应界，普通缓刹不能证明最坏制动下界 |
| wheel_speed_mps、assumed_body_cap、degraded_cap | m/s；实际驱动组合限值、独立全车身包络及轮速退化适用范围，不能用仿真 1.52 或固件教学限幅冒充 |
| linear_error/yaw_error、sensor_error、position_error、orientation_error_rad | m/s、rad/s、m、m、rad；对应测量/预测/感知/全时域跟踪标定 |
| drive_linear_overshoot/drive_yaw_overshoot、lateral_drift_mps/lateral_stop_s | m/s、rad/s、m/s、s；零命令响应和侧滑收敛独立界 |
| stationary_linear_mps/stationary_yaw_rps | m/s、rad/s；物理近静止容差；须大于测量误差，控制器采用扣除误差后的阈值，最终恢复再核算预测年龄 |
| evidence_basis、operating_conditions、risk_basis、brake_tracking_basis | 对应原始资料/采集记录和适用载荷、地面、坡度，不填虚构“已验证” |

Response/Controller 的物理数据只有 profile 一份；嵌套旧配置若与派生配置不同会拒绝。
ROS 中重复的年龄/测速/静止参数也必须匹配 profile，不允许两套矛盾数据通过加载。
厂家默认 odom child 为 base_footprint，共用核心为 base_link；适配器按原始时间戳
查询真实 TF，并转换 twist/pose，缺少或非平面 TF 则拒绝，不直接改 frame 标签。
GPS 可缺失，IMU 只用已知角速度并做真实 TF 旋转，不积分未校正加速度当车速。

### Humble 真实来源通道

所有新增真实 Python 输入共用 `native_receipt.cpp` + `native_receipt.py`：每个 consumer
一个原生进程，订阅 wheel/IMU/ScanMotion/core_reference、map/pose、可读 ESTOP，以及
CommandNode runtime/执行回执。rclcpp 从每个 MessageInfo 取实际 GID，和唯一配置 writer
绑定后，连同原 native steady receipt、RMW source/receive timestamps，经继承私有 socketpair
交给 Python。无文件系统监听口、无 ROS relay publisher，不把 discovery GID 填成 sample GID，
不打开 simulation-only topology fallback。

有界机制：最多 16 输入、每输入一个最新观测帧（执行回执保留至多 16 条，溢出撤销）、消息上限 20 MiB（map 最多 16,777,216 cells），
native 20 ms 检查来源、Python 10 ms 排空，传输年龄预算 250 ms、native 发送预算 100 ms。
这些是软件预算，尚非已测最坏时延。来源变化/多 writer/断连清除 pending，并通过原 admission
owner journal 撤销融合证据；core_reference 和定位/急停缓存同步失效，不能重计旧 receipt TTL。
原运行状态的 seq/revision frontier 保留，恢复不会接纳旧 JSON 回放。

部署方案只有一条：在同版 Humble ROS workspace 构建/安装 `senior_motion_msgs` 的新增
`native_receipt` target，再运行 Python 消费端；缺 target 或通道失败会撤销证据并诊断。
本机 macOS 无 /opt/ros，不能声称 C++ ROS target 已编译或运行；须负责人审阅后在目标
Linux 做构建和实测。本次尚未同步代码到车上。

### 硬件急停与 watchdog 能力

厂家 wheeltec_robot.cpp:582 把 Flag_Stop 标为“预留位”，chassis_security 是输入，
不是已证实的物理急停状态发布者。教学 C30D 停机/使能逻辑不等于目标固件能力；
当前目标独立 watchdog/急停回路仍未知，不新增所有车型必需的周期 Bool 接口。

`hardware_estop.mode` 区分：

- readable：仅硬件确有可读状态时，配置实际 topic/writer/namespace/age；缺流不可恢复，
  物理 true 优先，物理 release 后保留软件锁存，仍需显式复位且不续跑旧 authority。
- independent_unreadable：不创建 ROS 心跳订阅；必须有独立切断及释放要求控制器重启的联锁依据，
  以避免硬件释放后恢复旧速度。网页不能操作该硬件回路，也不宣称实体状态 clear。
- unknown：不伪造 clear 或虚构接口，运行诊断明确能力未知；运动/恢复不获准，基本 Agent 仍能运行。

runtime 另带 hardware_estop_observation 表达 unknown/unreadable/unavailable，不能将
软件 latch 的 confirmed_estop_status=clear 当作硬件状态证明。watchdog_mode 保留 unknown/verified，
实际超时切断机制仍须依据目标固件与独立现场验证。

真实导航仍需固定 WorkspaceGrid+SHA、map/localization 真实来源及唯一 TF 链；
共享 core 当前 laser_only，不声称已纳入相机低障碍 sweep。已有 mapping/navigation launch
保留，不导入其他 navigation 分支的 terminal_release 修改。
