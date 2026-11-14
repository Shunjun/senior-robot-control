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

## 同一运动核心的实车启动配置

`config.json` 的 `kind: simulation` 仍走现有 Gazebo 启动，原隔离候选的
`use_sim_time=true`、localhost/domain 门保留；`kind: real` 不启动 Gazebo，
启动共享扫描 receipt/filter/matcher、motion_fusion、safety_gate 和单个
agent_adapter/CommandNode，随后启动 Agent。产品 SettingsService 两种模式均创建，
五项驾驶偏好和日志偏好不再依赖 `SENIOR_SIMULATION_SETTINGS`。

实车 `config.json` 参照 `config/vehicle-real.example.json`，新增字段：

- `motion_profile`：按 `config/hardware-motion.schema.json` 填写的硬件响应 JSON。
  `config/hardware-motion.example.json` 的 null 是缺项，启动会拒绝，不能直接使用。
- `safety_parameters`：按 `config/hardware-motion.example.yaml` 填写的各节点 ROS 参数。
  每个节点必须 `use_sim_time: false`。参数路径可相对 control-dir；容器内在该目录解析。
- 可选的一对 `navigation_reference` / `navigation_parameters`：固定地图与定位来源配置，
  以及真实 Nav2 参数。缺省为手动模式，导航明确 unavailable。

实车服务在创建 ROS 子进程前校验配置，`doctor` 也检查同一加载器；容器模式在容器内检查，
不在宿主机猜测容器路径。启动时移除继承的 `SENIOR_SIMULATION_*` 和
`SENIOR_MOTION_CORE_*`，不借仿真环境变量解锁。停止时先撤销 Agent/CommandNode，
再关闭共享栈；必要节点退出会让 launch 整体退出，由监督器关闭其他子进程。

驱动仍由现场启动管理：底盘订阅唯一 `/cmd_vel`，禁止同时启动厂家遥控、Nav2 默认
cmd_vel 输出或另一个 safety_gate。资料中的 `wheeltec_robot` 直接发布 `/odom` 和
`/imu/data_raw`；融合按配置 remap 消费原消息，不增加会掩盖 DDS 身份的 relay。
实际驱动命名空间、雷达 topic、base/odom/IMU/laser frame 和 TF 必须核对。
IMU 使用已核实板载角速度接口；不把未校正加速度积分成独立车速。GPS 非启动依赖。
真实消息仍要求每样本 DDS GID；Humble/RMW 缺少该元数据时会拒绝并诊断，
不会借用仅隔离仿真允许的 topology-only fallback。

当前共享 T4 输出适配只消费激光自由空间，因此实车配置显式限定 `depth_policy: laser_only`，
不能声称已具备相机低障碍防护。原深度节点/仿真 RGBD 环境保留；把深度几何纳入共享
T4 sweep 需要后续适配，不能仅启动 depth_ingress 就宣称完成。

### 尚缺的实际依据

厂家资料与 `docs/reviews/2026-11-12-vehicle-hardware-audit.md` 只能提供车型、
串口消息与设计几何线索；0.3 m/s²、0.2 s、1.52 m/s 等仿真假设均未填入硬件模型。
最少还需要：

- 含附件车身包络、有效轮距和驱动组合限值；安装外参/遮挡、扫描采样时序、来源与年龄预算。
- 在明确载荷/地面/坡度下的制动下界、加速度上界、响应滞后、jerk、零命令跟踪过冲、
  全车身速度包络与侧滑/横向收敛界；模型有效期与证据出处。
- 测速/姿态/位置误差和预测误差界，正常及轮速+IMU退化模式限速、恢复与静止判据。
- 独立底盘 watchdog 依据与硬件急停状态来源。当前接入契约为周期 `std_msgs/Bool`，
  true 表示物理急停有效，需配置真实 publisher/topic/命名空间与最大年龄；
  缺流/来源不符拒绝运动，物理释放后仍需显式软件复位。厂家串口 flag_stop 不被自动
  改称独立急停，当前资料没有可直接填入的这个发布者。

这些缺项不影响代码装配，但目前不能称为已接通可驾驶实车，也没有硬件验收结论。

### 真实地图与定位依赖

导航使用相同任务入口、execution-specific Nav2 controller 和 CommandNode 来源 fence，
只有 safety_gate 输出 `/cmd_vel`。`navigation_parameters` 传给 planner 与专用 controller，
各自使用 `use_sim_time=false`，odom topic 透传自硬件模型。外部需要运行已标定的
map_server/定位（例如 AMCL）以及唯一的 map→odom、odom→base_link、传感器 TF；
现有 `mapping.launch.py` 可独立启动 SLAM，不能把持续变动或未归档的 `/map` 自动
认作已有任务的地图身份。

`navigation_reference` 的 schema 为 `config/hardware-navigation.schema.json`。
其 `grid_path` 是显式带 `map_ref`、`grid_id`、width/height/resolution/origin/cells 的
WorkspaceGrid JSON，`grid_sha256` 固定该文件，另指定已归档锚点、map 与定位发布者、
定位 topic/协方差/时效预算。该配置只消费来源唯一且内容与固定地图一致的 `/map`，
再接纳同地图坐标中的真实定位；不发布地图或 TF，不使用 `/ground_truth/odom`。
`display_only` 地图导出不能作为导航资格。变图、缺定位、错误来源或过期都会撤销资格，
原任务须走既有明确恢复/重规划入口。
