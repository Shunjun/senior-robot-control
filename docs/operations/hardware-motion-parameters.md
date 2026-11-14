# hardware-motion.json 参数说明

对应 `config/hardware-motion.json`，由用户要求先用假设值填写。
“软件选定”不是硬件测量，“资料名义值”不是实车安装确认。
当前没有任何一项猜填的物理响应/误差数值被确认为这台车的实测保证。

| 变量 | 当前值 | 含义与单位 | 状态/依据 |
| --- | --- | --- | --- |
| schema_version | 1 | 本项目配置格式版本 | 软件确定 |
| profile_id | wheeltec-senior-4wd-dl-assumed-v1 | 本份配置的识别名，不是车辆序列号 | 软件指定 |
| evidence_basis | ASSUMED… | 参数证据来源说明；目前明确写无物理测量 | 已确定是草稿说明 |
| operating_conditions | 室内、干燥、平地、名义空载 | 声明适用工况；不会凭字符串自动检测地面/载荷 | 假设，现场未确认 |
| simulation_only | false | 文件面向实车配置，不是 Gazebo 专用 | 软件确定，不代表已验证 |
| unvalidated | true | 尚未完成实际标定/验收 | 确定；加载器拒绝用此草稿驱动车辆 |
| max_age | 0.5 | 证据最大接纳年龄，s；不是实测传感延迟 | 软件预算，实际流仍须核对 |
| accel | 0.2 | 规划/响应模型采用的线加速度上限，m/s² | 猜填 |
| decel | 0.1 | 停车计算采用的可保证线减速度下界，m/s² | 猜填，没有保证证据 |
| jerk | 0.4 | 线加速度变化率限制，m/s³ | 猜填，需验证实际响应可实现 |
| yaw_accel | 0.3 | 角加速度上限，rad/s² | 猜填 |
| yaw_decel | 0.15 | 停车计算采用的角减速度下界，rad/s² | 猜填 |
| yaw_jerk | 0.6 | 角加速度变化率限制，rad/s³ | 猜填 |
| lag | 0.5 | 命令到执行响应的延迟界，s | 猜填 |
| requested_linear_mps | 1.388888… | 请求线速度上限，m/s，即 5 km/h；还会受实车能力约束 | 已确定的产品目标，不证明车能安全达到 |
| degraded_cap | 0.15 | 独立测速退化、使用轮速+IMU时的线速度上限，m/s | 猜填 |
| yaw_cap | 1.0 | 角速度请求上限，rad/s | 软件选定；安全执行能力未验证 |
| length | 0.674 | 以 base_link 为中心的包络矩形长度，m | 项目名义模型；实际附件待核对 |
| width | 0.552 | 包络矩形宽度，m | 项目名义模型；实际附件待核对 |
| track_width_m | 0.481154 | 左右轮中心几何轮距，m | 厂家 URDF 坐标推算；有效滑移转向轮距未测 |
| wheel_speed_mps | 1.5 | 单侧轮组等效轮缘线速度限值，m/s；不是 rad/s | 猜填，不能用作电机规格 |
| assumed_body_cap | 1.5 | 独立全车身线速度包络，m/s；不是用轮速冒充真速 | 猜填 |
| linear_error | 0.02 | 车身线速度估计的基础误差界，m/s | 猜填 |
| yaw_error | 0.02 | 角速度估计的基础误差界，rad/s | 猜填；不是航向角误差 |
| sensor_error | 0.05 | 激光几何误差/膨胀量，m | 猜填 |
| position_error | 0.1 | 预测时域内跟踪/定位的位置误差界，m | 猜填 |
| orientation_error_rad | 0.05 | 预测时域内姿态/航向角误差界，rad | 猜填，约 2.86° |
| stationary_linear_mps | 0.05 | 可接受的物理近静止线速度容差，m/s | 猜填；控制器扣除基础误差，恢复时再核算年龄误差 |
| stationary_yaw_rps | 0.05 | 可接受的物理近静止角速度容差，rad/s | 猜填 |
| model_seconds | 3600 | 启动后模型使用期限，s，即一小时 | 软件期限，不代表物理模型已有效一小时 |
| horizon_s | 12 | 停车搜索/约束预测的最长时域，s | 软件搜索预算；物理包络仍须覆盖该时域 |
| drive_linear_overshoot | 0.1 | 制动/零命令跟踪允许的线速度过冲界，m/s | 猜填；不是停车距离 |
| drive_yaw_overshoot | 0.15 | 制动/零命令跟踪允许的角速度过冲界，rad/s | 猜填 |
| lateral_drift_mps | 0.03 | 横向滑移速度界，m/s | 猜填，不假设四驱一定没有侧滑 |
| lateral_stop_s | 1.0 | 横向滑移收敛/停止的时间界，s | 猜填 |
| reaction_linear_accel | 0.3 | 反应窗口内可能继续发生的线加速度上界，m/s² | 猜填；必须覆盖 accel |
| reaction_yaw_accel | 0.5 | 反应窗口内可能继续发生的角加速度上界，rad/s² | 猜填；必须覆盖 yaw_accel |
| watchdog_mode | unknown | 对目标底盘失联独立停车机制的已知程度 | 确定为尚未核实，不等于硬件一定没有 |
| watchdog_basis | 未取得目标固件证据 | watchdog 能力的来源说明 | 当前事实记录 |
| hardware_estop.mode | unknown | 物理急停能力：可读、独立不可读、未知三种之一 | 当前尚未核实；不虚构周期 Bool 来源 |
| hardware_estop.basis | 未取得状态/联锁证据 | 急停回路、状态和释放联锁的依据 | 当前事实记录 |
| perception_basis | 原始激光，无回波未知，安装未确认 | 感知输入/无回波处理/安装状态说明 | 软件策略与当前确认状态；不是雷达型号精度证据 |
| risk_basis | ASSUMED… | 风险包络的来源说明 | 明确为假设，非资格证明 |
| brake_tracking_basis | ASSUMED… | 零命令/制动跟踪模型的来源说明 | 明确为假设，未实测 |
| wheel_topic | /odom | 轮式里程计输入话题 | 厂家默认，车端实际配置尚未核对 |
| imu_topic | /imu/data_raw | 原始 IMU 输入话题 | 厂家默认，车端实际配置尚未核对 |
| wheel_publisher | wheeltec_robot | 唯一允许提供轮式里程计的节点名 | 厂家驱动声明，运行图尚未核对 |
| imu_publisher | wheeltec_robot | 唯一允许提供原始 IMU 的节点名 | 厂家驱动声明，运行图尚未核对 |
| source_namespace | / | 上述来源节点的 ROS 命名空间 | 厂家默认，运行图尚未核对 |

控制器线速度实际 cap 为请求上限、wheel_speed_mps、assumed_body_cap 的最小值；
左右轮组合仍会进一步约束转向请求。规划矩形再在统一 Footprint 处乘 1.05，
不能在配置中重复加同一几何膨胀。

本份 JSON 有完整示例数值，但 unvalidated=true 和未知硬件能力
都是真实状态。它不是可驾驶配置；不能仅改验证标志就当作实车测量完成。
旧配置不选择运动模型时，Agent/IPC 仍可启动，缺模型不再阻断整个服务。

配套的 ROS 参数草稿为 `config/hardware-safety.yaml`，对应 config.json 的
`safety_parameters`；当前安装确认仍为 false，不会伪装已校准。
