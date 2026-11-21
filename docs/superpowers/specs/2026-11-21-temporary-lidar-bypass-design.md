# 临时全部雷达屏蔽设计

用户要求在独立分支全部屏蔽雷达以定位异常计算量；开发 worker 禁止连接车端或部署。代码经负责人亲审后由负责人部署，车辆由用户本人操作，负责人只做 OS 级 CPU 采样，不主动动车；本次不补测试。实现选择采用显式 `temporary_lidar_bypass`，不是根据缺扫描自动降级。

正常缺省为 `false`；本分支 `config/hardware-safety.yaml` 的 `safety_gate.ros__parameters.temporary_lidar_bypass` 为 `true`。服务读取该配置，将 `SENIOR_TEMPORARY_LIDAR_BYPASS=true` 传给全部子进程；配置、ROS 参数和环境冲突必须报错。直接启动独立地图、导航、定位入口时需显式传入同一环境变量，它们不会自动读取未传入的硬件配置。

停掉驱动、接收、适配、扫描匹配、扫描补偿、自体过滤和学习资源，并跳过控制中的扫描新鲜度、净距与距离限速。只去掉最后避障仍会保留雷达热路径，不满足计算量诊断用途。临时模式下定位、建图、导航和学习明确不可用，不伪造扫描、地图或定位；没有雷达避障。

保留急停与显式恢复、手动来源身份、释放与松杆、原始指令 deadline、唯一 `/cmd_vel` owner、最大速度、加减速与换向、wheel/IMU 输入、控制频率和非雷达时间语义。非雷达估计和可选输入计算保持原逻辑。

回退：将硬件配置开关改为 `false`，清除或同步设置环境变量为 `false`，重新由正式入口启动；不要热切换运行中的进程。正常模式恢复原雷达链。此变更未部署、未实车测试，语法和静态检查不代表运行验证。

直接独立启动时，例如 `SENIOR_TEMPORARY_LIDAR_BYPASS=true ros2 launch senior_robot mapping.launch.py`，入口会在创建消费者前拒绝；导航 launch 和定位 console 同理。缺少该环境变量的独立入口不会自动读取本分支硬件配置。

安装涉及 Python 分发 `senior-control`（新增选择器）和 `vehicle-manager`（服务），以及 ROS 包 `senior_robot`、`senior_motion_control`、`senior_motion_estimation`、`senior_perception`、`senior_navigation`、`senior_localization`；未改 C++，无需为此重编雷达驱动或 matcher。厂家底盘 include 的源码不在本仓库，不能仅凭此工作区确认其是否嵌套雷达，部署前需负责人核对实际安装的 `turn_on_wheeltec_robot.launch.py`。

可观察性使用现有 `/safety/status` 的 values：`temporary_lidar_bypass=true`、`lidar_enabled=false`；现有 IPC `state.get.navigation_block_reason` 为 `temporary_lidar_bypass`，不扩展严格状态协议。run/claim 名称仍用于 adapter 身份，但不创建 LearningService、LearningClient 或 LearningFrames，无自动学习动作。残余计算包括 wheel/IMU 接收与融合、位姿历史与可选 GPS/RGBD 估计、TF、命令守卫、限速渐变、运行报告、相机/视频及 agent/cloud 链路；不增加 CPU 埋点或脚本。

采样区分部署前停车基线、部署后稳定停车窗口，以及用户本人实际操作窗口；分别记录起止时间、版本、开关状态和操作情况，不将启动/构建阶段混入稳定窗口。部署、重启与回退在停车状态进行，用户操作结束后停车，再记录停车窗口；负责人不发送运动指令。

整机 CPU 与各进程 CPU 分开统计：整机使用采样窗口内 OS CPU 总计数和空闲计数的增量，进程使用同一窗口内进程 CPU 时间增量除以墙钟时长，注明按单核还是整机容量归一化。进程重启或 PID 变化时拆分窗口，不把 `ps` 的进程生命周期累计均值当作本次窗口负载；部署前后比较应保持停车或用户操作条件可比。此处只约定 OS 级采样口径，不增加产品埋点、工具或测试。
