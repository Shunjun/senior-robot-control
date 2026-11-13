# senior_4wd_dl 只读设计依据核查（2026-11-12）

范围：仓库 HEAD `4dd9da0`；未运行服务、连接设备或更改仓库。厂家资料根目录记为 V=`/Users/zhangshunjun/Wheel 机器人/1.WHEELTEC ROS机器人通用资料`，项目根目录记为 P=`/Users/zhangshunjun/Wheel 机器人/senior-robot-control`。以下路径均为这两个绝对根目录与相对路径的拼接；压缩包内路径用 `!` 表示。厂家模型是设计资料，并非该台实车的出厂配置或测量记录。

| 项目 | 厂家资料证据 | 项目仿真/状态及待核对 |
|---|---|---|
| 运动学、轮数 | V/7.ROS源码/ROS2源码/Humble源码/src/wheeltec_robot_urdf/urdf/senior_4wd_dl_robot.urdf:47-99、100-152、153-205、206-258 四个连续轮；V/3.STM32底层源码讲解教程/6.运动控制与PID/6_运动控制与PID.pdf 第3–4页将四驱列为车型并介绍逆运动学。四轮同侧差速/滑移转向是结合轮轴与项目控制器的推断，非实车轮胎接地标定。 | P/ros2/src/senior_sim/config/controllers.yaml:4-15 左右各两轮 diff_drive；实际转向、打滑、轮子方向核对。 |
| 轮径、轮距、轴距 | 厂家 URDF 四轮关节中心 x≈±0.214、y=0.242590/-0.238564（上述 URDF:87-98、140-151、193-204、246-257），推得轴距约 0.428 m、中心轮距约 0.481154 m；轮径未在 URDF 标量明确给出，只有 STL 网格（:63-83），须测量。 | P/ros2/src/senior_sim/urdf/senior.urdf.xacro:19-26 轮半径 0.076 m（直径 0.152）、轴距约 0.428、轮距 0.481154；controllers.yaml:8-11 同样半径及轮距。轮径为模型假设，需实车测含载有效滚动半径。 |
| 外形 | 厂家 URDF:23-44 引用 base_link.STL，未找到该模型的标量外形规格。 | P/ros2/src/senior_sim/urdf/senior.urdf.xacro:8-16 主体包络 0.674×0.54×0.15425 m，另有可选尾附件；P/config/simulation.yaml:2 footprint 0.674×0.552 m，P/config/hardware.example.yaml:1-3 为示例。需实测全附件包络、离地高。 |
| 质量/惯性 | 厂家 URDF:9-21 base 3.6181 kg、惯性对角 0.01234/0.01914/0.03006 kg·m²；:49-61 等每轮约 0.7000 kg，还有其他组件，不能将 base 质量当整车质量，CAD 值不保证含电池/负载。 | P/ros2/src/senior_sim/urdf/senior.urdf.xacro:5-6、19-20 base 30 kg、惯性 0.8/1.2/1.7，每轮 1 kg；系仿真假设。称重、重心、载荷惯性未知。 |
| 传感器位置/覆盖 | V/7.ROS源码/ROS2源码/Humble源码/src/turn_on_wheeltec_robot/config/robot_model.yaml:158-163 laser (0.15699,0.00201,0.14652)、camera (0.2316,0.00224,0.1282)、gyro 原点；厂家 URDF:299-304、352-357 给出相近位置，但 laser yaw=π，而配置为 0，须核对实际 TF 与传感器正方向。真实型号、FOV、遮挡、安装误差未知。 | P/ros2/src/senior_sim/urdf/senior.urdf.xacro:27-33 仿真激光、相机和 IMU；激光 360°、10 Hz、0.08–20 m、720 点；相机水平 1.047 rad、10 Hz，IMU 50 Hz。P/config/simulation.yaml:5 标称 2π 不代表实际有效覆盖；附件遮挡须实测。 |
| 最大速度/电机能力 | V/3.STM32底层源码讲解教程/6.运动控制与PID/6_运动控制与PID.pdf 第3页称讲解版 Drive_Motor 目标限幅 3.5 m/s；仅 C30D 讲解版上限，不是 senior_4wd_dl 电机额定速度或实车能力。未找到该型号的电机转速、减速比、功率、扭矩、电池/驱动电流及连续速度规格。 | P/ros2/src/senior_sim/urdf/senior.urdf.xacro:20-21 速度接口 joint limit 20 rad/s、effort 100（声明值，实际插件是否执行须测）；半径 0.076 对应 1.52 m/s，不能假设上限一定被执行。电机和刹车无经实测动力模型。 |
| 速度反馈及频率 | V/7.ROS源码/ROS2源码/Humble源码/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:474-487、570-587 接收 24 字节帧，2–7 为机体 vx、vy、wz；:638-660 每个有效帧积分并发布 odom，:243-245 发布 twist。V/2.运动底盘的控制与运动学解析/5.串口控制讲解/串口通信控制与反馈_2026-8-12.pdf 第3–4页给出 mm/s、角速度×1000、flag_stop 与 IMU 等；无单轮转速字段。第22–23页说明发送频率可配置，20 Hz 是示例，不是当前车速率。 | 仿真关节位置和速度反馈接口 P/ros2/src/senior_sim/urdf/senior.urdf.xacro:34-36；controllers.yaml:3、12-19 控制更新 100 Hz、位置反馈、odom 30 Hz。实际串口/odom 频率、时戳及丢包需测。 |

## STM32 的范围与停机行为

V/3.STM32底层源码讲解教程/补充说明：R550PLUS、S系列 ROS大型科研机器人用户点我.txt:1-2 明确 0.5–12 教程基于教育机器人 C30D 源码，C50C/C63A 仅整体结构类似。该资料根目录中本次找到 C30D 基础例程压缩包和教程 PDF，未找到可确认对应 senior_4wd_dl 目标生产版本的完整固件源码或烧录版本；以下均不可作为该车当前行为证明。

- C30D 讲解版加减速：V/3.STM32底层源码讲解教程/6.运动控制与PID/6_运动控制与PID.pdf 第3页仅明确全向轮车型做 XYZ 每次 ±0.01 的平滑，目标全零时乘 0.9；第1页运动任务 100 Hz。不能推定四驱路径也有斜坡或将 0.01 外推为该车的标定加速度。
- 闭环：同 PDF 第1–2、4–5页说明四编码器、各电机目标速度与实际速度、增量 PI（D 项省略）及 PWM 限幅。另 V/3.STM32底层源码讲解教程/0-3.C30D V2.0基础例程（ROS教育机器人底盘STM32主控板）/07-电机闭环控制例程/PID.zip!PID/USER/main.c:25-27、145-167、210-219 有四路编码器及各自 PI，示例参数需按电机配置；这是 C30D V2.0 教学例程，不是整车生产固件。
- 超时停车：所查资料没有足够证据确认 C30D 讲解版 ROS 串口丢命令 watchdog 及超时时间，更不能推断目标车；P/docs/operations/hardware.md:17-23 明确要求另行确认独立 watchdog。仿真 0.3 s 仅是 controller timeout（P/ros2/src/senior_sim/config/controllers.yaml:12）。
- 急停：C30D 教程 PDF 第4、6页讲电压、使能开关与软件停止位不满足时输出零；这证明条件停机逻辑的讲解，未证明独立硬件急停、失电制动、触发链、恢复机制或目标车实现。串口 PDF 第4页 flag_stop 是反馈状态，不证明安全级急停。

## 修正目标：5 km/h 的三层判断

用户更正目标为 **5 km/h = 25/18 ≈ 1.3889 m/s**，先前 5 m/s 为误解，原高速计算不适用。用户称真车满速可能超过 20 km/h（5.556 m/s），这是用户陈述，尚未由本次厂家资料、规格或实测证实，也不能因此确认 5 km/h 的安全性。

1. 软件：P/src/senior_control/protocol.py:62-64 的 vx 类型可表达 1.3889；但 P/ros2/src/senior_robot/config/simulation_motion.yaml:21-29、62-86 的 scan motion 0.5 和 safety gate 0.3 m/s，以及 P/ros2/src/senior_robot/config/nav2.yaml:31-41 导航 vx_max 0.5，均低于目标，因此现有配置不具备该目标端到端运行资格。厂家 V/7.ROS源码/ROS2源码/Humble源码/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:97-130 将速度×1000 转成 short，约 1388 的数值可表示，并不证明当前固件执行；C30D 讲解版的 3.5 m/s 限值也不是当前实车证据。
2. 仿真物理：以 P/ros2/src/senior_sim/urdf/senior.urdf.xacro:19-21 的假设半径 0.076 m，直线无滑动轮速约 **18.275 rad/s = 174.51 rpm**，低于该 URDF 声明的 20 rad/s（对应 1.52 m/s = 5.472 km/h），余量较小；转弯时外侧轮速可能更高，须考虑角速度及不同有效轮半径。关节限制实际是否执行、跟踪是否达标以及摩擦与电机动力学均未验证，1 ms 物理步长（P/ros2/src/senior_sim/worlds/motion_qualification.sdf:4-5）不证明真实性。
3. 实车：能否达到及安全运行尚不可判定；需要实测含载轮径、驱动/电池额定曲线、负载与地面条件、传感器时延/遮挡、独立停车机制以及对应固件配置。用户对满速的陈述可作为待核实线索。

旧配置仅作量纲计算：以 P/ros2/src/senior_robot/config/simulation_motion.yaml:74-76 的假设 a=0.3 m/s²、反应时间 0.2 s 和 P/src/senior_control/safety/guard.py:85-87 的公式，5 km/h 对应反应距离约 0.278 m、理想恒减速距离约 3.215 m，合计 **3.493 m**（不含其他余量、采样及额外时延）。此值既非实测也非固定停车距离或安全保证；不同速度、坡度、负载、附着和响应会改变它。

## 激光真实运动估计与在线响应

真实车身速度以静态环境的激光相对运动估计为主要依据，轮速/轮式里程计仅作为参考、执行反馈和一致性检查，不作为车身真实速度的替代。P/ros2/src/senior_scan_matcher/src/scan_motion_node.cpp:24-58、222-295 已将合格扫描匹配成带源时间区间、机体位移、协方差和质量的 `/motion/scan`；P/src/senior_control/safety/motion_estimator.py:157-214 根据扫描位移/时间形成**区间平均**速度并结合 IMU、轮里程计作时间与一致性检查，P/ros2/src/senior_robot/senior_robot/motion_fusion.py:155-162 负责接入。它不是即时真值，不能只依赖低频轮速来补上失效的激光估计。

需要验证：激光扫描周期和时间戳语义、逐束采样期间运动畸变；节点目前要求 `time_increment == 0`，非零会拒绝（scan_motion_node.cpp:222-229），实际雷达驱动是否满足以及预处理能否可靠去畸变必须核实。检查采集、传输、滤波、TF、匹配、融合到制动输出的端到端年龄与抖动、丢帧和时钟同步；0.8 s 允许年龄、0.4 s 最大区间及 0.25 s 对齐等待是 P/ros2/src/senior_robot/config/simulation_motion.yaml:23-49 的现有仿真阈值而非合格的实车时延，5 km/h 下 0.8 s 已移动约 1.11 m。动态障碍占主导、长直走廊、空旷或重复纹理、遮挡、稀疏反射、外参漂移及运动过大均需验证退化和误匹配；P/ros2/src/senior_scan_matcher/src/scan_matcher.cpp:120-125、269-285 有可观测性、残差和内点筛查，不能证明所有错误匹配均可检测。当前速度界限 0.5 m/s（scan_motion_node.cpp:267-271，simulation_motion.yaml:28）需基于新目标重新论证而非盲目放宽；质量不足、延迟过大或独立估计失效时需进入预先定义的保守降速/停车状态，不能使用过期数据假装闭环。

可以从日常**普通**加速、减速、零命令响应和激光速度变化，结合命令时间与数据置信区间在线估计当前条件下的响应、时延与变化趋势；需处理区间平均、噪声、坡度、载荷和历史失效，记录证据年龄、覆盖条件和不确定性。普通减速观测受控制策略、动力和地面共同影响，**不能辨识未发生的极限制动能力，更不能当最坏条件下可保证的减速度下界**；控制不能把短期观测或缺少数据时的乐观估计当安全保证。停车范围应随当前独立速度、估计时延及经过论证的保守响应界动态计算，缺少有效界限则降低允许速度或拒绝运行；本阶段不主动进行极限制动实验。

## 碰撞几何待确认提案

将已有矩形车体 footprint 的长和宽各乘 1.05，围绕其中心缩放，仅为待确认的约 5% 几何膨胀提案。按 P/config/simulation.yaml:2 的 0.674×0.552 m，结果为 0.7077×0.5796 m，即每侧增加 0.01685 m、0.0138 m；P/config/hardware.example.yaml:3 的示例宽 0.54 m 则得到 0.567 m。需先核对实车含附件形状及旋转中心，明确何处应用，避免双重膨胀；该几何余量不能替代感知误差、运动时延与动态制动范围。

## 仿真可验证范围、最小修改及验收建议

现有系统可观察速度命令、轮关节位置产生的轮式里程计和独立 ground truth：P/ros2/src/senior_sim/urdf/senior.urdf.xacro:34-41、P/ros2/src/senior_sim/config/controllers.yaml:8-19、P/ros2/src/senior_sim/config/bridge.yaml:11；P/ros2/src/senior_robot/senior_robot/motion_fusion.py:156-158 接收 wheel odom、命令，P/ros2/src/senior_robot/senior_robot/safety_node.py:1006-1026 的 drive_feedback 描述输出命令，不能当轮速或制动力测量。可验证应用层反馈融合、超时置零及在当前假设模型中的停车响应；不能验证真实 STM32 PI、斜坡、watchdog、驱动饱和、物理制动距离或急停。

最小设计改动建议：保持现有安全链，在仿真驱动与轮关节之间增加可配置电机/执行器动态（速度/加减速或扭矩限幅、滞后、受控停止及超时），显式公开四轮状态和时间戳，并能注入编码器偏差、打滑、丢帧及命令断流；参数分为占位与实测标定，勿改称硬件指标。用 `/ground_truth/odom` 对比 `/motion/scan`、`/wheel/odom` 与 `/cmd_vel`，逐项验收阶跃/斜坡跟踪误差、过冲、响应时间，正反向及转弯、载荷/摩擦变化，零指令/断流后停止时间与位移、反馈陈旧和故障触发，以及碰撞前的余量；阈值由已确认的场景与标定值事先约定，记录仿真时间和采样频率。现有 0.3 m/s² 和 0.2 s 均为仿真假设，不是实车制动能力；应在仿真中验证扫描估计延迟和退化、在线响应估计的置信与过期处理，以及动态停车范围。

阻碍：未取得目标车型真实配置、匹配生产固件和现场测量，厂家反馈默认频率与制动能力未知；本阶段未进行运行或实车验证。
