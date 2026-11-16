# 全系统控制时间：具体producer契约与最小补丁触点（待亲审）

本轮只读调查厂家资料，未改厂家原件，未新增全域产品实现，未启动ROS/车辆。已通过的第8及原导航第9项产品版本保持；本文件承接`2026-11-16-control-clock-system-audit.md`，按负责人最新意见修正硬件边界和学习职责。

## 已找到的来源及版本边界

厂家资料根为`/Users/zhangshunjun/Wheel 机器人/1.WHEELTEC ROS机器人通用资料/7.ROS源码/ROS2源码/Humble源码/src`，不是本功能子仓库。已逐文件锁定43份厂家源码/头文件/构建声明/选择配置，并记录6份本地仿真接线文件，见`2026-11-16-control-clock-producer-sources.json`。资料包名版本不能证明车辆安装版本；没有据此自动升级、启动、部署或复制整套源码。

| 来源 | 资料中包版本及声明 | 关键生产文件与原始SHA256 |
| --- | --- | --- |
| 底盘wheel/IMU | turn_on_wheeltec_robot 0.0.0；package.xml license为TODO，保留原声明 | `turn_on_wheeltec_robot/src/wheeltec_robot.cpp`：`c96da6f956f3568f0ba2fe3de2c151c41127fe5a0b3f386f1213c5b3609ee55d` |
| 雷达 | lslidar_driver 5.1.1，Apache-2.0 | `wheeltec_lidar_ros2/lslidar_ros2/lslidar_driver/src/input.cpp`：`7c1d770beb33ce346cfac82e3976931fdbc43af77bdf589771a7b6fb27b40bc8`；各型号及串口文件逐份见JSON |
| Astra/OpenNI及UVC | astra_camera 1.1.0；package.xml为TODO，源文件保留Orbbec proprietary notice | `ros2_astra_camera-master/astra_camera/src/ob_camera_node.cpp`：`903c5e1630d91b391bcf207497c4639e9782f6021b3f0ba3b652140699d25df6` |
| 可选USB RGB相机 | usb_cam 0.6.1，BSD | `usb_cam-ros2/src/usb_cam.cpp`与node/header逐份见JSON |
| 定位 | 厂家navigation2-humble/nav2_amcl 1.1.20，LGPL-2.1-or-later | `navigation2-humble/nav2_amcl/src/amcl_node.cpp`：`29a6402bf643fc8ea0d2dcea29619d4cbdcaf7014c811332ed4e86e593d3f1af` |
| 可选GPS/独立AHRS | nmea_navsat_driver 2.0.0，BSD；fdilink_ahrs 0.0.0，license TODO | `nmea_serial_driver.py`、NMEA `driver.py`、`ahrs_driver.cpp`，逐份见JSON；当前wheel/IMU生效配置仍指wheeltec_robot，不把这些可选源列全车依赖 |
| 仿真实际接线 | senior_sim 0.1.0；Fortress、ros_gz_bridge、gz_ros2_control、diff_drive_controller | 本仓`senior.urdf.xacro`、`bridge.yaml`、`simulation.launch.py`等散列见JSON；总目录实际文件检索未发现这些外部插件/bridge实现源码，仅找到调用与声明；未假称厂家Gazebo Classic资料就是本项目Fortress运行实现 |

厂家`turn_on_wheeltec_robot/config/wheeltec_param.yaml:29,50`示例选择`ls_M10P_uart`和`astra_pro`；这是资料默认，不等于现场型号；现场日志明确N10Plus UART 460800及camera_mode astra。后续overlay按明确选择的producer打最小补丁，不能把所有雷达型号、相机、GPS、AMCL都设成全车启动必备。

## 时间字段：不新增一套重复身份前沿

建议新增一个共用的**源时间证明**嵌套字段，随typed样本一起传递。最终命名在亲审后冻结，所有消费者共用同一表示。

- `clock_domain`：同车Linux boot_id所标识的CLOCK_MONOTONIC域；源进程重启仍在同host域，不重复引入receiver wall epoch。
- `source_epoch`与`sequence`：复用现有源epoch/顺序语义，由实际driver有效包/帧推进，贯通到RawCaptureHeader/Origin，不再由每层接收器重造一个“生产序号”。现有bridge boot/generation仍只用于桥自身重启与原生身份鉴别，不再用于系统校时换代。
- `capture_start/end_steady_ns`：最早可观测有效包/帧的驱动读取单调前沿，或有明确来源依据的采集前沿；所有后续队列保留，不在DDS订阅后重盖。点样本start=end；scan或视觉差分保留各原始端点。
- `time_basis`：描述`driver_read`、`device_tick`或`sim_step`事实，**不是必须满足device_tick的新资格门**。`driver_read`明确设备/内核前缓冲延迟未被此时间覆盖，不能宣传为物理采集真时。可选device tick/frame index原样保留作同设备配对及原始区间，不由host wall推导。
- 原始ROS `header.stamp`与外部测量域/帧identity另外保留用于TF/地图/对外接口；真实的producer重启/流重置才换源epoch，接收机器校时不换。派生样本保存所有根前沿，deadline取原始根的既有限期最小值。

不创建独立导航限速/传感器新超时常数，不为未知硬件前延迟编造误差预算。真实steady异常、实际源重启、旧sequence、过期根仍按原语义收束；选用driver_read不应使整车因缺硬件tick被禁止启动。

## 具体生产触点及缓冲调查

### wheeltec串口底盘：当前默认wheel与IMU来源

`wheeltec_robot.cpp:480`逐字节调用`Stm32_Serial.read()`；有效24字节基础包CRC通过才在630附近Control循环积分/发布。当前循环在读包前用`this->now()`算Sampling_Time，Publish_ImuSensor/Publish_Odom约200/235分别再次盖ROS时间，同一硬件包并无共同生产前沿。

最小补丁触点为wheeltec_robot.cpp、wheeltec_robot.h及该包CMake/package.xml：

1. 在串口**实际返回非空字节**时记录CLOCK_MONOTONIC；解析器为正在组装的包保留first/last-read前沿。先检查read实际返回长度，不能超时读到0字节后把旧数组内容视作新生产。
2. 有效基础包校验成功时递增一次共同sequence，保存一份不可变时间证明；同一包派生的wheel/IMU沿用它，不分别在publish时重盖。
3. Odom位置积分的dt改为相邻有效包的驱动read steady差；外部Odometry/Imu原始ROS stamp继续供TF/其他ROS消费者使用。Quaternion_Solution.cpp用固定20Hz模型参数（SAMPLING_FREQ），不是读wall时钟；此处不趁机修改姿态算法/增益或发明采样率。
4. 控制链typed的wheel/IMU envelope携带该时间证明，原标准话题仅保留外部ROS接口，不作为新控制路径的旧wall兼容分支。proof与对应payload同消息，不靠wall stamp sidecar配对。

**已证缓冲边界**：初始化803–806设置串口read timeout=2000ms，并仅在open后flushInput一次；未见运行中丢弃旧wheel/IMU帧或FIFO年龄记录。其serial_ros2 unix.cc使用CLOCK_MONOTONIC做读超时，**2000ms是等待上界，不是已在内核/USB/STM32缓冲中数据的年龄上界**。115200 baud也不是积压时间上界。主循环spin_some/调度可拖慢读包，因此前缓冲延迟目前不能从这份源码定量界定。

建议只在driver读取边界按当时可读批次解析并保留最新完整wheel/IMU有效包（充电/超声状态仍正常解析，不用无区别flush吞其他协议）；同一批原始read时间保留，丢旧/批次处理不能给旧包新时间。是否实施这一最小丢旧改动须随本触点审阅，不改变下位机协议。即使这样仍不声称设备端缓冲已定量有界；如资料/设备不提供其年龄，只报告这一具体限制，不据此默认所有源禁用。

### lslidar：串口/UDP到scan的组合证据

已证触点：input.cpp:113–129的poll/recvfrom，lsiosr.cpp:173–240的read/getSerialData，x10_driver.cpp:431起getPacket及580–609组圈/异步发布；CX/CH/LS型号也有各自poll和发布函数，见JSON锁定文件。

最小设计：在成功recvfrom或串口非空read边界记录driver_read steady，透传到packet解析、完整scan起止端点及最终typed scan发布。现场日志已明确`ls_N10Plus_uart`、`N10Plus`、`460800 baud`，因此实现以`lslidar_x10_driver.cpp`中N10Plus实际分支为首要覆盖；厂家M10P默认不代表现场。共享X10解析路径保持型号算法和协议，只追加读包时间事实与单份最新发布快照。型号自带GPS/PTP/设备时刻原样作为外部测量标签，点间dt可用有明确域/epoch的源tick；不能拿校时后的host now重新推导scan_age。

**已证缓冲边界**：UDP为nonblocking+2000ms poll，未配置基于到达steady的内核包时间或读旧批次策略；串口也用poll/read，无包年龄元数据。ThreadPool.h使用无容量上限std::queue，x10组圈后enqueue捕获this发布。故“2000ms”与线程数均不能证明数据最大年龄；异步排队还可能使payload与可变时间成员错配。

最小发布补丁需要把完整scan及其起止时间**同一不可变快照**传给任务，不让closure到运行时再读当前this时间；发布前按原始steady deadline丢过期结果。可用同源最新scan替换尚未处理旧scan，而非重写线程池/增加新进程架构。读取前的设备/内核缓冲仍标未知；没有测得来源就不声称已覆盖它。

### Astra/OpenNI与UVC；可选USB

OpenNI `ob_camera_node.cpp:101–107` readFrame成功后到onNewFrameCallback；505–549先resize/copy再以node.now盖header，CameraInfo跟同一ROS stamp。SDK随包OpenNI.h:505–534明确有getTimestamp与getFrameIndex：时间单位微秒，开启depth-color同步时匹配frame index有契约，未开启不能假设RGB/depth index相等。

最小触点：pollFrame在readFrame成功处保存driver-read steady并立即读取device timestamp/index，作为参数随该帧传入onNewFrameCallback；图像与CameraInfo共享同一帧proof，resize/copy/队列均不改时间。只有证明了SDK设备tick→hostmono关系才标device_tick，否则标driver_read，device tick继续用于该源内匹配；不通过`host_now-device_tick`随时重锚伪造物理采集时刻。

UVC `uvc_camera_driver.cpp:332` callback入口是本仓资料最早可观测边界，转换/裁切之后389又盖now；在入口保存一次proof并传到最终Image/CameraInfo，禁止ROI分支刷新控制时间。`uvc_frame_t`库字段或OpenNI SDK内部缓冲策略须以实际链接库/版本核实；当前驱动没有显式SDK队列容量/旧帧年龄界，不能从30FPS或2秒wait推出前延迟上界。

可选usb_cam-ros2在usb_cam.cpp:126、156的VIDIOC_DQBUF拿到V4L2 buffer，已有timestamp/sequence可核验。应检查V4L2 timestamp flag后才能把buffer时间认作CLOCK_MONOTONIC；否则在DQBUF成功边界标driver_read，并保留原timestamp标签。REQBUFS请求4个buffer，**容量4不是每帧最大年龄界**。不改相机格式/检测算法；只有选用该USB源时才需要其patch，不影响无相机的laser-only行驶。

### 定位AMCL：派生结果不能在其DDS入口重新开始寿命

已找到厂家nav2_amcl1.1.20：amcl_node.cpp:618的laserReceived、937的pose header沿用输入scan stamp、961的最终发布。它不是硬件采集者，不能在pose发布时盖mono使旧scan复活。

建议与固定Nav21.1.20相同commit的受控overlay增加最小AMCL typed输入/输出适配：输入scan连同原始driver proof进入TF message filter/laserReceived参数，运算及publishPose沿用相同root。必须把proof与scan对象显式一同传递，不按可能重复/回拨的ROS stamp从另一路缓存猜配。原TF/pose标准ROS标签保留；typed localization结果指明原scan根及同域steady有效期，HardwareReference使用该根，不以处理完成时间新建年龄。初始位姿人工重设不是新鲜scan证据，不借此刷新旧测量。

只在导航定位使用AMCL时需要此适配；不把AMCL变成手动/无定位模式的全车必备。若现场选择其他定位器，按其实际输入根与版本追加同等适配，不能拿这份AMCL资料冒充其实现。

### 可选GPS/AHRS及仿真

已只读找到NMEA nmea_serial_driver.py:54–61的GPS.readline→add_sentence，和AHRS ahrs_driver.cpp:121起串口读包、448发布ROS stamp及checkSN。可选GPS无可靠host采集tick时在readline成功边界记录driver_read；GNSS UTC/time_reference保留，不能因host NTP改变源epoch。AHRS已有设备SN须保留其wrap/源重启语义，不为每层增加多个源前沿。两者都不是当前wheeltec默认路径必须安装的来源。

实际Fortress组合来自本仓senior_sim：/scan、IMU、RGBD、ground_truth由Ignition系统生产后经ros_gz_bridge转ROS，wheel/odom来自gz_ros2_control+diff_drive_controller；本机总项目检索未找到这些真实插件实现，不能把simulation_camera的DDS回调当源头改盖mono。

拟最小方案是在**实际仿真sensor/physics输出生成边界**记录hostmono与world epoch/step，并同payload经bridge透明携带到DDS；已有仿真测量sim stamp仍供TF/源间对齐。控制timeout用hoststeady，仿真暂停会使没有新根的证据自然变旧；真实world reset由world/source epoch表征，不由接收系统时间回拨冒充。不可仅在parameter_bridge订阅回调把已积压Gazebo消息盖当前时间，也不可把/clock到达时刻当更早sensor生成时刻。

本仓只声明Fortress、ros_gz_bridge/gz_ros2_control等包，未锁定其源码commit/安装版本；下一步可只读查询实际构建/安装清单并取对应官方源码作最小patch，不要求用户重新寻找已有厂家文件，不预设外部插件API/patch触点。实际源码和可编译补丁边界未核实前，仿真项明确未完成，不用额外全车门替代缺证；硬件原始driver路径的实施不依赖可选仿真/相机/GPS同时具备。

## 外部补丁可复现交付设计

- 厂家资料只读、以JSON源文件SHA256锁定快照；准备脚本只复制选定包及必要声明依赖到用户给定的新overlay目录，不写资料目录，不复制整个Humble/Navigation2。
- 每份patch声明源文件SHA、输出SHA及精确上下文，`patch --fuzz=0`，源不同明确停止该patch。Nav2原controller与MPPI继续用已固定官方1.1.20 commit；厂家local AMCL同版但未证明每字节等同官方，不能混用散列。
- 仅为选中的数据链添加typed时间证明；标准ROS接口为TF/外部消费者保留，控制入口同版迁移后不保留旧wall资格回退，不靠旁路sidecar临时补时。
- 编译/安装说明分别标资料版本、实际目标包/ABI前置和未知设备前延迟；不自动升级部署。厂家原license/header保留，不假称这些全部Apache。
- 本轮仅设计，尚未生成或应用上述厂家patch，不声称已通过其构建。原导航patch独立证据仍有效；新增全域待触点和字段亲审后集中实施与验证。

## 与学习worker冻结的职责与接口

已与ctx_ebb465ebe313直接协调，对方回复当前仅设计、共同hunk待亲审后交接：

- 保留`advance(estimate,wheel,*,now=steady)`与`motion_emitted(now=steady)`接口；computed_steady用于推进，原始capture_start/end_steady用于pose/history及validation_after_steady。
- 我方提供显式capture identity+measurement steady视图，保留scan.stamp真实ROS几何标签，不在scan.stamp上偷换时钟；真实capture/pose lineage改变时分段，未知段不插值。
- 学习worker拥有3秒输出首帧t0、t0+3秒窗口、提交成功后的motion_emitted、durable动作预消耗及结果恢复；elapsed仅是本地输出提交时长，不等于位移/物理ACK。
- 我方不改3秒动作/恢复生命周期，不保留已撤销精确15cm、偏航/反向学习中断或30秒预算；分段不会重置动作预算。self_learning.py冻结不改，第7项几何hunk保持。

## 亲审重点与定向验证

请亲审driver_read事实边界、同包wheel/IMU共同前沿、无新增全车缺tick门、原始proof同payload贯通、设备前缓冲未知的标记，以及资料默认M10P/Astra选择与实际现场未核实边界。拟验证同一字节/帧在驱动read后、DDS后、worker后分别排队都不刷新deadline；前缓冲未知如实报告；系统校时不改源身份/积分/学习动作预算，真实boot/SN/reset/过期仍拒绝。

本轮只读扫描和文档修订，没有为文档变化重跑335项导航测试；没有把未实现外部producer或扩大的第9项报告完成。
