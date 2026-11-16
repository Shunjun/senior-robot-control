# 第9项同版时间协议与外部 overlay


`senior_motion_msgs` 固定本仓版本0.2.0。SourceTime的 boot/CLOCK_MONOTONIC 域、producer source_epoch/sequence、原始 capture_start/end_steady_ns 不可重盖；sim_step 另外携带 measurement_epoch 与 measurement_tick_ns（0合法），producer重启与world重置是两个不同事实。派生差分、AMCL定位、桥接与执行链透传原证据。ROS stamp只作为TF/几何配对/路径标签；host时间负责TTL、排队、控制等待与3秒学习动作，物理积分负责真实测量tick。同车Linux Python monotonic与C++ CLOCK_MONOTONIC须同boot、同time namespace。

最终执行仅 `/control/actuator_target`，唯一发布者 `motion_output`。从本轮 tick 计算入口捕获的 computed steady 与原始 deadline 一直保留到实际写前。旧Twist只可用于只读观测；部署必须同时核实并关闭或重映射厂家遥控、nav/recharge等旁路，不允许两个有效执行入口。现有串口写超时2000ms没有物理制动或及时硬件watchdog证明。

## 可复现准备

各命令的目标目录必须是新目录。源码准备校验每份原始SHA256、patch自身SHA256、零fuzz上下文和最终SHA256；厂家资料保持只读。脚本没有自动升级/部署分支。

```sh
# Linux/Humble，先由操作者完成目标依赖/ABI核验；本机未运行真实构建。
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2/src/senior_motion_msgs --packages-select senior_motion_msgs
source install/setup.bash

python3 tools/prepare_sensor_overlay.py --source /path/to/verified-vendor/src \
  --destination /tmp/sensor-clock-overlay/src --optional gps --optional ahrs
python3 tools/prepare_nav2_overlay.py --source /path/to/official-nav2-cache \
  --destination /tmp/nav2-clock-overlay/src --download
python3 tools/prepare_simulation_overlay.py --cache /path/to/official-sim-cache \
  --output /tmp/simulation-clock-overlay --fetch
```

检查准备结果后，在同一已核验Linux/Humble环境为所需脚本添加 `--build` 并另选新目的目录。脚本只构建选中包，不启动节点。不要在本macOS证据基础上宣称已生成可部署二进制。

| 固定官方来源 | tag | 不可变commit |
| --- | --- | --- |
| ros-navigation/navigation2 | 1.1.20 | a097086719c88f781aa59788eca29ac6ca5e56db |
| gazebosim/gz-sensors | ignition-sensors6_6.8.1 | 9b408698a74903c6e3c5eb5745567a0cddef2ef7 |
| gazebosim/gz-sim | ignition-gazebo6_6.17.0 | d9b18d85a17400d32494bca4c3589f519aca7e81 |
| gazebosim/ros_gz | 0.244.20 | 6be1a56e797d4a9e94c9aff280a943df3dfa6d48 |
| ros-controls/ros2_controllers | 2.50.0 | 58dc6db28040f017c992b7ecfa57310c7e78d18d |
| ros-controls/gz_ros2_control | 0.7.21 | 3f341cd49ed579cfb17422582eb14920c203f269 |

它们是工程构建基线，**不是现用车辆版本**。Linux构建前须记录现用dpkg完整版本并核对 Nav2插件、Gazebo core/系统插件、ros2_control及消息typesupport ABI。仿真脚本对上述固定依赖版本不符明确报错。Nav2入口检查同版包与marker，但声明版本不能替代插件ABI实际构建/加载检查；未完成这些检查不得部署。

厂家主路径是N10Plus的X10 UART分支、460800 baud、Astra，以及CRC有效且非空的底盘包。wheel/IMU共享原包前沿与序号，现有唯一串口owner非阻塞读；每个完整包积分，最新完整待发布包/scan可以替换旧待处理数据。driver_read只证明驱动读包时刻，设备/内核/SDK在此前的缓冲延迟未知；不能因这个未知而虚构物理采集时间，也不增加全车硬件tick门。GPS/AHRS按实际可选配置接入。


## 已批准的reset core补丁（取代上面的待决策说明）

0→0 reset执行hook现已实施，详见[最终边界与证据](../reviews/2026-11-16-fortress-reset-boundary.md)。准备脚本现在额外准备固定core的59份构建闭包，并构建core库；仍不复制维护整个engine树。world key基于实际ECM实例与world entity，避免多世界各自entity #1碰撞。消息协议没有增加第二套身份。

操作者完成目标SDK/ABI核验、构建并source同版0.2.0消息包后，运行overlay构建。core/插件必须共同解析到该消息包的唯一 `libsimulation_timing.so.0`，构建脚本会检查ldd解析路径与公开符号；实际加载与运行仍未在本机验证。部署环境须将新overlay的lib放在旧core前面，并把三个薄插件目录放在原system插件搜索路径之前，例如：

```sh
export LD_LIBRARY_PATH=/path/to/verified-clock-overlay/install/lib:$LD_LIBRARY_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/path/to/verified-clock-overlay/install/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

这只是人工部署前置说明，本任务没有执行这些部署步骤，也不自动升级/安装任何依赖。共享库target路径、消息版本、现用dpkg版本与插件ABI不一致时应明确失败，不能静默使用旧SDK。
