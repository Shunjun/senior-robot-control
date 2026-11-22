# 当前 ROS 包归属

2026-11-22，功能分支 `feat/vehicle-architecture-refactor`。代码迁移已提交，Linux ROS 原生构建与车端验收待完成，不代表已经部署。

| 包 | 职责及入口 |
| --- | --- |
| senior_interfaces | 26 个消息、5 个服务；仅 rosidl 接口，无业务代码 |
| senior_sensor_ingress | native/raw/scan/static-TF/RGB-D receipt、时序头文件、ros_lifecycle、ros_video、camera_rate_limit |
| senior_motion_estimation | 运动估计、MotionFusion、scan_motion_node、scan_matcher_replay、恢复观测值 |
| senior_perception | 雷达/深度/语义感知与 RGB-D 候选生产；六个 ROS Python 节点 |
| senior_motion_control | 安全门、命令与运动输出、控制保护程序 |
| senior_localization | 定位、导航参考 |
| senior_mapping | 建图、SLAM 图、mapping.launch.py 与 slam.yaml |
| senior_navigation | 导航、navigation.launch.py、Nav2 配置与行为树 |
| senior_robot | 实车预检、服务、agent_adapter 与整车组合；注入 MotionFusion 的硬件 profile loader |
| senior_sim | 仿真组合、相机、world_reference、仿真参数与 RGB-D transport |

`senior_scan_matcher`、`senior_motion_msgs`、`senior_slam_bridge` 已删除。29 个历史 robot Python 转发文件及未使用的 camera_source/vehicle_compat 入口已删除。调用方直接导入功能包。算法和接口字段保持；消息的包限定名称改变。

所有 MotionFusion 组合必须显式传入 profile_factory；实车已有 `senior_robot motion_fusion` 组合入口负责注入。ROS 通用 executor 和有界退出代码统一使用 `senior_sensor_ingress.ros_lifecycle`。手写时序头文件由 ingress 的 `sensor_timing` CMake target 导出；接口包不安装这些手写头。

## 安装切换

必须停止旧版本节点后，在新的构建/安装前缀中整体构建生产者与消费者。不要叠加旧 install，也不要混用新旧消息类型。以下只构建，不启动节点：

```sh
source /opt/ros/humble/setup.bash
cd ros2
colcon --log-base log-ownership build \
  --build-base build-ownership --install-base install-ownership \
  --packages-up-to senior_robot senior_sim
source install-ownership/setup.bash
colcon --log-base log-ownership test \
  --build-base build-ownership --install-base install-ownership \
  --packages-select senior_interfaces senior_sensor_ingress senior_motion_estimation senior_perception senior_mapping
colcon test-result --test-result-base build-ownership --verbose
```

建图使用 `ros2 launch senior_mapping mapping.launch.py`；导航使用 `senior_navigation navigation.launch.py`；仿真使用 `senior_sim` 的对应 launch。实车仍使用 `senior_robot` 的整车组合。这些命令会启动节点，本文未执行。

Python wheel 构建不生成 ROS interfaces，不替代 colcon。接口 typesupport、已安装时序头 consumer、原生 SLAM 链接、DDS 和实际相机链路必须在 Linux ROS 环境验证。当前依赖声明无环；历史 Python 感知/估计相互引用及感知对硬件 profile 类型的引用仍存在，本轮不宣称功能包完全独立。

验证详情见 [本轮测试记录](../docs/testing/2026-11-22-ros-package-cleanup.md)。
