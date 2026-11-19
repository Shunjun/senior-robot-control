# senior_motion_control

车辆执行功能包；本批迁入最终输出和 FinalCommand 原生接收，Python/C++ 同包且原生守卫继续为独立进程。其余仲裁、速度计算、感知与 MotionCore 的完整迁移在后批，不把本批当整车结构已完成。

```text
senior_motion_control/
  CMakeLists.txt package.xml pyproject.toml
  native/{motion_output_guard.cpp,final_command_guard.cpp,final_command_frame.hpp}
  senior_motion_control/{motion_output.py,final_command_frame.py,native.py,__init__.py}
  test/{test_final_command_frame.py,final_command_frame_sender.cpp}
```

构建采用 ament_cmake + ament_cmake_python，安装 Python 包和 `lib/senior_motion_control/{motion_output_guard,final_command_guard}`。依赖 rclcpp、geometry_msgs、senior_motion_msgs 和运行时 ament_index_python；独立 wheel 只用于无 ROS 的 Python 安装验证，**不包含 native 可执行文件，不能代替 colcon 构建用于运行车辆**。

- `MotionOutput` 使用私有 Unix SOCK_SEQPACKET，发布既有 `=ddddQ` 40 字节包（vx m/s、wz rad/s、原绝对截止秒、计算单调秒、序号）。deadline=0 的停止包和原停止屏障保持不变；发布失败按原语义撤销通道，不排队重放。
- `motion_output_guard` 是最终 `/cmd_vel` geometry_msgs/Twist 唯一发布者，保留原截止、顺序、断连和停止保护；计算时间用途本批不改。
- `final_command_guard` 消费既有 senior_motion_msgs/FinalCommand，校验来源/GID/代次和事件，通过原 private frame 输出给 SafetyGate，不发布底盘速度。`final_command_frame.py/.hpp` 是同一原 v3帧契约，无缺字段默认回退。
- `native.native_executable` 通过安装后的 ament 包索引找程序；生产 `SafetyGate` 和 `MotionOutput` 共用这一路径。不存在消息包内的第二份 guard，也不查找旧程序兜底。
- 实车 `senior_robot/launch/safety_motion.launch.py` 与仿真 `simulation_motion.launch.py` 仍组装 SafetyGate，它现在直接使用本包；不新增 launch/进程，不改变原监管、peer identity、环境和继承FD。

启动/停止仍由现有监管入口创建/关闭，不能单独把第二个 guard 接到同一底盘；本批不改变机器人参数或引入新 250ms 接收规则。native_receipt 暂留消息包，后批迁移。

验证：`colcon build --packages-up-to senior_motion_control senior_robot`；`colcon test --packages-select senior_motion_control`；无 ROS 主机可运行 `tests/unit/test_native_output_protocol.py` 的原生确定时钟 harness 和包内 frame roundtrip。Linux SOCK_SEQPACKET 场景在 macOS 跳过时须列为未验证。
