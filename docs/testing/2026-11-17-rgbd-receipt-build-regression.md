# RGB-D receipt 时间工具声明缺失回归

## 已确认问题与修复

现场在 ROS 2 Humble 中执行 `colcon build --packages-up-to senior_robot`，
`rgbd_candidate_receipt.cpp:378–404` 报 `senior_motion_msgs::timing has not been declared`。
`dc9db96` 新增了 `measurement_ns` 和 `same_measurement_axis` 调用，
但翻译单元没有包含定义它们的 `senior_motion_msgs/source_timing.hpp`。
修复仅补充直接 include，不改变时间校验行为。

## 定向回归场景（1 项）

输入：包含修复的源码、已加载 Humble 和厂商 underlay 的环境，以及项目 ROS 构建依赖。
停止车端服务后，在容器内项目 `ros2` 目录执行：

```bash
../.ros-venv/bin/python -m colcon build --packages-up-to senior_robot \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -Usenior_motion_msgs_DIR
```

预期：`senior_sensor_ingress` 实际重新编译 `rgbd_candidate_receipt.cpp` 成功，
下游 `senior_robot` 完成，汇总无 failed/aborted 包。
该场景复用真实生产构建目标，不以字符串检查或替身编译代替 ROS 节点编译。
现场提供的修复前日志为失败复现证据；修复后的车端执行结果仍待确认。

## 可复用的本地回归与边界

```bash
.venv/bin/pytest -q \
  tests/unit/test_simulation_clock_overlay.py::test_real_shared_world_epoch_source_identity_and_same_payload_transport_cpp \
  tests/unit/test_shared_sensor_cpp.py
```

这些测试编译并运行真实时间工具和传感器策略代码，可检查相关工具的既有行为，
但不编译 ROS 节点，不能代替上面的定向构建回归。
此次不迁移其他 ROS 运行测试，也不代表 IPC、快照或车辆运行已验证。
