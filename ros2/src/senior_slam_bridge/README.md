# senior_slam_bridge

B5.3 后只提供原 ROS transport 类型；运行实现归同级 senior_mapping，不反向依赖它。

```text
senior_slam_bridge/
  CMakeLists.txt, package.xml, README.md
  msg/Keyframe.msg
  srv/{CaptureGraph,ReadGraph,ReadGraphCommit,ReadLoadedGraph,MatchStationaryScan}.srv
```

六份定义、包名、版本、字段/枚举/界限及嵌套 senior_motion_msgs 类型未变。
CMake 使用 rosidl_default_generators 并导出 rosidl_default_runtime、builtin_interfaces、senior_motion_msgs。
运行包以 find_package/ament_target_dependencies 消费标准导出；本包不链接私有 SLAM 库或安装运行程序。

运行命令已变为：

```sh
ros2 run senior_mapping slam_graph_node
ros2 run senior_mapping acquire_scan_receipt
ros2 run senior_mapping candidate_preflight --help
```

旧 `ros2 run senior_slam_bridge` 的上述三个命令不再提供，不安装运行 shim、回退或跨 prefix symlink。
现有客户端仍导入 `senior_slam_bridge.msg/srv`，服务类型/话题名保持，不迁到 WS contracts。
使用全新或独立安装前缀验证，避免把旧安装目录残留的程序当新兼容入口；本片不自动清理既有安装树。

原16份 C++ 文件→`senior_mapping/src/slam/`，原preflight→其 scripts/，18份原单元与探针→其 test/native、test/probes。
[运行包 B5.3 记录](../senior_mapping/README.md#b53-原生-slam-迁移) 提供实际树、测试命令/版本/结果与缺口。
没有在无ROS主机上把XML/CMake或SDK验证宣称为真实typesupport导出、隔离安装或DDS验证。
