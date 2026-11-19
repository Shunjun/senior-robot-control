# senior_mapping

B5.1 从 `7f1d603` 迁入既有 SLAM Python adapter，并接原默认 mapping launch。
原 native SLAM 后端、地图事务/存储与每图 Home 实现不在本片；不能把 adapter 迁移称为完整建图功能交付。

```text
ros2/src/senior_mapping/
  CMakeLists.txt, package.xml, pyproject.toml
  scripts/slam_adapter
  senior_mapping/
    __init__.py, py.typed
    slam_adapter.py
ros2/tests/test_mapping.py
ros2/tests/test_relocalization.py
```

源为 `senior_robot/senior_robot/slam_adapter.py`，旧文件明确 Compatible 转发同一实现。
公开 PoseGraphBackend、SlamAdapter.serialize/load/anchors、finished_deserialize_response 与原常量保持。
原 graph/snapshot/anchor/store 类型继续复用根 mapping 公开库，单一状态 owner 与类型身份不变。

生产 `senior_robot/launch/mapping.launch.py` 默认仍启动 stock slam_toolbox/async_slam_toolbox_node，
但原 slam_adapter 的安装包改为本包；node 名、参数、restart、进程数、scan_topic=/scan 不变。
非空绝对 capture_directory 或 verified_static_lidar 仍选择原 senior_slam_bridge/slam_graph_node，
原 relay/receipt 和真实原始扫描来源保持；不把补点或避障派生几何写进持久地图。
定位壳未添加到任何 launch，仿真总组合尚在 senior_robot 的收尾另批处理。

SlamAdapter 保留 SerializePoseGraph 成功码、DeserializePoseGraph 空响应对象成功/None失败、
单图 odom owner、map_ref/格尺寸/空 store 前置条件和图 marker 无可信 yaw 则不可用的规则。
main 仍是原服务客户端/graph marker 订阅适配，并不新建 MapOperations/store 或完整地图操作服务。
未接通能力明确保留，不伪造 ready、归档成功或运动输出；最终 cmd_vel 仍仅归原生控制守卫。

ament_cmake_python 安装模块和实际 slam_adapter CLI；manifest 声明 rclpy、slam_toolbox、geometry_msgs、
visualization_msgs，未复制 bridge 的 ROS 消息或链接原生私有库。无 ROS SDK 默认导入不加载 ROS。
旧 senior_robot CLI 名称仍直接指向 canonical main；root 锁、Docker 和 CI 安装闭包同步。

地图 evidence/occupancy/reference/store/archive/operations/workspace 继续原位，事务与数据库/管理 Home
端口在后片单独处理；native SLAM、typesupport/服务原类型名及候选工具迁移单独核定。
新建/保存/切换/删除、每图可编辑 Home、可信定位协作与新算法均不借本片提前实现。

原 map_references 用例保留并读取真实 adapter 源，四份相关用例迁入所属 ROS 位置，不复制测试。
[定位包 B5.1 记录](../senior_localization/README.md#b51-范围与验证记录) 提供实际 45 文件范围、
完整目录、命令/版本/结果、77 passed/2 ROS skipped 和三项目 SDK 安装验证。
真实 Linux/ROS/ament、SLAM、DDS/TF、AMCL和车辆运行未验证，静态/SDK 不替代实测；没有启动服务或部署。
