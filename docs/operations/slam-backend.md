# SLAM 后端：Humble 2.6.10 位姿图与长期占据层

本文记录适配器要对接的 ROS 2 Humble 接口。这台 macOS 没有 Humble，`slam_toolbox` 没有被导入，节点和 launch 都没有执行。下面的服务字段来自 `slam_toolbox` 标签 `2.6.10` 的源文件，不是本机运行结果。

定位用位姿图。导航和显示用另一层长期占据。两层一起保存，回环时按关键帧锚点重投影，再按同一份记录恢复。清掉代价地图不能代替删除：位姿图里的旧扫描仍会被栅格化写回去。

## 目标版本

- 发行版：ROS 2 Humble
- 包：`slam_toolbox` `2.6.10`（`docs.ros.org` 上的 Humble API 文档指向这一版）
- 节点：`async_slam_toolbox_node`，名字 `slam_toolbox`
- 模式：`mapping`
- 里程计：参数 `odom_frame: odom`，一图一个主人。仿真真值话题不作为这个坐标系

`2.6.10` 之后的主线增加了 `slam_toolbox/msg/PoseGraph`、`/slam_toolbox/pose_graph` 和回环事件。那些不是本任务对准的 Humble 接口。

## 序列化

服务 `/slam_toolbox/serialize_map`，类型 `slam_toolbox/srv/SerializePoseGraph`。

请求：

```text
string filename
```

响应：

```text
uint8 RESULT_SUCCESS=0
uint8 RESULT_FAILED_TO_WRITE_FILE=255
uint8 result
```

`serialization.hpp` 的 `write` 把位姿图写到 `filename.posegraph`，把关联扫描写到 `filename.data`。适配器只传文件前缀，不改这两份定位数据，也不从里面删掉已经从长期层消失的箱子。

## 反序列化

服务 `/slam_toolbox/deserialize_map`，类型 `slam_toolbox/srv/DeserializePoseGraph`。

`2.6.10` 的请求是：

```text
int8 UNSET = 0
int8 START_AT_FIRST_NODE = 1
int8 START_AT_GIVEN_POSE = 2
int8 LOCALIZE_AT_POSE = 3
string filename
int8 match_type
geometry_msgs/Pose2D initial_pose
```

响应是空的。上游注释写的是 `inital_pose`，含义是 map 到 `base_frame`（本车是 `base_link`）。

续建时：没有初始位姿用 `START_AT_FIRST_NODE`，调用方给出位姿用 `START_AT_GIVEN_POSE`。`LOCALIZE_AT_POSE` 是定位模式，不拿来冒充续建。空响应没有结果码，超时或服务不在视为失败。失败时不把长期占据层说成已经恢复。

## 关键帧位姿

`2.6.10` 没有按节点查询校正位姿的服务。`LoopClosureAssistant::publishGraph` 发布 `slam_toolbox/graph_visualization`（`visualization_msgs/MarkerArray`）：

- 节点球：`ns` 为 `slam_toolbox`，`id` 是顶点 unique id，`pose.position` 的 x/y 来自 `GetCorrectedPose()`
- 同一函数不拷贝航向。标记模板把四元数留在单位旋转
- 边：`ns` 为 `slam_toolbox_edges`，不是锚点
- `DELETEALL` 只用于清旧标记

交互标记的名字是节点 id 字符串，平移控制也只写了 x/y。求解器上的 `GetNodeOrientation` 只在拖动节点时内部使用，没有对应的 ROS 查询服务。

因此只凭这张 MarkerArray 不能做带旋转的回环重投影。`anchor_from_graph_marker` 对没有航向的节点返回 `available=false`。存储层遇到不可用或缺失的锚点时保持原坐标，并把这次参考标成待核查，不猜测位移。

C++ 里 `GetCorrectedPose().GetHeading()` 和 `GetNodeOrientation` 才是完整校正位姿。要在不改厂商目录的前提下拿到它们，需要单独的 `ament_cmake` 桥。本机无法编译或加载这个桥，所以适配器停在上面的窄接口后面，没有把桥写进仓库。

## 窄接口

`PoseGraphBackend`：

- `serialize_pose_graph(prefix) -> int`，0 对齐 `RESULT_SUCCESS`
- `deserialize_pose_graph(prefix, match_type, initial_pose)`，对应空响应的反序列化
- `corrected_anchors()`，带航向的关键帧；航向未证实则为不可用
- `odom_owner()`，必须等于这张图已经绑定的那一个里程计主人
- `painted_cells()`，定位栅格会画上的格子。ROS 侧固定返回空，因为不调用 `dynamic_map`
- `grid()`，宽高和原点。变化不修改 `MapRef`

`SlamAdapter.serialize` 同时落盘位姿图前缀和 `occupancy.json`。`load` 先反序列化位姿图，再把长期层和删除记录灌回 `MapReferenceStore`。

## 不把定位栅格当导航图

`/slam_toolbox/dynamic_map` 和 `/slam_toolbox/save_map` 画的是位姿图扫描。箱子从长期层删除之后，这些扫描仍在 `.data` 里，下一次栅格化会把箱子画回来。

长期层只接受几何确认写入，删除记录按该格确认时的几何锚点保存。回环时活格子和删除记录都按各自锚点重投影。`project_pose_graph` 不接收定位栅格。`clear_costmap` 不修改长期层，也不删除“已删除”记录。

`map_id` 与 `reference_revision` 在栅格变大、原点变化和已证实的锚点重投影之后保持不变。另一张图或另一个里程计主人被拒绝。
