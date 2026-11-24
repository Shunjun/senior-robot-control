# SLAM 后端：Humble 2.6.10 位姿图与长期占据层

前半部分保留初始适配阶段的接口记录；当前生产能力和验证范围以文末更新为准。本文记录适配器要对接的 ROS 2 Humble 接口。这台 macOS 没有 Humble，`slam_toolbox` 没有被导入，节点和 launch 都没有执行。下面的服务字段来自 `slam_toolbox` 标签 `2.6.10` 的源文件，不是本机运行结果。

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


## 正式建图入口的里程计帧（2026-11-21）

`senior_robot` 的 `mapping.launch.py` 提供显式 `odom_frame` 参数，默认仍为 `odom`，
不修改 `slam.yaml` 或其他部署入口的默认值。stock SLAM graph 和 Python adapter 共用
同一个选定值；native capture 和 verified static lidar 分支也将同一值覆盖到 graph 参数。
每张图仍只绑定一个里程计主人，选择参数不会创建 TF、认证来源或自动迁移既有地图。
参数按本 launch 既有 frame 命名校验，拒绝空值、空白/控制字符、超过128字符，以及
与固定地图帧相同的 `map`；不自动寻找或猜测运动来源。

现场在 `9b029b7`、Humble SLAM Toolbox 2.6.8 上曾临时改用 EKF 的 `odom_combined`，
收到 115 × 188、0.05 m 栅格与 `map → odom_combined`，当时默认 `odom` 未产图。
临时节点已停止，这份证据不代表当前默认入口已验证、已存地图已切换，或活动地图已接到生产/网页。
现场具备相应 TF/来源并明确选择该帧时，可通过以下参数形式启动；这里只记录用法，
本片未运行此命令、未部署或改变任何正在运行的车辆：

```sh
ros2 launch senior_robot mapping.launch.py odom_frame:=odom_combined
```

使用 native capture 或 verified 分支时，在原有参数后追加相同的
`odom_frame:=odom_combined`，保留 capture 目录、扫描 receipt 和静态安装来源要求。
切换里程计帧不是修改已绑定地图的捷径：已有地图仍须核对其 odom_owner，
旧图恢复后可信重新匹配、连续证据接入、自动一致 checkpoint 和重启恢复另行实现。

本文前面关于“桥尚未入仓”的内容是 Python 适配初始阶段记录；原生桥现已由 B5.3 迁入
`ros2/src/senior_mapping/src/slam/`，`senior_slam_bridge` 保留消息/服务类型。
本片只配置已有启动组合，不新增生产 transport 或来源认证，不开放导航/持久写图，
亦不完成活动地图选择、独立 Home、WS 或网页刷新；没有手动保存步骤。

## 活动建图生产组装（2026-11-23）

当前生产路径为 `vehicle_manager.agent.mapping_catalog` → `senior_mapping.runtime/native_session` → 本地IPC → 共用 `agent_adapter` 的 `RosMappingBackend` → 自有 `slam_graph_node`。不需要在控制页面之外另起一个stock mapping launch；手动诊断launch与此所有者不能同时运行同名SLAM服务。运行入口不在导入时启动进程，只有明确新建或恢复地图才启动SLAM。

切图要求停车、无在途路线，串行进入loading、关闭入口、flush、退出旧图、加载目标、匹配/待校准、再恢复更新。窗口中的扫描丢弃；新进程不消费旧窗口缓存。原生MappingGate是新增ROS服务，需要interfaces和mapping协同clean rebuild，不能仅更新Python wheel就认为原生端已升级。旧接口安装缺少该类型时，现有控制仍可启动，建图报告未就绪。

Parallels Ubuntu 22.04 实际安装 ros-humble-slam-toolbox 2.6.10-1jammy.20260916.165114。CMake与包清单统一固定2.6.10，与存档BACKEND_VERSION及既有适配代码一致；不升级或安装另一个ROS环境。已有标准C++几何/历史测试通过不等于ROS新节点编译通过，隔离colcon验证结果另行记录。

默认checkpoint最长延期10秒，停车优先，只有一个重计算/保存工作在途；100000个占据track、8MB临时图传输与既有16K原生图节点限制均有界。超限明确失败，不静默吞掉已接受的图commit。保存失败保留最后完整checkpoint，并阻止切到下一图；不会因此禁止手动驾驶。这些是实现边界，尚未测得四核设备的性能预算。

图、稳定占据、参考与冻结Home一起保存。当前Home只有WaypointStore这一份实时真值；每图的冻结绑定随checkpoint恢复。加载不匹配进入待校准，Home仅作参考，不发布真实车辆位姿。自动初始化为Home附近局部匹配与稳定几何复核；任意位置全局重定位未实现。手动位姿提示、Home编辑和导航参考的后续实现见下面2026-11-24更新。

Parallels Ubuntu 22.04 / Humble 2.6.10隔离构建现已通过，三个包均成功。senior_mapping的20个原生CTest分轮全部通过，另验证production gate暂停丢帧、图捕获、串行退出/加载和恢复后新commit；日志与源码哈希见/home/toby/senior-audit-20261031/active-mapping-20261123-native-v1。没有部署到车端。

上游2.6.10 Ceres插件硬编码options_.num_threads=50，VM曾将其截为4；OMP_NUM_THREADS=1不能限制这个CXX线程池。生产默认现已改为下面的独立有界插件。上游依据：https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/solvers/ceres_solver.cpp#L114。

## 手动校准、Home编辑与运行时导航参考（2026-11-24）

生产使用`senior_mapping/BoundedCeresSolver`，同版2.6.10算法及许可证随包安装。`mapping_solver_threads`默认1、允许1–4；`mapping_worker_threads`另将TBB并行预算设为1。原生非空图重载已报告Ceres Threads Given=1/Used=1及收敛。线程上限不等于整个SLAM进程只有一个线程，也不代表车端CPU/时延已经达标。

新版manager快照`mapping_protocol_version=2`开放`map.calibrate`和`map.home`。网页现有位姿选择器分别提交车辆位置/朝向提示和Home坐标；前者只作为新会话原生局部匹配的种子，成功匹配才恢复更新。失败继续待校准，不能直接把提示作为车辆位置。旧版协议仍不发送这些命令。操作走既有operation_id与查询链路，失去回执后查询同一操作，不重发编辑。

Home仍只有WaypointStore一份实时真值。Home编辑与checkpoint共享Home事务和revision校验，保存途中发生的新编辑不会被旧绑定覆盖；图锚点修正后保留编辑点相对原锚点的位置和朝向。车辆位姿与Home互不代替。完整checkpoint保留该图的冻结Home绑定，重开后恢复。

导航栅格由同一原生SLAM会话/commit重新生成，保留明确观测的free/occupied/unknown，不能把稳定显示层的unknown标成free。原生生成前检查尺寸，最多1,048,576格。导航栅格不回填长期占据记录；长期几何/删除证据及其checkpoint算法继续保留。运行期栅格最多每1秒（闲时）或5秒（行驶中）重绘；只合并栅格快照，不跳过已接受的观测序列。

新增`ReadMappingGrid`/`ReadMappingPose`，`ReadGraphCommit`附加对应观测的原始odom位姿；必须协同重建interfaces、sensor_ingress、mapping，禁止只换Python包。物理适配器验证自有boot、commit及静态安装来源后，安装地图参考与map/odom配对；关键帧之间使用新鲜的原始观测/odom receipt推进车辆位置，不重新盖时间戳。超过0.5秒、来源变化或撤销后停止提供可信位姿；切图先撤销旧参考，不回退旧静态图。

原生子进程的/map和metadata重映射到私有名称，原生map→odom定时发布关闭。物理适配器单独发布获准的/map及map→所选odom，已有接收侧路线资格、安全门和碰撞约束仍生效。新栅格id不代替参考generation；回环参考变化会使旧路线失效。

实际启用导航仍需车辆profile配置绝对路径`navigation_parameters`及原有合格Nav2/运动安全条件；允许仅配参数、等待活动建图初始化参考，不再强制预置`navigation_reference`。没有这些配置时明确保持导航不可用，不能仅因网页有地图就宣称可以驾驶。

macOS完整回归2479 passed / 116原有skip，网页182项测试及构建通过；本机六个改动核心模块定向类型检查、五个重建wheel及已有core的隔离安装/源哈希检查另行记录。Parallels原生三包构建与针对性生产会话/静态安装来源测试已通过。尚未部署到树莓派，四核地图增长、视频并行、控制时延和实车路线执行未验收。

最终收尾复验：本机2480 passed / 116既有skip，ROS实际栅格/静态来源/新鲜odom→Python导航参考桥通过，正常刷新保留位姿、来源撤销后停止资格。21项原生建图CTest已通过，proof断言修正后的静态来源与drift两项另行通过；160个冻结源码hash一致，测试进程清理通过。proof按所有typed字段比较，避免将不稳定CDR填充字节当作来源变化；相机退出回归的等待预算覆盖现有10秒关闭策略，未修改相机启停行为。
