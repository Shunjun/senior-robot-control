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

## T3/T4 runtime 与归档接线（2026-11-26）

管理侧沿原 0.05s 循环读取 start 握手给出的独立 latest-scan 槽。graph PENDING
不阻止同 anchor 的新 scan；完整 source/boot、原 capture/deadline/floor、同一 canonical
anchor digest 和 TF basis 合格后，才在原 ActiveMap 内 accept → GeometryTracker →
C3 提交 → bind_support。tracker/free confidence 的有界 transient stage 也在提交成功后
替换；容量拒绝保留旧引用和完整墙，Q 的 adapter 回调独立运行。未知 raw hit 不再走
原 unknown_static 的固定升级旁路。旧 free 代次遇到合格 hit 会失效，不把该 hit 直接认成
fixed occupied；有限 hit 之前的射线才可积 free，缺束/遮挡/无协议证明的无回波不清图。

source/next/capture 失效、停止和已观察的参考变化使用本地 fence。旧 valid 文件不能重开
已退休资格；新 graph basis 到达前的旧 R 批次不绑定到新图，尚未观察到的窗口仍受原
500ms deadline 约束。恢复/跳帧后的稳定窗口只消费新的独立帧。恢复关联只查询当前局部
持久摘要及已有空间索引；歧义、候选/查询容量饱和时不撤旧贡献、不靠分配新 ID 绕过。

导航改走 `map.backend.reference` 的 owner 选择与 `map.backend.install_fixed` 私有文件
安装。文件不超过 8MiB、同 uid、私有 regular inode、no-follow、SHA-256 验证，桥再核对
当前 boot/MapRef/R、完整 graph/digest、floor 和 fixed/evidence 水位。owner 串行发送，
没有无界安装队列；存盘期间 graph/fixed 冻结，入口 latest 槽只保留最新候选。桥发布唯一
`/map` 并安装 reference 后回传原 watermark 和内容 G；owner 比较实际 ACK 后才把导航
view 标为已安装。没有 free 格也可安装，不伪造 free。ACK 不证明 Nav2 StaticLayer 已消费
或完成 resize；原 sole-map-owner 检查和 stock map remap 继续生效。

`grid` RPC 必须显式指定 diagnostic 或 migrate_legacy；后者每个 backend boot 只允许
一次，V2 恢复永不调用。原 1/5s native raster 刷新不再作为导航 fallback。
`mapping_navigation.hpp` 的原三态服务仅作诊断/一次 V1 迁移。独立 scan 回调持续使用原始
stamp 发布 TF，包括 graph gate/capture 期间，不依赖 graph 后的 pose 回调。

保存先 gate/drain，捕获固定的 graph/fixed immutable cut，磁盘 I/O 不持管理 intake 锁或
Q 锁；原 Home/save 事务统一锁顺序，也保护导出 checkpoint 租约。旧保存只确认其捕获的
change_sequence。V2 的成员/hash、durable 发布和读取规则见 map-bundle 文档。失败 capture
退休 fixed 资格并保留最后 catalog 指向，不把远端取消当作已停止的证明。

当前生产 `NativeMappingBackend.geometry_profile` 为 None，原因是没有本 profile 的实测
误差和协议证据；可靠 occupied 升级保持 `profile_missing`，没有部署示例物理阈值。
测试显式注入 synthetic-admitted 的 T2 profile/轨迹，只验证接线。仍需原装配责任人提供
原 scan/exact odom/TF 与误差来源，才能激活物理升级。无 ROS/colcon 本机不声称 native、
Nav2 安装或实机通过；可运行 `SENIOR_SHARED_LAYERS_PROCESS=1` 的
`ros2/tests/test_shared_layers_process.py`，另需隔离测试 adapter socket 和真实有资格入口。
默认 skip 明确代表未验证，不算三进程通过。VM 模拟树莓派的资源实验也尚未执行。

资源限制保留：dense bytes、G、三态验证和部分 staging 仍 O(N)，support/free 和 tracker
stage 会同时持有旧/新有界结构，部分 legacy mask 仍整份复制。当前局部查询、dirty tiles
及有界记录不等于端到端 O(dirty)，也不构成 CPU/RSS/Hz 已达标的承诺。

## Task7 真实链 harness 与记录入口（2026-11-26，实施交付，未运行 ROS）

`ros2/tests/test_shared_layers_process.py` 现在可自行组合真实 LaunchService、原
`_NavigationStartup`、planner/localization ExecuteProcess、agent_adapter 与
DedicatedController。它使用私有 catalog/socket、真实 robot_state_publisher/原 URDF、
普通 `/scan`、`/wheel/odom`、`/imu/data` 和同 stamp odom→base_link；没有第二个
Q、authorization、coverage 或 motion-runtime writer。静态资产由原 world_reference
从同一 motion_qualification.sdf 生成，扫描直接 raycast 原 collision box 和原 S02 曲面。
原 launch 仅抽取 `motion_stack_actions(context, *, before_navigation, after_navigation) -> MotionStackActions`，原入口仍依原顺序
插入 environment/physics/camera，默认参数、startup 状态机未改；两个必需的输入动作工厂在原列表位置调用，保留构造及
Launch 执行顺序。原入口固定传原 physics/environment 和 camera 工厂，没有默认新开关。

功能 episode 检查 static→live 两旧 group exit0、同 overlay 路径、新原 planner 启动；
manager/catalog 真实选 R/install 后，由原 global/local 插件写 coverage，最后 state.get
ready。两 coverage 可用同源不同有效 sequence，各自回找原 Q/TF，不要求同帧。S05
用原 PublishFiles 屏障与原 capture RPC，一次 A→B，核 native/manager graph 水位不变、
同 anchor evidence/fixed 推进；S07 包含原支持撤销、墙保留、真实保存/close/restore、
ProductionMappingRequests.map.export（map-transfer 边界）及逐格 O/F/U/原重grid核验。
S09 保留真实旧 Q/slot 与 late fixed IPC 输入，停 scan 后不能续期，新 backend/R/source
只能靠新完整 capture 与原授权恢复。R4 从原 route 的 running 状态正常 Launch shutdown，
以 PID+starttime（含 zombie）核实际退出，未确认时保留证据，不以目录清理冒充退出。

本机仅 helper/语法/相关 portable 检查；完整十包、ROS/Launch、实际双 costmap、上述真实
episode 及四核性能均未验。磁盘屏障只证明 Python checkpoint 段；capture RPC 起止是
可得上界，没有 native SaveToFile 子阶段时刻，不能声称该慢调用与 Q 已重叠。global 内部
重建插件需要新的 startup run 的原限制保留，harness 不代签 permit。synthetic-admitted
只改原 synthetic_profile 的 profile_id，其数值与 evidence_digest 不改，不是部署几何证据。

以下是**后续 VM 派单命令**，不是本机已执行命令。先将负责人最终正常集成后的准确 commit
冻结到私有源码/独立 build/install/log；禁止混 T6 未提交源。先复核 Humble/ABI、磁盘、空闲
localhost domain 和原 dependency report，再进入源码根目录。原 12+5 Python prefix 仅是
依赖：`/tmp/navigation-final-python-20261126/prefix` 与
`/tmp/navigation-candidate-5d929e-20261126/python-prefix`，不能复用后者旧产品源码。

```bash
source /opt/ros/humble/setup.bash
source /tmp/navigation-costmap-a097-lethal-first-20261126/install/local_setup.bash
source /tmp/nav2-s13-unknown-cost-20261126/install-mppi/local_setup.bash
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/tmp/navigation-final-python-20261126/prefix:/tmp/navigation-candidate-5d929e-20261126/python-prefix:${PYTHONPATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# task_run 指本次新建的 0700 私有目录；不覆盖旧 prefix/build。
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
colcon --log-base "$task_run/log" build --base-paths ros2/src \
  --build-base "$task_run/build" --install-base "$task_run/install" \
  --packages-up-to senior_mapping senior_navigation senior_robot senior_sim \
  --executor sequential --parallel-workers 1
source "$task_run/install/local_setup.bash"
export PYTHONPATH="$PWD:$PWD/contracts/src:$PWD/senior_control/src:$PWD/ros2/src/senior_mapping:$PWD/ros2/src/senior_localization:$PWD/ros2/src/senior_robot:$PWD/ros2/src/senior_navigation:$PWD/ros2/src/senior_perception:$PWD/ros2/src/senior_motion_estimation:$PWD/ros2/src/senior_motion_control:$PWD/ros2/src/senior_sensor_ingress:$PYTHONPATH"
colcon --log-base "$task_run/test-log" test --build-base "$task_run/build" \
  --install-base "$task_run/install" --packages-select senior_mapping senior_navigation senior_interfaces
colcon test-result --test-result-base "$task_run/build" --verbose
# ROS_DOMAIN_ID 由 VM owner 核空闲后指定，不在此猜一个已空闲域。
export ROS_LOCALHOST_ONLY=1 SENIOR_SHARED_LAYERS_PROCESS=1
python3 -m pytest -p pytest_asyncio.plugin \
  ros2/tests/test_shared_layers_process.py::test_owner_adapter_native_capture_restore_and_retirement \
  --basetemp "$task_run/functional-pytest" -vv -s
```

十包闭包为 interfaces/localization/mapping/motion_control/motion_estimation/navigation/
perception/robot/sensor_ingress/sim。先核真实导入、生成 typesupport、插件 XML/.so、ELF/maps
及实际消费者参数；fixture 会查询 controller_server 的 batch=400、vx_max=.22、vx_min=-.15、
wz_max=.4，并记录 overlay/实际 descendant argv。不能用旧 S13 的 batch=2000 结果替代。
build 使用 CMAKE_BUILD_PARALLEL_LEVEL/MAKEFLAGS 请求单编译；VM 须核实际编译并发，不能将此文本当构建证据。

Stationary-only sensor/SLAM reference 输入仅生成一次（不满足持续 MPPI/S16）：

```bash
python3 ros2/tests/shared_layers_sensor_source.py \
  --freeze "$task_run/reference-input.jsonl" \
  --world ros2/src/senior_sim/worlds/motion_qualification.sdf \
  --urdf ros2/src/senior_sim/urdf/senior.urdf.xacro
# 冻结文件通过同一 schema/hash 后供两版各用一次，不重新抽样或以控制输出改输入。
export SENIOR_SHARED_LAYERS_REFERENCE_TRACE="$task_run/reference-input.jsonl"
python3 -m pytest -p pytest_asyncio.plugin \
  ros2/tests/test_shared_layers_process.py::test_frozen_stationary_system_reference \
  --basetemp "$task_run/reference-pytest" -vv -s
# TRACE 指对应 pytest 私有目录内的 native/system-trace.jsonl 或 reference/system-trace.jsonl。
python3 tools/occupancy_replay_benchmark.py --system-trace "$TRACE" --output "$task_run/summary.json"
```

执行 reference pytest 前，由 VM owner 在独立 transient **task-only** scope 内核实际
`cpu.max=400000 100000`、四 CPU affinity/cpuset、`memory.max=4294967296`、swap 策略和
MemAvailable≥6GiB；不要改用户/system slice，不在 Bot1 创建 scope。两版串行、同输入/hash、
同包络，scope 内包括 recorder。若需 taskset fallback，逐 descendant 核继承。历史产品基线
`11a26906c9831b000b09e2279ad7ef4ea514d903` 的旧装配没有新 factory/Q/coverage，不能把
当前 pytest 原样当作旧版 runner；其原入口适配及缺项 N/A 仍需后续 VM 对照 owner 核实，
本轮没有构建/取得可比旧版栈或补写旧版产品。

stationary-only reference 是 60s/300×5Hz×2048 的物体 stable A→motion→stable B，原曲面从3.0m到+0.4m，
第150帧一次显式 save，第299帧结束后原 restore/export；20Hz wheel/TF 固定零轨迹，普通
IMU 与原 mount 一致。参考 stamp 按冻结偏移与每次运行记录的 epoch 形成，不以晚到回调
重新盖新鲜时间；实际 lateness 另记。bootstrap/postlude 不计入300帧，恢复 postlude 为同一 B。
功能模式的原 SE2 则仅跟随最终 `/cmd_vel`。冻结模式仅发起同一原 route 观察其生命周期，
不跟随私有 MPPI Twist，也不伪造车动；零 odom 无法满足原 progress 的 .2m/10s，不能提供健康持续60s控制；真实失败时测试
保留失败并 RED，不重复发任务、调超时或把记录成功当导航成功。restore/export/close
可能按原 lifecycle 必须再 capture，不能把它们藏成“只有一次 native SaveToFile”。

trace schema=1：metadata（run/commit/tree/host、输入/地图/URDF/profile/params hash）、
stage/steady_ns、完整 host+ScanKey、原 stamp/sample/deadline、原契约 R/G/role/authority/
permit/plugin_instance/cycle、进程 PID/starttime/PPID/PGID/cgroup、resource/file/DDS 观察。
CPU 200ms，PSS/RSS/IO/context-switch 1s；JSONL writer 队列4096、单行262144 bytes，
capture 索引512，summary capture 窗4096、进程512，overflow/eviction/error 明记。
控制回调只入队；磁盘 writer 与资源采样独立 task-owned 线程，结尾有 drop/queue高水位/
writer CPU/emit耗时，开销不冒称零。summary 的 complete 仅表示存在 trace_end，不能当
功能通过；无 footer、丢 trace、未配对 coverage 分列。接收/周期延迟是观测上界，Twist
没有 capture key，只能窗口关联；未导出的 native 发布时刻、锁等待、DDS queue/match/loop
指标 N/A，sequence gap 不是 DDS 纯丢包。最终 T6 UI 未集成前不能签完整 S16/实车。

VM reference scope 可直接用以下命令包住上面的 reference pytest（仅 VM 执行 owner 在功能
门槛、prefix/输入 hash、swap 与原 warm-chain 资源亲核后执行）。作用域是本次唯一任务，
标准库 preflight 在任何 ROS 导入/启动前核实际 cgroup/affinity/可用内存，失败即停；
cpuset 不支持时由 VM owner 记录失败与批准的 taskset 继承核验，不改系统 slice。

```bash
systemd-run --scope --unit=navigation-task7-reference-20261126 \
  --property=CPUQuota=400% --property=MemoryMax=4G --property=AllowedCPUs=0-3 \
  python3 -c '
import os, pathlib, sys
line = next(s for s in pathlib.Path("/proc/self/cgroup").read_text().splitlines() if s.startswith("0::"))
root = pathlib.Path("/sys/fs/cgroup") / line[3:].lstrip("/")
assert line != "0::/", "task-only cgroup required"
quota, period = root.joinpath("cpu.max").read_text().split()
assert int(quota) == 4 * int(period)
assert root.joinpath("memory.max").read_text().strip() == "4294967296"
assert os.sched_getaffinity(0) == {0, 1, 2, 3}
available = next(int(s.split()[1]) for s in pathlib.Path("/proc/meminfo").read_text().splitlines() if s.startswith("MemAvailable:"))
assert available >= 6 * 1024 * 1024
print("task cgroup", root, "swap.max", root.joinpath("memory.swap.max").read_text().strip(), flush=True)
os.execv(sys.executable, [sys.executable, "-m", "pytest", "-p", "pytest_asyncio.plugin", "ros2/tests/test_shared_layers_process.py::test_frozen_stationary_system_reference", "--basetemp", os.environ["TASK7_REFERENCE_BASETEMP"], "-vv", "-s"])
'
```

`TASK7_REFERENCE_BASETEMP` 指新的私有 pytest 目录，须在执行前显式 export；scope unit 名称也
应为本次唯一且未占用名称。上面没有设置 MemorySwapMax，也没有把历史 scope 的25%/256MiB
探针当作车辆规格。完整基线/UI 并发与最终 S16 仍由负责人另派，不能只跑这段就签通过。


### 八项独立审核返修边界（2026-11-26）

当前入口名称含 `stationary`，仅记录静止 sensor/SLAM；采满300帧不代表全负载成功，
终态/idle之后MPPI指标N/A，真实failed/missing等仍保留并使pytest RED。运动参考仅有
仓库外提案，未实现/运行；短原route不能承诺一次健康60s，T6 UI未集成仍不签完整S16。

coverage contract的schema=1与trace envelope的schema=1独立：完整payload置于record，
拒未知contract schema；解码兼容原file.record和原扁平helper。原async pump每100ms只读
slot与两coverage，即使主loop在save/restore await仍观察；磁盘读在IO worker，capture
索引只由asyncio owner访问，控制回调只入队。每角色各自按完整host/ScanKey回Q/TF/slot，
不要求双角色同sequence，不续原sample+500ms。summary按bootstrap/stationary_reference/
save/restore_export列样本数、缺角色N/A与缺Q/TF/slot；bootstrap不能冒充主窗60s。

原navigation.task_status按execution_id/task_epoch观察starting/running/cancel_pending/
failed/succeeded/canceled/missing，accepted不算active。原私有controller Twist与最终
cmd_vel仅按窗口关联；输出只在最近200ms内确有running及ready观察时计入该窗口，查询
阻塞/缺样时未知，不外推负载时长。终态/失格时刻是观察上界，分开保留。取消仅发一次，
按同effect_id/sequence/epoch查询原revoke，须accepted/output_exited/fenced-through及
原progress退出同时确认；ok本身不够，未确认失败留trace，不造退休file。

CPU ticks另带实际identity读取前后的monotonic区间，完成时刻作为统一的采样端点上界，
不采用full字段读完后的入队时间。两次CPU读区间之间或区间内触及phase边界的差分标
straddling，不分配阶段CPU；同名phase往返也不合并。阶段按CPU读区间归属，整次平均
同样用CPU读完成端点；这是有界分次观察，非原子快照。旧trace缺CPU读取区间则CPU
指标缺项（N/A），不从入队时间猜端点；非CPU字段仍为原入队观察口径。IO/上下文切换为
同PID:starttime整次first-last差分，不称阶段计数。cgroup按字段保留最近实际值及时间，
轻量cpu行不抹内存/events/peak/max；kernel memory.peak与sampled current max口径不同。
RSS不相加异时峰值，PSS没有异时求和；task cgroup含producer/recorder，不称产品独占。

发现后代时将原starttime及父身份带到register复核，full采样前后复核incarnation/cgroup；
不符丢弃并记unconfirmed。stat/cgroup为同incarnation内的有界分次观察，不是原子内核
快照。受控读取序列覆盖竞态分支，未在Linux现场制造PID复用；无按名字补认/进程控制。
本次仅本child定向helper/summary/factory及必要旧回归；Humble/十包/真实链仍留后续VM。
