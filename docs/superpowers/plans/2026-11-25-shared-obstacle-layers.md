# Shared Obstacle Layers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 同一mapping owner输出可撤销、可存档的fixed三态图和逐scan当前层Q，让现有Nav2导航至地图内unknown，并让最新UI消费同源图层。

**Architecture:** 原`NativeSession → ActiveMap → MapReferenceStore`保留唯一持久地图写入职责，Karto仅提供定位graph/anchor。合格scan事实先独立到达ROS桥的最新槽并发布Q，原owner再做一次有界几何及fixed更新；桥仅安装owner已认可的fixed cut。两costmap使用fixed→Q薄插件→inflation，UI使用独立Q身份/期限而不按scan重拉fixed tiles。

**Tech Stack:** 原Python/asyncio、C++17、ROS2 Humble接口、slam_toolbox 2.6.10 EXACT、Nav2 Smac Lattice/MPPI、pytest/ament、TypeScript/React/Vitest。Nav2部署/补丁版本待Task 0取得，不能将上游审计SHA当作部署版本；不增加模型、ICP、通用tracker框架或第三方运行库。

**Spec:** [2026-11-25-shared-obstacle-layers-design.md](../specs/2026-11-25-shared-obstacle-layers-design.md)。旧frontier文档不是本计划输入。

## Global Constraints

- 本轮只归档设计和计划：仓库文档A2/M2/D0；产品/测试/配置/构建/工具均0。不安装、不运行测试/构建/ROS/服务/实车，不checkout/merge，不改UI树，不推送/操作PR。以下命令和勾选项全部是**未来开发派单**，本轮未执行。
- 用户已经批准孤立静止可入图、移动撤旧贡献、重新稳定可再入图；无语义类别、墙连接要求、永久motion veto、自研frontier、几秒/30秒等待、通道封锁。
- Q第一帧合格hit即可避障；分类、keyframe gate、存盘及通用RPC锁不能阻塞Q。缺测/遮挡/撤销不证明free；稳定墙出范围保留；仍有独立可信occupied支持的格不能降unknown。
- 只有原mapping owner写fixed/保存/迁移；ROS桥、Nav2和UI只读。原运动最终输出、任务/action/epoch、取消终态屏障、来源/时钟/TF/定位资格继续有效。
- 原档保留；首次V1迁移可冻结一次旧native三态为legacy-derived，后续权威副本不从raw重绘。恢复Q为空且未就绪；新格式缺失/损坏拒该cut。
- 固定功能主分支始终为`feat/frontier-navigation`。本轮起点HEAD=`390bd6235c4e0dab434ce93f69dd11691e6a0dff`，产品基线=`11a26906c9831b000b09e2279ad7ef4ea514d903`。
- 产品开工前负责人重查最新main，正常merge进本功能分支；子工作从这个功能主分支分叉，正常merge回它，不cherry-pick，不重命名/替换功能分支。只为确实并行的几何、Nav2、UI工作使用短期工作分支，不给每小任务另建永久分支或准备scaffold。
- 2026-11-25只读远端：main=`66125807b7189ef28b250168df944a308eac9d33`，PR42 MERGED/非Draft，mergeCommit同main，UI head=`1adcaf8503c3c78a4267a2e2f6f5b74cddb291a1`。[PR42](https://github.com/Shunjun/senior-robot-control/pull/42)已满足当时UI布局依赖；开工仍重查，不把本工作树旧UI覆写最新UI。若届时依赖不在合入基线，UI阶段等待或只读设计，其余后端可推进。
- 不拿mock/静态配置/跳过测试冒称实际Nav2通过，不拿离线fixture冒称实车通过，不编造性能门槛。统一速度alpha/k与来源无关反向ramp单列独立工作，不是共享层必需前置条件。

## Review Focus

- 整数capture stamp、倒装束索引、缺少姿态或更新期望：沿用原exact-stamp/姿态负例，Task 1/7不得删除旧保护或放松断言。
- 存盘占住native单线程executor而不仅是Python锁：Task 1分离合格入口callback group，Task 4/7用真实生产跨进程S05验证。
- 多个Q在两个costmap周期之间替换、reset/disable和rolling origin：Task 5在S12同一A→B→撤销序列中验证旧实际渲染域。
- 不同anchor的free触发撤销及free区域旋转边界：Task 3的S04/S06分开原支持anchor与观测anchor，保护同格独立墙。
- 新权威层损坏或历史原档再次被选中：Task 4的S07/S08拒raw fallback和静默退回旧cut，保留原档且不复活撤销贡献。

---

## 源码证据、开工输入与冻结边界

路径均相对仓库根；`M=ros2/src/senior_mapping`、`N=ros2/src/senior_navigation`、`I=ros2/src/senior_interfaces`仅作正文缩写，末尾manifest使用完整路径。新增API为本计划决策，**不是现有能力**。

| 实际接点（390bd源码；UI为1adcaf85） | 现状及本计划决定 |
|---|---|
| `M/src/slam/bounded_scan_ingress.cpp:173`、`.hpp`、`slam_graph_node.cpp:272` | source/capture/时钟、exact TF、static receipt后才submit；gate和cached mount目前在后者锁内。先做资格事实，graph消费另走有界入口，不从graph commit推导逐scan。 |
| `slam_graph_node.cpp:488`及`main()` | capture持锁序列化，当前`rclcpp::spin(node)`默认执行器。只将Q移到gate前仍会被SaveToFile阻塞；Task 1必须给合格入口独立可调度callback group，graph/capture维持原串行owner。 |
| `M/senior_mapping/ros_runtime.py:106,303`、`native_session.py:340,553` | backend RPC全共用锁、session存盘锁，单`cut.json`8MiB槽；简化cut丢capture。Q独立订阅/最新槽，不复用`cut.json`或新增逐scan IPC请求队列。 |
| `M/senior_mapping/store.py:323` | 连续sequence且每次anchor恰增1；单批8192、总records65536。另加证据提交，不取消graph连续性，不靠重复anchor伪造新keyframe。 |
| `occupancy.py:242,397,469`、`references.py:108` | 只存occupied、free仅遍历已有track。不同anchor清除候选不一致未运行复现。改真实free tile、支持组及独立撤销，free触发anchor不替换被撤支持anchor。 |
| `native_session.py:333,358`、`navigation_bridge.py:42,115` | native raster优先且1/5秒刷新，桥另进程发布/map。显式owner fixed安装取代生产raster来源，保持唯一/map及定位权限。 |
| `references.py`、`archive.py`、`checkpoints.py` | 三个格式都是V1；旧PGM非occupied=205，未存完整native free。三者升级2；原档只读，首次迁移一次legacy绝对基线。 |
| `N/config/nav2.yaml` | global static+scan+inflation，local只有scan+inflation；allow_unknown=false，scan marking2.5m。两层改fixed+Q+inflation，local也铺fixed，参数必须从实际装配确认。 |
| `direct_navigation.py:77,93`、`agent/workspace.py:79` | free-only，R错误比较/下传为G。导航专用0/-1准入；比较G，下传R，航点仍free-only。 |
| `live_mapping_reference.py:175`、最新`useWorkspaceMap.ts` | 前者scan_points为空；后者scope已有MapRef/R/G，MapCanvas已有scanPoints绘制。沿用页面结构，补scan期限/来源，Q变化不进入tile scope。 |
| `vehicle_core/workspace_limits.py`、`mapping_navigation.hpp` | 通用读图16,777,216格/边65536；native导航另限1,048,576格、16384历史scan；不是同一个预算。保留生产导航1Mi格上限，不顺手扩大。 |
| `M/CMakeLists.txt:21`、`M/package.xml` | 前者锁2.6.10 EXACT，后者version_eq=2.6.8冲突；Task 0取部署证据，Task 1对齐原2.6.10契约，不擅自改Nav2发行版。 |

**地图范围证据与边界：** `bounded_native_raster`从历史scan计算有限extent；`native_session._apply`用该cut更新grid，`WorkspaceMap`只读manifest/tile，`WorkspaceReader._verify_free_target`明确拒图外；没有目标驱动reserve/extend RPC。Task 3在同owner按已合格射线/端点的dirty tile自然扩大fixed extent（只扩当前观测所需且总格数≤1Mi），不得因任意目标无限padding。global StaticLayer按/map尺寸覆盖；Q若落在当前global域外，管理侧原owner消费同一最新槽，在该预算内先扩**unknown边框**，不认证free，扩展确认前报告`coverage_pending`并保留现运动资格失格，不能静默截点还宣称全量可规划。该小型内部请求签名见C3；到预算上限报告`coverage_limit`、不删墙、不停止传全部Q，不能将截断视为就绪。**图外目标接纳、超预算导航窗口或额外规划owner未获批准**，Task 0给负责人范围证据，单列收敛，不阻止图内unknown及共享层任务。

### Task 0：取得真实部署与物理证据，收口契约（负责人/原装配owner）

**Files:** 只读`M/CMakeLists.txt/package.xml`、`N/config/nav2.yaml`、`N/launch/navigation.launch.py`、`ros2/src/senior_robot/senior_robot/{platform_runtime,real_runtime,dedicated_controller}.py`、`ros2/src/senior_sim/launch/simulation_motion.launch.py`、现sensor/odom profile；记录到`docs/operations/slam-backend.md`（D0）。不预造工具/服务。

**Interfaces:** 输出可复查的部署清单（包版本、源码/补丁SHA、编译器/架构、参数实际消费点、profile及证据来源），和`GeometryProfile`参数证据表；不是一个虚构运行时DTO。

- [ ] 读取当前远端main/PR42并核对UI基线；由负责人执行正常merge最新main进本功能分支后记录新的产品HEAD，再从它派发工作。确认本地文档commit仍保留。
- [ ] 向已有部署维护记录/目标离线安装镜像取`ros-humble-nav2-{costmap-2d,smac-planner,mppi-controller}`、slam_toolbox版本、补丁/源码对应、`ros2 pkg prefix`结果与Nav2实际配置快照；不要在本机安装或启服务来猜。核对原map_server/AMCL模式和live mapping启停交接，确认只读历史模式不冒充增量共享层。
- [ ] 取现有sensor协议（有限range_max、Inf/缺束/无回波含义）、采样频率/scan_time、exact stamp/静态mount、原odom来源/误差与同段记录；至少包含S01静止轮廓随车平移、S02对象自身运动后的再静止，以及实际时间同步证据。物理噪声/odom误差缺失时可做身份、Q、store/格式及几何确定性算法；**禁止冻结可靠稳定/运动数值门限或启用实车固定升级**。最小补证是同profile上述原scan+exact odom/TF记录和误差来源，无须视频/语义模型。
- [ ] 从部署包对应源码确认`Layer::updateBounds/updateCosts`、reset/disable及rolling StaticLayer ABI，和FootprintCollisionChecker/Smac/MPPI unknown路径；只能从本仓代码锁定的身份、字段和持久格式先冻结。Nav2版本缺失时Task 5仅能实现接口无关的状态逻辑、准入及配置草案，插件ABI和安装态S12–S14等待该最小版本证据，不写占位接口。
- [ ] 将地图extent及预算证据提交负责人：本轮承诺地图内unknown，任意图外目标需要新增产品边界决定；不自行加目标驱动扩图服务。记录版本/profile/误差缺口、可先完成项及取证责任人；提交`docs: record shared layer deployment evidence`供审核。

## 契约（Task 1先冻结；物理参数与Nav2 ABI由Task 0证据门控）

### C1：逐scan事实、当前Q及独立传播

新增`I/msg/QualifiedMappingScan.msg`：`string<=36 backend_boot`、`uint8 status`（0=INVALID、1=QUALIFIED、2=RETIRED）、`string<=64 reason`、原`BoundedRawLaserScan scan`（保留完整RawCaptureHeader整数stamp/received/sample steady、clock/capture generation、source epoch、raw/bridge GID、sequence）、`BoundedStaticTf static_lidar_tf_receipt`、`bool static_receipt_valid`、`geometry_msgs/Pose2D odom_base`、`geometry_msgs/Transform base_lidar`、`int8 raw_ray_angle_sign`（±1）、`string<=128 geometry_id`。`geometry_id`为已验证mount+扫描几何的身份，不是分类质量。未取得合格odom时不能标QUALIFIED。RETIRED不充当新scan、不更新截止。

`M/senior_mapping/shared_scan.py`定义immutable records：

- `ScanKey(backend_boot: str, bridge_boot_id: str, source_epoch: str, capture_generation: int, clock_generation: int, raw_gid: str, bridge_gid: str, sequence: int)`；capture原字段保存在`QualifiedScan.capture`中，不转换成仅float stamp。
- `Beam(index: int, origin_odom: tuple[float,float], endpoint_odom: tuple[float,float] | None, kind: Literal['hit','self','invalid','clear'], clear_limit_m: float | None)`；保留原束索引，倒装重排时索引显式映射。finite max-range若协议不明不能凭数字臆断hit/free，应按Task 0来源profile降该束未判定，不降低整条原source资格。
- `QualifiedScan(key: ScanKey, capture: dict, geometry_id: str, odom_base: Pose2D, beams: tuple[Beam,...], valid_until_steady_ns: int, host_boot_id: str)`；`decode_qualified_scan(message: object, profile: DeviceProfile, *, learning_view: LearningFilterView | None, host_boot_id: str) -> QualifiedScan`只执行一次原自体过滤和束转换，不跑分类。沿现`ScanAdapter`采用`DeviceProfile()`与`LearningClient`只读claim：`RosMappingBackend`从原`SENIOR_STARTUP_LEARNING_CLAIM`持有只读client，回调只取其immutable view，调用现`filter_self`及`external_feature_ranges`，不信任用户自造mask、不每scan读文件；关闭时释放client。缺学习模型沿原未知自体语义，不假设已校准。
- `LatestScanSlot.write(scan: QualifiedScan | None, *, retired: bool) -> None`、`read_after(previous: ScanKey | None) -> QualifiedScan | None`：adapter写、管理owner读，启动握手返回私有目录`qualified_scan_path`及session nonce。独立`latest-scan.json`原子replace，单记录含schema=1/nonce/完整capture和payload，≤8MiB，权限0700目录/0600文件，同uid/非symlink验证；读者先打开inode再读一份，避免path+digest跨代。不复用串行`cut.json`，不每scan发RPC，不用另一个永久队列。内部校验值在同一记录内，不把散列误当授权。写文件工作只保留一个进行中及一个可替换最新候选，慢I/O不挡ROS Q发布；source/reference退休立即失效。

新增`I/msg/CurrentObstacles.msg`，topic=`/senior/mapping/current_obstacles`：

| 字段 | 类型/含义 |
|---|---|
| `backend_boot`, `host_boot_id` | `string<=36`各一；steady deadline仅同主机boot可比较 |
| `map_id`, `reference_revision`, `map_generation` | `string<=128`, `uint64`, `string<=128`；MapRef与R；**不携G来续Q期限** |
| `capture` | 原`RawCaptureHeader`；Q身份就是完整ScanKey，不另造可循环计数 |
| `valid`, `reason`, `valid_until_steady_ns` | `bool`, `string<=64`, `uint64`；合格空Q可valid=true；失格空Q为false |
| `frame_id`, `points` | `string<=128`固定map；`geometry_msgs/Point32[<=2048]`，z=0；全部当前外部hit，不采样，不加历史/预测点 |

adapter先以同一qualified事实的原capture/odom调用原`LiveMappingReference.observe`，不依赖gate后的`ReadMappingPose`轮询保鲜；再用当前合格`LiveMappingReference.map_to_odom()`投影同一份QualifiedScan到map后先发Q，再把事实交最新槽；无定位不能冒用odom作map点。截止从原sample/receipt资格计算且不超过原0.5s，桥与插件无新消息也独立检查单调期限。发布Q使用KEEP_LAST=1、volatile，校验唯一owner/publisher GID、backend boot、host boot、MapRef/R和源capture完整身份；旧消息不能刷新截止。桥向原`LiveMappingReference`增加`set_current_coverage(valid: bool, reason: str | None) -> None`，原`qualified/state.navigation_ready`同时要求当次Q有效且所有hit落在已安装global范围；不新造导航就绪owner，不改变原raw运动保护。新R只接受新合格scan，不重投旧Q续期。退休/断流输出失格清空并撤贡献。DDS跨主机不能比较该steady字段，本计划部署限定现同机Nav2；远程消费者需额外时钟协议，未授权扩展。

**线程/锁边界：** native合格入口、static receipt/source/clock检查和TF订阅置于一个串行callback group，使用一个受生命周期管理的独立executor线程；graph/capture留原owner执行器。入口只读graph侧原子发布的immutable cached mount/几何见证，不跨线程访问`lasers_`/mapper；首次见证尚未建立时不发布合格map Q，由原graph初始化后提供，拒绝未核几何。入口先发布qualified事实，再将scan以≤8项有界队列交原graph owner；graph gate关闭可以丢弃其建图候选，但Q不断。所有graph/cache写仍在原锁内；新线程加入/退出有join，不能把原非线程安全StaticLidarTfIngress跨组随意读取。adapter的Q回调/失效timer不await通用request锁，重存盘工作不占该callback。两个ingress构造函数在原参数后增加`rclcpp::CallbackGroup::SharedPtr group`，subscription options/timer显式绑定它；graph读`source_active`也只读原子资格见证，不能跨组调用mutable ingress方法。几何不在native ingress锁内运行。Task 7测executor调度/CPU竞争；不是仅声称“gate前”即完成。

### C2：轻量几何（无持久写权限）

新增`M/senior_mapping/scan_geometry.py`：

- `GeometryProfile`字段：`range_sigma_m`, `odom_translation_error_m`, `odom_yaw_error_rad`, `max_scan_distortion_m`, `cluster_gap_m`, `cluster_gap_per_m`, `stable_residual_m`, `motion_residual_m`, `min_common_fraction`, `min_independent_frames`, `min_span_s`, `max_gap_s`。必须finite、非负/比例合法、motion带严格高于stable带，附`profile_id/evidence_digest`；这些物理值由Task 0证据生成，缺失返回不可判定，不能填视觉tracker旧参数或把3hits当稳定门限。参数归现`M/config/slam.yaml`的shared_geometry条目（缺证据不提供物理默认值）；adapter启动用仓库已有PyYAML读取安装配置、验证后在start握手返回`geometry_profile`及limits，管理NativeSession只读接收一次，记录profile/evidence_digest；不另起参数服务或重复配置owner。
- `GeometryLimits(max_frames=8, max_clusters=256, max_tracks=256, max_candidates=8, samples_per_contour=32)`：这是**有界初始工程容量选择、未经性能验证**，不是物理/速度保证；分类超cap为unresolved，全量2048 hit仍进Q。
- `GeometryTracker.update(scan: QualifiedScan, *, profile: GeometryProfile | None) -> GeometryEvidence`；`reset() -> None`。`GeometryEvidence(key: ScanKey, evidence_revision: int, stable: tuple[ContourEvidence,...], moved_support_ids: tuple[str,...], unresolved_reason: str | None)`；`ContourEvidence(local_track_id: int, beam_indices: tuple[int,...], odom_surface: tuple[tuple[float,float],...], first_key: ScanKey, last_key: ScanKey, common_fraction: float, max_residual_m: float, support_ids: tuple[str,...])`。support_ids由唯一fixed owner回绑：`bind_support(local_track_id: int, support_ids: tuple[str,...]) -> None`；纯算法不分配永久地图ID。
- 有限角顺序分簇，局部bin的最多8候选、共同可见表面双向残差与有界采样贪心关联；不用质心位移单独判断。状态`unresolved → stable_candidate → stable_confirmed`，显著运动变`motion_evidenced`并清旧稳定窗口；再稳定按完整新窗口准入。split/merge、遮挡歧义、gap/误差不足reset候选，不强行传递支持撤销身份。source/ref切换清短期关联；loop correction只改map投影，不进入odom运动判断。

### C3：原owner的fixed证据与修订

扩`M/senior_mapping/evidence.py`定义（不再增加第二个fixed模块/owner）：

- `FixedWatermark(map_ref: MapRef, reference_generation: str, backend_boot: str, graph_sequence: int, anchor_digest: str, fixed_revision: int, evidence_key: ScanKey | None, evidence_revision: int)`；graph sequence与fixed revision各自单调，不互相代替，anchor_digest涵盖实际ID/pose集合。
- `StableSupport(support_id: str, anchor_id: str, local_surface: tuple[tuple[float,float],...], first_key: ScanKey | None, last_key: ScanKey | None, provenance: Literal['geometric','legacy','legacy-derived'])`；同格可多support，但每组关联摘要最多32表面采样，固定支持保留本scan已确认的全部端点/格（单组≤2048，聚合索引计入65536 cap），不能以关联抽样截断固定墙，重复观察复用/更新支持。重见无法可靠关联时只靠真实free清旧贡献，不新增无限永久轨迹。
- `FreeTile(anchor_id: str, tile_x: int, tile_y: int, cells: bytes, evidence_revision: int)`；32×32局部格，bytes只存有效free/unknown及必要局部废止代次；短期计数仍有界稀疏状态。occupied写入覆盖的free立即作废，该区域以后只有新合格free可再次成立。
- `FixedDelta(key: ScanKey, evidence_revision: int, upsert_supports: tuple[StableSupport,...], revoke_support_ids: tuple[str,...], free_tiles: tuple[FreeTile,...])`；运动撤销原因新增`motion_evidenced`，不能伪装`observed_free_space`；触发free的观测anchor独立保存，不覆盖support原anchor。
- `FixedSnapshot(watermark: FixedWatermark, grid: WorkspaceGrid, supports: tuple[StableSupport,...], free_tiles: tuple[FreeTile,...])`；dense cells=0/100/255，wire tile=0/100/-1；snapshot不含Q/短期tracker。持久化还保存legacy绝对三态base（存在时）、局部覆盖/撤销后的绝对结果和支持引用，不无限累积tombstone。

`ActiveMap.commit`保留graph验证职责；新增`ActiveMap.apply_evidence(source: object, *, expected_graph_sequence: int, expected_anchor_digest: str, delta: FixedDelta) -> FixedSnapshot`调用`MapReferenceStore.commit_evidence(binding: OccupancyIngestBinding, *, expected_graph_sequence: int, expected_anchor_digest: str, delta: FixedDelta) -> FixedSnapshot`。只接受当前已验证anchor cut，允许同cut多次证据，拒旧boot/source/重复/倒序证据；source sequence可因最新槽跳帧，gap必须重启稳定窗口，graph连续性不变。graph校正先reproject再使新delta绑定新cut；无可用anchor时暂缓fixed升级，Q独立。

`ActiveMap.ensure_observed_extent(source: object, *, expected: FixedWatermark, bounds: tuple[float,float,float,float]) -> FixedSnapshot`仅供桥对**当次合格Q**覆盖申请；沿原地图owner扩大unknown边框，≤1Mi格、不改MapRef/R；重复无变化不递增fixed修订。管理侧直接从独立latest-scan槽取得同一QualifiedScan，以当前已验证anchor/map←odom投影其bounds，复核ScanKey/期限后调用；不发反向`map.backend`RPC，不开放用户坐标/导航目标padding端口。通过原owner下一轮fixed安装确认后才能声称该Q已进入global覆盖；等待期间Q仍全量发送且桥报告pending，不新增Q阻塞依赖。

预算：总支持/索引/waypoint记录沿现65536聚合cap，单次delta≤8192项；free anchor tiles总量≤1024（32×32，对应生产1Mi格预算），几何短期cap见C2；绝对legacy dense base最多1Mi格，不强制拆成百万CellBinding。独立支持不能为压缩而合并丢失撤销关系。dirty更新正常O(ray格步数+dirty格)，reprojection只重算受影响anchor块；达到record/tile/字节cap时停止新增fixed升级并上报`fixed_capacity`，保留旧墙，仍接受可安全表达的撤销/真实free，Q不受影响。捕获一个immutable cut后解锁做I/O，fixed冻结期间只保留1份最新scan候选，gap重开稳定窗口，不把丢帧当连续。8MiB用于传输，archive/checkpoint原64MiB总字节边界仍验证；这些cap不是实测内存承诺。

### C4：fixed安装、存档及UI期限

`NativeNavigationBridge.install_fixed(cut: FixedSnapshot, *, graph_cut: dict) -> None`：owner通过`map.backend.install_fixed`发送私有`fixed.json`只读transfer（≤8MiB、同uid/非symlink、digest；接收确认后释放，最多一个in-flight及一个合并候选）。内容包含C3完整watermark、`width/height/resolution/origin/cells_b64/grid_id`及该anchor cut的定位基础；不传所有短期track或每scan全anchor。adapter复核boot、当前MapRef/R、graph cut/digest、fixed/evidence水位，G按geometry+cells内容身份，installed ACK原样返回watermark/G。owner未收到ACK不宣布导航图就绪；新fixed撤销尽快发布，不沿旧1/5秒native raster刷新节流。graph/raw诊断服务可保留且明确不作生产fallback；唯一/map owner检查与stock图remap保留。

内层`references.FORMAT_VERSION=2`、外层`archive.BUNDLE_FORMAT_VERSION=2`、`checkpoint.json.version=2`。新payload保存完整watermark、absolute O/F/U、anchor局部support/free证据和算法profile来源；raw graph标`localization_only`。legacy dense base保留局部cell provenance；新几何支持替换其对应局部来源，不能把整片legacy墙绑定一个短期track一起撤。恢复后支持关联仅在当前局部候选中由owner查持久摘要，歧义不撤，不能靠重启分配新ID绕过已知撤销。V1解析仅由明确迁移入口调用：`migrate_legacy(snapshot: SlamSnapshot, *, legacy_grid: WorkspaceGrid | None, native_legacy_grid: WorkspaceGrid | None) -> FixedSnapshot`放原`references.py`，只有实际V1+未迁移catalog cut可接受一次native legacy图，叠加原removals后原子创建当前副本；V2不调用。manifest/checkpoint checksum绑定所有权威成员；失效拒cut，不默退原档/重绘raw。旧加载器因版本!=1拒新档，实际旧loader验证此断言。

已核实`vehicle_core/ipc.py:27`的state.get帧上限只有8192字节；不能把2048点塞进原状态RPC或放宽运动IPC上限。UI保留`scan_points`最多96点作为**明确标示的当前扫描预览**，Nav2始终消费ROS Q全部≤2048点。预览按原束角分96桶、每桶选最近有效hit，返回`hit_count`与`preview_count`，不冒称完整轮廓或保证每个小障碍均在预览。完整Q与原始束仍在C1独立槽/ROS路径；不把预览作导航输入。新增嵌套`scan_layer`字段到现state/snapshot wire（`contracts/src/senior_contracts/wire.py`）：`ScanLayerPayload(map_ref: MapRefPayload, map_generation: ShortText, backend_boot: ShortText, source_id: ShortText, sequence: int, sampled_at: float, expires_at: float, remaining_s: float, hit_count: int, preview_count: int, display_mode: Literal['preview'], valid: bool)`，source_id为完整源元组的hash（不含sequence），sampled_at/expires_at为现manager同机monotonic秒；remaining_s在现`main._collect`采样发送时由原截止减本机now计算，范围[0,0.5]，不是续签。ROS原完整ScanKey仍在内部资格链。管理端每次透传重算剩余期但**不**把snapshot采样时间当scan时间；过期置valid=false及空points，旧source/ref不续。

cloud仅维护显示缓存年龄：在`cloud/src/cloud/snapshots.py`按接收monotonic记录Q的本地截止=now+remaining_s，同backend/source/sequence重复只能取更早截止，`SnapshotCache.get/_view`扣除已驻留时间并在过期清preview/置valid=false，不沿5秒整体snapshot新鲜度续Q；该缓存没有地图写权限。新浏览器收到缓存也不能复活已过期Q。浏览器不比较车端monotonic与本地performance，只用cloud已扣除驻留后的remaining_s启动本地timer，同Q重复只取更早deadline，stale/断连/旧R立即清空；缺scan_layer的旧状态不冒充本轮合格Q。

现`RobotSnapshot`只有wall-clock collected_at及cloud last_synced_at/stale，不能减不存在的snapshot.sampled_at或混减wall/steady。manager→cloud→browser的单程网络延迟未有可信时钟上界，Task 6必须记录这一显示延迟边界并验证现envelope失效规则；本计划保证各跳驻留不续期，不能声称跨主机物理0.5s精确同步。Nav2同机原absolute deadline不受此UI边界影响。新增`useCurrentScan(snapshot: RobotSnapshot) -> readonly MapPoint[]`集中显示判断；fixed的MapRef/R/G tile scope不含Q。

## 第一部分：共享合格scan事实与轻量几何

### Task 1：独立逐scan输入与Q（原native ingress + ROS adapter owner）

**Files:** 下方manifest的T1；不修改`ReadGraphCommit.srv`，保留原capture commit历史。**Consumes:** 原BoundedRawLaserScan、exact TF/static witness和选定DeviceProfile。**Produces:** C1及C4 latest-slot握手；Q无需C2/C3完成即可更新，但无合格reference时为失格。

- [ ] 在新增`M/test/test_shared_scan.py`写`test_original_capture_and_retired_scan_do_not_renew_deadline`：完整整数stamp/GID/epoch不变，重复/退休不续期、有效空与失格空分开；使用已有exact-stamp/epoch输入，不造简化float身份。先运行应因新接口不存在失败。
- [ ] 扩原`M/test/probes/probe_bounded_scan_ingress.py`：同一来源在graph gate关闭且capture进行时继续供scan，计数QUALIFIED消息；cached geometry不匹配、静态receipt退休均不能产合格事实。探针验证原native入口，不以Python mock顶替。先在安装态观察新增断言失败，再改入口。
- [ ] 实现C1双消息及有界executor交接，精确采样odom/base和mount签名；static receipt/TF/clock回调在入口同组串行，graph只消费有界队列和发布immutable几何见证；加入关闭join及退休清空。保存不能持入口组锁。
- [ ] 实现`decode_qualified_scan`：复用`senior_perception.self_filter.filter_self`现语义，保持倒装角序/原beam索引；不得将Shapely coverage全图运算复制到每scan。桥先投影/发Q，latest file写入独立合并I/O。对齐mapping manifest的2.6.10并补原perception运行依赖，不引新库。
- [ ] 未来运行`python3 -m pytest ros2/src/senior_mapping/test/test_shared_scan.py ros2/tests/test_scan_exact_stamp.py ros2/tests/test_scan_epoch_live.py -q`；安装环境运行原bounded ingress probe（由其现`arguments()`及CMake注册命令启动，不猜CLI）。预期新断言通过，原missing pose/旧capture拒绝保护仍过；缺ROS的skip记未验。
- [ ] 审核线程数/所有锁与shutdown，提交`feat: publish qualified scan snapshots independently of graph intake`；负责人亲阅后合入功能主分支，冻结C1后允许Task 2/5/6隔离并行。

### Task 2：有界几何证据（原mapping感知职责，无地图写权限）

**Files:** manifest T2。**Consumes:** C1 QualifiedScan、Task 0证据profile。**Produces:** C2 GeometryEvidence；sole fixed owner回绑support IDs，算法不写store/archive。

- [ ] 新`M/test/test_scan_geometry.py`用S01/S02的一段真实轮廓生成独立ego/对象位姿：`test_ego_translation_preserves_isolated_stability`、`test_motion_revokes_bound_support_and_requires_new_stability_window`；第一项断言补偿后孤立物可stable，第二项只撤原支持、再稳定须新完整窗口。另从同序列插入实际遮挡/歧义帧，断言unresolved且Q原hit不减；不机械参数化扩矩阵。
- [ ] 运行该文件，预期新接口缺失失败；实现C2线性分簇/局部候选/共同可见残差，阈值未取证时显式unresolved，不以“没测到移动”升级。
- [ ] 同一输入把source gap和R更换插入原段，验证reset；loop closure只改map anchor不改变odom比较。达到C2容量只停止新固定证据，N=2048全部hit仍在Q。
- [ ] 未来运行`python3 -m pytest ros2/src/senior_mapping/test/test_scan_geometry.py tests/unit/test_perception.py -q`；保留原缺独立位姿/语义诊断负例，不将语义分类引入新固定准入。预期所有有效旧断言与新用例通过；物理门限缺失不能勾可靠稳定验证。
- [ ] 提交`feat: derive bounded motion and stability evidence from qualified scans`并独立审核；可先合纯算法，激活profile依赖Task 0证据。

## 第二部分：权威fixed三态与存档（同一原mapping owner连续收口）

### Task 3：同anchor证据提交、free图块和独立支持（原ActiveMap/store owner）

**Files:** manifest T3。**Consumes:** C1/C2及真实graph cut。**Produces:** C3 FixedSnapshot；Task 3与4由同一责任人顺序执行，不能各自在分支发明格式。

- [ ] 扩`tests/unit/test_occupancy.py`和`M/test/test_active_map.py`的S02/S04/S06：多份独立支持同格，运动只撤对象组，墙仍100；不同anchor合格free按原次数+belief+跨度清旧支持，遮挡期间不清；旋转free边界不外溢为free；旧free在occupied后失效，撤对象后无新free为255。新功能前应失败，不删除原稳定累积/缺参数/重复frame保护。
- [ ] 保留graph `commit_ingest`连续+新增anchor校验，新增C3 `commit_evidence`及`apply_evidence`允许同cut多次证据；graph_boot/anchor digest旧值拒绝，处理per-scan gap不伪造keyframe。`OccupancyMaintainer`只有明确stable证据可新增fixed，旧unknown_static不能作为新生产旁路。
- [ ] 扩evidence/store/references现结构承载多支持与32×32 free tiles；真实hit前射线积分，遮挡后/缺束/未知无回波不清。hit清旧free代次；motion按支持ID撤销，free触发与原支持anchor分开；active snapshot和renderer输出同一绝对三态。
- [ ] 沿`ProductionMappingRequests._observe`现0.05s管理循环，在`NativeSession.ingest`即使graph next返回PENDING也读取latest槽、处理同anchor evidence；不因现early-return停fixed更新，不另建长期线程/worker。过载跳帧重启稳定窗口并记录，Task 0取scan频率后验证现owner节奏能提供足够独立有效帧，不能靠重复帧补数。实现C3自然观测extent的同owner复核（管理侧读独立latest槽、adapter核对/map覆盖，不开反向RPC/新服务/用户目标入口），1Mi预算外报告容量并撤就绪声明；原图不裁墙。dirty tiles增量更新，anchor校正更新受影响块，65536/8192/1024及字节cap不丢独立支持、不阻断Q。
- [ ] 未来运行`python3 -m pytest tests/unit/test_occupancy.py ros2/src/senior_mapping/test/test_active_map.py ros2/src/senior_mapping/test/test_map_references.py -q`；预期同anchor刷新、原连续性、S04/S06及容量断言通过。原不同anchor清除冲突是源码风险，先见新增断言失败再修，不宣称已复现生产故障。
- [ ] 提交`feat: maintain authoritative anchored tri-state occupancy`并审核数据职责/复杂度；不切给第二保存owner。

### Task 4：权威安装、V2 checkpoint/bundle及真实V1迁移（同一Task 3 owner）

**Files:** manifest T4。**Consumes:** C3完整cut及原graph capture。**Produces:** C4安装ACK和所有恢复/导出路径一致的V2；不将Q写进长存档。

- [ ] 扩`M/test/{test_map_archive,test_checkpoints,test_map_references}.py`的S07/S08：由**当前V1真实writer**产生checkpoint+bundle，保留原档hash，native旧三态仅迁移一次；运动/真实free撤旧hit→保存→重载/导出不复活。V2损坏拒cut、实际旧V1 parser拒V2，不能用手写“含free”的假旧fixture。先看到现格式/回流断言失败。
- [ ] 实现C4完整watermark fence：gate/drain只冻结graph和fixed cut，固定immutable快照含已提交撤销；capture核对graph/anchor+fixed/evidence水位后I/O；未一致则拒，不取消旧校验。新scan在独立Q路继续，fixed候选仅1份合并，存盘后重开丢帧稳定窗口。
- [ ] 升级三个版本、archive/checkpoint校验和及PGM（0/100/255分别真实free/occupied/unknown）；同步`workspace_export.py`、`operations.py`、`slam_adapter.py`的显式导入/历史/导出。native raw明确localization_only；V2渲染、restore、regrid/选图无raster fallback。旧导航资格按原证据保留，不强制所有旧图重校准。
- [ ] 显式`install_fixed` transfer/ACK替换native导航grid路径，桥的内容全取owner，仍用原graph/odom配对授权reference；没有0格不自动pending，不能伪造free。`mapping_navigation.hpp`的raw raster仅用于诊断/一次迁移，生产调用拒绝回流。迟到fixed cut和reference退休不能覆盖新图或续Q。
- [ ] 在新增`ros2/tests/test_shared_layers_process.py`写S05/S07/S09，用真实管理`NativeSession/ActiveMap`、真实adapter IPC和native进程/临时catalog：无新keyframe同时save，Q继续更新/撤销，fixed证据revision可变；源/参考退休清Q；ACK/map/导出/重载一致。不只扩native child探针然后声称已跨生产owner。
- [ ] 未来运行`python3 -m pytest ros2/src/senior_mapping/test/test_map_archive.py ros2/src/senior_mapping/test/test_checkpoints.py ros2/src/senior_mapping/test/test_map_references.py ros2/src/senior_mapping/test/test_active_map.py -q`；安装环境再运行`python3 -m pytest ros2/tests/test_shared_layers_process.py -q`，不得以skip作通过。复用`M/test/probes/probe_production_session.py`检查原capture/shutdown回归。预期S05/S07/S08/S09均有真实owner记录，旧cut不回灌。
- [ ] 更新map-bundle/slam-backend文档、提交`feat: persist and install authoritative map cuts without raw fallback`；同一owner自查归档/checkpoint并审核后merge。

## 第三部分：Nav2当前层与unknown接入

### Task 5：薄Q层、正确准入及部署Nav2验证（原navigation/management owner）

**Files:** manifest T5。**Consumes:** C1 Q与C4唯一/map，Task 0部署ABI。**Produces:** 两costmap正确撤格/膨胀及保墙；管理导航free/unknown准入，R/G正确消费。

接口：新增`senior_navigation::CurrentScanLayer final : public nav2_costmap_2d::Layer`，头位于`N/include/senior_navigation/current_scan_layer.hpp`，实现`N/src/current_scan_layer.cpp`、导出`N/current_scan_layer.xml`。覆盖部署ABI的`onInitialize()`、`updateBounds(double,double,double,double*,double*,double*,double*)`、`updateCosts(Costmap2D&,int,int,int,int)`、`reset()`、`isClearable()`；`activate/deactivate`若该部署Layer ABI提供则按Task 0查明签名实现，不在版本缺失时编造override。内部只有latest Q、本周期immutable Q、上次实际渲染域、一次待清域、source/R/host身份及deadline；KEEP_LAST=1，2048点，无tracking/分类/持久地图。bounds与costs使用同一周期快照；普通reset不能丢旧域。

管理接口：`WorkspaceReader.verify_navigation_target(map_ref: MapReference, grid_id: str, pose: PoseReading) -> None`，`WorkspaceTargetPort`增加同签名，`NavigationStateReading.map_grid_id: str | None`；与`verify_free_target`共用私有tile/identity读校验，但前者仅cell in(0,-1)，后者仍cell==0。`DirectNavigation`比较state G，下传`start_poses(execution_id, map_ref, live.map_generation, (pose,))`中的R；保留localized/safety/Home/权限及最终generation复核。

- [ ] 用现`senior_control/tests/test_direct_navigation.py` free目标输入明确R≠G，并新增同图unknown目标S10、复用occupied/free-only航点/旧身份拒绝输入S11；先验证现实现失败，再实现上述薄端口，不能绕过持久任务/IPC/RouteService直接发Nav2 goal。
- [ ] 新增`N/test/test_current_scan_layer.cpp`验证S12同一A→B→撤销序列的pending/last-rendered域及255→254命中，不存第二份fixed；先在真实Nav2库单元实例看到旧行为不满足，再实现插件。数值数组mock不算部署通过。
- [ ] 插件updateBounds合并上次**实际渲染**与本周期Q域、失效域；master重置该域后fixed StaticLayer重铺，插件只mark当前hit为lethal，inflation重算；不用写free擦墙、不用普通max压过unknown。多个新消息、reset/disable、rolling origin移动在这同一S12序列插入，保留一次清旧域机会；失效current=false。
- [ ] 两costmap plugins顺序改static_layer/current_scan_layer/inflation_layer，移除原/scan ObstacleLayer重复历史贡献；local rolling也用/map，`use_maximum=false`，保持track_unknown_space=true、footprint和原padding；allow_unknown=true进真正Smac配置。不增加MPPI伪allow_unknown参数，不绕足迹。全量Q测距遵守Task 0协议，不沿用2.5m裁点；global域覆盖C3确认，local窗口外由global承接。现`simulation_motion.launch.py`的map_yaml静态模式沿原controller_params临时overlay明确保留原static/scan/inflation装配（含原scan资格），仅live mapping选shared插件；不额外造永久配置模板。该兼容静态模式不宣称可更新共享fixed，选择可更新旧图需走Task 4原mapping生命周期交接。Task 0若确认硬件仍使用独立fallback，必须先列出它的实际参数消费/交接证据，由负责人收敛需改的原装配路径；未收敛前不能全局启shared配置而让旧静态导航丢障碍输入。
- [ ] 在原`ros2/tests/test_navigation.py`安装态probe扩S12/S13/S14：同部署库两costmap实际取样A→B→撤销含inflation、固定墙；同图unknown通道让真实Smac和MPPI规划/控制离线fixture，记录路径、controller请求和确定的离线执行模型位置，不声称实车。mixed unknown/lethal footprint按原审计风险构造早边unknown/后边lethal及同输入边序对照，中心不依靠lethal挡住候选；真实FootprintCollisionChecker/Smac/MPPI不得穿已知障碍。**先复现**，未失败仅证明该版本组合；失败记录版本/调用栈/最小输入，阻止unknown发布，向负责人提交最小上游修复或可验证版本选择，不自动扩大产品补丁范围、不关足迹/unknown改free。
- [ ] 未来先`python3 -m pytest senior_control/tests/test_direct_navigation.py ros2/tests/test_navigation.py -q`验证准入与静态装配；再在Task 0安装态离线环境`colcon build --base-paths ros2/src --packages-up-to senior_navigation senior_robot`、`colcon test --packages-select senior_navigation`及`colcon test-result --verbose`。在隔离域设置`NAV2_TEST_INSTALLED_LAUNCH=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187`运行原安装态probe；187只是示例，先确保域无其他参与者，不能连车；S12–S14必须实际执行不skip。包XML/插件导出/安装prefix都从install目录加载验证。
- [ ] 更新navigation README说明部署版本/unknown边界与未验项，提交`feat: consume current scan layers and admit in-map unknown goals`并审核。mixed footprint失败或版本缺失时准入代码可评审，unknown实际交付保持阻塞；其他后端可推进。

## 第四部分：最新UI consumer与整链/资源验证

### Task 6：实际Q、独立期限与最新页面接线（原state/协议/cloud缓存/UI owner）

**Files:** manifest T6（基于已merge PR42最终布局）。**Consumes:** C1/C4 Q及同源fixed G。**Produces:** C4 `ScanLayerPayload`与`useCurrentScan`；不改变页面结构，不让UI抽样进入导航。

- [ ] 用最新`web/src/api/vehicleState.test.ts`、`features/map/map.test.tsx`和`features/devices/devices.test.tsx`的既有snapshot输入扩S15：G不变/Q更新，tiles读取计数不增加；相同Q重复snapshot不续期限；旧R/旧source、断连、Q到期清空，固定G仍在。以2048点输入断言hit_count=2048、preview_count≤96且标签明确预览；末束若为其角桶最近hit应出现；不声称抽样全量；缺姿态/期望更新等旧回归仍跑且不削弱。
- [ ] 在`LiveMappingReference.state`接桥已验证Q；C4嵌套scan_layer及最多96 preview scan_points沿现local_state/state_ports/ipc_state/snapshots/snapshot_state/main和contracts唯一wire透传；`main._snapshot_change_key`加入Q source/sequence/期限，不能只看前64点抑制更新。状态采样不得重签scan有效期，退休即失格空层；serialize完整真实state.get后必须≤8192字节，预览可按剩余字节预算再少取并如实标preview_count，不能增IPC帧上限或挤掉原安全字段。cloud同Q缓存截止只取更早并按驻留失效，增量改原snapshot缓存，不能让重连取缓存复活Q。
- [ ] 实现`web/src/features/map/useCurrentScan.ts`集中C4过期规则，`ControlSurface.tsx`和`TaskList.tsx`用同hook传MapCanvas已有scanPoints；`MapCanvasProps`增加`scanPreview?: { shown: number; total: number }`，来自同一scan_layer的preview_count/hit_count；MapCanvas明确显示“当前扫描预览 preview_count/hit_count”，区分当前点与fixed底图现样式，必要的标识/绘制改动归该组件。`useWorkspaceMap`保留MapRef/R/G scope，不把Q或snapshot整体作为tile effect依赖。
- [ ] 未来`python3 -m pytest senior_control/tests/test_agent.py senior_control/tests/test_mapping_production.py senior_control/tests/test_mapping_catalog_wire.py cloud/tests/test_agent_snapshots.py -q`与现snapshot/wire回归（Task 7输入表指明）验证字段跨链；`npm --prefix web test -- --run src/api/vehicleState.test.ts src/features/map/map.test.tsx src/features/devices/devices.test.tsx`，再`npm --prefix web run typecheck`。若包未装，记录缺口交负责人安排已授权环境，不能本轮安装。
- [ ] 提交`feat: display qualified current scan with independent expiry`，负责人在PR42结构亲阅审核后合功能主分支。Q刷新不拉fixed tiles且真实UI收到当前Q才可完成S15。

### Task 7：生产闭环、安装态离线及四核资源收口（负责人协调，原各owner修自己模块）

**Files:** manifest T7。**Consumes:** 已合T1–T6及Task 0版本/profile、同一S01–S16输入集。**Produces:** 分层证据、失败边界、测量报告；不是一套新测试矩阵或新生产调度框架。

- [ ] 在**不接车**的既有同版ROS2离线构建/安装环境完成协调rebuild：`colcon build --base-paths ros2/src --packages-up-to senior_mapping senior_navigation senior_robot`，核对install前缀生成消息typesupport、插件XML及.so、Python包导入与CMake/manifest版本相符。`colcon test --packages-select senior_mapping senior_navigation senior_interfaces`及`colcon test-result --verbose`须无失败；本机无/opt/ros不假装能跑，移交该安装环境而非擅自安装。
- [ ] 实际管理owner/adapter/native进程的S05–S09使用临时catalog和隔离IPC，记录/map唯一发布者、capture身份/水位及archive字节/hash；包括map_server/AMCL静态模式原启动不回归，切live mapping由原生命周期交接owner，不能把静态模式废掉来让唯一/map检查过。新增`ros2/tests/test_shared_layers_process.py`承担跨进程场景，原native probe只是支撑。
- [ ] 对照下表逐项填“unit / owner跨进程 / 部署Nav2离线 / 目标四核 / 实车未验”，列真实命令/版本/产物；S13离线fixture的控制输出不能当真实轮速。若发现关键需求缺验证，先报告负责人缺口，不自行扩任意新场景。
- [ ] 原benchmark只有纯算法trace，不能直接冒称整机测量；扩其`--system-trace PATH --output PATH`读取新增process probe/安装态fixture产出的同机JSONL，`summarize_system_trace(path: Path) -> dict`按原capture关联实测事件与进程资源采样，不新增生产调度框架。保持原纯算法模式及输出，区分tracemalloc临时内存与实际进程RSS/PSS。复用同一输入、相同地图/profile/四核目标设备，基线与新链各一次完整重放，并发SLAM、定位、共用控制、MPPI、UI、存盘；在原probe/fixture记录器输出版本/profile/host boot、capture键、阶段steady_ns及进程PID/资源采样；记录本身开销单列，不为丢trace续Q。按capture关联记录capture→Q→global/local→控制消费p50/p95/p99/max、分类/owner锁等待、queue高水位/丢帧/失格、各进程CPU（单核100%口径）、RSS/PSS、I/O/上下文切换/温度降频。单列loop重投影/save/reload峰值、raw物体移走后pose/匹配/错误loop与重定位质量；不是扩大为native graph清洗。
- [ ] 资源仅沿原0.5s源资格及既有控制时限验退化，新增CPU/延迟预算依据实基线由负责人收敛；不编Hz/CPU百分比/RSS成功阈值。未取得四核记录则S16未验，不能靠开发机benchmark完成；定位若恶化，保留原失格保护并提交局部scan/cache/constraint证据另派，不默认全graph修复。
- [ ] 更新`docs/operations/slam-backend.md`和navigation README记录最终证据/版本/边界，提交`test: qualify shared obstacle layers across production owners`。负责人整分支审核实际A/M/D并集、任务生命周期及回归结果；未经新授权不推送、建PR或发布、不启动实车。

## 验收清单：单一输入序列、预期、现有位置

每行只规定该需求的一段实际序列；覆盖条件嵌在原序列，不机械扩参数矩阵。纯算法unit先跑失败→最小实现→再跑通过；同编号的跨owner/安装态证据必须另外给出，不能用unit替代。

| ID / 负责Task | 输入与必须断言 | 现有位置与未来归属 / 证据级别 |
|---|---|---|
| S01 / T2 | 同一孤立静止轮廓，车平移，原capture/可信odom补偿；几何可判稳，非墙连接/语义条件，补偿不可信时不升级 | `tests/unit/test_perception.py`外参/缺位姿负例；新`M/test/test_scan_geometry.py`；unit+物理profile证据 |
| S02 / T2/T3/T4 | 同轮廓稳定→移动→再稳定；撤原支持，当前实际hit一直在Q，再稳定完整新窗口可重新入图 | `tests/unit/test_occupancy.py`稳定累积有效用例保留；新geometry、`test_active_map.py`及生产process串成同一输入；unit+owner |
| S03 / T1/T5 | 临时点离开实际覆盖/遮挡后Q撤，扫描外已稳定墙保留；不凭半径内仍有track续占据 | 现self_filter/coverage与`ros2/tests/test_scan_epoch_live.py`支撑；`test_shared_scan.py`、Nav2 S12序列；owner+Nav2 |
| S04 / T3 | 原occupied先被遮挡，后真实free射线重访；前段不清，后段按次数/belief/跨度清，缺束/不明无回波不证明free | `tests/unit/test_occupancy.py:test_occlusion_does_not_clear/test_single_miss_preserves_wall`及free用例；unit+owner |
| S05 / T1/T4/T7 | 无新keyframe且同时save，逐scan Q A→B→失效不等gate/RPC，fixed证据可在同anchor更新 | 现`M/test/probes/probe_bounded_scan_ingress.py`、`probe_production_session.py`；新`ros2/tests/test_shared_layers_process.py`真实manager/adapter/native；跨进程 |
| S06 / T3 | anchor校正/旋转重投影，墙和对象独立支持同格，撤对象保墙；free边界保守，不同anchor free不伪造原支持anchor | `M/test/test_map_references.py/test_active_map.py`、`tests/unit/test_occupancy.py`；unit+owner |
| S07 / T4 | raw保留旧hit，权威对象已撤；保存→重载→导出/重grid→选图一致不复活；损坏V2拒cut | `M/test/test_map_archive.py/test_checkpoints.py/test_map_references.py`、`workspace_export`现测试；新process；unit+owner |
| S08 / T4 | 用真实当前V1 writer形成旧档一次迁移；原档hash不变，更新副本再重载不复活、不再调用native legacy绘图 | 同上archive/checkpoint测试、现V1加载函数；实际旧loader拒V2与新migration入口验证；owner/安装态 |
| S09 / T1/T4 | 旧source/clock/R退休，迟到Q/分类/安装ACK不得续期或跨图显示；恢复Q为空 | `ros2/tests/test_scan_epoch_live.py/test_scan_exact_stamp.py`、`M/test/test_map_references.py`；新process及UI同输入；owner |
| S10 / T5 | 图内unknown点击、R≠G；当前G准入，持久task/IPC传R，实际Nav2接收；图外仍拒 | `senior_control/tests/test_direct_navigation.py`、`ros2/tests/test_navigation_reference.py`及原RouteService测试；unit+owner |
| S11 / T5 | 优先沿现occupied/旧G/权限输入：occupied拒；free航点可创建，unknown航点拒；Home/定位/取消终态保护不变 | `senior_control/tests/test_direct_navigation.py`及现workspace回归，`ros2/tests/test_navigation.py`任务取消用例；unit+owner |
| S12 / T5 | 实际global/local Q A→B→撤销，旧lethal与膨胀撤，fixed墙仍lethal；在此序列嵌入多消息/reset-disable/rolling origin而非新矩阵 | 新`N/test/test_current_scan_layer.cpp`＋现`ros2/tests/test_navigation.py`安装态；实际部署Nav2离线 |
| S13 / T5 | 同部署版unknown通道真实Smac生成路径，MPPI实际控制输出并经离线执行fixture跟随；已知障碍重规划，不能称实车 | 现`ros2/tests/test_navigation.py:test_installed_nav2_bringup_with_synthetic_map`扩执行链；部署Nav2离线 |
| S14 / T5 | 同版mixed unknown/lethal footprint，先unknown边后lethal边/同输入边序对照，真实checker/Smac/MPPI不能穿已知障碍；先复现源码风险 | 同上安装态probe及新插件test的同版库调用，记录真实ABI/补丁；部署Nav2离线，失败需负责人收敛修复 |
| S15 / T6 | 实际Q/独立deadline、重复snapshot不续期、全量2048的明确96点预览/计数，固定G不因每scan重拉tiles | 最新`web/src/api/vehicleState.test.ts`、`features/map/map.test.tsx`、`features/devices/devices.test.tsx`，`senior_control/tests/test_agent.py`、`cloud/tests/test_agent_snapshots.py`及process实际消息；UI+owner |
| S16 / T7 | 同输入四核并发重放，记录延迟/CPU/RSS/队列/锁/save-loop峰值及raw动态移走后的定位质量，无编造门槛 | `tools/occupancy_replay_benchmark.py`复用；同一S02/S05/S07输入与已有profile；目标四核实测，实车未验 |

**既有回归复用（默认无需改文件，仍须执行）：** `ros2/tests/test_scan_exact_stamp.py`、`test_scan_epoch_live.py`、`test_navigation_reference.py`、`test_navigation_collision_qualification.py`；`M/test/test_mapping_runtime.py/test_checkpoint_worker.py`；`senior_control/tests/test_direct_navigation.py/test_mapping_production.py/test_mapping_catalog_wire.py`；`tests/unit/test_occupancy.py/test_perception.py`。现snapshot/wire用例已查到`senior_control/tests/test_vehicle_state.py/test_workspace_data.py/test_agent.py`与`tests/integration/test_manual_wire.py/test_settings_logs_wire.py`，先运行涵盖C4字段的已有回归；若需修改其文件，先在下列manifest中报告范围差异，不默删旧失效断言。缺姿态或更新期望的原有效回归必须保留，fixture修复应说明现契约依据，不能弱化保护。

## 依赖顺序、审核与范围

Task 0证据分两条：身份/store/格式可在部署证据未齐时先做；可靠几何profile和Nav2 ABI/安装资格分别等待物理与版本证据。`T1契约 → (T2纯几何 ∥ T5插件/准入 ∥ T6消费者)`；`T1+T2 → T3 → T4`由同一个原mapping owner收口；T5/T6可用冻结contract fixture先做开发，最终只以T4真实输出联验；`T4+T5+T6 → T7`。T3/T4与T1共享ros_runtime/navigation_bridge/native_session路径，owner串行落地，消费者分支不同时改它们。每个Task按失败断言→最小实现→定向通过→本地commit→Orca独立审核→负责人亲阅正常merge，冲突在原owner边界处理，不cherry-pick。不是本轮派工或授权自动启动helper。

本计划比早期“只改14个产品接点”审计范围大：真实跨进程Q、同anchor证据、全部恢复/导出、state wire/manager透传及PR42消费者均须接线，不能漏算。比“净化全部native graph”更窄：不改matcher/solver/graph历史撤销、无新SLAM/永久tracker、不改ReadGraphCommit消息，也不默认改运动算法。精确预测按下列文件manifest去重，不把各任务相加当并集；任务开始后最新main若改路径或证据要求新增补丁，先报告差异给负责人再收口，不能把预测当允许随意扩范围。

### 未来产品实施文件manifest（预测，不是本轮实际改动）

A=新增，M=修改，D=删除；每行一个去重路径，任务列即逐任务精确文件集。XML插件声明计产品，CMake/package.xml计构建；配置只计yaml；工具与操作/协议文档合计工具文档。以下所有D=0，跨任务重叠在并集只计一次。

| 类别 | A/M | 任务 | 完整路径 |
|---|---|---|---|
| 产品 | A | T1 | `ros2/src/senior_interfaces/msg/CurrentObstacles.msg` |
| 产品 | A | T1 | `ros2/src/senior_interfaces/msg/QualifiedMappingScan.msg` |
| 产品 | A | T2 | `ros2/src/senior_mapping/senior_mapping/scan_geometry.py` |
| 产品 | A | T1 | `ros2/src/senior_mapping/senior_mapping/shared_scan.py` |
| 产品 | A | T5 | `ros2/src/senior_navigation/current_scan_layer.xml` |
| 产品 | A | T5 | `ros2/src/senior_navigation/include/senior_navigation/current_scan_layer.hpp` |
| 产品 | A | T5 | `ros2/src/senior_navigation/src/current_scan_layer.cpp` |
| 产品 | A | T6 | `web/src/features/map/useCurrentScan.ts` |
| 产品 | M | T6 | `cloud/src/cloud/snapshots.py` |
| 产品 | M | T6 | `contracts/src/senior_contracts/wire.py` |
| 产品 | M | T1, T6 | `ros2/src/senior_localization/senior_localization/live_mapping_reference.py` |
| 产品 | M | T3 | `ros2/src/senior_mapping/senior_mapping/active.py` |
| 产品 | M | T4 | `ros2/src/senior_mapping/senior_mapping/archive.py` |
| 产品 | M | T4 | `ros2/src/senior_mapping/senior_mapping/checkpoints.py` |
| 产品 | M | T3 | `ros2/src/senior_mapping/senior_mapping/evidence.py` |
| 产品 | M | T3, T4 | `ros2/src/senior_mapping/senior_mapping/native_session.py` |
| 产品 | M | T1, T4 | `ros2/src/senior_mapping/senior_mapping/navigation_bridge.py` |
| 产品 | M | T3 | `ros2/src/senior_mapping/senior_mapping/occupancy.py` |
| 产品 | M | T4 | `ros2/src/senior_mapping/senior_mapping/operations.py` |
| 产品 | M | T3, T4 | `ros2/src/senior_mapping/senior_mapping/references.py` |
| 产品 | M | T1, T4 | `ros2/src/senior_mapping/senior_mapping/ros_runtime.py` |
| 产品 | M | T4 | `ros2/src/senior_mapping/senior_mapping/slam_adapter.py` |
| 产品 | M | T3 | `ros2/src/senior_mapping/senior_mapping/store.py` |
| 产品 | M | T3 | `ros2/src/senior_mapping/senior_mapping/workspace.py` |
| 产品 | M | T4 | `ros2/src/senior_mapping/senior_mapping/workspace_export.py` |
| 产品 | M | T1 | `ros2/src/senior_mapping/src/slam/bounded_scan_ingress.cpp` |
| 产品 | M | T1 | `ros2/src/senior_mapping/src/slam/bounded_scan_ingress.hpp` |
| 产品 | M | T4 | `ros2/src/senior_mapping/src/slam/mapping_navigation.hpp` |
| 产品 | M | T1 | `ros2/src/senior_mapping/src/slam/slam_graph_node.cpp` |
| 产品 | M | T1 | `ros2/src/senior_mapping/src/slam/static_lidar_tf_ingress.cpp` |
| 产品 | M | T1 | `ros2/src/senior_mapping/src/slam/static_lidar_tf_ingress.hpp` |
| 产品 | M | T5 | `ros2/src/senior_sim/launch/simulation_motion.launch.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/ipc_state.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/local_state.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/main.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/snapshot_state.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/snapshots.py` |
| 产品 | M | T6 | `senior_control/src/vehicle_manager/agent/state_ports.py` |
| 产品 | M | T5 | `senior_control/src/vehicle_manager/agent/workspace.py` |
| 产品 | M | T5 | `senior_control/src/vehicle_manager/waypoints/direct_navigation.py` |
| 产品 | M | T5 | `senior_control/src/vehicle_manager/waypoints/ports.py` |
| 产品 | M | T6 | `web/src/api/protocol.ts` |
| 产品 | M | T6 | `web/src/features/devices/ControlSurface.tsx` |
| 产品 | M | T6 | `web/src/features/map/MapCanvas.tsx` |
| 产品 | M | T6 | `web/src/features/map/useWorkspaceMap.ts` |
| 产品 | M | T6 | `web/src/features/tasks/TaskList.tsx` |
| 测试 | A | T2 | `ros2/src/senior_mapping/test/test_scan_geometry.py` |
| 测试 | A | T1 | `ros2/src/senior_mapping/test/test_shared_scan.py` |
| 测试 | A | T5 | `ros2/src/senior_navigation/test/test_current_scan_layer.cpp` |
| 测试 | A | T4, T7 | `ros2/tests/test_shared_layers_process.py` |
| 测试 | M | T6 | `cloud/tests/test_agent_snapshots.py` |
| 测试 | M | T1 | `ros2/src/senior_mapping/test/probes/probe_bounded_scan_ingress.py` |
| 测试 | M | T3 | `ros2/src/senior_mapping/test/test_active_map.py` |
| 测试 | M | T4 | `ros2/src/senior_mapping/test/test_checkpoints.py` |
| 测试 | M | T4 | `ros2/src/senior_mapping/test/test_map_archive.py` |
| 测试 | M | T3, T4 | `ros2/src/senior_mapping/test/test_map_references.py` |
| 测试 | M | T5 | `ros2/tests/test_navigation.py` |
| 测试 | M | T6 | `senior_control/tests/test_agent.py` |
| 测试 | M | T5 | `senior_control/tests/test_direct_navigation.py` |
| 测试 | M | T5 | `senior_control/tests/test_workspace_data.py` |
| 测试 | M | T3 | `tests/unit/test_occupancy.py` |
| 测试 | M | T6 | `web/src/api/vehicleState.test.ts` |
| 测试 | M | T6 | `web/src/features/devices/devices.test.tsx` |
| 测试 | M | T6 | `web/src/features/map/map.test.tsx` |
| 配置 | M | T1, T2, T3 | `ros2/src/senior_mapping/config/slam.yaml` |
| 配置 | M | T5 | `ros2/src/senior_navigation/config/nav2.yaml` |
| 构建 | M | T1 | `ros2/src/senior_interfaces/CMakeLists.txt` |
| 构建 | M | T1 | `ros2/src/senior_mapping/CMakeLists.txt` |
| 构建 | M | T1 | `ros2/src/senior_mapping/package.xml` |
| 构建 | M | T5 | `ros2/src/senior_navigation/CMakeLists.txt` |
| 构建 | M | T5 | `ros2/src/senior_navigation/package.xml` |
| 工具文档 | M | T0, T4, T7 | `docs/operations/slam-backend.md` |
| 工具文档 | M | T4 | `docs/protocol/map-bundle.md` |
| 工具文档 | M | T5, T7 | `ros2/src/senior_navigation/README.md` |
| 工具文档 | M | T7 | `tools/occupancy_replay_benchmark.py` |

### 每任务计数及并集

每格为A/M/D，绝不把重复路径累计为并集。T0只记录证据到已有owner文档；T7复用T4已新增测试而计A，表示相对实施前基线的归类，不表示再次新增。

| 任务/集合 | 产品 | 测试 | 配置 | 构建 | 工具文档 | 总计A/M/D |
|---|---|---|---|---|---|---|
| T0 | 0/0/0 | 0/0/0 | 0/0/0 | 0/0/0 | 0/1/0 | 0/1/0 |
| T1 | 3/8/0 | 1/1/0 | 0/1/0 | 0/3/0 | 0/0/0 | 4/13/0 |
| T2 | 1/0/0 | 1/0/0 | 0/1/0 | 0/0/0 | 0/0/0 | 2/1/0 |
| T3 | 0/7/0 | 0/3/0 | 0/1/0 | 0/0/0 | 0/0/0 | 0/11/0 |
| T4 | 0/10/0 | 1/3/0 | 0/0/0 | 0/0/0 | 0/2/0 | 1/15/0 |
| T5 | 3/4/0 | 1/3/0 | 0/1/0 | 0/2/0 | 0/1/0 | 4/11/0 |
| T6 | 1/14/0 | 0/5/0 | 0/0/0 | 0/0/0 | 0/0/0 | 1/19/0 |
| T7 | 0/0/0 | 1/0/0 | 0/0/0 | 0/0/0 | 0/3/0 | 1/3/0 |
| 并集 | 8/38/0 | 4/14/0 | 0/2/0 | 0/5/0 | 0/4/0 | 12/63/0 |

工具文档拆分：工具A0/M1/D0（原benchmark），文档A0/M3/D0（原操作、格式及导航README）；本轮A2/M2的设计归档不计入未来产品实施并集。

上述预测包含新消息、独立最新槽、三态/多支持、跨进程安装/迁移、Nav2插件及完整state/UI接线；没有默认纳入native graph净化或独立速度改动。Task 0若发现部署版必须修mixed footprint、profile几何/速度装配冲突、static map_server模式必须新增共享写行为，或新main改变接点，单列证据与增量文件并集给负责人，不把这些未收敛改动藏入数字；除已列simulation_motion静态兼容overlay外，现launch/profile仅只读复核，不擅自全面重写。

## 本轮亲自self-review（文档审阅，未执行产品验证）

1. **覆盖：** 获批设计的逐scan资格/不等keyframe→T1/T3/S05；孤立稳定、移动撤销、再稳定→T2/T3/S01–S04；free/unknown、多支持/anchor→T3/S04/S06；所有保存、恢复、导出/legacy→同一owner T4/S07/S08；source/R退休→T1/T4/S09；Nav2 unknown/R/G→T5/S10/S11；真实撤格/膨胀、通道、mixed足迹→T5/S12–S14；最新UI/独立期限→T6/S15；四核/raw定位→T7/S16。未将图外目标或独立速度工作偷加为共享层需求。
2. **接口一致：** 全文只用C1 ScanKey/Q、C2 GeometryEvidence、C3 FixedWatermark/FixedDelta/FixedSnapshot、C4 install_fixed；graph sequence与evidence/fixed水位分离。管理侧读独立槽，不需反向map.backend通知或新RPC。source/reference资格退休不允许旧结果续期；`LiveMappingReference.observe`直接消费逐scan原odom，不能在存盘时仍依赖graph后pose回调。
3. **职责/复杂度：** 几何只产证据，ActiveMap/store唯一写fixed，archive/checkpoint同人收口，桥/插件/cloud/UI无写图权限。只新增两Python职责模块、两消息、三插件文件及一个UI期限hook；无第二SLAM、永久tracker、全graph清洗、全图每scan合成、长期分支群或通用并发框架。独立入口executor是避免native capture阻塞的必要边界，不把to_thread当作无锁证据。
4. **Review Focus：** 五项分别有T1旧exact-stamp/姿态负例、T4真实S05、T5真实S12、T3 S04/S06、T4 S07/S08断言；没有用额外任意场景替代S01–S16。原有效回归保留，缺姿态/更新期望等不靠删断言过关。
5. **资源与真实性：** 亲查发现读图16Mi/导航1Mi、CMake2.6.10/manifest2.6.8、IPC8KiB及snapshot wall/steady差异，均在计划纠正。UI采用明确96点预览避免放宽运动IPC，完整导航Q不截点；cloud缓存只减寿命。物理门限、部署ABI/mixed风险、四核数字没有伪造；最终预测为75个去重文件（A12/M63/D0），以本文件manifest并集为准。所有命令是未来派单，不声称已跑。

## 必要依赖、阻碍与独立事项（交负责人收口）

- **可立即派发的清晰部分：** merge最新main后，C1身份/独立Q、C3同anchor/三态/支持模型、C4格式/原档保护及准入R/G逻辑可按冻结字段推进；mock/fixture仅限开发，不替代生产/安装验收。
- **物理判稳依赖：** 原scan协议与sensor/odom误差证据缺失，可靠GeometryProfile和固定升级激活受阻；最小取证见Task 0，纯算法与其余owner工作可前进。不能再问用户孤立静止是否入图或偷加类别规则。
- **Nav2依赖：** 本机无/opt/ros；部署包/补丁/参数与profile、实际offline构建安装环境尚缺，插件ABI冻结与S12–S14不得声称完成。canonical footprint .674×.540m与旧hardware默认.55×.55m、sim .674×.552m的对应仍须核实，所选footprint/padding及速度不能靠缩小足迹或放宽资格过关；若证据要求装配实改，列真实文件增量由负责人收口。mixed footprint仍为源码风险，复现失败后另审最小补丁/版本选择，未默认计入产品清单。
- **静态fallback依赖：** 已知sim map_yaml有map_server/AMCL，T5用原临时overlay保留其旧导航输入，不宣称已增量共享；旧图可更新路径走同一mapping owner。硬件若也有独立fallback且要求全模式共享，先取得实际使用/交接证据并报告额外行为/文件范围，不能在未知装配下全局启新配置、停旧导航或冒称没有双发布风险。
- **目标范围边界：** 当前没有目标驱动图范围扩展API；T3只按真实观测/当前Q必要覆盖扩unknown边框且≤1Mi。任意图外目标、跨上限规划域若需新增显著行为/owner，单列负责人决定，不阻止有限图内unknown交付。
- **资源/定位依赖：** S16必须同四核profile实测。raw影响尚未证实不可用，失败才按局部证据另评；native graph写入容量亦须测，不把16384读图上限当写入保证。UI远程单程延迟无可信上界，只能如实界定显示TTL误差，Nav2同机截止必须准确。
- **独立运动工作：** 统一速度alpha/k与来源无关反向ramp由原共同运动owner另行实施/审核；本计划不增加来源分支、不顺手修、不作为共享层必需条件。只有其既有失格/安全输出已影响S16实验可执行性时记录该真实依赖，仍不把整项运动需求并入共享层。

交接执行方式已经确定为Orca分阶段开发/审核。本轮到文档本地commit及交付为止；负责人亲阅后另派产品开发，不自动启动实施或helper。
