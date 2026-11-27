# Frontier Navigation Implementation Plan

> **Superseded（2026-11-25）**：本文仅保留历史记录，已由[2026-11-25-shared-obstacle-layers.md](2026-11-25-shared-obstacle-layers.md)取代；旧要求与旧实施授权状态不再作为开发依据。共享障碍层行为已获用户批准，后续按新设计及新计划由负责人亲阅后分阶段派发。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保留原任务目标，只探索仍可能连接当前位置与目标的相关前沿，经同一Nav2和共用运动层执行。

**Architecture:** RouteService仍拥有整条路线，NavigationAdapter拥有真实Nav2 handle；纯候选分析与单原目标编排分开。既有reference提供原子只读切片，同一GridBased查询＋当前可信图整车复核决定可执行性。profile在平台装配处派生同一几何/规划参数；新用户比例限速归共用SpeedCalculation，不增最终输出owner。

**Tech Stack:** Python ≥3.10、asyncio、现有WorkspaceGrid/MapRef/Pose2D、ROS2 Humble、Nav2 SmacPlannerLattice/MPPI、pytest/pytest-asyncio；不新增第三方库、服务、进程或线上DTO。

**Spec:** [2026-11-25-frontier-navigation-design.md](../specs/2026-11-25-frontier-navigation-design.md)，执行者必须同时读设计，包括最后“共用比例过渡与硬限制”覆盖节。

## Global Constraints

- 状态：待负责人亲阅；本轮仅两份文档，不写产品/测试/构建、不装依赖、不运行测试/服务/设备、不推送/PR。下述命令均为**后续执行命令，当前未运行**。
- 基线 `11a26906c9831b000b09e2279ad7ef4ea514d903`；功能主分支 `feat/frontier-navigation`。负责人收敛后另派实施，不自行开始下一阶段。
- 原目标已知但无路也探索相关未知绕路；仅保留可能形成S→T未闭合通路的前沿。图外目标接纳是负责人推荐、待确认；未确认不得实施该分支。
- unknown/临时阻挡仅供乐观拓扑；实际目标及整车路径只用已知free，`allow_unknown=false`，同一GridBased及DedicatedController私有来源。
- 临时阻挡单次最多30.0秒；29.9秒不提前失败，30.0秒找其他路线；其他候选均尝试仍受阻，收口“暂时无法到达”。不得设15秒等图、60秒移动信息超时，也不得将30秒当全运行/新图超时。
- 生产False接纳保持；True兼容语义不改。子goal不完成原action，不更换task epoch，不调用整路线退休；未知取消不发替代。
- 同参考可有raster_sequence≠anchor_cut_sequence；无Nav2 native-cut ACK，不新建ACK服务；当前图整车保守复核＋真实planner不叫同cut原子证明。
- 所选HardwareMotionProfile居中包络是几何唯一输入，候选/launch/DedicatedController用同份派生身份；不凭UI扩几何、不预先扩profile、不重复叠sensor_error。
- 新用户决定及纠正：共用层先 `u_smooth=u_base+alpha*(u_target-u_base)`（普通为既有输出，换向首周期为零、后续为该过渡进度），再 `u_out=k*u_smooth,0<=k<=1`；手动与导航共用，alpha/k必须分清。用户最新批准：方向反转时从零向量按配置accel/yaw_accel共同增长反向(v,w)，首帧即反向，再施共同硬k；不是直接目标或先把正输出减到零。后续职责纠正明确手动/Nav2同标准，速度/碰撞模块不得辨来源或导航阶段。五项比例验证已选，替换原S03/S06/S14/S18/S20，不扩大20项矩阵。
- 不重写碰撞模型，不声称前后净距或比例保持提供完整转弯扫掠安全。已有来源租期、离线、接管、急停、取消、重启与终端释放断言不能削弱。

## Review Focus

以下是20项场景以外最易遗漏的输入类，均复用既有回归或在所属场景内保留断言，不扩一套新矩阵：

1. 非有限目标、过时客户端grid、display-only以及航点写入入口：拒绝非法/过时/不可驾驶数据，航点仍free-only；归Task 1现有workspace/direct_navigation参数用例。
2. planner旧图正路径跨当前occupied或路径段中间墙：同一planner成功仍须当前完整footprint扫掠过滤；归Task 4a查询适配测试（作为场景5的路径验证断言）。
3. publisher/clock/pose source换代与迟到accept：不能借新snapshot重盖旧结果；归Task 1/4场景13及既有navigation_limits来源回归。
4. 百万格更新风暴与取消：分块yield、有界单在途与候选分页，不阻塞shared loop或积压任务；归Task 2场景17的取消/容量断言，资源回放另留证据。
5. 任务离线、手动接管/急停、重启和终端回执丢失：保留原task绑定、原样本截止与独立释放屏障；归Task 4既有ownership/terminal/limits回归。

---

## Nav2剩余事项与解锁顺序

设计末节的七项源码核查是本计划共同边界：来源分支提案已撤回，所有已接纳意图统一规则；剩余是MPPI预测/真实odom与共同ramp的偏差、progress checker和30秒等待的执行冲突、profile派生、非终结暂停、地图代次/static layer异步、观察新证据进展，以及实测运动方向碰撞缺口。前五项按原owner做最小接线，观察驻留/到达scan证据与有效物理制动契约未定，不能通过生成计划视为获批。

执行依赖：负责人亲阅完整plan和最新spec → 收敛observe总驻留/收口及scan关联 → 查实际部署Nav2包版本/模型和odom链路并审阅对齐方案 → 另行派实施；定向复现grid/reference bug先于修复，S20只读推导仍须定向复现。单次≤1秒可取消await仅为响应机制，取消/资格失效要立即唤醒（或现有监视器有界检查），不能等总驻留结束；guard零输出确认不代替硬件制动证据。物理制动新契约未定时如实交接，不擅自改用户反向规则。

## 四部分、文件owner与依赖

这是一个共同交付的导航变更，分四部分审阅，不是四套可独立部署系统。以下owner指未来派单文件责任，本轮没有派helper。

| 部分 | 任务与唯一写入owner | 产物与依赖 | 可并行边界 |
|---|---|---|---|
| A 可信输入/准入 | Task 1，输入owner | 修代次bug、导航专用目标校验、snapshot/结构占据证据 | 接口冻结后与B/C并行；不写agent_adapter |
| B 相关前沿 | Task 2，候选owner | 无业务owner的拓扑/候选/分页；消费A DTO、C几何 | 可用冻结夹具与C并行；只写frontier及其新测试 |
| C 车型与共用运动 | Task 3a，参数owner；Task 3b，运动owner | 同profile派生参数；同alpha过渡＋共同k硬限制 | 3a可与A/B并行；3b按最新反向规则更新已选五项验证，待整体实施授权及模型接线审阅后解锁，不与其共用测试文件并写 |
| D 查询/编排/接线 | Task 4a查询、Task 4b编排，整合owner | A/B/C后接同planner、非终结暂停与原action执行；最后离线/安装回归 | 4a可先用DTO开发；4b独占navigation_adapter、agent_adapter与现有navigation测试 |

建议派单粒度为下面6项（1、2、3a、3b、4a、4b），其中3b单独审阅；产品整合顺序1→3a/2→4a→4b，3b按已选范围与整合合版。若需仅交付前沿部分而不合新比例行为，必须由负责人明确调整交付范围，实施者不能自行遗漏用户新决定。

### 文件地图（路径为仓库相对路径）

为避免重复长前缀，表内定义的缩写只用于本文：`NAV=ros2/src/senior_navigation/senior_navigation`；`MAP=ros2/src/senior_mapping/senior_mapping`；`LOC=ros2/src/senior_localization/senior_localization`；`MOTION=ros2/src/senior_motion_control/senior_motion_control`；`ROBOT=ros2/src/senior_robot/senior_robot`；`MANAGER=senior_control/src/vehicle_manager`。执行git命令必须用实际路径。

| 路径 | 增改 | 责任 |
|---|---|---|
| `MAP/navigation_snapshot.py` | 新增 | frozen只读切片值；不持有可写状态 |
| `LOC/live_mapping_reference.py` | 修改 | 原子snapshot读取与既有资格复核 |
| `MAP/navigation_bridge.py` | 修改 | install水位、结构占据传递，保持唯一可信发布owner |
| `MAP/native_session.py` | 修改 | 从现有稳定证据派生只读占据，随已有reference请求交付 |
| `MANAGER/waypoints/direct_navigation.py` | 修改 | grid/reference分离，导航unknown准入 |
| `MANAGER/waypoints/ports.py` | 修改 | map_grid_id与导航专用校验端口 |
| `MANAGER/agent/workspace.py` | 修改 | 导航目标分类；原verify_free_target仍free-only |
| `NAV/frontier.py` | 新增 | SE2、乐观相关性、整车候选、分页与取消分块 |
| `NAV/planner.py` | 新增 | ComputePathToPose与路径保守复核，单查询在途 |
| `NAV/goal_navigation.py` | 新增 | 一个原目标的阶段、30秒、候选账本 |
| `NAV/navigation_adapter.py` | 修改 | handle owner与原action边界，RouteService暂停回调 |
| `NAV/dedicated_controller.py` | 修改 | 实际读取同份派生参数，保持私有run生命周期 |
| `ROBOT/navigation_parameters.py` | 新增 | profile→只读几何/参数产物；无可编辑配置owner |
| `ROBOT/platform_runtime.py` | 修改 | 选定profile派生一次、launch与adapter同一路径 |
| `ROBOT/agent_adapter.py` | 修改 | 注入snapshot/profile/planner及pause/resume回调 |
| `MOTION/command_node.py` | 修改 | 非终结停转发、清held target、保留原绑定 |
| `MOTION/arbiter.py` | 修改 | 暂停/恢复样本接纳但不封epoch |
| `MOTION/vehicle_speed.py` | 修改 | 唯一k、限制原因和兼容Velocity入口 |
| `MOTION/motion_core_output.py` | 修改 | 消费同一决策与现有真实速度，诊断原因 |
| `ros2/src/senior_navigation/launch/navigation.launch.py` | 修改 | planner启动用已派生参数，不独立推车型 |
| `ros2/src/senior_navigation/config/nav2.yaml` | 修改 | 移除生产几何/速度独立硬编码来源，保留非车型规划参数 |

`route.py` 的QualifiedRoute仍按原poses顺序执行；让它调用“一个原目标完整运行”而非一个观察goal即可，预期不改。现有package.xml已有nav2_msgs，CMake用ament_python_install_package，不因新增Python文件机械改构建。`ROBOT/real_runtime.py`、`ros2/src/senior_sim/launch/simulation.launch.py`仅在实际参数装配核对发现遗漏时条件修改（真实入口已核对存在，见3a）；不预先扩hardware_profile/schema。

## 车内接口（已展开部分与待收敛边界）

下列为计划新增接口，不冒称源码已经存在或全部获批；观察进展/驻留与MPPI模型对齐按设计末节先收敛。避免复制现有MapRef/Pose2D/WorkspaceGrid/RouteProgress。

`MAP/navigation_snapshot.py` frozen dataclasses：

```python
@dataclass(frozen=True)
class CutIdentity:
    map_ref: MapRef
    generation: str
    boot: str
    grid_id: str
    raster_sequence: int
    anchor_cut_sequence: int
    anchor_id: str
    map_basis: Pose2D
    odom_basis: Pose2D


@dataclass(frozen=True)
class NavigationSnapshot:
    identity: CutIdentity
    grid: WorkspaceGrid
    pose: Pose2D
    sample: SampleTime
    pose_source: tuple[object, ...]
    pose_sequence: int
    structural_grid: WorkspaceGrid | None
    structural_sequence: int | None
```

pose_source沿reference已有六元source tuple（bridge_boot/source_epoch/capture_generation/clock_generation/raw_gid/bridge_gid），不重新编码线上契约。结构图只读display-only，只有100参与结构阻隔；其他值不给free。安装请求增加可选`structural`字典：`sequence:int, grid_id:str,width:int,height:int,resolution:float,origin:[float,float,float],cells_b64:str`；含义为本次稳定证据cut，不强求等于raster_sequence，需同参考/boot和不晚于anchor cut。使用既有有界IPC传输机制，不增加ROS服务；超出传输预算则structural=None，禁止闭合证明。

- `LiveMappingReference.install(...现有位置参数..., *, raster_sequence: int = 0, anchor_cut_sequence: int = 0, structural_grid: WorkspaceGrid | None = None, structural_sequence: int | None = None) -> None`；默认只兼容旧固定图/夹具，水位未知不给闭合证明。
- `LiveMappingReference.snapshot() -> NavigationSnapshot | None`：不await、不拷贝cells，沿pose()相同年龄/唯一owner检查。fallback无native来源时可返回带显式boot="fallback"、零水位的保守snapshot，仍需原qualified；不得编造native身份/稳定证据。
- `WorkspaceTargetPort.verify_navigation_target(ref: MapT, grid_id: str, pose: PoseT) -> Awaitable[None]`：仅导航，inside free或unknown通过；occupied/1..99/display-only/非有限拒绝；outside默认拒绝。图外政策批准后同一方法放行合法有限outside，无新增配置开关。原verify_free_target不变。

`NAV/frontier.py`：`Footprint = tuple[tuple[float,float], ...]`；`RegionKey = tuple[int,int,int,int]`为map坐标量化边界（量化分辨率随账本保存，不按数组行号）。

```python
@dataclass(frozen=True)
class FrontierCandidate:
    key: RegionKey
    pose: Pose2D
    observation_cells: tuple[int, ...]  # 本批有界，使用输入grid索引
    corridor_key: RegionKey


@dataclass(frozen=True)
class FrontierPage:
    identity: CutIdentity
    candidates: tuple[FrontierCandidate, ...]
    next_cursor: int | None
    potential_connection: bool
    closure_proven: bool
    diagnostic: str


async def frontier_page(
    snapshot: NavigationSnapshot,
    target: Pose2D,
    footprint: Footprint,
    *,
    cursor: int,
    checkpoint: Callable[[], Awaitable[None]],
) -> FrontierPage: ...
def world_to_cell(grid: WorkspaceGrid, pose: Pose2D) -> tuple[int, int]: ...
def cell_center(grid: WorkspaceGrid, column: int, row: int) -> Pose2D: ...
async def path_is_known_free(
    grid: WorkspaceGrid,
    path: tuple[Pose2D, ...],
    footprint: Footprint,
    *,
    checkpoint: Callable[[], Awaitable[None]],
) -> bool: ...
```

拓扑、block-cut分析均按设计分块；禁止在checkpoint之间做完整洪泛。cursor是确定性全候选域次序，16个一页；账本由GoalNavigator拥有，frontier模块不留跨任务全局缓存。目标outside仅用于分析外域，不意味着管理入口已批准接纳。

`ROBOT/navigation_parameters.py`：

```python
@dataclass(frozen=True)
class NavigationParameters:
    path: Path
    geometry_id: str
    footprint: tuple[tuple[float, float], ...]
    xy_goal_tolerance: float
    yaw_goal_tolerance: float


def derive_navigation_parameters(
    profile: HardwareMotionProfile, base_params: Path, destination: Path
) -> NavigationParameters: ...
```

content hash含所选profile内容及相关派生参数，不仅profile_id。矩形顶点±L/2、±W/2；global/local footprint一致；MPPI vx_max=cap、vx_min=-cap、wz_max=yaw_cap。端点容差读实际参数（当前均0.25），不另硬编码。产物在既有runtime资源目录写一次，进程退出按既有资源清理；对同产物启动两端，不能只launch RewrittenYaml而controller读原yaml。BT/lattice路径解析仍走既有装配。

`NAV/planner.py`：

```python
@dataclass(frozen=True)
class QueryContext:
    execution_id: str
    task_epoch: int
    effect_id: str
    action_id: str
    snapshot: NavigationSnapshot
    geometry_id: str


@dataclass(frozen=True)
class PlanCheck:
    context: QueryContext
    status: Literal["reachable", "no_path", "unavailable", "timeout", "stale", "canceled"]
    path: tuple[Pose2D, ...]
    diagnostic: str


class PlannerQuery:
    async def check(self, context: QueryContext, target: Pose2D) -> PlanCheck: ...
    async def cancel(self) -> None: ...
```

构造函数 `PlannerQuery(node: Any, current: Callable[[], NavigationSnapshot | None], footprint: Footprint, geometry_id: str, xy_tolerance: float, yaw_tolerance: float)`；ROS action与便携fake transport隔离在该小模块内。`use_start=true`、start=context.snapshot.pose、goal=target、frame=map、GridBased。每次请求2秒本地查询截止、一次在途；取消查询本身不能取消实际导航goal；晚回包丢弃。Humble结果按实际status/可用字段分类，未知失败诊断temporary，不杜撰error_code字段或永久墙。

`NAV/goal_navigation.py`：

```python
class GoalNavigator:
    def __init__(
        self,
        snapshot: Callable[[], NavigationSnapshot | None],
        planner: PlannerQuery,
        footprint: Footprint,
        geometry_id: str,
        run_child: Callable[[Pose2D], Awaitable[bool]],
        cancel_child: Callable[[], Awaitable[OperationResult]],
        pause_source: Callable[[], bool],
        resume_source: Callable[[], bool],
        qualified: Callable[[], bool],
        monotonic: Callable[[], float],
        sleep: Callable[[float], Awaitable[None]],
    ) -> None: ...
    async def run(self, context: QueryContext, target: Pose2D) -> bool: ...
```

run_child只在真实accept后恢复转发/通知RouteService真实goal_changed；bool仅该子goal终态成功，不直接action_completed。内部任务当前性由RouteService为该闭包捕获route/action identity，snapshot再检查参考与来源；只有原目标成功才让run返回True。内部终结诊断保存 `diagnostic: str | None` 属性并沿既有日志输出，不新增线上状态契约。每个原action一实例，失败不越过该action。

`GoalNavigatorFactory`在goal_navigation.py定义为Protocol，精确签名：

```python
class GoalNavigatorFactory(Protocol):
    def __call__(
        self,
        *,
        run_child: Callable[[Pose2D], Awaitable[bool]],
        cancel_child: Callable[[], Awaitable[OperationResult]],
        pause_source: Callable[[], bool],
        resume_source: Callable[[], bool],
        qualified: Callable[[], bool],
    ) -> GoalNavigator: ...
```

工厂闭包仅注入已冻结的snapshot/planner/footprint/clock等既有装配值，不建注册表。RouteService的route/action身份由上述闭包固定；每个原action创建一实例，`run(context,target)`输入明确。

非终结控制接口（新增于现有owner）：

- `Arbiter.pause_navigation(execution_id: str) -> bool`、`resume_navigation(execution_id: str) -> bool`：保留run/epoch/sequence，清velocity/deadline；paused时拒绝样本，resume需新鲜新sequence，不调用shutdown/end_navigation。
- `CommandNode.pause_navigation_source(execution_id: str) -> bool`、`resume_navigation_source(execution_id: str) -> bool`：校验绑定/GID/资格；暂停置forwarding=false并清旧target，以现有`prepared`新generation零barrier送下游；它在VehicleArbitration.revoke清held target但不做TASK_RETIRED，不新增close_reason。resume只在真实新goal接纳后调用，不能因同topic迟到样本提前恢复。
- `RouteService.__init__`新增可选 `pause_source: Callable[[str],bool] | None`、`resume_source: Callable[[str],bool] | None`、`goal_navigator: GoalNavigatorFactory | None`、`goal_context: Callable[[RoutePlan,int],QueryContext|None] | None`（生产全部提供；旧兼容直达测试可省略）。agent_adapter的goal_context闭包从当前snapshot和geometry_id构造含plan/action_ids[index]的QueryContext；缺snapshot不发goal。
- `NavigationAdapter.run_route`保留原参数并新增keyword-only `run_original: Callable[[int,Pose2D,Callable[[Pose2D],Awaitable[bool]],Callable[[],Awaitable[OperationResult]]],Awaitable[bool]] | None = None`。RouteService传入的该回调取得goal_context，调用factory并await GoalNavigator.run；两个末参数是adapter当前唯一child runner/canceler。adapter私有 `async def _run_child(self, goal: MapGoal, *, accepted: Callable[[],None], qualified: Callable[[],bool]) -> bool` 持有原_active/_goal_lock；accepted闭包关联原index，不能借child次数递增原index。run_original返回True后才提交该原action；省略回调时兼容直达执行。

## Part A / Task 1：先复现代次bug，再接可信输入

**Files:** 修改上述A的6个现有文件，新增`ros2/src/senior_mapping/senior_mapping/navigation_snapshot.py`；测试修改 `senior_control/tests/test_direct_navigation.py`、`senior_control/tests/test_workspace_data.py`、`senior_control/tests/test_mapping_production.py`、`ros2/tests/test_navigation_reference.py`。

**Interfaces:** 产出冻结snapshot/verify_navigation_target，消费既有state.map_grid_id、SampleTime、ActiveMap稳定证据和既有IPC；不改RoutePlan/线上契约。

- [ ] 写真实bug定向回归 `test_direct_goal_separates_grid_id_from_reference_generation`：请求grid="grid-A"、live.map_grid_id="grid-A"、map_generation=localization_generation="ref-7"；应接纳且start_poses generation="ref-7"，不是grid-A。另保持过时grid拒绝的原断言。该项是**已发现bug回归，单列于20场景之外，尚未运行**。
- [ ] 运行 `.venv/bin/pytest senior_control/tests/test_direct_navigation.py -q`；预期上述用例在旧实现因CONFLICT失败，记录红灯原因，不把环境失败当行为复现。
- [ ] 修DirectNavigation两种代次比较/下发，补NavigationStateReading.map_grid_id；不重新合并两者。
- [ ] 按冻结端口写导航unknown准入用例，并复用workspace非法值/过时/航点free-only断言；图外接纳仅保留独立冻结推荐，不占S03、不先写放行实现。
- [ ] 按场景13及既有snapshot回归保护写身份断言：同参考raster=4、anchor=7合法；旧generation/boot/source结果无权推进，同参考grid增长不改epoch；复用mapping_production真实reference刷新夹具。
- [ ] 实现reference.snapshot与bridge/install水位；native_session用已有稳定快照生成有界structural字段，未知/缺证据时保持None。读取无await，未复制百万格cells。
- [ ] 运行 `.venv/bin/pytest senior_control/tests/test_direct_navigation.py senior_control/tests/test_workspace_data.py senior_control/tests/test_mapping_production.py ros2/tests/test_navigation_reference.py -q`；预期全部通过，原非法输入与参考资格断言不削弱。
- [ ] 仅提交本Task明确的7个产品文件及4个测试文件，建议提交信息 `fix: separate navigation grid and reference identities`；不要 `git add .`。

## Part B / Task 2：相关拓扑、整车候选、分页

**Files:** 新增 `ros2/src/senior_navigation/senior_navigation/frontier.py`、`ros2/tests/test_frontier.py`。复用现有`senior_control/tests/test_workspace_data.py`和`ros2/src/senior_mapping/test/test_active_map.py`的网格构造习惯；不把mapping夹具改成导航owner。

**Interfaces:** 消费A NavigationSnapshot、C footprint；产出frontier_page/坐标转换/path_is_known_free，供4a/4b调用。仅不可变输入与显式checkpoint，不依赖ROS/任务store。

- [ ] 在`ros2/tests/test_frontier.py`按下表S01/S02/S05/S17各写一个独立输入测试，命名固定为`test_s01_relevant_frontiers`等；断言候选集合/closure/整车free/后页游标，不用实现函数回算预期。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_frontier.py -q`，预期缺接口失败。
- [ ] 实现坐标函数、四邻free候选、八邻聚类和符号外域的乐观连接/block-cut相关性；闭合只有稳定证据与必要几何的乐观上界不通才true，缺证据输出temporary诊断。S05只有点能过而整车不能过时无可执行pose，不能用点连通发goal。
- [ ] 实现整路径矩形扫掠覆盖与2048操作/2ms分块checkpoint；候选每批16，按map区域确定性排序，不硬裁背离目标方向。
- [ ] 在S17同输入的分页断言中加入取消checkpoint和单在途调度检查：取消旧分析后退出、再接最新输入，无队列堆积；不新增另一矩阵。前16个被尝试，后页第17个有效必须被找到；覆盖摘要不因LRU遗忘无限重游。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_frontier.py -q`，预期通过；静态复核没有同步全图工作藏在checkpoint之间。CPU/延迟尚需授权回放量测，不凭单测宣称四核通过。
- [ ] 提交上述2文件，建议 `feat: select goal-relevant reachable frontiers`。

## Part C / Task 3a：同一profile派生真实Nav2参数

**Files:** 新增 `ros2/src/senior_robot/senior_robot/navigation_parameters.py`；修改platform_runtime.py、NAV/dedicated_controller.py、navigation.launch.py、nav2.yaml；整合owner在Task4b修改agent_adapter.py。测试修改 `tests/unit/test_motion_launch_profile.py` 与 `ros2/tests/test_navigation.py` 的现有参数测试，模拟平台必要时核对 `ros2/src/senior_sim/launch` 下真实入口再报告条件文件。

**Interfaces:** 产出NavigationParameters/derive_navigation_parameters；平台把既有RuntimeProfile.navigation_parameters派生后的路径传给launch的parameters与adapter的navigation_controller_params，几何只读值接向相同产物，不增加第二可写profile。使用config.json所选原motion_profile，不固定仓库默认文件。

- [ ] 保留并复用现有参数/安装回归保护（不占选定S20、不删除原断言）：选非默认L/W与cap/yaw_cap的同一个profile；候选footprint、global/local costmap footprint完全相同，planner launch与DedicatedController参数路径/内容identity相同；MPPI三上限分别为cap/-cap/yaw_cap，保持consider_footprint=true与allow_unknown=false。
- [ ] 运行 `.venv/bin/pytest tests/unit/test_motion_launch_profile.py ros2/tests/test_navigation.py -k 'profile or footprint or costmap' -q`，预期旧固定参数与所选profile不一致而失败。
- [ ] 实现派生函数/平台产物生命周期；launch仅解析并使用产物、保留BT/lattice路径；DedicatedController也用此产物，禁止只改launch。标准仿真按其现选profile同样派生；若现入口不经platform_runtime才条件改真实入口并更新清单。
- [ ] 先记录部署nav2_mppi_controller实际版本及对应optimizer/motion_models源码，核对是否支持加速度字段与真实odom接线；Humble上游getParams无ax_max/ax_min/az_max，而main有，不能照最新文档填yaml或假称已装配。若部署不支持，保留统一末级规则，模型偏差列待审方案，不静默升级/加来源例外。复核MPPI上限是物理规划上限，保留lattice minimum_turning_radius等规划运动学；不将传感误差重复叠到footprint，不新增profile字段。
- [ ] 运行上项命令及 `.venv/bin/pytest tests/unit/test_ros_package_ownership.py -q`，预期通过。Nav2参数真正生效仍属后续真实服务验证，不能用文本断言冒充运行值。
- [ ] 按上述文件逐个git add并提交 `fix: derive navigation geometry and limits from selected profile`。

## Part C / Task 3b：共用比例过渡与硬限制

用户已明确共同alpha/k，并最新批准反向从零按配置accel增长，首帧负、后续继续增长；覆盖d5e5050直接目标alpha=1及旧跨零规则。手动与Nav2同标准，节点不跨职责辨来源。负责人已选S03/S06/S14/S18/S20五项比例验证，合计为保留原15＋此5，不另建R候选或参数矩阵；当前仍未实施授权。

**Files:** 修改 `ros2/src/senior_motion_control/senior_motion_control/vehicle_speed.py`、`ros2/src/senior_motion_control/senior_motion_control/motion_core_output.py`；复用修改 `ros2/tests/test_motion_core_perception.py`，必要时修改 `tests/unit/test_navigation_core_output.py` 与`tests/unit/test_navigation_limits.py`受共同角过渡影响的期待。不改碰撞算法或CommandNode速度职责。

**Interfaces:** 在vehicle_speed.py定义frozen `SpeedLimitDecision(velocity:Velocity, smoothed:Velocity, alpha:float, ratio:float, reasons:tuple[str,...])`；`limit_velocity_decision(requested:Velocity, previous:Velocity, measured_linear:float|None, clearance:DirectionalClearance, profile:HardwareMotionProfile, dt:float)->SpeedLimitDecision`，DirectionalClearance复用 `senior_perception.collision.DirectionalClearance`。原limit_velocity六位置参数/Velocity返回值保持；增加可选keyword-only `record:Callable[[SpeedLimitDecision],None]|None=None`（兼容旧六参数调用），calculate以实例回调记录`last_decision:SpeedLimitDecision|None`，只算一次。不用模块全局缓存；原monkeypatch包装同步接收可选record，断言仍验证原调用链，没有record的旧替身则诊断unknown。

**过渡接口补充（推荐）:** SpeedCalculation.calculate保留原五参数（requested/previous/measured/clearance/dt），方向/ramp历史只在该实例；纯limit_velocity_decision增加可选keyword-only `transition_origin: Velocity | None = None`供实例在换向首帧传零、随后传保存过渡值，None保持普通插值。原limit_velocity兼容转交，不新增source参数；reset()清方向/进度/last_decision。该值只是运动计算起点，不创建新仲裁状态，硬限制仍只算一次。

**算法决定:** 普通非换向alpha按线性accel/decel与现profile.yaw_accel/yaw_decel的可行约束求共同值，零差不除零，不独立裁剪w；**换向首周期过渡起点为(0,0)，后续沿保存进度向目标增长，线/角用accel/yaw_accel共同alpha**，再按平滑向量的方向净距、profile.cap/yaw_cap求共同硬k。双零/撤销/急停立即停止；纯旋转只用角约束。手动与导航复用现SpeedCalculation，接口不传source/任务类型/探索状态。真实轮速不替换为命令值；负向从零起算不证明实车即时反转或更短刹停，STM32制动效果尚未核实。

五项已选输入及精确预期在下表S03/S06/S14/S18/S20定义，不再保留R1–R6候选。S14按用户最新批准改为从零反向accel序列，包含下一周期不重置的断言，不新增独立输入。另将设计中的实测方向净距缺口按源码定向复现纳入S20同一输入/trace检查：不得将实测正速度伪改成负命令，也不能借前方净距强行把反向请求压零或要求静止后才反向；物理制动契约仍待证据收敛。

- [ ] 为已选输入写失败断言；只替换新决定覆盖的角独立期待，保留来源截止、真实core输出、零请求/实际制动事实；旧reversing时先减正速到0、直接目标alpha=1及-.015/-.01875期待均按最新从零反向accel序列迁移；松杆/失联/撤销/急停仍立即停止并清过渡。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_motion_core_perception.py tests/unit/test_navigation_core_output.py tests/unit/test_navigation_limits.py -q`，预期旧角独立行为在所选输入失败并记录红灯。
- [ ] 实现上述同alpha/同k计算与兼容入口；calculate开始清last_decision，缺record诊断unknown而不重算；无二次各轴裁剪；SpeedCalculation仅保存本实例意图方向/反向未硬缩放过渡值，以意图方向边沿初始化一次，不按实测正速度或每个样本重复清零；新增reset()并由原MotionCoreOutput._stop_output同步调用，双零请求也清空。来源更换/暂停等停止沿原回调清状态，速度算法不看来源。
- [ ] MotionCoreOutput在现trace记录同一alpha/k/reasons，继续真实odom反馈，不新增speed_limit前馈/velocity_smoother，不重写碰撞事实。
- [ ] 执行本Task命令与既有共用core回归，预期通过；插值/比例单测不等于真实轨迹或完整扫掠验收。
- [ ] 仅提交已选产品/测试文件，建议 `feat: interpolate and limit shared motion intent proportionally`；保持20项选定范围，不自行扩参数矩阵。

## Part D / Task 4a：同planner查询与当前图复核

**Files:** 新增 `ros2/src/senior_navigation/senior_navigation/planner.py`、`ros2/tests/test_frontier_planner.py`。不新增Nav2服务、costmap插件或ACK接口。

**Interfaces:** 消费A snapshot、B path_is_known_free和C容差/footprint；产出PlannerQuery/PlanCheck。

- [ ] 按S05在新测试中固定S05同一窄口输入的planner成功稀疏点路径，段中间整车扫掠碰窄口侧壁；必须拒绝（不是只查顶点）。按S13把旧generation查询回包留到当前参考后才返回，必须stale。按S15保持native grid_id不变，fake实时planner先no_path后reachable，应允许再次查询。复用既有navigation测试fake executor样式，状态字段与Humble实际接口一致。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_frontier_planner.py -q`，预期缺接口/旧路径未过滤失败。
- [ ] 接 `/compute_path_to_pose`同一GridBased，2秒查询截止、单在途；返回后重取当前snapshot检查身份、起点/终点/frame/有限值及完整footprint扫掠。positive只能当前保守复核后采用；no_path保守temporary，不声称cut ack或墙分类。
- [ ] 实现查询取消与晚回包丢弃；本模块无实际NavigateToPose handle，不能误取消正在执行goal；查询失败重试由4b预算owner控制。
- [ ] 运行本Task命令，预期通过；用便携fake只能证明本地适配契约，真实action字段与costmap滞后另列ROS层验证。
- [ ] 提交上述2文件，建议 `feat: check frontier paths with the existing Nav2 planner`。

## Part D / Task 4b：非终结停转发、单原目标编排和整合

**Files:** 新增 `ros2/src/senior_navigation/senior_navigation/goal_navigation.py`；修改NAV/navigation_adapter.py、MOTION/command_node.py、MOTION/arbiter.py、ROBOT/agent_adapter.py。测试修改 `ros2/tests/test_navigation.py`、`tests/unit/test_navigation_limits.py`；复用`senior_control/tests/test_home.py`旧取消断言（需要接新编排时才修改其夹具，不削弱原断言）。

**Interfaces:** 消费A/B/C/4a全部冻结接口；产出既有RouteProgress原action进度与既有终端结果，不新增线上“探索成功”。C3b按已选五项验证；observe水位等待须按设计剩余决定收敛，不能用旧snapshot立即判无收益。完整交付须如实列未验状态。

- [ ] 先在现有真实CommandNode→MotionCoreOutput测试rig中写S19：已有非零held target→pause→prepared零barrier实际消费后target为idle/零、task_id/epoch仍在，旧样本不再前进；新goal真实accept前不恢复，新鲜新sequence后恢复，最终cancel仍需原退休/释放证明。
- [ ] 执行 `.venv/bin/pytest tests/unit/test_navigation_limits.py -q`，预期缺非终结接口失败。
- [ ] 在Arbiter加paused布尔与pause/resume，在CommandNode保留subscription/GID和task绑定，暂停清velocity/deadline并用现有prepared零barrier真实清下游target；publish_command暂停分支不调用clear/end_navigation。失资格/源换代仍走原终结撤销，禁止把资格丢失当普通等待。
- [ ] 在`ros2/tests/test_navigation.py`按S04/S07–S12/S15–S17逐项写独立输入测试（下表精确定义），使用fake monotonic推进，禁止实际sleep30/60秒。保留原True/False接纳断言，并增加False计算前预留、True真实首goal后完成的检查（归S10生命周期输入）。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_navigation.py tests/unit/test_navigation_limits.py -k 'not installed_nav2_bringup' -q`，预期新行为失败而既有契约仍可定位；记录红灯。
- [ ] 实现GoalNavigator.run单原目标循环、有关走廊账本、30秒等待/1秒重查、16项分页和观察后水位进展等待/真实收益复核（先按设计收敛后冻结）；没有15/60秒窗口。行驶中仅一个child结果task与一个0.25秒身份观察等待并存，owner循环await二者；发现T可能可达时只发一次有节流查询，确认后暂停/取消h1并等待同一child task真实终态，再发原T。普通grid增长不取消child；所有监视等待在该child退出时join/清理，不能每tick create_task堆积，也不能直接await run_child而永远不检查地图。查询故障3次间隔1秒仅限有资格query失败，实际no_path按30秒候选规则。进入blocked_wait前pause→cancel当前child→等真实terminal，原action和原30秒deadline保留；取消未知不得发替代，不能保持活动FollowPath停车等待30秒撞上现.2m/10s progress checker，更不能放宽全局超时。executor bool失败不直接失败原action，外层重评估/同planner查询按可信结果决定等待；无提前诊断不宣称10s前即可识别阻挡，进展失败也不是永久墙。
- [ ] NavigationAdapter提取child接受/终态逻辑，保持唯一_active/cancel锁；原action_completed只在GoalNavigator.run原目标True时调用一次。子goal切换pause→cancel→真实terminal→当前复核→新accept→resume，不调用整路线_revoke/clear；True/False公开接纳语义不变。
- [ ] agent_adapter在同一RouteService实例注入reference.snapshot、同参数geometry、PlannerQuery与node pause/resume；prepare=controller.open覆盖整路线，整路线finally才退出并按原输出退休/terminal_release收口。
- [ ] 运行 `.venv/bin/pytest ros2/tests/test_frontier.py ros2/tests/test_frontier_planner.py ros2/tests/test_navigation.py tests/unit/test_navigation_limits.py senior_control/tests/test_home.py -k 'not installed_nav2_bringup' -q`；预期所选可实施场景与旧取消通过，图外接纳推荐仍文档冻结、不在选定S03内，不用skip伪装完成。
- [ ] 运行下面“影响回归”命令；全部通过后逐文件提交 `feat: preserve original actions through frontier navigation`，记录已运行证据/未验层级。不推送、不PR，留负责人审阅。

## 已选20个独立输入场景：精确断言、归属与复用

以下正好是负责人选择的20项：保留原15项，原S03/S06/S14/S18/S20替换为已选5项比例验证；不另造矩阵、不把参数组合计作多个场景。图外接纳推荐未获批且不在选定实现内；真实代次bug回归单列，仍有效原有回归/安装检查继续复用。新测试复用现有真实owner/fake执行器，不另写一套模拟任务生命周期。

S01共用基础切片定义：12×9，resolution=1m，origin=(0,0,0)，cell中心记(c+0.5,r+0.5)；未列格均为native occupied且同cut稳定占据，封闭外边界。free为c=1..3,r=3..5；unknown为c=4..10,r=4，以及c=2,r=6..7。S=(2.5,4.5,0)，T=(10.5,4.5,0)，profile居中0.55×0.55。横向unknown桥通向T，向上unknown支路仅经S可达free域返回，不能连接T；完整free域是S端点，不能借在S内绕一圈伪造相关性。其他场景需要不同输入时在各行明确写出，不偷偷复用变化后的状态。

| # / 固定测试名后缀 | 独立输入 | 精确预期 | 所属测试 / 复用真实位置 |
|---|---|---|---|
| S01 `relevant_frontiers` | 上述S01切片与unknown T | 仅横桥入口观察候选（邻c=4,r=4）可选；c=2,r=5朝上死支路排除；potential_connection=true，无实际unknown goal | Task2 `ros2/tests/test_frontier.py`；复用`senior_control/tests/test_workspace_data.py`、`ros2/src/senior_mapping/test/test_active_map.py`网格构造 |
| S02 `closed_target` | S01输入，将c=6全列置同cut稳定occupied，T格改free，外边界仍封闭 | 乐观上界S/T断开，有separator依据；closure_proven=true，结果“无法到达”，探索goal计数0 | Task2同新文件；复用`ros2/tests/test_navigation.py` fake executor以断言无start |
| S03 `curve_common_k` | previous=target=(0.4,0.6)，cap=0.16、yaw_cap=1.0、净距无限（其余字段复用合法profile）使共同硬k=0.4 | alpha=1，smoothed=(0.4,0.6)，out=(0.16,0.24)，两分量同比，不独立裁剪w | Task3b `ros2/tests/test_motion_core_perception.py`；替换旧角独立断言，保留硬上限保护 |
| S04 `known_target_unknown_detour` | S01切片T改free，直接已知路径planner no_path，相关unknown横桥仍连S/T，观察点planner reachable | 进入观察子goal，保留known-free T与原action；不能因T已知而直接失败 | Task4b `ros2/tests/test_navigation.py`；复用同文件QualifiedRoute/RouteService fake |
| S05 `narrow_gap` | 0.1m栅格，0.4m净宽free开口后唯一unknown前沿，车体0.55×0.55，点路径存在；同一开口的query fake返回稀疏点路径，顶点在宽区域free、段经过窄口 | 不生成穿口可执行观察goal；仅点/路径顶点free不足，整段包络复核拒绝；不以保守筛选自动宣称永久闭合 | Task2 `test_frontier.py`与4a `test_frontier_planner.py`分别承担候选/路径断言；复用`ros2/tests/test_navigation_collision_qualification.py`几何及现Nav fake |
| S06 `pure_rotation` | previous=(0,0)，target=(0,0.6)，dt=0.05，已有yaw_accel=1.0、yaw_cap=0.04，无其他更紧约束 | alpha=1/12，smoothed=(0,0.05)，k=0.8，out=(0,0.04)；纯角约束、无线速除零；配置仅为fixture非实测 | Task3b同文件；复用纯旋转旧用例并按共同过渡更新期待 |
| S07 `wait_29_9` | 当前通路A于monotonic=100收到可信临时no_path，仍qualified，clock推进129.9，无相关通路恢复 | 保留A的等待，deadline=130，原action不终结、不提前换B；等待前当前child已真实terminal，无活动FollowPath；允许受节流重查，不发无验证goal | Task4b `ros2/tests/test_navigation.py`注入clock；复用同文件route状态夹具 |
| S08 `wait_30_select_other` | 独立初始化A于100受阻，B尚未尝试且planner reachable；clock到130 | 结束A本次等待并查询/执行B，原任务不failed，A不因重算重新得到30秒 | Task4b同文件；复用`senior_control/tests/test_home.py::test_rejected_cancel_does_not_start_replacement`的替代goal计数方式 |
| S09 `all_temporarily_blocked` | 独立A/B两候选均可信临时no_path，各自一次30秒已满，分页完成，无新相关证据 | 原action未completed，路线failed诊断“暂时无法到达”，正常退休/释放后收口；不轮回A继续等，不报永久墙 | Task4b同文件；复用`tests/unit/test_task_terminal_release.py`真实收口屏障 |
| S10 `observation_not_action` | 原action a1/T冻结，一个观察goal真实接受并成功，重复status查询 | completed_action_ids仍空、原T/task_id/execution/epoch/effect/a1不变；False预留不等计算，True只在真实首goal绑定后完成 | Task4b同文件；复用`tests/integration/test_task_ownership.py::test_accepted_targets_are_immutable_when_future_schedule_is_edited` |
| S11 `growth_returns_original` | 观察handle h1正在行驶；同参考grid A→B使T free且planner reachable；h1 cancel先接受后terminal，最终原goal成功 | terminal前不发T，之后发原T一次；仅原goal成功完成a1一次；中间原任务绑定不释放 | Task4b同文件；复用`senior_control/tests/test_mapping_production.py::test_native_reference_refresh_preserves_a_fresh_pose_within_the_same_generation` |
| S12 `cancel_unknown` | h1取消返回FAILED/未知终态，T此时已可达 | 不发替代，不释放原身份，不完成原action，保持pending与零输出，迟到结果只按h1处理 | Task4b同文件；复用`senior_control/tests/test_home.py::test_rejected_cancel_does_not_start_replacement`及`tests/unit/test_task_terminal_release.py` |
| S13 `reference_changed` | 在generation g1启动分析、planner查询与h1；切到g2后依次让三种旧结果返回 | 三旧结果均不能安装/发新goal/完成当前action；沿原失格撤销，不重盖当前generation；新任务不被旧结果影响 | Task1 reference、Task4a planner及4b route各负责自己边界；复用`ros2/tests/test_navigation_reference.py`和`tests/unit/test_navigation_limits.py`来源换代 |
| S14 `reverse_from_zero_accel` | previous=(0.01,0.01)，target=(-0.4,-0.4)，dt=0.05，measured_linear=0.01，fixture accel=yaw_accel=0.5，profile/净距无更紧硬约束；同一输入推进下一周期 | 换向起点=(0,0)，共同alpha=0.0625，首步smoothed=out=(-0.025,-0.025)，k=1；下一周期(-0.05,-0.05)而非重置；不等待实测归零，不直接目标 | Task3b同文件换向用例；覆盖直接目标alpha=1及旧-.015/-.01875；fixture角配置不改生产yaw_accel |
| S15 `dynamic_clears_same_grid` | native grid_id固定gridA，当前候选planner先因实时障碍no_path，等待内障碍消失后同planner reachable | 不依赖native更新解除失败记录；按1秒节流重查恢复通道，旧30秒deadline不重建 | Task4a planner/4b route；复用`ros2/tests/test_navigation.py`与`test_navigation_collision_qualification.py` |
| S16 `unrelated_update_no_reset` | A从100等至129.9，无关角落gridA→B更新，相关走廊未变；clock到130 | deadline仍130，改选/收口规则照常，无关图变化不能再等30秒 | Task4b同文件，沿S07 clock fixture但独立初始化输入 |
| S17 `later_page_candidate` | 确定性候选域17项，第一页16项已尝试失败，第17项在后页且planner reachable；重复同cut并模拟新计算取消 | 游标继续后页找到第17项，不前16失败就exhausted；已尝试覆盖不因缓存淘汰重游；取消在checkpoint退出、无后台堆积 | Task2分页与Task4b账本，`test_frontier.py`/`test_navigation.py`；复用审计现`test_navigation.py`新增前沿断言位置 |
| S18 `lower_target_transition` | previous=(1,1)，target=(0.4,0.4)，dt=0.05，decel=yaw_decel=0.8，cap/yaw_cap及净距不更紧 | 普通同向减速alpha=1/15，smoothed=out=(0.96,0.96)，输出大于新目标但k=1且始终<=1，无比例伪冲突 | Task3b同文件减速用例；同alpha平滑过渡、共同k原因可追踪 |
| S19 `pause_clears_held_target` | 真实core rig已有导航非零target；进入等图/切goal，pause；旧样本/未accept新goal；最后真实accept并送新鲜样本 | 下游消费prepared零barrier后held target真实零/idle，保留task_id/epoch/subscription/GID，不封epoch；accept前不恢复，后新鲜样本恢复；整路线最终仍需原退休证明 | Task4b `tests/unit/test_navigation_limits.py`；复用`tests/unit/test_navigation_core_output.py`和`tests/unit/test_task_terminal_release.py`真实core rig |
| S20 `zero_pair_measured_trace` | previous=(0.01,0.01)，target=(-0.4,-0.4)，measured_linear=0.2仍前进，dt=.05，fixture accel=yaw_accel=.5，front=0、rear=0，其余profile合法且证据新鲜 | 从零反向alpha=.0625、smoothed=(-.025,-.025)；rear硬上限给k=0，out成对零，trace保留measured=.2、front/rear及方向不一致；实测方向前方覆盖缺口仅源码推导待复现，不伪报已停车/更快制动，不把front=0单独当阻止反向依据 | Task3b同文件与tests/unit/test_navigation_core_output.py真实core rig；同一输入记录实际反馈，不扩异常矩阵 |


被替换编号的有效保护继续复用，不增加选定独立输入：原S06旋转origin/floor断言由Task2坐标函数与现有workspace坐标回归承担；原S14同参考增长/raster≠anchor由Task1 reference和mapping_production回归、Task4b当前图复核承担；原S18长已知路线无60秒信息超时由Task4b现有顺序运行/生命周期回归保留；原S20同profile装配由Task3a原参数测试和后续安装验证保留。原S03图外放行未批准，不实现、不混入S03；既有非法/过时grid/outside拒绝保护保留。

观察等图复用S10子goal成功同一输入：成功时仍拿到旧raster不得完成原action、耗尽前沿或记永久无收益；随后安装覆盖新commit的raster才作收益判断。Task1在原reference install/withdraw加进展revision/单等待者唤醒，建议接口`wait_for_progress(revision, *, max_wait_s=1.0)`返回当前revision，单次有界且取消清理；Task4b及agent_adapter接此等待，缺进展不忙循环。现有NativeMappingSession建议补request_navigation_refresh()（车内方法、非新服务）：在原锁内合并单次请求，消费已有可用commit并调用现有刷新/发布路径，返回progress/no_progress；无commit不能伪造新观测，重绘旧cut也不能证明到达后scan进入图。经原bridge安装后才由reference通知；本轮不新增native协议，若确需强制scan/commit则另列范围。该接口是待审核推荐、不是已冻结源码契约；到达后观测关联、无commit与总驻留/收口策略先按设计收敛，不能拿1秒await边界冒充用户任务超时。通知/等待落既列reference/bridge/native_session/goal_navigation/agent_adapter与现有测试owner；主动刷新目前仅owner内部推荐，不授权GoalNavigator跨进程直接调用session，需先核对既有mapping请求路由能否复用，不能复用则协议/装配条件范围另估；未决定前不实施observe终结策略。

## 影响回归、安装与分层验证

**离线影响回归（后续实施环境，现有.venv可用才执行；当前不安装/运行）：**

```bash
.venv/bin/pytest senior_control/tests/test_direct_navigation.py senior_control/tests/test_workspace_data.py senior_control/tests/test_home.py senior_control/tests/test_mapping_production.py ros2/tests/test_navigation.py ros2/tests/test_navigation_reference.py ros2/tests/test_navigation_collision_qualification.py tests/unit/test_navigation_limits.py tests/unit/test_navigation_core_output.py tests/unit/test_motion_launch_profile.py tests/unit/test_arbiter.py tests/unit/test_navigation_completion_handoff.py tests/unit/test_task_terminal_release.py tests/unit/test_software_estop_owner.py tests/integration/test_task_ownership.py tests/unit/test_ros_package_ownership.py -k 'not installed_nav2_bringup' -q
```

预期全部通过；仍有效任务离线/接管/急停/重启/来源租期/取消/终端释放/安装归属按影响复用，不放宽时间或删除断言。`test_task_ownership.py`重点保留offline same identity、takeover waits output ack、restart no automatic plan；navigation_limits重点保留原始样本截止/GID/private run；home重点保留cancel未知、迟到accept不发替代。比例新决定覆盖的旧角独立、跨零换向与直接目标断言在Task3b显式列改动理由，其他安全断言不减。

```bash
.venv/bin/pytest ros2/tests/test_motion_core_perception.py tests/unit/test_navigation_core_output.py tests/unit/test_navigation_limits.py -q
.venv/bin/ruff check ros2/src/senior_navigation/senior_navigation ros2/src/senior_robot/senior_robot/navigation_parameters.py ros2/src/senior_mapping/senior_mapping/navigation_snapshot.py
```

Ruff若发现既有无关错误先区分基线，不顺带重构；新增模块类型需用仓库现有mypy配置定向检查，ROS动态入口不冒称全仓strict通过。各任务通过后扩大一次相关回归即可；没有新改动/新失败不反复全仓运行。

**安装/资源回放（需负责人另派并提供资源；不自动运行服务）：** 新Python模块应由现有ament/hatch目录安装，无需加机械CMake条目。可在获授权的隔离输出目录执行 `.venv/bin/python -m pip wheel --no-deps --no-build-isolation ./ros2/src/senior_navigation ./ros2/src/senior_robot ./ros2/src/senior_mapping ./ros2/src/senior_localization ./ros2/src/senior_motion_control ./senior_control -w /tmp/frontier-wheels`，然后核对各wheel的实际模块路径/hash与ROS-free导入。此命令依赖已备构建环境，不授权联网安装依赖；若缺环境报告缺口。现有`tests/unit/test_ros_package_ownership.py`只验证归属，不代替wheel真实安装。回放授权后用选定地图/scan资源测1,048,576格、SLAM并发、CPU/RSS、checkpoint取消延迟、shared loop心跳/IPC与pose时效；2ms是分块目标，不是已测保证，不发明通过数值。

**真实ROS/Nav2（另行授权，当前禁止）：** 在已批准Humble环境、冻结源码、资源锁和隔离输出目录执行现有colcon安装流程：`colcon build --base-paths ros2/src --packages-up-to senior_robot --executor sequential --parallel-workers 1`；随后 `colcon test --packages-select senior_navigation senior_mapping senior_localization senior_motion_control` 与 `colcon test-result --verbose`。检查实际ComputePathToPose字段/取消、global/local实际footprint与MPPI生效上限、static layer滞后与同planner动态障碍恢复、prepared零barrier真实传播；synthetic map bringup在`ros2/tests/test_navigation.py::test_installed_nav2_bringup_with_synthetic_map`会启动服务，未经授权不得执行。便携整文件命令在无ROS环境会沿原skip；若有ROS且会触发bringup，先用`-k 'not installed_nav2_bringup'`执行离线部分，不把skip叫验收。

**实车（另派，不在当前授权）：** 测所选profile整车包络/base_link/附件、实际输出过渡与真实odom、曲率变化/跟踪、制动与转弯、四核并行负载。共同k不能证明未建模侧向障碍安全，不以发零或数学比例当实际停车/轨迹通过。

## 文件增改删与工作量估计

本轮以d5e5050为文档基线，新增0/修改2/删除0；以下是未来原方案范围，不是本轮代码交付。共同运动仍只改vehicle_speed/motion_core_output，来源无关，无新增入口分流文件；观察通知/一次刷新落既列reference/native_session/bridge/goal_navigation，不借此承诺原生强制commit能力。MPPI模型改造/升级、动态speed_limit接线、原生scan→commit新协议及有效物理制动契约若确需，均为未确认条件范围，未计入下表；负责人确认具体方案后再重估，不能假称35–41已覆盖这些未知行为。

下表不重叠计数，包含最新共用比例子范围，范围仍是两份文档的未来估算，非本轮改代码授权。前述临时范围报告34–42文件被本次精确清单**35–41**替代；条件修改须记录实际触发，不能预先扩profile。

| 分类 | 新增 | 修改 | 整文件删除 | 明确落点 |
|---|---:|---:|---:|---|
| 产品 | 5 | 14–16 | 0 | 新snapshot/frontier/planner/goal_navigation/navigation_parameters；必改direct_navigation、waypoints/ports、agent/workspace、live_mapping_reference、navigation_bridge、native_session、navigation_adapter、dedicated_controller、platform_runtime、agent_adapter、command_node、arbiter、vehicle_speed、motion_core_output；条件real_runtime、senior_sim/launch/simulation.launch.py |
| 测试 | 2 | 9–11 | 0 | 新test_frontier/test_frontier_planner；必改test_direct_navigation、test_workspace_data、test_mapping_production、test_navigation_reference、test_navigation、test_navigation_limits、test_motion_launch_profile、test_motion_core_perception、test_navigation_core_output；条件test_home夹具、test_startup_learning_chain的原limit_velocity包装签名（原断言保留） |
| 配置/构建 | 0 | 2–3 | 0 | navigation.launch.py、nav2.yaml；仅证实真实安装/依赖差异才条件修改1个package.xml或CMakeLists.txt；当前已知无需改nav2_msgs依赖 |
| 工具/文档 | 0 | 3–4 | 0 | 本轮修改已有2份文档；后续ros2/src/senior_navigation/README.md更新（已核对存在）；可选更新docs/operations/navigation-monotonic-overlay.md。无新增工具脚本 |
| 合计 | **7** | **28–34** | **0** | **35–41文件**；本轮实际新增0、修改2、删除0 |

预计产品净新增/修改约1,050–1,750行（含新比例owner局部约60–120行及进展通知/等待约50–100行，最终依等图策略收敛），测试约450–750行；未来文档更新约60–120行，配置约20–50行。行数是评估量级非交付指标，不预写函数体充计划；原内部直接go循环会被最小替换，无必需整文件删除。角色拆分建议为A输入、B候选、C参数、C运动、D查询、D整合六个可审任务；不可并写的现有测试及装配文件按上述owner串行合版。

## 计划自审与交接

- **需求覆盖：** 用户最新未知绕路、相关性/闭合/临时30秒、原任务与取消、snapshot水位、无ACK复核、有界计算、同profile、共用alpha/k均有设计和Task owner；旧15/60秒与True pending已删。图外接纳独立冻结，五项比例验证已替换选定编号，观察等图收口待收敛，当前没有隐藏实施授权。
- **接口一致性：** MapRef/Pose2D/WorkspaceGrid/SampleTime复用原类，snapshot独立值模块避免workspace↔references循环；所有消费者用同一CutIdentity/geometry_id。未增加线上DTO、未知cancel替代或第二地图/速度owner。
- **步骤可执行：** 每任务列文件、精确接口、红灯输入、命令、实现决定、绿灯与提交；命令是待运行，当前未装/未测。旧安装/ROS测试可能启服务已明确隔离；不能未经授权照全文件跑服务。
- **Review Focus：** 五项分别落Task1/2/4a/4b及原回归，不以20项之外新矩阵扩大范围。真实grid/reference bug单列先复现修复，未假装已经复现运行。
- **比例复审：** 已按用户纠正区分共同插值alpha与硬缩放k，删除伪冲突门与上游重构建议；纯旋转、从零反向accel序列/零请求、实际反馈和更紧yaw上限在原owner内明确，五项输入已选；实测方向净距缺口待定向复现及制动契约收敛。未宣称每帧与新目标同曲率或完整SE2安全。
- **证据限制：** 闭合依赖同参考可信结构证据；缺数据诚实temporary/unknown。外域和乐观图是上界近似，Nav2无cut ACK；当前图整车复核不等于原子同图证明。性能、安装、真实ROS与实车均未验。
- **比例与体量：** 计划约为设计的接口/任务展开，代码块只冻结值/签名，无产品函数体。无额外工具、框架或测试实现。本轮仅两文档，可正常本地文档提交，交负责人亲阅后另派实施。

待亲阅/收敛项：图外接纳推荐是否批准（当前冻结且不在选定实现内）；观察到达后scan/commit关联及缺进展时总驻留/收口规则；部署Nav2版本/MPPI模型与共同ramp和真实odom的对齐方案（参数查证不是来源例外）；实测仍前进而命令已反向时的有效物理制动契约。用户反向从零accel序列及来源无关职责已明确，五项比例验证已选，不再把旧提案或输入选择当确认门；当前不自行开下一阶段。负责人收敛后按任务依赖另行派发，当前执行到此结束。
