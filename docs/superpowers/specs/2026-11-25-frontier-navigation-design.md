# 前沿导航设计（2026-11-25）

状态：**待负责人亲阅、收敛；不是实施授权**。本稿覆盖旧草稿相反行为，不把负责人推荐写成用户原话。工作树 `feat/frontier-navigation`，代码依据 `11a26906c9831b000b09e2279ad7ef4ea514d903`。

输入：`/tmp/frontier-navigation-audit-2026-11-25.md`、`/tmp/frontier-navigation-design-proposal-2026-11-25.md`、`/tmp/frontier-navigation-design-builder-review-2026-11-25.md`；仓库依据为 [架构确认稿](../../architecture/ROS2与车端架构重构确认稿.md)、[活动建图获批设计](2026-11-23-active-mapping-production-design.md)、[运动设计及后续覆盖](2026-11-12-vehicle-motion-redesign.md)、[控制工作区设计](2026-11-24-control-workspace-design.md)。旧15秒等图、移动60秒信息超时、free无路不探索、True接纳pending建议全部废止。

## 决定归属与目标

**用户已明确的行为**：

- 保留原始目标和任务/action身份。目标在未知区，或者目标已知但现有路被堵住，只要未知区域仍可能形成当前位置通向目标的未闭合通路，就允许探索相关可达前沿；不为无关未知区域绕行。
- 临时障碍通道仍是潜在路径，单次最多等30秒；到时寻找其他可行路线。其他候选已尝试仍不可通行，结束本次并提示“暂时无法到达”。已探测障碍形成闭合阻隔且无潜在通路时，报告“无法到达”，不发探索goal。
- unknown不是free。真实驶入只允许已知free，沿同一Nav2规划、跟踪、DedicatedController私有来源和共用最终运动输出；稳定结构、unknown、display-only和实时障碍保持分层。普通grid增长不改变原task epoch或参考代次。
- 不设固定15秒等图或移动60秒无地图增长失败；30秒仅约束一次临时阻挡等待，不能改作全部运行/新图超时。

**负责人推荐，尚未单独获用户批准**：同一可信参考下，当前raster之外的合法有限目标按unknown接纳。它与未知绕路一致：原坐标是意图，只有存在通向目标的潜在通路才探索，观察点仍完整在图内free；不许可直接驶入图外。实施计划把该入口修改单独冻结，亲阅确认前不得实施此行为，不加线上用户开关。

**本稿技术选择，供负责人审核**：现有RouteService内增加单原目标编排、纯前沿候选与同一GridBased planner查询；有界、可取消分块计算；只读snapshot补在现有owner；当前可信图保守复核，不造Nav2 cut ACK服务；统一选定hardware profile派生几何与规划速度上限。

不做自主扫全图、UI改造、新任务/地图/速度owner、SLAM融合重写、新碰撞算法或profile扩模。仅本稿与[实施计划](../plans/2026-11-25-frontier-navigation.md)属于本次交付。

## 现状、复用与取舍

生产链为 DirectNavigation → 持久任务/IPC → RouteService → NavigationAdapter → Nav2 → DedicatedController私有cmd_vel → CommandNode/Arbiter → MotionCoreOutput → 最终guard。原Plan和持久action不插入观察pose；只有原目标真正成功才完成原action一次。

本项目Smac `allow_unknown=false`是配置选择；Nav2库支持允许unknown，但允许规划穿未知不等同主动探索。首版维持false，前沿只决定“去原目标/哪个观察点/是否等待”。同一个 `/compute_path_to_pose`、`planner_id=GridBased` 做实际查询，BT沿现有约1Hz重规划；不用第二planner/costmap，不自己生成速度。

两层确有障碍相关计算：MPPI `CostCritic.consider_footprint=true` 在costmap上评价轨迹；MotionCoreOutput → CollisionPrediction.clearance → SpeedCalculation 基于实时scan前后净距作制动/限速。职责不同，不能说完全没有重复，也不能把后者宣传为完整SE2转弯扫掠保证。新候选几何检查只作观察点和旧查询结果的保守过滤，不能代替这两层。

选择纯候选＋单目标编排，理由是沿用任务、取消和控制owner，便于离线验证。自定义BT探索插件会增加构建和黑板身份桥接；纯BFS直接导航无法验证动力学及实时障碍；首版均不采用。不用策略注册表、工厂框架或独立进程。

已发现真实bug：`waypoints/direct_navigation.py` 把 `live.map_generation` 与请求 `grid_id` 比较，并以grid_id下发执行。应先用定向回归复现，再分别以 `live.map_grid_id` 校验客户端切片、以 `live.map_generation` 下发任务；不让旧相等夹具掩盖bug。航点创建的verify_free_target保持free-only。

## 地图与一次只读快照

导航基础图是owned native scan raster，0=free、100=occupied、-1/255=unknown，1..99保守非free。native图来自历史scan，不是已经完成的稳定融合导航图。稳定层可提供多帧占据依据，但display-only不能提供可驾驶free。Nav2实时scan obstacle/inflation仍可阻塞native free。

新增 `senior_mapping/navigation_snapshot.py` 仅定义不可变车内值；不拥有地图或缓存。LiveMappingReference在既有串行上下文一次读取、不await，返回grid、MapRef、reference generation、boot、raster_sequence、anchor_cut_sequence、anchor/map/odom basis，以及真实pose、SampleTime/source tuple/sequence。bridge在既有install中传入水位。**允许raster_sequence小于anchor_cut_sequence**，同参考旧raster配新锚点合法；pose是独立新鲜样本，不是与栅格同时测量。

为闭合判断提供可选 `structural_grid` 与其cut身份：NativeMappingSession从已有ActiveMap稳定占据快照经现有render_stable_grid派生，随已有reference安装请求传递，不增服务/owner，不把其unknown当free。只接受同参考且可验证cut的占据；缺失/过旧/不一致则不给“稳定闭合”证明。此小型只读接线属于地图快照部分，不重写证据融合。稳定证据意味着多帧结构支持，不是“墙”或“永久障碍”语义分类。

各身份分开：MapRef/真generation决定坐标和资格；grid_id决定内容；raster/anchor水位决定输入证据；task_epoch/effect/action/goal handle/private run决定执行归属。旧结果保留原身份，真参考变化沿既有撤销规则；同参考增长重新验证未使用结果，不无条件重启行驶goal。anchor basis变化使pose派生结果需要当前条件重验，但不强制不断丢弃所有计算。

## 潜在连接与相关前沿

采用两张**分析图**，都不授予驾驶许可：

1. 执行候选域：native精确free，四邻接从当前可信位置作可达预筛；free四邻unknown是前沿，八邻只聚类。世界坐标逆SE2后floor，cell→world取中心，支持通用旋转origin（生产native目前仅零旋转）。观察pose整车居中矩形及进出/旋转检查必须落在已知free，未知质心或窄口不作可执行goal。
2. 乐观连接图：unknown与确认临时阻挡允许“可能连接”，稳定占据支持的结构阻隔不可穿；native occupied或planner no_path无法判断种类时，诚实标临时/unknown，不冒充永久墙。采用紧凑cell图及可穿边界的符号外域；使用允许角接触的乐观邻接作上界，避免以四邻接栅格伪断开证明物理闭合。车体必要间隙检查仅删**可证明对所有朝向都不够宽**的通道，例如真实开口宽度小于min(length,width)；分辨率不确定取宽度上界，临界值保留为不确定。不能把按外接圆膨胀后的断开当闭合证明。

从当前位置节点S到原目标T，保留乐观图上可处于简单S→T通路的区域；用block-cut tree的S/T链删除经单一关节点进出、不能连接目标的死支路。只保留与该走廊相邻且执行候选域可到的前沿。不能只按目标方向硬裁剪，绕路可能先远离目标。连通上界仍可能包含实际不通区域；这是“仍可能”的近似，不声称最短、完备或精确可观测性。一个巨大unknown连通块内信息不足时允许保留多个潜在入口，不能谎称知道其内部通路。

图外不是墙：当前raster外部用一个符号外域节点及边界区间连接，不向目标方向铺无限数组；有限图外目标（待批准）接到外域T。符号域只表示尚未排除的连接，不是已证实的外部通道。已知结构可能在现图内围住S或T，仍可证无外域通路；结构触到图边缘但没有实际闭合，不能凭边缘截断判死。相关性在外域上较粗，多个开放边界可能都相关，这是诚实的信息限制。

“无法到达”的充分依据是：当前可信稳定占据/整车必要几何约束下，连外域也考虑的乐观上界仍不连通，并且没有缺失或不确定的证据边。算法须保留separator/cut身份用于诊断。普通BFS无候选、一次planner no_path、资源上限或缺结构数据均不构成此证明。窄口若只能确认观察pose不安全，先排除执行goal；不能仅凭保守观察pose筛选失败宣称整车永久无路。缺证明且候选暂不可执行时，以保守“暂时无法到达”结束本次，不强行探索无关区。

## 单原目标编排、等待与收口

每个原action在同一RouteService执行范围内有内部阶段：evaluate、query、travel_original、travel_observation、observe、blocked_wait、switch_pending、terminal。内部状态不新增线上DTO。原目标已知free时总先尝试同planner；no_path但仍有相关未知走廊时进入探索。原目标unknown时选相关可达观察点；原目标变occupied不得发往占据处或替换原坐标。

观察goal成功只结束子goal。取一个**成功终态之后取得的最新可信snapshot**作观测复核；有新相关观测立即重新评估原目标，没有新信息则将本区域记为本轮已观察，换下一区域。这里不设15秒等图窗口，也不拿30秒当等新图预算。若信息仍不足且可执行候选已耗尽，收口“暂时无法到达”；这是本稿技术选择，不宣称地图必在观察终态时刷新或该观察获得了新信息。产品若以后要求专门驻留扫描，应另审行为与证据。

临时阻挡等待从该通路第一次可信no_path/执行受阻诊断的monotonic时刻起，deadline=t0+30.0；29.9秒保留该候选，30.0秒离开等待并尝试其他尚可行路线。期间用仍qualified的planner有界重查（初始技术值每1秒至多一次，同planner全局一次在途；BT活动时不额外查询同目标）。native grid_id不变也可重查，因为实时costmap障碍可能已消失。无关grid、anchor cut、轮询、失败重算不重置deadline。相关变化可提前恢复可通路线，但不能通过重复no_path给同一次阻挡续期。

候选尝试账本绑定reference和map空间区域/通路，不绑定数组行号或goal UUID。无收益观察与临时no_path分别记录；每批16只是分页。遍历游标覆盖后续页，已超时通路不因缓存淘汰重入；有真实相关新信息才更新可尝试域。记录容量用native单图上限1,048,576格内的紧凑覆盖位图＋游标，不用256条LRU遗忘制造无限轮转。跨extent/origin用map区域对应；超出容量明确temporary/resource diagnostic，绝不伪报永久闭合。其他候选均尝试仍阻挡时不再循环30秒，结束本次“暂时无法到达”，原持久目标保留供后续显式执行。

planner timeout/unavailable不能伪装结构no_path；有资格但查询故障采用现有action请求截止和有限重试（初始3次、间隔1秒），耗尽报暂不可用。此预算仅查询故障，不约束行驶、等图或整个任务。planner lifecycle失活、定位/来源失格沿现有撤销，不用重试屏蔽资格丢失。长已知路线超过60秒无地图增长仍可继续。

## 子goal切换与真实停转发

生产 `RouteService.start(plan, wait_for_goal=False)` 仍是有界本地预留：starting、index=None、completed为空，不等计算/planner/新图；真正goal接受且绑定source后才running。`True`兼容语义保持原样，等待真实首goal开始，不新增1秒pending异常或假True。

`clear_navigation_source` → `Arbiter.end_navigation` → shutdown会封原epoch；仅在整路线终结/撤销走该退休路径。**子goal切换和等图绝不能调用它。** 在既有CommandNode/Arbiter补非终结暂停：保留execution/epoch、publisher subscription/GID和DedicatedController上下文，禁收/禁转发非零、清Arbiter旧velocity/deadline并发布新代次zero barrier，使下游held target真实清空。此barrier沿现有输出协议，不伪装task_done/task_revoke或terminal release。样本在暂停期间不续期，恢复不能重放旧target。

切换顺序：暂停转发并清旧target → 取消旧子goal → 等该handle真实terminal → 当前可信图/参考复核 → 发新goal → 真实接受后启用转发（仍检查原资格与新鲜样本）。未确认取消只保持switch_pending/cancel_pending，不发替代、不释放原任务。终态后迟到结果按handle/原action/epoch丢弃。第一次goal之前没有绑定则保持输出关闭。同一次DedicatedController来源跨子goal保留，不因切换退出上下文。

## 同planner查询的证据边界

查询记录原task/action、输入snapshot、真实起点和profile身份；回包先拒绝跨参考/任务/goal的旧结果。正结果必须map frame、非空有限、端点满足既有Nav2配置容差，并在当前可信native图做**完整矩形包络沿整段路径含yaw变化的保守扫掠覆盖**，不能只查中心/离散顶点。可用段长和转角引起的最大顶点位移≤resolution/2的细分，再按该误差保守扩张矩形作覆盖；不知道的扫掠格拒绝，不能漏掉中段墙。实际Nav2仍独立验证动力学、costmap和实时障碍。

`/map` publish返回、costmap时间戳/几何、同实例readiness都不是native cut ACK；不增加ACK服务。新图后的no_path可能是static layer滞后，视暂时结果并有限重查。当前图复核能过滤旧正路径，不能证明Nav2使用同cut，也不能证明动态障碍状态与查询瞬时一致；实际发goal前再检查，BT持续重规划，最终运动层继续限制。这是分层保守证据，不叫同cut原子证明。

## 一份profile、运行预算与验收

以既有平台装配选中的HardwareMotionProfile为唯一几何/物理数值输入；base_link居中±length/2、±width/2沿现有real_runtime约定。新增只读派生模块在平台装配生成同一参数文件，传给planner launch和DedicatedController实际启动参数，同时候选接同一content identity与矩形。MPPI vx_max=profile.cap、vx_min=-profile.cap、wz_max=profile.yaw_cap，替代当前0.5/-0.35/1.9硬编码；保留规划动力学、lattice转弯约束和最终控制限速，不把规划上限当最终许可。costmap padding/inflation按原参数职责保留，候选不重复叠sensor_error。现profile没有Nav2加速度专用字段则不编造映射或顺带改算法。

当前0.55×0.55 profile仍标ASSUMED；旧Nav2 0.674×0.54外形与它不一致应修正，但软件一致不等于实物已测。不从四轮UI推几何、不预先扩profile；base_link、附件和满载外凸实测留实车层。

重计算采用shared asyncio内可取消分块，每处理最多2048单元/边操作或2ms（先到者）检查取消并yield；长连通/聚类/距离/覆盖检查都拆分，不能仅create_task包装同步全图。一次计算和一次planner查询在途，更新合并到最新输入，旧计算退出后才启动新计算，不排后台队列。紧凑bytes/array，最多当前/上一输入及单图有界账本；不声称线程消除GIL或已满足四核预算。

验证分层：本轮仅文档静态自审；后续离线所选场景（S03获批后纳入）及现有回归 → 授权资源回放量CPU/RSS/取消延迟和shared loop时效 → 授权真实ROS/Nav2参数/action/取消与安装验证 → 实车几何及运动验收。当前未运行测试、构建、服务或设备，不能声称已通过任何一层。

## 2026-11-25 后续用户覆盖：共用比例过渡与硬限制

本节是负责人转达的**用户新决定及随后纠正**，覆盖旧“角速度独立立即执行”条款，适用于手动与导航共用链。先以同一alpha对车体向量从已有输出向请求过渡：`u_smooth=u_prev+alpha*(u_target-u_prev)`，再以统一`0<=k<=1`施加硬限制：`u_out=k*u_smooth`。alpha与k不是同一个量；目标降低时平滑输出可以大于新目标，不因此要求k>1，也不把它当必须再问用户或上游重构的冲突。

Nav2输出车体(v,w)意图，不是轮速；末端驱动才转执行器。旧实现只降vx保留wz会改变曲率；共同k保持**当帧平滑意图**曲率。起止曲率相同的共同插值保持曲率；起止曲率不同的正常过渡不能声称每帧与最新Nav2目标严格同曲率。实际动力学、反馈、跟踪和转弯扫掠仍需验证。

实际owner是safety_gate进程内MotionCoreOutput → CollisionPrediction.clearance → SpeedCalculation.limit_velocity；CommandNode主要接纳/转发，motion_output/guard最终输出，不在那里另算比例。碰撞owner保留现有前后净距事实，不改成完整SE2碰撞模型。所有硬上限合成为同一个k，禁止先各轴裁剪、再比例或下游第二次独立裁剪。

最小技术选择：在原vehicle_speed.py增加frozen `SpeedLimitDecision(velocity, smoothed, alpha, ratio, reasons)`与 `limit_velocity_decision(...)`。原limit_velocity六位置参数/Velocity返回值保留、委托同一计算；SpeedCalculation.calculate仍走该原入口，并从该次计算取得decision供trace，不绕开现有monkeypatch。无模块全局诊断缓存，无二次限速计算。

alpha由target-previous各非零分量的允许增量比得出，夹[0,1]：线性同向加速用profile.accel、普通同向减速用profile.decel；**换向整段用反向profile.accel**，不得被旧reversing/decel或measured判定改回分段停车；dt与measured_linear继续提供真实反馈/制动事实；角向可用**已有HardwareMotionProfile.yaw_accel/yaw_decel**，不发明新角加速度常数，不把ASSUMED配置说成实测。零差分量不除零；统一alpha取各上界最小值。真实速度反馈只提供原测量事实，不能伪造完整车体反馈或延长其截止。

**用户最新换向决定覆盖先停零方案**：从已有输出直接朝有符号反向目标按profile.accel连续推进，同一alpha用于(v,w)，不设零中间目标、不丢当轮越零剩余步长、不插等待帧。例如v_prev=0.01、v_target=-0.4、accel=0.5、dt=0.05且无更紧约束时，alpha=0.025/0.41，v_next=-0.015而非0。现_approach是有限线性步进，不描述为渐近无限接近零。真实轮速反馈继续参与硬制动诊断，不能被命令值替代，也不再要求静止确认才换向。双零请求、来源撤销、资格失效、急停保持既有立即停止优先，不能由平滑续命。纯旋转从静止出发只用yaw差量计算alpha，不除v；若尚有平移先按原停止优先清零，不由插值保留已明确停止的平移意图。不增加上游状态机/可写配置。

k综合profile.cap、yaw_cap、按**平滑后实际输出方向**的前后净距制动上限；每个非零分量给比例上界，零分量跳过。k=0同时零，yaw更紧同时降v；硬制动优先于alpha，不能为平滑越过硬上限。记录同一次决策的alpha/k/reasons，缺证据保持原失格/零输出，比例保持不等于原地转向或完整扫掠安全。

静态profile约束对齐Nav2，真实odom反馈维持跟踪；持续动态限制的前馈可评估MPPI speed_limit，但当前无装配证据，**本阶段不新增接口/节点**，也不把官方velocity_smoother当本项目现状。原20导航输入不变，运动补充候选在计划单列待负责人选择；不拿旧角独立测试否定新用户行为。
