# 前沿导航设计（2026-11-25）

> **Superseded（2026-11-25）**：本文仅保留历史记录，已由[2026-11-25-shared-obstacle-layers-design.md](2026-11-25-shared-obstacle-layers-design.md)取代；旧要求与旧实施授权状态不再作为开发依据。共享障碍层行为已获用户批准，后续按新设计及新计划由负责人亲阅后分阶段派发。

状态：**设计已由负责人亲阅，本次修订及计划仍待亲阅、收敛；不是实施授权**。本稿按用户最新“嗯 可以”及后续职责纠正同步反向规则，覆盖d5e5050中直接反向目标的误读；其余接口推荐仍待审核，不把负责人推荐写成用户原话。工作树 `feat/frontier-navigation`，代码依据 `11a26906c9831b000b09e2279ad7ef4ea514d903`。

输入：`/tmp/frontier-navigation-audit-2026-11-25.md`、`/tmp/frontier-navigation-design-proposal-2026-11-25.md`、`/tmp/frontier-navigation-design-builder-review-2026-11-25.md`；仓库依据为 [架构确认稿](../../architecture/ROS2与车端架构重构确认稿.md)、[活动建图获批设计](2026-11-23-active-mapping-production-design.md)、[运动设计及后续覆盖](2026-11-12-vehicle-motion-redesign.md)、[控制工作区设计](2026-11-24-control-workspace-design.md)。旧15秒等图、移动60秒信息超时、free无路不探索、True接纳pending建议全部废止。

## 决定归属与目标

**用户已明确的行为**：

- 保留原始目标和任务/action身份。目标在未知区，或者目标已知但现有路被堵住，只要未知区域仍可能形成当前位置通向目标的未闭合通路，就允许探索相关可达前沿；不为无关未知区域绕行。
- 临时障碍通道仍是潜在路径，单次最多等30秒；到时寻找其他可行路线。其他候选已尝试仍不可通行，结束本次并提示“暂时无法到达”。已探测障碍形成闭合阻隔且无潜在通路时，报告“无法到达”，不发探索goal。
- unknown不是free。真实驶入只允许已知free，沿同一Nav2规划、跟踪、DedicatedController私有来源和共用最终运动输出；稳定结构、unknown、display-only和实时障碍保持分层。普通grid增长不改变原task epoch或参考代次。
- 不设固定15秒等图或移动60秒无地图增长失败；30秒仅约束一次临时阻挡等待，不能改作全部运行/新图超时。

**负责人推荐，尚未单独获用户批准**：同一可信参考下，当前raster之外的合法有限目标按unknown接纳。它与未知绕路一致：原坐标是意图，只有存在通向目标的潜在通路才探索，观察点仍完整在图内free；不许可直接驶入图外。实施计划把该入口修改单独冻结，亲阅确认前不得实施此行为，不加线上用户开关。

**本稿技术选择，供负责人审核**：现有RouteService内增加单原目标编排、纯前沿候选与同一GridBased planner查询；有界、可取消分块计算；只读snapshot与地图进展等待补在现有owner；当前可信图保守复核，不造Nav2 cut ACK服务；统一选定hardware profile派生几何与规划速度上限。

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

观察goal成功只结束子goal，进入observe并沿既有非终结暂停清held target。**成功之后读取不等于成功之后产生新地图**，撤回“取旧snapshot即记无收益换点”的技术选择；观察等图属于本次方案必须收敛的行为，不能默认为范围外。

只读源码依据：`NativeMappingSession.ingest`仅在收到新graph commit后调用`_refresh_navigation`；后者idle时1秒、非idle时5秒尝试raster，`_publish_reference`允许新anchor cut配旧raster。`NativeNavigationBridge.install`校验两水位后安装；`LiveMappingReference`目前没有地图进展等待接口。`config/slam.yaml`另有`map_update_interval=5.0`，但不能把这个配置或成功终态当新raster保证；静止时也未证明必有新keyframe/commit。

**负责人审核用的最小推荐，尚未冻结全部收口细节**：成功时记录同参考/boot的anchor与raster基线；在现有reference install/withdraw上下文补进展revision及单等待者通知，GoalNavigator至多保留一个可取消等待，每次等待上限1秒后重验资格/取消，不忙循环、不堆任务、不新增地图owner或ROS ACK。只有同参考新commit已由更新raster覆盖（示例：基线anchor=7/raster=4，后raster>=8且anchor>=raster），才对相关区域比较真实观测收益；grid_id相同但raster水位前进也能证明处理进展，不能只等内容hash变化。只有anchor增长或pose新鲜不算raster已覆盖；该门槛仅证明输入处理进展，不保证观察pose的新scan已进入commit，不能冒称到达后观测证明。

新相关信息立即重评估原目标；有覆盖证据而本区域无变化，仅记该证据版本下本轮无收益，可换点，后续相关更新可重试，不永久标死。无覆盖证据保留observe/pending，不消耗候选或报永久闭合。等待通知/取消/资格失效均唤醒，withdraw撤销仍走原规则；单次await有界不等于总驻留有界。**剩余决定**：需要何种到达后scan/commit关联证据、静止不产commit时是请求既有owner一次合并刷新还是等待其他观察、未获覆盖时允许驻留多久及最终暂不可用收口；均需负责人收敛后冻结接口与策略，不能擅造15秒/60秒用户超时或借临时障碍30秒作等图预算。现有owner可以实施上述等待骨架，不以等待骨架代替最终产品决定。

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

以既有平台装配选中的HardwareMotionProfile为唯一几何/物理数值输入；base_link居中±length/2、±width/2沿现有real_runtime约定。新增只读派生模块在平台装配生成同一参数文件，传给planner launch和DedicatedController实际启动参数，同时候选接同一content identity与矩形。MPPI vx_max=profile.cap、vx_min=-profile.cap、wz_max=profile.yaw_cap，替代当前0.5/-0.35/1.9硬编码；保留规划动力学、lattice转弯约束和最终控制限速，不把规划上限当最终许可。costmap padding/inflation按原参数职责保留，候选不重复叠sensor_error。加速度参数必须按部署版本查证，不能凭新版本文档假定Humble已支持，详见下节剩余接线项。

当前0.55×0.55 profile仍标ASSUMED；旧Nav2 0.674×0.54外形与它不一致应修正，但软件一致不等于实物已测。不从四轮UI推几何、不预先扩profile；base_link、附件和满载外凸实测留实车层。

重计算采用shared asyncio内可取消分块，每处理最多2048单元/边操作或2ms（先到者）检查取消并yield；长连通/聚类/距离/覆盖检查都拆分，不能仅create_task包装同步全图。一次计算和一次planner查询在途，更新合并到最新输入，旧计算退出后才启动新计算，不排后台队列。紧凑bytes/array，最多当前/上一输入及单图有界账本；不声称线程消除GIL或已满足四核预算。

验证分层：本轮仅文档静态自审；后续离线所选20项（原15项＋替换的5项比例验证）及现有回归 → 授权资源回放量CPU/RSS/取消延迟和shared loop时效 → 授权真实ROS/Nav2参数/action/取消与安装验证 → 实车几何及运动验收。当前未运行测试、构建、服务或设备，不能声称已通过任何一层。

## 2026-11-25 后续用户覆盖：共用比例过渡与硬限制

本节是负责人转达的**用户新决定及随后纠正**，覆盖旧“角速度独立立即执行”条款，适用于手动与导航共用链。普通同向请求先以同一alpha对车体向量从已有输出向请求过渡；方向反转时仅把本次过渡起点设为零向量，再按配置accel增长反向幅值：`u_smooth=u_base+alpha*(u_target-u_base)`（普通过渡u_base为既有输出，换向首周期为零，后续为该反向过渡进度），再以统一`0<=k<=1`施加硬限制：`u_out=k*u_smooth`。alpha与k不是同一个量；目标降低时平滑输出可以大于新目标，不因此要求k>1，也不把它当必须再问用户或上游重构的冲突。

Nav2输出车体(v,w)意图，不是轮速；末端驱动才转执行器。旧实现只降vx保留wz会改变曲率；共同k保持**当帧平滑意图**曲率。起止曲率相同的共同插值保持曲率；起止曲率不同的正常过渡不能声称每帧与最新Nav2目标严格同曲率。实际动力学、反馈、跟踪和转弯扫掠仍需验证。

实际owner是safety_gate进程内MotionCoreOutput → CollisionPrediction.clearance → SpeedCalculation.limit_velocity；CommandNode主要接纳/转发，motion_output/guard最终输出，不在那里另算比例。碰撞owner保留现有前后净距事实，不改成完整SE2碰撞模型。所有硬上限合成为同一个k，禁止先各轴裁剪、再比例或下游第二次独立裁剪。

最小技术选择：在原vehicle_speed.py增加frozen `SpeedLimitDecision(velocity, smoothed, alpha, ratio, reasons)`与 `limit_velocity_decision(...)`。原limit_velocity六位置参数/Velocity返回值保留、委托同一计算；SpeedCalculation.calculate仍走该原入口，并从该次计算取得decision供trace，不绕开现有monkeypatch。无模块全局诊断缓存，无二次限速计算。

普通非换向alpha由target-previous各非零分量的允许增量比得出，夹[0,1]：线性同向加速用profile.accel、同向减速用profile.decel，角向用已有HardwareMotionProfile.yaw_accel/yaw_decel，不发明新角加速度常数，不把ASSUMED配置说成实测。零差分量不除零，统一alpha取可行约束，不独立裁剪w。dt与measured_linear保持真实时间/测量事实，不伪造完整车体反馈或延长截止。

**用户最新确认的反向规则（覆盖d5e5050误读）**：当前车前进，目标反向-1m/s时，指令从0按配置accel向反向增长，首个有正dt的控制周期即负输出；既不直接跳到反向目标，也不从当前正速度先减到0再负向。accel=.5、dt=1s时首步-.5、下一秒-1；实际dt=.05s时首步-.025。该例是指令序列，不是假造轮速或硬件制动保证。车体(v,w)从同一个零向量向对应目标作共同alpha过渡，线/角均受已有accel/yaw_accel约束，再施共同硬k；普通同向加减速沿已有配置。

**用户后续职责纠正**：手动与Nav2采用同一标准，Nav2输出反向意图也适用；后级不辨来源，节点不跨职责判断。任务/前沿拥有目标、等图与换路；Nav2产车体意图；原准入/仲裁只选有效意图及期限；碰撞/感知提供净距事实；SpeedCalculation只消费已选(v,w)、输出历史、必要真实反馈、配置、净距与dt；guard维持协议与时效，不重算限速。撤回“在手动入口单独表达反向语义/给Nav2例外”的推荐，不增加source、任务类型或探索阶段参数，不造重复算法/状态owner。

方向变化检测留在既有SpeedCalculation的运动过渡生命周期，用上一有效非零意图方向及当前过渡/输出判断一次换向，保存本次反向ramp进度，后续同方向周期继续增长；不能因实测轮速仍正、硬k后输出为零或每次收到新样本反复重置。最小推荐是在该实例保存方向和未硬缩放的过渡值，显式`reset()`由现有MotionCoreOutput._stop_output同步清空；零请求、松杆、失联/到期、撤销、急停、暂停清target等既有停止路径都清进度，恢复只消费重新接纳的有效意图，不让隐藏ramp续命。此接口接线是实现推荐，不授权改原身份/租期规则。静止纯旋转仅用角约束；尚在平移而请求纯旋转仍按既有平移停止优先。

S14更新为previous=(.01,.01)、target=(-.4,-.4)、accel=yaw_accel=.5、dt=.05：从零共同alpha=.0625，smoothed=(-.025,-.025)，无更紧硬约束时out相同；下一周期同意图为(-.05,-.05)，不重置。fixture角加速度显式取.5以避免与生产假设yaw_accel=.3混淆；生产必须取真实所选配置，不能为凑例子改参数。旧alpha=1直接目标、-.015/-.01875跨零规则全部已覆盖，不保留为另一有效规则。

用户反向操作旨在及时抑制前进、防碰撞；验收分别记录请求、软件输出、实测轮速和真实前进距离。负向指令从零起算不能证明车辆瞬时反转或刹停更快；现硬件说明只证实Twist下发STM32，未核实固件限流/PID/主动制动。

**实测方向的真实证据缺口（只读推导、待定向复现）**：`motion_core_output.py`将`sensors.wheel_linear(now)`传入`speed.calculate`；`vehicle_speed.limit_velocity`先按请求linear选front/rear，再按当帧vx选actual_distance，measured仅参与braking速率判定。因此命令已负而实测仍正时，现有链没有独立的实测前进方向净距制动检查，不能声称前方安全。现trace已记录measured与clearance；最小契约补充复用这些事实，在同一decision/trace明确实测方向对应的制动上限、命令/实测方向不一致及证据不足原因，复用S20真实反馈输入作定向回归；如何把此事实落实为有效物理制动仍待技术与固件证据收敛。**不能简单用实测前方距离将反向请求k压成0就宣称更安全，也不能阻止反向目标直到轮速为0**；不增加急停映射、静止确认或新场景矩阵。

k综合profile.cap、yaw_cap、按**平滑后的软件输出方向（不是实测运动方向）**的前后净距制动上限；每个非零分量给比例上界，零分量跳过。k=0同时零，yaw更紧同时降v；硬制动优先于alpha，不能为平滑越过硬上限。记录同一次决策的alpha/k/reasons，缺证据保持原失格/零输出，比例保持不等于原地转向或完整扫掠安全。

静态profile约束对齐Nav2，真实odom反馈参与跟踪但不自动消除模型偏差；持续动态限制的前馈可评估MPPI speed_limit，但当前无装配证据，**本阶段不新增接口/节点**，也不把官方velocity_smoother当本项目现状。负责人已选择五项比例验证，替换原S03/S06/S14/S18/S20，选定20项为保留原15项＋此5项；原有有效回归/安装保护继续复用，图外接纳独立冻结且不在选定实现中；不拿旧角独立测试否定新用户行为。

## 与Nav2的真实剩余接线与执行依赖

以下结论核对本地源码，均未运行产品验证；反向规则本身已确认，不再列为来源冲突。原15项＋五比例场景保持20项，下面内容复用所属场景/既有回归，不新增矩阵。

| 剩余点 / 源码证据 | 最小落点与边界 | 执行依赖或待确认 |
|---|---|---|
| 共同alpha/ramp会改变MPPI输出的时序；`vehicle_speed.py:24–61`现已有线性ramp，改共同(v,w)仍会使预测与执行有差异 | 所有意图统一进入SpeedCalculation，保留一个alpha/k owner；`calculate(requested, previous, measured, clearance, dt)`内部维护方向/过渡，`reset()`只清运动历史，原停止回调调用；诊断同次记录request/smoothed/output/alpha/k/真实wheel反馈。DedicatedController继续把实际odometry_topic接到controller，不把软件输出写回假odom | 不能宣称同profile即消除重复平滑或预测误差；后续核对部署MPPI模型、odom remap/时延及控制周期。若要绕过共同alpha、增加动态speed_limit或改预测模型，需先给具体方案并另行确认，当前未批准 |
| `config/nav2.yaml:22–25`要求.2m/10s进展，活动FollowPath停30s会先触发progress checker | 临时阻挡进入blocked_wait前由GoalNavigator暂停输出、取消当前子goal并等真实terminal；保留原action/30s deadline，等待时无活动FollowPath，恢复以新子goal真实accept为准；规划阶段尚无子goal则无需取消 | 不能把仅停转发等价Nav2暂停，不能全局把10s放宽到30s以上；取消未知仍pending/零输出、不替代。原executor bool失败未证明临时障碍；该子goal失败不直接失败原action，外层重评估/同planner查询后按可信结果决定等待；尚无提前障碍诊断时不假装能在10s前识别阻挡，更不能把Nav2进展失败写成永久墙 |
| profile .55×.55与yaml .674×.54及.5/-.35/1.9不一致；`DedicatedController`直接读params启动 | 平台同源派生footprint与速度上限，同时传planner launch和DedicatedController，候选用同identity；padding/inflation保留职责 | 本机`/opt/ros`不存在，未读取部署包、仓库未锁定Nav2补丁版本；只能证明配置不一致，不能声称部署加速度参数已生效 |
| `clear_navigation_source→end_navigation→shutdown`封epoch，子goal/等图误用会阻止恢复 | 原CommandNode/Arbiter增加非终结pause/resume、清held target与prepared零barrier，整路线才退休；等图等待响应取消/资格失效，取消不能只等新图通知 | guard只证明协议、时效和零barrier消费，不证明物理停车或同publisher中迟到样本来自新Nav2 handle；保留原GID/sequence/截止边界，不为虚构更强证明重启任务 |
| `DirectNavigation:79,93`混grid/reference；bridge先publish再install，Nav2 static layer异步 | 先定向复现代次bug，再分离客户端grid校验与执行generation；当前图完整footprint复核过滤旧正路径，no_path暂时重查 | publish返回/时间戳均非同cut ACK；no_path非永久墙；普通增长不退休/重启整任务 |
| 观察点静止可能无新commit，`NativeMappingSession.ingest:340–350`pending就返回；`_refresh_navigation:358–391`仅随commit推进 | reference install/withdraw提供revision/单等待者通知；GoalNavigator记录到达基线，以可取消单次≤1s等待重查资格；现有NativeMappingSession可提供一次合并刷新请求，在原锁内读取可用commit并重绘已处理cut，回报progress/no_progress，经原bridge安装后通知，不强造keyframe/假scan或新地图owner | 刷新已处理cut不能证明到达后scan已融合；scan→commit的因果证据、无新commit时的驻留总预算及暂不可用收口仍需用户/负责人收敛。旧snapshot不永久记无收益，不能无限等待，也不借30s/伪造15s或60s预算；未收敛不得实施observe终结策略 |
| 输出负而实测仍前进时，现限距按请求/输出方向；反馈只影响旧ramp速率 | 复用S20记录命令/实际方向、front/rear与限制原因；源码推导待定向复现，保持反向从零规则 | 前方覆盖缺口未复现、硬件制动未核实；k=0不是更快制动证明。若修复涉及有效物理制动新契约，单独提出，不擅自将反向压零/等待静止 |

MPPI版本核查（2026-11-25只读）：项目标Humble但无精确部署版本；官方[Humble optimizer源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp)的getParams/约束没有ax_max/ax_min/az_max，[Humble motion_models](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp)直接传播候选控制速度，不能声称已支持加速度模型参数；[当前main optimizer](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp)则读取ax_max/ax_min/ay_max/ay_min/az_max。这是上游分支差异证据，不是部署包取证；后续先取实际package版本/对应源码再决定支持字段与含义，不能直接把profile.accel/decel映射进去或静默升级Nav2。参数接线、真实odom与执行模型一致性仍待ROS/实车层验证，不设来源例外。

上述一次刷新只是既有NativeMappingSession内部调度推荐：GoalNavigator不直接拿跨进程session或越权驱动SLAM。当前先补真实reference通知与可取消等待；若需从导航主动请求刷新，须核对既有mapping请求路由，无法复用时的协议/装配文件作为待确认条件范围另估，不把方法签名写成已有端到端能力。

当前文档基线为d5e5050，工作区起始干净；本轮新增0、修改2、删除0，只同步spec/plan并本地提交。未来范围与条件增改删见计划分类表；未定的新制动契约、MPPI模型改造或原生scan/commit协议扩展均不隐含在原估算内，确认后重估。负责人已亲阅旧spec，最新完整plan及本修订仍待亲阅；完成文档不解锁实施。
