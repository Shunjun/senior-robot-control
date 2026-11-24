# 活动建图生产实现计划

> 设计：docs/superpowers/specs/2026-11-23-active-mapping-production-design.md。需求来自获批E/G规格；用户已于2026-11-23批准实施。继续feat/vehicle-architecture-refactor，分批验证/提交，更新PR38。

**目标：** 完整接通活动建图、自动恢复、地图管理/每图Home、定位协作及网页操作，并在四核树莓派预算下验证算法正确性与性能。

**范围预报：** 原建图范围约55–90路径；新增性能遥测预计额外12–20文件，更新合计约70–110路径：产品新增10–18/修改30–46，测试数据新增4–8/修改12–22，配置构建修改3–5，工具文档新增/修改5–8。此为开始实施前已向用户汇报的总范围；本批进度见执行台账。

## 1. 只读候选用例与接口核对

- [x] 核对ActiveMap、WorkspaceMap、MapOperations、SlamAdapter、Relocalizer、agent_adapter和网页MapManager的实际调用。
- [x] 精读并复用senior_mapping/test中的引用、占据、归档、事务、active-map及ros2/tests/test_map_operations.py与地图工作区集成测试。
- [ ] 先审查现有算法的定位/环境变化区分、动态障碍更新、图栅格一致性与恢复正确性，并评估线程、算法复杂度、内存和I/O；不能默认算法无问题。
- [x] 核对后端live观测/图捕获/重定位接口、配置、Home唯一存储和operation_id链路；固定线上契约，记录所有状态/坐标/时间/版本语义。
- [x] 对下面候选逐项审核并确定实际场景后，才编写测试和实现。原有效回归保留。

## 2. 活动地图与一致持久化

- [x] 新增senior_mapping的session/catalog/checkpoint owner，扩展active公共接口，复用references/store/operations持久边界；不从外部访问私有_store。
- [x] 新建、唯一活动会话、可信更新、稳定MapRef与独立grid_id、每图Home。
- [x] 完整checkpoint自动落盘、重启恢复和未完成版本处理；数据与状态同代提交。
- [ ] 实现有界重计算、增量更新、合并checkpoint与最长延期；重计算和I/O不阻塞控制/通信，处理旧会话取消与队列容量。具体预算由四核目标设备实测确定。
- [x] 独立回放与真实临时数据库/目录验证后，审阅diff并提交。

## 3. 后端/定位/生产入口

- [x] 复用native SLAM capture/graph/scan-match适配及必要ROS接口；消费已验证来源，不以无身份/map topic代替。
- [x] 切图与删除生命周期、旧会话异步结果退役；自动匹配、Home待校准标记、手动位姿局部微调。
- [x] 建图进程与本地IPC、real/sim组合及配置；接入agent_adapter、WorkspaceReader、任务地图引用与定位/导航资格。
- [x] 实际服务边界/串行化测试、原生可用环境检查及安装验证；审阅并提交。

## 4. 线上契约与网页

- [x] 用户取消预览确认，直接接入现有控制页：性能面板、地图目录及新建/切换/删除请求；未就绪运行时明确禁用。Home/车位定位生产协作已在后续批次接通，验证限制见文末。
- [x] contracts、云端路由、manager消息处理、web协议/RobotLink及目录请求按实际线上消息同步更新。
- [x] 对接原地图画布、HomeControls和MapManager，处理更新/切换/失败状态；删除临时预览。
- [ ] 定向网页测试、类型/构建与实际UI检查；不擅自启动开发服务。

## 5. 低频性能遥测

- [x] 只读盘点现有Metrics、状态快照/心跳、真实/仿真进程登记及云端状态转发，复用可用数据，明确宿主/容器与CPU归一口径。
- [x] 实现独立低频采样器与缓存，默认1秒；整机CPU/内存、视频/导航/建图归属、身份重启识别及有界开销。不通过top/ps子进程高频采集。
- [x] 沿现有连接更新实际线上摘要契约、云端转发、网页状态；旧端缺失指标不影响控制，必要兼容明确标注//Compatible。
- [x] 按用户要求直接在原控制页面接入性能面板；区分未启动/不可用/过期，进程内存不伪装成独立线程内存。
- [ ] 验证整机/模块统计、重启归属及端到端展示；测量监控自身开销与控制/心跳影响，和建图负载验证共用采集证据。

## 6. 整链路与交付

- [ ] 新建→可信更新→自动落盘→重启/切图→自动/手动重定位→继续更新→每图Home→删除的完整链路。
- [ ] 测量四核设备CPU/线程、控制周期时延、建图/定位时延、队列峰值、RSS和保存耗时；同输入比较优化前后，覆盖驾驶/视频并行及地图增长/后台重计算。
- [ ] 各批有效回归和最终全量检查；隔离wheel/ROS安装验证。记录未执行的native/实车与性能项。
- [ ] 本人亲审diff、状态归属、复杂分支及删除项；同步文档，推送既有分支/更新Draft PR，不合并main。

**顺序：** 接口/测试盘点→session及checkpoint→后端/生产生命周期→真实线上接口与UI→整链路。UI预览可在后端阶段独立准备；在接口冻结前不并行修改同一组装文件。沿用用户批准的当前Codex执行方式；本环境仍无orca命令，不伪造worker派发。

## 候选场景（只读阶段，尚未编写测试）

每项最终落地时记录具体输入、预期和复用节点；不展开无必要参数矩阵。

1. 新建地图启动唯一会话；Home绑定开始时可信车体位姿，与其他地图独立。
2. 连续可信更新保持MapRef/航点身份，内容改变才更新grid_id；旧tile不可混读。
3. 暂态障碍与稳定地图分开；多帧可见性证据使稳定变化生效。
4. 自动checkpoint完整绑定图/栅格/参考/Home，重启恢复最近完整版本。
5. 中断的未完成checkpoint不替换完整版本，现有事务故障回归优先复用。
6. 持续更新合并落盘，切图/关闭flush不遗漏最后一个已接收切片。
7. 切换成功后旧地图会话及异步结果不能覆盖新地图。
8. 自动匹配成功才开放导航与持久更新。
9. 自动匹配失败只显示目标Home待校准标记，不发布假位姿。
10. 用户位姿经局部匹配确认后提交，失败保持待校准。
11. Home编辑只改指定地图Home，车辆位姿编辑不移动Home。
12. 删除非活动地图不影响当前地图；活动图删除先退役使用且不误恢复旧图。
13. 重复operation_id幂等，冲突请求不产生第二个会话/删除/写入。
14. 生产IPC→manager→云车→网页读到实际活动版本；普通更新不使任务地图引用失效。
15. 网页实际操作新建/切换/删除，状态和异常有明确反馈。
16. 网页Home与车位朝向交互及待校准标记分离，无手动保存步骤。
17. 重计算繁忙时任务/队列/内存有界，旧会话可取消，证据序列和地图引用不被合并策略破坏。
18. 闲时任务有最长延期，持续输入下checkpoint仍按承诺推进；切图flush恢复到完整状态。
19. 真实系统计数器窗口生成CPU/内存与模块归属；模块退出/重启后旧PID不误归属，线程内存不虚构独占值。
20. 低频遥测经现有云车链路到控制页，缓存/样本新鲜度与未知状态呈现正确，不阻塞控制或心跳。

候选依据与复用落点：senior_mapping/test/test_active_map.py、test_map_references.py、现有归档/事务/证据测试，ros2/tests/test_map_operations.py，现有Home和地图工作区集成用例，web/features/mapping与map测试。下一步补精确节点并审核必要性，不能把候选数量当必须新增数量。


## 性能补充状态

2026-11-23用户明确设备仅四核树莓派，要求算法审查同时考虑闲时计算等优化。已纳入设计和实施验收；当前只是方案约束，尚无本轮车端CPU/时延测试结果，不宣称性能达标。候选扩为18项，仍在上限20以内；实现前继续审核复用和必要性。


## 遥测需求补充状态

2026-11-23新增整机及视频/导航/建图性能上报与控制页展示，刷新无需高频。用户已确认默认1秒；未实现或进行车端性能采样。候选合计20项，为上限，审核时优先复用/合并已有回归，不按此数量强行新增用例。按用户要求，设计/计划本身不单独提交，与后续实现一起提交。


## 执行台账（实施起点0b193db）

用户已批准整份计划，并确认性能采样/上报1秒。计划与实现一起提交，不再单独提交文档。按executing-plans连续执行，由当前Codex实施；先做性能采集基础以支持算法负载基线，这与地图会话开发无接口冲突。UI沿现有控制页面风格，以原位置预览确认后接真实接口。

### 只读候选审核

- 已检查test_active_map.py（稳定MapRef/栅格版本、错误来源、gap/撤销）、test_map_references.py（锚点纠偏/地图增长/原子commit/冻结）、test_map_archive.py（真实目录/归档一致性/输入界限）、现有MapOperations恢复与Home事务、Relocalizer及ROS图服务。
- 重用既有场景2、3、5、13的引用、证据和幂等保护，新增断言仅在有实际新接口时加入。其余候选围绕新增生产会话、checkpoint、目录/每图Home、切图/定位与线上链路组织。总上限20，不将参数组拆分绕过上限。
- 首先实施候选19：输入两次系统/进程计数器、内存快照及进程重启，预期CPU按整机归一、RSS按页换算、PID重用不产生虚假峰值；复用真实目录fixture。候选20验证采样缓存和实际线上状态呈现，不重复已有心跳/IPC吞吐回归。
- 性能方案初始单个额外重计算任务，最终预算仍待四核设备实测。MapReferenceStore只读显示不能直接授予导航；原marker图存在无真实yaw的局限，生产采用精确图commit和相应来源/定位验证，不把marker推断成可信后端。

### 本批测试选择补记

选用候选19的计数窗口/PID重用及真实登记根子树两条路径，候选20的车云快照往返与网页新鲜度两条路径，候选6/17/18合为一个连续繁忙→在途更新→flush场景，共5项新增功能场景；不扩展参数矩阵。射线回放复用既有占据行为，并对同一固定输入比较输出与分配峰值。另增加实际bug回归一项：重复同帧误确认稳定障碍，已观察失败后修复。

原ROS命令/参数断言全部保留；mapping launch fixture仅适配新增的显式目标生命周期handler，shell fixture纠正已退役根src路径为实际vehicle_core所属仓库根。两者为测试环境修正，不退役原行为保护。


### 首批实施与验证（未完成整份计划）

- 车端默认1秒后台采样、只读缓存、带PID出生身份的有界子树归属、共享SnapshotPayload及云缓存往返已实现。相机服务及建图/导航launch登记明确所有者，不按进程名猜测。采样后再次核对身份，PID重用不传递CPU/RSS归属。视频编码线程尚未单独登记，外部启动且没有生命周期通道的进程仍为不可用。
- 性能面板、地图列表/新建/切换/删除的原位置模拟交互预览已构建；桌面/手机Chrome检查无页面错误。临时静态检查服务器均已关闭。预览由VITE_MAPPING_PREVIEW构建开关隔离，无真实连接。已异步请求用户布局确认，正式UI接线与移除临时代码待后续完成。
- CheckpointWorker的单在途、脏水位合并、非闲时最长延期、在途更新和flush测试通过。它当前只是调度器，尚未连接真实图/栅格/Home落盘，不将测试回调完成宣称为已经自动保存。
- 实际占据算法bug已修复：0、1、1秒的重复帧原先错误达到三帧确认；定向回归已观察失败→通过。正常1.5秒新帧仍完成确认。已优化清除区域临时集合，证据/掩码/终点语义不变；固定回放输出完全相同。详细限制与数据见docs/operations/active-mapping-algorithm-audit.md。
- 全量Python回归：2458 passed / 116 skipped / 1已有forkpty警告，123.70秒，日志/tmp/active-mapping-python-tests-final.log。随后加严遥测身份二次核对、子树预算和新增登记根回归，定向performance三项通过；不把后加场景计入之前的全量数字。ROS mapping fixture修正后14 passed/1原有skip。无新增skip/xfail、未删除有效保护。
- 选定功能场景实现5项：计数/PID重用、登记子树重用、车云缓存往返、网页样本过期、繁忙后台flush。实际bug回归1项（不计功能上限）。复用全部占据/地图引用/归档/事务及车端快照保护；地图定向77项通过。前端原180项及新增性能展示场景单独通过，最终前端全量与production build见后续记录。
- 实车key路径/Users/zhangshunjun/.ssh/whelltec当前不存在，本批未连接车端、未测得树莓派数据；不启动车辆/开发服务。native/ROS安装和四核性能验收未执行，继续保留Draft。
- 下一主体工作仍是生产source verifier、session/catalog与每图Home迁移、完整checkpoint保存/恢复、自动/手动重定位、真实IPC/线上命令及UI正式接线。当前批次没有宣称这些已完成。

### 下一批选择：目录持久层

复用MapDatabase连接/写锁，新增目录存储，不复制实时Home位姿：每图只保存已完成checkpoint与其Home ID。候选1/4/7/12的存储部分合为一条真实SQLite重开场景，验证两图独立、稳定MapRef、活动切换令牌、过期结果拒绝和删除约束。本条为第6项功能场景；生产source、Home事务与图恢复仍须后续组装，目录元数据本身不授予导航资格。

### 完整checkpoint存储批次选择

候选4新增一条真实目录+SQLite重开回归，复用test_map_archive中的一致图捕获/验证后端与占据fixture。保存调用现有MapArchive捕获证明，文件完整持久化后才CAS更新目录指针；Home ID必须属于同一快照。恢复重新验证图/稳定层，不赋予定位资格。序列与身份、已有事务故障场景继续复用。该场景为第7项功能场景。

checkpoint审阅补充：清理旧版本时取消保存任务，不能让新读者进入即将被删除的版本。选用候选5/7的一条真实文件/取消/读租约场景（第8项功能场景），不增加故障矩阵。

### 目录与checkpoint存储实施结果

- 8189e87提交每秒遥测基础与原位置预览，计划同批提交；d94f742提交重复帧修复与稀疏自由空间分配优化。
- 新增MapCatalog（共享MapDatabase/写锁）与MapCheckpoints：稳定MapRef独立于checkpoint版本；创建操作可重放且删除保留墓碑；活动切换代次/令牌拒绝迟到结果，已有地图的新checkpoint只允许当前已定位会话发布；重启保留地图选择并回到待校准。目录只记录冻结Home ID，未创建第二个实时Home位姿所有者。
- checkpoint复用MapArchive一致捕获/验证；完整图/occupancy/锚定Home绑定持久化后才CAS更新目录，保留当前及前一版本。读取在整个graph load期间持有租约；重计算/读写串行有界。取消后的磁盘任务继续被跟踪；旧版本清理不能与后来的读取租约竞争。未知捕获结果进入需协调状态，不持续制造staging树；它不禁止手动驾驶。
- 新增3项目录/文件场景（功能场景累计8项）：两图目录重开与迟到令牌、完整图/稳定层/冻结Home重开、取消清理与旧版本读取竞争。最后一项先复现失败再修复。目录/文件fixture用真实SQLite和目录，图提供者复用明确标记的synthetic backend，不能当native或实车链路证据。
- 最新地图/事务/占据/geometry/遥测相关回归142 passed，4.96秒；随后仅收紧checkpoint任务/读租约边界，目录及checkpoint三项再次通过。前端完整181 passed、production build成功。未新增skip/xfail。
- 图来源验证、恢复后持续ingest、Home实时事务协调、生产入口/IPC、自动/手动重定位和正式网页接线仍未完成；这些纯库组件尚未在车端启用。UI布局问题仍等待已发出的异步确认。

### 恢复后流绑定

候选4/7/14选一条恢复既有锚点/占据/Home后，在新已验证源上从新boot第一条连续commit继续更新的场景（第9项功能场景）。普通bind_ingest继续拒绝非空图，不能用兼容旁路削弱保护。另已复现实际bug：replace同MapRef快照后旧binding仍可commit；先失败→修复撤销，不占新增功能场景数。

### 活动图公共接口选择

候选1/11/14增加一条Home绑定更新与后续连续观测场景（第10项功能场景）：Home更新必须保留当前流绑定、原MapRef和参考代次，并拒绝旧观测水位。现有place_waypoint仍保留撤销流的旧保护；新路径须显式传当前绑定与水位。此接口只协调地图内冻结绑定，不代替实时Home事务所有者。

候选4/14增加公共ActiveMap→Home绑定→真实checkpoint→已验证恢复的单条文件链路（第11项功能场景），验证调用者无需接触私有_store。source token只是fixture中的上游验证结果，仍不冒充真实ROS来源验证或定位。

### 恢复与公共接口实施结果

- replace同MapRef快照会撤销旧流；已复现旧binding仍能commit的失败，再修复并保留回归。resume_verified只创建新writer并验证本地结构/容量/锚点一致性，必须由上游先验证实际loaded graph、来源代次与定位，不把字符串或校验和当外部证明。
- 恢复保留原MapRef、占据与Home绑定；新boot按新流第一条连续commit推进。锚点不变时跳过重复坐标/朝向换算，修复了恢复用例中Home朝向因反复归一而漂移的失败。
- ActiveMap新增窄公共接口：有绑定/水位保护的waypoint更新、dirty watermark、checkpoint委托及已验证恢复。调用者不读取私有_store。已有place_waypoint/bind_ingest保护不放松。通过新增真实目录链路验证Home进入同一快照并可恢复；这不表示实时Home数据库事务已经对接。
- 此批功能场景累计11项，仍在整项上限20内；实际原有bug回归2项（重复帧误确认、replace后旧源仍可写），另有选定恢复路径覆盖Home无变换时的数值漂移。对恢复、连续摄入及事务的定向测试131 passed；随后公共Home/checkpoint接口增加后再次执行相关地图回归。先前最后一次全量2464 passed / 116 skipped / 1已有警告，124.03秒；不把之后新增两项计入该全量数字。
- 四个wheel离线构建并隔离安装，哈希比对验证core/contracts/manager/mapping实际源文件，ROS-free import通过；之后公共接口扩展仍需重建mapping wheel再核对，不能沿用旧包结果。
- 找到了实际私钥路径/Users/zhangshunjun/.ssh/wheeltec。只读SSH尝试未成功：wheeltec.local当前无法解析，因此无车端CPU/时延基线。没有改成IP猜测连接、没有启动车辆服务。

公共接口最终定向回归：146 passed，4.97秒。功能场景11项、实际原有bug回归2项已分别列明；保留原Home/地图/事务保护。此批无开发服务或车端服务启动，UI正式接线仍等待原位置预览确认。


### 用户更新：直接接线

2026-11-23用户明确“直接对接吧，网页端之后有重构计划”。本轮取消待确认的UI预览步骤，直接在现有页面接实际状态与命令，移除临时预览入口和代码，不启动另一次视觉重构。先把候选20现有网页场景迁移到真实ControlSurface，已观察未接入时失败，再接真实snapshot.performance；仍是原场景，不新增计数。本轮产品预计新增3–5/修改12–18/删除1，测试新增1–2/修改4–7，配置文档1–3，共22–36路径，属于原总范围内。

直接对接候选审核：新增两条功能场景（累计13项）。一条网页目录读→新建/切换/删除→刷新，以及迟到请求隔离；一条共享契约→云转发→管理操作日志→实际本地IPC→目录/生命周期端点→返回，复用既有SQLite/一致图fixture。原有性能场景改为真实ControlSurface消费，不重复增加场景。缺少生产SLAM runtime时目录必须明确报告不可用，禁止把目录行创建成功冒充开始建图；此处不因网页重构计划改变图/定位的验证边界。

对接兼容补记：旧车必须先在快照中明确声明mapping_protocol_version=1，网页才发送新增地图请求，避免未知消息使旧车控制连接被关闭。已有页面性能场景中增加旧协议地图页不发送请求的路径；按独立行为计为第14项功能场景。已观察缺少门控时发送了新请求的失败。另已复现原管理日志将OPERATION_UNKNOWN记作FAILED的问题，修复为NEEDS_REVIEW；以已落库后丢失回执的输入补定向bug回归，不扩展一般异常矩阵。


### 现有网页直接对接结果

- ControlSurface直接消费1秒性能快照；MapManager接入有界目录读通道与地图生命周期命令，移除临时MappingPreview及构建开关。不涉及网页视觉重构。manager显式声明mapping_protocol_version=1才开放新增请求，旧车仍能使用原功能。
- 目录读与栅格共享单在途队列；地图修改沿已有journal/map lane，通过实际IPC委托显式运行时。目录代次、operation_id回执与车辆作用域均校验；未确认结果只查询同一operation_id，不再次发送修改。manager重连目录不会撤销实际建图源租约。
- 本人审阅实际diff与线上调用：车端AdapterCore.main尚未注册生产SLAM runtime，因此目录正确返回unavailable，禁用新建/切图/删除。真实Home事务、source/graph验证、持续ingest、重定位和实车验收继续待办；测试Runtime明确为synthetic，不当作已完成生产建图。
- 选定功能场景累计14项（原11项+网页目录生命周期+云/journal/真实IPC链路+旧协议门控），未超过整项20项上限；实际bug回归新增1项：落库后丢失回执，原日志错误写FAILED，现保留NEEDS_REVIEW（原有两项bug回归保留）。原导入/导出/清除、Home/地图/事务及控制回归全部保留。
- 全量初检2466通过/2失败/116跳过：一项Docker未加入PATH；另一项相机fixture恢复后立即退出，导致有消费者的监管器按设计重新启动。修正fixture令恢复叶子持续存活，同时保留只恢复相机、不重启其余owner的全部断言；更新旧根src路径，不改产品监管逻辑。
- 最终命令`PATH=/usr/local/bin:$PATH .venv/bin/pytest -q`：2468 passed / 116原有skip / 1原有forkpty警告，131.78秒。日志/tmp/mapping-direct-full-final.log。定向相机/部署/地图IPC/agent配置26 passed / 1原有skip；真实IPC链路使用实际SQLite、UDS及序列化，不代替native源验证。
- 前端`npm run build`与`npm run test -- --run`：构建成功、182 passed。随后仅加强旧地图测试spy对所有请求的断言，定向4 passed及typecheck通过。生产bundle在原/robots/r1页面以mock传输检查性能展示、真实目录请求及运行时未就绪禁用，桌面/手机无页面错误；临时静态调试服务已关闭。
- contracts/manager/mapping/robot四个wheel离线重建并隔离安装；含已有core的`python -I -S`检查安装来源、当前源文件哈希和ROS-free imports通过。新增目录端点/manager组装定向mypy通过；Ruff检查和格式、CRLF原文件适配的diff空白检查通过。
- 本批验证对应本次直接接线实现及fixture修正提交，不含部署、服务重启或车辆动作。PR继续Draft，整份建图计划仍未完成。

### 串行切图修订（用户明确批准）

切图不支持行进中执行，不让新旧会话并行。停车后进入loading，先关闭观测入口（期间新数据丢弃），等待已接收数据处理和checkpoint完成，退出旧SLAM进程，再加载目标图并完成自动匹配/待校准初始化。清空输入缓存后重开入口。保留现有完整文件保存、错误保护与必要时间戳校验；不增加跨会话并行/迟到结果协议。

本批使用executing-plans继续当前Codex直接实施，功能主分支不变。预计产品新增5–7/修改8–12、测试新增1–2/修改2–4、配置文档修改3–5，共19–30路径。剩余候选选用：15串行切图的loading/旧图flush/丢弃窗口；16真实文件/DB连续建图与重开；17本地生产后端桥及原生切换接线；18保存失败不加载下一图。复用现有checkpoint、占据、Home及来源保护，最多新增6项，不重做已完成14项。

### 串行生命周期与生产组装实现

- MappingRuntime现已由manager生产factory创建，负责唯一会话、停车检查、loading期间拒绝入口、旧会话flush/退出、新建/恢复及活动/非活动图删除。已知不完整目标在退役当前会话前拒绝，不能因为点击失败创建留下的目录行而中断好地图。
- NativeSession消费连续的原生graph commit，复用OccupancyMaintainer、ActiveMap、MapCheckpoints与CheckpointWorker。CPU更新、投影、图文件复制和校验移到后台；最多一个建图重任务。每秒缓存停车状态，停车优先保存，行驶时默认最长10秒请求保存；该值是初始配置，不是已经测得的树莓派预算。
- RosMappingBackend已接入真实与仿真共用agent_adapter入口，拥有唯一原生SLAM子进程和独立静态雷达TF receipt relay；资源登记指向实际子进程。启动不再只是注册空端点。新SDK接口缺失时返回unavailable，不阻断原有控制启动。保存/切图通过MappingGate关闭原生接收并取得最后commit水位，排空已接收切片再保存；重新打开设时间下限，丢弃loading窗口的缓存扫描。
- NativeGraph通过已验证本地IPC peer和自己启动的原生boot/连续水位消费数据，不接受网页提供的图/路径。大图经单个私有、限8MB的RAM-backed transfer文件传递，控制IPC仍保留8KB上限。捕获校验原生pair digest、精确锚点和commit水位后才保存；加载再次校验文件和实际安装的图。此边界服务于地图持久层，不授予运动或导航资格。
- 每张checkpoint冻结独立Home绑定；当前Home仍复用唯一WaypointStore行。加载阶段写目标Home的needs_review状态，匹配通过后才confirmed。全量实时Home编辑事务、手动位姿提示接口和导航栅格资格仍未完成，不能把本批接通误报成整份计划结束。当前自动恢复以目标Home为局部种子，并用已有稳定结构复核；不能宣称已具备任意位置全局重定位。
- 恢复不匹配时保持pending_calibration、原生入口关闭、目标图只读，不伪造车辆位姿。当前地图输出仍display_only，navigation_ready保持false；这是尚未完成导航/定位组装的明确边界，不以SLAM服务ACK或只读栅格冒充可导航。
- 审阅修复了新组装中的回环绑定偏差：原先把真实维护点换成栅格中心，旋转后两者不同。已观察真实文件链路断言失败，改为保留精确维护点；已有占据快照的最新锚点语义和全部旧断言保留，持久绑定另行增量刷新。取消时等待写线程结束，跟踪原生文件复制，最后才回收进程/目录。
- 此批选定6个功能场景：串行loading/drop/flush、旧保存失败不启用目标、真实IPC+DB+文件的持续更新和重开、匹配失败Home待核查、回环精确绑定、未完成目标不退役当前图。全项累计20项，未绕过上限。没有新增skip/xfail或删除原保护。此前三项实际bug回归保留；本批新集成问题由上述场景覆盖，不另拆假想异常矩阵。
- 全量初检仅新增接口基线与模拟ROS组装环境适配失败；原接口字段哈希全部保留并新增MappingGate。最终全量2473 passed / 116原有skip / 1原有forkpty警告，126.86秒，日志/tmp/mapping-runtime-final.log；该全量尚不含随后追加的未完成目标场景。追加保护后的地图/占据定向77 passed，mypy六个核心模块通过。此前ROS/地图/ownership72 passed。C++已有snapshot、graph_commit_history、laser_geometry、static_lidar_tf四个便携程序均编译运行成功，它们不验证新增ROS节点ABI。
- 前端完整182项及构建通过，随后loading/未完成地图按钮调整后再次执行前端回归/构建。四个wheel离线构建并隔离安装，新运行时源码与安装来源按哈希复核；mapping在最终保护后单独刷新，不能沿用之前旧wheel。
- 只读实车连接仍失败：wheeltec.local无法解析。本机Docker守护进程未运行；没有为此启动服务。因此新增原生节点/生成接口的Humble clean colcon、ROS通信/实车定位及四核CPU/延迟/内存/落盘负载验收均未完成。没有部署、重启车辆服务或发送运动指令。PR保留Draft。

退出审阅补充：实际复现了通用500ms安全清理预算提前取消最终地图保存（定向测试缩短为20ms，80ms保存被取消，exit先发生）。为生产持久化清理单独使用既有地图操作的120秒有界预算；所有运动撤权/安全动作仍保留原短期限，并先撤权再保存、保存后再退出适配器。该项是实际bug定向回归，不占上述20项功能场景；已观察失败后修复，并运行原Agent回归。模块性能登记当前覆盖原生SLAM与receipt relay，manager内部后台Python计算仍未单独拆分，整机CPU包含其开销。

类型检查范围说明：新建运行时与catalog/composition共6个核心模块定向mypy通过；ROS绑定在缺少生成包的本机使用ignore-missing-imports检查其余逻辑通过。扩查既有Agent/main与runtime_bindings时出现6项历史类型报告（gesture可空比较、learning参数注解及4个转导出声明），已对照HEAD确认对应逻辑未改；没有用ignore或修改保护测试掩盖，也不将其宣称为全仓mypy通过。

删除审阅补充：已实际复现“目录退役成功但完整位姿图仍占磁盘”，新增真实文件定向回归并修复。活动图先flush/退出，非活动图不打断当前会话；目录保留删除墓碑，随后回收该图的checkpoint目录，重复删除不会复活地图。删除路径拒绝符号链接，失败不伪称文件回收成功。此为本批第2项实际bug回归，与退出保存一并单列，不扩充20项功能场景。

### Parallels原生构建与补充排障

用户指定使用现有Parallels ROS虚拟机，不使用Docker构建。Ubuntu Linux 22.04（ARM64，4核）中实际安装slam_toolbox 2.6.10-1jammy.20260916.165114；统一CMake/package.xml为2.6.10 EXACT，与现有存档版本及适配代码一致。修复旧包迁移遗留的C++ msg/srv命名空间引用。源码冻结到/home/toby/senior-audit-20261031/active-mapping-20261123-native-v1，记录source-sha256.json并取得根.vm-test.lock，干净/opt/ros/humble underlay，顺序包构建、最多两编译任务。senior_interfaces、senior_sensor_ingress、senior_mapping三包colcon build成功（7分21秒）；仅sensor_ingress原有显式构造器warning，未改该无关源码。build.log保留于上述目录；本结果不等于实车定位/性能验收。

后续完整便携回归曾因沙箱拒绝Unix socket/本地监听失败，解除对应限制后只剩一条corrected恢复场景失败。定位为synthetic native fixture把next读取实现为不可重读的消费操作，取消丢回执后无法按原水位重取；改为保存各水位的独立快照，保留原序列连续性及地图/绑定断言。另实际暴露NativeGraph无法处理IPC内部错误码internal，错误处理中再次抛ValueError；先以定向回归复现，再将不属于领域枚举的后端错误报告为unavailable。此为本批第3项实际bug定向回归，功能场景仍为20项；不通过延长等待、skip或弱化断言隐藏失败。最终复测结果待下文记录。

Docker说明：未安装Docker或拉取ROS镜像。此前打开已有Docker误触发其已有容器自启动，已停止这六个容器；清理未引用镜像约1.114GB和构建缓存25.43GB，数据卷/容器保留，仍有7个被容器引用的镜像约3.534GB。清理后退出Docker，宿主机df显示约71GiB可用。Docker仅用于用户授权的清理，不作为本任务ROS环境。

最终便携验证：`PATH=/usr/local/bin:$PATH .venv/bin/pytest -q`在允许本地socket/子进程的环境中2477 passed / 116原有skip / 1原有forkpty警告（127.77秒），日志/tmp/mapping-runtime-final-accepted.log。功能场景20项，本批实际bug回归3项（退出最终保存预算、删除回收图文件、内部IPC错误处理）与此前bug回归分别保留；原回归均复用。四个wheel加已有core隔离安装检查当前模块源哈希和ROS-free imports通过。新运行时/catalog/composition六模块以MYPYPATH指向当前源码、follow-imports=silent定向mypy通过，不能据此声称全仓类型检查通过。前端182项及build已验证，本次未再修改前端。

首轮ROS原生测试尚有5项失败（colcon结果同时记录missing_result，不能把其重复计为更多独立场景）：graph_commit_delivery、static_lidar_tf及drift因共用helper默认use_sim_time=true但自身未发/clock；acquire_scan_receipt与candidate_live_scan仍解析旧ROS时间日志。保留原断言并修正helper时钟选择和单调采样新鲜度对比，同时将诊断日志输出为单调纳秒。此为既有场景迁移，不新增功能场景；第二轮隔离测试仍待取得结果，暂不声明ROS回归全通过。

### Parallels原生验收结果

- 干净Humble underlay下三包首轮构建成功；修正诊断日志后的增量构建再次成功，日志build-v3.log（12.2秒）。冻结备份目录使colcon一度重复发现包，修正测试命令显式`--base-paths src`，没有删除历史源码。长脚本通过共享目录传递给`prlctl exec ... --current-user python3`，避免长命令启动失败；没有重启虚拟机。
- 既有senior_sensor_ingress测试通过；senior_mapping的20个原生CTest中18项在native-test-v3.log通过，静态TF两项在native-test-v5.log定向复测通过。最终colcon汇总74条结果、0 errors/0 failures/0 skipped；包含CTest及ament结果记录，不能当作74个独立新增场景。算法Python测试在本机已有2477项回归覆盖，原生本轮明确排除mapping_algorithms，没有宣称Linux Python全仓回归。
- 静态TF修正保留原UNAVAILABLE断言、generation/来源/漂移与旧cut保护：显式保留扫描原始单调采样时间，避免用稍后ROS/单调取样推算时跨过准入门槛；另断言graph_commit_sequence=0。排障时对PENDING的假设经定向复测否定并撤回，没有放松状态断言。
- 原生production_session探针通过（production-serial.log）：初始入口关闭；打开后接收第一条commit；暂停后新扫描不增加水位；捕获实际原生图；退出旧进程且等待服务消失；新UUID会话加载同一图，实际锚点与保存值一致；重新开门后继续产生新commit。此为现有串行/恢复场景的原生复验，不代替完整车辆manager+真实传感器/定位/导航链路或全局重定位。进程使用本机DDS隔离域，无车辆运动。
- Ceres重载实测警告请求50线程、被当前构建截为4。对照2.6.10上游solvers/ceres_solver.cpp第114行确认是插件内部硬编码，未暴露ROS线程参数；OMP/OPENBLAS=1不能声称已控制其内部线程。这是明确未完成的四核线程预算/负载验收项，不擅自修改上游求解算法或以虚拟机结果冒充树莓派性能。源码：https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/solvers/ceres_solver.cpp#L114。

本次要求的串行切图、连续摄入和生产入口已组装并取得便携/原生分层证据。整份原计划仍有手动校准、完整Home编辑事务、合格导航位姿/栅格与四核性能工作，地图仍display_only、navigation_ready=false，PR继续Draft。代码主提交6bb9844；原生探针/诊断修正作为后续提交，与上述日志对应。

原生最终源码/清理审计：149个冻结文件与当前工作区逐字节一致，生成source-sha256-final.json；未发现引用本测试工作区的遗留进程。宿主共享临时脚本目录已由runner回收，VM中的构建、初始冻结源码及各轮日志保留。最终核验不是用新一轮全量掩盖中间失败：便携全量对应6bb9844，随后仅原生诊断/探针及文档改动由上述原生构建、18项通过+2项定向复测、production_serial探针及最终逐文件审计覆盖。

### 2026-11-24 剩余任务连续执行

继续executing-plans，由当前Codex直接实施，5781202为起点；沿用既有功能分支和Parallels，禁止Docker。顺序：Home保存/编辑事务→手动位姿匹配及线上交互→实际导航栅格/新鲜车体位姿与生产导航owner→Ceres内部线程预算→安装/原生链路/负载验证，过程中分批提交，不在中间任务结束处等待继续指令。

预检：Home真值仍为唯一WaypointStore；mapping保存前后缺少共享Home事务，后段重新读取revision会覆盖捕获期间的新编辑，已用真实SQLite失败回归复现。手动校准复用既有原生候选匹配和PosePicker，不能把用户seed当位姿。导航继续复用接收侧RouteService qualification及安全/执行代次保护；显示栅格不能直接切换标志授予导航。Ceres实际为CXX线程池，TBB或OMP环境不能代替options.num_threads；采用隔离的同版本求解器插件，保留上游数学与许可证，仅暴露线程预算，不修改系统安装或私有内存。

测试审核：保留全部既有有效保护。本轮Home覆盖为新增实际bug回归；Ceres50线程警告为已复现资源缺陷，单列原生回归。此前精确维护点被栅格中心替换、未完成目标退役当前图均有失败日志与修复证据，按实际bug回归单列（此前混列在20个功能场景内），因此功能场景累计按18计；余下选两条原需求路径：待校准→用户位姿→实际局部匹配→恢复更新/导航，以及Home坐标编辑→同图checkpoint/重开/导航引用一致。复用原生产roundtrip、Home、导航、前端交互与事务场景，不扩展参数矩阵。最终报告区分功能场景与实际bug回归。

Home覆盖回归先失败（保存后覆盖新值、未报告冲突），修复后通过；78项生产组装/Agent配置/Home现有回归通过，3个相关模块定向mypy通过。checkpoint前记录的Home revision用于后段CAS，不再重新读新revision授权覆盖旧绑定；保存/恢复/实际home.set端共用数据库的Home target锁，运动撤权与控制不等待此锁。线程缺陷原生回归先复现BoundedCeresSolver不存在；独立插件只改变2.6.10上游命名空间与options.num_threads，保存原始hash/修改日期及LGPL许可证，动态库和可重建源码随包安装。尚待实际编译/线程报告复验，不预告性能达标。

线程预算原生证据：独立插件三包增量构建通过（17.5秒），production_session_runtime通过。随后同一场景改为两节点/一约束的非空图重载：实际Ceres报告6参数、3残差、Threads Given=1/Used=1、收敛；偏移种子加载后继续产生新commit。日志solver-budget-nonempty.log；未修改上游优化数学，原始source hash、修改日期与许可证随动态库源码安装。此证据证明求解器线程配置生效，不代表大图、视频并行及树莓派控制时延已经验收。继续后续校准与导航任务，不结束本轮执行。


### 2026-11-24 手动校准/Home/导航参考实现与审阅

实现`map.calibrate`和`map.home`，由共享契约、既有operation journal、生产runtime与原控制页位姿选择器串联；protocol=2才发送新增操作。校准通过停车后串行重开原生会话传递seed，匹配实际扫描后才能恢复资格，失败保持待校准；Home坐标与实际车辆位姿独立。Home编辑共享事务与revision CAS，checkpoint重开保留绑定。另修复真实SQLite回归：编辑后的Home遇到锚点旋转时原先错误地绑定到新锚点，现按原锚点重投影精确坐标，失败证据及回归保留。

原生增加同boot/commit的有界导航栅格、带原始receipt的新鲜odom读取；图commit附带当时原始odom。物理适配器唯一发布获准的/map与map→odom，子进程地图私有化且停止原生TF定时发布。参考generation与grid_id分开，缓存raster最多按闲时1秒/行驶5秒合并，仍逐条处理已接受图cut；来源/时效失效撤销位姿，切图后不回退旧图。只接受原生明确free，不把unknown改成free；长期占据/删除证据不从原生栅格回填。原有接收侧路线与安全、碰撞门保留，尚未配置Nav2参数的车辆仍不宣称可导航。

负责人审阅实际diff发现并复现：NativeNavigationBridge每次安装同参考栅格都无条件清空位姿（mapping-reference-refresh-red.log），导致正常刷新期间localized=false。修复为同MapRef/同generation保留原始新鲜receipt；参考更换仍撤销资格。定向生产/Home/Agent配置/导航已有回归93 passed，包含新增该实际bug回归，mapping-reference-refresh-green.log。未删除或弱化已有保护。

生产profile原来将navigation参数与静态reference强制配对，且无reference时丢弃参数；当前动态建图初始化不依赖预置静态图，此组装缺口已用原提交函数对同一实际参数文件复现ValueError（mapping-dynamic-navigation-profile-red.log）。已允许独立配置Nav2参数，仍检查绝对路径、必需节点及真实时钟；只有静态reference不带参数仍拒绝。对应原有生产启动回归扩展，不新增无关场景。

测试场景台账：原功能18项加获选的手动校准、Home编辑/重开两条路径，共20项；保留所有原回归。新增实际bug回归单列：Home捕获后覆盖新编辑、Home编辑锚点重基、Ceres错误线程预算、正常地图刷新误撤销新鲜位姿、动态建图参数被旧静态reference门限拒绝。退出保存预算、删除文件回收、内部IPC错误、精确点变栅格中心、未完成目标退役旧图等先前实际bug回归继续保留。没有扩大一般功能参数矩阵。

验证：完整本机2479 passed / 116原有skip / 1原有forkpty警告（126.47秒，mapping-production-complete-suite.log）；此全量在最后“刷新撤销”修复前，修复后93项相关回归通过。网页182项及build通过，最后修复不涉及前端。六个改动核心模块定向mypy及Ruff通过；不能据此声称全仓类型检查无错误。contracts/manager/mapping/robot/localization五个wheel离线重建，连同已有core隔离导入、源码hash和ROS-free导入通过。原生最终三包构建通过，21项mapping CTest和sensor_ingress回归全部通过，colcon汇总94条结果0错误/失败/跳过（不是94条新增场景）。测试包装脚本误把“100% tests passed”中的“0% tests passed”当失败而提前退出；实际构建与测试均通过，未重跑掩盖，后续单独核验冻结源码和清理。

实车连接只读复查：wheeltec.local无法解析，未连接/部署或发送运动。树莓派四核并行视频、大图增长、重计算、控制周期时延/RSS/保存耗时仍没有现场证据；PR保持Draft，不能称整份计划已全部验收。实现与获批计划同提交，不单独提交计划。


### 最终收尾验证（2026-11-24）

补充实际原生栅格/静态来源/原始odom→Python NativeNavigationBridge→LiveMappingReference的ROS链路复验，正常刷新保持新鲜位姿，来源撤销后位姿不可用；pipeline通过。初次包装脚本frame仍用probe_map而生产桥要求map，统一诊断fixture帧后继续；随后复现proof字节不等但全部typed字段相等（STATIC_PROOF_SEMANTIC_EQUAL=True，差异位于CDR填充字节）。既有静态proof探针现按全部typed字段比较，包括来源、代次、原始时钟和所有变换值，不把未定义填充当身份，也没有删字段/弱化来源断言。实际静态证明与drift两个既有原生场景均复验通过。

最终全量首次为2479 passed / 1 failed / 116原有skip，相机恢复已成功，失败在退出时8秒测试期限短于相机既有10秒关闭预算。只调整该测试等待到覆盖关闭预算的20秒，所有恢复次数、进程归属、无重复启动、全部退出断言保留，未改相机生产启停策略；该项单独4.92秒通过。此前预报相机收尾可能扩大到产品文件，实际无需新增产品改动，仍在原文件数量范围内。

修正后最终命令`python3 /tmp/wheel-final-verify.py`：本机2480 passed / 116原有skip / 1原有forkpty警告，126.68秒，日志/tmp/mapping-final-all-accepted-v2.log；LOCAL_EXIT=0/NATIVE_EXIT=0。VM实际桥复验通过，原生21项此前全通过，断言修正后的静态两项通过；最终160个冻结源码hash与当前checkout一致，未发现引用本次VM工作区的遗留测试进程。五个wheel隔离hash/import验证、六核心模块定向mypy、Ruff、保留CRLF文件的diff空白检查均通过；网页182项与build通过且此后未改前端。完整浏览器人工交互和真实Nav2路线/四核并行视频性能没有现场验收，不能以分层验证代替整车运行证据。保持既有功能主分支及Draft PR，不合main。

自动权限审核中有请求被更新请求取代，另一次审核超时而未执行；按提示重试后完成VM验证。这些是工具执行过程，不是以重跑掩盖测试失败；原始失败日志保留，实际断言问题与修正已分别记录。
