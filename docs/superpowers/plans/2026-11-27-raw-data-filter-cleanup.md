# 原始数据过滤清理施工计划

> 按用户已确认要求施工；本地修改，逐步向负责人报告进度。此计划不授权暂缓项自动扩入。

## 目标与工作位置

清理传感数据接收、转发及SLAM接入中已确认的重复/过度过滤，统一来源和数据顺序规则。先将Bot1的feat/frontier-navigation推送并拉到Mac同名功能分支，保留原main。同步VM已验证的2048束去限增量；未验M6、临时诊断和全旁路版本分别封存，不当作已验正式实现。迁移时记录实际HEAD、tree和补丁SHA，不用旧归档覆盖新提交。

## 本批明确规则

- 移除2048束上限，接口和直接下游同步；空ranges过滤；方向数量与ranges长度一致。
- 起止角有限；angle_increment有限且>0；scan_time有限且>=0；range_min/max有限且min>=0、max>min。
- time_increment原样传递，取消必须为零的门槛；单束NaN/Inf/负数/越界值原样转交消费算法逐束处理，不据此拒整帧。
- frame_id只检查非空；移除项目额外名称字节长度/字符白名单及对应自定义消息字符串上限；ROS本身的合法命名规则不修改。
- 原始数据不按固定年龄判过期。消费入口对每个来源/boot/generation维护采集时间戳前沿，只接纳严格更新的帧；时间戳无法确定采集时间则拒当前帧；中间层不重复判序。
- sequence保留供记录，不作拒帧依据；boot只在确认新进程接替时切换，旧boot迟到数据丢弃，不按任意不同UUID自动切换。
- generation由一个入口维护并原样传递；同boot内明确重建接收流或时钟基准重建才推进。新代清理该流旧队列/时间戳前沿，小于当前代的数据丢弃。坏帧、短暂无数据、TF单次缺失、GID变化不推进。
- GID只作日志/来源识别。唯一发布者、预设节点名/命名空间在真正接收入口统一检查一次；SLAM自己的C++接收路径执行，不绕NativeReceipt的Python IPC通道。后续不重复核验。
- 删除传感链来源退休、黑名单及异常后永久关收；当前不合格数据拒绝，后续正常数据能恢复；正常stop/close资源释放及地图/任务正常生命周期保留。
- 删除额外BoundedStaticTf凭据的定义、生产、订阅和资格门槛；保留URDF、robot_state_publisher、标准/tf_static及实际TF查询。启动完成前必需安装TF缺失报错；运行中该帧必需TF缺失丢当前帧并报告，不退休。正常动态TF按该帧采集时刻使用；合法安装配置更新按新TF计算，不固定使用旧凭据。

## 顺序任务

1. **本地接续与快照**：推送并拉取准确功能分支，保存VM候选、完整增量diff/hash与既有测试证据，核补丁只包含已验证改动；原main和他人工作区不覆盖。
2. **统一通用规则**：整理单一职责的来源检查与boot/generation/时间戳状态组件，复用实现；移除GID拒帧、sequence拒帧、固定age与重复检查，不改消息内容和采集时间。
3. **雷达接收与SLAM数据流**：落实元数据和单束值规则，去掉退休/永久关收；保留算法真正需要的输入结构检查，更新冲突的旧断言。
4. **移除额外静态TF凭据**：清理消息、producer/consumer、launch/build引用和派生消息嵌套字段，直接使用标准TF；其它支路仅做保持接口可编译/流转所必需的适配，不重写暂缓算法。
5. **内部闭环与进度交付**：对已定规则做必要单测及内部数据流转测试，检查相同规则无重复、字段原样、旧boot/代次/时间戳不倒灌、异常后可恢复及正常退出；按用户最新要求负责人仅监督进度，不增加代码审阅、独立审核或额外验收流程。不逐步启动完整环境或整链求绿。
6. **仅在全部修改完成后做最终验证**：先运行一次真车流程，最后运行一次仿真。真车目标必须真实可用并明确，不能把VM原生流程冒充真车；若目标未具备则如实标待验。任何真实失败保留，不改阈值/断言盲重跑。

每步完成：更新本计划进度并向负责人报告完成项、版本和必要测试结果，不重复询问常规后续是否允许。

## 测试范围

施工期只单元测试和项目内部数据流转测试，优先复用有效现有测试，针对已变要求迁移旧断言；不机械增加场景、不重复全量/131CTest/环境suite/冷启动批跑。ROS/Linux相关编译和内部测试可使用既有VM作执行器，代码仅在本地编辑，保持源码内容/hash一致，不在VM另写一版。硬件、完整流程和仿真留至最终顺序执行。界面不启动开发服务。

## 暂缓，本批不处理

队列/历史账本/QoS容量策略；IPC整体字节容量与故障恢复策略；图像/CameraInfo具体格式与尺寸限制；RGB/深度/标定配对策略；下游算法自身质量/协方差/地图授权及安全采用条件；网页预览额外年龄/尺寸限制。相机压缩、15FPS、最新帧明确保留。接口删除导致的必要引用清理不等于这些功能重构。

## 初步范围与交付

结合清单63个相关源码文件及静态凭据跨包引用，预计产品/接口35–55个、测试10–18个、配置/构建3–6个、工具/文档2–4个，总50–83个，包含已完成去2048上限的23文件，不重复计数。正式编辑前在本地核实际路径清单；若超出或涉及新的算法职责，先向用户说明新增范围再继续。无无关重构、系统库修改、仿真特判或自动合并main。

## 进度

- [x] 阶段1：Bot1分支已推送GitHub，本地同名分支已接续f603330；main分支引用保留。23文件2048束增量已应用，其中18产品/接口逐文件SHA与VM一致；M6未验改动未混入。
- [x] 正式计划与需求清单已落入本仓库。
- [x] 阶段2：共用规则实现已完成，原生最终检查纳入阶段5。
- [x] 阶段3：雷达接收与SLAM数据流代码已完成，原生最终检查纳入阶段5。
- [x] 阶段4：额外静态TF凭据及引用已移除，接口/构建通过首轮验证。
- [x] 阶段5：本批定向单测、原生内部DDS流转及退出结算已完成。
- [ ] 阶段6真车流程：车端待安装，尚未验证。
- [x] 阶段6仿真执行：按用户最新指令先测仿真；已运行并记录失败，未作为整链验收通过。

施工仓库：`/Users/zhangshunjun/Wheel 机器人/senior-robot-control`；分支：`feat/frontier-navigation`；基线：`f6033308f86b9804826c88b41b8519a0003ed938`。

需求记录：[过滤点清单](../specs/2026-11-27-raw-data-filter-inventory.md)。

### 阶段2施工记录

- 已增加共用SourceOrder（Python/C++）：显式确认boot切换、源代次更新、采集时间戳判序；单帧错误不把来源永久关闭。
- RawCaptureAdmission/RawCaptureGroup已接入共用规则，移除GID一致性、sequence单调性、固定age、来源退休名单；保留原始SampleTime、明确确认新boot、跨boot/代次不能配帧。
- NativeReceipt去掉GID/sequence数据拒绝、原始时间戳重复判序和派发等待年龄门槛；代次仅在时钟基准变化时推进，来源配置变化不永久拉黑。旧代IPC包跳过，不再报整个通道损坏。
- NativeReceiptGroup记录受管原生子进程的receipt_boot_id；它不是物理传感器自报的boot。标准ROS传感消息没有这些项目自定义字段。
- Python接纳组件现有generation属性是算法缓存修订号，兼容保留；协议中的capture_generation/source_generation才是本次统一的源代次，两者不混用。不扩改暂缓的算法工作器版本策略。
- 单测：tests/unit/test_source_order.py、test_raw_capture.py、test_native_receipt_forwarding.py，18 passed；C++ test_source_order在Mac以clang++ C++17编译执行通过。受影响Python文件ruff及git diff --check通过。
- 本地.venv从离线缓存补齐uv.lock已锁定的shapely2.1.2，numpy保持1.26.4；未修改依赖锁或系统环境。
- 内部socket测试只替代ROS反序列化，其余读帧/排队/派发使用实际代码；不称作DDS或整环境验证。测试夹具socket超时已与生产0.1秒一致，线程退出断言保留。
- Linux/ROS NativeReceipt.cpp尚未编译验证，等待本批接口删除完成后统一构建相关包；未启动完整环境或硬件。

迁移说明：复用本地原有干净工作目录切入同名功能分支，未创建新的功能主分支或覆盖main引用。网页单测上轮因缺依赖未执行，不标通过；本轮按相关改动补必要单测，不追加环境测试。

### 阶段3/4施工中（尚未完成）

- 范围预报更新：静态TF凭据嵌入RGBD/SLAM/导航接口，连同名称上限清理，预计产品/接口60–75、测试18–26、配置/构建4–7、工具/文档2–4，总84–112文件。已先向用户说明；六类暂缓项不扩改。
- ScanReceipt去掉原始GID拒帧、坏帧推进generation、来源退休及转发前重复stamp排序；ScanLedger的历史容量不再阻断原始帧发布。
- 共用scan_layout元数据检查：不读取单束值或time_increment，方向数量允许包含/不包含终止角两种真实驱动约定。WHEELTEC X10现有源码明确使用后者；不改原始字段。
- BoundedScanIngress移除GID/sequence/原始age/静态凭据/永久退休；消费端共用SourceOrder判序，查询原stamp的真实TF，缺失当前帧丢弃并报告。
- 正在删除BoundedStaticTf定义、生产和嵌套引用；RGBD改用标准TFMessage承载算法所需的实际变换，不带身份/接纳/退休证明。原工作队列、像素格式、配对与质量规则不借机改写。
- 当前相关Python单测32 passed；旧GID拒绝断言冲突已迁移，原RED保留于/tmp/navigation-local-metadata-flow-20261127.log。ROS/C++目标尚未编译，不称完成；未启动环境/硬件。

- 继续核调用链后的预报更新：深度/语义/运动观测与RGBD入口也复制了来源黑名单、sequence/GID拒绝；需清理到实际消费边界，避免前段放行、后段重复丢弃。预计产品/接口75–90、测试迁移25–35、配置/构建7–9、工具/文档2–4，总109–138文件，已先向用户说明。测试以既有冲突断言迁移为主，不增加环境批跑。

### 最新范围与内部检查进度

- 当前预报产品90–95、测试迁移25–35、配置/构建7–9、工具/文档2–4，总不超过143；已先向用户说明。当前实际125文件，未完成部分继续迁移，不签交付。
- 运动来源/融合/位姿、标准TF可选输入及旧来源黑名单断言已迁移；本轮首次内部测试51通过12失败，修正接口适配和新规则后62通过1失败，最后一项为夹具采集时间倒退触发真实时钟重建，继续修正夹具时序。
- 正在准备Linux/ROS必要五包生产编译（关闭测试目标只用于先检查生产编译）；之后迁移并执行相关单测与内部流转，不用编译代替功能验证。

- 最新测试迁移范围：产品90–95、既有测试及测试数据41–46、配置构建7–9、工具文档2–4，总不超过154；增加的是被删除静态凭据接口及来源黑名单旧断言的迁移，已提前说明，不增加环境批跑。
- 私有VM首版冻结源五包生产编译build01通过（returncode0、timeout=false、含Z残留为空），monitor原样44e76c。入口传输255、缺run目录导致monitor在Popen前退出均保留，不算业务测试。该编译只覆盖source-manifest01，后续源码修改尚需补编译；未签当前最终版本通过。
- 共用来源接续及运动/位姿相关内部测试75 passed；后续感知旧断言迁移初轮53通过20失败、第二轮60通过13失败、第三轮71通过2失败，剩余是新boot清队列后的返回状态实现不一致，继续修正，不掩盖红项。

### 阶段5进行中：单测与内部流转

- 当前实际154文件：产品/接口97、既有测试及数据迁移48、配置/构建7、文档2（按文件去重）；相比预报，增加的是账本间接GID匹配清理与命令运行状态既有职责保留，未扩算法/业务功能。本轮临时VM执行工具、冻结源码及日志另计。
- 移除原来只验证额外静态凭据、GID拒收和来源永久退休的旧测试；有效内容对应、结构、采集时间及算法质量回归保留。用户已再次确认旧测试可移除，不为旧断言加回过滤，不增加独立审核或环境套件。
- 来源/运动/位姿等75通过；感知支路91通过；标准TF/预览契约35通过；最后Python改动相关35通过；命令运行状态既有规则9通过。各组有复用重叠，不相加当作独立场景总数。
- 网页雷达数据协议相关20通过，命令为 `web/node_modules/.bin/vitest run --environment jsdom web/src/api/vehicleState.test.ts`；未启动网页服务。最初未指定jsdom导致9失败/8异步错误，已保留日志并纠正测试入口，没有修改产品或断言求绿。
- 修改Python共78文件可解析，ruff通过（排除已有冻结GeometryLimits默认值的B008）；git diff --check通过。
- 真正的命令运行状态会话栅栏继续使用原规则，和传感数据软失效分别维护，未取消任务/控制权限生命周期。仅删传感数据来源黑名单。
- VM首版5包生产编译通过；snapshot03四包含原生单测目标编译通过，两个monitor均returncode0/timeout=false/含Z残留为空。RGBD账本最后发现的完整消息头比较仍间接依赖GID/sequence，已改为原始坐标与内容匹配，正在仅补编译senior_sensor_ingress；不能把前一编译结果套到此最后变动。
- VM当前仅作编译、单测及内部DDS流转执行器，未运行AMCL/Nav2整链、硬件、业务服务或最终环境测试；传输255及未落地核查与真正产品结果分列。
- 误选的无关runtime_identity IPC测试在Mac沙箱bind返回EPERM，两项未执行到业务；不扩改IPC或重跑该无关套件。实际受影响MotionRuntimeReceiver由runtime_guard_ready九项验证。
- 本地证据日志位于 `/tmp/navigation-*-20261127.log`；冻结输入和VM结果位于本机会话共享目录 `navigation-raw-cleanup-input01/03/05`，完整源manifest与工作diff保留，未写Bot1源码或全局ROS库。

### 阶段5最终结果（不代替阶段6）

- 最终实际155文件：产品/接口97、测试及测试数据49、配置构建7、文档2。新增场景以共用SourceOrder与真实内部转发为主，其余复用、迁移旧断言；退役额外静态TF/GID/永久黑名单功能对应测试删除，不为旧测试加回过滤。
- 接收入口复用共用判序：配置好的本地单一bridge以不可变首次接收monotonic坐标确认新boot接替；旧boot的旧接收坐标不能切回当前来源。受管进程/会话owner也可显式confirm_boot。没有新年龄阈值、重盖采集时间或GID认证。
- 原生：受影响五包首版生产编译通过；最终接口与映射/导航/运动原生输入由snapshot03及新接口snapshot01提供；传感接入最终snapshot05单包补编译通过。IDL及复用原生输入SHA前置比对通过，未覆盖旧install或系统库。
- ingress十个目标：初次9通过、1失败；失败为测试等待条件命中首条已有记录而未等排队回调，修正为等实际新增有效消息后只复验该目标，通过。所有原断言保留，旧失败XML/源码/测试ELF另存。
- 映射三个相关目标candidate_preflight、laser_geometry、stationary_match_quality通过；导航test_current_scan_layer通过。未例行运行其它CTest或完整pytest。
- RGBD四文件共50项：首轮29通过/21失败（主要为旧ROS deadline参数和旧接口夹具）；当前实现复验20通过/1失败，最后一项改为按既有1000ms配置构造真实过期窗口后通过，算法参数未变。共用消费判序接入前曾通过的29项也在当前代码补验，29通过；当前实现完整覆盖这50项。
- 当前真实ROS/DDS内部链路：原Python RGBD producer→标准TF→原生candidate receipt一项及真实生成接口两项，3通过。不是AMCL/Nav2/数据库/保存恢复整链，不是环境或真车验收。
- 映射Python转发层最后移除重复graph检查，复用native接收入口的配置校验与失效通知；现有单测直接禁止二次graph查询，覆盖回调和定时维护，1通过。
- 所有运行monitor均无超时/无监控错误；最终登记736个PID:starttime均结算、含Z残留为空、shutdown_settled=true。故障注入单测会主动终止worker，不能把顶层测试0解释为所有子进程都0；未逐个取得的内部退出码不补造。
- 成功、失败和修正轮分别保留。证据见会话共享目录navigation-raw-cleanup-input06/07/08/09/output，每次归档均独立时间戳/SHA；最终VM数据位于/tmp/nvraw-cleanup03-20261127、/tmp/nvraw-cleanup05-20261127、/tmp/nvraw-cleanup07-20261127至09，原历史目录未覆盖。
- 本轮剩余仅阶段6：真车流程一次，然后最终仿真一次。车端此前明确为待安装目标，当前尚无可执行真车目标/安装入口；不把VM内部测试称作真车。需确定该目标或明确该步骤待验后的后续顺序，未自行启动硬件或提前做最终仿真。

### 用户指令后的仿真实测（2026-11-27）

- 用户随后明确要求先测仿真，按此最新顺序执行；真车仍待验。正式测试使用本地当前清理源码及匹配的snapshot03/05原生install，不使用历史M6或旁路ELF，不重编、不放宽参数/断言。
- 工具准备阶段的路径转义、Parallels255、观察器目录冲突单列保留；目录冲突轮pytest未启动。实际标准case共两次，不能将工具尝试算作业务执行。
- 第一次实际case：43.13秒RED。最早安全节点导入egress_receipt残留已删除_text导致ImportError；另测试私有root过长触发原Unix socket100字节限制，尚未map.create。修正仅egress_receipt旧import/旧字节检查及既有测试，相关9项通过；私有root改短，不修改IPC限制。
- 第二次实际case：51.14秒RED。map.create已成功，进入live；原生qualified155条、Q151条、exactTF关联151条，global/local各有151条ready记录，state曾navigation_ready=true，固定层实际148×118；这些只证明本轮启动/实时数据链，不能签稳定性或完整保存恢复。
- 最终暴露checkpoint flush was not durable，其cause为prior checkpoint has not reconciled；finally/close遮蔽了更早body栈，首个body异常仍N/A。live_parameters之后未记录raw_native_A，不声称稳定障碍、后续对象迁移、保存/恢复/退休原断言已完成；不为求绿继续重跑或顺手改checkpoint。
- 本轮两次实际case均observer0、driver/case1、monitor无错误/无超时；登记35/38个PID:starttime的含Z残留均空。第二轮trace含shutdown_settled、10个顶层Launch退出均0；motion子进程SIGINT时KeyboardInterrupt原样保留，未取得的内部成员退出码N/A，不能以顶层0覆盖。
- 原始源码manifest/diff、ELF/实际maps/参数、完整stderr/trace/样本和逐文件归档SHA保留于会话共享目录navigation-raw-final-simulation01/02/03/output。修正版源冻结在03/source.tar.gz，基线仍f603加工作diff；仿真未通过不是整导航或性能结论。

### 首失败继续定位及必要修正（尚未完整通过）

- 在case03原日志找到第二capture的明确原生错误`Graph capture failed: Capture slot is unavailable`。基线f603的NativeGraph只删除文件、不删除已消费的空capture目录，而原生require_free_slot按目录存在拒绝。Python现在仅在两份文件复制、hash验证、关闭、unlink成功后rmdir精确空slot；不递归、不吞未知文件/权限错误、不改checkpoint保护。真实目录回归原2失败→2通过，相关生产装配文件21通过。
- case04（实际第3次）15.10秒RED：同boot图0/initial_active=false时查询首ReadGraphCommit而冲突，未到capture。仅迁入此前VM精确M2首commit等待：原_start的10秒绝对期限内，以ReadGraph同boot/OK/有效watermark/sequence>0/last_committed_scan_valid为准；不重置期限或改UNAVAILABLE/PENDING。相关17项通过。
- 既有集成测试增加body_failure原堆栈记录，原assert不变。case05（实际第4次）75.46秒RED：首body准确为live ready60秒超时，global长期3×3/pending、local237条ready；DB已关闭是退出次生，不修DB、不混入其它轮因果。
- 同一真实LatestScanSlot/NativeNavigationBridge/NativeSession的普通输入复现首次无basis/floor0/未接纳扫描的bootstrap invalid提前撤销新参考，1失败1通过→2通过。仅隔离未绑定启动状态，不接纳任何扫描；当前R invalid、明确retired保护均保留。随后相关23项22通过1 corrected即时ready失败；仅补原assert的现有state/phase/error输出，定向复查该项1通过1.90秒，旧瞬时失败原因仍未知并保留，不称稳定性通过。
- 同步此前有实际复现证据的测试pump IO修复：一个观测在途，读取异步等待不占scan/TF spin，原异常传播、退出join。case06先私有真实DDS+IO屏障回归，1通过0.70秒，原samples已归档，不当项目通过。
- case06（实际第5次）36.76秒RED，尚未map.create：原controller_server/get_state请求等30秒无回复，原生日志明确`failed to send response ... client will not receive response`；同期真实IPC诊断localized/navigation_ready/stationary/healthy均正常。日志不是证明整个环境无问题，也不是raw过滤或capture修复被验证失败。实际controller PID1456112:starttime45929118加载/opt/ros/humble的rclcpp/rmw_fastrtps/FastDDS，既有包证据为16.0.21/6.2.10/2.6.12；不升级、不改系统库、不删服务断言或盲重跑。
- 上游对应候选：[rmw_fastrtps#842](https://github.com/ros2/rmw_fastrtps/issues/842)描述Humble/Jazzy生命周期服务请求/回复发现竞态；[6.2.10原实现](https://github.com/ros2/rmw_fastrtps/blob/6.2.10/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)在回复订阅尚未匹配时返回TIMEOUT。仅症状和实现路径相符，尚未证明本机精确根因，更不能称VM专有或车端无影响。
- 最新原始源/工作diff/单blob SHA在navigation-raw-final-simulation06；各轮工具错误、原case/observer/monitor结果、PID含Z结算与实际库/参数完整保留。实际case总5次、全RED，首次工具观察器失败不算case；正式M6整体未套用，仅必要且有证据的局部修复逐项记录。06 trace有shutdown_settled、登记32个PID:starttime含Z残留空；DDS回归monitor0，formal case/driver1、observer0、monitor无错误无timeout。完整保存恢复/退休/R4、真车及性能仍未验。

### 已授权GetState调查结果（不算新的项目case）

- 对齐原06请求和超时：约30.0056秒等待内，原pump仍有141次raw_input、116次AMCL pose回调及持续IPC状态；不支持整个进程卡死。controller/get_state端点同时出现两条真实send_response timeout，但原日志没有请求GUID/sequence关联，不能断言其中哪条对应fixture那一笔。
- 用户随后授权调查，新增仅私有外部工具probe.py，产品/测试/配置未变，不换DDS或依赖。一次原装controller、一次真实GetState、原30秒预算、无重试：约22.20ms收到UNCONFIGURED，server debug取请求成功并发送回复；future完成且未取消，之后才销毁client。stock controller/probe/monitor均0，登记2个PID:starttime含Z残留空。
- 对照为未配置的单节点，没有原组合启动和生命周期configure/active并发，不能证明原组合稳定或回填旧轮。原controller与对照实际maps的6个公共库device/inode相同，包括rclcpp/rmw/FastDDS及原reviewed costmap core。
- 实际安装Future.done只判断FINISHED、取消为另一个CANCELLED状态，因此原done未完成不能单独排除内部取消；原源码无提前销毁/显式cancel，但当次cancelled未记录，仍N/A。精确阻塞点为回复等待，不夸大为全进程死锁或确定的VM问题。
- 私有navigation-getstate-control01/report.md、output/history-alignment.json、events.json、same-loaded-library-files.json及getstate-1795760327906303544.tar.gz/逐文件manifest记录全部事实。上游#842仍只作相符候选；剩余关键缺口是同一请求的client关联ID与server回复reader匹配决定。未追加整链运行，原项目case仍5次全RED，完整功能与真车待验。

### GetState调查暂停留档（用户决定，2026-11-27）

- 用户明确先保留日志，后续实车若有同问题再查；停止追加此项实验、整链运行和修复，不把暂停写成已解决。
- 已完成的一次独立GDB单请求对照关联到真实writer GUID/reader GUID及sequence=1，实际经过reader匹配YES、DataWriter写入成功、RMW_OK、客户端接收；约25.78ms返回unconfigured。调试器会扰动时序，不能回填原06轮或签稳定性。
- 实际已安装ELF核对将原警告定位到reply-reader未匹配的TIMEOUT分支、DataWriter写入之前；原失败的请求ID仍缺失，触发匹配失败的具体责任未知，不能直接认定项目代码无问题。
- 新对照驱动/三个GDB/monitor均0，登记7个PID:starttime最终含Z残留空；非ROS smoke错误及所有旧失败保留。日志索引在会话共享目录`navigation-getstate-witness02/report.md`；成功原始归档`output/witness-1795762286534830311.tar.gz`，未启动服务的失败smoke另档。此次留档仅改文档，产品/测试/配置未变。
- 计划整体仍未完整验收：原整链5次RED，保存恢复/退休/R4及真车未验；此次用户决定暂缓GetState，不自动改为完整通过。
