# 第9项全系统实时控制时钟审计与改造建议（待负责人亲审）

审计日期：2026-11-16。HEAD：`593e8f13f385892f0bd62cf75c593f7b07efd2da`，分支`fix/degraded-drive-speed`。本轮按用户扩展要求停止新增系统范围实现，先交实际消费者清单和端到端契约。已授权的第8项与原第9项导航producer补丁保留，其335项定向通过不代表本报告新增范围已完成。

## 2026-11-16 后续亲审修订

负责人已认可三域职责和必要MPPI timer补丁。硬件没有采集tick时采用最早可观测驱动读包的host CLOCK_MONOTONIC，标明设备前缓冲未知，不新增全车启动资格门；总项目厂家源码已找到，具体来源、43文件散列、缓冲调查及最小补丁触点见`2026-11-16-control-clock-producer-contract.md`和`2026-11-16-control-clock-producer-sources.json`。下表旧的“30秒预算保留”意见撤销：最新动作是3秒输出，旧15cm精确距离、偏航/反向学习中断和30秒预算由学习worker删除，本任务只拥有时钟契约与时间迁移hunk。

## 主要结论

第9项不能通过只改StartupLearning的dt完成。当前实时链存在四种独立耦合：

1. `SensorTiming.age()`以及C++同版策略以`max(now_ros-stamp, initial_wall_age+steady_elapsed)`取年龄，向前校时会使仍新鲜证据提前过期，向后校时因`now_ros < received_ros`直接无效。
2. 原始接收器、SafetyGate、NativeObservations、MotionEstimation、视觉ledger和SLAM桥会因接收侧ROS倒退撤销来源、增加代次或永久关闭，最终造成学习中断/控制失格；这不是实际源重启。
3. 学习、相对位姿、运动估计的加速度/对齐、GPS航向和扫描/RGBD速率仍使用ROS或裸消息时差。校时可能产生负dt、夸大dt或丢弃正常后续样本。
4. 传感器原始producer缺少可信采集steady前沿。现在`RawCaptureHeader.received_steady`是**首次本仓接收回调时间**，不是硬件采集时间；wheel/IMU同样只有DDS wall时间和原生回调steady时间。不能删除初始年龄后把首次回调当产生，或通过重试/重新发布续命。

`/tmp/control-clock-audit-hits.txt`的358条命中只是索引，其中包含测试、数据字段及正确的外部ROS时间。补充检索覆盖Python/ROS/C++、视觉/定位/SLAM、任务/云端/网页；本报告按实际消费者分类，不把命中条数当缺陷数。外部驱动及Nav2全依赖源码不在本仓，不能宣称完成其实现/部署核验。

## 按模块的实际消费者与处理意见

下表路径以仓库为根；`motion/`指`src/senior_control/motion/`，`vision/`指`src/senior_control/vision/`，`robot/`指`ros2/src/senior_robot/senior_robot/`，`ingress/`指`ros2/src/senior_sensor_ingress/`。需改表示**建议待审**，不是已改。

| 实际消费者／位置 | 目前用途与影响 | 建议 |
| --- | --- | --- |
| `src/senior_control/sensors/policy.py:43–82` | 统一年龄、失效和deadline同时依赖现在ROS及首次wall差；向前/向后校时都可使有效观测失效 | **需改**：统一消费可信measurement steady/原始deadline；仅以当前steady递增年龄，不重新算wall差，不新增时限 |
| `ros2/src/senior_motion_msgs/include/senior_motion_msgs/sensor_timing.hpp:31–48` | C++策略与Python相同耦合 | **需同版改**，与Python用同一契约及既有soft/hard配置；不可仅改单端 |
| `RawCaptureHeader.msg`、`BoundedHeader.msg`、`ScanMotion.msg`及嵌套Depth/Semantic/RGBD消息 | 保存原始ROS stamp、首次回调双时钟、源epoch/序号；缺少采集steady与主机boot域，ScanMotion保留两端capture但区间仍ROS | **需改同版schema**：新增明确采集steady区间/时钟域来源，完整透传；ROS字段保留作TF/配对标签 |
| `robot/native_receipt.py`、`senior_motion_msgs/src/native_receipt.cpp` | 原生回调先捕获CLOCK_MONOTONIC，经IPC不重盖；传输TTL已steady，但不能证明回调前原始测量年龄 | **保留**既有私有IPC、GID、receipt和稳态期限；Python文件属于独立冻结改动，不改。新增producer时间应在消息内传递；如需注册新的有界类型可单独改C++类型表，不能借机改诊断实现 |
| `ingress/src/raw_receipt.cpp:228–249`及`raw_receipt.hpp` | `clock_ok`因接收ROS倒退invalidate_all、换全部源epoch | **需改**：接收系统校时不撤销实际writer/source epoch；仅真实boot/GID/producer_epoch/steady异常触发已有隔离机制 |
| `ingress/src/scan_receipt.cpp:141–158`及`scan_receipt.hpp` | ROS倒退使raw scan换代并清空；回调生成sequence | **需改**：源顺序由producer可信sequence/capture steady维持；接收序号只能证明回调顺序，不冒充生产顺序 |
| `ingress/include/.../{capture_ledger,scan_ledger,rgbd_candidate_ledger}.hpp`及`src/rgbd_candidate_ledger.cpp` | 对原始/派生样本去重、exact-source闭包和first-receipt范围 | **需检查并迁移时间字段消费者**；保留GID、generation、sequence、内容一致性、容量及真实旧代次拒绝 |
| `ingress/src/rgbd_candidate_receipt.cpp:141–152,366,480–493`及header | 双时钟倒退使ledger永久failed；最终ROS年龄再次判断 | **需改**控制资格为steady；TF stamp一致性保留在独立测量域，结果用所有原始capture deadline最小值 |
| `ingress/src/static_tf_receipt.cpp:152–164`及header | ROS倒退撤销静态外参、ROS必须继续走才clock_ready；新外参重放绑定ROS frontiers | **需改**静态外参生命周期与source/boot/steady绑定；**保留**tf edge原始ROS stamp、静态内容与拓扑身份，不能把静态变换日期转换成测量dt |
| `robot/safety_node.py:271–289,526–550` | `_clock_ok`因为ROS倒退清空scan/depth、退休fence；scan年龄/顺序还依赖ROS，clock_advance_steady间接绑ROS进度 | **需改**：去除系统校时到资格/来源换代链，按新capture steady和源顺序接收；原始期限不得用当前回调重盖 |
| `src/senior_control/safety/freshness.py` | SafetyGate仍实际创建；`usable()`要求ROS年龄与单调回调年龄同时通过 | **需改或合并到统一策略**，删除无用途双时钟前沿，保留真正断流/单调异常停车 |
| `robot/raw_safety_observation.py`、`safety_scan_ingress.py`、`scan_adapter.py` | raw/filtered scan之间传递stamp和首次receipt；scan_adapter按ROS年龄拒绝并按原始ROS stamp查TF | **需改**年龄/receipt契约，**保留**TF查找的ROS stamp；scan_adapter第7项删除hunk完整保留，自己的时间增量单列 |
| `ros2/src/senior_scan_matcher/src/scan_motion_node.cpp:235–303` | 入站/最终年龄依赖ROS、last_stamp前沿拒绝；区间dt和最大速度检查用ROS差 | **需改**：配对两端可信采集steady计算运动区间/速度，并保留两端原始ROS stamp用于TF；源身份/模型、几何与算法不变 |
| `robot/motion_observation_adapter.py:143–171,326–363,501–598` | receiver now_ros倒退增加`_clock_epoch`并清窗口/pose；原始wheel/IMU从标准消息和DDS receipt组成Origin，laser沿两端capture回传 | **需改**：去除接收ROS换代；沿完整源identity、生产顺序和新steady测量区间归一化，不创建新鲜度 |
| `motion/estimator.py:19–99,250–258,276–338,480–507` | roots结构把received_ros>=stamp当资格；measured_ros作为正样本前沿、加速度dt/外推；now_ros倒退generation++ | **需改**为measurement steady；计算dt、外推、顺序、有效期只用可信同域steady；generation只因真实源/模型/单调时钟变化，不因校时 |
| `robot/motion_sensor_runtime.py:62–166,277–300,353–391` | raw interval ROS差→rate、统一估计、姿态、原始wheel/IMU静止、GPS course及反馈期限 | **需改**适配新Origin/measurement区间，传递steady；保留各分量来源、独立证据标记、原始deadline与既有静止政策 |
| `motion/pose_estimator.py:314–357,367–505` | root ROS正前沿；相对路径、段误差和预测dt用computed_ros；由上游generation变化清历史 | **需改**内部路径积分/查询轴为steady；对外PoseObservation原始ROS标签与TF/map frame保留，按明确测量域映射查询，跨未知域不硬拼 |
| `motion/startup_learning.py:125–138,190–216,235` | previous_ros/computed_ros积分、validation_after_ros门、pose_history及scan匹配用ROS；上游generation变化中断 | **需改**：用原始measurement/computed steady定义dt/验证前沿/历史索引。3秒输出窗口及删除旧15cm/偏航反向中断/30秒预算由学习worker负责，本任务不保留已撤销规则；第7项删除hunk完整保留 |
| `motion/learning_pose_history.py:15–63` | 按stamp_ros排序/插值，回拨后新点不入历史；学习查询依赖同stamp | **需改**单调历史；保留禁止外推与跨真实generation插值，不能用tick时间替代旧capture |
| `src/senior_control/safety/self_learning.py` | 模型`frame_stamps`为传入scan.stamp的不透明有序去重键，不主动读系统时间 | **冻结不改**。上层需在学习适配边界提供独立的steady时间视图/帧identity，保留原始ROS TF字段；不能仅改pose_history而留下ROS frame索引。此接口细节须与下节契约一并亲审 |
| `robot/motion_core_output.py:614–632,694–829,1130` | 多处统一age按now_ros，pose查询在ros轴，学习接收的scan stamp也ROS；控制tick/原始命令有效期本身steady | **需改前半感知/学习时间适配**；**保留**控制dt、deadline、停车和第7项已有产品增量，禁止借机改变回波容差/误差模型 |
| `src/senior_control/safety/space_estimator.py:103–110,150` | 选最新root及同源stamp前沿、年龄；向前跳错选新鲜度、向后拒正常样本 | **需改**measurement steady +可靠源sequence；保留空间一致性、来源独立性和占用几何 |
| `motion/observations.py`、`sample_types.py`、`admission_events.py`、`source_admission.py:66–105` | MotionFusion可走的本地owner路径还用ROS年龄/顺序/暂停与回拨退休epoch，不能忽略成纯死测试 | **需迁移实际时间契约**；确认所有生产组合调用后删除确实退役的链及相应测试，不能留一条旧wall回退路径 |
| `motion/temporal.py`、`fusion.py`、`source_fences.py:56–78` | 对齐窗口/缓存裁剪/未来样本等待用ROS轴；deferred receipt保留steady但等待结果仍依赖now_ros赶上 | **需改**对齐为同域测量steady，等待与覆盖期限steady；保留ZOH/min-gap/原始hold配置、有限窗口和真实epoch fences |
| `src/senior_control/localization/gps_course.py:48–71,106–125`、`robot/optional_motion_inputs.py` | GPS与轮/IMU配对、航向积分/预测ROS时差、simulation ROS回拨换代；外参TF查询ROS | **需改**积分/年龄/顺序，**保留**GPS测量时域及TF stamp，并要求硬件时钟域归一化依据 |
| `vision/raw_capture.py`、`runtime_evidence.py`、`egress_receipt.py` | raw admission和depth/scan最终接收用ROS前沿和ROS年龄；DepthAdmission回拨退休 | **需改**可信capture steady与固定deadline；GID/闭包/first receipt/容量保持 |
| `vision/depth_runtime.py`、`depth.py`、`rgbd_joiner.py`、`synchronizer.py`、`rgbd_frame_builder.py` | exact-stamp/最近帧配对、顺序和等待；部分裸stamp同时作TTL/顺序 | **分离**测量域配对键与steady等待/资格。原始像素帧及CameraInfo必须同采集域/epoch；裸ROS数值相等不够证明跨校时同帧 |
| `vision/semantic_runtime.py`、`semantic_coordinator.py`、`depth_ingress.py`、`semantic_ingress.py` | 原始证据与检测异步结果TTL/TF/帧配对，派生结果不得比任一root更新 | **需改**年龄/等待/撤销，保留同帧身份和TF外部域一致性；检测算法与阈值不变 |
| `vision/independent_motion.py:307–334,369–380,490–499`、`rgbd_body_motion.py` | 双wall/steady年龄、同帧序前沿、两RGB stamp差作为body motion dt | **需改**采集steady区间及同域sequence，保留图像/深度几何/模型，禁止利用处理完成时刻算速度 |
| `vision/rgbd_parent_ledger.py`、`rgbd_coordinator.py`、`rgbd_producer_engine.py`、`rgbd_spawn_worker.py`、`robot/rgbd_candidate_producer.py:257–263` | ROS倒退失败/重启、fresh_after_ros、worker双deadline，重启补发/重新取结果易刷新时间 | **需改**工作截止/源前沿为steady与真实boot/run；结果保持所有原始root，重启不续命，不把系统校时当worker故障 |
| `vision/static_tf_binding.py`、`static_tf_wire.py` | 外参有效期、挂起/重确认、ROS admission frontiers和source重放 | **需改**本地生命周期前沿为真实source+steady；**保留**外部TF树/原始stamp和静态edge语义，不以系统校时重建外参 |
| `ros2/src/senior_slam_bridge/src/{bounded_scan_ingress.cpp,static_lidar_tf_ingress.cpp,scan_receipt_admission.hpp,acquire_scan_receipt.cpp}`及配套headers | ROS倒退导致bounded scan永久closed/static lidar退休；fresh以ROS年龄；acquire等待本身steady | **需改**本仓采集资格/生命周期/TTL；**保留**SLAM消费的外部ROS stamped LaserScan和TF查询，外部SLAM算法不重写 |
| `robot/hardware_reference.py:135–181` | AMCL/map身份用于导航持续qualified；pose的ROS回拨清空、ROS年龄/正前沿可能取消导航来源 | **需改**pose新鲜度为可信采集steady/sequence；map/GID/地图serial及外部map frame保持；AMCL无生产mono同样是上游契约缺口 |
| `robot/simulation_reference.py`、`simulation_camera.py` | `/clock`、truth、scan顺序、future等待、pose合资格及相机文件年龄有ROS时间逻辑 | **需分离**仿真测量域与控制时基；显式simulation world重置应由源/world epoch表征，不能把NTP或接收ROS倒退等同源重启；视频展示年龄不提升为车辆运动门 |
| `tools/nav2_overlay/navigation-monotonic.patch` | 已实现controller生产前沿/零输出/失败容忍/进度超时/等待steady | **保留已授权实现**，其证据独立；新增全系统未完成前不宣称第9项结束 |
| 官方Nav2 `nav2_mppi_controller/src/controller.cpp:33–34,81–84`（同一固定commit） | `clock_ = node->get_clock()`用于reset_period，系统时钟前进触发optimizer reset，后退可推迟reset | **需追加最小时间补丁建议**：新增独立steady reset clock，保留path_handler/optimizer返回消息ROS stamp；不变更MPPI算法、model_dt或代价配置。仅此依赖包的必要文件，需负责人认可扩展原controller-only构建范围 |
| 官方Nav2 `nav2_mppi_controller/src/optimizer.cpp:63,134` | `model_dt`是配置的预测步长；返回TwistStamped使用plan.header.stamp | **保留**模型参数与外部路径时间，不机械换为当前mono；真正producer时间由已改controller入口捕获 |
| `motion/cadence.py`、`controller.py`、`trajectory.py`、`constraint_worker.py`、`constraints.py`、`stopping_input.py`、`state_machine.py`、`stillness.py` | tick实际dt、permit/evidence期限、恢复/停车监督、调度预算均以注入steady/CLOCK_MONOTONIC运行；数值模型不自行读wall | **保留主体**，只迁移上游root时间表示，增加校时不触发generation/停车的跨链验证；不增加误差、阈值、固定门 |
| `motion/manual_session.py`、`manual_authority.py`、`agent/manual_control.py`、`safety/arbiter.py`、`runtime/ipc.py` | 权限/断流/队列deadline使用单调时间；网络drive.intent在Agent/cloud已排除wall expires准入 | **保留**手动期限与连接epoch/gesture/sequence；不得拿UTC传输字段重新限制已接收实时控制 |
| `senior_motion_msgs/src/{final_command_guard.cpp,motion_output_guard.cpp}`、`robot/motion_output.py` | 最终命令/执行兜底用CLOCK_MONOTONIC和固定原始deadline | **保留**；增加全链校时测试，真正断流仍输出停车/收束，不移除deadline |
| `robot/agent_adapter.py`、`robot/motion_lifecycle.py`、`runtime/*` diagnostics/worker/IPC | 异步超时、执行器预算、快照交接均monotonic/asyncio loop；native receipt wall字段仅元数据 | **保留**已分离调度；学习清理worker对supervise的小块独立，不修改/纳入本任务 |
| `robot/navigation_adapter.py:579`、`src/senior_control/navigation.py`、`tasks/{runner,runtime,executor,navigation_port}.py` | PoseStamped目标/导航计划是ROS外部语义；任务日历UTC、执行/取消等待为asyncio mono；runner的clock_jump只记skip区间，不因该分支暂停已运行任务 | **保留日期/计划语义**，增加运行中的任务不因校时撤销的跨层验证；本地HardwareReference失格的间接取消已列需改 |
| `mapping/{evidence,occupancy,references,store,operations,archive}.py` | 地图版本/历史/文件日期及历史测量stamp，确认跨度按传入observation.stamp；操作超时已asyncio | **保留外部地图/归档标签**；任何用于活体证据确认跨度的输入需由同域measurement steady提供，不把持久化日期当控制dt。没有证据表明修改地图几何/历史格式为本任务必要，不猜改 |
| `agent/{websocket,main}.py`心跳、`cloud/relay.py`心跳失效耦合 | heartbeat有wall freshness/固定窗口，可能经离线影响新请求；第10项已独立排队 | **记录交叉风险但不混改**：第9项确保已接受控制/导航不因墙钟直接失格，第10项的心跳与停车解耦交其独立任务 |
| `agent/snapshots.py:224–250` | SnapshotCache以wall算展示stale；为云端只读副本，不直接车端控制 | **保留本任务范围外展示语义**，记录时钟倒退可延迟展示stale，不冒充实时控制问题；若后续统一优化应单独改展示elapsed与UTC标签 |
| `cloud/{auth,pairing,registry,relay}.py`、`settings/service.py`、`vehicle_logs/service.py`、`waypoints/store.py` | 令牌/配对/签名绝对expires、审计UTC/日志/持久化日期 | **保留系统时间**，不得将跨进程/重启持久化日期换本机monotonic；不把它们的日期校验引入已运行运动控制 |
| `web/src/api/{client,vehicleState}.ts`、`features/drive/ManualController.ts` | 浏览器发送100ms timer、RTT/展示received_at_ms用performance.now；Date.now expires是UTC协议字段，不是车端已接收意图有效期 | **保留**；RTT不作驾驶资格。PairingPanel绝对期限属授权UI，App轮询/视频/日志超时不属于车辆运动循环 |
| `tools/vehicle_motion_{capture,calibrate,analyze}.py`及replay/fixtures | 采集工具同时记录ROS/steady，calibration脚本的truth年龄有ROS；非生产自动控制入口 | **保留日志原始域**，随新消息迁移解析与控制性校准资格，未经授权不运行车辆采集/标定；不从离线记录“补造”mono证据 |

## 建议的端到端时钟契约

这是待审工程设计，不是现有消息已经提供的保证。

### 三类时间明确分开

- **控制执行域**：同车Linux `CLOCK_MONOTONIC`，标识`host_boot_id`；必要时还须明确monotonic time namespace。接收/调度/积分/年龄/有效期/恢复/停车均在此域。Python/Linux `time.monotonic()`、C++控制steady及注入时钟必须经同域契约一致；不得把macOS Python的mach_absolute_time与POSIX CLOCK_MONOTONIC相等当部署证据。
- **源测量域**：设备采集tick或仿真measurement time，带producer boot/measurement_epoch、生产sequence。真实设备重启、计时器重置或未知跨域数据需要明确隔离/新epoch；接收机NTP校时不是测量epoch变更依据。源提供可信同车capture steady，或在硬件采集点依据明确设备tick→steady适配生成，不能在下游猜初始年龄。
- **外部ROS/UTC域**：原始header.stamp、TF查找/地图/路径标签、日历和日志日期。保存原值及其epoch/配对键用于外部语义，禁止拿`now_ros - stamp_ros`重复决定控制寿命，禁止全局把ROS stamp改成mono后交给TF。

### 根证据到最终输出

1. **生产/采集边界**生成不可变`source_boot/measurement_epoch/sequence`、原始测量区间、`capture_start/end_steady`、同车clock_domain，附原始ROS stamp/外部测量域以供TF。信息必须在进入可能积压的DDS/IPC/worker队列之前建立。
2. **首次可信接收**记录`received_steady`但不改变capture时间。若生产端在设备队列中已延迟，该延迟必须已包含在capture→now年龄中；单靠收到wall stamp后冻结`received_ros-stamp_ros`在任意校时跨越下不能证明初始年龄。
3. **统一Origin/Observation**同时承载不可变根来源及measurement steady，`deadline_steady = 原始有效测量前沿 + 既有配置时限`；复合证据取所有roots的最早真实期限。没有时间域依据的样本不能被重新盖时间接入，不以旧双时钟路径回退。
4. **估计/对齐/积分**用同域measurement steady与computed steady；可靠源顺序以生产sequence/真实epoch为主。同一捕获多次消费、异步结果晚到不增加dt、不刷新根deadline。区间测量的start/end必须来自真实采集端点，不能把回调间隔冒充采样间隔。
5. **学习与空间**以capture steady查询LearningPoseHistory/PoseEstimation。上层向冻结self_learning模型提供单独的时间视图/帧键，模型几何和1cm匹配不改；原ROS TF字段保留，禁止裸改scan.stamp后让动态TF校验混域。验证前沿、帧去重、代次均随新的capture identity贯通。
6. **命令/轨迹/输出**沿现有steady deadline和20Hz绝对调度，真实采集断流过期继续停车。系统前/后校时不变更来源、motion generation、pose frame generation、学习attempt或已运行任务执行代次；真实单调异常、新boot、源重启/模型更换仍按各自既有边界处理。
7. **TF/定位**静态外参不因系统校时退休；动态TF需要匹配源测量域/外部ROS epoch的原始标签，无法匹配时报告真实缺证，不能伪造变换或用最新TF覆盖旧capture。外部AMCL与SLAM仍须提供可验证的原始采集时间；本仓桥只做透明同版适配。

## 当前上游契约缺口与须负责人决定的边界

- 功能子仓库的`config/hardware-motion.json:44–46`指定wheel/IMU producer为`wheeltec_robot`；总项目厂家Humble资料现已找到该串口驱动，以及lslidar、Astra/UVC、USB/GPS/AHRS和AMCL来源，散列与触点见producer契约报告。RawReceipt/NativeReceipt仍是DDS接收端，不能冒充这些更早的驱动读包点。
- 因此**不能先删ROS年龄再把received_steady当capture**。这是信息不足，不是需要增大误差/时限。应为实际运行的硬件/仿真生产者做最小同版capture消息/适配，像已认可Nav2做法一样固定源码版本、保留原始采集时间，列出安装与ABI前置；不自动替换车辆依赖。
- 可证明设备tick与host mono关系时采用采集层适配；无tick或映射依据时使用最早可观测驱动读包steady，显式报告设备/内核前缓冲延迟未知，不补造硬件采集时间，不以缺tick新增全车启动门。实际版本匹配与缓冲调查结果见producer契约报告；仿真实际插件源码在资料中未发现实现，已列具体来源边界。
- `RawCaptureHeader/ScanMotion/BoundedHeader`等schema是同版迁移，不保留旧wall资格分支；外部原ROS接口仅因TF/地图语义保留，不把这个双语义设计误称旧兼容链。现有已冻结`native_receipt.py`不改；若producer消息引入新类型，其原生C++订阅注册可独立增补。
- MPPI的reset timer是已证额外控制消费者，需要追加固定1.1.20 `nav2_mppi_controller`最小时间补丁；其算法/模型_dt/路径stamp不改。是否将这一必要声明依赖纳入受控overlay构建，由负责人亲审本报告后决定。
- self_learning.py冻结不改；StartupLearning、MotionCoreOutput和scan_adapter的第7项删除hunk保留。清理worker在docker/simulation supervise的小块不在本任务自有diff；学习状态持久化恢复不混入。

## 受影响文件清单与实施切分建议

**已实施并有335项导航范围证据**：command_node、dedicated_controller、navigation_overlay、navigation_clock、共用constraints、NavigationSample/schema注册与头文件导出、controller固定patch/manifest/准备脚本、navigation launch nonce及相应导航/输出/退役测试。完整自有清单及冻结指纹另见导航证据报告。

**建议新增修改清单**：以上表中标“需改”的每个实际消费者（包括学习、估计、统一年龄、raw/derived ingress、Python视觉ledger/worker、C++scan matcher/SLAM bridge、定位reference）与其被引用的时间字段类型/消息和测试。这是跨层的同版时间接口迁移，须按完整链实施，不能先孤立改dt或年龄门；外部producer文件须取得确切源码后列名，当前不假造文件路径。只作机械字段透传且无新增消费的文件由实现时据编译/引用补充清单，不能借此扩大算法政策。

建议待审通过后按依赖顺序实施：

1. 固定原始producer采集契约、时钟域与同版消息，建立Python/C++共用年龄计算；先保证最早证据前沿可追溯。
2. raw/derived/TF/SLAM ingress、视觉worker与MotionObservation完整透传，删除接收ROS校时到source retirement链；真实boot/sequence/steady异常的隔离保持。
3. 统一估计/pose/GPS/空间/学习积分与capture-time查询迁移，保留第7项几何匹配和所有原始根期限。
4. 系统时间仅留外部ROS/UTC语义；补齐MPPI reset最小timer patch和导航reference持续合资格链；不混入第10项心跳排队。
5. 最后集中迁移并执行受影响测试，按同一源码版本交实际diff、指纹和失败/修复记录；不逐模块反复全套。

## 新范围定向验证矩阵（待实施）

| 场景 | 应有结果与端到端证据 |
| --- | --- |
| steady按真实间隔前进，接收ROS向前/向后大幅校正 | 同一writer/boot/run/measurement epoch与执行代次不变；后续新样本可达最终受限输出 |
| 学习移动/制动/验证各阶段校时 | distance/yaw/SE(2)等于相同steady轨迹的未校时对照；attempt不中断；验证只取实际制动后capture，不混旧帧 |
| MotionEstimation/PoseEstimation/GPS course/扫描及RGBD差分校时 | 数值dt/预测量与对照一致，不因now_ros倒退generation++，外部TF查询仍接原ROS标签 |
| 消息在producer队列/DDS/native IPC/worker/重试各阶段积压后到达 | 最早capture时间不变，过期根拒绝；worker重启/新接收时刻不能续命，无“到达即新鲜” |
| 系统时间暂停/校正与真实源重启分别注入 | 单纯校时不退休来源；实际源boot/GID/measurement epoch更新仍隔离旧消息并拒绝重放 |
| 单调时钟非有限/倒退、不同host boot/namespace或未知测量域 | 明确缺证/按既有真实异常收束，不伪造新鲜度，不通过wall映射勉强放行 |
| 单轴/独立证据变旧、正常断流 | 各自deadline不会被其他新轴续命，学习/估计/约束不使用旧根，最终停车保持 |
| 静态TF与动态TF外部域 | 静态内容/真实source生命周期不因NTP撤销；动态TF必须在正确原始stamp/epoch可用，未知跨域不接最新TF冒充 |
| UTC任务日历/日志/地图/云认证 | 日期语义保留；已进入控制执行的任务不因接收系统时间改变撤销；心跳差异按第10项另审 |

本机无ROS/Humble/colcon，不把消息生成/目标平台ABI/实际DDS/硬件时间同步标为已验证；需要Linux同版构建与外部producer证据。本报告不修改任何新增系统范围产品代码，也不把335项导航回归当作系统范围完成。
