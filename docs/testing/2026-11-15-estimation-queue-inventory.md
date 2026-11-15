# 实际运行队列盘点与统一积压告警

只读源码基线 `07b815d`。本表的容量是源码声明/配置容量，不是实测深度；未启动服务、VM 或实车。`pending` 指已入队尚未开始处理的任务，`running/inflight` 另计；缓存、单槽和 DDS 不假装成同一种 FIFO。

## 获准实施的告警参数与接口

统一 `QueueWarningPolicy`：`warn_pending_count=32`、`warn_capacity_ratio=.75`、`repeat_s=5`、`recovery_ratio=.5`；当前使用进程级统一设置。已知任务容量 C 时 `threshold=max(1,min(32,floor(.75*C)))`，pending **大于** threshold 时告警；容量未知但 pending 可靠时 threshold=32。边界明确：C=4 在 pending=4 触发，C=16 在 13 触发，C=64 在 33 触发，C=1024 在 33 触发。告警阈值不是容量，不更改任何原队列溢出/安全优先级语义。

单槽/latest-only 容量 1，pending 最大 1，不因正常有一个待处理项就算数量积压；单独暴露覆盖次数与 `oldest_wait_ms`。采用通用 `warn_oldest_wait_ms=300` 作为可配置**诊断**等待阈值（与 sensor soft_age 独立，单位不同），单槽卡住超过它也可 WARNING，不能据此拒绝驾驶。历史缓存只报利用率/淘汰计数 DEBUG，不以“缓存已满”触发任务积压告警。

统一数据接口：`QueueSnapshot(name, kind, pending:int|None, capacity:int|None, running:int|None, oldest_wait_ms:float|None, dropped_total:int|None, overwritten_total:int|None, depth_basis)`。缺可靠数据用 null/unknown，不伪造 0。计数点在入队、出队/开始、完成、拒绝、丢弃、覆盖处，读取处不改变任务生命周期。

首次上穿即在本次队列变更后提交 WARNING；同队列每 5 秒最多一条重复/新 episode 摘要，带期间 peak、suppressed 计数。退至 `pending <= floor(threshold*.5)` 且等待也解除，可记一次 INFO 恢复；快速抖动恢复也受统一限频约束。默认每进程最多注册 128 个稳定 queue name，不把 request_id/连接 ID 拼进无界标签；多连接按固定角色聚合并报告最差活跃 owner 的状态，不输出连接身份。

必需字段：`queue, kind, pending, threshold, capacity, oldest_wait_ms, dropped_total, overwritten_total`，只有可靠值才填；可附 `running, peak, suppressed, depth_basis`。WARN 自身不能阻塞关键线程/反向占满日志队列：生产侧只更新固定状态/非阻塞有界诊断槽，由共用 reporter 汇总，warning 不递归触发自身日志积压。丢失诊断摘要要计一次 bounded suppressed，而不是重试塞日志。事件及时性是进入 reporter，不等待 5 秒 DEBUG flush。

线程 queue 的公开 `qsize()` 只能标 approximate；需要精确阈值时复用 owner lock 更新自有计数，不读取私有 `_queue/_waiters`。ThreadPoolExecutor 不读 `_work_queue`；通过 submit/start/done 包装记录。multiprocessing.Queue 在 macOS 可不支持 qsize，使用父端 job-id 发/子端开始/完成的显式计数，缺 ACK 则报告 inflight、pending=null，绝不猜。

## 采集、原生接收、感知与估计

| 稳定队列名 / 代码事实 | 类别与容量 | 目前可靠可见 | 拟补计数点 / 限制 |
| --- | --- | --- | --- |
| `dds.motion_inputs` / `dds.native_input.{index}`，`senior_motion_msgs/src/native_receipt.cpp` | DDS middleware backlog，sensor/reliable depth=4、latched=1；汇总容量未知 | QoS容量；业务 sample receipt 时间 | pending/oldest/drop 未提供可靠公共 API，填 null；采用公开 message_lost 事件如 RMW 支持，仍不能等同全部丢包；不访问 rclpy 私有队列 |
| `socket.native_receipt`，C++→Python inherited socketpair | 内核字节通道，非消息任务队列 | send failure/接收时间 | pending messages=null；socket byte capacity 不伪装消息 capacity，计成功发送与 Python接收可得的传输等待，未 ACK 不宣称精确内核深度 |
| `native_receipt.pending.{role}`，`native_receipt.py::_pending` | Python有序队列，每输入64 samples，总32MiB预算 | 在锁内 len/deque、queued_bytes；原receipt | 入队/交给回调/撤销/overflow计数，最早入队steady另存；payload bytes与任务数分开；保留有序负/正转换 |
| `raw_sensor_callbacks`，`raw_receipt.cpp/scan_receipt.cpp` | ROS callback/正在处理，不是可直接遍历的业务队列 | callback start/end、QoS配置 | 公开回调计时；DDS层pending未知；不把正在处理一帧计成等待N帧 |
| `rgbd.inbox`，`rgbd_candidate_producer.py` + `vision/rgbd_bounded_inbox.py` | 有序业务队列，C=16 | owner锁内_items长度、generation/overflow | 新增公开 snapshot及入/出队时刻，overflow清空实际丢弃数，不能只记1次丢弃；保留 barrier语义 |
| `rgbd.pending_tf.python` 与 `.native` | 等TF证明的待处理项，C=4 | Python dict / C++ pending_tf_.size | 新增入队steady、覆盖/弹出计数；这是pending，不是静态TF缓存 |
| `rgbd.raw_capture_ledger`、`scan.capture_ledger`、`depth.history` | provenance/重放核对历史缓存；Capture/ScanLedger构造参数，depth history独立 | 本模块容器容量/长度可加公开快照 | 只DEBUG缓存利用率/淘汰，不作任务积压；不把原始样本被复用当重复新任务 |
| `vision.rgb_depth_pair`，`vision/synchronizer.py` | 等配对队列，RGB/Depth默认各4 | owner deque长度 | 入队steady、配对消耗/超skew淘汰数；pair skew是算法维度，不能偷偷用作传感器年龄 gate |
| `vision.depth_info_pair`，`vision/depth_runtime.py` | 等配对，depth最多4、info最多16 | owner deque长度/失败原因 | 区分待配对动态深度与可复用静态标定，新增准确等待/淘汰计数 |
| `vision.future_depth_headers`，`runtime_evidence.py::_pending` | 等时钟赶上的待处理，C=16 | deque长度、header时间 | 入队steady和overflow实际删除数；未来时间不能变成新鲜样本 |
| `vision.detector.pending`，`vision/detector.py::InferenceWorker` | drop-oldest，默认C=1，另有busy | len/pending、busy、dropped | 在condition锁内公开状态、补enqueue时刻；latest result为缓存，不当pending |
| `vision.spawn_detector.pending` | 父端latest-only单槽，另有inflight | _pending非空、job serial、condition | 公开snapshot、准确覆盖计数与等待；保留原替换策略 |
| `vision.spawn_detector.jobs/replies` | 子进程 multiprocessing.Queue(maxsize=1) | 父端发/回job ID、结果到达 | 不调用不可用qsize；补started/completed ACK才有精确pending，否则用inflight并明确pending=null |
| `vision.spawn_detector.results` | 待消费结果deque，默认retained_capacity=4，范围1..8 | deque长度 | 结果未消费属于pending，补result enqueue时刻；retained/issued映射另属历史缓存 |
| `motion.deferred.{wheel,imu,scan}`，现 `motion_fusion.py` | future样本队列，各16 | DeferredSourceQueue内部deque、peek原始receipt | 加公开depth，等待/丢弃计数；迁移到唯一owner后不得双重统计 |
| `motion.progress`，现 ProgressQueue | 待验证进度区间，C=64 | 内部deque、peek | 加公开depth/入队；与控制反馈估计区分，不因导航progress队列满就定义某运动源无效 |
| `motion.scan_pending` / `motion.reference_pending` | 分别等对齐/未来reference单槽 | None/非None、原receipt | 补覆盖/丢弃计数；新owner取代的reference待处理槽同步删除，不能保留并行资格链 |
| `motion.admission_events`、temporal history、poses(64)、scan_events(256)、safety scans(8) | journal/历史缓存，非等待任务 | 明确maxlen/epoch/cursor | DEBUG存量与eviction；持续满缓存是正常历史保留，不WARNING假积压 |
| `motion.estimator.ingress`（拟新增） | 仅在来源回调与control owner不共线程时有业务队列 | 尚未实现，不能报已存在深度 | 优先复用已计数native receipt队列；确需新增时有界并保持负证据顺序，入/出队/覆盖统一计数，不能再叠无界队列 |

## 约束、手动/导航、IPC、任务、日志

| 队列名 / 代码事实 | 类别与容量 | 可见 / 拟补计数 |
| --- | --- | --- |
| `constraint.pending`、`constraint.running`，`motion/constraint_worker.py` | latest pending最多1，flight最多1；实际ThreadPoolExecutor worker=1 | 用scheduler自有_pending/_flight及提交时刻，不读executor私有queue；补覆盖与pending开始时刻，pending/running分开 |
| `ros.control.callbacks`，`motion_lifecycle.py::MotionExecutor` | rclpy SingleThreadedExecutor调度 | 公开handler start/end、实际取到sample；executor内部待调度数未知，pending=null；不为计数再扩展现有兼容私有hook |
| `agent.manual_intent` / legacy `.velocity`，`agent/main.py` | latest-only各1，已有received/deadline | 正确区分占位/执行任务，补覆盖计数；**手动TTL不变**，诊断不重排或延寿 |
| `agent.safety_pending` | dict最多64，安全事件优先 | len可靠，补entry enqueue时刻/merge计数；保留安全优先与真实溢出规则，不加入传感器1秒 |
| `agent.inflight` / `.stragglers` | running异步任务集，MAX_INFLIGHT=64（安全路径另有保留位），stragglers无新硬cap | 这是inflight不是待处理queue；在_spawn/start/done补任务类别/起始时间，不遍历asyncio私有ready队列 |
| `cloud.control_outbox`、`.agent_outbox`，`cloud/relay.py::Outbox` | C=CloudLimits.outbox_frames默认32，速度/快照合并，安全优先 | 公开pending已有，补enqueued_at与evict/overwrite/拒绝分项；合并更新对象时等待按新对象入队时间计算 |
| `websocket.receive`，`agent/websocket.py` max_queue=16 | 第三方库/网络待收，声明16 | 公共API不能给可靠消息深度，pending=null；在收到应用帧时记录时间，不读取库私有assembler/frames；若需精确应用pending就在现有调度入口计数，不虚造网络pending |
| `navigation.route` / `.latest_velocity` | RouteService当前一路任务/最新本地速度；任务状态cancel_pending不是FIFO | 报running路线、最新样本age；不能用completed_waypoints冒充pending队列深度；若新增等待任务需在实际接纳点计数 |
| `ipc.server.connections`，`runtime/ipc.py` | _connections集合，每连接串行handler；socket backlog未知 | connections和active_handlers可自有计数；kernel pending=null；补已解码但未开始handler的数量（当前同协程为0/1流程），不要把空闲连接当等待请求 |
| `ipc.client.lock_wait` | 一个请求inflight，asyncio.Lock等待者数量没有稳定公开API | 在request入口/得到锁/finally补自有waiting/inflight和入队时刻；不读_lock._waiters；capacity=null不改变既有协议 |
| `tasks.lifecycle/db_wait` | 持久任务状态、DB写锁与异步提交等待 | pending任务由DB状态查询且明确过滤；锁等待在获取前后计数；取消/暂停/unknown状态不是内存队列，不能用总任务数告警 |
| `logs.writer`，`vehicle_logs/service.py` queue.Queue | C=1024，已有queue_full损失计数 | qsize只approx，补owner锁下精确入/出队计数与最老时刻；日志文件数/保留记录是存储量，不是待写任务 |
| `logs.subscription_outbox` | asyncio.Queue(maxsize=4) | 公开qsize/maxsize可靠，补enqueue时刻及慢客户端丢弃计数；不递归记录自身WARNING |
| `agent.logs_requests`，`agent/logs_wire.py` | pending Task dict最多32，一个client一次 | 当前任务集包含运行，区分waiting/running；补request enqueue/start，client标签固定分组防无界metrics |
| `cloud.logs_outbox`，`cloud/logs.py` | 默认队列32，实际open peer用Queue(4) | 按实际maxsize统计，不假定32；qsize公开，补oldest时刻/queuefull计数 |
| `vision.signaling.candidates` | asyncio.Queue(VideoLimits.max_candidates) | 公共qsize，容量取实际maxsize；ICE候选不是测速来源，不套传感器时效 |

这是一份可实施的静态盘点，不宣称所有底层队列都能观测。实现时应以表中owner公开snapshot或自有计数点为准；未接入/未观测项输出 coverage 状态，不伪造0填满仪表盘。

## 年龄与耗时统计

共用 reporter 保持每流 `count,sum,max,over_soft,over_hard` 及固定大小直方图/丢弃数，默认5秒输出 `avg,max,count,over_*` 并清周期桶，计数溢出饱和。每条原始观测在入估计入口仅计一次raw age；每tick实际使用年龄另名use_age，允许同一原始观测被多次使用，但不能充当独立样本数。queue wait从enqueue/start差计算，compute从start/end算，control period从连续tick算，分别命名不混入sensor age。ROS暂停/时钟代次也有单独计数，禁止用wall-time减ROS-time。

实现测试应覆盖：容量4/16/64阈值边界、单槽正常待处理不报数量积压而卡住会报等待、突发上穿立即一次/5秒摘要、恢复一次及flap抑制、未知深度null、qsize不可用、覆盖/清队实际计数、日志告警不递归、统计字段不改变任何控制资格。


## 当前落点与可观测性边界

以上表格保留最初源码盘点及计数点设计，具体运行名以代码 QueueSnapshot/name 为准。已接入共用 reporter：native Python 有序64队列、RGB-D inbox16和 Python/C++ TF4、RGB/depth和depth/info配对、semantic三路配对4、future depth16、detector单槽/结果、legacy motion deferred16/progress64/scan单槽、constraint单槽、agent intent/velocity单槽和安全pending64、relay outbox、IPC锁等待/server handler、storage/task lane锁等待、vehicle log1024与订阅outbox、cloud/agent log传输、video ICE。入队等待使用真实 enqueue steady，原始采集 receipt 另列 sensor age；覆盖与丢弃分别计数，清空N项计N，不改变原容量或调度。

运动/位姿历史公开 retained DEBUG，其他原有 capture ledger/缓存依然是历史，未全部新增利用率计数；不将它们宣称待处理任务。active route和已提交async任务属于运行状态。DDS/executor/websocket内部/内核socket/无started ACK的进程工作队列深度未知，报 null，未增加私有API读取或伪造0；其未来精确观测需公开middleware指标或显式 started ACK。原生 DDS 按输入报告已知 QoS 容量，混合汇总容量 null。

统一配置 DiagnosticsSettings：period_s=5，queue.count=32、ratio=.75、repeat_s=5、recovery_ratio=.5、oldest_wait_ms=300。agent 从实际环境 SENIOR_DIAGNOSTICS__PERIOD_S / SENIOR_DIAGNOSTICS__QUEUE__COUNT 等读取；cloud 使用现有 CommonSettings 嵌套设置；硬件 profile.diagnostics 经 launch 注入原生参数。未改运行配置。统计/标签最多128，pending token计数上限4096，超诊断计数能力报告unknown，计数饱和而非无界增长。

队列变更只更新有界状态；现有 ROS timer 或共用 asyncio .1秒pump输出，C++由现有timer输出。首次/短暂峰值会被锁存供下一次pump，默认重复5秒；logger在队列锁外调用且防递归。等待单槽也可告警；告警不改变速度、权限或传感器资格。队列已释放的owner转unknown而非永久重复告警。
