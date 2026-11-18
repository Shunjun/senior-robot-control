# 同 run 学习：连续录帧、独立计算与单写保存

状态：2026-11-18 用户确认的流式解耦与显式新校准已在原隔离分支实现，完成本机定向验证，尚未合入或部署。负责人已确认临时原位置预览，正式入口已替换预览组件与开关。负责人已将主分支源码推进至 `94f07ef` 并构建，但尚未重启，本分支尚未集成该恢复接线；已知学习模型仍为 None；本机结果不能代替 Linux/ARM 与实车运动、停车及模型验收。识别算法继续使用[稳定连续表面设计](2026-11-16-self-return-matching-design.md)。

## 职责与执行上下文

本文替代旧“Core 内 StartupLearning 收集扫描并同步重算”的接线，以及根任务中对应的恢复步骤。用户新要求是：接收连续录下原始帧，计算收到后流式串行处理，慢可停车后排空；运动和控制权从来不等 CPU。

| 角色 | 唯一责任与执行位置 |
| --- | --- |
| MotionCoreOutput / StartupLearning | 原控制进程中的轻量动作时钟与接管；0.05 m/s 目标、首次成功非零私有提交起算 3 秒、到期零输出并退休学习运动目标；保留当前感知及原纯过滤 |
| SafetyGate 接收进程 | 原准入成功后、补点前非阻塞录入每个原始帧；正常控制仍只有最新轻量快照 |
| LearningFrames 发送器 | 同进程 I/O 线程负责有界 FIFO 与私有 IPC，不执行 observe，不 Attach／Reserve 或抢写会话 |
| LearningService | 复用 supervisor asyncio 控制面：原始流接纳、顺序／窗口事实、进度和计算调度；不运行重计算 |
| 新 learning_compute 子进程 | 一个常驻 SelfReturnModel、逐帧串行 observe／连通性／候选交集／validate；不含运动出口或磁盘写权 |
| LearningRecordOwner 与原 I/O ProcessWorker | 单写事实和一次性窗口封闭；最终不可变模型原子落盘、durable 后交付 |

仅新增一个非 ROS CPU 子进程。LearningService 自身共享 supervisor 事件循环，不能把同步 observe 包进 asyncio task 当作隔离。现有 ProcessWorker 只有单 callable flight，不是有状态 FIFO；新计算 loop 复用 child_lifetime、stop_process 和共享槽原语，使用固定 batch、单 flight／ACK，parent 未获 ACK 不覆盖下一 batch。模型及历史常驻 child，不逐帧 pickle 整个 model，不使用隐藏全局模型缓存。原磁盘 worker 仍只做记录 I/O，不能被学习占用；无 Core 同步 fallback。

## 连续原始帧与内存上限

录帧入口是 `SafetyGate._on_scan` 原身份／时间／源准入通过、构造 raw 后、补点 update 前。不能从 `_publish_inputs` 或 Core latest 抽取学习历史。录入端复制有界不可变值，编码、socket 等待移到发送线程；溢出或发送失败不能 clear 正常 scan、阻塞接收／避障或造成手动／导航 fault。

每帧保留原 RawCaptureHeader／SampleTime、capture 序号／来源／stamp、ranges、角栅格、实际外参及 transform_stamp；Inf、-Inf、NaN 使用可逆标签，不合并成一个无效值，不造 hit。预测点从不进入学习或测速。另记本机 `recorded_steady_ns`，只表示接收端成功入录帧队列的事实，不覆盖原采集／接收时间。

工程容量固定为：发送 FIFO 16 帧、服务 FIFO 128 帧、单 flight 最多 4 帧；每帧最多 2048 束、私有包沿用 256 KiB 上限。它们是内存／传输界，不是采样率、识别阈值或运动资格。模型目录仍最多 2048 个真实参考点，不抽样缩减。没有无限队列、后台文件 spool 或 raw 历史磁盘恢复承诺。

私有通路使用同 run／attempt／segment、stream_id 和递增入队 index，原 capture 身份不变。FIFO 连续 ACK 与 frontier 只核对接收端已接纳记录是否完整送达；同身份同 payload 可幂等去重，不多算，不把传输序号冒充传感器序号。不能声称硬件／DDS 上游没有丢帧。动作／取消控制 RPC 与数据 FIFO 分开，不被数据积压挤掉。

完整性缺口或预算溢出为本学习 attempt 的粘性事实；不能默默丢旧帧后发布完整模型。动作仍由原时钟与真正接管／传感安全处理，到点停车；数据不完整只阻止模型发布，最终报告 failed／unconfirmed 和具体原因，不夺取普通控制权，不靠重复运动补数据。

## 三个不同的完成事实

| 事实 | 条件 | 后果 |
| --- | --- | --- |
| motion-complete | Core 首次成功私有非零提交建立 start，host 单调到 start + 3 秒，发零并退休运动目标 | 立即让出学习运动身份；不等计算或结果保存，原停车验证观察可继续 |
| capture-complete | 原三秒采集窗口、原 5 个停车验证帧均已成功录入，连续数据 frontier 无缺口，生命周期顺序已收尾 | 单写 owner 封闭一次性窗口事实；后续普通接管不取消已录完数据 |
| compute-complete / durable | worker 排空封闭窗口并完成原算法；原 I/O owner 再成功原子提交 | durable 前只显示等待／验证，不能假报 completed 或安装冻结 view |

初始 5 个预帧 ready 由真实已接纳记录的小计数给出，不以 observe 完成为前提，不增加新资格。原预消耗持久确认和成功非零提交起点不变；3 秒是原固定目标窗口，不是位移积分或物理执行 ACK。输出仍经过当前 directional_clearance／limit_velocity、原 150 ms 输出期限和 scan TTL。

窗口归属按原 `capture.time.sample_steady_ns` 与 host 窗口比较；原 stamp_ns 保留源物理顺序，不能将 ROS 标签直接混比 host 单调时钟。start／end 或停车确认消息晚到时，有限记录暂留尚未定归属帧，按原采集时间分类，不按计算到达／完成时间移动窗口。历史帧已在入口合法接纳，不能在计算时再次以当前 freshness 抛弃；行驶当前 scan TTL 完全不变。

首次提交开启原新 segment，三秒内帧交原 SelfReturnModel，原停车验证边界与 5 帧规则保留；停车后帧只确认生命周期，不撤销或重算窗口内候选。相同有序输入的 1 cm、车内连接、稳定交集、坐标及全部参考点输出保持原算法；完整流会包含旧 latest 曾漏掉的帧，不能把不同输入宣称同点数。

## 完整录完后的手动接管与跨通路顺序

用户明确的场景：三秒运动及停车验证数据已经完整录完，CPU 仍忙，此时手动接管。手动控制立即生效，已完整数据继续计算并保存；不再等 computed-ready 后向仍占学习身份的 Core 请求结果许可。Core 不向服务发送点集／批准候选，单写 owner 根据一次性封闭事实接受该 attempt 的计算结果。

封闭不能仅靠 Core 延迟看到的 progress，也不能按两条 socket 的消息到达先后判断。只用以下收尾事实，不引入新资格证明框架：

- 数据通路的连续 ACK／frontier 给出已录入前缀；帧采集时间决定窗口归属，recorded_steady_ns 决定“接管前已经录完”。
- 控制通路保留本 attempt 首次有效接管时刻（Core 接受命令的 host 时刻），按同一个有序控制 FIFO 传递；退出运动身份后仍保留这一项事实，不要求当前 mode 是 learning。
- 收尾时使用一次轻量顺序屏障：数据已够时，服务取得运动 owner 已处理至该数据录入时刻之后的生命周期事实；取消先到时，取得录帧端的有序截至 frontier，排空到该 frontier 后再判定。回执只回答时间／序号事实，不等待 CPU、不重新申请控制权、不询问当前 source 是否准许保存。
- 服务只有在数据前缀完整且生命周期顺序已确定后，才由原单写 owner 固定 capture-complete。屏障前不抢先提交，即使计算恰好已经结束。

两种真实顺序必须分别成立：

1. 第 5 验证帧已在接管前录入，但其 batch／ACK 比取消消息晚到：排空同 attempt 的截至前缀，确认完整后封闭；当前 Core 即使已 manual，后台照常计算和保存。
2. 接管时尚未录齐窗口或验证帧，后来 batch 或封闭候选先于迟到取消到服务：用接管时刻和录入事实判定不完整；不能拿接管后补录的第 5 帧倒填。未收尾前禁止发布模型。

收尾私有接口限于 `FrameAck(through_index)`、`IngressFrontier(stream_id, last_index, gap)` 和 `MotionObservedThrough(segment_id, through_steady_ns, first_takeover_ns)`；共同带原 run／attempt 身份。已有 start／end／停车边界仍是生命周期报告，不复制第二套时钟。只在收尾取这些事实：它们没有“允许运动／允许保存”字段，不按当前 source 投票，不需要每帧交换证书。若计算先完成，结果仅暂存到相同 attempt 的封闭事实确定；一旦封闭，保存不再向 Core 请求任何许可。

封闭后普通接管不是撤销数据。封闭前 manual／nav／estop 仍立即停止学习动作，窗口不足不造结果，已消费 action 不清除、不重播。已完成持久模型不因缺帧、遮挡、换控制源或后续无匹配被撤销。本文新增的用户验收场景是正常手动接管；不扩为急停复位、自动恢复旧目标或程序崩溃后的 raw 恢复承诺。

## 保存、恢复与资源退出

最终仍由原单写 owner 持稳定 `.owner.lock`，I/O worker 执行完整记录写入、fsync、原子替换及目录 fsync。durable 之前不交付模型；完成后 Attach／只读 State 返回冻结模型，不发新动作或写会话。正常控制／测速只用已有内存 view，不逐帧读文件，不等待学习文件。

动作预消耗、原 start／deadline／累计本地提交事实保持不可变约束。同 run 消费者重建不能重新发动作；计算消息、队列重试和子进程退出均没有运动重启能力。已 failed 的当前 attempt 不能原地复活。CPU 失败不能假报模型；程序关闭沿既有有界清理，未算完 raw 只在内存中，可能丢失；只承诺恢复已经成功持久化的最终记录／模型。

保留同 UID SO_PEERCRED、abstract SOCK_SEQPACKET、8 个有界 peer、16 项控制请求／回复及原传输期限。模型和记录仍拒绝非有限参考点、重复键、超量、身份不符。原撤销 pipe／memfd／证书、独立车速资格、复杂扫掠、定位积分、运行期清空模型、兼容 frame API 及同步 fallback 均不恢复。

## 不变规则、范围与验证边界

前后各 60°、两个独立布尔开关、70% 学习遮挡阈值与当前 +Inf 角覆盖语义保持；不恢复旧规格中的扫掠／未知区运动门槛。1 cm 不是 sensor_error；尺寸乘 1.10、单次 sensor_error、lag、输出 150 ms 和 scan TTL 均不改。固定参考可按原离散规则延伸到车外，不另裁剪。

采用单一有状态流式 CPU 子进程，不选停车后整个窗口才启动的 batch 替代。自动启动和网页显式校准共用一条流式实现；新校准创建独立 attempt，不复用旧已消费动作。以下公开接口、身份和持久化规则覆盖旧“同 run 只可能有一次学习”的接线假定；单个 attempt 的动作仍不可重播。

原预报范围为产品新增 1／修改 21（22），测试新增 1／修改 20（21），文档新增 1／修改 1（2），最多 45 项目文件；根 CURRENT_TASK 由负责人另行维护 1 文件，总预报 46。具体文件与只读测试候选见[实施计划](../plans/2026-11-18-startup-learning-isolation.md)。不改配置／构建／工具，不删除文件。

新架构目前仅完成本机逻辑与网页编译/定向测试，尚无实车验收。54 帧 JSON 可核对候选输入和采集时刻，不代表原 Core 实际选帧；Linux IPC／spawn 与实际模型仍需负责人按阶段核对。不能用局部测试、completed 但 0 点、候选点数或模拟提交替代真实运动／停车和模型点分布证据，不承诺端到端 30 ms。


## 网页新校准：公开 API 与接纳

公开命令确定为 `learning.start`、`learning.status`，沿原 v1 Envelope；robot_id／session_id／connection_epoch／request_id／seq／expires_at 保留，controller_generation 仍只能由 relay 注入。严格 DTO 同时进入根协议和 TS；请求／回执按原 controller 上下文关联，不广播给其它控制端，不把 start 放进 drive.intent latest 合并通路。

| API | 最小 payload／回执 |
| --- | --- |
| learning.start | payload：expected_active_revision（非负整数）、expected_authority_id（当前 prepared authority 或 null；null 只代表预期 idle 且无 holder，不是通配）。request_id 使用 Envelope 的原值 |
| start 回执 | data：start_request_id、attempt_id、state=accepted 或 busy；busy 指当前在途 attempt，不分配新一轮、不排队。accepted 只证明接纳请求，运动／结果以状态为准；鉴权／版本／状态冲突仍沿原错误类型 |
| learning.status | payload：attempt_id 或 start_request_id，最多一个；都不填查询当前。查询自己的新 request_id 与被查 start_request_id 分开 |
| status 回执 | run_id、active_revision、active_model（attempt_id／model_id／point_count 或 null）、当前／被查 attempt_id、phase／reason、motion_complete／capture_complete／compute_complete／durable、can_start／不能开始的原因；不传点集或 raw 历史 |

busy 是有类型的请求处理回执，不是新动作被接受；不能把其解码成 succeeded。相同 start_request_id 与相同指纹返回原 attempt／结果，payload 或归属冲突拒绝，不借新的 active_revision 重新执行。超时／断线重连先查询原请求或当前 attempt，绝不自动再 start。

完整链路为 VehicleStateView → VehiclePort／VehicleTransport → client／TS 协议 → relay → Agent → IpcMotion → AdapterCore → CommandNode → Core。CommandNode 做真实操作者／当前来源检查，Core 在真正开始时再次原子核对同一可信上下文、原期限及停车／来源状态；检查后出现新手动／导航目标，旧请求不能等它结束后再突然发动。使用已有真实 stationary／来源，不新增独立车速资格。学习服务只分配 attempt／记录，不绕过 Core 发运动。

不抢占行驶／导航、不自动 stop 旧任务、不隐式恢复旧目标。idle 无 holder 或同操作者 prepared 且停稳可显式开始；其它来源冲突返回拒绝。busy 包括录制、计算和提交，即使已停车后台仍忙也不接受另一不同 request；完成或明确失败后才由操作者重新点击。初始启动也占同一串行 attempt 槽。

学习能力通过已有 vehicle_ui.capabilities 的 learning.v1 明确宣告；校准明细用 status 查询，不扩成高频历史报告。无能力的旧 peer 不接收未知命令。后台完成／普通接管不会重新申请运动；封闭后切到 manual 仍按上文完成保存。

## 程序身份、attempt 与 active 引用

固定 `run_id` 是本次程序运行／原学习服务地址／原采集运行身份；网页校准不调用 start_environment、不改 ROS 采集源、连接 epoch 或进程 boot。`attempt_id` 是 owner 生成的 32hex 校准身份；初始自动 attempt 由 run_id + startup 标记唯一确定，手动 attempt 由 run_id + 原 start_request_id 确定性摘要得到，保留请求内容和可信发起上下文的指纹并拒绝碰撞／不一致；普通投递 seq／expires_at 不用来制造另一个 attempt，原请求时效仍由接纳检查。因此按请求可直接查原记录，不用无限内存幂等表，也不回收旧动作再用。

最小持久化结构（新结构 version 2；不在旧 v1 run 文件中原地清 action）：

| 记录 | 必需内容 |
| --- | --- |
| ProgramManifest | schema_version=2、run_id、owner_epoch、revision、latest_attempt_id、active_revision、active_ref=null 或 {attempt_id, model_id, record_revision} |
| AttemptRecord | schema_version=2、run_id、attempt_id、trigger=startup/manual、start_request_id／请求指纹（自动为空）、原 owner_epoch／revision／committed_steady_ns、geometry／phase／reason／action／model、capture_summary |
| capture_summary | open/complete/incomplete、stream_id、连续 through_index、validation_after_steady_ns 与录入收尾时刻；仅事实摘要，无 raw 数组。原 action 已有 start／end，不能重复改写另一套期限 |

每个 attempt 独立保存完整原 action，consumed／首提交／截止不被下一轮重置；对旧 attempt 的 Attach／State 只是读取，不发新写会话。当前 candidate 与 active 模型是两个对象：旧 active 存在不能让新 attempt 被误判 completed。Core／只读观察者只从 manifest.active_ref 安装运行过滤 view，新计算始终从未补点原始流学习。

owner 单写程序 manifest 和各 attempt，不增加第二磁盘 writer。文件名由 owner 根据 ID 确定：程序 `.startup-learning-<run_id>`；attempt `.startup-learning-<run_id>.attempt-<attempt_id>`，临时提交仍同目录 `.pending`。网页不能传路径；记录仍有界、完整原子提交。当前旧 v1 已完成事实不被新校准覆盖，也不作为可再执行的空 action。

切换顺序固定：先将候选完整 AttemptRecord 原子写入并 fsync；再将 manifest.active_ref／active_revision 原子切到该已持久模型；两个持久步骤都成功后才对外发布新 active。失败／不完整保留旧 active；候选已写但引用未写不等于已生效，不假报整体 durable。成功后按版本在一次扫描／控制帧边界替换 view，不改变当前控制源、目标或重播学习运动。既有有 view 就停止 State 的客户端规则须改为小版本观察，未变不重传目录，只在新 active durable 后安装一次；不逐帧读文件。

cleanup 按程序组识别生成文件，保留当前 run 的 manifest、active、candidate 及该 run 旧 attempt 动作事实，不在每次校准清除上一模型；旧程序组只在原成功启动清理边界、既有 peer 锁约束下清理。扩展精确白名单，不递归、不跟链接、不碰非生成文件；没有 raw spool，也不承诺崩溃后重建未算完数据。

## 原位置交互提案与临时预览边界

在现有 VehicleStateView 卡片内，速度／连接信息之后、任务区之前增加“固定回波校准”区，复用本组件 CSS module；任务与手动控制不被遮罩或禁用。显示生效模型及参考点数，单个“重新校准”按钮旁常驻文案：“停稳且无任务时，将以 0.05 m/s 前进 3 秒并停车；可随时手动接管。”不另加隐式 acquire／取消导航／重启步骤。

状态用实际事实区分：等待接纳、录制／前进、已停车验证、已录齐后台计算、保存中、已更新或未完成；busy 只禁用新校准按钮。旧模型仍生效要明确显示；0 点显示真实参考点数，不说识别成功。超时显示“结果未确认”及“查询本次结果”，不能用另一个 start 充当查询；无能力／离线保留最后状态并禁用 start。

负责人安排后才制作临时交互预览：新增 `web/src/features/devices/CalibrationPreview.tsx`，仅 mock 场景／本地交互；临时修改 `VehicleStateView.tsx`，在原卡片位置、仅现有 demo + 显式 preview 开关下挂入。共临时 2 文件（新增 1／修改 1），不改 App／路由／依赖／配置／服务；preview 不接真实 API、socket 或车端。使用既有 demo 壳的模拟标记，场景包括 busy、录齐后手动接管、未录齐中断、旧模型保留、持久化成功切换和超时查询，不自动控制车辆。

此时工作区会临时多 1 文件，不计最终 45 文件；预览确认后删除临时组件与开关，最终真实 UI 仍落回原已预报 VehicleStateView。是否运行现有网页预览由负责人安排，本轮不启服务、不创建预览文件。

## 本阶段接线与验证边界

- `FrameBatch` 的原 `stream_id/index/after_index`（空心跳也保留原索引前沿）、`through_steady_ns/gap` 和回复 `through_index` 分别承载上述录帧 frontier 与 ACK；发送器仅在此前前缀已出本地 FIFO 后报告通过时刻。Core 的 `MotionObservedThrough` 携带首次接管时刻及已有短触发原因，服务比较这些已发生事实，不要求当前 source 许可，也不等待 CPU。
- State 的缓存键为 `record_attempt_id + record_revision`，active 使用独立 revision；不同 attempt 的相同小 revision 不能遮蔽新轮状态。完成模型只传一次，不同时重复放入 current 与 active 两个字段；未改目录/包容量。
- 收尾保留原语义：提前撤回已开始动作时不重启前进，原三秒 deadline 不改变；停车验证采集需同时在停车确认之后及原窗口末尾之后。数据封闭后，模型计算与持久化不再依赖 Core 是否仍处于学习身份。
- 实际范围比预报少：`manual_control.py` 与 `test_vehicle_state.py` 无需修改；能力由实际 IpcMotion 私有版本探测后在 Agent 状态中宣告。项目产品 21、测试/夹具 20、原两文档，共 43 文件；根任务由负责人维护另计。临时预览文件已删除，最终基线中不留该文件。
- 保留原 1 cm/连接/冻结算法及时间身份回归；退役同步 `StartupLearning.frame`、Core 几何 worker/扫掠资格及文件 mtime 过滤缓存断言。CPU 慢、两种收尾交叉顺序、真实 FIFO 溢出、显式请求幂等/busy、durable 前保旧和网页超时查原请求，均在各自直接边界验证。
- 未启动开发服务、未 SSH/控车、未执行物理校准；Linux abstract SEQPACKET/SO_PEERCRED 与实际 spawn 服务用例保留但本机跳过。接收与控制时序、ARM 负载、真实原始流及最终模型点数/分布，仍由负责人部署后验证，不承诺 30 ms。

停车验证仍使用原 wheel stationary 事实；服务只暂存有界的轻量状态变化以匹配原始录入时刻，不引入独立速度资格。状态变化与取消事实通过原单个有序控制 FIFO 传递，观察事实最多占 12 项，为动作/phase 留 4 项；记录丢失只影响模型完整性，不停止已预留的三秒输出。Root 新恢复接线需要正常合并；恢复时禁止自动学习与恢复后显式网页新 attempt 的接纳边界待合并后共同收敛。


### launch 恢复与新的显式校准

恢复 Core 永久保留 `_launch_recovered` 历史、boot echo 和急停 revision，初始仅 State 观察；不将旧 startup 或已消费 manual attempt adopt 为可执行动作。因果交接完成后，仅一次异步通报旧 `attempt_id / owner_epoch / motion_owner_id / motion_generation` 失活，附新 output boot。motion_owner_id 是 owner 创建 attempt 时固定的运动分配身份，不是新的运动资格；没有 raw 或写 session 也能准确结束旧未完成尝试，action/consumed 原值保留。迟到通知不匹配当前 attempt/代次则无操作，不依赖墙钟或“最新记录”猜身份。

失活处理先等同一服务此前已接纳连接中的 raw/收尾事实处理完，再按原 frontier 规则确认录齐；已封闭继续 CPU/持久化，不因 seal 尚在另一通路误丢。接收进程的新 stream 不覆盖旧 stream；未接纳或进程丢失的内存历史不承诺恢复。此等待只在异步服务，最多原有 8 连接和原有 2 秒单包收发超时，不阻塞 Core，也不等 CPU。

新显式 BeginAttempt 额外携带可信 CommandNode 所见 `output_boot_id`。Core 在交接、原 T2 owner/generation、原 deadline、停稳/无冲突复核后，才异步 scoped Attach(`attempt_id, output_boot_id, generation`) 绑定写 session，再确认 AcceptAttempt；服务在确认前拒绝 Reserve。仅观察到 record 不绑定、不执行。不允许的新请求以本 attempt/output boot 的 DeclineAttempt 结束，不为拒绝而 Attach。原已消费动作不重放，已失败 run 不原地复活；当前控制来源与后台模型保存相互独立。
