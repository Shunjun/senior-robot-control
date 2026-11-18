# 同 run 学习：连续录帧、独立计算与单写保存

状态：2026-11-18 用户已确认流式解耦方向；本规格更新待负责人亲阅，产品尚未实现。当前实车仍部署 `61555f8`、本次模型为 None；`5b65286` 仅保存阶段成果，未合入／部署，不作为最终方案，也不为它追加测试。识别算法继续使用[稳定连续表面设计](2026-11-16-self-return-matching-design.md)，不把历史报告改写成新架构验收。

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

封闭后普通接管不是撤销数据。封闭前 manual／nav／estop 仍立即停止学习动作，窗口不足不造结果，已消费 action 不清除、不重播。已完成持久模型不因缺帧、遮挡、换控制源或后续无匹配被撤销。本文新增的用户验收场景是正常手动接管；不扩为急停复位、自动恢复旧目标或程序崩溃后的 raw 恢复承诺。

## 保存、恢复与资源退出

最终仍由原单写 owner 持稳定 `.owner.lock`，I/O worker 执行完整记录写入、fsync、原子替换及目录 fsync。durable 之前不交付模型；完成后 Attach／只读 State 返回冻结模型，不发新动作或写会话。正常控制／测速只用已有内存 view，不逐帧读文件，不等待学习文件。

动作预消耗、原 start／deadline／累计本地提交事实保持不可变约束。同 run 消费者重建不能重新发动作；计算消息、队列重试和子进程退出均没有运动重启能力。已 failed 的当前 attempt 不能原地复活。CPU 失败不能假报模型；程序关闭沿既有有界清理，未算完 raw 只在内存中，可能丢失；只承诺恢复已经成功持久化的最终记录／模型。

保留同 UID SO_PEERCRED、abstract SOCK_SEQPACKET、8 个有界 peer、16 项控制请求／回复及原传输期限。模型和记录仍拒绝非有限参考点、重复键、超量、身份不符。原撤销 pipe／memfd／证书、独立车速资格、复杂扫掠、定位积分、运行期清空模型、兼容 frame API 及同步 fallback 均不恢复。

## 不变规则、范围与验证边界

前后各 60°、两个独立布尔开关、70% 学习遮挡阈值与当前 +Inf 角覆盖语义保持；不恢复旧规格中的扫掠／未知区运动门槛。1 cm 不是 sensor_error；尺寸乘 1.10、单次 sensor_error、lag、输出 150 ms 和 scan TTL 均不改。固定参考可按原离散规则延伸到车外，不另裁剪。

采用单一有状态流式 CPU 子进程，不选停车后整个窗口才启动的 batch 替代。本阶段针对原自动启动 attempt；用户新要求的网页手动新校准入口正在只读收口，不能把它接成复用已消费动作，也不在本阶段擅定公开命令、并发或旧模型切换策略。

项目完整范围为产品新增 1／修改 7，测试新增 1／修改 10，文档新增 1／修改 1，共 21 文件；根 CURRENT_TASK 由负责人另行维护。具体文件与只读测试候选见[实施计划](../plans/2026-11-18-startup-learning-isolation.md)。不改配置／构建／工具，不删除文件。

新架构当前尚无验收。54 帧 JSON 可核对候选输入和采集时刻，不代表原 Core 实际选帧；Linux IPC／spawn 与实际模型仍需负责人按阶段核对。不能用局部测试、completed 但 0 点、候选点数或模拟提交替代真实运动／停车和模型点分布证据，不承诺端到端 30 ms。
