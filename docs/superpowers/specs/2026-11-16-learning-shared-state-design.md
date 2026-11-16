# 同run学习结果保存、只读恢复与动作防重放

状态：2026-11-16 用户已批准固定结果并删除运行期撤销，产品已返修；验证结果以本轮固定参考测试报告为准，旧shared-state/recorded-path报告不改写为新行为通过。识别规则见[稳定连续表面设计](2026-11-16-self-return-matching-design.md)。

## 实际职责与生命周期

学习是safety_node内MotionCoreOutput的StartupLearning状态机，不是完成后退出的独立进程。现有实车/仿真supervise先分配run文件，创建唯一LearningService及LearningRecordOwner，再启动ROS和Agent；finally先关闭既有子进程，再关闭学习服务及记录owner。owner持稳定 `.owner.lock` inode，磁盘原子提交由其专用ProcessWorker执行；结束关闭fd，真实进程退出也由内核关闭fd。完成保存不会退出控制/监督进程。

文件同时单写沿用此唯一owner，不新增学习接管、PID存活猜测、等待超时或强杀策略。正常受管入口已有进程互斥锁；绕过入口直接启动控制节点的单实例缺口单独记录，不在学习模块补另一套互斥。完整新启动生成新run；同run消费者重建可恢复已保存结果，不新增跨run标定或重新学习功能。

| 模块 | 职责 |
| --- | --- |
| motion/startup_learning.py | 固定0.05m/s、host steady 3秒学习目标，收集扫描及生命周期；不积分位姿 |
| safety/self_learning.py | 离散连接与逐帧1cm位置稳定分类；固定参考逐点过滤 |
| motion/learning_record.py | 严格有界不可变记录、模型目录及动作预消耗事实 |
| runtime/learning_owner.py、learning_store.py | 单写owner、异步完整快照原子保存、只读模型交付及动作确认 |
| runtime/learning_service.py、learning_client.py、learning_wire.py | 有界本地请求/回复、客户端线程与本地快照；控制不等待磁盘或记录IPC |
| safety/learning_view.py | 已完成模型的纯只读过滤视图，无回写和逐帧状态 |
| runtime/learning_cleanup.py | 保留现有成功启动后一次旧文件精确清理 |

## 保存、恢复与动作

动作仅在最终选出合格非零学习输出时申请持久预消耗，持久确认前输出零；首次私有提交成功记录host steady起点，截止固定为起点加3秒，控制期限仍按既有安全输出裁剪。已消费或不确定是否输出的动作不重放，消费者重建也不再次发放。私有提交不是底盘物理执行ACK。

模型首次完成只有对应完整快照原子保存成功才向消费者发布。目录和动作事实由原schema严格检查，拒绝非有限数、重复键、超容量及未知字段。完成模型可包含按批准离散连接规则获得的车外参考点，因此记录层不再用严格车内几何否定整个模型。当前几何保存用于首次学习及matcher原有物理边界用途；正常运行不能以几何报告撤销完成结果或增加模型资格门。

完成后Attach返回保存结果，不发写会话；同run多次只读恢复不修改模型或消费新动作。学习未完成时沿用owner的同一写会话身份，不新增抢占或接管流程。完成后旧写会话也不能替换模型。owner异常退出后恢复已原子保存的完成模型；原writer_state及基于它清空模型的链路均删除；已消费动作事实继续保留。

## IPC与已删除机制

保留Linux abstract SOCK_SEQPACKET、同UID内核SO_PEERCRED校验、最多8个异步peer、客户端16项有界命令/回复队列、一次一请求和2秒传输期限，服务有界退出沿用既有资源清理。这些用于模型交付和动作防重放。

彻底删除撤销pipe、SCM_RIGHTS传fd、49字节memfd、IN_FRAME/IDLE/FAILED/CLOSED逐帧门、撤销事件容量/PIPE_BUF核验、owner drain、consumer PID/EOF撤销会话同步、永久删除集合、accepted/durable撤销前沿、撤销持久化及崩溃后模型失效。删除仅供学习积分的LearningPoseHistory；不删除其他运动估计消费者的MotionPath/原始分段积分。

正常运行过滤不向owner报告、不写文件、不等待记录线程、不改变目录或模型身份。没有兼容开关、空壳撤销接口、逐帧持久化或额外模型健康资格门。完成保存和动作预消耗仍异步原子提交，记录I/O错误局部报告，不导致主Agent主动退出。

## 验证边界

集中验证学习→保存→恢复、邻近方向和车外连续段、无效束断连、位置不稳定反例、1cm内外、遮挡/缺帧后回归、未匹配外部点保留、运行多帧无写入、完成后只读、动作不重放、host3秒与实际core链。Linux原生SO_PEERCRED/abstract SEQPACKET和spawn I/O在macOS不能声称通过；本地线程到真实owner适配测试不代替目标Linux部署或ROS/实车执行。

## 亲审后明确bug返修

参考目录上限与原扫描点预算统一为2048，保存全部稳定点而非从128个抽样点冒充完整表面。规范记录/IPC统一使用256KiB有界载荷；Complete及anchor id上限同步，客户端不再保留64KiB收包截断。空间索引仅加速完全相同的1cm逐点距离判断，不扩大过滤区域。新证据与非空core可用性未解决限制见[本轮报告](../../testing/2026-11-16-learning-surface-review.md)，旧136项报告不改写。

## 用户后续批准：启动学习可用回波政策

原先非空core因后方小块未知区无法开始的局限，按用户新授权只在启动学习阶段处理：前后各60度，独立默认true开关，遮挡达到50%阻止，false忽略该方向遮挡。以单独政策字段豁免相应未知覆盖要求，真实free/occupied数据及原风险误差不变；完成后不用于正常驾驶。当前[局部设计](2026-11-16-startup-learning-visibility.md)及[新验证报告](../../testing/2026-11-16-startup-learning-occlusion.md)记录本轮实现，不改写此前失败或Linux缺证。
