# 学习证据双时间域适配建议

历史建议；其后Nav已冻结MeasurementTime/MeasurementInterval、独立measurement_epoch及显式tick接口，学习产品适配见`docs/testing/2026-11-16-learning-dual-clock.md`。

该轮任务`task_16b740fce514 / ctx_c9b6d5235913`仅核查接口并形成建议，未修改产品/测试，未重复跑旧套。先前122 passed / 2 skipped不覆盖新的sim物理tick契约。分支`fix/degraded-drive-speed`，核查HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`；工作区有此前学习与并行Nav的未提交改动，不提交/推送、不启动服务。

## 现状与具体问题

当前`SourceTime`只有host capture start/end_steady与time_basis标签；标签sim_step没有承载真正物理tick。`StartupLearning.advance()`用`estimate.computed_steady`差分积分并把它写入LearningPoseHistory，`frame()`用capture.end_steady查询位姿，SelfReturnModel用capture.end_steady排序与保留。该视图在暂停/非1x仿真下不能作为物理积分/对齐依据；不能把ROS stamp临时当隐式例外，也不能把simtick写入名为computed_steady或measurement_steady的host字段。

## 建议冻结的最小接口

1. **SourceTime显式物理证据**：沿负责人选择增加`measurement_tick_ns`，`time_basis=sim_step`时要求显式非负整数，零合法，原`source_epoch`就是该仿真世界的物理epoch。host `clock_domain/capture_start/end_steady_ns`继续保留真实捕获/驱动前沿，不从simtick反算，不在接收处重盖。若派生差分有两个物理端点，两端点均须来自原始proof，不能按host时间比例推算起点。
2. **共用只读测量坐标视图**：由Nav在既有time_proof层提供直接投影，供estimate/pose/history/model复用，不建立第二映射。建议语义为`(basis, physical_domain_or_epoch, tick_ns)`：sim_step使用原world source_epoch与measurement_tick_ns；硬件driver_read沿同boot CLOCK_MONOTONIC实际视图，不凭空要求硬件tick，也不因不同producer的source_epoch不同就拒绝同host硬件对齐。`device_tick`只有已有可证明跨源对齐时才参与同域查询，不能比较两个无映射设备的整数。
3. **MotionEstimate另带真实测量坐标/区间**：computed_steady仍是本次host控制计算入口。学习消费的是已对齐独立运动估计的物理端点/区间，而不是computed_steady，也不从不同roots取max/min猜一个端点。各axis无法对齐时提供未确认/无可积分区间，保留原根host期限；不能用目标速度积分填补缺口。
4. **LearningPose/History显式查询域**：样本带该测量坐标和必要host标签，`at()`接受相同typed坐标；只在同物理域/epoch及同连续证据段内插值，禁止跨world重置或未知间隔。sim暂停同tick不算倒退、dt=0不积位移；相同tick的多观测不能通过覆盖旧pose掩盖矛盾，按现有估计对齐结果保留/合并保守支持，无法证明则暂不可对齐。真实tick倒退且epoch未变是异常/证据分段，不是系统校时。
5. **SelfReturnModel双视图**：`frame_keys`继续按原capture身份去重；帧几何排序/模型保留采用真实测量坐标、以sequence作同tick稳定次序。host前沿另用于原始年龄和已撤回/换段前输入排除，不把host捕获延迟冒充物理运动。世界epoch变化开新证据段，不把上一段与新tick拼接；相同tick的新sequence不虚构位移，也不擅自增加更苛刻的最小tick跨度/样本数门。

`frame()`应先用host capture验证当前run、原始时效、学习开始/segment host下界及验证后采集门，再用物理测量坐标查询真实pose。两个条件不同，不能用一个字段同时替代。`validation_after_steady`仍表示完成停车确认的host边界，名称/语义保持清楚；若另需物理frontier则明确另列，不把host阈值与simtick直接比较。

当前`advance()`把`dt >= hard_age_s`当积分断档也需一起迁移：hard_age_s是host根时效，不应原样施加到快速仿真的物理tick间隔。物理连续性应依据Nav提供的原始区间/同epoch前沿与有效估计契约；host失联仍按原根期限，未知区间仍分段，不能为了支持快仿真取消真实异常检查。

## 保持不变的边界

- `advance(now=host steady)`推进状态/等待，`motion_emitted(now=host steady)`记录首个成功提交；3秒动作窗口及预消耗恢复防重放完全沿host monotonic，暂停时也不延长/续开。计时结束只停止动作，不假称有物理位移或学习成功。
- canonical LearningRecord里的动作t0/deadline、commit ns、host boot身份不塞入simtick；冻结几何digest/1cm匹配/单向撤销不依赖可变仿真速率。完成模型恢复不因控制计算epoch变化人为失效；原始证据跨world重置不拼接。
- 不刷新TTL、原根deadline或ROS/TF标签，不重新引入15cm/8cm/.05rad/30秒链，不提高最低证据样本数，不假称暂停获得独立运动。
- owner/mailbox/shared-record无需因双域引入第二事实owner或逐帧提交；本轮不搬目录、不展开总体架构重构。

## 并行边界与需要Nav冻结的内容

已发送`msg_576105021a7a`到Nav `ctx_78d207f1bc18`，请求冻结真实typed measurement坐标、estimate区间/pose对齐接口，以及sim所有producer是否已用同一world source_epoch。后一项是跨源sim对齐必需的生产证据；如果各producer仍各自随机epoch，不能在学习消费者擅自声称它们是同一个世界，也不应另造world epoch绕开既定字段。

建议Nav拥有SourceTime/wire/producer、MotionEstimate/pose对齐视图；学习原owner后续按冻结接口接手StartupLearning积分/frame、LearningPoseHistory与SelfReturnModel排序/保留。先前Nav按measurement steady迁移的hunk应与本轮物理坐标接口一起审，不通过简单重命名computed_steady冒充完成。当前没有抢改任何共享生产hunk，后续具体产品返修由负责人追加。

## 新契约必须补的同版验证

- 同一真实sim物理轨迹以0.5x/1x/2x及变化速率投递，积分/位姿/分类几何一致，host TTL按各自真实前沿有效；不存在用hostdt偷代物理dt。
- sim_step=0首样本合法；暂停时host继续前进、simtick不动，不积距离/不跨未知段，首次成功输出起host3秒后动作永久结束，恢复不重放。
- world epoch变更且tick归零开新段；同epoch真实tick倒退、乱序/重放分别按契约处理；ROS标签大幅前后跳不影响物理tick排序。
- host超时、排队迟到与物理tick进展独立；旧根deadline不得因新计算/新simtick更新。
- 同tick重复源帧不产生假位移；跨源不兼容epoch/tick不能伪造插值。硬件driver_read在没有设备tick时保持既有行为。

本文件为接口适配建议和交接点，不是产品返修完成或新增测试通过证据。
