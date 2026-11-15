> 实施授权：协调者 msg_170d686f2959 已批准按本方案串行开发；其中历史待审措辞保留提案来源，现以本授权及缺轴预测自身硬截止修订为准。独立停车包络/decel/自回波学习/旧链清理不在本轮。

# 统一传感器时效、多源估计及队列诊断实施计划（协调者准予实施）

> **实施状态：** 协调者已批准，由当前 worker 串行实施，未派 helper。下列原始工作清单保留用于核对范围，逐项实际落点、验证与未完成边界见[实现报告](../../testing/2026-11-15-unified-estimation-implementation.md)；不能用未勾选的历史计划判断当前产品状态。无需中间阶段批准，最终累计 diff 交协调者亲审。

**Goal:** 用统一时效和逐物理量的明确估计算法替换多层挑源/提前拒绝，缺侧向不禁行，并统一年龄 DEBUG 和队列积压 WARNING。

**Architecture:** 输出组合每控制 tick 调用一个 MotionEstimation owner；来源 facade 只适配。PoseEstimation 与 SpaceEstimation 独立拥有位姿与空间计算，共用同一时效/lineage 契约和诊断 reporter；controller/runtime 消费结果，不再原始挑源。

**Tech Stack:** 现有 Python/Pydantic/pytest、ROS2 Humble Python/C++、既有 geometry 库与记录协议；不引新运行服务或第三方融合框架。

**Spec:** [完整算法提案](../specs/2026-11-15-unified-sensor-estimation-design.md)；[实际队列盘点和参数](../../testing/2026-11-15-estimation-queue-inventory.md)。两者已获协调者实施授权；本次实际代码仍待最终 diff 审阅。已批准的新 main 设计仍适用，以下明确修订取代其中旧三分量/多源强门及旧时间窗，不重新撤销已有可运行仿真范围。

## Global Constraints

- 保留 `07b815d`，当前 `fix/degraded-drive-speed`/当前工作区继续；不重试 GitHub、不改 main、不改其他工作区或运行配置。
- 共享默认 soft=.3 s、hard=1 s，等于hard失效，实车仿真一致；原始时间不可被处理/转发/预测刷新。
- 逐分量单源可估计，vy缺测按0且标假设；两个冲突取原始新者，三路以上选两两一致组再加权，不重复计同源派生票。
- 手动 TTL、authority/gesture/revision、任务期限、控制 dt、计算预算不作为传感器年龄改动；区分并测试，禁止字面量全替换。
- 保留来源/代次、时钟、结构与实际障碍保护；不伪装协方差/独立真值，不用这些要求退回“全拒绝”。
- 独立停车包络是排队任务，本轮不实现、不派 Agent，不修改其速度域或停止轨迹算法；估计接口明确 observed/independence/error 供后续 owner 消费。
- 不启动实车、VM 或开发服务；无完整ROS编译环境继续 Draft。由当前实现 worker 跑新增/受影响测试，负责人只审实际diff/证据，不重复例行测试。
- 兼容代码标 `//Compatible`；组件超3属性用命名props，局部样式不用全局CSS。

## Review Focus

1. 一个旧贡献者到1秒，其余活源仍有值：同tick重算不产生全局 feedback_stop。任务2/5覆盖。
2. 共用原始 wheel 或重发布数据不能因多topic成为多数派；共同heading误差不得被独立票数缩小。任务2/3覆盖。
3. ROS停顿时steady仍把旧数据带到硬过期；外推、重复snapshot不能延寿，.3仅降可信。任务1/2覆盖。
4. 位姿关联失败/两个不同障碍/未知空闲区不能被数值平均抹掉。任务3覆盖。
5. 日志满时告警自身不能递归制造积压；未知qsize不能伪造0；latest槽忙不等同FIFO堆积。任务4覆盖。

## 文件与接口归属

建议新增 `src/senior_control/sensors/policy.py`（唯一时效定义与加载）、`runtime/diagnostics.py`（统一有界统计及队列告警）。新增 `motion/estimator.py` 作为唯一数值owner，重用/修订已有 `observations/sample_types/source_admission/temporal/fusion`，删除被替换的生产分支，不保留第二套可独立调节策略。位姿建议 `motion/pose_estimator.py`，空间建议 `safety/space_estimator.py`；不把所有内容堆进 ROS 大节点。

明确输出接口（实现前可微调命名，语义不能缺项）：

- `SensorTiming.classify(origin, clock) -> AgeState(age_s, confidence_band, original_deadline)`，0.3/1唯一来源；来源验证仍在可信适配入口。
- `MotionEstimation.ingest(MotionSample, provenance)`；`estimate(T_ros_ns,T_steady_ns) -> MotionEstimate`，三分量独立含 observed/value/confidence/raw_times/roots/error/independence/reason。
- `PoseEstimation.at(T, motion_history) -> PoseEstimate`，含原绝对观测时间、地图/时钟代次、插值/预测类型与累计误差。
- `SpaceEstimation.at(T, pose) -> SpaceEstimate(occupied, observed_free, unknown, tracks, provenance, error)`。
- `RuntimeDiagnostics.observe_age(...) / observe_queue(QueueSnapshot) / flush(now)`；队列owner提供公开snapshot，不泄漏可变私有容器。

## Task 1：统一时间、根来源及部分分量契约

**Files:** 新 policy/defaults；修改 `motion/observations.py`、`sample_types.py`、`contracts.py`、hardware/simulation profile派生与配置加载/schema；相关 ROS profile/launch。

- [ ] 写失败测试：所有动态传感器 .299/.3/.999/1；采集到接收延迟+steady排队；暂停/倒退/未来时间；两个frame派生年龄取真实根；vy缺测不损坏vx/wz。
- [ ] 实现唯一软件默认文件和全流共用参数对象；旧max_age兼容输入只迁移到共同策略，不能悄悄保留硬件.5/仿真.8两个运行值，兼容处标记。冲突旧字段给明确迁移说明，不改设备文件。
- [ ] 明确 interval代表时刻、原始动态根时刻、静态标定代次；统一整数时间边界与输出confidence/observed，不在纯契约里做控制。
- [ ] 针对性 RED/GREEN：新 `tests/unit/test_sensor_timing.py`；旧observations与profile受影响测试只迁移与本条矛盾的断言，保留原结构/来源反例。
- [ ] 回报该阶段diff/证据；最终累计交协调者审阅，不因局部绿灯声明链路验收。

## Task 2：唯一逐分量运动估计算法

**Files:** 新 `motion/estimator.py`；`source_admission.py/temporal.py/fusion.py`；不改输出/包络。

- [ ] 以文档明确的数值表写失败测试：单源、接近按权重、二源冲突取原始新者、同时间质量平局、三源一致组/非传递链、无一致组、离群最新源、同源派生去重、最多8独立组。
- [ ] 加权和对齐严格按 spec：点/interval规范化、同源因果斜率或ZOH、每轴tau及误差附加上限、年龄/质量权重；不把未知covariance当拒绝条件或已知低误差。
- [ ] 输出partial estimate和vy assumed_zero；删除CI-only资格、双边冲突清空、三个分量齐全、5份恢复才能首次使用等旧门。保留clock/source fences，不递归反馈融合输出。
- [ ] 固定数值验证confidence公式、低年龄上限、冲突降可信、未知误差标记与“不以分数再设禁行门”；只有全相关实测无效才走feedback lost，.3边界不撤销authority。
- [ ] 写并通过“过期一源仍有另一源”“预测到T不刷新age”“单wheel同时给vx/wz”“仅gyro给wz、vx缺测可由控制模型使用但不可称实测”的精确断言。
- [ ] 用合成场景 RED/GREEN 和真实可用的离线记录（若存在）检验。报告软件阈值影响，不自动调整阈值造通过；不把spike当新模块已实现。

## Task 3：位姿与障碍空间的统一结果

**Files:** 新 pose/space owner；修改 `motion/kinematics.py` 必要复用、`motion_core_output.py::_observation`、`hardware_reference.py`、现有depth/vision输入边界；只改感知对齐，不改停车域/包络算法。

- [ ] 写失败测试：有历史时位姿插值、稍旧位姿以运动预测、角度+179/-179、跨map/epoch拒绝混算、绝对过期但相对dead_reckoning标记清楚。
- [ ] 实现同物理量一致规则、原始根和误差传播；GPS body转换缺真实heading只影响不可转换分量，其他有效来源可用。
- [ ] 写失败测试：左右两个障碍不平均、同一track冲突占用不能被中心选择消除、来源缺失仍unknown、单一健康感知方向可用、旧free到1秒撤销、位姿误差收缩free/扩占用。
- [ ] 实现按同坐标/时间关联的占用/可通行结果，去掉0.15秒pose对齐差全局拒绝。保留真实不可支持所需扫掠时的制动与遮挡语义。
- [ ] 报告接口对原停车包络的消费依赖，若其旧策略仍导致某场景停驶，登记到后续排队owner；本轮不顺带修。

## Task 4：有界年龄统计及队列积压WARNING

**Files:** 新 `runtime/diagnostics.py`；按队列表中owner分别接计数点：native_receipt/RGB-D inbox、vision处理、constraint scheduler、agent/relay/IPC、tasks、vehicle_logs等；C++使用同一配置schema及状态规则，Python/C++公共用例验证，不各自发明策略。

- [ ] 写纯时钟失败测试：avg/max/count/soft/hard窗口、重复原始根不重复raw计数、use_age单列、compute/control_period/queue_wait不混算，5秒DEBUG不逐帧。
- [ ] 实现固定容量注册/统计桶和可配置阈值；按队列盘点补公开snapshot/入出队/开始完成/覆盖丢弃计数，unknown明确null。
- [ ] 写并通过队列阈值4/16/64/1024、首次上穿、5秒限频、恢复/flap、单槽等待、qsize不可用、overflow清空N计N、日志自告警不递归测试。
- [ ] 保持既有容量、手动最新值覆盖、安全优先、任务自主和日志drop策略；WARNING不新增任何运动资格。源码审查不得读取executor/Lock/第三方库私有队列。

## Task 5：ROS/原生通道与控制消费接成单owner

**Files:** `motion_observation_adapter.py`、`motion_core_output.py`、`safety_node.py`、`motion_fusion.py`、`controller.py`、runtime/protocol/agent投影及 launch；native Python/C++ receipt、RGB-D/scan相应路径。

- [ ] 写失败链路测试：已认证raw样本→唯一ingest→estimate→controller/runtime，源缺失/过期/冲突不再各层重选；fresh wheel缺IMU仍可估计，valid vx/wz缺vy不反馈停车，所有相关实测过期才无反馈制动。
- [ ] 将数值owner移至输出组合，每tick一次；来源facade支持可选GPS、RGB-D并保留真实来源/外参信息。撤销旧core_reference的驾驶挑源责任，legacy progress若保留明确仅导航进度用途。
- [ ] 控制参考逐轴消费，未观测轴关闭PI、按已审模型fallback；独立body证据另给包络，不把wheel/model写成independent_body，不修改排队包络公式。
- [ ] 替换native transport .25、RGB-D .3、profile .5/.8、hold/predict/pose .15等同义sensor早拒绝为唯一共同策略；真正matcher配对算法跨度/结构约束按含义审，不能称同义年龄而盲改。latched静态标定/控制状态/急停通道使用自身语义，不套动态传感器年龄。
- [ ] 移除 controller重复body/wheel矛盾拒绝及固定源组合 gate；新mode切换只调参考/积分权重，不重置轨迹或authority；保持手动TTL、急停版本、轮限和方向障碍测试。
- [ ] 运行受影响链路定向pytest及静态检查，报告前后资格路径盘点；不以人工 monkeypatch healthy 冒充接通。

## Task 6：覆盖缺口、文档与Draft交付

**Files:** 定向tests、已批准spec/plan的修订记录、testing报告；如协议字段变化更新同版schema/原生codec、前端类型与投影。

- [ ] 单一来源/慢源.3至1/冲突组/缺vy/队列洪峰的真实软件链和现有离线记录覆盖；每项说明合成、回放还是ROS/实车，不混称。
- [ ] 新增改动或明确覆盖缺口才补测；已通过同版同环境结果可引用，不为流程机械重跑。负责人审实际diff和原始输出，worker负责测试。
- [ ] 在已有明确可用的隔离Linux/Humble环境才做ROS消息、guard/receipt完整构建及CTest；当前不唤醒挂起VM、不启实车。无条件则保持Draft并列确切待验项。
- [ ] 保留原07b815d、当前停车包络排队项、网络发布阻碍和所有历史失败事实；新的实现完成也不自动推main/合并。

## 已获授权的实现决定

用户七项算法要求和补充队列/空间要求均映射到以上任务；没有安排子Agent或停车包络实现。协调者 msg_170d686f2959 已授权的软件初始参数及决定：唯一owner移至control组合；基础一致阈值 .10 m/s/.15 rad/s 与位姿 .10 m/.10 rad；权重公式及最差无一致组/同时间平局规则；vx/wz缺轴时的model_only feedforward而非伪实测0；队列阈值32/.75、5秒限频和300ms等待诊断。已按上述决定实现并做模块验证；数值不是用户指定或实车标定，不代表完整验收。
