# 接收端短时雷达缺失补偿 Implementation Plan

> 执行方式已明确：原隔离分支单 worker 顺序实施，不派 helper；本轮仅编写设计/计划，负责人审阅后再开发。

**Goal:** 仅用连续真实局部表面给曾有真实回波的 +Inf 方向最多补三帧，预测只供现有行驶避障。

**Architecture:** 接收进程保留有界真实种子，原始 RawSafetyObservation 附当前稀疏预测；共享槽仍只传当前帧。控制端把原始学习观测与派生避障观测分开，复用同一已学过滤和方向限速。

**Tech Stack:** 当前 Python/ROS 2、numpy、冻结 dataclass、现有 LatestSnapshot；不新增依赖。

**Spec:** [接收端短时雷达缺失补偿设计](../specs/2026-11-18-scan-gap-compensation-design.md)

## Global Constraints

- 固定功能主分支 `fix/ros-typesupport-export-order`；现有 `fix/directional-speed-control` 正常 merge 同步/合回，不 cherry-pick，不推 main。
- 前置学习状态和无回波遮挡提交独立，不混进新算法提交。保持 70%、1cm 学习匹配、原三秒学习、唯一 Twist owner、lag/外框/单次雷达误差。
- 预测仅原始 +Inf；连续最多3个新采集帧，第4帧清除；重复快照不计数，预测不作真实种子。
- 原消息、原时戳/截止/身份、学习/测速、采样率与TTL不变；运动关联不足只跳过补偿，不阻断控制/原扫描。
- 本轮不写产品/测试、不跑benchmark/pytest，不SSH/启服务/运动。后续遵循产品修复→负责人非运动回放/部署核对→必要定向回归→负责人获授权一次真实学习的现场顺序，不能用mock代替现场结果。

## Review Focus

- 现有 odom 记录是否能在 raw scan 原始时间找到包围样本；不足时跳过补偿，不新增控制资格或速度积分。
- 栅格变化/运动/换代后种子与缺失计数绑定是否仍正确，不能按新index续命。
- 边缘/门缝/角点支持不足不补，不能跨越空档找远端支持；空间窗口按设计的局部实现选择，拟合残差直接用既有sensor_error；两者均不改变外框。
- 派生扫描的ranges与过滤标签必须来自同一当前观测，已学自体过滤不能被预测绕开。
- 新数据不进入学习或测速，四轮预算不会被重复发布/预测自喂延长。

## 文件清单

新增产品：`ros2/src/senior_robot/senior_robot/scan_gap_compensation.py`。

修改产品：同目录 `raw_safety_observation.py`、`safety_node.py`、`motion_core_perception.py`、`motion_core_output.py`。

新增测试候选：`tests/unit/test_scan_gap_compensation.py`；可能修改现有 `tests/unit/test_motion_core_perception.py`。本轮仅列候选，不写测试。

新增文档：本计划及设计，共2。总预计8～9文件，删除0，配置/构建/工具0；不修改ControlInputs、MotionSensorRuntime、scan_adapter、标准消息/厂家源码。

## 顺序任务

### 1. 冻结当前帧接口与数值建议

- [ ] 按负责人认可的空间最多3格、两侧2点/单侧3点、最多各3点窗口及有限延伸条件执行；残差e=profile.sensor_error，不按s/4缩小，不新增可调参数。真实回放报告支持率与误补，不保证辨识所有墙缝。
- [ ] 在 `raw_safety_observation.py` 定义冻结 `PredictedReturn(index, distance, missing_frames, seed_capture)`，原始观察只新增默认空 predictions 元组，旧构造保持可用。
- [ ] 明确 real 元数据由原始合法有限回波和本帧capture表达，不复制全射线标签/历史。

### 2. 接收端有界补偿与一帧缓存

接口：`ScanGapCompensator.update(raw, poses) -> RawSafetyObservation`；`clear() -> None`。输入poses直接复用现有bridge的有界本地记录，输出仅当前稀疏预测。

- [ ] 新模块实现同epoch包围位姿插值、真实种子坐标变换/当前角单元关联；不依赖独立估计资格，不等待/外推位姿。
- [ ] 按设计实现内部直线交点和边缘有限延伸；当前真实支持/原真实覆盖共同约束，退化/歧义/断层时不补。
- [ ] 缺失预算绑定真实采集种子，第四个新raw清除；不依赖控制tick或snapshot version，真实回波立刻替换。
- [ ] `_on_scan` 在原有接纳成功之后调用唯一实例推进新raw预算；缺包围位姿时发布回调可重试当前帧，不等待/不多计帧，成功后缓存结果；`_clear_scan` 清状态；八帧deque收为一帧，继续原LatestSnapshot传递，不新增历史通路。
- [ ] 亲审有界内存及O(N)数据流；必要语法、导入/格式检查，产品未完成前不运行回归矩阵。

### 3. 派生避障接入，保持学习原始输入

- [ ] perception保留`scan_observation`真实语义；仅有预测时构造局部派生ranges，绝不写回message。
- [ ] core的原始学习/遮挡路径不改，将派生避障观测交同一个冻结view或既有provisional过滤（只filter）。无预测直接复用已有filtered。
- [ ] directional_clearance枚举同一避障观测与标签，保持前后条带/物理排除/边界/年龄；limit_velocity不改。
- [ ] 亲查产品diff，提交产品独立SHA及实际检查结果；负责人正常merge、非运动同数据回放和耗时核对。若需超出5产品文件，先说明具体原因，不扩改。

### 4. 负责人回放核对与必要回归候选

以下只读候选由实现阶段转成最少独立场景，不展开参数矩阵，不新建公共fixture或端到端脚本。

| 候选 | 输入与预期 | 可复用位置 |
|---|---|---|
| 内部缺口 | 同一墙面真实种子，当前两侧真实支持中间+Inf；交点正确，metadata真实种子不变；真实回波回归立即替换 | 新test_scan_gap_compensation，复用perception现有scan/profile构造思路 |
| 边缘有限延伸 | 单侧3真实点和原连续真实覆盖；范围内补，超原覆盖/支持长度不补；带角点或门缝历史不跨连 | 新文件本地夹具，不新公共工具 |
| 三轮生命周期 | 同一个真实种子连续4新raw；1/2/3补、第4清；期间重复raw与快照不增轮、预测不续命，从无真实回波不补 | 新文件，复用sensor_time_test_support的真实SampleTime/capture字段 |
| 运动与栅格 | 标准odom两包围样本、平移转动及新栅格；世界同表面交点映射当前射线；无包围/跨epoch/FOV外不补、原始仍可消费 | 新文件，复用motion_observation_adapter的位姿字典字段，不镜像拟合实现 |
| 异常和真实优先 | NaN/-Inf/量程外不补；当前有效有限回波直接替换；断层取消旧候选 | 合入上述候选而非参数矩阵 |
| 控制/学习隔离 | 实际directional_clearance消费predictions后前后限速与当前公式一致；scan_observation仍读原+Inf，原message/stamp/deadline不变；过滤与range同索引 | 现有test_motion_core_perception；必要最小core接线替身放新测试文件 |

- [ ] 负责人先只读回放已有bag，同输入同帧记录旧/新接收处理、pickle编码/解码、载荷bytes、接收到快照发布的median/p95/max，保留输出差异及跳过原因。明确在哪台机器测量，不冒充车端。
- [ ] 若缺必要时间字段，先报最小临时入口/处理完成/发布时间和capture身份范围，不直接添加日志/benchmark脚本。
- [ ] 产品确认后只实现以上直接路径需要的用例，定向运行新文件及确有修改的现有perception文件，不全suite，记录通过/失败/跳过和SHA。

### 5. 负责人一次真实启动学习验收

- [ ] 负责人亲审、正常merge/push/部署；worker不访问车辆。
- [ ] 先被动确认停车/轮速/输出与雷达场地；按最新授权通过项目CLI新启动一次，保留原始扫描记录。
- [ ] 报告运动/停车、phase/reason、action字段、模型真实点数及车体坐标分布；不把候选当障碍数或0点completed当识别成功，不自动重试。
- [ ] 实际耗时和行为无证据的部分如实保留；不承诺30ms端到端、不调TTL、频率、画质或其它独立功能。
