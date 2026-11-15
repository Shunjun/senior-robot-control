# 统一估计审阅返修：R1–R3

> 后续审阅更正：R2同anchor直接相加端点累计误差会重复计入共同积分历史，且曾使本报告中的捕获近墙场景过度拒绝；本轮已改为有证据的局部积分区间，见[后续返修](2026-11-15-relative-interval-sync-review.md)。本报告保留28d0635阶段的执行事实，旧R2保守相加/固定负例不代表当前结论。

当前状态：负责人已亲读 policy/consensus/estimator、运行时、controller diff、pose/space及输出组合的相关代码并提出本轮三项修正；累计100文件审阅尚未完成。本轮以 `437b0c86ef72a77da6ab0fca3782fb8cd1a6d01c` 为父版本追加返修提交，保留 `07b815d`，仍在 `fix/degraded-drive-speed` 原工作区；三项实现和定向验证完成，待负责人复审实际增量 diff，不宣称验收。

## R1：按来源统计观测与实际评估尝试

- `MotionEstimation` 在候选选择前记录实际检查过的原始 observation；同一 observation 在 vx/vy/wz、普通/独立视图的评估合并为一条年龄记录，结果集合分别标 selected/not_selected/selected_independent/alignment_history/expired/invalid_time。因此未选中的慢源、冲突淘汰者及硬过期都保留其真实年龄。两个不同历史原始样本若实际用于候选或斜率计算，可各记一次尝试；不是按轴或视图重复算同一个样本。
- `MotionSensorRuntime` 的 raw age 在准入结果之外统计，quality=0/迟到/重放拒绝都可观察。raw age按原始 family/epoch/stamp 去重，处理/交付序号不能把同一次采集变成另一条 raw。每源去重窗口最多256，窗口跨5秒DEBUG刷新保留；这是有界去重窗口，不宣称无限历史唯一。
- `RuntimeDiagnostics.sensor_attempt` 将年龄与结果计数分开：raw age是唯一样本数，`sensor_results.raw.*` 是每次入口尝试结果（含重放拒绝），`sensor.use.{laser,wheel,imu}` 是每次估计实际评估的样本年龄。一个样本可有不同轴/视图结果标签，这些结果计数不是可直接相加的独立样本数。非法时间无年龄值，只计拒绝；`SensorClockError` 仍是 ValueError，不改变既有控制异常保护。
- scan每次 `_observation` 调用记录use，重复使用只增加use，硬过期/非法时钟尝试也记录。原生桥迟到/撤销输入使用原始激光两帧或 wheel/IMU header+原receipt统计raw/use，非突变身份方法读取provenance，不推进源序号/资格或重入估计。适配前的非法结构/来源拒绝只计结果、不伪造年龄0。
- DEBUG仍由原有有界reporter默认5秒统一输出，stats/结果标签/去重流各有容量上限，超限增加unregistered；未增逐帧日志、任务队列或禁行条件。

## R2：未知全局误差与相对运动误差分开

- `PoseEstimate`保留可空的完整位置/角度误差，新增原始anchor roots及 `PoseErrorBudget`。后者分别保留可空已知预测误差和显式数值软件预算；未知观测误差没有改成已知0。
- PoseEstimation使用现有profile.linear_error/yaw_error/lateral_drift/position_error/orientation_error等软件策略，不把软件预算叫物理认证。相对积分现在累加非零线/角速度误差、偏航对位移的杠杆影响及失去分量时的有界模型误差，既有已知误差不再停在0；已有未知误差继续None。
- 两次pose明确共用同一原始anchor时，其公共左侧SE(2)坐标变换误差可以抵消；锚后两端预测误差因没有完整相关性证明而保守相加，并计旧端角度误差对相对位移的影响。不同fix/无anchor证明时不抵消，完整误差未知则用明确profile软件预算做数值处理，metadata仍为None。已知anchor偏航误差影响全局位置外推，但不再次混进同anchor相对运动误差。
- `_observation`保持scan几何创建时的pose关联，重复使用不悄悄重估/移动原参考。每次以old/current pose计算相对误差；角误差按当前车体 footprint 半径换算位移裕量，因为不确定的是车身在scan参考系内的位置/角度，不是绕传感器旋转整片20米地图。
- `SpaceObservation.error_m`允许None，显式fallback政策独立保留；generic owner没有提供fallback时也使用声明的软件策略，非默认为未知=0。与完全相同scan参考变换相关的全局pose误差抵消，原sensor error仍由prepare_perception计一次；动态相对对齐误差在查询时扩占用/缩free一次。`SpaceEstimate`公开unknown_pose_sources及可空alignment_error与实际software预算，可从实际组合输出核对。

## R3：当前投影保留原始支持域、年龄和预测误差

- 每轴输出保留support_start/end、observation_age_s、prediction_error及可空总error；model_only的原始期限不续命。候选预测误差覆盖ROS对齐跨度及原始receipt到当前steady的保守年龄，因此ROS暂停不能让预测误差伪装成年龄0。
- runtime ref中的start/end恢复为实际输入支持域，另用computed_ros/estimate_kind明确当前投影；不再拿计算时刻当采集区间。原始roots仍保留采集/接收两套时钟，独立视图保留自己的误差与来源。
- `_snapshot`直接读取唯一owner的独立逐轴证据：`dv=max(profile.linear_error + axis.prediction_error, axis.error)`（axis.error未知时仅使用前项），dw同理。已知总error已经含预测/组内差异，不再按当前时间再次加一份年龄扩展；未知总error继续None，profile基准加预测误差为显式软件假设。
- interval半宽/最老原始capture支持已包含在prediction_error，不在停车消费者再加第二次。原速度区间构造、停车轨迹公式、仅轮速/IMU的±最大停车域、decel、自过滤/启动学习、旧链清理均未改。

## 验证、失败记录与限制

新增 `test_estimation_review_contracts.py` 先得到8项预期失败，复现三项接口问题，再实现修正。新增覆盖包括慢源冲突淘汰/一源过期另一源存活、跨轴及独立视图去重、quality=0、非法clock、native迟到拒绝无状态变更、scan重复使用/过期、跨DEBUG去重与有界注册、已知误差积分、共享/不同anchor、实际_observation到空间输出、原参考关联、0.3/0.7/0.999秒独立投影、1秒失效、暂停ROS及interval不重复扩误差。

受影响组第一次为120 passed/16 failed：16项输出时序测试共用近墙旧观测，补齐误差后不能证明原通行。保留实际捕获场景及新增“估计健康但无许可”负例；输出期限/commit测试明确改为合成开阔几何，仍用原始motion证据、真实controller/T4/期限逻辑，不能称捕获场景或物理验收。没有缩小生产误差来恢复旧驾驶结果。补generic未知fallback时的None校验短暂暴露3项失败，已修正并由最终组覆盖。

最终代码的精确pytest argv、源码指纹、stdout及mypy证据见同目录 `2026-11-15-unified-estimation-review-fixes-verification.json`：

- 运动/诊断/位姿空间/输出时序/适配受影响定向组：**157 passed**。
- 退化满杆比例设置回归：**59 passed**；未修改该设置或映射。
- 10个变更/新增Python文件Ruff check、format --check与py_compile通过，git diff --check通过；5个变更纯源码模块严格mypy通过。
- 完整mypy src仍只有此前基线复现的4错（tasks/lifecycle:318、lifecycle_store:647、controller:625/627），未声称完整类型检查通过。

无ROS消息或构建配置增量，未重复跑未受影响的前端/原生纯模块构建。完整Linux ROS消息生成及guard/节点链接、实际DDS与实车验证仍缺，继续Draft；未启动服务、VM或实车，未改运行配置、未推送建PR或改main。以上软件误差策略仍需真实场景验证，其保守性可能收缩近墙通行，不能据模块测试宣称整车体验或物理精度通过。
