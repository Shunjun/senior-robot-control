# 停车风险输入接入统一运动估计（已实施，Draft）

基线 `a88bd548e73abf4d57d329826adbfbb62e6b0104`；按本次派单直接在 `fix/degraded-drive-speed` 串行实施。优先遵循用户最新要求，本阶段替代旧的“缺独立 body 即 ±assumed_body_cap”停车策略；独立静止/复位证据仍独立保留。

## 职责及公式

- MotionEstimation 继续唯一决定逐轴数值、源冲突、预测误差、原始年龄及期限，停车输入不再选 independent_axes 或重选 wheel/IMU。
- `motion/stopping_input.py::StoppingRisk.domains` 把统一 vx/wz 转成有符号区间，并单独保留失效轴的停车残余。每轴误差 `max(profile测量误差假设 + prediction_error, 已知总error或0)`；已知总误差已经包含预测，取 max 而非再次相加，不重算年龄。误差/能力/减速度都是未实测软件假设，不是物理保证。
- 合并非零轨迹残余速度；零轨迹只是命令目标，不能把明确前向实测区间拉到0而抹掉 `reverse_requires_stop`。正负域不由用户请求方向决定，实测前向惯性及轨迹残余仍参与前障碍检查。
- 有界缺轴 hold 沿用该轴原始 deadline。轴 value 到期变 None 时，现有输出组合关闭该轴 target，停车账本仍保留最后融合域及增量不确定性；实际零轴输出后的有限制动响应按下述返修模型推演，另一轴不刷新旧轴年龄。全相关反馈丢失仍沿原无反馈停车路径。vy 缺测按估计器明确的0假设，不新增齐轴门。
- ConstraintEngine 的几何、占用/free-space、未知空间、轮能力、反应延迟、jerk、换向及急停逻辑保留。只抽出 `stop_prediction` 供几何和诊断共用相同计算，未修改扫掠公式。
- 仓库 `config/hardware-motion.json` 仅 decel 从0.1改0.5 m/s²；用户指定偏保守估算，ASSUMED/no measurements 原文保留。schema无数值冻结，不需改生成schema；实际 profile loader、launch选择函数及schema一致性有测试。车端文件未操作。

## 诊断与限制

复用 RuntimeDiagnostics 默认5秒 DEBUG pump，以有界 latest summary 保存停车速度域、预测时长、允许时长、线停车距离、角停车角度、逐轴原始年龄和期限、约束原因；同一reporter的有界数值流保留周期内原因次数和预测最大值。`model_horizon_exceeded` 是停车预测超过模型允许时长；`stop_envelope_blocked` 是扫掠与已知占用相交；年龄不是停车模型时长。没有第二个 reporter、额外定时器或逐帧日志。

保留保守扫掠的适用限制：硬件默认参数、静止/0.02 m/s、前方1 m障碍的原夹具当前停车可验证，但候选行驶仍因既有最大加速度/角扫掠得到 `no_clear_candidate`。没有移动该障碍或缩小误差使之通过。另一个前方2 m、后方有显式已观测free-space的场景形成倒车许可及真实controller负向候选；不等于任意前障碍距离均可倒车，也不是实车验收。

自审已确认移除按独立来源分叉的旧速度域构造，没有新增来源组合资格门；公开旧_snapshot形参暂留，不夹带兼容链清理。15厘米自回波/GPS朝向/最终整体审查留后续任务。验证及失败迁移说明见[测试报告](../../testing/2026-11-15-stopping-fused-input.md)。

## 评审返修计划：过期轴残余（2026-11-15）

已复现 vx=.8、持续新鲜IMU、trajectory=0/.2 的1秒截止跳变：上界1.1197变.02/.22。选择独立于传感器融合的逐轴停车残余账本：保存最后有效融合域及原始截止；无轴时保留该域并按响应加速度延伸增量误差，绝不改MotionEstimate或其TTL。仅实际成功发送零轴输出后启动软件制动响应；沿原stop_axis的lag/jerk/decel相位推演剩余速度，重复零输出不重置计划，非零输出取消收敛；新鲜观测恢复时替换该轴残余。选择依据：仅保留trajectory会清掉惯性；永久冻结旧速度会在已实际执行零命令后无依据持续拒绝。制动推演仍ASSUMED，不能用于独立静止或故障复位。

实施顺序：1. 组合RED（已失败2项）及恢复/实际零输出/无反馈测试；2. stopping_input引入仅两轴有界状态并接_snapshot和实际emitter；3. TickEvaluation保留本tick停车预测，几何直接复用预测域/峰值，不为诊断另算；4. 复用RuntimeDiagnostics有界数值流统计最大值/原因次数，latest仅保留上下文；5. 仅跑受影响运动/诊断/快照组合、lint/type/diff检查，追加提交并回报。不得改扫掠算法、profile误差或其他参数。

本次返修已按上述顺序实施，334项受影响检查通过；实际接口、响应假设和边界见[返修证据](../../testing/2026-11-15-stopping-expired-axis-review.md)。停车预测复用本tick proof，未改车体扫掠公式或profile数值。
