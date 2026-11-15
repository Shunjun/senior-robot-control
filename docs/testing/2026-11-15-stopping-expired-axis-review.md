# 停车模型评审返修：过期轴残余

基线 `e277e6326b416e0726ad3cf4f650b56b468180ef`，仍在 `fix/degraded-drive-speed` 原分支追加，开始时工作区干净。负责人指出的缺轴清零及诊断重复计算/瞬态丢失均在本阶段处理；本报告修订首轮报告对缺轴停车风险的结论，首轮370项通过不再作为该缺口已覆盖的证据。

## RED与根因

先新增真实 `MotionSensorRuntime→_snapshot→ConstraintEngine/MotionController` 截止组合测试：vx=.8，持续更新IMU，trajectory=0/.2，前障碍仍为原来的Point(1,0)。`PYTHONPATH=src <共享venv>/bin/python -m pytest tests/unit/test_stopping_fused_input.py -q -k expired_linear --tb=short` 得到 **2 failed,29 deselected**：截止前上界1.1197，截止后变.02/.22。根因是缺轴分支仅取trajectory±测量误差，把过期当成停止，与零指令不是静止证据的原则冲突；旧测试甚至把±.02固定为期望。

## 实现与假设

`StoppingRisk` 只保留两轴的停车/执行器响应状态，绝不把过期值写回MotionEstimation或重新参与融合。每轴保留最后有效融合域、计算时刻和原始支持截止；没有后续观测时按原响应加速度扩张增量误差，不重复叠加之前已计的年龄误差。目标关闭和逐轴原始TTL不变；停车风险域在截止处不缩小。未解决的残余还使用响应加速度上界进入未来停车模型，不能因为轨迹加速度0就假定物理加速度0。

只有实际emitter成功返回、所下发该轴值确为0后，才从当前残余和既有drive overshoot假设建立stop_axis响应；包含原lag、jerk、decel及未知向外加速度。正负端分别按相位计算剩余速度，非零输出取消零响应；重复零输出不会重启计时。该账本在_inputs即更新，即使后续感知读取失败也不会丢最后运动。异常/无反馈路径实际下发零输出同样记录，但全反馈失效始终不能因此变成healthy。

模型可有限收敛为`assumed_settled`，这是未实测的零命令响应假设，不是新鲜速度观测、独立静止证据或故障复位条件。新鲜轴观测恢复时重新以统一估计构造风险；既有ESTOP/执行故障锁存不自动清除。启动以来从未观测的轴仍明确为`no_axis_history_assumption`，不伪造传感器票；本返修针对已知运动失效后的丢失，未扩展为独立body资格门。

`TickEvaluation` 保留本tick的BrakingProof及StopPrediction，诊断直接读取该对象；_sweep使用预测携带的域和峰值，不再重复取bounds/预测。即使模型时长超过horizon而被拒绝，已计算预测仍可汇总。RuntimeDiagnostics继续默认5秒DEBUG，仅保留一份latest上下文；原因用同一有界数值流的count，预测时长/距离/角度用max，周期中间错误不会被最后正常值覆盖，没有第二个reporter或逐帧日志。

## 验证

精确argv、环境、输出和最终产品/测试文件SHA256见[验证JSON](2026-11-15-stopping-expired-axis-verification.json)。新证据包含截止前后trajectory0/非零及前障碍组合、实际controller拒绝、无实际输出不收敛、实际零轴响应有限收敛/重复命令不续期、非零输出取消、fresh观测恢复、全反馈失效始终停用、真实tick发零后才开始响应、感知失败仍保存最后运动，以及诊断缓存复用和瞬态原因次数/峰值。

最终15文件受影响组 **334 passed in 21.11s**，Ruff check/format、三纯模块mypy和diff检查均通过；精确结果见JSON，不把中间重复运行计入独立用例。三项纯模块mypy使用`--follow-imports=silent`；没有宣称完整仓库mypy通过。未改减速度或任何其他profile参数、原障碍/扫掠算法/误差夹具，1m处既有候选角扫掠限制仍保留。

未启动服务、VM、Docker、ROS或车辆，未操作车端配置、网络或PR，继续Draft。完整Linux消息/scan matcher/guard链接、SCF3/823及ScanMotion双capture同版重建、ROS/实车仍未验，负责人需亲读本次实际diff后决定下一步。
