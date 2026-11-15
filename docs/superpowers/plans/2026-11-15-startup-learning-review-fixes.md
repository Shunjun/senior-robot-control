# 启动学习评审返修计划

基线b64bca0，当前分支原地实施，不派helper、不启车辆/ROS/开发服务。

1. R1：StartupLearning记录带原ROS时刻、独立积分位姿、累计误差、静止状态和代次的有界历史；frame仅在采集时刻有可对齐历史时入模。模型按采集时间排序并去重；后验计数只接受停稳后实际采集且捕获位姿为静止的独立新帧，不以回调时间代替capture。
2. R2：body/world完整SE2变换保留；世界预测的端点误差、首尾位姿支持误差与显式附加motion_error组成同一支持域，同时用于无匹配判定和明确free覆盖。中心free而支持跨遮挡保持歧义。
3. R3：每次_observation在缓存判断前核对当前physical polygon及软件误差参数；变化立即撤销启动/模型/空间缓存，重建模型和run内feature快照，不按旧body过滤。核对ROS实际参数更新入口。
4. R4：初始化等待不计学习30秒预算；取得初始化运动/静止证据并转sampling后起计，同时保留run claim一次性。
5. 实际统一估计+raw scan驱动完整core周期，覆盖合法不同延迟/乱序、停稳后旧运动scan、completed/ambiguous；受影响回归而非191全套。审查supervisor测试只临时socket和假子进程后按权限机制运行。

每项先RED再GREEN，最终记录命令、实际版本指纹及ROS/实车未验证边界，提交后负责人继续亲审累计diff。

状态：R1–R4已串行实施；最终69项受影响测试通过，临时socket定向11通过/1平台跳过，详见[返修证据](../../testing/2026-11-15-startup-learning-review-fixes.md)。未重跑191全套；首次运动、模型与GPS之前功能仍需负责人继续审累计diff，ROS/实车仍Draft。
