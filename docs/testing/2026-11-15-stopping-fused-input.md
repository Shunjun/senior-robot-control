# 融合停车输入修复证据（纯模块，Draft）

代码基线 `a88bd548e73abf4d57d329826adbfbb62e6b0104`；分支 `fix/degraded-drive-speed`，开始时HEAD一致且工作区干净。最终产品/测试文件SHA256、完整测试argv、环境和原始结果保存在[验证JSON](2026-11-15-stopping-fused-input-verification.json)，代码提交以该报告所属提交为准。

## RED及修复效果

先新增实际 `MotionSensorRuntime→MotionCoreOutput._snapshot→ConstraintEngine` 组合测试，原代码结果 **4 failed**：三个轮速静止/低速/加IMU场景实际仍给 `(-1.5,+1.5)`，另一个断言decel仍是0.1。原场景 `Point(1,0)` 保留；实测旧组合输出 `model_horizon_exceeded`，仅把decel改0.5依然 `stop_envelope_blocked`，证实只改减速度不能解决速度域。

修复后的29项新测试覆盖：静止/低速单轮速和wheel+IMU、laser失效剩轮速、冲突新wheel/新laser分别成为统一owner选中结果、0.3/0.6/0.999低可信仍可用和1秒失效、有界缺轴hold及独立截止、已知总误差不重复叠加、真实前向惯性、换向制动、转弯侧障碍、非零轨迹残余、实际controller前障碍倒车/急停候选、硬件0.3默认满杆和正负半杆比例、profile/schema/launch选择、decel时长与限频DEBUG。

实际默认硬件模型（非实测）：静止线速度域±0.02 m/s，线停车距离0.014472 m，含侧向settling的模型时长1 s；0.02 m/s时域[0,0.04]，线距离0.032649 m、时长1.132456 s。融合1.5 m/s时域[1.48,1.52]，decel=.5时4.79 s/4.0204 m；同一输入仅换旧decel=.1变15.95 s/12.502 m，超过12 s模型允许时长。上述线距离不包含位置误差/车体/横漂/角扫掠；这些几何保护仍保留。

原1 m前障碍场景没有改小障碍或误差：修复后当前停车可验证，候选倒车依然因保守角扫掠 `no_clear_candidate`。新增独立2 m前障碍和显式后方free-space场景可产生倒车许可，真实controller得到verified负向候选。相同1 m前障碍、实测0.8 m/s前向速度仍返回 `stop_envelope_blocked`。没有将障碍改free或忽略实测前向惯性。

## 回归及测试迁移

初次扩大检查 **260 passed,62 failed**。其中50项旧手动比例测试仅伪造Feedback，没有调用统一owner；改为真实轮速/IMU输入并移除旧的assumed_body_cap=.01/linear_error=0缩窄夹具，正常模式也由真实laser输入产生。原比例/设置/导航能力期望保留。

原捕获启动场景此前依赖虚构最大速度撞到远墙；修复后暴露真实 `sweep_unknown`，仍没有许可、仍输出零。测试保留原始占用见证及误差，不再期待由缺独立body误报FEEDBACK_LOST。为继续覆盖既有一次性启动/故障语义，相关用例显式增加合成的当前车体内占用点；没有改产品状态机，也没有删除真实碰撞验证。已有estop/fault锁存回归保留。

在 `/private/tmp/stopping-a88bd54-baseline` 用 `git archive a88bd54` 原码复现三个受影响旧测试文件，结果 **93 passed,2 failed**，两项均为test_tick_evaluation命令夹具遗漏SCF3手动零元数据。补齐这三个零字段，并把其历史laser_reference显示期望迁移为统一owner实际fused_reference。首次最终扩展组另有一次 **369 passed,1 failed**，原因是同一文件第二条旧laser_reference字符串断言遗漏更新，修正后重跑最终组；不把重复运行数累加为独立用例数。

最终20个测试文件定向组 **370 passed in 21.81s**，精确argv及输出见验证JSON；Ruff check/format、三项改动纯模块的mypy（`--follow-imports=silent`）及diff检查同时记录。没有宣称完整仓库mypy、完整ROS构建或整车验收通过。

## 未验证边界

未启动服务、VM、Docker、ROS或车辆；未写车端已部署配置，未网络重试、推送或创建/合并PR。宿主无ros2/colcon，完整Linux消息生成、scan matcher/RGB-D receipt/guard链接、ROS/实车效果仍未验证。部署必须同版重建SCF3/823消费者及ScanMotion双原始capture字段相关消息/原生消费者，保持Draft。

这是用户授权停车模型阶段的代码与纯模块检查，负责人还需亲读实际diff后决定发布；后续自回波学习/GPS朝向、兼容链清理及整体优化审查不在本次范围。
