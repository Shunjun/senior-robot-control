# 启动学习 Implementation Plan

> Implementer: 当前worker串行执行，不派子Agent；用户已指定设计调整及直接实施，无额外审批。

**Goal:** start初始化后，通过既有安全链提出低速15厘米学习目标，同时支持可选单天线GPS朝向。
**Architecture:** StartupLearning管理生命周期与一次性claim；SelfReturnModel仅判断端点支持域并保留遮挡；GPSCourseObserver只输出有精度证据的旁路朝向。ROS组合消费这些owner，最终输出仍经过原ConstraintEngine和controller。
**Tech Stack:** 现有Python、Shapely、pytest、ROS适配；无新服务。
**Spec:** 本轮负责人指令及2026-11-12 vehicle-motion-redesign；本计划替代前版“必须额外硬件才能继续”的结论。

## 授权先验及限制

负责人明确调整：实车polygon是启动内部先验，严格内部且误差支持域不跨边界的端点可在学习期间provisional处理。这是先验，不是learned证据；外点与边界歧义仍occupied，遮挡后方unknown。保留车体几何/激光外参，不用射线相交判自身。目标.15m、软件学习速度.05m/s；总任务时限30s、至少5帧静止和5帧后验（采样计数不是传感器年龄）。无free则原避障等待，有运动后失去安全许可中断。共享.3/1秒传感时效；没有GPS也照常学习。失败/中断清除provisional与learned，后续不自动重试。

## Review Focus

- 车内起射的外部hit不能屏蔽；端点误差跨边界必须保留。
- 匀速空转/平行墙不能被wheel计程伪装成运动验证。
- 组件重启/重连不能重领一次性启动claim，急停/接管不恢复。
- 无free的遮挡区域不能因内部先验成为free。
- 缺GPS精度或15厘米短基线保持未初始化，不影响相对估计。

### Task 1: 端点学习模型与启动owner

Files: src/senior_control/safety/self_learning.py, motion/startup_learning.py, tests/unit/test_startup_learning.py。
- [x] 写失败测试并运行：内部/边界/外部、模型验证/歧义/撤销、claim重复、初始化/中断/只一次。
- [x] 实现 filter(observation, provisional) 与多帧body/world假设；仅独立平移与wheel/IMU一致时可learn；不足ambiguous。
- [x] GREEN并提交。

### Task 2: 控制链与配置迁移

Files: motion_core_output.py, docker_start_helper.py, safety_node.py, scan_adapter.py, shipped YAML; tests/unit/test_startup_learning_chain.py。
- [x] RED：实际start环境claim→核心tick→原controller输出；障碍/unknown不走，急停/接管清理。
- [x] 在服务启动进程环境生成run-owned claim路径，核心初始化领取，组件重启重复claim进入interrupted；无start上下文不自动学习。
- [x] 原安全仲裁接 startup target，过滤revision进入perception key；学习状态变更清除缓存，防止中断后沿用旧free/过滤。
- [x] 移除生产手填self_mask_xy读取及出厂配置；旧输入在loader标Compatible忽略，不取消实际polygon/外参。
- [x] GREEN并提交。

### Task 3: GPS旁路

Files: localization/gps_course.py, optional_motion_inputs.py, sensors/input_config.py; tests/unit/test_gps_course.py。
- [x] RED：精度未知/短基线/倒车/转弯/方向冲突/重放。
- [x] 仅显式ENU输入使用位置/速度及协方差；累积直行窗口，结合wheel方向与IMU，输出航向与原根期限；不依赖既有heading。
- [x] GREEN；定向回归、diff审阅、指纹报告及提交；ROS/实车仍Draft。

## 实际实现与自审补充

- 三项任务已串行实现，最终作为一个可审阅提交交负责人；没有派helper。前版阻塞由负责人明确授权的内部先验解除，不将先验冒充运动测量。
- 世界固定假设只有在其预测误差支持域处于首帧明确观测free内、且无可匹配回波时才排除；遮挡/盲区的“没找到匹配”不算反证。每模型最多64帧、每帧2048点、128个验证候选；未选点不过滤。
- 原始MotionEstimate给目标计程；验证另消费独立vx/vy/wz的相对积分和实际raw wheel一致性。vy假设0不冒充平移分类证据，也不阻止相对驾驶。独立位移误差沿现有profile线/角误差和已知逐轴error积累，扩世界假设/观测支持域；这仍是软件假设，非物理保证。
- 初始零目标采样/等待不会因内部回波先锁execution fault；没有permit绝不输出非零。首个实际非零输出后丢失permit即中断，撤销先验、重建原始感知后走原停车路径。急停和接管经真实command/revoke路径中断，不恢复旧尝试。
- 实际物理polygon现在进入Footprint.physical_vertices与许可identity；原5%约量外扩仍保守包围物理区域，未改停车扫掠算法、decel或其他profile数值。
- scan_adapter的匹配候选新增外部特征筛选。程序start的run上下文中，由输出owner从既有polygon生成只读边界快照；它不是自过滤mask，内部/边界回波仅不作为独立激光匹配特征，原始安全scan不变。快照尚未生成则matcher候选等待；轮/IMU相对估计仍可初始化。没有新topic/msg/年龄门。
- GPS course_frame是显式ENU、heading-only模式，位置/速度协方差有单位约束，未知/零精度不可信；IMU/轮速连续tick资格覆盖整段GPS窗口，不能仅以GPS瞬间的零角速度当全程直行。输出runtime.gps_heading及统一周期诊断，不冒充已有地图航向变换。
- 编写者已亲读新增owner、过滤、外部特征、start环境、仲裁和配置diff；遵照派单不另派reviewer，负责人后续独立审diff。已有手工mask的离线低层API暂留，生产节点不再读取self_mask_xy，兼容链独立清理不夹带。
