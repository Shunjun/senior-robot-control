# 车辆改版阶段索引（2026-11-14）

本索引用于区分历史报告与当前范围，不修改旧报告当时的失败、待审、未提交或未接线事实。权威需求是[用户已批准的新设计](../superpowers/specs/2026-11-12-vehicle-motion-redesign.md)及[当前实施计划](../superpowers/plans/2026-11-12-vehicle-motion-redesign.md)：目标5 km/h，已接受非手动任务断网继续，手动接管暂停且须明确继续，允许有方向感知支撑的受限轮速退化。

功能主分支为 `fix/simulation-runtime-qualification`；先审阅收口其全部未提交改动，再合版。`vehicle-motion-integration` 只作临时验证，后续 merge 回功能主分支，不 cherry-pick。文档列出阶段提交不等于授权提交剩余脏文件或合并分支。

|阶段|原报告入口|当前解释/边界|
|---|---|---|
|设计/审计|[硬件](../reviews/2026-11-12-vehicle-hardware-audit.md)、[控制](../reviews/2026-11-12-vehicle-control-audit.md)、[设计复审](../reviews/2026-11-12-vehicle-design-builder-check-v2.md)|审批前评审及旧分支状态是历史证据；新设计已于2026-11-12获用户批准，后续用户修订优先|
|T1工具及默认基线|[工具](2026-11-12-vehicle-motion-baseline.md)、[默认基线](2026-11-12-vehicle-motion-t1-review-baseline.md)|工具通过不等于流完整或物理停车通过；旧默认失败保留，不能当新版拒绝依据|
|T1有限标定|[低速](2026-11-13-vehicle-motion-low-calibration.md)、[扩展](2026-11-13-vehicle-motion-expanded-calibration.md)、[可观测场地](2026-11-13-vehicle-motion-observable-corridor.md)|仅支持报告对应速度/场地候选；高档依赖停止、弱特征/估计失败不抹去。29项正式边界仍null、未冻结|
|matcher诊断/回放|[诊断](2026-11-13-vehicle-motion-matcher-diagnostics.md)、[后续隔离与epoch复核](2026-11-13-vehicle-motion-t3a-foundation.md)|较晚报告补充诊断失败隔离和body epoch准入；不改匹配评分门槛，不把诊断通过当精度通过|
|T3a/T3b纯估计链|[T3a](2026-11-13-vehicle-motion-t3a-review-fixes.md)、[T3b](2026-11-13-vehicle-motion-t3b-review-fixes.md)|源码阶段已分别提交0959041、6e4adbe；历史报告“待审/未提交”保持当时事实。真实数值CI仍0，统计模型缺口未解决|
|T4a/T5a纯约束与闭环|[T4a](2026-11-13-vehicle-motion-t4a-review-fixes.md)、[模型假设](2026-11-13-vehicle-motion-t4a-assumptions.md)、[T5a](2026-11-13-vehicle-motion-t5a-core.md)|源码阶段已提交c569658、dae9f568；synthetic模型/理想plant逻辑回归不证明Linux实时、物理包络或5km/h|
|任务/配置/日志与UI|[任务生命周期](2026-11-13-vehicle-task-lifecycle.md)、[任务接线](2026-11-13-vehicle-task-runtime-wiring.md)、[设置](2026-11-13-vehicle-settings-core.md)、[日志](2026-11-13-vehicle-logs-core.md)、[UI提案](../reviews/2026-11-13-vehicle-ui-contracts.md)|主分支已有76f260e、f1ae293、4f3a750、3473d54、3e22507分阶段提交；独立worktree后续增量仍须审阅后merge，UI提案不替代当前未提交前端增量验证|
|T5b隔离仿真实际接线|[输出](2026-11-13-vehicle-motion-output-wiring.md)、[Humble入口](2026-11-13-vehicle-motion-dds-entrypoint-fix.md)、[正常运行](2026-11-13-vehicle-motion-normal-runtime.md)|主分支f06d046已收录获审simulation-only素材；同版Linux构建成功，r6低档未通过、停稳未证，其余三场未运行。retired是fence+独占本机制动接管，不是硬零或车体静止|

已批准显式隔离、simulation-only/unvalidated candidate 正常接线与有界调试，不要求29项测量全齐才开发。候选的 assumption、原证据与有效范围必须保留，不能据此放行实车、伪造独立body测速或将acceptance null填成已测。当前唯一仿真 `simulation_settings_profile()` 默认倒车请求上限0.3 m/s，允许的用户设置仍受该profile范围及实际方向/轮能力约束，不是硬件能力声明。

早期无Gazebo代码harness在受控调度、worker及时完成假设下展示低档数值闭环；后续真实wall驱动及ROS synthetic为独立调度对照，不等待worker完成才推进时钟，其合成plant结果仍不能替代真实驾驶。临时验证分支6fae同版实际Gazebo受限低档运行仍失败；235/349ms缺口完整根因尚未证明。高档达速、网页/导航整链及实车均未验收，不能用阶段单测数或80%阈值宣称5km/h达标。旧日志和失败曲线原样保留；后续正常主路径验证应绑定合版源码、安装、配置和原始证据。
