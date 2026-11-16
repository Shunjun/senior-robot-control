# Navigation limits and clock Implementation Plan

**Goal:** 第8项有效导航目标经过共用能力限速；第9项控制时间与系统时间解耦。
**Architecture:** 共用 ConstraintEngine 根据当前 RiskEnvelope 同比缩放目标，再使用既有组合扫掠约束。导航样本须在最终生产端携带同车单调时间及可靠顺序，接收端不补盖新鲜度。
**Spec:** `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md` 与根目录《车辆启动约束审查与处理方案》第8—9项。

## Constraints

- 已派发 worker 自行实施，无子代理；不提交、不推送、不启动服务/ROS/车辆。
- 保留既有21文件未提交改动及三份独立改动；冻结记录 `/tmp/navigation-8-9-dispatch-baseline.json`。
- 不改变导航 degraded_cap 策略，不混入第10项。
- 按任务要求全部实现后集中迁移并验证测试，覆盖真实受限输出及身份/代次/新鲜度拒绝。

## Steps

- [ ] 在 constraints.py 以当前 RiskEnvelope 生成同比限速候选，保留原始目标身份与低速意图；删除 command_node.py 旧入口速度拒绝，保留非有限值校验。
- [ ] 核对官方 Humble controller_server 生产端与发布端；向负责人交付最小消息/适配方案及固定版本证据，确认后实现第9项，不以接收时间伪造生产时间。
- [ ] 集中迁移 navigation clock / small intent / directional constraints / output chain 测试，覆盖超旧阈值、组合、低速、零、负方向和真实受限输出，以及过期、重放、旧代次、NaN、来源身份、断流停车。
- [ ] 执行受影响离线测试与静态检查，记录命令、环境、失败与限制；核对既有冻结指纹，提交实际 diff 报告由负责人亲审。
- [ ] 根表格仅更新第8—9项实施状态，保留用户原文；仓库外文件按权限机制处理。


## 当前执行状态与扩展范围

- [x] 原第8项共用约束限速与入口有效性迁移。
- [x] 原第9项导航生产端固定1.1.20 patch/消息/接收接线，删除wall映射与撤销原因。
- [x] 原导航范围集中迁移验证：最终335项通过；完整ROS构建/ABI/实车未验证。
- [x] 用户扩展第9项后的全repo消费者审计与契约建议，见`docs/reviews/2026-11-16-control-clock-system-audit.md`。
- [ ] 负责人亲审系统方案及外部采集producer时间契约后，实施新增学习/估计/感知/SLAM时间链。
- [ ] 扩大范围全部完成后的集中验证与最终亲审交付；当前不得将第9项标为整体完成。
