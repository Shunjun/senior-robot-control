# 共同积分历史与乱序配对返修

当前状态：负责人已亲读R1–R3增量，主要修正已到位，继续累计审阅发现P1/P2两项回归；本轮以 `28d0635a561a176a52fba70596707a0a815dbc28` 为审阅基线追加修复，先提交 `58677cb931753c02472fa261a1c91b0d6509534e`，再追加原过滤谓词边界补充；没有改写原提交。累计100文件审阅仍未完成，本次提交待亲审，不声明ROS或整车验收。分支及工作区保持 `fix/degraded-drive-speed` / `/Users/zhangshunjun/workspace/orca/senior-robot-control/fix-degraded-drive-speed`。

## P1：计算相对区间，抵消共同积分前缀

原实现同anchor相加两个从长期起点累计的prediction_error，确实将共享历史重复加入最近scan到当前的对齐。新实现不减小参数，也不周期清零完整姿态误差。

- PoseEstimation保留最多256段积分历史，段内记录实际使用的motion、积分速率、缺轴/过期区间标记。PoseEstimate附加仅本owner/路径有效的integration_path；同anchor且路径有证据一致时，在查询 `[old.stamp,current.stamp]` 内重新累计误差，起点为旧车身参考系。共同前缀作为同一左侧SE(2)变换抵消，区间内偏航误差仍影响后续位移；没有用全局累计标量相减。
- 完整pose的全局prediction_error照常累计；原始来源时间、质量、共享0.3/1秒策略不变。路径仅在原有真实坐标/clock重置位置撤销，不因运行时间或误差大小重新起锚。
- 段内历史pose查询使用该段原先的积分速率，不能用当前新速度改写旧轨迹，因此前后pose确实是同一积分路径的两个端点。中点查询、转弯与端点速度突变有单独测试。
- 真正缺轴/硬过期覆盖的区间使用原有unknown_speed/unknown_yaw软件域，未知误差保留None；还计入非零截断名义路径与未知运动的差。恢复后的新鲜局部区间不再继承过去缺轴之前的整条累计误差。
- 不同anchor、无同路径证据仍使用保守端点政策；历史窗口不足则只对查询中缺失的区间给软件模型预算，不把全程误差塞入短区间、不新增年龄拒绝。
- 新256段是有界历史，不是等待任务队列；现有输入队列未扩容。retained_samples及DEBUG历史capacity同步计入这256段，声明上限从1280到1536；原姿态256和原始pose各源128的容量没有扩大。

同样的最近0.05秒区间，软件初始参数 `.02 m/s`、`.02 rad/s`、lateral drift `.03 m/s`，精确输入 `(0.2,0,0)` 的结果：

| 已运行 | 相对位置预算m | 相对角度预算rad | 完整姿态累计位置预算m |
| --- | --- | --- | --- |
| 0.15s | 0.00201 | 0.001 | 0.00606 |
| 10.05s | 0.00201 | 0.001 | 0.60501 |
| 60.05s | 0.00201 | 0.001 | 9.62001 |

完整误差没有被重置。另有vy缺测、0.4rad/s转弯同区间对照，以及实际MotionSensorRuntime→`_observation`→space输出在10/60秒的预算/自由空间一致断言；这里的“实际”指真实软件函数和合成ROS形状输入，不是ROS运行或物理验收。完整数字及retained数量在 `2026-11-15-relative-interval-results.json`，复算入口为 `test_relative_interval_regressions.run(duration,yaw=...,lateral=...)`。

## P2：恢复全deque消费，保留原入队统计

RgbdSynchronizer的offer没有单调输入前提，原while队首算法确会遗漏后面的matched帧。现在反向遍历完整有界deque：消费matched对象一次，删除所有更早及重复stamp槽，保留所有更新帧的原顺序及原enqueue时间。未增加单调门或来源拒绝。

DiagnosticDeque新增take_index，和discard_index区分真正消费/旧帧丢弃，不重建deque或重新入队。RGB先到/Depth先到、乱序、二次重复配对、未消费新帧、相同对象重入以及pending/drop/oldest_wait统计均覆盖。输入容量及原max_skew规则不变。最终核对还恢复了原函数严格 `stamp > matched` 的保留谓词：NaN不能被当成更新帧残留；该边界先复现pending=2而应为1，再修正并覆盖。

motion_core_output相邻注释已统一为footprint半径，移除farthest observed range的矛盾表述。

## 验证与范围

- 新用例先得到 **10 failed,1 passed**，复现P1/P2；后续补齐精确三分量、源切换边界、vx/wz真实缺失及恢复、部分到期仍有非零名义积分等覆盖。
- 最终受影响运动/感知/配对/core组合组 **201 passed**，包含20项新增回归。中途199项及后补精确三分量时19项通过记录保留在verification历史检查点；随后新增NaN过滤谓词生产边界修正，才重新验证本轮受影响组，不重复累计测试数；精确argv、父版本、生产源码/测试指纹和stdout见 `2026-11-15-relative-interval-sync-verification.json`。
- 首轮受影响组为197 passed/1 failed，失败来自上一轮人为固定“捕获近墙记录必须无许可”的测试期望。P1修正后该软件低速候选可证明通行，因此改为验证速度误差仍保留、正确局部对齐可形成许可，并用显式障碍覆盖验证仍拦截。没有删掉障碍保护、清零R3速度误差或压低软件参数来恢复旧结果；上一轮把这一记录固定为负例的结论现被更正，旧执行事实仍保留在阶段报告。
- 初期恢复序号夹具低于原frontier被正确拒绝，已使合成输入序号单调；组合用合成ROS纳秒时钟，避免测试生成的亚纳秒浮点差异。未放松生产replay/clock保护。
- 8个变更Python文件Ruff check/format --check、py_compile、git diff --check通过；3个变更纯源码模块严格mypy通过。未例行重跑不受影响前端/C++构建或全量mypy，既有4个基线类型错误状态见前一报告，不声明已修复。

本轮未修改停车速度域/停车公式、decel、自过滤或排队旧链清理；R1的源级年龄统计及R3的速度证据接线本轮未改。没有启动服务、VM或实车，没有改运行配置、推送、建PR或改main。Linux ROS生成链接、实际DDS及实车验证仍缺，保留Draft限制；软件误差策略仍需真实场景验证，累计审阅由负责人继续。
