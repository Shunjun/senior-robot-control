# 启动学习流式解耦 Implementation Plan

> 执行方式已指定：原 gpt-6.1-sol／medium worker 在原隔离分支顺序实施，不创建 helpers。本轮仅更新两份授权文档，负责人亲阅后再安排产品。

**Goal:** 接收完整原始流、独立 CPU 串行学习；Core 首次成功输出后固定三秒运动，录完后接管不取消后台保存。

**Architecture:** LearningService 只做控制面、有限记录和收尾事实；新有状态子进程独占 SelfReturnModel，原单写 owner／I/O worker 落盘。Core 不等待计算，控制仍只消费最新轻量快照。

**Tech Stack:** 当前 Python、ROS 2、Shapely、SOCK_SEQPACKET、spawn 与已有共享槽／生命周期工具，不加依赖或 ROS 节点。

**Spec:** [同 run 学习职责与保存设计](../specs/2026-11-16-learning-shared-state-design.md)。

## 约束与审阅重点

- 固定功能主分支 `fix/ros-typesupport-export-order`，原隔离分支正常 merge，不 cherry-pick、不推 main。当前部署 `61555f8`；`5b65286` 仅保存未合未部署，产品阶段再决定普通过期帧语义，不能把它当最终解耦。
- 0.05 m/s、成功提交起算三秒、原 150 ms／scan TTL、1 cm／离散连接／固定交集、70%／+Inf、尺寸及单次误差不变。后台不重新按计算时刻 freshness 扔掉合法历史帧。
- 录帧从 `_on_scan` 准入后、补点前直送，不从 latest 拼历史；发送 16／服务 128／flight 4 是工程内存界，不是运动资格。
- 只允许后台 CPU 排空；无 Core 同步 fallback、兼容 frame、整模型逐帧 pickle、隐藏全局缓存或后台 raw 文件 spool。
- capture-complete、compute-complete、durable 分开。Core 到点让权；已录全后正常手动接管仍计算保存，封闭前接管不倒填结果。跨通路先后按采集／录入和控制事实判，不按消息到达顺序。
- 不承诺程序崩溃后恢复所有 raw；失败／关闭不重启动作。网页新校准命令另行只读收口，本阶段不擅自定义。

## 准确文件清单

| 类别 | 新增 | 修改 |
| --- | --- | --- |
| 产品（8） | `src/senior_control/runtime/learning_compute.py` | 同目录 `learning_service.py`、`learning_client.py`、`learning_wire.py`、`learning_owner.py`；`src/senior_control/motion/startup_learning.py`；`ros2/src/senior_robot/senior_robot/safety_node.py`、`motion_core_output.py` |
| 测试／夹具（11） | `tests/unit/test_learning_compute.py` | 同目录 `learning_test_support.py`、`test_learning_client.py`、`test_learning_service_linux.py`、`test_learning_record_owner.py`、`test_startup_learning_chain.py`、`test_startup_learning.py`、`test_learning_dual_clock.py`、`test_startup_learning_latch.py`、`test_startup_learning_review.py`、`test_startup_learning_cycle.py` |
| 文档（2） | 本计划 | `docs/superpowers/specs/2026-11-16-learning-shared-state-design.md` |

共 21 项目文件；配置／构建／工具新增修改 0、删除文件 0。根 CURRENT_TASK 由负责人同步，另计 1，总预报 22。不改 self_learning 算法、learning_store／record schema、ProcessResources、ControlInputs、补点／测速、视频／launch 或厂家接口。

## 阶段 1：产品通路与职责一次收敛（仅负责人安排后实施）

### 原始流与固定接口

- [ ] `learning_wire.py` 定义原始 `FrameBatch`、连续 ACK／frontier、窗口及收尾事实；范围有 run／attempt／segment、原 capture、传输 index、recorded_steady_ns、可逆非有限 range。取消／窗口控制只走有序控制 FIFO；数据 ACK 不携整模型。
- [ ] `learning_client.py` 实现 `LearningFrames.offer(frame)` 非阻塞有界入队，发送线程只编码／IPC；原 LearningClient 保留预留、动作报告与只读结果，删除 Core 发点集完成的通路。
- [ ] `safety_node.py` 在原准入成功、补点前调用 offer；原控制快照不增历史。初始 5 预帧仅计真实接纳记录，不等 observe。

### 有状态 CPU 与原单写 owner

- [ ] 新 `learning_compute.py` 固定 typed batch／结果，单 flight ACK，子进程常驻 model。原 helper 管父死清理／有界退出；不改通用 ProcessWorker、不占原磁盘 worker。
- [ ] `learning_service.py` 持 128 帧 FIFO、数据完整性及窗口归属；单 flight 最多 4，按原采集时刻流式交计算。start／验证 marker 晚到时有限暂留未定归属数据，不能丢中间帧。
- [ ] `learning_owner.py` 单写 capture-complete／终态和结果接受。服务根据连续录入 frontier 及一次轻量控制／录帧收尾事实消除取消与 batch 重排；owner 不以当前 Core mode 或 compute-ready 许可来决定已录完数据是否可保存。
- [ ] 对规格列出的两个顺序逐项自审：完整先发生但取消先到，仍保存；接管先发生但 batch 先到，未收尾前不能提交，接管后帧不能补全。封闭后普通手动接管不抹掉窗口，计算结果仅匹配原 attempt。
- [ ] 结果沿原 I/O worker 原子提交，durable 前不安装 view；overflow／失败不造模型，不 clear scan，不阻塞正常输出。关闭不增加 raw spool 或动作重试。

### MotionCore 移除重计算

- [ ] `startup_learning.py` 只保留轻量三秒动作、预留／真实提交起点、停止／接管事实及后台采集收尾观察；移除同步 frame／候选／validate。
- [ ] `motion_core_output.py` 只用当前纯感知／过滤和原方向限速；到期零输出并退休学习运动目标，不为了后台计算或验证重新登记零速 startup task。
- [ ] Core 让权后仍能提供该 attempt 的已发生生命周期事实，不能以 mode 不再 learning 拒绝记录收尾；未收尾前取消立即控制停止，后台不延长原窗口。
- [ ] 亲查 8 产品 diff，做必要语法／导入／格式检查并提交准确产品 SHA；不为了旧 5b 追加测试或新的验收轮次，不把 profile 作为产品前置。

## 阶段 2：负责人非运动核对与现场优先

- [ ] 负责人亲阅实际产品 diff，正常 merge／push；用本次 54 帧候选原始 JSON 检查数据顺序／窗口边界及隔离，不能宣称原 Core 选帧已经还原。
- [ ] 先独立无运动核对控制面响应与 CPU 子进程隔离；后台慢／积压不能让三秒结束和手动接管等待。无新 benchmark 文件，不调 GC／TTL 或频率。
- [ ] 任何物理新学习都由负责人重新协调用户状态／授权后串行操作；worker 不 SSH／启服务／控车，不自动再次尝试当前 failed run。
- [ ] 如安排真实验收，先被动查停车与雷达，再记录运动／停车、phase／reason／action、capture／compute／durable进度及真实模型点数／车体分布。未完成或 0 点须如实说明，不把候选等同障碍物数量。

## 阶段 3：负责人选定后迁移必要定向回归

本轮仅候选，不先写测试。候选按下表真实场景分别实现，不展开参数矩阵、不用单个测试塞入大量排列，不新建端到端／benchmark 脚本，不全 suite。使用真实生产 kernel、服务／时钟与既有 I/O 替身；不镜像学习算法。

| 候选 | 真实需求、输入与预期 | 拟复用位置 |
| --- | --- | --- |
| C1 新增：慢 CPU 与固定运动解耦 | compute flight 保持未完成，接收继续记录；真实 Core tick 仍成功起算、三秒截止归零并让权，不等队列排空 | `test_startup_learning_chain.py`，真实 Core 与现有 LearningRig／时钟 |
| C2 新增：完整流与采集边界 | 单条有序时间线含窗口前、内、后帧与重复 batch；Core latest 已覆盖中间帧，计算仍恰好收到合法原始前缀，按采集时刻选窗，保留非有限标签／身份 | 新 `test_learning_compute.py`；已有 capture／sample_time 构造 |
| C3 新增：录齐后手动接管 | 三秒与 5 验证帧已录齐，CPU 未完成；手动立即生效，释放计算后仍持久化原 attempt 模型，无第二次 Reserve／运动 | `test_startup_learning_chain.py`、原 owner／I/O 替身 |
| C4 新增：取消先到而数据已录齐 | 第 5 帧在接管前录入，其 batch／ACK 晚于取消；连续 frontier 与收尾事实确认后封闭并保存，不查询当前 Core source | `test_learning_service_linux.py`，真实双通路 IPC／spawn；kernel 单元场景放新文件 |
| C5 新增：接管前未录齐 | 原 4 验证帧已录，第 5 帧接管后才录，即使 batch 比取消先到也不得补全；收尾前没有模型发布，动作事实保留 | `test_learning_record_owner.py`／新 compute 用例；同一个相反顺序独立场景 |
| C6 新增：有限记录不伪造完成 | 服务 FIFO 容量边界发生真实缺口，控制继续原周期，最终无模型；只测一次溢出，不铺 transport 故障矩阵 | `test_learning_client.py` 与新 compute 文件，使用实际容量边界 |
| C7 迁移：身份／动作与时间 | 复用现有同 run 重建、首次提交起点、三秒期限、原采集轴和接管断言；后台帧／失败不能重启已消费动作 | `test_startup_learning.py`、`test_learning_dual_clock.py`、链测试 |
| C8 迁移：不可变保存／过滤 | 保留原 durable 后模型交付、只读无 Attach／Reserve、1 cm 与固定连接结果；新同输入 kernel 与原 model 输出相同，慢完成不改变参考 | `test_learning_client.py`、`test_learning_record_owner.py`、新 compute 文件 |

11 文件的性质明确区分：`learning_test_support.py` 仅共用夹具／新通路 API 适配，不新增场景；`test_startup_learning_latch.py`、`test_startup_learning_review.py`、`test_startup_learning_cycle.py` 仅迁移仍有效断言／退役同步 Core 历史或旧几何调用；`test_startup_learning.py`、`test_learning_dual_clock.py` 迁移已有时间／身份断言；真正新增行为集中在 compute、Core 链、client、owner、Linux service 五个位置，按 C1～C6 选择最少独立场景，不在每层重复全部测试。

- [ ] 负责人选定候选后再实施上述 11 文件迁移；删除与新职责冲突的同步重算断言，不削弱急停／身份／原 1 cm 算法，也不为了旧调用保兼容生产 frame。
- [ ] 只运行有实质变更的必要定向文件／用例，记录准确命令、版本、通过／失败／跳过；Linux-only 结果不能由 macOS 替代。
- [ ] 自审与提交后交负责人亲阅，保留已充分证据，不重复审计／全回归。网页重新校准入口仍须另行收口产品／测试范围后才进入实现。
