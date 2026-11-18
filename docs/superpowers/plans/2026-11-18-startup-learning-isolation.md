# 启动学习流式解耦 Implementation Plan

> 执行方式已指定：原 gpt-6.1-sol／medium worker 在原隔离分支顺序实施，不创建 helpers。已在原隔离分支实施产品与选定定向回归，尚未合入/部署；现场验收由负责人安排。

**Goal:** 接收完整原始流、独立 CPU 串行学习；Core 固定三秒运动；网页可显式开启新 attempt，录完后接管不取消后台保存。

**Architecture:** LearningService 只做控制面、有限记录和收尾事实；新有状态子进程独占 SelfReturnModel，原单写 owner／I/O worker 落盘。Core 不等待计算，控制仍只消费最新轻量快照。

**Tech Stack:** 当前 Python、ROS 2、Shapely、SOCK_SEQPACKET、spawn 与已有共享槽／生命周期工具，不加依赖或 ROS 节点。

**Spec:** [同 run 学习职责与保存设计](../specs/2026-11-16-learning-shared-state-design.md)。

## 约束与审阅重点

- 固定功能主分支 `fix/ros-typesupport-export-order`，原隔离分支正常 merge，不 cherry-pick、不推 main。当前部署 `61555f8`；`5b65286` 仅保存未合未部署，产品阶段再决定普通过期帧语义，不能把它当最终解耦。
- 0.05 m/s、成功提交起算三秒、原 150 ms／scan TTL、1 cm／离散连接／固定交集、70%／+Inf、尺寸及单次误差不变。后台不重新按计算时刻 freshness 扔掉合法历史帧。
- 录帧从 `_on_scan` 准入后、补点前直送，不从 latest 拼历史；发送 16／服务 128／flight 4 是工程内存界，不是运动资格。
- 只允许后台 CPU 排空；无 Core 同步 fallback、兼容 frame、整模型逐帧 pickle、隐藏全局缓存或后台 raw 文件 spool。
- capture-complete、compute-complete、durable 分开。Core 到点让权；已录全后正常手动接管仍计算保存，封闭前接管不倒填结果。跨通路先后按采集／录入和控制事实判，不按消息到达顺序。
- 不承诺程序崩溃后恢复所有 raw；失败／关闭不重启动作。learning.start/status、同请求幂等／忙时不排队、新 active durable 前保旧，按规格实现；不隐式抢占行驶／导航或恢复旧目标。

## 准确文件清单与临时预览

原流式 21 文件继续复用：

| 类别 | 新增 | 修改 |
| --- | --- | --- |
| 原产品（8） | `src/senior_control/runtime/learning_compute.py` | 同目录 `learning_service.py`、`learning_client.py`、`learning_wire.py`、`learning_owner.py`；`src/senior_control/motion/startup_learning.py`；`ros2/src/senior_robot/senior_robot/safety_node.py`、`motion_core_output.py` |
| 原测试／夹具（11） | `tests/unit/test_learning_compute.py` | 同目录 `learning_test_support.py`、`test_learning_client.py`、`test_learning_service_linux.py`、`test_learning_record_owner.py`、`test_startup_learning_chain.py`、`test_startup_learning.py`、`test_learning_dual_clock.py`、`test_startup_learning_latch.py`、`test_startup_learning_review.py`、`test_startup_learning_cycle.py` |
| 文档（2） | 本计划 | `docs/superpowers/specs/2026-11-16-learning-shared-state-design.md` |

新入口额外修改 14 产品，新增 0：

- 根协议／路由：`src/senior_control/protocol.py`、`src/senior_control/cloud/relay.py`。
- Agent／接纳：`src/senior_control/agent/main.py`、`connection.py`、`manual_control.py`；`ros2/src/senior_robot/senior_robot/agent_adapter.py`、`command_node.py`。
- attempt／active 引用：`src/senior_control/motion/learning_record.py`；`src/senior_control/runtime/learning_store.py`、`learning_cleanup.py`。
- 网页：`web/src/api/protocol.ts`、`client.ts`、`vehicleState.ts`；`web/src/features/devices/VehicleStateView.tsx`。

额外修改 10 现有测试，新增 0：`tests/unit/test_protocol.py`、`test_agent_ports.py`、`test_learning_cleanup.py`、`test_vehicle_state.py`；`tests/integration/test_cloud.py`、`test_agent.py`、`test_limited_manual_ingress.py`；`web/src/api/client.test.ts`、`vehicleState.test.ts`；`web/src/features/devices/vehicleState.test.tsx`。

原预报项目 45 文件＝产品新增 1／修改 21（22）＋测试新增 1／修改 20（21）＋文档新增 1／修改 1（2）。根 CURRENT_TASK 由负责人同步另计 1，总 46。配置／构建／依赖／工具新增修改 0，删除文件 0；不改 self_learning 算法、ProcessResources、ControlInputs、补点／测速、视频／launch 或厂家接口。

临时预览单独安排：新增 `web/src/features/devices/CalibrationPreview.tsx`、临时修改原 `VehicleStateView.tsx`，共 2 文件；仅前者是最终名单以外的临时文件。用既有 demo 壳及明确 preview 开关在原卡片位置展示同效果交互，无真实 API／socket／车辆；不新增服务／路由／配置。负责人安排后才写；确认后删除临时文件／开关，最终 View 修改仍在上述 45 内。

## 阶段 0：当前文档与原位置交互预览

- [x] 本轮仅自审并提交两文档。当前界面提案、文案、状态布局和临时文件边界以规格最后一节为准，不先实现产品 API 或测试。
- [x] 负责人安排后制作上述临时预览：原位置按钮、busy／已停车计算／旧模型保留／成功切换／超时查询；场景切换工具明确只模拟。无能力／行驶／导航冲突的按钮状态不能掩盖最终 Core 复核。
- [x] 预览亲阅确认后才进入真实 UI 接线；不擅自启动开发服务，也不把模拟按钮当作真实校准已接通。

## 阶段 1：契约与持久化边界（产品须负责人安排后实施）

- [x] 根／TS 协议固定 learning.start/status 及规格的最小 DTO；relay 注入可信上下文、私有请求回执只回原发起方；不改成 latest intent 或默认广播。
- [x] learning_record/store/owner 固定 ProgramManifest 与 AttemptRecord v2、请求确定性 attempt_id、每轮独立 action、active_ref 及两阶段原子落盘；不得重置旧 consumed 来开新轮。
- [x] cleanup 按当前程序组保留 active／candidate／旧 action 记录，不在新校准时删旧模型；不加 raw spool 或无界内存幂等表。

## 阶段 2：流式后端与唯一运动 owner

### 原始流与固定接口

- [x] `learning_wire.py` 定义原始 `FrameBatch`、连续 ACK／frontier、窗口及收尾事实；范围有 run／attempt／segment、原 capture、传输 index、recorded_steady_ns、可逆非有限 range。取消／窗口控制只走有序控制 FIFO；数据 ACK 不携整模型。
- [x] `learning_client.py` 实现 `LearningFrames.offer(frame)` 非阻塞有界入队，发送线程只编码／IPC；原 LearningClient 保留预留、动作报告与只读结果，删除 Core 发点集完成的通路。
- [x] `safety_node.py` 在原准入成功、补点前调用 offer；原控制快照不增历史。初始 5 预帧仅计真实接纳记录，不等 observe。

### 有状态 CPU 与原单写 owner

- [x] 新 `learning_compute.py` 固定 typed batch／结果，单 flight ACK，子进程常驻 model。原 helper 管父死清理／有界退出；不改通用 ProcessWorker、不占原磁盘 worker。
- [x] `learning_service.py` 持 128 帧 FIFO、数据完整性及窗口归属；单 flight 最多 4，按原采集时刻流式交计算。start／验证 marker 晚到时有限暂留未定归属数据，不能丢中间帧。
- [x] `learning_owner.py` 单写 capture-complete／终态和结果接受。服务根据连续录入 frontier 及一次轻量控制／录帧收尾事实消除取消与 batch 重排；owner 不以当前 Core mode 或 compute-ready 许可来决定已录完数据是否可保存。
- [x] 对规格列出的两个顺序逐项自审：完整先发生但取消先到，仍保存；接管先发生但 batch 先到，未收尾前不能提交，接管后帧不能补全。封闭后普通手动接管不抹掉窗口，计算结果仅匹配原 attempt。
- [x] 结果先提交该 attempt，再原子提交 active_ref／版本；两步成功才安装新 view，未变只观察小版本，旧模型保留至切换。overflow／失败不造模型，不 clear scan，不阻塞正常输出；关闭不加 raw spool／动作重试。

### MotionCore 移除重计算

- [x] `startup_learning.py` 只保留轻量三秒动作、预留／真实提交起点、停止／接管事实及后台采集收尾观察；移除同步 frame／候选／validate。
- [x] `motion_core_output.py` 只用当前纯感知／过滤和原方向限速；到期零输出并退休学习运动目标，不为了后台计算或验证重新登记零速 startup task。
- [x] Core 让权后仍能提供该 attempt 的已发生生命周期事实，不能以 mode 不再 learning 拒绝记录收尾；未收尾前取消立即控制停止，后台不延长原窗口。
- [x] Agent／IpcMotion／Adapter／CommandNode 接完整新请求路由，Core 对当前上下文及原期限做最终接纳；不同请求 busy 不排队，行驶／导航冲突不抢占，prepared 停稳可显式开始。核对检查后来源已变的竞态。
- [x] Core 只接纳新 attempt 的一次动作，旧 active 与 candidate 分开，完成／失败不恢复旧目标。亲查产品 diff，做必要语法／导入／格式检查并提交准确 SHA；不为 5b 追加回归，不以 profile 阻塞确定职责实现。

## 阶段 3：真实网页接线与集成交付

- [x] VehiclePort／transport／client 接 accepted/busy 与 status 查询；当前能力、请求关联、类型化 data 校验贯穿链路，不把 accepted 当完成。未知结果先查原 attempt，不自动重发 start。
- [x] 在已确认的原 VehicleStateView 位置实现 UI，复用组件 CSS module；busy 仅禁新校准，普通手动控制不等 CPU。移除临时预览组件／开关，不新增全局样式。
- [x] 明确 learning.v1 只在车端真实支持时宣告；无能力／离线不发送未知命令。query 状态不搬模型点集／历史，不改控制采样率或 TTL。

## 阶段 4：负责人非运动核对与现场优先

- [ ] 负责人亲阅实际产品 diff，正常 merge／push；用本次 54 帧候选原始 JSON 检查数据顺序／窗口边界及隔离，不能宣称原 Core 选帧已经还原。
- [ ] 先独立无运动核对控制面响应与 CPU 子进程隔离；后台慢／积压不能让三秒结束和手动接管等待。无新 benchmark 文件，不调 GC／TTL 或频率。
- [ ] 任何物理新学习都由负责人重新协调用户状态／授权后串行操作；worker 不 SSH／启服务／控车，不自动再次尝试当前 failed run。
- [ ] 如安排真实验收，先被动查停车与雷达，再记录运动／停车、phase／reason／action、capture／compute／durable进度及真实模型点数／车体分布。未完成或 0 点须如实说明，不把候选等同障碍物数量。

## 阶段 5：负责人选定后迁移必要定向回归

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

- [ ] 负责人选定候选后再实施原 11 文件迁移及新增入口的额外 10 文件；删除与新职责冲突的同步重算断言，不削弱急停／身份／原 1 cm 算法，也不为了旧调用保兼容生产 frame。
- [ ] 只运行有实质变更的必要定向文件／用例，记录准确命令、版本、通过／失败／跳过；Linux-only 结果不能由 macOS 替代。
- [ ] 自审与提交后交负责人亲阅，保留已充分证据，不重复审计／全回归。不因文档完成自动启动产品或新的实车学习。


### M 系列与 C 系列合并，不重复场景

| 原入口候选 | 合并的真实需求、输入／预期与位置 |
| --- | --- |
| M1 → C1／C7 | 给 C1 的单次动作接上类型化请求／attempt_id，原三秒与身份断言保留；根 protocol／cloud／agent／agent_ports 做各自 DTO 或转发边界，真正 Core 行为只在链测试验证，不各层复制全流程 |
| M2 → C3／C7 | 在 C3 已封闭但 CPU 忙的同一场景加入另一个 request 返回 busy、同 request 查原轮、Reserve 不增；UI/client 只验证不排队／不自动再发，不另造后台忙排列 |
| M4 → C8 | 将已有 durable／不可变保存场景扩为旧 active + 新 attempt：旧 action 不动，候选或引用保存失败保旧，两步成功才切换；原 owner/client/service 复用。额外 cleanup 文件只新增当前引用组保护断言 |
| M5 → C7 | 在 existing limited_manual_ingress 验证网页认为 idle 后原来源已变化，Core 拒绝校准且原目标不变；cloud 复用可信 generation／epoch 的新类型路由断言，不铺一般安全矩阵 |
| M6 → C8 | 同一旧 view 客户端，新 active durable 后替换一次，未完成保旧、不 Attach／Reserve、不逐帧读文件；只在原 learning_client 中检验模型观察，不再造 matcher 算法用例 |
| M7 → C1／C3／C8 的 UI 边界 | 在既有 client.test／vehicleState.test／vehicleState.test.tsx 检验真实文案／busy／旧模型／查询语义；超时是独立真实场景，查询原 start_request_id，不当作另一个新学习。test_vehicle_state 仅能力／数据投影迁移 |

M3 就是 C3～C5 的录齐前后接管和双通路顺序，不再增重复候选。其余额外 test_protocol／test_agent_ports 有接口适配和必要 M1 断言，原夹具/API迁移仍按原分类，不把一个测试塞几十种参数绕数量。

## 可并行边界与提交顺序

依赖顺序是文档／预览确认 → 阶段 1 的公开及私有契约／record schema → 后端与网页实现 → 集成／负责人验证 → 必要回归。阶段 1 未固定不能让单 attempt 产品与新接口同时各写一套。

契约固定后，网页 4 文件与后端不重叠，可由负责人另行安排并行隔离工作区；原位置预览也可与只读后端准备独立。learning_service／client／wire／owner／store 与 Core 生命周期共享同一 attempt／frontier 语义，应由同一顺序 lane 收敛，不让并行分支各自增加事实或 fallback。测试候选可先选定，但本轮不提前写测试；是否派并行 worker 由负责人安排，本 worker 不创建 helpers。

文档、临时预览、完整产品、必要回归分别提交可说明的阶段；不部署缺一半链路的 start 能力，不为每个小步骤增加现场验收轮次。5b 保留历史但未部署，最终整体产品再决定普通时效语义；主分支仍通过正常 merge 交付，不 cherry-pick 或替换分支。

## 2026-11-18 本机实施记录

契约、后端、网页分别保存为 `c796633`、`eb68eb7`、`445d24f`，收尾/状态关联修正保存为 `45156bb`；阶段提交不独立部署，交付时按完整分支审阅。无同步 Core observe/validate 或模型上传 Complete/fallback，CPU 使用单一有状态子进程，原 I/O worker 继续单写 attempt 与 active manifest。正式 UI 已移除临时预览。

实际使用预报范围的子集：产品新增 1/修改 20，测试新增 1/修改 19，文档新增 1/修改 1，项目 43 文件；`manual_control.py`、`test_vehicle_state.py` 无需迁移。临时组件只作为预览阶段成果并已删除；配置/构建/依赖/工具为 0。

验证执行仅定向文件：13 个学习/协议/端口单元文件，三条入口链路集成边界，三个网页文件；旧有效 1 cm、源身份、急停与时间回归保留。Linux 专属服务用例在 macOS 明确跳过，不把内存 transport/clock 替身或前端 mock 当作实际车辆通过。旧 frame/几何 scheduler/文件等待路径对应测试已迁移或退役，不为旧调用保留生产兼容 API。

待负责人亲阅完整实际 diff，与另一隔离工作区的 launch/Core 恢复正常 merge，随后串行安排非运动 Linux/ARM 核对、实际流与负载验证，以及另行协调的一次物理学习验收；本 worker 不启动任何这些操作。

补充本机直接回归已复现并修正：新 attempt 首先收到空心跳时的全局索引误判；活跃控制客户端完成模型后继续查询新 attempt；已消费手动 attempt 重建不依赖已清除的 proposal；原五个停车验证帧仍逐项关联真实 stationary 事实。最新四个相关文件定向 20 passed，先前相关单元 96 passed/1 Linux skip、入口集成 3 passed，网页 25 passed 加修正后的单项 passed 与 typecheck/build 通过；这些是阶段证据，不是实车验收。

负责人要求同步主分支 `94f07ef` 的 boot echo、急停 revision 恢复与只读学习恢复。先保存当前成果再正常 merge，逐段处理 Core/Command 冲突，保留恢复不自动运动、显式新校准与新 attempt 的边界；未完成该集成前不交付部署。
