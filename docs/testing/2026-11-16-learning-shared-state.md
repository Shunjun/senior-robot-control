# 同run学习共享恢复与3秒动作实施证据

代码修改完成，待负责人亲审；本地定向证据如下，目标Linux/aarch64/ROS与实车尚未验收。未提交、推送、暂存，也未启动ROS、车辆、开发服务。测试仅使用临时目录、mock监督入口及显式测试子进程。

## 版本与范围

分支`fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。本轮30个源码/测试文件冻结清单见[指纹](2026-11-16-learning-shared-state-fingerprints.json)，该清单不包含本报告本身。共享文件可能继续被Nav按已交接hunk修改；清单记录此刻完整文件，不把Nav自有改动算本任务成果。

本任务新增：`motion/learning_record.py`，`runtime/learning_{store,mailbox,owner,wire,service,client}.py`，`safety/learning_view.py`及对应owner/mailbox/目标Linux IPC/3秒动作测试和`learning_test_support.py`。修改StartupLearning生命周期、SelfReturnModel捕获排序与几何分类、matcher规范记录读取、清理真实配套及监督学习组件接线。既有第7项1cm匹配和启动成功后一次清理成果继续保留。

共享hunk边界：`motion_core_output.py`只拥有学习初始化/模型同步与本地过滤/目标生命周期/私有提交成功后的动作计时和3秒裁剪/summary/close；观测年龄、Origin、pose、computed_steady传递是Nav自有。`learning_pose_history.py`仅本轮补构造与add的返回注解，measurement steady迁移由Nav完成。两个supervise只新增LearningService生命周期并保留原一次清理；导航启动命令不属本任务。

实际diff已导出`/tmp/learning-shared-state-head.diff`（完整30文件相对HEAD，包含保留的前次1cm/清理及共享文件Nav改动，不能整体归因本轮）。更精确交接增量：`/tmp/startup-lifecycle-increment.diff`对照Nav时间交接快照，`/tmp/model-increment.diff`对照已冻结1cm模型；新模块源码完整可审。未使用整文件覆盖解决并行冲突。

## 产品行为与职责

run监督进程内唯一LearningRecordOwner持有JSON事实和稳定`.owner.lock`；复用一个spawn ProcessWorker执行完整快照fsync/replace/目录fsync。JSON严格校验schema/run/boot/owner/模型/几何身份、最多128个anchor、64KiB大小、有限值/重复键；读者一次open得到完整旧/新版本。废弃文本claim及新写入`.features.json`协议删除，matcher直接读取规范记录的实际物理边界。清理精确增加`.pending`、`.owner.lock`，保留当前run及仍锁定的旧owner。

LearningClient专用IPC线程使用各16项有界队列、一次一个请求；最终控制只poll本地快照，不等记录IPC/磁盘/云端。Linux abstract SOCK_SEQPACKET、同UID kernel peer credentials、SCM_RIGHTS传递唯一会话fd；最多8个异步peer。旧会话write fd全部EOF或明确封闭后才发新会话，恢复先排空accepted撤销。模型目录冻结，consumer当前帧按1cm与真实车身边界匹配，只减不扩张；失配anchor先发布到非阻塞pipe再返回mask，旧几何/cache前沿不能复活删除。最多128个首次uint32事件，运行核验pipe容量/PIPE_BUF。

增加owner持有49字节memfd（run/model/session绑定+一字节帧状态）封闭“第一次删除write失败后消费者立即死亡”的窗口：匹配前IN_FRAME，全部成功发布后IDLE，异常FAILED，正常关闭CLOSED。生产无磁盘模拟fallback，不逐帧fsync或IPC往返；owner死亡未落盘时恢复作废模型，保留已消耗动作，不能声称恢复丢失事实。accepted与durable分离，慢fsync下正常匹配持续可用，一anchor撤销不禁用其余安全anchor。commit失败局部报告，不确认新动作/新模型，不自动无界重试，主Agent不因此主动下线。

动作在最终已选合格非零学习输出时申请一次持久预消耗，确认前本轮零输出；首次成功私有提交记录t0，学习输出期限裁剪为t0+3秒，超时idle旧轨迹也不续走。控制消费者恢复不重放已发放或不确定是否输出的动作；可靠且明确从未发放时可继续首次学习。计时记录是本地成功提交有效区间，不是物理执行ACK。删除精确15cm/固定8cm双门、累计偏航/反向整次中断、模型首尾0.05rad拒绝、30秒预算及旧专用状态链；不存在改名保留的30秒门。

反馈短缺、积分断档、generation/source变化开新证据段，不拼未知位姿、不积分假定目标速度。分类使用真实SE2和保守独立位姿误差支持、完整自由空间与世界固定反假设，沿原最少采样契约；3秒结束不是成功证据。捕获排序去重独立于ROS标签，显式SourceTime identity+measurement steady；迟到帧仅在实际历史被丢弃后才受新下界限制。实际车内候选先筛选再限制128，外部点数量不使孤立车内点在任意stride中丢失。

流程和等待：监督async owner约10ms轮询 → 单独文件I/O进程；control → 专用IPC线程 → owner；control当前帧本地filter → 非阻塞pipe+memfd → owner。每个IPC请求超时2秒，慢peer异步回复最多2秒；退出最多1秒等待最终记录，再按既有ProcessWorker有界回收。控制本轮仍执行本地几何/匹配及kernel write/pwrite，不宣称已实测硬实时上界。owner不承担重几何，不扩大排队的第11项监督连坐改造。

## 集中验证与失败交代

环境：macOS Darwin arm64，CPython 3.12.15，仓库`.venv`。Python3.10语法解析12个核心学习模块通过，不能代替目标Python3.10运行。

第一轮集中全学习组最终记录：`/tmp/shared-learning-final.log/.xml`，110 passed、7 failed、2 skipped（70.57秒）。7个失败均为新增清理测试把函数内import的LearningService误patch到supervisor模块，已改为patch真实定义模块，不改产品行为/场景/断言。补测`/tmp/shared-learning-final-followup.log/.xml`：58 passed、3 failed（1.26秒）；包含全部清理、学习review和几何通过，额外三个共享core时钟fixture失败详情见下。新增存储故障/当前run新配套/旧活跃writer等定向补充后：`/tmp/shared-learning-storage-final.log/.xml`，35 passed、2 skipped（2.45秒），包含先前7个清理失败的实际修复证据。

完整学习组命令：

```sh
.venv/bin/pytest -q tests/unit/test_startup_learning.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_latch.py tests/unit/test_startup_learning_review.py tests/unit/test_startup_learning_timed.py tests/unit/test_body_return_geometry_only.py tests/unit/test_self_return_process_revocation.py tests/unit/test_learning_record_owner.py tests/unit/test_learning_revocation_mailbox.py tests/unit/test_learning_service_linux.py tests/unit/test_learning_cleanup.py --junitxml=/tmp/shared-learning-final.xml
```

失败修复及额外core边界补测：

```sh
.venv/bin/pytest -q tests/unit/test_learning_cleanup.py tests/unit/test_motion_core_handoff.py tests/unit/test_motion_core_chain.py tests/unit/test_startup_learning_review.py tests/unit/test_body_return_geometry_only.py --junitxml=/tmp/shared-learning-final-followup.xml
.venv/bin/pytest -q tests/unit/test_learning_record_owner.py tests/unit/test_learning_revocation_mailbox.py tests/unit/test_learning_service_linux.py tests/unit/test_learning_cleanup.py --junitxml=/tmp/shared-learning-storage-final.xml
```

补测三个core失败：`test_motion_core_handoff::test_body_projection_preserves_original_hard_expiry_under_slow_rtf[ros_age/steady_age]`合成Origin缺measurement_steady；`test_motion_core_chain::test_native_recovery_then_intent_and_local_release_are_actually_consumed`合成scan缺measurement_steady。已交Nav，msg_65b02afb6536；Nav回执msg_aba9bf220c5c已迁实际字段，并将旧“ROS校时导致过期”断言迁为源steady到期边界，待其整体同版验证。本报告不把该回执当测试通过、不改Nav自有fixture来掩盖失败。

本轮验证中另修真实问题：未丢弃帧却推进历史下界导致同段迟到被拒；实体几何变化时client明确关闭旧能力再rebind，避免旧模型通道故障/失效；世界反假设自由支持的扫描角覆盖改为真实方形角点范围而非更大的外接圆（仍核验整个支持，原位姿预算与缺束反例保留）；未开始动作等待不能提前消耗资格；超时idle学习尾部不能沿旧轨迹输出。没有放宽传感器期限/配置误差/ranges。

覆盖包含：严格记录/原子替换失败/非项目文件符号链接保护/重复writer；durable动作确认、同run消费不重放、未移动继续、完成恢复；慢提交accepted单向撤销、旧磁盘/旧回调不能复活；128事件容量、重复fd关闭、EAGAIN/短写、子进程os._exit帧中死亡/发布后死亡；2000帧连续1cm噪声匹配不依赖owner drain或fsync、部分撤销其余锚点继续；3秒到停/无30秒/微转弯与短暂反向不整次中断；捕获steady随ROS前后跳排序、迟到/重放/跨段；世界固定、未知/缺束与外部障碍负例。

## 静态与冻结

12个学习源码严格mypy：`Success: no issues found in 12 source files`，日志`/tmp/shared-learning-mypy.log`。命令为`.venv/bin/mypy`依次检查motion/learning_record.py、motion/startup_learning.py、runtime/learning_store.py、learning_mailbox.py、learning_owner.py、learning_wire.py、learning_service.py、learning_client.py、learning_cleanup.py、safety/learning_view.py、self_learning.py、external_motion_features.py。没有Any泛滥、ignore或配置降低。

Ruff check同上产品范围及learning测试范围通过（`/tmp/shared-learning-ruff.log`）；Ruff format --check为27 files already formatted（`/tmp/shared-learning-format.log`）。共享core文件只调整本任务新增表达式排版，不重排Nav CRLF等无关文件。三保护文件相对任务前SHA256全部UNCHANGED；工作区原有这些文件脏态不是本轮改动。无暂存、提交或推送。

## 未验证边界

两个明确跳过：Linux实际memfd/pipe容量测试、真实LearningService+LearningClient+spawn I/O+SCM_RIGHTS端到端测试。当前macOS没有目标能力；纯测试使用临时文件模拟memfd读写契约，不是产品fallback，不能声称Linux故障注入通过。真实Linux消费者kill/进程重启/慢fsync/介质故障及ROS输出deadline→串口实际执行需部署验证。当前ROS原生输出接口迁移由Nav继续同版验证，物理watchdog参数未新增也未宣称验证。厂商测量proof/实际独立位姿误差与不可辨识车内外结构同样不是理想数据测试能消除的边界。

额外验证真实LearningClient线程（测试内transport适配到真实owner，未模拟客户端状态机）：` .venv/bin/pytest -q tests/unit/test_learning_client.py --junitxml=/tmp/shared-learning-client-final.xml`，2 passed（0.35秒）。覆盖首次消费→完成提交→能力rebind、当前帧撤销→同run新消费者不复活，以及真实几何变更主动清洁关闭旧fd/恢复新会话；该证据不代替Linux SCM_RIGHTS原生链。首轮该补充有一处预期reason字符串写错，按owner真实`learning_geometry_changed`契约修正后通过，未改产品或放宽模型断言。

冻结清单SHA256：`3201ac2d9872a1042b4b270cd89c34e374cc8802db777d9070d16dc126c3829f`；HEAD对照diff SHA256：`abb14155ea80ddd70b0f3013a85f59118ca4bffcf73a740fb7389237260769c6`。

按JUnit node id合并本任务学习范围、取最后一次结果（排除Nav共享core扩展场景）：{'passed': 122, 'skipped': 2}；这是多次有原因定向补测的去重汇总，不伪称单次全绿运行。

后续双域适配：上述measurement steady单域学习视图已按Nav显式物理tick接口进一步迁移，真实增量与新证据见`docs/testing/2026-11-16-learning-dual-clock.md`；本报告旧122/2证据不能代替新接口验证。
