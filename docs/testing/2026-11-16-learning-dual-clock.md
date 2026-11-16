# 学习双域产品适配与定向验证

任务`task_d956e63f31a0 / ctx_4fc303996ebd`。本报告只覆盖学习消费者对Nav冻结双域接口的实际适配，先前122通过/2跳过不是新契约证据。分支`fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`；未提交、推送或暂存，未启服务/ROS/车辆。

## 实际增量与职责

本轮独占修改3个产品文件：

- `motion/startup_learning.py`：删除`previous_steady`及无用`previous`，改持有`previous_measurement: MeasurementTime`。只在`measurement_interval`的同轴连续覆盖包含前一个物理端点、end等于当前measurement时积分，并仅积分未消费的尾段；无区间/不连续/异轴时开新证据段，从当前真实观测建立新锚，不桥接未知段。第一次观测不倒推位移。重复底层计算和控制跳槽不能要求消费者恰好消费每一原始子区间，已认证连续跨度可包含上次消费端点；不覆盖该端点时仍分段，不能从零区间猜补运动。不再拿`computed_steady`差分或把host hard_age_s施加到物理dt；实际轴root期限仍由host now核验。相同capture roots也须同时看物理端点，暂停零dt不积位移。`frame()`改用capture.measurement查询历史，world轴变化与producer重启分别分段，迟到旧段按host前沿排除。
- `motion/learning_pose_history.py`：LearningPose明确持有物理measurement与对应measurement_host_steady标签。at接受typed物理坐标及host标签，只同轴/epoch内查询和插值，不外推。暂停同tick可保留多份host状态；精确tick查询选不晚于该capture host标签的最新样本，不让后来的停车结论覆写早先帧。同tick不同pose拒绝混入。host标签按原纳秒粒度比较，避免0.3与0.30000000000000004表示误差造成同一原始纳秒查询失败；不增加物理匹配容差。插值仍保留完整段位移/偏航误差支持。
- `safety/self_learning.py`：旧scalar frame_times视图退役，改为typed frame_measurements；同物理轴按measurement秒/producer sequence稳定排序，保留capture clock/source_epoch/sequence去重和独立host前沿异常核验。retain_recent的丢弃下界同时包含物理坐标与sequence，暂停多个同tick帧被裁剪后，旧身份不能因时间相同重新插入。物理world不同必须新段；producer身份与world身份独立。

模型1cm冻结锚点、真实车身边界、世界固定反假设、外部近障碍/缺束负例、共享owner/mailbox及单向撤销未改。未新增第二时间映射/记录事实owner。没有改共享core调用hunk。

host `now/computed_steady`继续负责控制入口、原始root TTL、等待、segment host下界和validation_after_steady；3秒动作始终按首次成功提交的host t0+3，暂停不延长、不复发。顺带修正已消耗grant的查询契约：动作结束后prepare_action/action_permitted不再返回旧moving grant；既有最终输出deadline保护继续保留，没有改3秒数值或计时域。

采用Nav真实冻结接口：sim物理视图来自显式measurement_tick_ns（0合法）和measurement_epoch，producer的source_epoch/sequence仍独立，不能复用为world epoch；driver_read沿host:+boot/epochNone实际视图。MotionEstimate.measurement_interval及measurement_host_steady直接消费，不从ROS header或多root猜测物理端点。源proof/estimate/pose/JSON生产及解析由Nav修改，本轮不冒领其hunk。

## 测试迁移

学习自有synthetic estimate明确提供物理endpoint/interval/host标签；原真实core场景保持原ranges、几何、误差、期限和正负断言。新增`learning_test_support.simulation_point`仅在测试producer明确标1x物理视图，未改Nav拥有的通用point/helper，也未在产品补盖时间。旧frame_times断言改查frame_measurements.seconds，不留兼容scalar别名。

新增`test_learning_dual_clock.py`20项真实学习类测试覆盖：0.5/1/2x同物理轨迹位移与capture pose一致；2秒物理区间在0.1秒host内合法积分；暂停tick0不积位移且host3秒仍结束；缺区间/非连续区间/world reset分段不重放；新物理tick不能刷新已过期host根；暂停历史同tick不同host状态、不跨world、不外推；真实几何正负例随ROS标签前后校正仍正确；物理乱序、重放、producer restart与world reset、暂停保留窗口后旧身份不重入；driver_read硬件视图保留。不是Gazebo/ROS实机运行证据。

## 集中命令与结果

本轮产品适配和新测试全部完成后才集中运行：

```sh
.venv/bin/pytest -q tests/unit/test_learning_dual_clock.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_review.py tests/unit/test_startup_learning_timed.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_latch.py tests/unit/test_body_return_geometry_only.py tests/unit/test_self_return_process_revocation.py --junitxml=/tmp/learning-dual-final.xml
```

第一次`/tmp/learning-dual-batch.log/.xml`为88 passed、15 failed（2.61秒）；其中学习侧3项暴露上述纳秒host查询与过期grant问题，已修；其余Nav新路径的query未赋值、MeasurementTime JSON不能序列化已按文件边界交回修复。

第二次`/tmp/learning-dual-final.log/.xml`为101 passed、3 failed（62.84秒），当时已写的19项双域用例全过，原真实core周期5组几何场景、1cm/撤销/学习生命周期均通过。仅剩`test_startup_flows_through_real_tick_and_safety_checks[False/True/front]`在Nav严格JSON读取侧未把measurement字典重建成MeasurementTime，交接msg_86bef479827b，Nav按msg_87dbdfe9986d修产品typed解码，不删fixture字段绕过。

只对这3项补测（不重复已过101项）：

```sh
.venv/bin/pytest -q tests/unit/test_startup_learning_chain.py::test_startup_flows_through_real_tick_and_safety_checks --junitxml=/tmp/learning-dual-chain-final.xml
```

补测结果与最终静态/指纹见文末冻结记录。

## 环境与边界

Darwin arm64、CPython3.12.15、仓库.venv；非目标aarch64 Linux/ROS Humble/Python3.10。mypy保持项目strict，未使用ignore/降低配置。真实producer仿真速率、暂停Gazebo、ROS/串口执行链、Linux原生owner IPC与崩溃注入需Nav最终全域回归及目标部署验收；本轮合成物理proof与真实学习几何测试不能冒称这些目标通过。三保护文件保持任务前指纹，原工作区脏态保留；架构重构仍在用户合PR后，本轮未搬目录。


## 冻结记录与最终结果

上述仅补3项的第一次`/tmp/learning-dual-chain-final.log/.xml`仍3失败（8.76秒），进入真实链后长期停留sampling。诊断插件仅写在`/tmp/learning_dual_debug_plugin.py`，证据`/tmp/learning-dual-chain-debug.log`：host10.025时，学习上一物理端点9.975，当前10.0，生产interval却被重复estimate覆盖为[10.0,10.0]，无法证明过去区间，学习正确分段而每轮只留1帧。未把精确断言改弱或放宽连续性猜补运动。

与Nav按msg_9c1f80fee604/msg_21cf3573327d明确契约：producer只在原host期限允许、同物理轴且真实连续时保留已对齐覆盖起点；重复endpoint不缩小；expiry/无有效观测/epoch变更重开。学习只消费这个连续覆盖中previous→endpoint的尾段，不重复积分、不对未覆盖部分推断。新增第20项明确覆盖控制跳槽消费尾段，原未知gap负例仍在。

最终学习源冻结版本的必要定向检查：

- `/tmp/learning-dual-pure-final.log/.xml`：20 passed，0.58秒，命令`.venv/bin/pytest -q tests/unit/test_learning_dual_clock.py --junitxml=/tmp/learning-dual-pure-final.xml`。
- `/tmp/learning-dual-chain-covered.log/.xml`：3 passed，8.55秒，命令同上3项链补测，XML改为`/tmp/learning-dual-chain-covered.xml`。
- 因连续覆盖生产契约改动会影响实际积分/完成路径，额外只复核5组完整周期，不重复其余已过测试：`.venv/bin/pytest -q tests/unit/test_startup_learning_cycle.py --junitxml=/tmp/learning-dual-cycle-covered.xml`，`/tmp/learning-dual-cycle-covered.log/.xml`为5 passed，54.45秒。

最终源码针对性28项通过；各有原因批次按JUnit节点取最后结果共105 passed、0 failed，属于去重证据汇总，不能伪称单次同环境105全绿。先前旧122/2证据不纳入此新域通过数。Nav仍在完成其全域producer/执行端迁移，本报告不能替代其最终全系统锁版回归。

静态命令`.venv/bin/mypy src/senior_control/motion/startup_learning.py src/senior_control/motion/learning_pose_history.py src/senior_control/safety/self_learning.py`：Success: no issues found in 3 source files，日志`/tmp/learning-dual-mypy.log`；Ruff check通过，format --check为10 files already formatted，日志`/tmp/learning-dual-ruff.log`和`/tmp/learning-dual-format.log`。10个自有产品/测试文件按Python3.10 grammar解析通过，范围git diff --check通过，三保护文件对原任务前指纹未变，无暂存项。

自有修改清单：上述3个产品文件；`tests/unit/learning_test_support.py`、`test_startup_learning.py`、`test_startup_learning_review.py`、`test_startup_learning_cycle.py`、`test_startup_learning_latch.py`、`test_startup_learning_timed.py`及新增`test_learning_dual_clock.py`。另新增本报告/冻结清单，并在此前接口建议、共享恢复报告追加指向本轮证据，保留历史内容。无共享core产品hunk修改。

实际3产品增量对照任务起始快照：`/tmp/learning-dual-product-increment.diff`（133新增/66删除），不是相对HEAD中夹杂先前完整生命周期/1cm的合并diff。快照在`/tmp/learning-dual-before/`。完整最终源码与Nav依赖快照SHA256见`docs/testing/2026-11-16-learning-dual-clock-fingerprints.json`；自有文件冻结，Nav依赖仅记录本次实际检查快照，不冒称替其冻结全部并行工作。

冻结清单SHA256：`307e5864930193fabf1ff07ea87411da7f29910b0b32a9ec17539228468fa2b6`；3产品实际增量diff SHA256：`0379897326a8956fcdf84cde4eeafdc1d04b4cef0e183117da636e20a847fb39`。

后续返修：本轮常速coverage消费未证明变速跳槽积分正确，已在`docs/testing/2026-11-16-learning-recorded-path.md`以真实有界分段路径/原始SE2区间增量替换；本文原先28/105证据不代表该缺陷当时已解决。
