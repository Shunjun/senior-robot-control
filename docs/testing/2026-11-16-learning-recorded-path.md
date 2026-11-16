# 学习跳槽分段积分返修证据

代码修改完成，待负责人亲审。本报告对应`task_54c39bc5b1a4 / ctx_b961930eda71`，分支fix/degraded-drive-speed，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。未暂存、提交、推送，未启ROS、车辆或开发服务；全部验证使用本地临时目录、实际非ROS类和既有mock接线。

## 根因与原探针

实际复现：host10/tick0/vx.05，10.1/.1/vx.05，10.2/.2/vx0，控制逐次消费pose_x=.005，跳过中间一次pose_x=0，而position_error相同为.000282842712474619。旧MotionEstimate.measurement_interval只提供连续覆盖，不能把最新endpoint速度扩展到整段。先前常速跳槽和双域证据未发现此问题，不能继续当作变速正确性证明。

## 真实产品增量

本worker唯一产品修改为`src/senior_control/motion/startup_learning.py`；Nav原owner拥有estimator/pose/runtime/core及共享接收fixture，未互相覆盖。Nav在既有MotionEstimation接收owner、原历史淘汰前记录至多256段MotionPath，经现有有界snapshot交接，不新建运动估计owner/进程/无界队列。

MotionEstimate.path包含path_id、真实物理endpoint、原host标签、原始分段及当前fused/independent轴。每段保留自己的MeasurementInterval、两端原host标签、三轴与误差/roots。真实ScanMotion/RGBD/JSON body delta还保留MotionObservation.measurement_delta及完整原interval；恰好覆盖原段时MotionPathSegment.independent_delta是该原始SE2增量。没有原delta的point段才用producer明确的上一已认证对齐rates作ZOH，并保留既有acceleration预测支持；不把point事后装成精确delta。

StartupLearning不再用当前independent endpoint rate乘累计覆盖尾段。首次观测选局部原点，随后path.tail(previous)逐段返回未消费的真实记录：有independent_delta就直接compose原SE2；否则仅对该原始分段执行exponential。每段独立原误差累计，转向误差对后续平移的支持也逐段传播，历史query因此不受控制读取频率改变。没有抬配置误差、放宽断言或要求控制逐帧到场。

历史段不按今天TTL重新否定；当前path必须仍有真实有效输入，当前独立资格单列，不能用它抹去先前认证段。缺独立段、换代、物理轴变化或淘汰的未知前缀只开新局部段；完整保留尾段可继续，不拼未知前缀、不据末速补齐。当前独立轴缺失不伪造验证成功；原host root期限、3秒单次动作/预消耗防重放、1cm模型与共享owner/mailbox均保持。过去新增历史点不回填今天wheel/IMU静止状态，最新endpoint仍只使用实际轮速/IMU判定，未用独立速度冒充静止证据。

实际必要Nav修复依赖（单独归原owner）：同版typed JSON measurement透传；仍有有效body区间时wheel/IMU/GPS快采样不切碎body区间，但同body源明确负观测必须推进资格变化；独立ZOH根期限必须覆盖本段end_host；未知段保留为independent_axes=None而非抹去前后已知历史；raw measured body delta直接保留，不误用ZOH预测。契约回执见msg_524f65722442、msg_1d8c7f772972、msg_f534328f2967。

## Fixture契约迁移

新`MotionFeed`明确使用真实MotionEstimation.ingest/estimate，接收每个原始样本；控制dense/sparse只选择消费哪些不可变快照。没有隐藏全局history或在产品猜补字段。旧endpoint-N仅可代表无记录历史，不能作可积分路径。

原完整plant fixture的v就是上一实际输出在刚结束区间内的实际速度，现补原MeasurementInterval及Pose2D(v*.05,0,0)，保持原ranges、误差、场景与1e-8位置断言。原理想verified-side同样由明确synthetic measured_intervals源生成已结束区间的真实SE2和两端原roots。point情形仍用原预算；没有为通过正例把point误差改小或假称任何实车精度。

过期负例迁到真实producer契约：两个捕获host分别10和10.001（同一host前沿的新原始样本会被正确拒绝），新物理tick100仍在11.011超过其原deadline11.001后不可使用；测试不通过新tick刷新旧期限。

## 集中验证与失败修复

环境Darwin arm64、CPython3.12.15、仓库.venv；未运行目标aarch64 Linux/ROS/Python3.10。

全部相关产品及测试适配后集中必要43项，没有重跑旧全套。命令：

```sh
.venv/bin/pytest -q tests/unit/test_learning_recorded_path.py tests/unit/test_learning_dual_clock.py tests/unit/test_startup_learning.py::test_owner_completes_verified_side_return_after_target_and_post_frames tests/unit/test_startup_learning_review.py::test_capture_pose_is_independent_of_delivery_order_and_current_tick tests/unit/test_startup_learning_review.py::test_old_moving_scans_cannot_fill_post_stop_validation tests/unit/test_startup_learning_timed.py::test_small_yaw_and_brief_reverse_are_geometry_not_attempt_termination tests/unit/test_startup_learning_timed.py::test_feedback_gap_and_generation_change_segment_without_new_action tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_chain.py::test_startup_flows_through_real_tick_and_safety_checks --junitxml=/tmp/learning-path-final.xml
```

首轮`/tmp/learning-path-batch.log/.xml`为32 passed/10 failed（66.42秒）。无服务诊断`/tmp/path_debug_plugin.py`及`/tmp/path_owner_trace.py`定位：reference wheel/IMU丢measurement导致host/sim路径每帧换代；interval fixture过去字段没有声明已结束区间，和point ZOH语义不同；point ZOH预测支持使旧“精确区间”正例不再有证据，必须给真实区间证据而非减预算。以上按原owner/fixture边界修正。

补测`/tmp/learning-path-final.log/.xml`为42 passed/1 failed（68.51秒）。仅余缺独立段负例：producer用于忽略快wheel/IMU的宽早退错误忽略同body源independent=False，旧root替代了显式负观测。Nav收紧后只补直接相关9项：

```sh
.venv/bin/pytest -q tests/unit/test_learning_recorded_path.py --junitxml=/tmp/learning-path-gap-final.xml
```

`/tmp/learning-path-gap-final.log/.xml`为9 passed（0.49秒）。此前42项未重跑；按JUnit节点取最后结果共43 passed、0 failed，是有原因补测后的去重汇总，不假称单次43项全绿。原始失败报告完整保留。

新9项覆盖：变速/停走point-ZOH dense/sparse相同nominal pose与原支持；转向+侧移有序SE2组合及历史capture pose/支持相同；原measured-variable-interval delta dense/sparse直接组合；纯coverage无rates历史不可积分；负独立段前后已知尾段可继续，累计已知距离不依赖消费频率；256段窗口淘汰不补旧前缀；重复endpoint不双算。原20项双域和5个完整学习周期、3个真实reference链、原理想区间正例亦在本轮必要批次通过。

最终无服务真实owner探针`/tmp/learning-recorded-path-probe.log`：同一三点point输入、一个真实接收owner，dense与sparse均pose_x=.010000000000000002、position_error=.00884462408671319、history=3。此数值是producer声明的上一值ZOH估计及既有预测支持，不是物理执行ACK；有实测区间时直接采用原delta。修复改变了实际路径消费，不是只把原0/.005差异藏到更大误差里。

## 静态、实际diff与冻结

`.venv/bin/mypy src/senior_control/motion/startup_learning.py src/senior_control/motion/learning_pose_history.py src/senior_control/safety/self_learning.py`为Success: no issues found in 3 source files，日志`/tmp/learning-path-mypy.log`。Ruff check通过、format --check为8 files already formatted，日志`/tmp/learning-path-ruff.log`和`/tmp/learning-path-format.log`；8自有文件Python3.10 grammar解析通过，范围git diff --check通过，无ignore/配置降低。

自有8文件：StartupLearning、learning_test_support、test_learning_recorded_path（新）、test_learning_dual_clock、test_startup_learning_review、test_startup_learning、test_startup_learning_timed、test_startup_learning_cycle。另本报告、冻结JSON及review返修记录；此前双域报告追加本轮指针，不把旧证据冒充此次通过。

实际增量`/tmp/learning-recorded-path-increment.diff`对照本任务开始快照`/tmp/learning-path-before/`，覆盖全部8文件；cycle旧版本由去掉本轮显式interval/delta hunk后验证SHA256与上一轮冻结源码完全一致，未把此前时钟/生命周期变更算本轮。源码与Nav依赖快照见`docs/testing/2026-11-16-learning-recorded-path-fingerprints.json`；自有文件已冻结，Nav并行依赖只记录此次快照，不替其宣布全域冻结。

三保护文件native_receipt.py、tests/test_native_receipt_diagnostics.py、docs/reviews/2026-11-16-software-estop-owner.md相对原任务前SHA256均UNCHANGED，工作区既有脏态未覆盖。目标Linux真实producer/ROS执行链及整车条件仍需统一验收；未扩展监督架构或总体目录重构，用户合PR后再开展后续架构工作。

冻结清单SHA256：`6099e82443143f754221304b30111b804089320af7e9b417b7932d0c03ec0e6b`；实际增量diff SHA256：`46ee074e79d63a5c57810eae921b68ee20861441c71aab4a39bfaaf952e88218`。
