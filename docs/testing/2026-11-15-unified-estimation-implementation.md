# Unified estimation implementation ledger

> 最新原生返修：负责人已认可P1/P2主要修正，继续累计审核发现RGB-D年龄输入异常逃逸；本次边界修复和未构建ROS测试说明见[原生年龄报告](2026-11-15-native-age-boundary-review.md)。累计审阅仍未完成，以下阶段事实继续保留。

> 最新返修：负责人已亲读R1–R3并继续累计审阅，发现共同积分前缀误差和乱序配对两项回归；本轮修正与证据见[区间/配对返修报告](2026-11-15-relative-interval-sync-review.md)。累计100文件审阅未完成，待本轮复审；以下保留先前阶段事实。

> 返修状态：负责人已亲读部分累计代码并提出R1源级年龄统计、R2相对位姿误差、R3独立预测证据接线三项问题，累计100文件审阅尚未完成；当前返修与定向证据见[审阅修正报告](2026-11-15-unified-estimation-review-fixes.md)，待复审，未验收。下文保留437b0c8及更早阶段事实。

Plan: docs/superpowers/plans/2026-11-15-unified-sensor-estimation.md
Base: 07b815de1c92bb11469483d2b99f94fa922e2cb5, fix/degraded-drive-speed.
Coordinator authorized serial execution in msg_170d686f2959; no intermediate approval gates.
No service/VM/vehicle/network publishing. No stopping-envelope/decel/self-filter/legacy-cleanup scope.

Pre-flight: T1 time/provenance -> T2 estimation -> T3 pose/space -> T5 controller all share original-root times, observed masks and per-axis deadlines. T4 diagnostics observes these contracts, never grants permission.
Ruling: Missing vx/wz model prediction must expire on its own original axis evidence at 1 second; other-axis samples cannot renew it (coordinator clarification).
Ruling: Common defaults live inside the installable sensors package JSON; native launch receives the same values, no editable per-stream age copies.

Task 1: in progress. New timing tests initially fail importing the absent module; implementation follows.

2026-11-15 implementation checkpoint (not acceptance):
- T1/T2: shared installable .3/1 policy; original ROS receipt delay plus steady elapsed; per-axis bounded MotionEstimation, provenance groups and scalar consensus implemented. New timing/estimation tests ran green before later integration edits.
- T3/T5: control output owns raw ingress and estimator; per-axis PI masks; optional vy=0. Pose/space owners now feed scan alignment; 150ms wheel-pose gap removed. Optional explicitly configured GPS Odometry and authenticated RGB-D candidate adapters added; no topic auto-enables them. Body evidence remains distinct from wheel/control estimates.
- T4: reporter, pending counters, diagnostic work deques/keyed maps/async queues instrument native receipt, RGBD inbox/pending-TF, detector/result queues, depth/synchronizer/progress, constraint, agent safety/intent, transport/log outboxes, IPC and task lane waits. C++ shared age/queue helper and RGBD native TF warnings added. Remaining diagnostics owner edge cases and full queue inventory reconciliation are still being checked.
- Ruling: remove only the replaced inactive fused-snapshot driving selector from MotionCoreOutput, not unrelated legacy/progress infrastructure. Retaining source selection code here would obscure the unique owner; historical snapshot tests now cover original raw rows through that owner.
- Ruling: condensed startup capture omitted raw laser values. The offline adapter does not invert fused averages into measurements. Startup recovery tests explicitly supply a synthetic independent interval and preserve captured occupied geometry; they are composition tests, not physical laser replay.
- Ruling: historical captures lack receipt ROS times. Offline compatibility charges the later snapshot ROS time conservatively, never infers ROS time from steady elapsed/RTF=1. Production adapters preserve native receipt ROS clocks.
- Incremental evidence: 26 timing/estimation/pose/diagnostic/runtime tests passed; 16 migrated raw replay/clock/boundary tests passed; 13 startup invariants passed; 3 optional RGBD/calibration tests passed. Affected detector/controller/constraint/semantic group: 73 passed, 1 skipped. Depth/RGBD group after common-age facade: 44 passed, 1 skipped. These are versions at the time of each run; later modifications get affected coverage again.
- IPC lifecycle test initially could not bind Unix sockets in sandbox; same 12 bounded tests passed after approved sandbox escalation. No development service, VM or vehicle was started.
- Standalone C++ shared timing/queue helper compiled with -std=c++17 -Wall -Wextra -Werror and assertions passed. ROS message generation/node/guard compilation remains unavailable and unverified.
- Strict whole-src mypy currently includes pre-existing tasks/lifecycle, lifecycle_store and controller optional-result errors; new annotation errors were identified and are being resolved. Ruff/style checks are not final yet.
- Work remains: missing queue instrumentation edge cases, targeted optional GPS/pose/consensus/diagnostic stress coverage, schema/doc synchronization, final affected checks and diff review. Branch remains uncommitted above 07b815d; nothing pushed.


## 当前交付检查点（实现完成，等待负责人亲审本轮累计 diff）

代码位于现有 fix/degraded-drive-speed，工作区 `/Users/zhangshunjun/workspace/orca/senior-robot-control/fix-degraded-drive-speed`。保留已审退化修复 `07b815de1c92bb11469483d2b99f94fa922e2cb5`，其 main 基线为 `ad1c0c6e5764f436d0d25ec6622a9ff9d32afd09`。上文“in progress / work remains”是中途历史事实；以下更新现状。协调者已审方案并授权实施；新统一估计代码尚待其最终亲审，不能沿用前一项代码审阅通过作为本轮通过。

### 实际实现与关键入口

- T1：`sensors/defaults.json/policy.py`唯一 .3秒软/1秒硬默认；原始 ROS 采集与接收延迟加 steady 经过时间取保守年龄，等于1失效。硬件/仿真与原生launch共用，旧配置年龄只在内存迁移，明确 Compatible。手动TTL/控制dt/计算预算未统一成传感器时间。
- T2/T5：`motion/estimator.py`逐轴 owner，`sensors/consensus.py`一致组、权重及确定性冲突，`motion_sensor_runtime.py`组合，`optional_motion_inputs.py`显式 GPS/RGB-D 接入。MotionCoreOutput每tick重新算剩余有效来源，不再订阅融合快照作为驾驶反馈。短时缺轴模型受自己原始根截止；vy缺测0。controller逐轴PI mask、去重复body/wheel冲突门，正常轨迹/轮约束/急停保留；停车回退校验同一原始证据身份，允许当前时刻合法预测而不延寿。
- T3：`pose_estimator.py`有界历史、角度插值、同源去重、相对/绝对切换与 frame_generation；`space_estimator.py`显式坐标关联后保守并集，不平均不同物体，不把缺测变空闲。没有实现候选全局track关联器，按设计附录明确取舍；现有空间消费者仍有既有可用用途边界。
- T4：`runtime/diagnostics.py`共用有界 reporter 与 queue/lock/counter wrappers；年龄raw/use、排队等待、控制周期和计算耗时分开；DEBUG默认5秒，积压上穿锁存、下一pump告警，重复5秒及恢复限频。实际队列落点、参数和未知深度详见队列盘点。C++ header复用相同配置与用例；未读私有qsize或增大任务队列。
- 原生：ScanMotion新增 first_capture/second_capture，laser区间最老原始接收时间不会被匹配/交付时间刷新；原生 RGB-D 时效/TF队列参数由共同策略注入。消息生成、C++节点和 guard 需要同版重建。
- 新增 semantic 公共1秒构造/年龄边界覆盖，修正旧 .3秒构造限制会拒绝共同参数的问题；仿真图像写槽 .5秒同义过期使用共同1秒。匹配算法本身的有效帧间跨度、future等待、任务超时等不是传感器年龄，按其原语义保留。

### 实际验证与失败修正记录

精确 pytest argv、文件列表、结果及代码SHA256指纹见 `2026-11-15-unified-estimation-verification.json`；所有 Python 检查均使用原项目共享venv并显式 `PYTHONPATH=src`，避免editable安装误读其他工作区。

- 最后运动/诊断/感知/真实纯软件组合定向组：**242 passed**。包含新增逐轴/冲突/去重、部分过期、单源/缺vy、原始两帧处理延迟、非法未来输入不污染前值、模式/控制连续性、pose/space、可选源、C++公共header、队列洪峰及实际core回退。不是ROS/DDS或实车运行。
- 传输、日志、agent/manual/config/IPC frame组：**258 passed**；之后生产传输代码未变。
- 扩大检查storage/task/输入边界组初跑：**359 passed,29 failed**。在 `git archive 07b815d` 的独立 `/tmp` 源码复现对应旧夹具 **27 failed,8 passed**；失败夹具遗漏上一项 SCF3 的手动元数据，另外两项还依赖旧融合快照/源组合强门。只迁移测试输入和已明确变更的期望，没有修改任务生命周期生产逻辑；受影响两文件后续 **35 passed**，同版其余边界检查353项此前通过。共用导航夹具另补零元数据，所属文件 **9 passed**。不隐藏初跑失败，也不把重跑数累加成独立用例总数。
- 早期受影响vision组 **102 passed,1 skipped**；后续新增semantic改动由上述242覆盖。缺OpenCV的相关RGB-D数值用例被跳过，不计验收。IPC lifecycle曾因sandbox不能绑定Unix socket失败，经授权有界运行12项通过；未起产品开发服务。
- Ruff对全部82个变更/新增Python文件 `check` 与 `format --check`通过；Python编译检查通过。`git diff --check`通过。无本轮前端改动，未重复跑上一项前端构建。
- 完整 `mypy src`检查158个源码文件，仍为 **4个既有错误**：tasks/lifecycle:318、tasks/lifecycle_store:647、controller:625/627的Optional ConstraintResult。已与07b815d归档基线同样4错核对；没有称完整类型检查通过，不顺带修这些既有问题。
- 离线用本机缓存 hatchling 构建wheel成功，核对其中 common defaults.json、estimator与diagnostics字节等于当前源码；公共C++ timing/queue header以C++17、Wall/Wextra/Werror编译并断言通过，包含在242项中。

### Draft与后续边界

本机 `ros2/colcon`不可用且 `/opt/ros`不存在。仓库 `.github/workflows/backend.yml`仅Python检查/pytest/wheel，没有ROS underlay/消息生成/guard完整链接；未恢复VM、未启动服务或实车、未改实车配置、未重试GitHub发布。完整Linux/Humble构建与生成接口、scan matcher/RGB-D receipt/guard链接、ROS真实接收与整车测试均未完成，必须保留Draft限制。

部署必须在同版源码下重建 senior_motion_msgs、senior_sensor_ingress、senior_scan_matcher、senior_robot以及原生消费者；前项FinalCommand私有帧仍为SCF3/823，本项ScanMotion原始两帧字段同样不能混用旧生成消息。可在明确可用的隔离Linux环境运行colcon build/test并记录CTest；本轮宿主C++纯模块测试不能替代。

独立停车速度域、decel=0.5、自回波启动学习及旧兼容链清理均未实现，继续由队列中的后续新Agent负责。本轮估计健康不代表既有独立停车包络必然发放通行；轮/模型不被声明为独立物理真值，软件容差/质量/权重也不是用户指定或实测保证。可选GPS/RGB-D真实ROS接线、定位/空间误差、传感质量及模式切换的实际驾驶效果均留作ROS/物理验证，不宣称整车验收。

本轮只提交当前功能分支供负责人审累计diff；未推送、未建PR、未改main或其他工作区。
