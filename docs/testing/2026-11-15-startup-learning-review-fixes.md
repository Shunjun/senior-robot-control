# 启动学习 R1–R4 返修证据（软件验证，仍 Draft）

基线 `b64bca01dc31936d9b0de407b153bce7e8ec71d2`，`fix/degraded-drive-speed` 原工作区，开始时干净。按负责人逐项反馈串行修复，无helper、不切分支、不推送、不运行ROS/开发服务/车辆；原停车与估计成果、decel=.5、其他排队项未改。

## RED → GREEN 与实际实现

- **R1**：基线对同一组capture时刻按顺序或乱序送到，三个原始scan全部被绑为当前x=.05，正确应为 .01/.025/.04；5个运动期旧scan还会直接凑满后验计数。新增LearningPoseHistory保存独立积分的原ROS时刻、平移/转角误差、静止状态与代次，frame只按原scan.stamp查采样位姿，不外推、不改timestamp。支持同代次插值并额外保留整段位移/转角支持，不把插值当精确真值。模型按capture排序、去重，首帧free证据也随最早capture更新。后验只计停止阈值时刻之后采集、捕获时静止、且确实保留入模型的独立新帧；历史/模型容量耗尽不伪造5帧。完整core测试进一步复现后验阶段微小余速被当零、位姿冻结的误差，现后验期仍继续积分真实统一估计的余速并记录误差。
- **R2**：原world匹配阈值加了motion_error，但free只查endpoint tolerance，非零 .1 m 误差跨未知区仍得到learned=1。现完整SE2变换预测使用同一个world_error同时检查无匹配和已观测free，包含端点、首尾位姿平移、首尾转角在各自lever arm上的支持及显式附加motion_error。上述反例转为0，未清零误差。
- **R3**：在_observation缓存检查之前核对当前physical polygon、sensor/pose误差参数和原scan几何身份。任一变化先撤销启动与旧模型/空间缓存，再重建模型、同步run内feature边界快照；完成后的模型同样撤销，不按旧body屏蔽新body外点。实际SafetyGate使用start_parameter_services=False，未发现geometry的参数更新callback；_profile每次从当前footprint_xy读取，仍对本地/未来参数变化防守。新增body缩小及误差变更测试，并以同一原始capture/key直接调用_observation验证缓存必须失效。进一步RED复现原feature缓存仅按mtime、替换文件保留mtime时仍用旧polygon；现按实际内容缓存，原子替换不会因mtime相同留下旧边界。
- **R4**：基线在estimate=None的初始化等待100秒后直接failed。现只有完成5份静止运动证据、转入sampling时才开始30秒学习预算。程序claim仍在任何运动前一次性领取。初始scan位姿对齐暂缺会继续记录共享feedback_stop，但在尚未提过运动目标时不退役初始化尝试；显式急停/取消等仍中断。完整core先真实tick等待40秒，再接入原始数据，证明学习时间未被初始化耗掉。

完整周期还RED复现了正常目标停车没有drive permit、却被当作学习许可丢失。现braking的verified stop proof被正确识别；已进静止后验采样时不要求驾驶permit，原controller/停车/障碍检查继续执行，急停及故障事件照常撤销。没有为完成学习放宽扫掠或补造free。

## 测试环境与最终命令

沿用共享venv CPython 3.12.15，无依赖安装。测试均明确为合成输入/假ROS传输调用真实Python owner、统一估计、空间/约束、controller和状态机，不是ROS或实车验收。

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_startup_learning_review.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_chain.py tests/unit/test_motion_core_startup.py tests/unit/test_motion_core_event_time.py tests/unit/test_motion_core_handoff.py tests/unit/test_startup_learning_cycle.py -q --tb=short
```

最终 **69 passed in 41.15s**。未重跑191全套。新增review探针先得到5项真实失败（R1顺序/乱序、旧帧后验、R4、R2），R3接口/mtime以及停车边界也分别实际RED后修复。原纯模块夹具迁移到真实capture时刻和computed_ros，不再靠同一个stamp重复造帧。

core全周期5例保持原simulation profile及其非零误差，不清零风险参数：

| 输入 | 完整结果 |
|---|---|
| 40秒初始化等待，正常扫描，无内部候选 | 真实前移/减速/零输出/5个后验capture，completed |
| 同场景，停止后出现新内部回波 | 完整前移及后验，ambiguous |
| 运动期25ms延迟，后验期25/75/125ms变化及乱序 | 完整completed，所有存储pose逐帧对照原capture时刻合成轨迹 |
| 同延迟/乱序，停止后出现新内部回波 | 完整ambiguous，旧运动capture不能充后验 |
| 运动期也使用25/75/125ms变化/乱序 | 原扫掠sweep_unknown，安全中断且输出0，保留负例 |

上述completed正例没有内部候选，不是“已经实车学成设备”的声明。多帧learned正例与非零完整误差域反例另由纯算法测试覆盖。运动期较大延迟的保守负例没有改场景、缩误差或调停车参数消除；报告不保证任意合法延迟都能行驶。

## 临时socket覆盖补齐

先亲读test_docker_start_helper：service_root为TemporaryDirectory，运行对象是临时Python sleep/exit脚本或mock Child/Adapter，只有本地AF_UNIX socket，不包含ROS、Docker、开发或车辆服务。按本轮明确授权申请受管权限后运行：

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_docker_start_helper.py -q --tb=short
```

结果 **11 passed, 1 skipped in 0.30s**。此前7项socket bind沙箱缺口在本机该测试层已补齐；skip为macOS不具备Linux SO_PEERCRED，不能冒充Linux peer ownership验收。

新增/受影响Python的ruff检查及git diff --check通过，源码指纹见同目录 `2026-11-15-startup-learning-review-fingerprints.json`。完整ROS构建、真实DDS/C++安装组合、SCF3/823、ScanMotion双capture同版重建与实车仍Draft；calibrated旧测试矛盾继续留给独立兼容链，不在本轮改动。

## R3缓存路径的独立基线复现

为了不只依赖新增helper接口的缺失失败，保存了测试专用plugin；它仅从b64bca0提取原_observation方法，在隔离pytest进程内回放该方法，不改生产文件、不启动服务：

```sh
PYTHONPATH=src:ros2/src/senior_robot:tests/fixtures/startup_learning_review '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest -p baseline_observation_plugin tests/unit/test_startup_learning_review.py -q -k actual_observation_cache --tb=short
```

结果为预期 **1 failed, 10 deselected**，直接显示模型仍是旧polygon `[-.3,.3]×[-.2,.2]`，当前真实polygon已缩为 `[-.1,.1]²`。随后不加载plugin，正常命令相同用例 **1 passed, 10 deselected**，新body外的端点重新进入occupied；最终69项也包含该用例。这个有意失败的历史方法回放不是当前版本未解决的测试失败。
