# 启动学习前后遮挡准入与非空完整core证据

任务 `task_42785d15d6ab / ctx_a5fb055edf96`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。实现及本地集中验证完成，待负责人亲审；不启开发服务、ROS、仿真、车辆，未暂存、提交、推送。完整源码规则见[当前局部设计](../superpowers/specs/2026-11-16-startup-learning-visibility.md)。

## 已授权规则与统计口径

前、后方向各中心±30度、共60度。新增两个独立布尔项：`startup_learning_front_occlusion_stop`、`startup_learning_rear_occlusion_stop`，默认true。true时该方向不可用角覆盖>=50%阻止启动学习，低于50%用其余可用点；false时该方向遮挡不构成阻止理由。比例50%及角度60度保持固定，没有新增可调阈值或网页设置。

实际方向由扫描angle_min、angle_increment和安装yaw转换到车体坐标；ray.index仅参与其真实角坐标计算，不按固定数组区间或模型参考点数量计数。每束以半步长角单元与60度扇区相交，可用单元求并集，避免inclusive重复端点或重叠覆盖重复计数；不可用比例=1−可用角宽/60度。self、无效/缺测和未观测FOV不提供可用角覆盖；验证过的no-return沿原clear合同可用。边缘束只按落入扇区的角宽贡献，安装yaw和扫描数组起始角改变不能改动车体方向定义。

只在StartupLearning活跃且来源为startup_learning或其idle停车尾部时生成独立StartupVisibility字段。ConstraintEngine先检查真实occupied，之后执行50%开关策略，并仅在相应有不可用覆盖的前/后60度内豁免未知覆盖要求；不补写free_space，不把未知区标成已测安全，不隐藏可用观测中的真实障碍。其它方向未知区、普通manual/navigation、完成后的正常驾驶仍执行原规则。原风险误差、停机/时效、0.05m/s学习目标、首次实际私有提交后host3秒、保存恢复和固定1cm只读模型均未改变。

该豁免是用户明确批准的启动策略，不是测量获得的安全证明。普通行驶即使误带该字段也不采用豁免；缓存身份随有效政策变化，不以低于阈值的比例噪声另加资格门。实际free/occupied对象保持原数据，覆盖检查只改变启动策略是否要求相应未知区证据。

## 配置与实际接线

HardwareMotionProfile新增严格bool字段，hardware-motion.json及schema同版接入；SimulationMotionProfile和实际simulation_motion.yaml提供默认true。SafetyGate先选择硬件文件/隔离仿真profile，再以同名ROS参数默认承接该profile的布尔值，经configure_startup_learning集中校验和应用；ControlInputs沿原所有参数快照转发，没有第二份配置owner。字符串false和数字不冒充bool。

StartupVisibility由实际tick及退出停车重算处建立，普通_snapshot调用不依赖学习对象。ConstraintSnapshot只新增可选政策字段，正常快照默认None；最终_clear和退出输出检查消费同一明确政策。没有改StoppingRisk误差或用未知空间伪造free。

## 实际非空完整core回归

此前失败原输入完整保留：rear index0=0.15，其余有效值10，分别不加缺束、仅index1缺束、仅末束缺束、双侧缺束。新测试走原完整cycle，真实core/约束/控制器和显式synthetic测量/假私有传输；不替换permit或障碍结果、不恢复空输入造通过。

四组均实际发出非零学习输出，目标及输出上限0.05m/s、wz=0，首次提交后host截止3秒；结束输出归零、保存非空模型、回归点被标self，同run恢复模型相同且动作不能再消费。另有前/后各自关闭开关的>50%遮挡完整周期正例，证明加载的false实际影响core，而非只测配置对象。前后配置相互独立的四种组合、真实SafetyGate配置块及硬件文件加载均独立验证。

真实障碍负例：rear小self遮挡、front外部0.38m实际命中，即使前后开关均false，前方点仍是hit，实际core始终零输出、不消费动作，并报告stop_envelope_blocked。纯约束回归同时验证原free_space未改、侧方未知仍拒绝、普通manual携带字段也不能豁免。49%/50%/51%、60度边缘角单元、安装yaw四方向及不同扫描起点、既有no-return合同分别覆盖。

这些是本地软件完整链证据，不是实际ROS发布或底盘执行ACK，也不等同真实车辆完成3秒运动。旧非空core失败报告和指纹不改写，当前新证据仅推进上述场景；完成模型后普通驾驶的其它限制未扩大或宣称解决。

## 集中测试

产品最终版本集中组单次：**246 passed，87.65秒**，`occlusion-final-current.log/.xml`。这是相关用例集合，不是全仓库或全部运动控制测试；本轮按参数展开净增38个用例，其中新增遮挡规则/加载文件35个，原core回归文件由4个增至7个、净增3个；另迁移旧后方小self必须零输出的断言，真正前方障碍负例保留。

```sh
.venv/bin/pytest -q tests/unit/test_startup_visibility.py tests/unit/test_learning_core_surface_review.py tests/unit/test_learning_frozen_reference.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_latch.py tests/unit/test_startup_learning_timed.py tests/unit/test_stopping_fused_input.py tests/unit/test_directional_constraints.py tests/unit/test_motion_controller.py tests/unit/test_common_sensor_composition.py tests/unit/test_motion_launch_profile.py tests/unit/test_motion_core_stop_fallback.py --junitxml=/tmp/occlusion-final-current.xml
```

覆盖新遮挡政策、配置、非空完整core、原学习/冻结过滤、锁存、停车与方向/控制器，以及输出期限后重新计算停车。集中组之后仅补强硬件文件默认值承接及未设置ROS覆盖时的断言，4个配置参数用例定向补测通过，产品未再修改；不将它描述为整组重跑。没有重跑其它类别，也没有将旧147/35项报告混入本轮计数。

严格mypy的4个模块（startup_visibility、hardware_profile、motion_profile、simulation_motion_profile）通过；本轮10个修改源码/测试Ruff通过，format --check为10 files already formatted。constraints.py单独严格mypy仍有2个原有问题：Footprint.vertices返回tuple维数及buffer(join_style=2)的stub类型；同一命令检查本轮启动快照精确复现相同2项，本轮未降低检查、加ignore或改这些非本轮代码。SafetyGate/core沿项目原动态ROS接口，未声称严格mypy全覆盖。

## 失败与修正证据

完整日志/JUnit保存于[evidence目录](2026-11-16-startup-learning-occlusion-evidence)。早期诊断文件只运行约20帧却断言整个3秒完成，1失败时动作已开始约0.65秒；正式回归采用原完整周期，不能把短探针算完整成功。普通回归首轮34失败均因新_snapshot直接要求学习对象，已改为实际启动owner显式传入政策，普通接口恢复；其余120项当轮通过。移动政策建立位置时一次漏接主tick，仅接退出停车处，阶段集中6失败45通过后中断，已修主tick及退出停车同版接线，不恢复空输入。之后245通过1失败是旧后方小self必须零输出断言，与新批准规则相反，按新规则迁移，9项链路补测通过；最终上述246项同次全部通过，所有失败日志仍保留。

## 缺证与冻结

当前环境macOS arm64、CPython3.12。未运行ROS、Linux/aarch64原生IPC/真实spawn控制链或实车，本轮没有Linux跳过项混算成通过；这些本轮未复测。实际硬实时预算、外参/传感器标定精度及底盘执行仍需目标环境验收。

本轮启动快照 `/tmp/learning-occlusion-start` 含823文件和SHA清单；相对此快照的精确增量见 `2026-11-16-startup-learning-occlusion-owned.diff`，逐文件启动/最终SHA见 `2026-11-16-startup-learning-occlusion-fingerprints.json`。三保护文件及原StoppingRisk误差策略字节不变；未整体reset/checkout文件，其他工作区改动保留。指纹不自引用；diff不包含自身或指纹文件。
