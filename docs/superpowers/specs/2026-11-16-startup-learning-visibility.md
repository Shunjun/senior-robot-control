# 启动学习：前后60度可用回波准入

本节记录2026-11-16用户最新批准的局部政策，覆盖此前“后方一小块self未知区即可否决全部学习动作”的行为。只用于StartupLearning仍活跃且控制源为startup_learning或其idle停车尾部的快照；完成后的正常行驶、手动/导航、其它方向和原运动风险误差不采用此豁免。

配置为 `startup_learning_front_occlusion_stop` 与 `startup_learning_rear_occlusion_stop`，各自独立布尔，默认true。硬件配置由HardwareMotionProfile和hardware-motion.json/schema加载；仿真由SimulationMotionProfile及simulation_motion.yaml加载；SafetyGate同名ROS参数默认承接已选profile，再经统一typed override入口应用，接收子进程沿现有参数快照转发。无网页设置需求，也不增加比例或角度配置。

车头中心0、车尾中心π，各±π/6，共60度。实际束方向为angle_min＋ray.index×angle_increment＋安装yaw，不能用固定数组区间、参考点数量或抽样数决定方向。每束按半步长角覆盖单元与扇区相交，可用单元求并集，避免重复端点/重叠计数；扇区总角度减去可用覆盖即不可用比例。self、无效/缺测和未观测FOV不提供可用覆盖；已验证no-return沿原clear合同可用。扇区边界按相交角宽计，不按束中心硬切。

开关true时该方向不可用覆盖>=70%阻止学习；低于70%使用剩余可用点，遮挡未知区不再一票否决。false时该方向不因遮挡比例阻止。允许方向的未知覆盖要求在60度扇区内按该明确政策豁免，既有free_space从不补写为已测自由；扇区之外的未知覆盖继续原检查。没有实际不可用覆盖的方向不获得额外豁免。

ConstraintSnapshot单独携带本次启动政策，_clear仍先检查真实occupied障碍；真实障碍即使两个开关均false也阻止。仅idle/startup_learning来源可使用该覆盖政策，普通manual/navigation即使误带政策字段也维持原未知区判断。其它时效、风险、方向与输出截止不变。策略缓存身份只随有效规则/方向状态变化，正常比例的小幅变化不另加资格门。

动作仍是首次成功私有提交后固定0.05m/s目标、host steady3秒；结果仍按已批准的多帧稳定位置学习、完成保存冻结、运行1cm只读过滤，无撤销或每帧持久化。以前非空core未成功的失败报告不改写；本轮新软件完整链证据见[测试报告](../../testing/2026-11-16-startup-learning-occlusion.md)，不代替Linux/ROS/实车验收。
