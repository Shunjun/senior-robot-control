# 实车共用运动核心接线（待负责人审查）

本工作树基于 CLI 联合启动已提交基线；未复制或覆盖原树未提交文档，未提交/推送。
用户在开发中明确要求“不写测试”，因此本轮未新增/修改测试、未运行 pytest/ROS/Gazebo，
未启动任何服务、连接实车、输出速度或占用 VM。只运行 compileall、修改文件 Ruff、
8 个变更 Python 源文件的严格 Mypy 和 git diff --check。

实际改动：SettingsService 及同一 RuntimeGate 在默认 real Agent 中创建；产品偏好和
硬件响应分离；硬件 JSON 严格 schema/缺项诊断/ROS YAML 加载和 CLI→systemd/helper
→launch 参数透传；共享 T3/T4/T5 selector；原 simulation profile 隔离门与接口保留并
标注 Compatible。实车共享栈装配扫描 receipt/filter/matcher、融合、安全唯一输出，
Agent adapter 作为单独受监督的原 PID 子进程，保留 IPC peer 校验。

motion_core_output 从 profile 消费物理响应、误差、侧滑、反应加速度及风险有效期，
不把 simulation_only/unvalidated 改名为实车。真实传感器身份不放宽；WHEELTEC
源码已核对 node=wheeltec_robot、odom、imu/data_raw，实际安装与元数据仍待现场核对。
增加物理急停状态输入契约，缺流或身份变化拒绝运动，实际 publisher 资料缺失未伪填。
地图/定位增加固定 WorkspaceGrid+SHA、唯一 publisher 和有界协方差/时效 facade，
复用既有任务/专用 Nav2 controller/来源 fence，仍不认 display-only 导出为导航资格。

当前硬件模型数值、急停 publisher、扫描与 TF 标定均缺实车依据，样例 null 故意拒绝；
现状为代码装配完成待审，不能称实车可驾驶。共享核心当前 laser_only 路径的深度限制、
Humble 缺 per-sample GID 时硬件拒绝，以及真实导航外部 map/localization 依赖，见
`docs/operations/hardware.md`；没有用合成或静态检查冒充硬件/仿真回归。

与 navigation-validation 的未来接口交集：agent_adapter 使用 reference.grid/pose/qualified
及相同 RouteService；DedicatedController 只增加 odometry_topic 与时钟透传，未改
terminal_release、source_retired、执行结束协议、任务持久化或导入其他工作树成果。
集成前负责人需亲读这些交集实际 diff，再做统一树的验证；本轮无相关回归结论。
