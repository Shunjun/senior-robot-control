# 启动15厘米学习与单天线GPS旁路（软件实现，Draft）

基线 `4eeed6426a72acd9e76ccd50cbe0e0ca5229d2a7`，在 `fix/degraded-drive-speed` 原工作区追加；初始干净，不切分支、不推送。本轮按负责人明确允许的**严格物理内部先验**继续开发，替代前版阻塞判断。未启动开发服务、ROS、VM、Docker或实车，未改车端配置。

## 实际变化

- 真实real/simulation程序start owner将run专属claim路径传给子进程；输出owner在非零运动之前O_EXCL持久化领取，重连/组件重建只得到interrupted，不再次运动。完整程序的新一次start可重新尝试，不自动恢复旧行程。
- StartupLearning初始化静止证据→5帧采样→最高.05 m/s目标前移→约.15m估计行程制动（.148m阈值，末段按剩余距离降速）→5帧验证→completed/ambiguous；超30秒failed，急停/接管/数据失效/运动后失去许可中断。目标经过原snapshot、ConstraintEngine、scheduler和controller，无旁路cmd_vel。
- 开始前的内部hit可在指定学习状态暂作provisional；端点及软件误差方形支持域必须严格在既有物理polygon内。边界歧义、外部点保留，遮挡后方不生成free。起步前无permit保持零并等待；首个实际非零输出之后若无permit，则撤销临时模型并重建未经先验过滤的空间再停车。
- 保留并接入实际polygon到碰撞Footprint和许可identity，未改decel=.5、轨迹/制动扫掠公式及任何profile数值。原profile尺寸仍约束可配置硬件polygon。
- 多帧比较body/world两假设；外部激光/其他独立运动与raw wheel一致性、完整相对平移/转角证据及现有软件误差参与验证。vy缺测的0假设不作分类证据，但不阻止相对运动。世界预测域若遮挡或未观测则ambiguous，不能用未匹配到点冒充free。最多64帧×2048点、128验证候选；未验证的点不屏蔽。模型只保留局部距离/端点支持，不黑整个角度；每新帧回波移走或消失立即撤销对应项，源代次/外参变化撤销。
- 新external_motion_features把输出owner从现有polygon生成的run内边界快照提供给scan_adapter。它只排除内部及边界不确定的**匹配特征**，防止自身回波认证独立激光平移；原始安全scan和障碍路径不改，未将这些点标为learned。快照未就绪时匹配候选等待，wheel/IMU相对估计仍可推进初始化。没有新增ROS消息或topic。
- 生产节点移除self_mask_xy读取，出厂YAML去掉手填区域，real loader明确Compatible忽略旧键；低层离线replay self_mask API保留供独立兼容链清理。本轮没有清理该链。
- 显式gps_input.course_frame启用ENU heading-only原始Odometry路径，不依赖已有heading/TF解环。已知位置精度、直行基线、速度精度一致性、轮速方向和连续IMU窗口决定是否初始化，倒车course转π，未知/不足精度/跳变/转弯/重放无航向。原始GPS/轮/IMU期限共同约束输出，沿共享.3/1秒策略；输出gps_heading与统一gps_course周期诊断。没有GPS不阻止相对估计或学习。原body-frame GPS输入行为保留，schema同步生成。

## 验证

Python环境：共享既有venv（CPython3.12.15），源码用 `PYTHONPATH=src`；没有安装依赖。

最终定向命令：

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_startup_learning.py tests/unit/test_startup_learning_chain.py tests/unit/test_gps_course.py tests/unit/test_optional_motion_inputs.py tests/unit/test_motion_core_chain.py tests/unit/test_motion_core_handoff.py tests/unit/test_motion_core_startup.py tests/unit/test_motion_core_exit_deadline.py tests/unit/test_motion_core_event_time.py tests/unit/test_motion_core_stop_fallback.py tests/unit/test_limited_motion_core.py tests/unit/test_stopping_fused_input.py tests/unit/test_directional_constraints.py -q --tb=short
```

结果 **191 passed**。新增API先跑到缺模块/接口失败，启动实链探针也实际复现了内部遮挡造成sweep_unknown；随后明确验证保守等待分支，没有移动障碍、填造free或放宽profile来使它通过。端点分类、一次性claim、真实急停command、核心tick输出、前方障碍、边界/外伸端点、遮挡、重复纹理/平行面、外部跟随点、撤销、轮空转/独立证据不足、GPS精度不足/转弯/跳变/倒车/原始ENU适配均有定向探针。

ruff对全部新增/受影响Python文件检查通过，git diff --check通过。前序停车测试的纯_snapshot夹具显式补node._profile依赖，未改变原障碍、速度或断言。

额外尝试的 `tests/unit/test_docker_start_helper.py` 有7项Unix socket bind被当前sandbox以PermissionError拒绝、1项skip；未提权重跑，也没有启动真实服务。起初合并回归中的其他34项失败均为纯_snapshot夹具缺node，补齐夹具后已被最终191项覆盖通过。前轮已记录的旧 `test_self_filter::test_uncalibrated_mask_is_unusable` 与生产旧标志忽略规则矛盾，本轮不改、不将其计入通过。

## 解释边界与剩余限制

这是合成输入调用实际Python调用链、假ROS传输及离线算法测试，**不是ROS、Gazebo或实车验收**。完整ROS构建、真实DDS/C++新Python安装组合及进程监督生命周期还需验证；SCF3/823、ScanMotion双capture同版重建限制保留，未运行旧capture冒充新版本结果。

硬件仍无接收机型号/RTK精度证据，默认草稿未配置GPS。15cm不能保证物理行程精度、独立位移证据或回波分类，轮/IMU-only可估计推进但不会生成learned。给定默认hardware误差较大时，分类可全部ambiguous；没有修改误差参数制造成功。后方设备遮挡若与原有保守扫掠需求相交，即便内部hit不再导致occupied自重叠，仍会sweep_unknown而不前移，这一负例已保留。

暂定先验与learned均依赖正确物理polygon、激光外参及profile软件误差；这些不是已实测保证。无法区分同一支持域内的设备与完全同观测替代物，不能声称解决所有物理不可观测情形。GPS输出为统计朝向观察，不自动校正地图、TF或轮里程计，也未引入任何接收机驱动。

作者已自审实际diff，未派helper；负责人应亲审后统一交付。独立兼容链清理、1米前障碍保守角扫掠等继续排队，没有夹带。具体代码SHA256见 [指纹](2026-11-15-startup-learning-fingerprints.json)。

## 追加状态投影修复

首个功能提交 `6f43312` 后自审补获：启动输出虽受安全链约束，共享MotionState仍可能显示idle。新增 `test_startup_registers_and_retires_shared_runtime_task` 实际RED复现，随后通过TASK_START/TASK_RETIRED接入共享仲裁，并在完成/中断/急停/接管时退役启动身份；MotionTarget类型显式包含startup_learning。未得到共享仲裁接纳不能设置启动运动目标，期间运行状态现在为task或原有obstacle_hold。定向22项先通过，最终完整命令191项通过；没有把该修复隐藏在旧190项证据中。
