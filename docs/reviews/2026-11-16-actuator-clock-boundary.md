# 第9项：执行端 DDS 时间边界补充

状态：发现现有消息缺字段，待负责人确认最小同版协议修改；尚未修改该执行端协议。

现有 `ros2/src/senior_motion_msgs/src/motion_output_guard.cpp` 从私有包读取 `(vx,wz,deadline,sequence)`，以 CLOCK_MONOTONIC 核验原始 deadline，但只向 `/cmd_vel` 发布 Twist。WHEELTEC `wheeltec_robot.cpp` 订阅 Twist 后不能独立判断一个已排队的非零目标是否过期；官方 ros2_controllers 2.50.0 `diff_drive_controller.cpp:124` 用 ROS time 减命令 header 判断超时，unstamped callback 在接收处以 ROS now 填 header。后者会因 ROS 时间校正提前停车或延长命令寿命。

建议增加单一 `ActuatorTarget` 同版消息：`SourceTime timing`、`geometry_msgs/Twist data`、`uint64 valid_until_steady_ns`。沿已批准的同车 boot/CLOCK_MONOTONIC 契约，正向输出由控制帧开始时的 `now` 建立时间依据，私有输出包仅增加该原始 computed steady 字段，native transport 保留它和原始 deadline，不在 DDS 发布或订阅处重盖。native producer epoch/sequence 描述这一个 ROS producer 的发布前沿；原有私有管道 sequence 继续只承担私有包完整性，不新增另一套源配置或时间阈值。

非零目标在接收处必须仍在原始 deadline 内，且保留现有执行超时（仿真当前 0.3 秒）取较早失效时间。底盘按原有串口协议写速度，独立检查原始单调失效时间并在真正过期时写零；新合法非零帧也不能让旧帧延寿。零输出、断流和进程退出仍闭合，执行代次/同源顺序与 boot 域不可被旧消息替换。来源时间事实不宣称串口发送即硬件 ACK。

仿真 diff_drive_controller 的控制超时、控制 dt、里程积分及发布周期按单调时间推进，Odometry/TF header 仍用传入 ROS 时间作为外部标签。wheel 的 TimedOdometry proof 在 update 计算入口产生并随本次输出透传。保持既有速度算法、限幅参数和实际时长，不增加车端限速配置；该修改不处理第10项心跳队列。

需要联动的具体文件：`motion_output_guard.cpp`、`senior_robot/motion_output.py`、消息注册、新消息；`safety_node.py:_emit_motion_core` 与 `motion_core_output.py:tick` 的原始 computed steady 参数；WHEELTEC 外部受控补丁的 Cmd_Vel_Callback/Control；固定官方 ros2_controllers diff_drive_controller 小补丁及本仓仿真 remapping。学习 worker 已拥有最终提交后记录 t0 的独立 hunk，新增 computed 时间参数必须保留其逻辑。

这些是开发固定基线，不代表已安装版本。Nav2 固定 1.1.20 commit a097086719c88f781aa59788eca29ac6ca5e56db；仿真官方资料已解析 ros2_controllers 2.50.0 commit 58dc6db28040f017c992b7ecfa57310c7e78d18d、Fortress sensors 6.8.1 commit 9b408698a74903c6e3c5eb5745567a0cddef2ef7、gazebo 6.17.0 commit d9b18d85a17400d32494bca4c3589f519aca7e81、ros_gz 0.244.20 commit 6be1a56e797d4a9e94c9aff280a943df3dfa6d48。来源元数据与逐文件 SHA256 已缓存于 `/tmp/simulation-clock-upstream/source-lock.json`，尚未完成仿真补丁或目标构建。

所有构建准备只写新 overlay，厂家资料只读；不启动 ROS、车辆或服务。当前没有目标 Linux/Humble 工具链，真实构建、插件 ABI 与实车 dpkg 版本仍是 Draft 部署前置，不计作已通过验证。
