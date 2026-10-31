# ROS 安全拓扑修复 wave 1

基线 5923581。Nav2 与脱困分别进入 CommandNode 的 `/control/navigation_cmd`、`/control/recovery_cmd`；仲裁按连接、模式和单调时钟有效期选择速度，安全门输入只由仲裁发布。急停、接管、失联、取消及重新获取均清除旧速度。脱困模式需调用 `begin_recovery` 明确切换；当前尚无运行时调用者，默认拒绝脱困速度。租约释放后新会话可获取，保留每会话代次序号；适配器停用后必须以更高代次明确获取才重新启用。CommandNode 的业务时钟不覆盖 rclpy 内部时钟。

安全门改用稳态定时器，查询扫描时间的 TF 并计算实际激光坐标；扫描、带时间戳的车体速度均需新鲜且时间递增，缺失、超时、时钟回退或 TF 无效停车。odom 配置统一 `/wheel/odom`，验收节点名统一 `safety_gate`。

## 证据

- RED `pytest tests/unit/test_arbiter.py -q`：2 failed, 10 passed（缺少自主输入和释放后新会话拒绝）；实现后 12 passed；追加恢复与取消重放测试并确认失败后修复。
- RED `pytest tests/integration/test_shutdown.py::test_disabled_adapter_requires_new_epoch_acquire_to_reenable -q`：1 failed；修复后 1 passed。
- `pytest tests/unit -q`：236 passed。
- `pytest tests/integration/test_shutdown.py -q -k adapter`：8 passed, 9 deselected。
- `pytest tests/unit tests/integration -q`：416 passed, 1 skipped, 1 failed, 4 errors；五个失败均为本环境禁止 Unix socket bind 的 PermissionError（test_ipc_frames_are_versioned_and_length_limited、test_shutdown_disables_motion_before_ros_exit、test_sigterm_revokes_motion_before_cleanup、test_cancel_propagates_and_releases_resources、test_ipc_motion_sends_validated_envelopes_and_the_epoch）。
- `pytest -q`：收集失败，e2e 与 unit 的 test_home.py、test_recovery.py 同名冲突，1 skipped、2 errors；项目 README 指定按 unit/integration 范围运行。
- `pytest tests/ros/test_velocity_chain.py tests/ros/test_navigation.py -q`：5 skipped，本机无 rclpy；不视为通过。新增真实 TF 和冻结 ROS 时间稳态定时器检查供 Linux 执行。
- `python -m compileall -q src/senior_control/safety ros2/src/senior_robot/senior_robot`：退出码 0。

## 未验收

Linux Humble 构建、启动及完整速度链、时钟冻结/恢复、真实 TF、底盘 watchdog、测量源质量和硬件停车均未在本轮执行。独立扫描匹配/运动估计的 ROS 供应者尚未接入；安全门没有新鲜质量合格的 TwistStamped 时保持零输出。脱困控制器、动作取消与网络 Agent 仍未组装，不据此声称可自动导航或实机安全。Nav2 原始 Twist 无来源时间戳，网络或 ROS 中间层排队后的旧消息无法根据消息自身时间识别，需 Linux 故障注入验证并在系统组装时处理。安全限值与测量源须现场标定和验收。
