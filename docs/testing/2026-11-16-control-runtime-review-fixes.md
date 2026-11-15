# 控制隔离审阅返修

代码返修完成；产品源码已交统一测试 worker 按同版集中验证，尚未收到整批结果。未提交、暂存、推送，未运行全套测试、ROS、仿真或车辆。本记录覆盖初版实现报告中 exact-scan-key 准入、pending 学习处理及固定参数硬门的描述。

## 五项返修

1. `MotionCoreOutput.tick` 对 `scan_geometry_pending` 同时跳过学习中断和 FEEDBACK_LOST，不撤销有效目标；真实反馈失效、证据到期或权限丢失仍按原生命周期处理中断。pending 输出未验证零，不宣称已完成停车。
2. 自回波模型只在主 owner 修改。`SelfReturnModel.filter_snapshot` 是纯检查，返回过滤观测及冻结的 `SelfReturnRevocation(model_id, base_revision, baseline, removed)`；`apply_revocation` 校验模型身份、版本和删除成员，只做当前模型减集，重复事件幂等，外来模型/未来版本拒绝。每个最新扫描先在主 owner 应用撤销，过滤版本马上变化，旧 worker 结果无法重新掩盖移除后重现的回波。重几何 worker 只接收已过滤不可变观测，不再接收模型，也不持有 learned/revision 副本。
3. `ScanGeometry` 按来源、坐标代次及完整过滤模型版本隔离；序号负责排序，不再因不断到来的下一条扫描永远丢弃刚完成结果。最多一个在途、一个最新 pending；结果携带原扫描快照，保留原采集/接收时间和原截止。`MotionCoreOutput` 用最新扫描的 `fast_scan_guard` 收紧：相邻射线距离取保守最小值，形成单个径向可见域多边形及近场孔，未知/自回波方向收缩为近界，占用以批量误差方框表达；只与慢结果 free 求交、向 occupied 追加最新证据，不扩大旧许可。最新扫描和原结果任一到期均不能继续输出；来源/模型/坐标变化仍丢弃旧结果。
4. 恢复正且有限的可配置频率，默认20Hz；cadence 周期为 `1 / simulation_rate_hz`，不再拒绝非20数值。去除 SafetyGate 新加的 `/cmd_vel` 硬门，将 `resolve_topic_name(output_topic)` 作为私有参数传入原生唯一发布进程，C++ publisher 使用该参数，不额外创建第二个 writer。
5. 唤醒入口先记录时钟基线，input update/prefetch 后重新采样才判断槽位，只有实际开始一轮的回调才调用 finished；核心在命令 guard 工作后自行采样真实计算时刻，controller dt 仍是实际两次计算间隔。进程/报告/接收/输出通道 close 改为幂等，关闭几何进程前将未完成 Future 取消，后续 done 不读取已关闭 Process；保留先撤销/零输出、再有界清理的顺序。

`fast_scan_guard` 在控制侧只有有界射线过滤、一次可见域构造/收紧和现有最终核验，不执行慢路径的逐窗口 buffer/union 或方向候选搜索。其最坏耗时及 aarch64 真实调度仍需统一验证，不能把本次算法探针称为20Hz硬实时或实车验证。

## 定向证据

未修改测试文件。统一测试 worker 已按新接口加入持续扫描、模型撤销和 pending 学习用例；本 worker 仅运行以下 `/tmp` 纯 Python 定向探针，所有输入标为合成，未创建 ROS 节点或发送运动命令。

`/tmp/control-runtime-review-probe.py` 使用真实 `MotionCoreOutput._observation`、`SpaceEstimation`、`ConstraintEngine` 和 `MotionController`；Future 延迟由确定性模拟时钟控制，持续扫描每帧带0.002m正弦扰动。2秒、41个控制槽的结果如下，非零计数要求 decision.verified 且 command.vx>0，不把 pending 零输出计入成功：

| 扫描频率 | 模拟重几何延迟 | 有效许可次数 | 非零已核验输出次数 | 首个许可 |
| --- | --- | --- | --- | --- |
| 10Hz | 80ms | 39 | 38 | 100ms |
| 10Hz | 120ms | 38 | 37 | 150ms |
| 20Hz | 80ms | 39 | 38 | 100ms |
| 20Hz | 120ms | 38 | 37 | 150ms |

四组均验证原几何 `deadline <= original_stamp + 1s`，未用最新扫描刷新旧截止；有新慢任务在途时插入0.38m障碍，最新 occupied 立即包含该点、旧远方 free 不再覆盖前方1m位置。最近一次探针输出 `PURE_SYNTHETIC_PROBE_PASSED elapsed=5.807s`，这是整段探针执行时间，不是控制最坏周期保证。

`/tmp/control-runtime-revocation-probe.py` 验证跨pickle纯过滤不改变主/副本 learned、主 owner 应用删除/重复幂等、回波重现不再成为self、外来模型拒绝；还覆盖扫描全周接缝末射线出现障碍或未知、range_min=0/0.06，以及0→80→100ms槽位和倒退时钟拒绝。输出：`REVOCATION_PICKLE_REAPPEARANCE / FAST_GUARD_SEAM_UNKNOWN_ZERO_MIN / CADENCE_0_80_100: PASSED`。

自有Python3.10 AST、Ruff及 `git -c core.whitespace=cr-at-eol diff --check` 已通过。CPP/ROS真正EOF、队列溢出、驱动看门狗、同机多核负载最坏耗时均不由这些纯探针证明；统一测试 worker 所在macOS不支持SOCK_SEQPACKET且无ROS，需要明确保留该部署验证缺口。

## 同版交接

HEAD 为 `ab241065e9446e933155bff5b1b58badc6d5e2cd`。37个产品文件的清单在 `/tmp/control-runtime-review.sha256`，清单SHA256为 `bdc6c08b0aa85cbc4a73205811b8653a878e3604a3128cafdfdce2d296bc028d`；此版本已交统一测试 worker 并锁定，之后仅补文档和探针，没有再改产品。

相较上次产品指纹 `21f44f262fa35531b17575a5da9eb24a1441843581a5af7654a8f054fb9d018c`，返修涉及以下10份产品文件：

- `ros2/src/senior_motion_msgs/src/motion_output_guard.cpp`
- `ros2/src/senior_robot/senior_robot/control_inputs.py`
- `ros2/src/senior_robot/senior_robot/motion_core_output.py`
- `ros2/src/senior_robot/senior_robot/motion_core_perception.py`
- `ros2/src/senior_robot/senior_robot/motion_output.py`
- `ros2/src/senior_robot/senior_robot/runtime_reports.py`
- `ros2/src/senior_robot/senior_robot/safety_node.py`
- `src/senior_control/motion/cadence.py`
- `src/senior_control/motion/process_resources.py`
- `src/senior_control/safety/self_learning.py`

其余原产品改动保持初版范围；本次没有编辑 tests、web 测试、native_receipt.py、tests/test_native_receipt_diagnostics.py 或既有急停review文档。负责人仍须审阅实际diff及统一测试结果，再决定验收/提交/更新PR25，不自动合并。

完整自有产品实际diff（含新增文件、相对开始基线）为 `/tmp/control-runtime-review.patch`，SHA256 `99ea5bc32604c0da7ab103b989585bacd955d8a43215faa3fc068c1bd1f03874`；初版实际diff仍保留在 `/tmp/control-runtime-owned.patch` 供比较。
