# 起步全零 sweep_unknown 调查与修复

结论：这是产品缺陷，不是该 fixture 的 odom 契约错误。本次只改两份产品代码，未改任何测试、配置误差、扫描ranges、外参或断言；未恢复状态准入门，未启动ROS/服务/车辆，未提交或推送。源码已冻结，交负责人亲审后由统一测试worker定向回归。

## 调查链与原数据

沿生产链核对：`NativeObservations.wheel` 实际生成wheel poses，`MotionSensorRuntime.ingest_poses` 构造 `PoseObservation`，`PoseEstimation` 查询旧/新锚点相对误差，`SpaceEstimation` 侵蚀自由空间，最后与最新 `fast_scan_guard` 求交。该测试使用 `header.frame_id=odom`（匹配仿真wheel_frame）、`child_frame_id=base_link`、单位平面四元数、pose恒0且twist=0，采集/接收时刻均为当前t，sequence由真实适配器生成；它与原生产接收路径一致，不需要重建或替换metadata。

原始同版失败见 `/tmp/batch-core-fixture-final.log/.xml`；`/tmp/debug_batch_chain_detail.py` 保持原fixture/原测试，在实际失败上下文记录：

- 旧扫描位姿t=11.95，roots为wheel序号79/80；当前位姿t=11.975，root为wheel序号80，二者均pose=(0,0,0)。不同绝对pose锚点没有共同积分误差证明，现有保守策略给出 `software_position_m=0.06`、已知position/yaw误差仍为None；没有把相同标称位置当成零误差。
- 原 selected raw perception 本身可给正向许可；加0.06m空间相对误差后，缺失区域落在车头外 `x=[0.337,0.35206249]`，面积约0.00149672m²，代表点(0.344512,-0.000547)确实在当前物理足迹之外。因此并非 `_clear` 把车体内部误判为未知，也不能通过填充车体或取消未知空间检查修复。
- 原 raw近场孔的x范围约 `[0.02188780,0.29210122]`；继续侵蚀0.06m后孔的前缘为0.35206249m，超过车头x=0.337m。最新快速扫描裁交没有制造该孔：原space和最终free的孔边界相同。

## 根因及实际diff

1. **预取被可选学习状态误禁用。** `prefetch_scan` 原先要求 `_learning_geometry is not None`，但学习disabled时 `_sync_learning_model` 直接返回，普通驾驶永远不能预取。即使LocalWorker同步完成，结果也只能下一tick消费，不必要地跨入另一个pose锚点。本次删除这一学习条件，仅保留扫描和motion输入存在检查；来源、坐标代次、过滤版本及原始期限检查不改。
2. **同一观测的窗口接缝被逐块侵蚀。** `scan_free_patches` 的意图是先合并重叠窗口，再施加传感器误差；实际 `prepare_perception` 却先对每块convex patch做负buffer，然后才union。窗口是同一观测的拆分，共用同一position误差；逐块侵蚀把人工窗口边缘变成未知边界，在近场造成额外空洞。本次先计算真实观测自由区域的并集，再用完全相同的error_m/mitre参数负buffer；占用方框、NoReturn策略、预算、感知原时间戳和截止均不改。

集合语义是 `erosion(union(observed_patches), error)`。这没有降低任何配置误差，也没有把真正未知间隙变成自由；只是去掉同一观测分窗造成的人工接缝。`SpaceEstimation` 的0.06m相对误差继续保守计入，`fast_scan_guard` 仍只收紧慢结果。未改PoseEstimation、SpaceEstimation、MotionSensorRuntime，也未对连续wheel anchors凭空宣称共同误差可以取消。

## 同场景证据

- 修改前执行 `/tmp/probe_union_before_erosion.py`：只在内存中把free合并/侵蚀顺序替换，原fixture及所有输入、占用、误差、断言不变，原测试通过，最终alignment仍为0.06。孔的前缘从0.29210122降为0.25709701；继续施加0.06m误差后仍在车头以内，不再侵入实际需要的新自由空间。
- 产品修改后执行 `.venv/bin/pytest -q tests/unit/test_motion_core_chain.py::test_native_recovery_then_intent_and_local_release_are_actually_consumed --junitxml=/tmp/native-chain-union-fix.xml`：**1 passed in 5.08s**。该测试自身要求手动起步正向输出、release后按jerk制动并最终零输出，未换场景或删除断言。
- `/tmp/probe_native_chain_delayed.py` 在同一个原fixture/原测试上，仅把扫描几何执行资源改为模拟80ms、120ms延迟，保证真正跨锚点：两组均通过，均有35次正向输出、最大vx≈0.050625m/s、最后vx=0，**最终software_alignment_error_m仍为0.06**。这排除了只靠同步预取消除跨锚点误差的伪修复；没有将标称静止强制为0误差。
- 纯几何反向探针验证结果等于 `union(raw_patches).buffer(-0.1)`，人工公共接缝可用；0.1m真实误差边界仍不可用，真实0.2m未知间隙仍不可用，输出 `UNION_EROSION_CONTRACT / TRUE_UNKNOWN_GAP / SENSOR_ERROR_BOUNDARY: PASSED`。
- 两份修改文件Python3.10 AST、Ruff、diff空白检查通过；相对上一冻结版本的增量patch已做 `git apply --reverse --check`，确认能精确还原上一版。没有运行全套测试。

## 交付与限制

产品变化只有：

- `src/senior_control/motion/constraints.py`：先union，再统一施加同一观测的error_m侵蚀。
- `ros2/src/senior_robot/senior_robot/motion_core_output.py`：普通预取不依赖学习geometry。

相对上次冻结指纹 `bdc6c08b0aa85cbc4a73205811b8653a878e3604a3128cafdfdce2d296bc028d` 的实际增量diff为 `/tmp/control-runtime-startup-availability-fix.patch`，SHA256 `5858c17c56d152981bbab1ca5d8b53e76a8a29eb9cb551d1fae5078a487ae59d`。当前37个产品文件清单为 `/tmp/control-runtime-startup.sha256`，清单SHA256 `3cf341215cf81a9a2c53d1ecaf218727d839ca6aaa5dedfdcf8a481399acd5b2`；其余35个产品文件与上一冻结清单逐个hash相同。

定向证据只证明纯Python原生形状输入到真实core/engine/controller的行为，延迟由确定性执行资源模拟；不证明aarch64实际并行调度、ROS/DDS、原生SOCK_SEQPACKET输出通道或物理watchdog。建议负责人审阅两份增量diff后，由统一测试worker补重叠窗口/真实未知间隙回归、普通disabled学习模式预取回归，并用上述指纹执行同版定向及既定集中验证；本worker不修改其测试文件。
