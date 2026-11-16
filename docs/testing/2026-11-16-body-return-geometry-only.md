# 第7项：用户指定1cm车身回波匹配与全局误差解耦

代码修改完成，集中定向93项通过，待负责人亲审；未提交、未推送。功能主分支 `fix/degraded-drive-speed`，当前HEAD/开始基线均为 `593e8f13f385892f0bd62cf75c593f7b07efd2da`，成果为未提交增量。本记录替代先前精确坐标相等版本的完成结论，先前71项证据不足以证明有噪声可用。

## 最新要求与实际增量

用户最新明确允许1cm车内回波匹配误差；负责人撤回先前“不允许任何固定容差”的过度解释。本次采用用户指定1cm位置匹配，不再保留精确坐标相等门槛，不实施此前仅建议的前后窗口区间算法，未增加新的配置或支持域数据结构。

本轮产品增量仅 `safety/self_learning.py`，测试增量为 `test_body_return_geometry_only.py` 与 `test_startup_learning_review.py`；计划/设计/本报告同步更新。此前已正确删除的链路保留：模型error_m/self.error、原3mm兜底、射线延长及ambiguous_self_overlap消费者、matcher边界文件误差字段、模型签名中的sensor_error/orientation_error_rad、无生产消费者的motion_error/motion_radius/motion_error_m。硬件配置两字段与其外部避障/行驶空间用途不变。

学习以最后实际观测端点为候选冻结锚点：锚点在实际车身严格内部、所有学习帧的实际回波云均在1cm内有匹配，且世界固定反假设被观测拒绝，才能学习。运行只在冻结锚点1cm范围内匹配且实际端点严格位于物理多边形内部；实际边界与车外点不被过滤。保留决策不移动锚点或扩大半径，失配仍经不可变model_id/revision/锚点删除身份单向撤销，迟到快照无法恢复。撤销身份的精确成员校验是消息完整性检查，不是测量坐标匹配门槛。

1cm为 `SELF_RETURN_MATCH_DISTANCE_M` 用户指定匹配参考，不是雷达实测性能或全局避障预算。`_match_limit`仅根据输入坐标浮点ULP补偿表示舍入，使数学上恰好1cm的上下边界同样包含；没有额外物理毫米阈值。

## 世界固定反假设与完整支持域

首尾观测各允许1cm匹配，故世界固定反假设比较首尾支持相加的2cm范围，并加入已有独立位置误差；转角误差按点杠杆臂加匹配半径保守变换。该组合圆域由完整轴对齐方形包络，首次实际自由区多边形必须通过GEOS covers覆盖整个方形，并排除与首次全部实际回波兼容的世界假设。检查不是中点/端点采样；方形包络可导致保守拒绝学习，但不会使用内接多边形低估圆域。

独立运动/位姿预算保留：有独立来源的前向、侧向速度和角速度按观测时间积分，得到逐帧position_error_m/yaw_error_rad；缺少显式轴误差时仍用已有linear_error/yaw_error速度预算。它们仅保护跨帧世界反假设坐标注册，不成为车身边界扩张或雷达误差性能声明。实际传感器安装/扫描几何、来源、车身变化仍使模型失效；两全局误差配置变化不重建模型。

观测不可辨识边界保留：新物体若恰好替换在同一匹配支持内，单回波激光无法保证区分；本次没有新增额外传感器需求。已有启动临时内部先验与一次性学习生命周期不变，最终模型不是整车内部静态mask。

## 同版集中验证

环境：macOS arm64，Python3.12.15、Ruff0.17.3、mypy2.4.0；Python3.10语法另行核查。未启动服务/ROS/车辆，未提交推送。

```sh
.venv/bin/pytest -q tests/unit/test_body_return_geometry_only.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_review.py tests/unit/test_startup_learning_latch.py tests/unit/test_startup_learning_feasibility.py tests/unit/test_self_return_process_revocation.py tests/unit/test_self_filter.py tests/unit/test_rear_geometry_equivalence.py tests/unit/test_rear_fan_optimization.py tests/unit/test_startup_learning_chain.py --junitxml=/tmp/body-return-one-cm-tests.xml
```

最终结果：93 passed in 12.38s；`/tmp/body-return-one-cm-tests.log`、`/tmp/body-return-one-cm-tests.xml`。覆盖：

- ±0.2mm的实际多帧变化学到回波，下一帧不同但在容差内的测量持续过滤，锚点及revision不漂移；连续靠近的接受值不让>1cm的新障碍被吸入。
- 学习和运行均验证1cm边界包含、1.001cm失配；更近移动障碍、旁侧新增回波、1cm内但实际在车身边界/外部的端点保持hit。
- 真实世界固定/重复面/遮挡/缺束等负例保留；自由支持离中心的一束缺失也拒绝全域覆盖，不能仅用中心可见冒充完整支持可见。
- 序列化噪声快照保持锚点，失配事件只减不增，晚到正常副本不复活旧模型；真实core学习生命周期及边界变更链仍验证。
- 本地0.05/0.05与其他全局误差配置变化不影响模型；原始外部障碍证据不被matcher边界文件修改。

测试迁移说明：此前0.1m独立位姿支持的完整观测用例，在新的首尾1cm匹配支持加入后覆盖设备/未知区，故完整/缺束两例均保留歧义；未缩小位姿预算来伪造正例。正向噪声可用性另由原侧向回波真实几何场景验证。

15个修改Python文件的 `ruff check`、`ruff format --check` 和Python3.10 AST均通过，`git diff --check`通过。`mypy src/senior_control --no-incremental`仍非零：同环境HEAD基线68项，当前65项，按文件/消息/错误码无新增；修复之前涉及的模型构造、validate参数及frame_observations注解3项。对账 `/tmp/body-return-one-cm-type-comparison.json`，当前日志 `/tmp/body-return-one-cm-mypy.log`，HEAD基线日志 `/tmp/body-return-mypy-baseline.log`。不是mypy全量通过。

## 实际diff与冻结指纹

本轮相对精确相等版本的产品及新增噪声测试实际增量：`/tmp/body-return-one-cm-review.patch`；相对HEAD的完整第7项源码/测试补丁：`/tmp/body-return-one-cm-full.patch`。完整补丁排除三份保护文件与其已有脏改动。

```text
a795c0e7e424c157666b7c4fb503df7517ac5bbd9e408d7115ba2ae8507c4b27  ros2/src/senior_robot/senior_robot/motion_core_output.py
51cefd16b94eebc4ca34bc36b09c465b99713a5b03b0846a3cba4feafb387eb2  ros2/src/senior_robot/senior_robot/scan_adapter.py
c83e97c7751be0bb30b2ea7ba0ea9b631adb1ad635572c01e5a9a57366250e93  src/senior_control/motion/startup_learning.py
7b5aa008a97a9f23ec8a3d8dd9ac9b20f442264a6dc3de3fad2c921437e84334  src/senior_control/real_runtime.py
b57e0a33b83918afc1e95a66a90cdce820ccc4df6af1ecfd7d437d2725705df8  src/senior_control/safety/external_motion_features.py
742e99335d92ab1928d844831c194519566a5c43d9c23c5bfa3e7dc6c378822b  src/senior_control/safety/self_filter.py
df9460f4857d3d6afb13bd1e04c0b1af52cbca05182c9ab8cfdfd1b9fea05725  src/senior_control/safety/self_learning.py
e7b3fdb818af7984491565950f4f31bc194c1e87ba4c3d18841f21173f496f8d  tests/unit/rear_legacy_oracle.py
de13c39c5da2ee630ba6e096ed846efd0c3415a6916ff9a12d004cca2d2f7834  tests/unit/test_rear_fan_optimization.py
34000668fadcadd6f87b61abe77b6d7d2aedd928da2569528a52aa3fd333469a  tests/unit/test_rear_geometry_equivalence.py
df5d465cef8fe8c8f6dca6fa7724f1cd0dfc41275d019afc81594c9676dab61d  tests/unit/test_self_return_process_revocation.py
923e13f9d8c8971b40ef402c0b0affb7f404ccd215b5865737f7709a7f6989ec  tests/unit/test_startup_learning.py
601067019e6b726f9310828eb56bc739f5cfe72fe5ffb85ae15f011efb825a64  tests/unit/test_startup_learning_latch.py
16ed95e83ecff326a87e0a8dbdabf239675da0646cd002217165e484dcad4191  tests/unit/test_startup_learning_review.py
802c9f969fe94e6d5507e6342bc26a1f454aeed7093b2a4ea0b9a22c09c07b3b  tests/unit/test_body_return_geometry_only.py
```

增量补丁SHA256：`ac1b0772943d4052549b1b5e93b773116796a809555bad122d2cb3679b4a376f`。
完整补丁SHA256：`bdae08da535a2d9b5d3edca84c60e3b37c9bbfdd814fc3474b86d9d7bf2310b6`。

根方案第7项仅追加/更新实施状态，保留用户原文。三份保护文件与原任务开始字节指纹完全相同：native_receipt.py、tests/test_native_receipt_diagnostics.py、docs/reviews/2026-11-16-software-estop-owner.md。未暂存任何文件。

限制：这些定向测试含明确合成/注入数据，不等于真实噪声统计、硬件雷达精度、ROS调度或实车验收。未运行全套测试，本轮遵照要求集中验证受影响范围；导航第8—9项未处理。
