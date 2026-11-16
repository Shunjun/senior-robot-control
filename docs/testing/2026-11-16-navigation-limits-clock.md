# 第8项与原导航第9项验证；扩展全系统第9项待审

## 版本与隔离

- 实际派发及当前HEAD：`593e8f13f385892f0bd62cf75c593f7b07efd2da`；分支`fix/degraded-drive-speed`，未提交、未推送、未更名分支。
- 派发时已有21份改动的逐文件基线在`/tmp/navigation-8-9-dispatch-baseline.json`，该基线JSON SHA256为`cb43171d1336bfeabd713dc2359d06b800e6934a9a435d903e0f106060d02adf`；完成导航增量后逐文件核对全部21份内容未变。
- 三份禁改独立文件native_receipt.py、test_native_receipt_diagnostics.py、software-estop-owner.md均维持派发时字节。第7项已有未提交增量不纳入本任务自有清单。
- 后加入的学习claim清理worker修改docker_start_helper.py/simulation_cli.py及其新模块/测试，本任务没有修改这四份文件，也不计入自有diff。
- 原导航部分已实现并验证；用户随后扩大第9项至全系统学习/估计/传感器时钟，新增范围仅审计、尚未实施，详见`docs/reviews/2026-11-16-control-clock-system-audit.md`。不能以本报告认定扩大的第9项完成。

## 实际改变

删除导航入口0.3m/s、0.5rad/s整条硬拒，保留有限值/来源/执行代次/原始新鲜度校验。共用ConstraintEngine按本轮RiskEnvelope线/角上限同比缩放原目标再搜索既有fraction候选并核验组合扫掠；不会将“整条拒绝”挪到固定fraction下游。低速目标不升速，零目标保持原优先语义，导航degraded_cap政策未改。

导航消息改为同版NavigationSample，携带boot域、run nonce、生产sequence及计算开始的CLOCK_MONOTONIC前沿。接收端移除wall→steady offset、10ms时钟撤销、wall顺序前沿和对应退役测试。原始deadline继续传递到FinalCommand，资格/图谱检查耗时计入原始年龄。Humble缺少逐样本GID时保留原有私有run-topic契约，有GID时校验，绑定的唯一writer图谱GID变化拒绝该旧绑定。

固定官方Nav2 Humble1.1.20 commit `a097086719c88f781aa59788eca29ac6ca5e56db`；仅17份构建文件缓存，不复制整个Nav2。`tools/nav2_overlay/manifest.json`锁定每份文件及5份patch文件原始/最终SHA256。controller非零与零输出完整走同一发布函数，失败容忍、进度超时和控制等待改steady；TF/路径/日志原ROS语义保留。构建脚本校验源/patch/最终内容，不覆盖已有目的目录；运行入口校验同版Nav2包与契约marker，不自动升级或部署。

## 命令与结果

环境：macOS26.4 arm64，CPython3.12.15，pytest8.4.2、Shapely2.2.0、Pydantic2.14.1；Apple clang21.0.0。Python本机monotonic实现为mach_absolute_time，非Linux部署环境。无`/opt/ros`或`colcon`。

最终集中命令（纯合成/注入输入，无节点启动）：

```sh
NAV2_SOURCE_CACHE=/tmp/nav2-1.1.20 .venv/bin/python -m pytest -q \
  tests/unit/test_navigation_command_clock.py \
  tests/unit/test_navigation_small_intent.py \
  tests/unit/test_navigation_limits.py \
  tests/unit/test_navigation_overlay.py \
  tests/unit/test_navigation_core_output.py \
  tests/unit/test_directional_constraints.py \
  tests/unit/test_motion_controller.py \
  tests/unit/test_motion_trajectory.py \
  tests/unit/test_constraint_scheduling.py \
  tests/unit/test_navigation_collision_qualification.py \
  tests/unit/test_navigation_completion_handoff.py \
  tests/unit/test_terminal_release_collection.py \
  tests/unit/test_terminal_release_wire.py \
  tests/unit/test_motion_core_chain.py \
  tests/unit/test_motion_core_exit_deadline.py \
  tests/unit/test_motion_core_event_time.py \
  tests/unit/test_motion_core_handoff.py \
  tests/unit/test_motion_core_stop_fallback.py \
  tests/unit/test_motion_path_convergence.py \
  tests/unit/test_unified_motion_runtime.py \
  tests/integration/test_manual_wire.py \
  tests/integration/test_shutdown.py
```

**335 passed in 32.23s**，无失败、跳过或收集错误；原始输出`/tmp/navigation-8-9-final-green.log`。IPC集成测试使用临时AF_UNIX测试socket，因沙箱禁止bind而经执行权限运行；不是真实服务、ROS、车辆或真实运动命令。

- 新用例覆盖线/角分别超旧阈值、组合及百万量级原目标、负方向、低速与精确零；校时前/后保持当前来源且新样本继续；过期/未来/重放/旧run/外国boot/GID/替换writer/NaN/Inf拒绝及断流收束。
- 除callback→Arbiter→FinalCommand，还执行真实ConstraintEngine→MotionController的verified非零受限输出，并有4项真实MotionCoreOutput/异步约束→最终输出接缝超限用例；不是只看入口收到了样本。
- 同版补丁上下文经`patch --batch --fuzz=0 -p1`和原始/最终散列验证；故意破坏源字节可在写目的目录前被拒绝。
- 用Apple clang实际编译并执行从**已打补丁官方源**提取的publishVelocity/publishZeroVelocity方法及真实navigation_timing.hpp，`-std=c++17 -Wall -Wextra -Werror -pthread`；ROS消息/发布器/Costmap接口用受控形状替代，非ROS整包编译。验证非零/零sequence、boot/run、原始production不重盖和系统时间前/后改动独立。
- C++采集时刻与Python读取POSIX CLOCK_MONOTONIC上下界一致。本机Python默认mach_absolute_time与POSIX时钟不等价，因此不能将它作为Linux Python/C++同域实测；Linux测试分支使用`time.monotonic_ns()`，本机使用`clock_gettime_ns(CLOCK_MONOTONIC)`，真实boot域限Linux且部署前明确检查namespace。
- 所有本任务Python产品/测试文件定向ruff通过；`ruff format --check`覆盖新增模块/测试/脚本通过。原command_node使用CRLF，为保留原格式运行`git -c core.whitespace=cr-at-eol diff --check`通过。
- `mypy --ignore-missing-imports`检查navigation_clock.py、prepare_nav2_overlay.py、navigation_overlay.py三个新模块通过；不加该选项时本机唯一报错为缺少ament_index_python.packages，未用代码抑制冒充具备ROS类型包。

## 失败与修正记录

没有隐藏失败：首轮151项导航/约束/控制验证通过。补充核心输出测试初稿调用不存在的reset，4失败/86通过；按实际fresh-core路径修正测试，4项输出用例随后全部通过。

合并335项初跑为316通过、16失败、3错误：一处C++测试stdout数字解析错误，另18个IPC用例因沙箱禁止AF_UNIX bind失败/报错。修正stdout后又发现本机macOS Python默认mach_absolute_time不等同POSIX CLOCK_MONOTONIC，测试按实际部署域分开比较并如实保留Linux未实测限制。IPC以必要权限独立48项通过，随后集中同版335项全部通过；记录未把环境错误算通过或通过skip消除。

## 限制与待办

ROS/Humble全包构建、生成消息、ament installed_header_consumer、实际DDS、真实插件ABI、aarch64、容器时钟namespace以及实车运行未验证。构建/安装说明见`docs/operations/navigation-monotonic-overlay.md`；普通未patch或不同版本依赖须明确失败，不暗中patch现车包。

本轮不运行全仓测试；按用户要求集中受影响范围验证。没有启动开发服务、ROS或车辆，没有发送真实运动指令。完整全系统第9项仍待负责人审计方案亲审与外部producer采集契约确定，不得将本阶段335通过当作整系统完成。

## 自有清单冻结

排除本证据报告自身的25文件内容冻结SHA256：`5d904cf3abc9b71d5eb65f495eea46e0ae7b2cae597470acbfab0632792f2567`。完整26文件逐份散列清单及实际diff分别交付`/tmp/navigation-8-9-owned-sha256.json`、`/tmp/navigation-8-9-owned.diff`，不包含第7项、三份独立改动或其他worker文件；根目录表格第8—9项外部状态改动单列`/tmp/navigation-root-table-update.diff`。

## 后续producer只读核查

厂家资料只读核查与学习worker接口已补入producer契约报告及43源文件散列清单。此后只有文档新增/修订，335项导航测试所覆盖的产品与测试代码没有改变；未实现新增全域。最初导航冻结实际diff/清单保存在`/tmp/navigation-8-9-pre-producer-review.diff`和`/tmp/navigation-8-9-pre-producer-review-sha256.json`，当前含producer审计文档的28文件清单及实际diff另见`/tmp/navigation-8-9-producer-review-sha256.json`和`/tmp/navigation-8-9-producer-review.diff`。
