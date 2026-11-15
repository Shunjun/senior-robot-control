# 启动约束整批后续 CI 基线、格式与打包核验

日期：2026-11-16。功能分支`fix/degraded-drive-speed`，交付基点`c00bc7f00a619bc8e15dccb75da9240f56b6dea0`，Draft PR26；对照基线`ab241065e9446e933155bff5b1b58badc6d5e2cd`。本worker仅机械修改四个指定测试文件及报告，没有修改产品、mypy配置或用户停用native诊断两文件，没有暂存、提交、推送或启动服务。

## 范围和环境

使用同一个工作区`.venv`：Python3.12.15、mypy2.4.0、Ruff0.17.3由当前venv运行，uv0.12.23，macOS26.4 arm64。基线使用`git archive ab24106 | tar -x -C /tmp/startup-ci-baseline-ab24106`独立导出，没有新建或切换worktree；提交版同样导出到`/tmp/startup-ci-current-c00bc7f`用于定位初始错误所属函数。基线与当前pyproject SHA256一致：`fa2ed95d49c1c16b04283c3cab3e75fc69eed2c5339d1251469846fb6ade6bc5`。

CI实际命令来自`.github/workflows/backend.yml`：ruff check .、ruff format --check .、mypy src/senior_control、uv build及干净venv安装wheel。mypy使用`--no-incremental`隔离旧缓存影响；基线在导出目录执行，使用工作区venv绝对路径，配置均由各目录相同pyproject发现。

## mypy按函数对账

| 版本 | 错误/文件/源文件 | 对基线新增 | 对基线消失 |
| --- | --- | --- | --- |
| ab24106 | 72 / 9 / 155 | — | — |
| c00bc7f原提交 | 104 / 13 / 159 | 33 | 1 |
| 后续定向类型返修快照 | 68 / 9 / 159 | **0** | 4 |

匹配键为`文件路径、AST限定类/函数名、错误码、完整错误信息`及出现次数，不使用行号匹配；行号只定位所在AST函数。初始104条包含71条相同基线错误，新增33条并移除1条，不能将净增32误称新增32；返修后68条均为原有基线错误，新增归零。严格检查仍失败，不能声称整个mypy/CI通过，GPS/tasks等基线债务没有扩大修复。

初始新增分组：

| 数量 | 文件 | 类/函数 | 错误码 |
| --- | --- | --- | --- |
| 1 | `src/senior_control/agent/ports.py` | `TaskAwareMotion.send_intent` | `override` |
| 1 | `src/senior_control/motion/cadence.py` | `ControlCadence.__init__` | `no-untyped-def` |
| 1 | `src/senior_control/motion/cadence.py` | `ControlCadence.due` | `no-untyped-call` |
| 1 | `src/senior_control/motion/cadence.py` | `ControlCadence.due` | `no-untyped-def` |
| 1 | `src/senior_control/motion/cadence.py` | `ControlCadence.finished` | `no-untyped-def` |
| 1 | `src/senior_control/motion/cadence.py` | `ControlCadence.observe` | `no-untyped-def` |
| 1 | `src/senior_control/motion/controller.py` | `MotionController._still` | `no-untyped-def` |
| 2 | `src/senior_control/motion/controller.py` | `MotionController.propose_tick` | `no-untyped-call` |
| 1 | `src/senior_control/motion/controller.py` | `MotionController.reset_at_stillness` | `no-untyped-call` |
| 1 | `src/senior_control/motion/controller.py` | `MotionController.reset_at_stillness` | `no-untyped-def` |
| 1 | `src/senior_control/motion/diagnostic_pump.py` | `DiagnosticPump.__init__` | `no-untyped-def` |
| 1 | `src/senior_control/motion/diagnostic_pump.py` | `DiagnosticPump._run` | `no-untyped-def` |
| 1 | `src/senior_control/motion/diagnostic_pump.py` | `DiagnosticPump.close` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `LatestSnapshot.__init__` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `LatestSnapshot.put` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `LatestSnapshot.take` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.__init__` | `no-untyped-call` |
| 1 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.__init__` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.close` | `no-untyped-call` |
| 1 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.close` | `no-untyped-def` |
| 2 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.submit` | `no-untyped-call` |
| 1 | `src/senior_control/motion/process_resources.py` | `ProcessWorker.submit` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `_ProcessFuture` | `type-arg` |
| 1 | `src/senior_control/motion/process_resources.py` | `_ProcessFuture.__init__` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `_ProcessFuture.done` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `_refinement_loop` | `no-untyped-call` |
| 1 | `src/senior_control/motion/process_resources.py` | `_refinement_loop` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `child_lifetime` | `no-untyped-def` |
| 1 | `src/senior_control/motion/process_resources.py` | `stop_process` | `no-untyped-def` |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.filter_snapshot` | `no-untyped-call` |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.filter_snapshot` | `no-untyped-def` |

返修后相对基线消失的四条（新增方法补充类型也消除了原调用点债务）：

| 数量 | 文件 | 类/函数 | 错误码 |
| --- | --- | --- | --- |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.filter` | `no-untyped-call` |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.filter` | `no-untyped-def` |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.interior` | `no-untyped-def` |
| 1 | `src/senior_control/safety/self_learning.py` | `SelfReturnModel.validate` | `no-untyped-call` |

原始证据：`/tmp/startup-baseline-mypy.log`、`/tmp/startup-final-mypy.log`（负责人原104条）、`/tmp/startup-ci-current-mypy.log`（本worker68条）；可复核完整对账`/tmp/startup-static-final-comparison.json`。本worker没有写入任何产品类型修复。

## Ruff与机械格式

基线`ruff check .`只有feasibility的I001，原提交仍同一条既有I001。基线格式9文件，原提交格式6文件：五个共同项为agent/main、adapter_shutdown、startup_learning、feasibility、vehicle_state；新增relay格式，旧command_node/motion_core_output/startup_learning_chain/startup_learning_review四文件格式问题已消失。

格式诊断首个定位点同样按AST限定函数归档（format没有独立规则码，用`unformatted`标识；完整diff内其余修改位置保留原日志）：

| 版本 | 文件 | 类/函数 | 诊断 |
| --- | --- | --- | --- |
| `ab24106` | `ros2/src/senior_robot/senior_robot/command_node.py` | `CommandNode.runtime_observation` | `unformatted` |
| `ab24106` | `ros2/src/senior_robot/senior_robot/motion_core_output.py` | `MotionCoreOutput._warn_reference` | `unformatted` |
| `ab24106` | `src/senior_control/agent/main.py` | `Agent._safety_blocked` | `unformatted` |
| `ab24106` | `tests/ros/test_agent_adapter_shutdown.py` | `<module>` | `unformatted` |
| `ab24106` | `tests/unit/test_startup_learning.py` | `test_owner_finishes_once_and_marks_wheel_only_motion_ambiguous` | `unformatted` |
| `ab24106` | `tests/unit/test_startup_learning_chain.py` | `test_startup_flows_through_real_tick_and_safety_checks` | `unformatted` |
| `ab24106` | `tests/unit/test_startup_learning_feasibility.py` | `forward_scan` | `unformatted` |
| `ab24106` | `tests/unit/test_startup_learning_review.py` | `test_planned_verified_braking_is_not_permission_loss` | `unformatted` |
| `ab24106` | `tests/unit/test_vehicle_state.py` | `test_pending_overlay_never_clears_or_relabels_confirmed_estop` | `unformatted` |
| `c00bc7f` | `src/senior_control/agent/main.py` | `Agent._safety_blocked` | `unformatted` |
| `c00bc7f` | `src/senior_control/cloud/relay.py` | `Hub._push_connection` | `unformatted` |
| `c00bc7f` | `tests/ros/test_agent_adapter_shutdown.py` | `<module>` | `unformatted` |
| `c00bc7f` | `tests/unit/test_startup_learning.py` | `test_owner_finishes_once_and_marks_wheel_only_motion_ambiguous` | `unformatted` |
| `c00bc7f` | `tests/unit/test_startup_learning_feasibility.py` | `forward_scan` | `unformatted` |
| `c00bc7f` | `tests/unit/test_vehicle_state.py` | `test_pending_overlay_never_clears_or_relabels_confirmed_estop` | `unformatted` |

唯一lint诊断在基线/提交版均为`tests/unit/test_startup_learning_feasibility.py / <module> / I001`，并非新增；dirty native诊断为未交付的`NativeReceiptGroup._report_receipts / B905`。

本worker执行`ruff check --select I --fix tests/unit/test_startup_learning_feasibility.py`及`ruff format`指定四文件，AST比较忽略import位置后与HEAD完全相同；feasibility仅import排序有AST顺序差异，断言/测试主体均保持。指定四文件ruff check/format通过，定向`pytest -q --tb=short tests/ros/test_agent_adapter_shutdown.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_feasibility.py tests/unit/test_vehicle_state.py --junitxml=/tmp/startup-test-format.xml`实测**32 passed、9 skipped，0.51s**，ROS跳过不计通过。

完整dirty工作区的`ruff check .`失败于用户停用`ros2/src/senior_robot/senior_robot/native_receipt.py:314`的B905，format还包含该文件及`tests/test_native_receipt_diagnostics.py`，共2文件；这两份未提交内容不属于c00bc7f或本批交付，不修复、不暂存。完整失败日志保留`/tmp/startup-ci-current-{ruff,format}.log`。

交付范围命令显式加`--extend-exclude ros2/src/senior_robot/senior_robot/native_receipt.py,tests/test_native_receipt_diagnostics.py`，分别运行`ruff check .`与`ruff format --check .`，实测**All checks passed / 549 files already formatted**，日志`/tmp/startup-delivery-{ruff,format}.log`；该结论明确不等于dirty工作区全通过。

## 意图接口定向核验

底层`IpcMotion.send_intent_frame`必填driving/driving_version保持，应用层`TaskAwareMotion.send_intent`保持不变，super调用由核心worker修改。搜索tests不存在直接调用旧底层send_intent的测试，因此不机械改写正确的高层fake接口；执行现有真实TaskAwareMotion→独立IPC→CommandNode owner链及ports测试：

```text
.venv/bin/python -m pytest -q --tb=short tests/integration/test_manual_wire.py::test_real_authenticated_relay_agent_ipc_and_native_owner tests/unit/test_agent_ports.py --junitxml=/tmp/startup-intent-interface.xml
```

实测**6 passed，0.77s**，日志`/tmp/startup-intent-interface.log`。没有重跑全套。

## 打包和干净wheel导入

```text
uv build --cache-dir /tmp/startup-build-cache --out-dir /tmp/startup-build-dist
uv venv --cache-dir /tmp/startup-build-cache --python .venv/bin/python /tmp/startup-wheel-clean-venv
uv pip install --cache-dir /tmp/startup-build-cache --python /tmp/startup-wheel-clean-venv/bin/python /tmp/startup-build-dist/senior_control-0.1.0-py3-none-any.whl
cd /tmp
/tmp/startup-wheel-clean-venv/bin/python -I -c 'import senior_control.protocol, senior_control.config; import importlib.util; assert importlib.util.find_spec("shapely") is None'
```

已获准网络获取pyproject声明的隔离构建依赖；未使用--no-build-isolation。实际生成sdist及wheel成功，clean venv安装10包（含wheel本身）成功，protocol/config实际来自clean venv site-packages且Shapely不存在。初次uv venv漏传cache选项曾被沙箱默认cache权限拒绝，补/tmp cache后成功，不作为项目依赖缺陷。初次构建在类型返修前完成（仅包含已完成main/relay格式），其161包文件摘要保留`/tmp/startup-built-wheel-manifest.json`，最终冻结包结果见下节。

这是macOS/Python3.12打包证据，不是CI Ubuntu/Python3.10运行或ROS/aarch64构建证据，原批次报告未验证项目保持。

## 当前定向快照SHA256

下列以c00bc7f为基点，仅产品改动9文件、pyproject/uv.lock及本worker四个测试的内容指纹；产品来自原worker；随后已核对负责人明确的冻结清单，与本节记录完全匹配。

```text
fa2ed95d49c1c16b04283c3cab3e75fc69eed2c5339d1251469846fb6ade6bc5  pyproject.toml
ec71bfb1bc696c0dbf454659f90f137bb41a1abc2b31273a55ca1a5220896213  src/senior_control/agent/connection.py
13e623b9a040e5e858924070afb0af72cfa965f8915b0708ba231ce3f1a9fdea  src/senior_control/agent/main.py
4d84332f3a5ad058a6bc0cc5cedaa6d0703ca5c809a6616d43a34d03b56a4ba4  src/senior_control/agent/ports.py
d16b1b36b352454f220faad75bea7a5e0af20234ff9754f4e009d3e67541fd1b  src/senior_control/cloud/relay.py
cba369892c8b7c8ca062ca198f4aee0986245ce0ff097ceaefc91b168a7ddf6d  src/senior_control/motion/cadence.py
1bbef72443f7bb7ed28cad6aefd5473ab0b5dbec5aba58d44b24ed6f5eb5c2a3  src/senior_control/motion/controller.py
70ad6f4cc279410cc19c315111c36295909c5c0fa042d8dc2ba43ffa1da0ca18  src/senior_control/motion/diagnostic_pump.py
4c5c61c312626f5553cfa6f8d2eb52fcc543e07926fde3a3f58122d3346e603e  src/senior_control/motion/process_resources.py
c7054c96f83939007b67f9570db5f2c29cb99a19d7aa1a011e7c41af098a0e1e  src/senior_control/safety/self_learning.py
486a9be57e4c269023071b09df2f138359c5600be86add0ac13e26f5bbb180c4  tests/ros/test_agent_adapter_shutdown.py
c3f7bb1ce3c1fc70b136cc7f71d21ba3fb5a21bc207467b85ffa51a3a07e4b1d  tests/unit/test_startup_learning.py
210e3ef6a94847d14b0fe2e85ecd0f21bd1e196540a25f6183a3888504d17f54  tests/unit/test_startup_learning_feasibility.py
5958d32d254164035bbfa650c72b1419d44923344d539ea5875b33e114ddfe2d  tests/unit/test_vehicle_state.py
fffca84480644f4397dbbe1aee5048a95ed53c0a65beb43d960c8f4b93e2f177  uv.lock
```

## 最终冻结包与交付核对

负责人确认核心正式冻结清单`/tmp/control-typing-owned.sha256` SHA256 `86ea6f029e50699ac81eb8ebc437215336e6b146e54494326969677dc2b66fb6`，六文件全部匹配；实际patch SHA256 `fdb72dff05320d3b2f5b1295db8652aa80536fef10fea8f8232762871a6fe814`，connection依赖SHA256 `ec71bfb1bc696c0dbf454659f90f137bb41a1abc2b31273a55ca1a5220896213`一致。本worker前述mypy68条、Ruff交付范围通过及6项接口链测试所记录源码与该冻结版本完全相同，故没有重复运行同版充分证据。

冻结后实际执行：

```text
uv build --cache-dir /tmp/startup-build-cache --out-dir /tmp/startup-final-build-dist
uv build --offline --cache-dir /tmp/startup-build-cache --out-dir /tmp/startup-final-build-dist
uv venv --offline --cache-dir /tmp/startup-build-cache --python /Users/zhangshunjun/workspace/orca/senior-robot-control/fix-degraded-drive-speed/.venv/bin/python /tmp/startup-final-wheel-venv
uv pip install --offline --cache-dir /tmp/startup-build-cache --python /tmp/startup-final-wheel-venv/bin/python /tmp/startup-final-build-dist/senior_control-0.1.0-py3-none-any.whl
cd /tmp
/tmp/startup-final-wheel-venv/bin/python -I -c 'import senior_control.protocol, senior_control.config; import importlib.util; assert importlib.util.find_spec("shapely") is None'
```

两次最终构建均保留隔离构建，均成功生成sdist/wheel；第二次显式offline也通过，证明初次获准获取的构建依赖缓存足够。初次创建最终venv曾因在/tmp使用相对.venv解释器路径退出，改为上述绝对路径后成功，没有计为项目故障。最终clean venv装10包，实际wheel中161个包文件内容均逐一与冻结工作区src匹配；隔离模式导入protocol/config成功，路径来自`/tmp/startup-final-wheel-venv/lib/python3.12/site-packages`，Shapely不存在。

日志：`/tmp/startup-final-build.log`、`/tmp/startup-final-build-offline.log`、`/tmp/startup-final-wheel-install.log`、`/tmp/startup-final-wheel-import.log`；完整包内文件摘要`/tmp/startup-final-wheel-manifest.json`。

最终包SHA256：

```text
895d110c4cda322eeda1a83f8ac21071a5bd2c90399178380074be1ff77da608  /tmp/startup-final-build-dist/senior_control-0.1.0-py3-none-any.whl
ac332c4462ceec6bf12369343c741a8d8525b735abcff2a9c8cf9e673436fe61  /tmp/startup-final-build-dist/senior_control-0.1.0.tar.gz
```

已按实际对账JSON同步纠正`docs/testing/2026-11-16-control-runtime-typing.md`，两份报告均为初始新增33、移除1（净增32），最终新增0、移除4。完整mypy仍有68条既有错误，dirty工作区仍有停用native Ruff失败，不宣称全部CI通过；交付范围Ruff、相关定向测试及最终隔离打包/无Shapely导入通过。
