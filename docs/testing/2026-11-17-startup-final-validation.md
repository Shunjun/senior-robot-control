# 第一阶段最终同版验证

日期：2026-11-17。分支 `fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`，与任务指定提交一致。工作区保持未提交。没有改产品、测试、断言、三份保护文件，没有暂存、提交、推送，没有 `git reset`/`checkout`，没有启动整套开发服务、ROS、仿真或车辆。

本报告只记录这次新执行。先前 `docs/testing/2026-11-16-*` 与 `2026-11-17-duration-milliseconds.md` 不计入本次通过数。各组 pytest 分开统计，下面的数字不能相加后当成一次更大的通过。

## 冻结

检查开始前写入 `docs/testing/2026-11-17-startup-final-validation-fingerprints.json`。

| 项 | 值 |
| --- | --- |
| 文件数 | 755 |
| 清单汇总 SHA256 | `89909fc2105732e6f4967bdb22bcfa7362942259f27c4b19db5509120eae4b86` |
| `pyproject.toml` | `fa2ed95d49c1c16b04283c3cab3e75fc69eed2c5339d1251469846fb6ade6bc5`，与 `git archive HEAD` 相同 |

范围是 `src/`、`tests/`、`ros2/`、`web/`、`config/`、`tools/`、`pyproject.toml`、`uv.lock`、`.github/workflows/backend.yml`，以及三份保护文件。排除 `.venv`、`node_modules`、缓存和本报告自己的路径。pytest 使用这份清单里的明确 `test_*.py` 路径，没有执行 `docs/testing/` 里归档的 `.py.txt`。

每个检查前后都重算了这份清单。全部结束后 `changed`、`missing`、`added` 都是空的，`source_drift` 为 false。三份保护文件散列与冻结时相同：

| 文件 | SHA256 |
| --- | --- |
| `ros2/src/senior_robot/senior_robot/native_receipt.py` | `53d08230fb94dce972ed72dbf6fb16532d3b34aea4e5ed7191772a9bd0ef910a` |
| `tests/test_native_receipt_diagnostics.py` | `5c33cba2555e0c0b9b0096a025620fbfdb9ef86ef03bfee53e485dc43c440408` |
| `docs/reviews/2026-11-16-software-estop-owner.md` | `cf258266704168979f6485658e6c8e08d18bd444e6a1175cfc2f848ff5cab4d1` |

`tests/test_native_receipt_diagnostics.py` 不在 unit/integration/deploy/e2e 四组里，本次没有执行它。

环境：macOS 26.4 arm64，工作区 `.venv` 的 Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、mypy 2.4.0，uv 0.12.23。前端使用已有 `web/node_modules`：Node v24.14.1、vitest 5.0.5、TypeScript 5.9.3、vite 6.4.5。没有安装或升级 `.venv` 里的包。`import rclpy` 失败，`/opt/ros` 不存在，`colcon` 不存在。本机 `c++` 为 `/Library/Developer/CommandLineTools/usr/bin/c++`。

原始日志在 `docs/testing/2026-11-17-startup-final-validation-evidence/`。构建、mypy 缓存、干净 venv 和前端产物在 `/tmp/startup-final-validation-*`。

## Pytest

四组按目录分开跑，避免 `tests/unit/test_home.py` 与 `tests/e2e/test_home.py` 同名导入冲突。`tests/ros` 单独一组。命令形态：

```text
PYTHONDONTWRITEBYTECODE=1 NO_COLOR=1 \
  .venv/bin/pytest -q -p no:cacheprovider --tb=short \
  --junitxml=docs/testing/2026-11-17-startup-final-validation-evidence/pytest-<group>.xml \
  --basetemp=/tmp/startup-final-validation-pytest-<group> --durations=20 \
  <该组 fingerprints 里的 test_*.py 清单>
```

清单文件是同目录下的 `pytest-<group>-files.txt`。

| 组 | 文件数 | 结果 | 耗时 | 退出码 |
| --- | --- | --- | --- | --- |
| unit | 149 | 1858 passed，6 failed，15 skipped | 268.69s | 1 |
| integration | 35 | 656 passed，1 skipped | 43.54s | 0 |
| deploy | 1 | 8 passed | 2.34s | 0 |
| e2e | 5 | 7 passed，1 skipped | 0.31s | 0 |
| ros | 27 | 43 passed，98 skipped | 0.78s | 0 |

unit 里包含两份本机 C++ 节点，它们的结果已经含在上表 1858 里，不再另计：`test_cpp_age_queue_boundaries_and_transient_crossing` 与 `test_native_checked_age_input_matrix_does_not_throw` 均为 passed。

跳过，均不算通过：

- unit：6 项 Unix `SOCK_SEQPACKET` 需要 Linux；4 项因缺少 `cv2` 在收集时跳过；2 项缺少 `NAV2_SOURCE_CACHE`；1 项 Linux Unix peer credentials；1 项 Linux abstract SEQPACKET/`SO_PEERCRED`；1 项缺少 `SIM_CLOCK_SOURCE_CACHE`。
- integration：`test_aiortc_loopback_delivers_a_real_frame`，本机没有 `aiortc`。
- e2e：1 项要求 Linux、已 source 的 ROS 2 Humble 和正在运行的仿真栈。
- ros：79 项 `could not import rclpy`；7 项收集跳过；5 项 opt-in DDS/native-guard；4 项要求新的隔离 Linux Fortress 世界；其余为未选择的 Nav2 bringup、baseline Fortress 或显式 ROS 集成。

ros 组退出码 0 只表示没有失败项。43 个 passed 分布在不依赖 `rclpy` 的文件检查：`test_motion_ingress_loss` 17、`test_motion_low_friction_window` 8、`test_navigation` 8、`test_mapping` 7、`test_motion_anomaly_trace` 2、`test_scan_adapter` 1。这不是 Humble、DDS、aarch64 或插件 ABI 通过。

## 失败节点

复现 unit 全组使用上一节命令和 `pytest-unit-files.txt`。下面四条是精确节点。

### `tests/unit/test_agent_config.py::test_composition_starts_reporting_without_adapter_identity` 的 True/False 两例

```text
tests/unit/test_agent_config.py:121: in test_composition_starts_reporting_without_adapter_identity
    from agent_fakes import FakeConnector, FakeLink, until
E   ModuleNotFoundError: No module named 'agent_fakes'
```

`agent_fakes.py` 在 `tests/integration/`。这次 unit 单独收集时，sys.path 只有测试文件所在的 `tests/unit`。集成组自己的 656 项能够导入该模块。分类：分组造成的导入路径缺口，不是产品逻辑失败，也不是缺第三方包。

分类复核没有改测试，也没有把这两项加进 unit 的 1858：

```text
PYTHONPATH=tests/integration:. .venv/bin/pytest -q -p no:cacheprovider --tb=line \
  --basetemp=/tmp/startup-final-validation-pytest-agent-fakes \
  tests/unit/test_agent_config.py::test_composition_starts_reporting_without_adapter_identity
```

结果：2 passed，0.50s。unit 组原始记录仍是这 2 项 failed。

### `tests/unit/test_limited_manual_rearm.py` 三项

节点：

- `test_completed_stop_uses_raw_wheel_imu_stillness_without_body`
- `test_epoch_change_and_missing_model_remove_permission`
- `test_static_stop_model_does_not_manufacture_raw_stillness`

共同堆栈：

```text
tests/unit/test_limited_manual_rearm.py:37: in context
    p.lag,
E   AttributeError: 'SimulationMotionProfile' object has no attribute 'lag'
```

调用在 `stopped_controller()` → `context()`，三项都没有走到各自的静止断言。当前 `SimulationMotionProfile` 的字段是 `lag_ms: float = 200.0`。`ResponseBounds.actuator_lag` 仍是这个测试传入的浮点参数。HEAD 隔离副本里同一类仍是 `lag: float = 0.2`，HEAD 上的该测试第 37 行仍是 `p.lag`。这是当前工作区把字段改名之后，现有测试仍读旧属性。保护没有执行到断言，不能当成已通过。

### `tests/unit/test_motion_core_event_time.py::test_unified_feedback_does_not_repeat_pair_qualification_at_output`

```text
tests/unit/test_motion_core_event_time.py:111: in test_unified_feedback_does_not_repeat_pair_qualification_at_output
    assert state["healthy"] and state["fault_revision"] == 0, (
E   AssertionError: ("...bounded() got an unexpected keyword argument 'visibility'", [])
E   assert (False)
```

测试把 `rig.core._snapshot` 换成只收 `*args` 的 `bounded`。当前 `MotionCoreOutput._snapshot` 的签名是 `_snapshot(self, ref, wheel, now, perception, pose, *, visibility=None)`，调用点传入 `visibility=`。HEAD 副本的 `_snapshot` 没有这个关键字。TypeError 被记进状态原因，后面的 `tracking`、`sampled_steady` 断言没有执行。这是现有测试替身与当前签名不一致。

## Ruff

当前工作区，同一 `.venv` 的 Ruff 0.17.3，`--no-cache`。HEAD 对照在 `/tmp/startup-final-validation-head`，配置来自同一份 `pyproject.toml` 散列。

| 运行 | 结果 |
| --- | --- |
| 工作区 `ruff check .` | 1 error，退出码 1 |
| 工作区 `ruff format --check .` | 3 files would be reformatted，618 files already formatted，退出码 1 |
| HEAD `ruff check .` | All checks passed! |
| HEAD `ruff format --check .` | 552 files already formatted |

工作区唯一的 check 诊断在保护文件，HEAD 上不存在这条：

`ros2/src/senior_robot/senior_robot/native_receipt.py:314:54` `B905` ``zip()` without an explicit `strict=` parameter``。Ruff 提示的 `strict=False` 被标成 unsafe fix。没有改这个文件。

format 的 3 个文件：

| 文件 | 位置 | 归属 |
| --- | --- | --- |
| `ros2/src/senior_robot/senior_robot/native_receipt.py` | 314 | 保护文件。HEAD 该文件已格式化；当前正文需要拆行 |
| `tests/test_native_receipt_diagnostics.py` | 2 | 保护文件。HEAD 树中没有这个路径 |
| `tests/unit/test_agent_config.py` | 148，函数 `test_composition_starts_reporting_without_adapter_identity` | 不在保护名单。HEAD 该文件已格式化；当前把一行断言写成了带括号的三行，Ruff 要收回成一行 |

因此整仓 check/format 都失败。拿掉两份保护 Python 文件之后，`test_agent_config.py` 仍然未格式化，不能把其余文件说成整仓全绿。diff 在 `ruff-format-protected.diff` 与 `ruff-format.txt`。

## mypy

两边都用工作区 `.venv/bin/mypy` 2.4.0、`--no-incremental`、各自的 `/tmp/startup-final-mypy-*` 缓存，检查 `src/senior_control`。`pyproject.toml` 散列相同，`[tool.mypy]` 为 Python 3.10、`strict = true`。HEAD 在 `git archive HEAD` 解出的 `/tmp/startup-final-validation-head` 里执行。

| 树 | 结果 |
| --- | --- |
| 当前工作区 | 7 errors，5 files，checked 170 source files，退出码 1 |
| HEAD | 68 errors，9 files，checked 159 source files，退出码 1 |

对照键是文件、错误码、完整信息，不含行号。

当前 7 条：

| 文件:行 | 错误码 | 与 HEAD |
| --- | --- | --- |
| `src/senior_control/tasks/lifecycle.py:318` | `assignment` | 同文保留 |
| `src/senior_control/tasks/lifecycle_store.py:647` | `index` | 同文保留 |
| `src/senior_control/motion/constraints.py:189` | `return-value` | 同文保留 |
| `src/senior_control/motion/constraints.py:191` | `arg-type` | 同文保留 |
| `src/senior_control/motion/controller.py:454` | `arg-type` | 同文保留 |
| `src/senior_control/motion/controller.py:456` | `arg-type` | 同文保留 |
| `src/senior_control/agent/websocket.py:148` | `arg-type` | 新增 |

新增这一条是：`Argument 1 to "get" of "dict" has incompatible type "Any | None"; expected "str"`。当前 `_pending` 是 `dict[str, tuple[float, float]]`，第 148 行把 `data.get("nonce")` 传给 `dict.get`；`isinstance(data.get("nonce"), str)` 没有收窄另一次调用的返回值。HEAD 的 `_pending` 是 `tuple[str, float, float] | None`，没有这条。没有改类型，也没有改 mypy 配置。

相对 HEAD 消失 62 条，分布在 `gps_course.py` 21、`startup_learning.py` 26、`self_learning.py` 9、`external_motion_features.py` 4、已删除的 `learning_pose_history.py` 2。消失不等于 mypy 通过；当前 strict 检查仍是 7 条失败。保留的 6 条继续有效。

## 前端

没有跑 Playwright，没有启动 Vite preview 或开发服务器。`vite build` 的 `outDir` 是 `/tmp/startup-final-validation-web-dist`。

| 命令 | 结果 |
| --- | --- |
| `vitest run --reporter=verbose` | 13 files：1 failed，12 passed。169 tests：1 failed，168 passed。9.36s，退出码 1 |
| `tsc --noEmit --pretty false` | 退出码 0 |
| `vite build --outDir /tmp/startup-final-validation-web-dist --emptyOutDir` | 90 modules，built in 1.55s，退出码 0 |

失败节点：`src/features/logs/logs.test.tsx` 的 `bounds the browser window and pauses only its independent subscription`。完整 vitest 与 unit pytest 同时在跑，该用例报 `Test timed out in 5000ms`，堆栈停在该 `it` 的第 64 行。

单独复核，没有改测试，也不替换上面的 168/1：

```text
cd web && CI=1 ./node_modules/.bin/vitest run src/features/logs/logs.test.tsx -t "bounds the browser window"
```

该用例 1038ms passed；同文件另 2 个用例被 `-t` 跳过。完整 vitest 记录仍保留这次超时。

## 包

从冻结清单复制 `pyproject.toml` 与 `src/` 共 173 个文件到 `/tmp/startup-final-validation-pkgsrc`，复制后散列与冻结一致，再在该目录构建。空的 `/tmp/startup-final-validation-uv-cache` 上，`uv build --offline` 因找不到 `hatchling` 失败；随后允许联网，只写入这个隔离缓存。干净 venv 使用工作区解释器创建在 `/tmp/startup-final-validation-wheel-venv`，离线安装同样因空缓存缺少 `aiosqlite` 失败，联网后装入 10 个包。工作区 `.venv` 的已装版本没有改。

| 产物 | SHA256 | 大小 |
| --- | --- | --- |
| `senior_control-0.1.0-py3-none-any.whl` | `de55d6687beb68c06de6d09c2f67faa5277c5bca13eacb92f4d6afa816a4f79d` | 445870 |
| `senior_control-0.1.0.tar.gz` | `5d8a92506ddb90dbd272bc6ab989b74ea042df37227c1a1c47f238570fbb28ba` | 347K |

在 `/tmp` 用干净解释器 `-I` 导入 `senior_control.protocol` 与 `senior_control.config`，`find_spec("shapely")` 为 `None`。打印出的文件是 `/tmp/startup-final-validation-wheel-venv/lib/python3.12/site-packages/senior_control/__init__.py`。这是 macOS / Python 3.12 的打包证据。

## 本次不能完成的验收

以下目标没有执行环境，本次没有通过记录：

- Ubuntu / CI 的 Python 3.10 矩阵。本次只跑了本机 Python 3.12.15。
- ROS 2 Humble、`rclpy`、`colcon`、`/opt/ros`。
- aarch64 车辆或交叉构建。
- 真实 DDS、native-guard 组合、Fortress 世界、已安装 Nav2 bringup。
- 插件 ABI。
- 底盘独立停车的实车验收。e2e 里要求运行中仿真栈的那一项已跳过。
- Linux abstract socket、`SO_PEERCRED`、`SOCK_SEQPACKET`、官方源码缓存、`aiortc`、`cv2`。对应用例是 skipped。

`tests/ros` 的 43 个本地文件检查通过，不代替上列任何一项。
