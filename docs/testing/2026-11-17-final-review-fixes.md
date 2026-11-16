# 最终审阅失败点修复与受影响项目复验

日期：2026-11-17（Asia/Shanghai；原始日志用 UTC）。HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`，工作区未提交。本轮只修复已定位失败，不新增测试用例、不调整超时、不跳过失败、不启动开发服务、不暂存、提交或推送。

原全量报告 [startup-final-validation](2026-11-17-startup-final-validation.md) 及其全部失败证据保留；本报告不把局部复验替换成原全量 Python 通过，也没有例行重跑全套 Python。

## 本轮快照及精确增量

开始时冻结 755 个文件，范围沿用原全量报告：`src/`、`tests/`、`ros2/`、`web/`、`config/`、`tools/`、`pyproject.toml`、`uv.lock`、`.github/workflows/backend.yml` 和三份保护文件，排除虚拟环境、依赖及缓存目录。本轮起始快照与原全量验证快照逐文件比较，changed/missing/added 均为空。

- 起始指纹：[before-fingerprints.json](2026-11-17-final-review-fixes-before-fingerprints.json)，本轮清单 SHA256 `ff10f4de43e6167a87feed12c57629e6d62e4e83965fd3c8b7a2d35866f7fadf`。
- 最终指纹：[fingerprints.json](2026-11-17-final-review-fixes-fingerprints.json)，本轮清单 SHA256 `d828c0878e38eab11161c58f16e293a194d28ae49492a8b5f7589bb26a5a8d5e`。
- 相对本轮开始的精确改动：[owned.diff](2026-11-17-final-review-fixes-owned.diff)。这是当前快照增量，不是相对 HEAD 的整个脏工作区 diff。

清单散列算法为对文件记录（path、sha256、bytes，路径排序）执行 `json.dumps(records, sort_keys=True)` 后 SHA256；因此不直接比较原报告采用另一种序列化方法的清单汇总散列。

仅改动以下四个源/测试文件，无新增、删除源文件：

| 文件 | 修复及保留行为 |
| --- | --- |
| `tests/unit/test_limited_manual_rearm.py` | `p.lag_ms / 1000` 传入 `ResponseBounds`；三项原始轮速/IMU 静止、epoch/缺失模型、静态模型不制造静止证据的断言均保留，没有恢复旧产品字段 |
| `tests/unit/test_motion_core_event_time.py` | `bounded(*args, visibility=None)` 把该关键字传给真实 snapshot；保留真实 tracking proposal、fallback、observe/healthy 仅一次、sample 时间及实际输出越过 deadline 的断言，没有 patch 结果 |
| `src/senior_control/agent/websocket.py` | 先取本地 `nonce`，`isinstance` 收窄后查 `_pending.get(nonce)`，校验通过后用同一 `nonce` 删除；未增加 Any/cast/ignore，非字符串仍拒绝，字段集合、epoch、robot、sent_at、server_time、时效校验均保留 |
| `tests/unit/test_agent_config.py` | 使用 `tests.integration.agent_fakes` 明确引用现有模块；pytest 配置已有根目录 `.` 路径，不需要额外 PYTHONPATH 或 integration 收集；fixture 无复制，测试路径配置未改；唯一非保护格式问题交由 Ruff 要求合并断言，导入按 Ruff 排序 |

三保护逐文件 SHA256 与本轮起始相同，详见 [source-delta.json](2026-11-17-final-review-fixes-evidence/source-delta.json)：

- `ros2/src/senior_robot/senior_robot/native_receipt.py`：`53d08230fb94dce972ed72dbf6fb16532d3b34aea4e5ed7191772a9bd0ef910a`
- `tests/test_native_receipt_diagnostics.py`：`5c33cba2555e0c0b9b0096a025620fbfdb9ef86ef03bfee53e485dc43c440408`
- `docs/reviews/2026-11-16-software-estop-owner.md`：`cf258266704168979f6485658e6c8e08d18bd444e6a1175cfc2f848ff5cab4d1`

## 现有失败节点与实际相关回归

环境沿用工作区 `.venv`：Python 3.12.15、pytest 8.4.2、mypy 2.4.0、Ruff 0.17.3；没有升级依赖。日志逐项包含精确命令、cwd、时间、退出码、耗时。

| 运行 | 结果 | 证据 |
| --- | --- | --- |
| 五个受影响/相关 unit 文件，独立于 integration 收集，无额外 PYTHONPATH | **41 passed**，2.60s，退出码 0 | [log](2026-11-17-final-review-fixes-evidence/pytest-unit-unrestricted.log)、[JUnit](2026-11-17-final-review-fixes-evidence/pytest-unit-unrestricted.xml) |
| 真实 WebSocket 延迟回复/多 nonce/send cadence 集成节点 | **1 passed**，0.27s，退出码 0 | [log](2026-11-17-final-review-fixes-evidence/pytest-integration-websocket.log)、[JUnit](2026-11-17-final-review-fixes-evidence/pytest-integration-websocket.xml) |

unit 命令的文件列表：`tests/unit/test_agent_config.py tests/unit/test_limited_manual_rearm.py tests/unit/test_motion_core_event_time.py tests/unit/test_websocket_link.py tests/unit/test_heartbeat_wire.py`。原报告 6 个失败实例均包含其中并执行至原断言；两项导入失败未新增场景。WebSocket 文件原有错误 nonce/epoch/robot/sent_at 拒绝、非有限时间拒绝、合法回复、收发视频、认证连接及心跳失效回归一并执行。

集成精确节点为 `tests/integration/test_agent.py::test_delayed_replies_preserve_send_cadence_and_connection`。

首次在沙箱内运行同一 unit 文件组为 **39 passed / 2 failed**，两项失败分别来自 Unix socket 和 `127.0.0.1:0` TCP socket 的 `Operation not permitted`，见 [首次 log](2026-11-17-final-review-fixes-evidence/pytest-unit.log)。解除测试 socket 的沙箱限制后执行上述 41 passed；未修改产品或测试来规避环境失败，没有启动开发服务。

## 静态检查

| 运行 | 结果 |
| --- | --- |
| `.venv/bin/mypy --no-incremental --cache-dir /tmp/final-review-fixes-mypy src` | **6 errors / 4 files / 170 source files**，退出码 1；仅剩原债 |
| Ruff check，排除三保护 | **All checks passed**，退出码 0 |
| Ruff format --check，排除三保护 | **619 files already formatted**，退出码 0 |

[mypy 原始日志](2026-11-17-final-review-fixes-evidence/mypy-worktree.log) 与原报告同版本的 worktree/HEAD 原始日志按「文件、错误码、完整信息，不含行号」比较，见 [mypy-comparison.json](2026-11-17-final-review-fixes-evidence/mypy-comparison.json)。6 条均与原 HEAD 相同，工作区由 7 减至 6，只消除 websocket 的 nonce 类型错误，无新增；没有重新跑旧 HEAD 检查来重复已有效证据。

保留原债：`tasks/lifecycle.py:318 [assignment]`、`tasks/lifecycle_store.py:647 [index]`、`motion/constraints.py:189 [return-value]`、`:191 [arg-type]`、`motion/controller.py:454 [arg-type]`、`:456 [arg-type]`。因此全 src mypy 仍非全绿。

Ruff 精确命令为 `.venv/bin/ruff check --no-cache --extend-exclude ros2/src/senior_robot/senior_robot/native_receipt.py,tests/test_native_receipt_diagnostics.py,docs/reviews/2026-11-16-software-estop-owner.md .`，format 使用同样参数并增加 `--check`。见 [最终 check](2026-11-17-final-review-fixes-evidence/ruff-check-final.log)、[最终 format](2026-11-17-final-review-fixes-evidence/ruff-format-final.log)。首次 import 调整引发 I001 的日志也保留在 `ruff-check.log`，排序修复后复验通过。

## 前端全量单独复验

前端源码及 timeout 未改。在 Python 测试和 mypy 启动前单独执行 `cd web && CI=1 ./node_modules/.bin/vitest run --reporter=verbose`，全量 **13 files / 169 tests passed**，无跳过，退出码 0，Vitest 耗时 3.61s，见 [vitest.log](2026-11-17-final-review-fixes-evidence/vitest.log)。原失败 `bounds the browser window and pauses only its independent subscription` 耗时 **1433ms**，低于现有 5000ms 限制。

本轮验证该快照在没有 Python 重负载并行时全量可以成功，不把它当作对上轮超时因果的确定证明；原全量超时记录继续保留。前端未改，没有重复执行已通过的 tsc/vite build，也没有启动 preview、Vite 或 Playwright 服务。

## 源变化后的同版隔离包验证

从本轮最终源复制 `pyproject.toml` 与 `src/` 共 173 个文件到 `/tmp/final-review-fixes-pkgsrc`，逐文件校验复制前后 SHA256，见 [package-source-fingerprints.json](2026-11-17-final-review-fixes-evidence/package-source-fingerprints.json)。`uv build --offline --out-dir /tmp/final-review-fixes-dist --cache-dir /tmp/startup-final-validation-uv-cache` 成功，复用上轮 hatchling 缓存；见 [build.log](2026-11-17-final-review-fixes-evidence/build.log)。

新虚拟环境为 `/tmp/final-review-fixes-wheel-venv`。最初 `uv venv` 使用默认用户缓存被沙箱拒绝，见 `clean-venv.log`；指定已有 `/tmp` 缓存后成功，见 [clean-venv-retry.log](2026-11-17-final-review-fixes-evidence/clean-venv-retry.log)。依赖从上轮有效缓存离线安装，没有安装或修改工作区 `.venv`。

首次缓存解析的安装与基础导入均成功，见 [install.log](2026-11-17-final-review-fixes-evidence/install.log)、[clean-import.log](2026-11-17-final-review-fixes-evidence/clean-import.log)。使用 `/tmp` 下新解释器 `-I` 导入 `senior_control.protocol`、`senior_control.config`，确认路径在新 venv 的 site-packages，且 `find_spec("shapely") is None`。此外直接比对 wheel 内 `senior_control/agent/websocket.py` 与当前源 SHA256 一致，见 [wheel-source-match.json](2026-11-17-final-review-fixes-evidence/wheel-source-match.json)。

| 产物 | SHA256 | bytes |
| --- | --- | --- |
| `senior_control-0.1.0-py3-none-any.whl` | `e387f1b4227747eecd75a4ec1ed2a795eedcb939fb66fb6e7a0566d2669e4011` | 445867 |
| `senior_control-0.1.0.tar.gz` | `b9b3012cd8469f5a4613c8c43181b92805f1fd0104bbbb21da4de251d1166bb9` | 355773 |

产物散列另存 [artifacts.json](2026-11-17-final-review-fixes-evidence/artifacts.json)。这是本机 macOS/Python 3.12 基础包验证，不覆盖 Linux、ROS、DDS、车辆或插件 ABI；原全量报告列出的环境缺口继续有效。

依赖版本的边界：本轮 wheel/源码版本一致，但离线解析得到 `pydantic 2.14.2 / pydantic-core 2.50.2 / pydantic-settings 2.16.2 / typing-inspection 0.4.4`，原干净 venv 分别为 `2.14.3 / 2.50.4 / 2.16.3 / 0.4.5`，其他基础依赖版本一致。为明确这个差异，额外尝试了原环境的[精确约束](2026-11-17-final-review-fixes-evidence/package-constraints.txt)，缓存解析报告没有 `pydantic-settings==2.16.3`，退出码 1，见 [约束安装失败日志](2026-11-17-final-review-fixes-evidence/install-same-dependency-versions.log)；未联网补包，也未把这次失败记成通过。

该额外约束失败不改变已完成的本轮同源码隔离构建/干净基础导入结果；不能宣称两次包验证使用完全相同的依赖版本。约束失败后重新确认当前干净环境导入成功并打印实际依赖版本，见 [最终基础导入日志](2026-11-17-final-review-fixes-evidence/clean-import-final.log)。原全量报告及证据共 67 个路径另存[保留证据散列清单](2026-11-17-final-review-fixes-evidence/preserved-original-evidence-fingerprints.json)，全部原始文件未改动。
