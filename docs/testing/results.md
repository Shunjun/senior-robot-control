# 任务 21 验收结果

执行日期：2026-10-30。代码版本：基线 `8e89c60d14d90e41af2f143966aaa0a23fc6c70c` 加任务 21 工作树。环境：macOS 26.4（Build 25E246），Python 3.12.15（`uv run`），Node 24.14.1，npm 11.11.0。配置：`config/simulation.yaml`，SHA-256 `f35d68d96c4b038f22239a898898a8079229a8309bf6e75bb3d3b85ea913f8ab`。

“未执行”不表示通过。仓库未生成或未收到的日志、截图、录屏均明确写为缺失。

| 检查 | 状态与结果 | 代码/环境/配置 | 日志或截图路径 |
| --- | --- | --- | --- |
| RED：`test_rear_occlusion_requires_bounded_map_assisted_reverse` | 预期失败：`AssertionError: wall_escape acceptance scene is missing` | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| 本机任务 21 纯逻辑断言 | 通过：`8 passed in 0.32s` | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| 任务 21 全目录收集 | `8 passed, 1 skipped in 0.32s`；跳过项为需要 Linux/ROS/运行中仿真栈的 `/cmd_vel` 发布者检查 | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| 新增 Python 模块 mypy | 通过：`Success: no issues found in 8 source files` | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| 新增 Python 模块 Ruff | 通过：lint 与格式检查均退出码 0 | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| Python unit + integration | 退出码 0：`386 passed, 1 skipped, 1 warning in 15.33s`；警告来自配对测试结束时 aiosqlite 工作线程向已关闭事件循环回调 | 上述工作树 / macOS / 上述配置哈希 | 缺失（仅保留命令输出） |
| Web 单元测试 | 退出码 0：`6 passed`，`100 passed` | 上述工作树 / macOS、Node/npm / 上述配置哈希 | 缺失（仅保留命令输出） |
| Web 构建 | 退出码 0：TypeScript 检查和 Vite 构建完成 | 上述工作树 / macOS、Node/npm / 上述配置哈希 | `web/dist/`（构建产物，不是验收截图） |
| Linux ROS + E2E | **未执行**：当前主机缺少 Linux、Humble、Gazebo Fortress 和运行中的完整栈 | 上述工作树 / macOS / 上述配置哈希 | 缺失 |
| Playwright CLI 入口 | 退出码 0：`--help` 显示 Playwright Test 帮助；`--list --project=chromium` 从 `web/e2e` 收集 2 个用例；两个命令均未运行用例或启动浏览器 | 上述工作树 / macOS、Node/npm / 上述配置哈希 | 缺失（仅保留命令输出） |
| Playwright Chromium | **未执行**：未提供 `base_url`，且未启动服务器、浏览器或下载浏览器二进制 | 上述工作树 / macOS、Node/npm / 上述配置哈希 | 桌面、手机横屏、手机竖屏截图均缺失 |
| 故障注入全链路 | **未执行**：半开连接、进程重启、时钟暂停/回退、损坏上传、GPS 跳点、感知过期、视频拥塞均未在完整栈注入 | 上述工作树 / macOS / 上述配置哈希 | 日志、录像均缺失 |
| 实车验收 | **未执行**：无实车、场地和现场安全员 | 上述工作树 / macOS / 上述配置哈希 | 日志、录像、签字均缺失 |

## 已执行命令

```sh
uv run pytest tests/e2e/test_recovery.py::test_rear_occlusion_requires_bounded_map_assisted_reverse -q
uv run pytest tests/e2e/test_recovery.py tests/e2e/test_maps.py tests/e2e/test_tasks.py tests/e2e/test_home.py tests/e2e/test_gps.py tests/e2e/test_occlusion.py -q
uv run pytest tests/e2e -q
uv run mypy tests/e2e/conftest.py tests/e2e/test_recovery.py tests/e2e/test_maps.py tests/e2e/test_tasks.py tests/e2e/test_home.py tests/e2e/test_gps.py tests/e2e/test_drive.py tests/e2e/test_occlusion.py
uv run ruff check tests/e2e
uv run ruff format --check tests/e2e
uv run pytest tests/unit tests/integration -q
npm --prefix web test -- --run
npm --prefix web run build
npm --prefix web run test:e2e -- --help
npm --prefix web run test:e2e -- --list --project=chromium
```

Python 全量测试首次在受限沙箱中因 Unix socket `bind` 返回 `PermissionError`，结果为 `381 passed, 1 skipped, 1 failed, 4 errors`。解除沙箱限制后以完全相同的命令复跑，得到上表退出码 0 的结果；复跑没有启动服务。

## 未执行命令

Linux 命令按要求只记录：

```sh
python3 -m pytest tests/ros tests/e2e --simulation-profile config/simulation.yaml -q
```

浏览器命令按要求只记录：

```sh
PLAYWRIGHT_BASE_URL=https://control.example.test \
SENIOR_CONTROL_PASSWORD='从安全渠道读取' \
npm --prefix web run test:e2e -- --project=chromium
```

没有启动用户 VM、Gazebo、coturn、uvicorn、Vite 或浏览器；没有启用 systemd 单元。`safety_node` 仍是设计中唯一允许发布 `/cmd_vel` 的节点，该进程和完整速度链本次没有在 Linux 上运行。


## 2026-11-08 驾驶工作台与遥控时序

| 范围 | 证据 |
| --- | --- |
| Python 单元与集成 | `2eb899a` 在 macOS 使用本地套接字和回环连接复跑，1173 项通过，4 项跳过。 |
| Web 与视口 | `17b75c1` 的 149 项 Vitest 和生产构建通过。Chrome 加载演示构建，在 1440×900、390×844 和 844×390 下确认视频区域、双摇杆、停车和急停同时在视口内，文档无需纵向滚动。竖屏视频区域高约 234 px。 |
| 遥控时序 | Python 集成夹具使用可控时钟，覆盖 400 ms 单向延迟下的接纳，以及 IPC 排队后累计 510 ms 的拒绝。首次同步的时钟偏差区间超过 250 ms 时，保守地拒绝新时序驾驶。 |
| Ubuntu 原生 ROS | Ubuntu 22.04 的隔离工作区 `operator-2eb899a` 复用 Humble 构建，重建消息、原生守卫和 Python 节点。27 项中 26 项通过，包含实际安全输出回执。Nav2 进程因路由模块间接依赖 Agent 的 `aiosqlite` 未能启动。在 `operator-1ff930d` 用拆分依赖后的源码复跑，27 项全部通过，包括使用合成地图的 Nav2 启动和发布者检查。 |
| 仿真 CLI | Ubuntu 同一隔离工作区通过启动、重复启动、状态查询和停止。ROS IPC 身份通过核验，停止后临时服务和套接字已退出。此测试的云端地址指向未开放的本机端口。 |
| 生产地图和导航 | 地图包只读展示及带地图代次的任务编辑协议已有测试。生产 ROS 尚未提供同一地图引用下的实时占据栅格和定位代次，生产导航保持禁用。 |
| 现场与联合验收 | 本阶段只在 Ubuntu 仿真验收。仿真视频联控、实时地图上的三点路线、取消与断线、故障注入下的停稳测量，以及隔离 HTTPS 配对仍待执行。实车验证（含实车 systemd 生命周期和停稳测量）按约定跳过。 |
