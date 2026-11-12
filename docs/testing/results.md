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


## 后续仿真集成

| 范围 | 结果 |
| --- | --- |
| 本地回归 | 1190 项 Python 单元和集成测试通过，4 项跳过。154 项 Vitest 和生产构建通过。 |
| HTTPS 配对 | 在本机临时 TLS 端点使用真实 Cloud API 和 CLI 配对。凭据为 0600，重复配对保留原文件。 |
| Gazebo 地图与运动 | 已启动同一份固定运动校验场景，网页和 Nav2 共用其栅格。实测安全门收到合格的激光、轮式里程计和 IMU 证据。 |
| Gazebo 多航点 | `0cec065` 的真实 Nav2 执行确认第一航点成功，第二航点因进展不足失败。后来记录到大部分安全评估票据在新证据或新速度到来后被丢弃。新版提高仿真安全结果读取频率，并在开放导航前预热控制器。冻结版本 `1446ec7` 在 Ubuntu 实际开始执行第一航点，约 6 秒后因运动证据失去时效资格而撤销路线。当时运动证据 ROS 时间年龄为 223 ms，接收时间已不可用。三航点顺序和断线后不续跑的同轮断言未通过。 |
| Gazebo RGB | `66a6bd9` 在 Ubuntu 从实际仿真相机读取到一张新鲜的 640×480 RGB 帧。浏览器端到端收帧另计。 |
| 本阶段边界 | 实车验收跳过。三点全程、实际停稳、浏览器收到 Gazebo 画面和联合故障注入仍需按独立仿真结果核对。 |


## 2026-11-12 仿真故障复核（进行中）

以下将宿主机回归与实际行驶分开记录。实车验证按约定跳过。

| 范围 | 证据与结果 |
| --- | --- |
| Python 回归 | 冻结提交 `f83e871` 的单元与集成测试：1340 项通过、4 项跳过，51.66 秒。包含真实本机 IPC、急停恢复、时序、设备撤销、视频回环、导航取时和运行中受阻资格分离。 |
| 网页设备删除 | 设备列表增加撤销配对、二次确认、提交中状态、错误保留及当前设备移除后的退出。该批网页 167 项测试和生产构建通过。 |
| Ubuntu 原生通信 | 冻结提交 `1e49133` 的运动连续性、最终命令 DDS、速度链测试共 45 项通过，使用 Humble 与实际 ROS 图。 |
| 实际手动驾驶 | 同一冻结版本、独立 Gazebo 运动场景、ROS 域 106：3 秒发送窗口内真值前进 0.115 m。松手后首次零输出 25.3 ms，首次观测车体近零 50.7 ms；连续近零窗口 245 ms。车体观测容差为平移速度 5 mm/s、角速度 0.01 rad/s；这组观测不是实车制动标定。测试经过真实 IPC、仲裁器、安全门、`/cmd_vel` 与独立真值里程计。 |
| 三点导航 | 尚未通过。`1e49133-d375` 完成首点后触发 `command_stall`，该次缺少同时段真值材料，尚不能归因为测量误差。补录 `1e49133-bfda` 的运行进入三个航点，最终真值 x=0.44555 m；ROS 1 秒后的 136/136 份运动证据均合格。实际失败来自 `hard_sweep_collision` 被误作导航资格失效。该次 uncertainty=0.86680 m，生产扫掠几何确与场景 feature 交叠，停止有依据；原直线终点 (1.05,0) 缺乏该安全模型要求的净空。`310f474` 保留安全门停车，允许正在执行的路线重规划，界面安全状态和新任务仍阻止。`3f2fdac` 改用 (.4,-.3)、(.8,-.6)、(1.2,-.9) 的独立净空路线等待实际复验，未改安全参数；即使该路线通过，也不代表原直线能完成或已验证一般绕障。 |
| 全项目原生重建 | `3f2fdac-ede7` 从冻结源重建消息、原生传感器入口、扫描匹配器、仿真与机器人五个项目包。45 项 ROS 测试通过；实际手动前进 0.116 m，松手零输出 8.7 ms，车体近零 34.2 ms、持续 271 ms。净空斜线路径仍未通过：同窗真值重放证实轮转角 −0.128 rad、真实车体转角 −0.01074 rad，轮纵向进展 0.04250 m、真值 0.01510 m，原四类滑移锁存有实际依据；尚未修改四轮接触模型或旧校准。 |
| 图像订阅恢复 | `f83e871-0a3f` 复用源代码未变化的 `3f2fdac` 原生依赖，重建当前 Python 与仿真资源。RGB/Depth/CameraInfo 等传感器契约及两图断开后重订阅恢复共 2 项通过，6.45 秒。桥保留源时间戳，两条图像流按需桥接。该次手动测试在静止预备阶段发生排队截止超时而失去资格，未开始行驶，不能记为通过。 |
| 仿真服务恢复 | 每次隔离运行结束，清理测试服务并恢复原配置服务；已观察到原服务状态为 `active`。 |
| 本轮边界 | 浏览器到云端再到实际 Gazebo 的完整驾驶、地图位置联动及视频联合验收未在本轮完成；本地 IPC 行驶结果不能替代该项。 |

实际运行材料保留于虚拟机 `/home/toby/senior-validation/20261112-1e49133-d375/`。同版补录目录为 `20261112-1e49133-bfda/`。后者的同名临时单元日志含历史 invocation，分析时必须按本次启动筛选；后续验证使用包含独立运行标识的单元名。

本批修复保留原有安全时限、锁存阈值与校准：延后接纳仿真时钟之前到达的定位样本，保留首次接收时间；仿真同一 ROS 时刻的合法输出切换不再被当作回退；导航资格检查耗时不再被算入墙钟偏差，并在 DDS 图查询后重新检查原样本时效。另增加受控的运动重资格命令、worker 预热及仅清理本用户被拒绝连接的残留 socket，仿真实际绑定阶段拒绝覆盖新占用路径。
