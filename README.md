# Senior 小车远程控制

> **2026-11-12 改版说明**：本轮需求与设计见 [车端运动改版批准设计](docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md)。用户已明确目标为 **5 km/h**、已接受的非手动任务不因网页断网而暂停、网页延迟仅作连接信息展示。下文及旧设计/计划包含历史行为，冲突部分不作为本轮开发要求；新版设计已于 2026-11-12 获用户批准，后续修订按用户最新明确要求执行；设计批准不等于产品验收通过。下文保留历史阶段正文，当前阶段与未通过项见[阶段索引](docs/testing/2026-11-14-vehicle-stage-index.md)。

面向 WHEELTEC senior_4wd_dl 的车端 ROS 2 程序、自托管云端和中文网页控制台。阶段一至阶段四已有代码并完成复审，包含控制基础、地图与任务、云端中转、车端代理、视频、网页和自托管部署资产。Linux 上尚未执行 `safety_node`、完整速度链、Nav2、`slam_toolbox` 和任务 21 端到端套件；此前任务 2 的仿真记录和任务 3 的后部遮挡扫描记录保留在本文及 `docs/testing/2026-10-27-linux-simulation.md`，未据此扩展其他通过结论。

速度只走一条链：导航、遥控或脱困进入控制仲裁，再进入独立安全门，最后由安全门发布 `/cmd_vel`。Nav2 和脱困的裸速度暂不接入仲裁：它们没有来源代次和生成时间，无法排除旧动作的迟到消息。`command_node` 独占发布 `/safety/command_in`，自主运动需待具备来源代次和时间戳的桥接实现后才能启用。仿真底盘把 `/cmd_vel` 接到差速控制器，控制器自己还有 0.3 秒命令超时。单独启动 `command_node` 时没有网络入口；车端代理可通过独立的 `agent_adapter` 使用本地 IPC，运行链仍需 Linux 验证。

## 已完成的功能

- 版本化命令信封。校验车辆、会话、连接代次、序号和数值；拒绝 NaN、错误车辆和超大消息。心跳、失联和速度有效期集中配置，单位是米、弧度、秒。
- Ubuntu 22.04 上的 Humble / Gazebo Fortress 仿真。输出雷达、RGB-D、IMU、轮里程计和独立真实位姿。命令停发后底盘看门狗把速度归零。这项已在虚拟机上跑通，记录见 `docs/testing/2026-10-27-linux-simulation.md`。
- 自车过滤。车体后方附件回波被去掉，且不会变成后方空闲；附件外的近墙仍保留。仿真世界是 `rear_occlusion.sdf`。后部遮挡历史仿真记录保留；未接入运行链的地图辅助倒车模块已删除，当前方向许可由实际选中的运动链判断。
- 独立安全门 `safety_node`。按车体矩形预测扫掠，硬边界拦截，预警区允许已观测空闲方向的倒车。传感器过期、扫描运动来源不对或打滑时停车。速度命令按车端单调时钟超时后发零。逻辑有单元测试；这个节点还没有在 Linux 上作为进程跑过。
- `command_node` 管理请求交付、手动来源与导航样本，通过 `/safety/final_command` 接入最终输出侧。软件急停只有 `MotionStateMachine` 一份权威状态；命令侧不镜像锁存。手动接管停掉导航，松开后不自动续跑，旧连接和旧序号不能激活过期速度。
- Nav2 适配。导航适配等待目标终态，接受取消本身不算结束。`navigation.launch.py` 把 Nav2 输出重映射到 `/control/navigation_cmd`，但目前仲裁器不接收无代次的裸速度。单元测试已通过；这个 launch 和 lattice 还没有在 Linux 上加载。
- 长期占据层。重复观测才确认障碍。自由观测不覆盖几何锚点。清空代价地图不能代替清除这一层。
- SLAM 适配。位姿图按后端写入的内容保存，长期占据层是单独的 `occupancy.json`。网格变大不改变地图参考。Humble 2.6.10 上，反序列化没有返回对象就算恢复失败。`slam_toolbox` 还没有在本机加载。
- 地图包导入、导出和清除。一次操作先把日志写稳，再切换位姿图和占据层。跨车导入的 Home 保持待确认，不会覆盖已经采纳的 Home。格式见 `docs/protocol/map-bundle.md`。
- 每台车一个 Home，位姿存在导航点表里。返航先等上一个目标终态。断线只取消断线前已经接受的目标，重连不自动再发。
- GPS 配准和被动航向。没有绝对航向时只给出位置。`navsat_status` 小于 0 的定位被拒绝。RTK 只认显式的 `fixed` 或 `float`。`localization_node` 不发布速度，也不订阅 NavSatFix。`tests/ros/test_relocalization.py` 还没有跑过。
- 本地任务。车端 SQLite 保存任务定义、版本和执行历史。一次扫描最多派出窗口里最新的一次，更早的触发记为跳过。断线不启动、不补跑；执行中断线暂停，重连不自动续跑。一个进程只准备一个 `TaskStore`。规则见 `docs/protocol/scheduling.md`。

## 仓库布局

| 路径 | 内容 |
| --- | --- |
| `src/senior_control` | 协议、配置、存储、安全、地图、导航点、定位和本地任务 |
| `ros2/src/senior_sim` | 仿真模型、世界和 `simulation.launch.py` |
| `ros2/src/senior_robot` | 雷达过滤、安全门、仲裁、Nav2 适配、SLAM 适配、定位节点 |
| `config/simulation.yaml` | 仿真外形与话题，供 Python 配置读取 |
| `config/hardware.example.yaml` | 实车配置示例，不能当作已核对的实车参数 |
| `docs/protocol/v1.md` | 当前命令类型 |
| `docs/protocol/map-bundle.md` | 地图包格式 |
| `docs/protocol/scheduling.md` | 本地任务调度 |
| `docs/superpowers/plans/2026-10-27-robot-control-implementation.md` | 实施计划 |

## 各端部署与启动

| 部署目标 | 当前操作指南 | 范围 |
| --- | --- | --- |
| 云主机 | [云端部署](docs/operations/deploy.md) | 网页、Compose、Caddy、可选 TURN、Jenkins、备份与升级 |
| 实车 Linux 宿主机 / 已有 ROS 容器 | [实车部署](docs/operations/vehicle-deploy.md) | 从克隆、安装 CLI、构建 ROS 环境到配对、部署、联合启停与排障 |
| Linux 仿真机 | [仿真部署](docs/operations/simulation.md) | Humble / Fortress、统一目录、用户服务、地图与恢复边界 |

[部署资产索引](deploy/README.md)说明现有文件用途；[硬件校准与检查](docs/operations/hardware.md)列出实车前置条件。程序启动或云端在线不代表运动资格通过。实车联合启停需要包含 [PR #17](https://github.com/Shunjun/senior-robot-control/pull/17) 修复；[PR #18](https://github.com/Shunjun/senior-robot-control/pull/18) 进一步接入共享运动栈，启动前必须填写已测量的实车参数。

仅克隆不会得到 `senior-robot` 命令。实车宿主机用项目 `.venv` 安装 CLI；实际 ROS / Agent 运行环境用独立 `.ros-venv`。Docker 模式宿主机无需 ROS，不能与容器共用 Python 虚拟环境。各指南先创建 `config.json`，默认位置为当前用户 `~/senior-control/config.json`，并明确何处执行命令。

### 开发与验收入口

控制库开发检查使用项目 [pyproject.toml](pyproject.toml) 和 `uv.lock`；网页构建入口见 [web/package.json](web/package.json)。ROS 传感器契约、独立 launch 与 watchdog 负对照见 [ROS 说明](ros2/README.md)，它们不代替当前部署指南。历史验收记录见 [Linux 仿真记录](docs/testing/2026-10-27-linux-simulation.md)，当前未通过项见[阶段索引](docs/testing/2026-11-14-vehicle-stage-index.md)。

提交前的 Python 格式化使用开发依赖里的 Lefthook 2.2.2。钩子只对已暂存的 `.py` 执行 `uv run --no-sync ruff format`，排版标准仍是根目录 [pyproject.toml](pyproject.toml) 的 `[tool.ruff]`。`ruff check`、mypy 和 pytest 不在这个钩子里。

Git hooks 由同一仓库的所有 worktree 共用，启用后也会作用于其他 worktree。当前 main 尚未包含此配置，先不要安装真实共用 hook；待本功能合并后，在实际开发 checkout 根目录一次性安装并启用：

```sh
uv sync --frozen --group dev
uv run --no-sync lefthook install
```

`uv sync --frozen --group dev` 按锁文件准备带有 Lefthook 和 Ruff 的项目环境。`lefthook install` 把 `pre-commit` 写进该仓库共用的 Git hooks，同一个仓库的其他 worktree 提交时也会走到这个钩子。之后本地 `git commit` 会格式化本次暂存的根目录和子目录 Python，路径可以带空格，并把格式化结果纳入同一次提交。提交时 PATH 里需要有 `uv`；钩子执行 `uv run --no-sync`，不会在提交时重新同步依赖。没有暂存 Python 时这次格式化跳过，其他文件保持原样。暂存的 Python 有语法错误时提交失败，HEAD 不变，已暂存内容保留，终端留下 Ruff 的解析错误。文件只暂存了其中一部分时，Lefthook 先藏起未暂存改动，只格式化暂存版本，再恢复未暂存改动。

只克隆、未执行 `lefthook install` 的仓库不会在提交时自动格式化。

## 任务路线图

当前待办以 [2026-11-07 剩余工作清单](docs/testing/2026-11-07-remaining-work.md) 为准。网页配对单独跟踪为 [Task 22](docs/superpowers/plans/2026-11-07-web-device-pairing.md)。下表保留原计划的模块范围和历史状态。

状态按代码和复审记录。Linux 运行栏只把已经在 Ubuntu 上执行并通过的检查标成已验证。

| 任务 | 功能 | 代码 | Linux 运行 |
| --- | --- | --- | --- |
| 1 | 协议、配置、时钟、SQLite 骨架 | 已完成 | 不依赖 ROS |
| 2 | 仿真底盘、传感器契约、底盘看门狗 | 已完成 | 已验证 |
| 3 | 自车过滤与覆盖判断 | 已完成 | 后部遮挡仿真已跑通 |
| 4 | 矩形扫掠、打滑检测、独立安全门 | 已完成 | 节点未跑 |
| 5 | 控制租约、急停锁定、断线停车 | 已完成 | 速度链未跑 |
| 6 | Nav2 导航与有界脱困 | 已完成 | 未跑 |
| 7 | 稳定障碍与自由空间清除 | 已完成 | 未跑 |
| 8 | SLAM、回环与持续建图 | 已完成 | `slam_toolbox` 未加载 |
| 9 | 地图包导入、导出与清除 | 已完成 | 未跑 |
| 10 | 导航点与 Home | 已完成 | ROS 结果等待未跑 |
| 11 | GPS 配准、重定位与被动航向 | 已完成 | 重定位测试未跑 |
| 12 | 本地任务与调度 | 已完成 | 未跑 |
| 13 | 单管理员认证、配对与多车中转 | 已完成 | 不依赖 ROS |
| 14 | 车端代理、任务与地图同步 | 已完成 | 代理进程入口未组装 |
| 15 | RGB-D 几何与可选 YOLO | 已完成 | 未跑真实传感器 |
| 16 | WebRTC 视频与 TURN | 已完成 | 未跑浏览器全链路 |
| 17 | 网页骨架、登录与选车 | 已完成 | 静态测试可在开发机执行 |
| 18 | 地图、驾驶与重定位界面 | 已完成 | 未跑浏览器全链路 |
| 19 | 任务、地图管理与视频界面 | 已完成 | 未跑浏览器全链路 |
| 20 | 自托管部署与硬件检查 | 已完成 | systemd 未启用 |
| 21 | Linux 全链路仿真验收 | 验收资产已添加 | Linux、Gazebo 与浏览器未执行 |

阶段一至阶段四已有分模块实现和历史复审记录。阶段退出条件写在实施计划对应章节。下面这些 Linux 检查还没有执行，不能记成通过：`safety_node` 进程、整条 ROS 速度链、Nav2 launch 与 lattice、`slam_toolbox`、返航的 ROS 结果等待、`tests/ros/test_relocalization.py` 和任务 21 端到端套件。任务 5 的 ROS 测试在第一次上 Linux 前，还要先补上订阅发现完成后再发命令的等待；当前测试可能在发现完成前丢消息。

### 启动随车回波学习（软件实现，实车仍 Draft）

程序 `start` 初始化且传感/空间条件满足后，会经原控制与避障链提出低速前移 **15 cm** 的学习目标。既有物理车身 polygon 和激光外参必须准确；不再手填 `self_mask_xy`。学习期间只暂定处理误差域严格位于物理边界内的回波，设备后方保持未知。外部障碍、边界歧义或空间不足会等待或中断，不能强行走完；急停、接管、组件重启均不会恢复旧尝试。完整程序的新一次 start 是新的启动尝试。

学习验证不足会报告 `ambiguous`，不会永久屏蔽未验证回波；设备移动/消失后撤销对应局部模型。状态与计程见统一周期诊断 `startup_learning`，原始回波与停车保护保留。本功能尚未通过完整 ROS 构建或实车运动验收。

单天线 GPS 可选。在既有 `gps_input` 中显式配置 `course_frame` 时，该输入按 ENU 的 heading-only Odometry 使用：header 与 child frame 均为指定 ENU frame，pose 为米、twist 为米/秒，平面协方差为对应单位的方差，`origin_family` 必须是原始 GNSS 采集族。程序结合连续轮速/IMU和位置、速度精度积累朝向，输出 `gps_course` 周期诊断；它不自动提供地图朝向或地图变换。15 cm 不保证精度，GPS未就绪不阻止相对运动和学习；默认硬件草稿仍未配置 GPS，也没有 RTK 精度验收证据。


运动路径已收敛：标准实车与仿真均使用共享 MotionCoreOutput；旧网页挑战/校时、旧租约速度仲裁、MotionEvidence 安全门与进展恢复入口已退休。上文若引用已删除测试文件，仅保留为历史验证记录，不能作为当前执行命令。当前清理映射与精确回归见 [路径收敛记录](docs/testing/2026-11-15-motion-path-convergence.md)。完整 Linux 同版重建与运行验收仍 Draft。

软件急停由最终输出侧 `MotionStateMachine` 单独拥有，仅在本次运行内锁存；完整启动不恢复急停文件。网页复位须车端确认并保持旧指令不恢复。实体急停由车辆独立控制，本项目不订阅其状态或要求网页联动复位。详见[硬件与急停职责](docs/operations/hardware.md)。
