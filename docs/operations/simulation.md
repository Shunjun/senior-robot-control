# Linux 仿真部署与运行


本轮仅迁移管理实现，本文原CLI、module启动名与安装命令保持兼容。
生产平台绑定仍需本项目完整SDK与既有ROS/媒体/IPC owner，不能仅安装vehicle-manager替换当前运行环境；
独立manager的配置解析/计划渲染与旧SDK的help/dry-run检查不代表独立生产部署验收。
仿真总装配launch目前仍由senior_robot安装与调用，归senior_sim的收尾将在后续单独验证。

本页只面向 Linux 仿真机。云服务先按[云端部署](deploy.md)配置；实车用[实车部署](vehicle-deploy.md)，不要把本页的 Gazebo 真值或运动恢复操作用于实车。浏览器与仿真机都连接真实云域名，跨机器不使用各自的 `localhost`。

## 1. 准备 Linux 环境与仓库

需要带 systemd 用户会话的 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。历史传感器契约环境为 Ubuntu 22.04.5 aarch64 / Fortress 6.18.0，该记录不代表当前完整安全、导航链已通过。需要预先安装对应官方 ROS / Gazebo 二进制及 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`、`controller_manager`、`diff_drive_controller`、`joint_state_broadcaster`、`robot_state_publisher`、`xacro`，并提供 Python 3.10 venv/pip、rosdep、colcon 和本机构建工具。无窗口模式还需要 `xvfb`、`xauth`、`mesa-utils` 与可用的渲染环境。

下列命令供操作者在仿真机执行，从项目根目录开始。只有克隆仓库还没有 CLI；CLI 单文件安装到 PATH，Python 运行仅使用项目 `.venv`，需包含原 ROS/仿真依赖；原监督进程和 Agent 由该解释器启动，不另设 CLI 虚拟环境。

```sh
git clone https://github.com/Shunjun/senior-robot-control.git
cd senior-robot-control
sudo install -m 0755 bin/senior-robot.sh /usr/local/bin/senior-robot
source /opt/ros/humble/setup.bash
python3.10 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -e '.[agent,safety,ros]'
rosdep update --rosdistro humble
rosdep install --from-paths ros2/src --ignore-src -r -y --rosdistro humble
cd ros2
../.venv/bin/python -m colcon build --packages-up-to senior_sim senior_robot
source install/setup.bash
cd ..
```

ROS 的 `rclpy` 由已安装的 ROS 提供，`--system-site-packages` 使该环境可访问它。用 `.venv` 的解释器构建，避免系统 colcon 生成的入口找不到项目模块。每个新终端加载 ROS 底层与本项目 `ros2/install/setup.bash`。具体 ROS 传感器契约与手动独立 launch 见 [ros2/README.md](../../ros2/README.md)；不要与下文 CLI 管理的同一实例重复启动。

## 2. 先创建统一目录与配置

升级涉及消息删除或重命名时，先参考 [ROS 接口定向重建与启动前检查](ros-interface-rebuild.md)，在本机实际路径执行，重建目标保留 `senior_sim senior_robot`；该检查不启动仿真或 ROS 节点。

仍在仿真机项目根目录，以当前普通用户执行，后续 pair/start/stop/status 都不要 sudo。默认读取当前用户 `~/senior-control/config.json`，CLI 不会自动生成它。多实例必须有独立私有目录、配对身份与未占用 ROS 域；下列 73 只是仿真示例，需核对未占用。

```sh
mkdir -p "$HOME/senior-control"
chmod 700 "$HOME/senior-control"
```

在该目录手工创建 `config.json`，例如（将 `/home/sim` 改为本用户实际绝对路径，域名改为实际 WSS 地址）：

```json
{
  "kind": "simulation",
  "project_dir": "/home/sim/senior-robot-control",
  "ros_setup": "/home/sim/senior-robot-control/ros2/install/setup.bash",
  "cloud_url": "wss://robot.example.com/ws/agent",
  "ros_domain": 73
}
```

`ros_setup` 可省略，默认由 `project_dir/ros2/install/setup.bash` 推导。不要逐项配置其他目录的 credentials/database/socket，不要将 `config/simulation.yaml` 的硬件/运动参数当成此 JSON。前者供 ROS/Python 配置读取，后者才是 CLI 运行配置。仿真与硬件参数必须分开。

统一目录保存 `device.json`（凭据 0600）、`agent.db`（含可能的 WAL/SHM）、`agent.sock`、仿真锁和 `ros-home/`（ROS 日志在 `ros-home/log/`）。软件急停仅当前进程内锁存，完整 start 不恢复或写入急停文件；旧文件保持原样。启动不恢复旧手动授权或旧任务运动。目录应为 0700，备份也要保护凭据与数据库。

## 3. 配对、检查与启停

先在云端网页为此仿真实例生成一次性配对码，再在仿真机项目根目录执行，按交互提示输入；不要将配对码或 token 写进命令行或文档。已有凭据拒绝覆盖；旧凭据被删除且无备份时要重新配对。

```sh
senior-robot pair --cloud-url wss://robot.example.com/ws/agent --name '隔离仿真'
senior-robot doctor
senior-robot start
senior-robot status
senior-robot stop
```

以上默认使用 `~/senior-control`。自选目录时，每条命令都显式追加 `--control-dir /home/sim/another-sim`，并先在该目录创建 `config.json`。仿真无需执行实车 `deploy`；`start` 通过 `simulation_cli.py` 创建 systemd 用户服务，先启动 ROS 仿真入口并完成适配器握手，随后启动 Agent，停止先 Agent 后本次仿真。它不管理其他实例或厂商进程。

`doctor` 不启动 Gazebo、设备或 ROS 节点。另需检查 ROS 参数文件时，可在项目根目录执行 `.venv/bin/python tools/doctor.py config/simulation.yaml`；没有运行 ROS 图时实时项为 `not_run`，不代表通过。检查和构建不会自动下载模型权重。

查看日志也在仿真宿主机，以同一普通用户执行：

```sh
# 先列出实际用户服务，再把名称替换为该实例的真实 unit：
systemctl --user list-units --all 'senior-sim-*'
unit='senior-sim-替换为实际实例哈希.service'
journalctl --user -u "$unit" -n 100 --no-pager
```

`status` 的 Process、ROS IPC、Cloud 分别表示进程、身份探测与本次运行最近连接事件；`last event connected` 不是持续在线、地图有效或运动资格证明。启动失败先检查构建、ROS 安装环境、用户会话、独立域、凭据、私有目录权限和显示环境，不盲删 socket 或急停文件。

## 4. 图形窗口与实例隔离

`start` 默认等同于 `start --headless true`，通过 Xvfb 无窗口运行。要打开本机 Gazebo
窗口，在 Linux 图形桌面的终端执行：

```sh
senior-robot start --headless false
```

窗口模式绕过 Xvfb，将当前终端的 `DISPLAY`、`XAUTHORITY` 及相关桌面会话环境传入
systemd 用户服务；缺少 `DISPLAY` 时会明确报错。需要可访问的 X11 或 XWayland 显示和
相应认证权限。纯 SSH 会话可继续使用默认无窗口模式。模式是本次启动参数，无需修改
JSON 配置。若同一配置已经启动，命令只提示正在运行；切换模式须先 `stop` 再重新 `start`，
不会额外启动第二套仿真。

`start` 需要可用的 systemd 用户会话。仿真启动后，命令先核对 ROS 适配器身份，再启动
Agent。停止时先给 Agent 时间完成停车与关停，再退出本次服务的仿真进程。软件急停仅本次运行有效。状态分别显示进程、IPC 身份核验和该次运行最新的云端连接事件。
若强制停止遗留 IPC socket，下一次启动只会在取得实例目录和 ROS 域两把锁后，清理
当前用户拥有且明确拒绝连接、检查期间身份未变化的旧 socket。活动连接、权限问题、
超时、非 socket 路径或路径被替换时仍拒绝启动；不会直接覆盖占用路径。

### 工作台实时地图

`senior-robot start` 的仿真档按 1 倍时间启动标准 Gazebo 输入，并通过 `navigation_enabled:=true` 编排原装 map_server、AMCL 和 Nav2。地图资产由 `senior_sim` 的 `world_reference` 脚本从固定 SDF 生成到本次运行的私有目录，退出时清理；生成通用 `navigation_reference.json`，Agent 使用与实车相同的地图/定位接入。
定位来自标准 `/amcl_pose`，真值 `/ground_truth/odom` 仅用于环境诊断，不授予控制或导航资格。只有地图、定位、传感器和任务生命周期满足通用检查时才能使用导航。默认不启动 `slam_toolbox`。
每次导航独立启动控制器和 BT 节点，结束或撤销权限后回收；底盘 drive 在环境中只启动一次。私有导航 Twist 经过项目唯一最终输出 owner 发布 `/cmd_vel`，原装 drive 使用 unstamped Twist 订阅。其他场景和实车仍使用各自明确配置的地图和定位来源。

网页地图通过状态快照接收地图坐标系中的车辆位置、朝向和实时激光命中点。
Agent 沿用每 0.25 秒一次的状态轮询，位置、至多 64 个激光点或导航阻断原因变化时，
在下一次成功轮询后推送一份完整快照；位置失效、点集清空和代次变化同样触发更新。
周期推送每轮最多一帧，状态未变化时仍每 2 秒发送健康快照，不新增 ROS 采样或轮询。
这里的 0.25 秒是轮询间隔，收集、排队、编码与网络传输仍会增加延迟，不能视为
端到端最大延迟承诺。
底图呈现固定场景几何，紫色扫描层随观测更新。单次扫描最多抽样 64 个命中点；
扫描与定位时间相差超过 150 ms 或数据过期时清空扫描层。断线、定位失效及地图
代次变化时隐藏对应的实时叠加，并提示位置不可用。同步更新云端、Agent、ROS 构建
和 Web，以便整条链路识别新增的状态字段。导航不可用时，快照携带车端检查未通过的
原因，网页分别提示地图定位、规划控制器、传感器证据或心跳权限问题。

### 运动状态与恢复

标准仿真入口通过 `hardware_motion_profile` 显式加载 `senior_sim/config/motion_profile.json`，格式与实车配置相同。文件中的环境候选数值只是未验证假设，不代表实车能力；控制层没有仿真许可、倍率补偿或旧协议回退。`use_sim_time` 通过启动参数传给各节点。世界 reset 由环境运行生命周期管理，标准消息无法观察的同值 reset 不提供自动识别保证。

通过 `/motion/core_reference` 查看原生来源与统一估计，`/motion/runtime` 查看当前运动模式、急停和恢复条件。传感器时效、原始来源身份和本地指令 TTL 各自保持有效；普通障碍和估计退化按当前核心状态处理。急停与执行故障使用工作台明确恢复操作，收到确认后重新准备手动 authority；不会恢复旧杆量或自动续跑手动输入。

### 仿真视频

要在浏览器查看仿真车辆的 RGB 摄像头，在仿真档 JSON 中设置 `"video": true`，
并让运行 Agent 的 Python 环境安装项目的 `video` 可选依赖。仿真和 Agent 共享
本次运行专用的私有最新帧，视频继续经现有 WebRTC 通道协商。

## 源码依据与验收边界

配置解析见 [cli.py](../../src/senior_control/cli.py)，用户服务、握手、隔离与恢复见 [simulation_cli.py](../../src/senior_control/simulation_cli.py)，场景/参考入口见 [simulation_motion.launch.py](../../ros2/src/senior_robot/launch/simulation_motion.launch.py) 和 [world_reference.py](../../ros2/src/senior_sim/scripts/world_reference.py)。地图栅格来自 SDF 的 环境 `baseline_grid` 固定场景生成工具，不是已经完成在线 SLAM 的证明。完整 Linux 渲染、启停、地图、导航、视频与运动恢复仍需对应运行环境验收，静态文档更新不增加通过结论。


### 录制与离线分析

`tools/vehicle_motion_capture.py` 只读订阅，环境录制显式传 `--use-sim-time`，实车使用默认标准系统 ROS 时钟。当前记录 schema 为 2：带测量 header 的行使用 `time` 的五个必填整数域 `stamp_ns`、`received_ros_ns`、`received_steady_ns`、`sample_steady_ns`、`clock_generation`；派生扫描另保留完整 `first_time` / `last_time`，后续回放不重新映射。`received_steady_ns` 表示本次录制回调，与原始测量记录分开。`/clock` 行仅记 `data.clock_ros_ns` 与回调 steady，不声称测量身份。无 header 的状态/命令只保留回调时间。

旧捕获文件没有自动兼容入口；缺原始 receipt 的历史几何只能作为明确标记的合成数据，不能证明采集年龄。`vehicle_motion_analyze.py` 保留故障窗口和区间平均分析，不给出物理停车或整车资格结论；录制缺流、解码/时间错误会明确报告 incomplete。离线 `scan_matcher_replay.py` 保留 capture 时间记录并按原顺序输出几何输入，不模拟 DDS/TF 接纳或重新证明年龄。

Shell `senior-robot restart` 仅在原 stop 成功后 start，`senior-robot logs --follow` 直接读取同一 transient user unit journal。此仿真仍是宿主机模式，不支持 docker 分组仿真。
