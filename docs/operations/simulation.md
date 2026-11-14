# Linux 仿真部署与运行

本页只面向 Linux 仿真机。云服务先按[云端部署](deploy.md)配置；实车用[实车部署](vehicle-deploy.md)，不要把本页的 Gazebo 真值、仿真急停初始化或运动恢复操作用于实车。浏览器与仿真机都连接真实云域名，跨机器不使用各自的 `localhost`。

## 1. 准备 Linux 环境与仓库

需要带 systemd 用户会话的 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。历史传感器契约环境为 Ubuntu 22.04.5 aarch64 / Fortress 6.18.0，该记录不代表当前完整安全、导航链已通过。需要预先安装对应官方 ROS / Gazebo 二进制及 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`、`controller_manager`、`diff_drive_controller`、`joint_state_broadcaster`、`robot_state_publisher`、`xacro`，并提供 Python 3.10 venv/pip、rosdep、colcon 和本机构建工具。无窗口模式还需要 `xvfb`、`xauth`、`mesa-utils` 与可用的渲染环境。

下列命令供操作者在仿真机执行，从项目根目录开始。只有克隆仓库还没有 CLI；本页使用 ROS 运行环境 `.ros-venv` 安装并执行 CLI，因为仿真监督进程和 Agent 使用启动 CLI 的 Python。不要用实车宿主机那份不含 ROS 的 `.venv` 代替它。

```sh
git clone https://github.com/Shunjun/senior-robot-control.git
cd senior-robot-control
source /opt/ros/humble/setup.bash
python3.10 -m venv --system-site-packages .ros-venv
.ros-venv/bin/python -m pip install -e '.[agent,safety,ros]'
rosdep update --rosdistro humble
rosdep install --from-paths ros2/src --ignore-src -r -y --rosdistro humble
cd ros2
../.ros-venv/bin/python -m colcon build --packages-up-to senior_sim senior_robot
source install/setup.bash
cd ..
```

ROS 的 `rclpy` 由已安装的 ROS 提供，`--system-site-packages` 使该环境可访问它。用 `.ros-venv` 的解释器构建，避免系统 colcon 生成的入口找不到项目模块。每个新终端加载 ROS 底层与本项目 `ros2/install/setup.bash`。具体 ROS 传感器契约与手动独立 launch 见 [ros2/README.md](../../ros2/README.md)；不要与下文 CLI 管理的同一实例重复启动。

## 2. 先创建统一目录与配置

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

`ros_setup` 可省略，默认由 `project_dir/ros2/install/setup.bash` 推导。不要逐项配置其他目录的 credentials/database/socket/estop_state，不要将 `config/simulation.yaml` 的硬件/运动参数当成此 JSON。前者供 ROS/Python 配置读取，后者才是 CLI 运行配置。仿真与硬件参数必须分开。

统一目录保存 `device.json`（凭据 0600）、`agent.db`（含可能的 WAL/SHM）、`estop-state.json`、`agent.sock`、仿真锁和 `ros-home/`（ROS 日志在 `ros-home/log/`）。仿真 start 只在首次缺失时按仿真规则初始化急停状态；已存在状态跨重启保留，无效状态不能靠删除来绕过。目录应为 0700，备份也要保护凭据与数据库。

## 3. 配对、检查与启停

先在云端网页为此仿真实例生成一次性配对码，再在仿真机项目根目录执行，按交互提示输入；不要将配对码或 token 写进命令行或文档。已有凭据拒绝覆盖；旧凭据被删除且无备份时要重新配对。

```sh
.ros-venv/bin/senior-robot pair --cloud-url wss://robot.example.com/ws/agent --name '隔离仿真'
.ros-venv/bin/senior-robot doctor
.ros-venv/bin/senior-robot start
.ros-venv/bin/senior-robot status
.ros-venv/bin/senior-robot stop
```

以上默认使用 `~/senior-control`。自选目录时，每条命令都显式追加 `--control-dir /home/sim/another-sim`，并先在该目录创建 `config.json`。仿真无需执行实车 `deploy`；`start` 通过 `simulation_cli.py` 创建 systemd 用户服务，先启动 ROS 仿真入口并完成适配器握手，随后启动 Agent，停止先 Agent 后本次仿真。它不管理其他实例或厂商进程。

`doctor` 不启动 Gazebo、设备或 ROS 节点。另需检查 ROS 参数文件时，可在项目根目录执行 `.ros-venv/bin/python tools/doctor.py config/simulation.yaml`；没有运行 ROS 图时实时项为 `not_run`，不代表通过。检查和构建不会自动下载模型权重。

查看日志也在仿真宿主机，以同一普通用户执行：

```sh
# 先列出实际用户服务，再把名称替换为该实例的真实 unit：
systemctl --user list-units --all 'senior-sim-*'
unit='senior-sim-替换为实际实例哈希.service'
journalctl --user -u "$unit" -n 100 --no-pager
```

`status` 的 Process、ROS IPC、Cloud 分别表示进程、身份探测与本次运行最近连接事件；`last event connected` 不是持续在线、地图有效或运动资格证明。启动失败先检查构建、overlay、用户会话、独立域、凭据、私有目录权限和显示环境，不盲删 socket 或急停文件。

## 4. 图形窗口与实例隔离

`start` 默认等同于 `start --headless true`，通过 Xvfb 无窗口运行。要打开本机 Gazebo
窗口，在 Linux 图形桌面的终端执行：

```sh
.ros-venv/bin/senior-robot start --headless false
```

窗口模式绕过 Xvfb，将当前终端的 `DISPLAY`、`XAUTHORITY` 及相关桌面会话环境传入
systemd 用户服务；缺少 `DISPLAY` 时会明确报错。需要可访问的 X11 或 XWayland 显示和
相应认证权限。纯 SSH 会话可继续使用默认无窗口模式。模式是本次启动参数，无需修改
JSON 配置。若同一配置已经启动，命令只提示正在运行；切换模式须先 `stop` 再重新 `start`，
不会额外启动第二套仿真。

`start` 需要可用的 systemd 用户会话。仿真启动后，命令先核对 ROS 适配器身份，再启动
Agent。停止时先给 Agent 时间完成停车与关停，再退出本次服务的仿真进程。复用同一档的
急停文件。状态分别显示进程、IPC 身份核验和该次运行最新的云端连接事件。
若强制停止遗留 IPC socket，下一次启动只会在取得实例目录和 ROS 域两把锁后，清理
当前用户拥有且明确拒绝连接、检查期间身份未变化的旧 socket。活动连接、权限问题、
超时、非 socket 路径或路径被替换时仍拒绝启动；不会直接覆盖占用路径。

### 工作台实时地图

`.ros-venv/bin/senior-robot start` 的仿真档启动 Gazebo 的运动校验场景，并以 `qualified_reference:=true` 请求带引用的地图和 Nav2；只有参考与运行资格满足时才可用，不保证导航通过。默认不自动启动 `slam_toolbox`。
栅格由启动使用的 SDF 碰撞几何生成，网页图块和 Nav2 的 `/map` 来自同一份栅格。
定位参考使用该隔离实例的 Gazebo 真值里程计和轮式里程计，任一来源过期、重复或切换时
停止提供定位资格。手动调试此链路时，应在同一隔离域的
`simulation_motion.launch.py` 上显式传入 `qualified_reference:=true`。
每次导航独立启动控制器和 BT 导航节点，结束或撤销权限后回收。仿真配置的速度限制与
既有平地校验参数一起使用，运行时仍要求安全门报告新鲜的激光和运动观测。
其他场景及实车配置继续依赖各自可验证的地图和定位来源。

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

### 运动连续性中断后的显式恢复

若扫描等运动证据源曾中断，源恢复后仍可能因 `progress_history_unqualified` 保持停车。
先确认仿真稳定、车辆静止，并松开移动控件，再在仿真宿主机执行：

```sh
.ros-venv/bin/senior-robot requalify-motion
# 使用另一个独立统一目录时：
.ros-venv/bin/senior-robot requalify-motion --control-dir /home/sim/another-sim
```

命令验证仿真档、该档的运行中用户服务和 ROS 适配器身份后，加载该档的 ROS 环境，
在对应隔离域请求现有 `/motion_fusion/requalify_progress` 服务。适配器身份必须报告实际
运行的 ROS 域，并与当前配置一致；修改配置中的域号后需重启对应仿真，不能沿用旧进程。
旧版适配器缺少域信息时此命令拒绝执行，请重新构建 ROS 工作区并重启仿真。未运行、身份核验失败、
服务拒绝或超时均返回失败；实车档不提供此操作。此命令不重启仿真、不发送速度命令，
也不清除急停或真实运动异常锁存。拒绝时终端区分无需重新核验
（`progress_history_not_broken`）、运动异常仍锁存（`motion_anomaly_latched`）、
静止校准未就绪（`stationary_profile_unqualified`）和时钟未就绪（`clock_unqualified`）。
未知服务错误只显示通用失败信息。

终端返回 `accepted (armed), not yet qualified` 只表示请求已受理，尚未恢复运动资格。
保留正常控制会话的零速请求，保持静止；请求后必须有新鲜的请求零速、实际输出零速，
并通过两个连续、独立对齐的扫描、轮速和 IMU 静止观测区间。持续非零请求、证据再次中断
或不满足静止阈值时，安全门继续停车；命令不会伪造观测或降低判断条件。

`.ros-venv/bin/senior-robot status` 只报告进程、IPC 和云端连接，不能证明运动资格恢复。
通过同一 ROS 域的 `/motion/evidence` 实时运动证据检查 `valid` 恢复且不再报告
`progress_history_unqualified`；再检查 `/safety/status` 的新鲜 `evidence_ready=true`
以及工作台中当前车辆的安全许可与拦截原因。保持零速时，安全门仍可能显示
`zero_requested`，这是正常停车状态；历史 `count.*` 计数不会因恢复而清零。
全部实时安全条件满足后再尝试驾驶；若仍被拦截，按当前原因检查，不能把请求受理当作
解除所有安全限制。

### 仿真视频

要在浏览器查看仿真车辆的 RGB 摄像头，在仿真档 JSON 中设置 `"video": true`，
并让运行 Agent 的 Python 环境安装项目的 `video` 可选依赖。仿真和 Agent 共享
本次运行专用的私有最新帧，视频继续经现有 WebRTC 通道协商。

## 源码依据与验收边界

配置解析见 [cli.py](../../src/senior_control/cli.py)，用户服务、握手、隔离与恢复见 [simulation_cli.py](../../src/senior_control/simulation_cli.py)，场景/参考入口见 [simulation_motion.launch.py](../../ros2/src/senior_robot/launch/simulation_motion.launch.py) 和 [simulation_reference.py](../../ros2/src/senior_robot/senior_robot/simulation_reference.py)。地图栅格来自 SDF 的 `baseline_grid` / `SimulationReference` 固定场景链路，不是已经完成在线 SLAM 的证明。完整 Linux 渲染、启停、地图、导航、视频与运动恢复仍需对应运行环境验收，静态文档更新不增加通过结论。
