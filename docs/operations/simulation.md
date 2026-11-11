# 仿真运行说明

仿真使用 `config/simulation.yaml`，只在虚拟机或开发宿主机运行 ROS 仿真栈。云服务仍位于
云主机；虚拟机连接云端时使用云域名，不使用虚拟机自己的 `localhost`。若浏览器在另一台
机器，也必须通过云域名访问。

启动任何仿真进程前可执行只读检查：

```sh
uv run python tools/doctor.py config/simulation.yaml
```

没有正在运行的 ROS 图时，doctor 会把实时项记录为 `not_run`，不会为了完成检查启动
Gazebo、设备、ROS 节点或模型下载。仿真与硬件 profile 必须保持为两个文件；不得添加重复
`mode` 或将两种模式拼接。仿真仍遵守唯一速度出口：导航和命令节点发布
`/safety/command_in`，只有 `safety_node` 发布 `/cmd_vel`。

需要 GPU 加速的仿真或视觉推理，虚拟机必须启用经过验证的 GPU 直通或虚拟 GPU，并安装
匹配驱动；否则使用 CPU 配置并降低负载。模型权重应预先校验并放在明确路径，构建和检查
过程不会联网获取。

## 使用车端命令管理隔离仿真

在已安装 Agent 依赖、ROS 2 Humble、Gazebo Fortress 和 Xvfb 的 Linux 用户会话中，
为每个实例准备独立的私有目录（权限 0700）。选用现场未使用的 ROS 域。
将下面的配置保存为 `~/senior-sim/simulation.json`，并把绝对路径换成该实例的路径。
省略 `--profile` 时，命令会读取此文件，并要求 `kind` 为 `simulation`。其他配置位置可用
`--profile` 指定，实车配置仍须显式指定。

```json
{
  "kind": "simulation",
  "credentials": "/home/toby/senior-sim/credentials.json",
  "database": "/home/toby/senior-sim/state.db",
  "socket": "/home/toby/senior-sim/adapter.sock",
  "estop_state": "/home/toby/senior-sim/estop.json",
  "ros_setup": "/home/toby/ros-workspace/install/setup.bash",
  "cloud_url": "wss://robot.example.com/ws/agent",
  "ros_domain": 73
}
```

```sh
senior-robot pair --cloud-url https://robot.example.com --name "隔离仿真"
senior-robot doctor
senior-robot start
senior-robot status
senior-robot stop
```

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
Agent。停止时先给 Agent 时间完成停车与关停，再退出本次服务的仿真进程。复用同一档的
急停文件。状态分别显示进程、IPC 身份核验和该次运行最新的云端连接事件。

### 工作台实时地图

`senior-robot start` 的仿真档启动 Gazebo 的运动校验场景，并启用带引用的地图和 Nav2。
栅格由启动使用的 SDF 碰撞几何生成，网页图块和 Nav2 的 `/map` 来自同一份栅格。
定位参考使用该隔离实例的 Gazebo 真值里程计和轮式里程计，任一来源过期、重复或切换时
停止提供定位资格。手动调试此链路时，应在同一隔离域的
`simulation_motion.launch.py` 上显式传入 `qualified_reference:=true`。
每次导航独立启动控制器和 BT 导航节点，结束或撤销权限后回收。仿真配置的速度限制与
既有平地校验参数一起使用，运行时仍要求安全门报告新鲜的激光和运动观测。
其他场景及实车配置继续依赖各自可验证的地图和定位来源。

网页地图通过状态快照接收地图坐标系中的车辆位置、朝向和实时激光命中点。
底图呈现固定场景几何，紫色扫描层随观测更新。单次扫描最多抽样 64 个命中点；
扫描与定位时间相差超过 150 ms 或数据过期时清空扫描层。断线、定位失效及地图
代次变化时隐藏对应的实时叠加，并提示位置不可用。同步更新云端、Agent、ROS 构建
和 Web，以便整条链路识别新增的状态字段。导航不可用时，快照携带车端检查未通过的
原因，网页分别提示地图定位、规划控制器、传感器证据或心跳权限问题。

### 运动连续性中断后的显式恢复

若扫描等运动证据源曾中断，源恢复后仍可能因 `progress_history_unqualified` 保持停车。
先确认仿真稳定、车辆静止，并松开移动控件，再在仿真宿主机执行：

```sh
senior-robot requalify-motion
# 使用其他仿真档时：
senior-robot requalify-motion --profile /home/toby/another-sim/simulation.json
```

命令验证仿真档、该档的运行中用户服务和 ROS 适配器身份后，加载该档的 ROS 环境，
在对应隔离域请求现有 `/motion_fusion/requalify_progress` 服务。未运行、身份核验失败、
服务拒绝或超时均返回失败；实车档不提供此操作。此命令不重启仿真、不发送速度命令，
也不清除急停或真实运动异常锁存。拒绝时终端区分无需重新核验
（`progress_history_not_broken`）、运动异常仍锁存（`motion_anomaly_latched`）、
静止校准未就绪（`stationary_profile_unqualified`）和时钟未就绪（`clock_unqualified`）。
未知服务错误只显示通用失败信息。

终端返回 `accepted (armed), not yet qualified` 只表示请求已受理，尚未恢复运动资格。
保留正常控制会话的零速请求，保持静止；请求后必须有新鲜的请求零速、实际输出零速，
并通过两个连续、独立对齐的扫描、轮速和 IMU 静止观测区间。持续非零请求、证据再次中断
或不满足静止阈值时，安全门继续停车；命令不会伪造观测或降低判断条件。

`senior-robot status` 只报告进程、IPC 和云端连接，不能证明运动资格恢复。
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
