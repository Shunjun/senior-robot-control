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

### 仿真视频

要在浏览器查看仿真车辆的 RGB 摄像头，在仿真档 JSON 中设置 `"video": true`，
并让运行 Agent 的 Python 环境安装项目的 `video` 可选依赖。仿真和 Agent 共享
本次运行专用的私有最新帧，视频继续经现有 WebRTC 通道协商。
