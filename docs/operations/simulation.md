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
将下面的配置保存为 JSON 文件，并把绝对路径换成该实例的路径。

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
senior-robot pair --profile simulation.json --cloud-url https://robot.example.com --name "隔离仿真"
senior-robot doctor --profile simulation.json
senior-robot start --profile simulation.json
senior-robot status --profile simulation.json
senior-robot stop --profile simulation.json
```

`start` 需要可用的 systemd 用户会话。仿真启动后，命令先核对 ROS 适配器身份，再启动
Agent。停止时先给 Agent 时间完成停车与关停，再退出本次服务的仿真进程。复用同一档的
急停文件。状态分别显示进程、IPC 身份核验和该次运行最新的云端连接事件。
