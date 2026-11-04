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
