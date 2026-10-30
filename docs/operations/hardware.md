# 硬件部署与检查

机器人 Linux 宿主机运行 ROS 2、驱动和代理。先把
`config/hardware.example.yaml` 复制到受管配置目录，逐项校准 footprint、frame、topic
和传感器覆盖；不要把仿真与硬件配置合并。

安装 `deploy/senior-agent.service` 前创建专用 `senior-control` 用户，以及仅该用户可读的
`/etc/senior-control/agent.env`。云端 URL 必须使用云主机域名，不能填写机器人宿主机或
虚拟机的 `localhost`。设备令牌通过配对获得，撤销后必须重新配对。

运行只读检查：

```sh
uv run python tools/doctor.py /etc/senior-control/hardware.yaml
```

doctor 不启动设备、ROS 节点或下载任务，也不加载或获取模型权重。未运行 ROS 图时，
话题、TF、时钟、发布者、底盘 watchdog 和模型状态会明确显示 `not_run`。任何重复
`/cmd_vel` 发布者、非 `safety_node` 发布者，或同一 child frame 的重复 TF 发布者都必须
先修复。`command_node` 只能发 `/safety/command_in`。

上线前另行确认底盘固件 watchdog 能在命令超时后独立停车。视觉推理使用 GPU 时，宿主机
需安装与硬件匹配的驱动、容器/运行时支持和足够显存；无 GPU 时应明确采用已验证的 CPU
性能配置。模型权重由运维人员离线校验摘要后放入配置路径，镜像构建与 doctor 均不会下载。
