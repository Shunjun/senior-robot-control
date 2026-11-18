# 时间协议 overlay 已退役

本页对应旧版本的第三方时间补丁操作，已随 2026-11-17 标准 ROS 迁移退役。准备脚本、补丁、manifest 和专用时钟库均已移除，不再提供安装命令。

使用原装 ROS 组件和标准 `header.stamp`；项目首次接入记录 ROS/单调时间并保留测量年龄，最终速度通过标准 `/cmd_vel` 发布。Nav2 使用每次运行的私有命令话题，底盘或仿真 drive 只启动一次。

当前要求见[批准设计](../superpowers/specs/2026-11-17-standard-ros-time-design.md)、[实车部署](vehicle-deploy.md)、[仿真操作](simulation.md)及[接口重建](ros-interface-rebuild.md)。旧版本行为证据保留在 Git 历史，不作为当前部署前提。
