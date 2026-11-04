# Linux仿真基础验收记录

受测代码：d3c5f0b。环境：Ubuntu22.04.5 aarch64、Python3.10.12、Humble、Fortress6.18.0，4CPU/7.7GiB内存。ROS使用系统解释器，独立venv继承系统ROS包。

- Python单元/集成测试：54通过。
- colcon：senior_sim、senior_robot两个包构建通过。
- ROS运行测试：传感器契约、独立真值与TF、命令超时停车三项通过，14.73秒。
- 停车负对照：临时配置cmd_vel_timeout=10.0，原样运行停车测试，确认先通过运动与净空检查，随后仅在watchdog failed to sustain low velocity after timeout断言失败；这是预期失败，证明测试可发现超时失效。

使用独立ROS_DOMAIN_ID=73与IGN_PARTITION，Xvfb+LIBGL_ALWAYS_SOFTWARE=1；Mesa llvmpipe提供OpenGL4.5。无显示服务器的原始Ogre2/EGL路径在该ParallelsVM中崩溃，未标为支持；Xvfb路径已验证。具体命令见ros2/README.md。每次测试用独立进程组并在退出时清理，本次未启动网页开发服务。

原始日志和校验清单位于本地审计工作区 .superpowers/sdd/2026-10-27-robot-control-implementation/linux-evidence-d3c5f0b/，不包含密码或设备凭据。

这份记录只验收仿真基础，不表示导航、网页、持续建图、盲区倒车、打滑保护或整套项目已完成。仿真暂停后的单调时钟保护由后续安全链实现，真实硬件需单独验收。
