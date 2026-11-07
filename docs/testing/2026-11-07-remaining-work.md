# 2026-11-07 剩余工作清单

这是以当前代码、历史复审和实际部署对话为依据的待办快照。原路线图的模块交付、专项验证、Linux 运行接线和实车验收分别跟踪。历史证据见 [10 月 31 日起的复核记录](2026-10-31-plan-audit.md) 与 [验收矩阵](matrix.md)。

已观察到公网 API 对未登录请求返回 401，以及浏览器登录成功。云端配对 API 和车端兑换 CLI 已存在，Agent 入口已组装，启动时先验证本地 ROS 适配器的设备身份。SLAM 原生图、扫描来源和静止候选匹配有专项证据。下面各组的关闭条件描述剩余工作，组数用于跟踪，原计划任务可跨组。

| 顺序 | 待办组 | 原任务 | 现状与关闭条件 |
| --- | --- | --- | --- |
| 1 | 网页设备配对 | 13、14、17 | [Task 22](../superpowers/plans/2026-11-07-web-device-pairing.md) 已实现并通过隔离 HTTPS 验收。公网部署后复验网页生成、车端兑换和设备在线。 |
| 2 | 车端上线与系统部署 | 14、20 | 使用配对凭据、正确 robot ID 和 Unix socket 在目标 Linux 上完成 Agent 与 ROS 适配器建连。补齐 systemd、重连、断线和关停证据。 |
| 3 | 运动安全、远程权限与 Nav2 | 4–6、10、18 | 有历史安全与仿真专项。生产 Agent 仍以 `source_clock_verified=False` 启动。完成浏览器到 ROS 的授权与源时序、导航动作代次、急停复位、返航、窄道和有界脱困，分别验证失联、陈旧命令和重启停车。 |
| 4 | 长期地图与包生命周期 | 7–9、14、19 | 有原生图历史和归档专项。完成合格的实际占据层消费、动态遮挡判定及生产 ROS 地图端口，实现并验证远程导入、导出、清除、失败恢复与引用切换。 |
| 5 | 可信定位、GPS 与导航参考 | 10–12、18 | 静止候选诊断已有实现，生产 `localization_node` 的 `scan_match` 仍是空实现。完成可信定位源、GPS 配准和被动航向、重定位歧义策略，并验证任务与 Home 使用同一有效地图参考。 |
| 6 | RGB-D、YOLO、WebRTC 与 TURN | 15、16、19 | 已有来源与模型接口专项。完成真实相机输入、三维覆盖及动态目标与激光关联，验证实际网络中的 WebRTC、TURN 中继和断线恢复。 |
| 7 | 网页与设备端业务闭环 | 17–19 | 登录已在公网验证。用真实运行端点验证驾驶状态、地图操作、任务编辑、视频和重连对账，按依赖复用第 2–6 组的生产能力。 |
| 8 | 公网部署复验与证书续期 | 20 | 公网登录已通。记录 Jenkins 重跑后的 `Deployment healthy` 与 `current`，并完成 Certbot 模拟续期、SSH 部署钩子及公网新证书验证。详见 [部署复盘](../operations/deployment-retrospective.md)。 |
| 9 | Linux 端到端和实车验收 | 21 | 在同一冻结提交执行整栈 Linux、Gazebo、浏览器和故障注入，再按 [实车清单](real-robot-checklist.md) 分阶段验证。原生测试、仿真、实车证据分别记录。 |

建议先推进第 2、8 组，并在公网部署后复验第 1 组。网页、部署文档和代码复审可以并行，完整仿真与重型构建按机器资源串行。完成第 2 组后可按接口边界并行推进第 3–6 组，再用第 7、9 组收敛整栈证据。

## 对应代码入口

- [网页设备列表](../../web/src/features/devices/DeviceList.tsx) 与 [配对协议](../protocol/http-ws.md)
- [车端入口](../../src/senior_control/agent/main.py) 与 [Agent 运行装配](../../src/senior_control/agent/composition.py)
- [ROS IPC 适配器](../../ros2/src/senior_robot/senior_robot/agent_adapter.py) 与 [生产定位节点](../../ros2/src/senior_robot/senior_robot/localization_node.py)
