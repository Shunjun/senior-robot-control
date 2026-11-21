# 根 src 迁移收尾实施计划

**目标：** 删除仓库根目录 `src`，有效功能由现有 ROS 功能包、管理程序、云程序和共享基础代码各自拥有。

**基线：** `main@cf64860`，执行分支 `feat/source-migration`。用户要求本人直接完成，不使用 Orca 或额外开发 Agent。

**依据：** `docs/architecture/ROS2与车端架构重构确认稿.md`，以及用户本轮要求根 `src` 可整体删除。

## 约束与范围

- 保留五帧学习、线加速度 0.5、减速度 0.8、直接角速度及限幅。
- 不启动车辆、ROS、仿真或部署服务，不自动合并 main。
- 真实算法迁入功能包；共享基础代码移到根 `vehicle_core` 包，继续使用根发行，避免新增空壳项目。
- 旧导入入口收敛到唯一实现；调用方、启动命令、构建、工具和测试同步更新。
- 删除前区分有效算法、仅供旧 API 的组装适配和退役代码，不以更换目录冒充迁移。

## 实施步骤

- [x] 1. 记录测试基线、依赖和实际文件清单。
- [x] 2. 感知/视觉归 `senior_perception`；运动估计原语归 `senior_motion_estimation`；控制实现归 `senior_motion_control`。迁移现有用例的导入，不改算法断言。
- [x] 3. 管理/平台具体装配收敛到 `vehicle_manager`，删除历史模块的转发层，更新生产调用和工具。
- [x] 4. 根共享包归 `vehicle_core/`，更新根 wheel 与所有配置路径，删除根 `src`。
- [x] 5. 定向回归、全体已有测试对比基线、构建 wheel、仓库外安装导入与入口检查、依赖扫描。
- [x] 6. 审阅实际差异，记录真实通过和未验证项，提交功能分支。

## 验证重点

复用现有学习、视觉/RGBD、估计、控制、管理/云测试；不新增重复算法测试。
安装检查必须在仓库外且没有根 `src` 或旧 editable 路径可兜底的环境完成。
检查 worker spawn 与 `python -m` 入口、包资源、云与车端隔离、共享值身份、旧配置读取和真实退出清理。
旧 SDK 兼容性断言随入口退役调整；与迁移无关的现有失败保留并与基线逐项对照。

## 进度与证据

- 基线 collect：2471 项，3 个已有收集错误（旧 ScanGeometry 两处、已删除的全帧学习测试函数一处）；完整输出 `/tmp/senior-source-baseline-collect.log`。

- 实现迁移：56 份实现、17 份共享模块及资源归包；Feedback 从旧 controller 中抽出归估计模块，100 份冗余旧文件删除（以最终 git 识别为准）。保留有现有调用/测试契约的控制计算工具，不借本次迁移擅自删除这些算法。
- 具体装配归位：实车服务归 senior_robot；操作/部署、Home/地图具体适配归 vehicle_manager；云默认诊断适配归 cloud，不依赖车端基础包。
- 日志归属：识别新功能包命名空间，管理日志上报订阅相同命名空间；现有日志测试恢复所有被配置的 logger，避免影响后续 caplog 用例。
- 根发行名称 senior-control 保留，实际仅发布根 vehicle_core 包；旧 senior_control 导入包不再发布。uv.lock 检查通过，无需变更依赖版本。
- 原完整范围基线 2107 passed / 208 failed / 40 skipped / 125 errors；最终同范围 2131 passed / 197 failed / 40 skipped / 112 errors，无新增失败节点，24 个原失败节点已通过。不能称全仓/CI 全绿。
- 11 个真实 wheel 构建、仓库外 base/full 安装、pip check、SQLite 迁移/回滚/备份/重开、共享资源读取与模块归属、实际 spawn 工作进程返回/关闭、两个正式入口 help 已验。安装只复制当前已安装的第三方依赖，项目包全部来自新 wheel，无 editable 或根 src 兜底。
- Unix socket 创建受当前环境禁止，完整 Agent IPC 生命周期检查真实失败，未绕过或冒充通过。Linux/ROS Humble、DDS、原生构建和实车部署未验证。
- 自审完成：确认控制参数与 vehicle_speed 行为无新增改动、迁移算法保留现有计算、模块/worker/平台启动入口已切换、根 src 不存在、生产代码无旧导入。未使用外部审核 Agent。
