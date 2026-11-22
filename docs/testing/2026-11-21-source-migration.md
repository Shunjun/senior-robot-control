# 根 src 迁移验证

基线 main@cf64860；分支 feat/source-migration。根 src 已删除；旧 senior_control.* 导入退役，发行名 senior-control 保留，仅发布 vehicle_core。功能实现归现有 ROS/manager/cloud 包。历史流程文档中的旧路径仅是历史记录。

## 同范围基线对比

```sh
.venv/bin/pytest -q tests/unit tests/integration senior_control/tests cloud/tests ros2/tests ros2/src/senior_perception/test ros2/src/senior_motion_estimation/test --continue-on-collection-errors --tb=line
```

| 版本 | 通过 | 失败 | 跳过 | 错误 |
| --- | ---: | ---: | ---: | ---: |
| 修改前 | 2107 | 208 | 40 | 125 |
| 迁移后 | 2131 | 197 | 40 | 112 |

按失败节点比对新增为 0，原失败节点减少 24。3 个原有收集错误仍存在：两份旧 ScanGeometry 测试，以及引用已退役全帧学习函数的测试。没有为本次迁移删断言或排除这些文件。
补充执行建图/定位包、Home 与静止判断相关现有回归：176 passed。此组与前述大组有部分重叠，不累计通过数。

日志迁移初轮暴露了 namespace 过滤与测试 logger 状态泄漏，已修正。学习清理单独及与日志配置组合 20 项通过；迁移最终同范围复跑确认没有新增失败。

## 安装与静态验证

- 11 个 wheel：根 core、contracts、manager、cloud 与 7 个 ROS Python 功能包；使用缓存中的现有 Hatchling 后端离线构建，未修改依赖版本。
- 仓库外 base 环境安装 core/contracts/manager 三包，full 环境安装全部 11 包；第三方依赖复制当前安装版本，项目安装使用 wheel，无 editable/.pth 源码路径。
- `tools/verify_batch_one_install.py --manager-tasks-only`：真实 SQLite 顺序迁移、失败回滚、备份、core schema、重开及任务/航点公开接口通过。
- 全功能安装检查：共享资源和惰性导入、独立 SQLite、导航/平台运行模块位置与唯一值身份通过。旧 senior_control import namespace 不存在。
- 独立安装环境的实际 ProcessWorker spawn、计算返回与关闭通过；SelfReturnModel 从安装的 senior_perception 加载，样本数为 5，Feedback 属于 senior_motion_estimation。
- `python -m vehicle_manager.operator_cli --help`、`python -m senior_robot.service_runtime --help`：通过，不启动服务。
- base/full 依赖检查通过；`uv lock --check --offline`：通过，71 包。
- Ruff check 通过；改动 Python 文件格式通过，CRLF 感知 diff 检查通过。全仓格式检查仍指出未修改历史计划 2026-11-21-safety-input-efficiency.md 中的一段代码格式。
- mypy 未全通过：原有泛型/重导出问题和缺失 ROS/类型标记仍存在；本次未为通过检查放宽规则，不能宣称 CI 全绿。

## 明确未验证

完整 Agent 安装态生命周期在真实 Unix socket bind 时被当前沙箱拒绝（PermissionError），没有改成 mock 伪装通过。Linux/ROS Humble、DDS、C++/ament、真实 SLAM 和实车部署未运行。本次不重启任何已有服务。旧安装和生成的 systemd unit 应在实际部署时更新。
