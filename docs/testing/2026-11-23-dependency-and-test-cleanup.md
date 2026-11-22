# 依赖解耦与失败测试清理

2026-11-23；分支 `feat/vehicle-architecture-refactor`，起点 `ed42e0a`，前续提交 `c6910cd`、`53d1df5`、`10343fd`。执行范围及逐阶段证据见 [实施计划](../superpowers/plans/2026-11-23-dependency-and-test-cleanup.md)。

## 当前结果

- 默认完整测试树：**2453 passed、116 skipped、0 failed、0 errors**，113.91秒，进程正常退出。1条既有 forkpty 多线程弃用警告。允许本机测试套接字，并仅将已安装 Docker CLI 加到 PATH 以运行只读 compose config；没有启动 Docker或任何业务服务。
- 最后审阅补回当前“实际输出失败→来源撤销→发零”的保护：对应 cadence/output 文件4项通过，0.32秒。它在全树运行结束前后追加，未改产品代码；不把未重新跑的整个树虚报为2454项单次通过。
- 允许套接字的集成定向集1101通过、29跳过；包含此前沙箱拒绝的两项IPC身份检查。unit非套接字部分1059通过；原始时间、乱序、去重、任务退役/收集和急停保护保留。
- 五份wheel离线构建并隔离安装通过。`python -I -S`检查新位置、旧模块缺失、共享值身份；感知学习导入不会加载估计/控制，也不加载rclpy。
- Ruff及修改文件format、diff检查通过。116项均为既有运行环境/ROS/native门控，未添加 skip/xfail 掩盖失败。跳过不是实际ROS或车端验收。

最终广泛命令：

```sh
PATH="/usr/local/bin:$PATH" .venv/bin/pytest -v --continue-on-collection-errors --tb=line -o faulthandler_timeout=45
.venv/bin/pytest -q ros2/tests/test_control_cadence_gate.py --tb=short
.venv/bin/ruff check .
git diff --check
```

详细日志在本机 `/tmp/ros-followup/`：`full-suite2.log`、`integration-round2.log`、`unit-round4.log`、`state-tests.log`、`retirement-audit.txt`、`check_installed.py`。第一次默认全树因旧相机启动终止条件挂起，被终止后修正夹具；不计通过。

## 实际修复

1. 时钟回退仍报告静止：故障锁存时清除旧时间坐标下的观测。保留故障revision与显式复位。原有回归修复前stationary=True失败，修复后通过。
2. 学习提前停止后采集永不结束：原验证时间早于动作窗口结束，无法满足采集服务水位条件。原窗口结束且确认静止后更新验证水位，不延长三秒动作窗口、不重播动作。原有窗口/实际输出时长回归通过。
3. 配置schema漂移：模型已要求length/width，schema仍可选。重生成schema，仅改变这两个字段的默认/required描述；保留原模型一致性断言。

## 测试迁移审阅

14个新增或实质改写场景，跨模块上限20；上述3项实际bug/config回归复用现有测试，单独记录。其余大量改动只调整真实owner、字段、注入时钟、协议请求或进程资源清理，不改变原保护断言。

| 旧测试范围 | 处理与保留 |
| --- | --- |
| ScanGeometry队列、慢扫描证明、prepare_stop事务回退 | 随已退役生产路径移除；保留独立约束/控制算法用例、当前输出deadline/250ms租期/过期归零和实际发送失败发零 |
| 独立运动/五样本/学习就绪准入、融合停车包络及settling | 按11月18日获批规则退役；保留独立估计/误差/空间数值测试，验证原始扫描可驾驶、缺失发零、有效恢复且不锁故障 |
| 输出端pose-aligned scan与空间膨胀 | 移除控制整合断言；空间与RGB-D解释算法移到估计包并保留原数值断言 |
| 松杆等同永久撤权、缓慢release、反馈恢复自动replan | 改为当前松杆/释放、立即归零、任务自主行为；保留显式任务生命周期、来源/epoch、任务退役证明和身份回收 |
| 完整自动学习snapshot/可见性门 | 退役旧接口耦合，保留当前真实障碍限速、显式学习窗口/采集、急停与不重播保护 |
| test_home同名冲突 | 只重命名e2e文件，内容及断言逐字节不变 |

审核实际删除/重命名清单为17路径71个函数名，包含重命名而非删除；没有按失败数量批量删除、降低断言或增加条件跳过。旧阶段文档仍保留当时证据，以本记录说明当前状态。

## 未执行

Linux ROS干净colcon构建、生成typesupport、已安装时序头consumer、实车DDS/相机和部署仍未验证。没有启动开发服务/仿真/实车服务，没有运动命令。PR保持Draft以标明这些环境验收边界；当前请求中的依赖解耦和自动测试失败清理已完成。
