# 学习生命周期与动作许可分离（软件返修）

开始HEAD：800d29f5de8da1c1bcb0ab769adc695def32c31a。共享工作区已有其他Agent的上报/急停/诊断改动，未覆盖；提交前HEAD推进至77f34e0（另Agent的硬件联动移除）。本轮按最新用户要求，学习不因锁存判失败/中断；实际软件急停及最终零输出独立保留，不自动复位。

## 改动

- _startup_target在软件ESTOP/EXECUTION_FAULT时仅撤回动作目标/任务身份，不再以latched_stop退役学习或清除临时严格内点先验。初始化、静态采样照常消费有时效依据的原始观测，边界外/边界歧义仍为障碍，遮挡仍unknown。
- started现在只在实际发出首个非零输出时记录，既是唯一“已尝试运动”证据，也是30秒实际学习预算起点；删除core的重复_learning_motion_emitted字段与所有旧引用/断言，没有兼容分支。等待动作许可不耗尽预算。
- 未执行动作的同一claim可在显式解除软件锁存后继续；已发出运动后撤回授权，forward永久撤回，仅继续原次静态验证。部分行程不伪称completed或learned，结果ambiguous，不自动重放前移、不创建新claim。撤回后的静态等待不继续耗动作预算；真实数据/几何失效仍独立处理。
- 长等候期间保留最新5帧静态窗口，持续更新并保持有界，不把最初旧帧一直留到起步。无运动时数据间隔需要重新采静态窗口，不凭旧段积分伪造位移；实际采集时间/原有统一时效不变。
- permission_lost不将已撤回、仅做静态验证的任务再判为运动许可失败；controller依然运行原停车/障碍检查，并对软件ESTOP/EXECUTION_FAULT给零输出。不触碰另一Agent的硬件删除、急停订阅/复位身份校验、状态发布尾部，也不触碰native诊断文件。

## RED/GREEN

先运行新增test_startup_learning_latch：4失败，直接复现锁存导致active=False、显式解除后无法继续，以及缺少实际发出运动事件。另复现撤回后validation_after_ros每tick刷新导致合法延迟帧永远不能完成后验，修复为只在进入验证时置时间；静态窗口长等候停在stamp=.9而非最新10秒也先失败后转绿。

最终最小生命周期相关回归：

```sh
PYTHONPATH=src '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest tests/unit/test_startup_learning_latch.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_review.py tests/unit/test_startup_learning.py -q --tb=short
```

**42 passed in 7.52s**。新增包含实际core.tick在软件急停/执行故障下持续静态采样、跨40秒等待仍未退役、输出始终零且锁存revision不增加；显式解除后未移动claim继续；已执行动作撤回后不重放；迟到静态帧验证与最新5帧窗口有界。原来要求estop必定interrupted/撤销先验的旧断言已删除替换，没有保留旧行为分叉。未运行大套、ROS、开发服务或实车。

ruff（本轮源文件与相关测试）和git diff --check通过。指纹见2026-11-16-startup-learning-latch-fingerprints.json。与另一Agent的共享core修改已按区域协调，core仅暂存本轮快照hunk，其他未提交改动不纳入本提交。

限制：这仍是合成观测、假ROS transport调用真实Python核心的证据；硬件及软件急停其他owner收敛由其Agent分别验证，完整ROS/DDS和实车仍未验收；本轮不修改备份位置、停用native诊断及其他排队问题。
