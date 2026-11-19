# ROS 工作区测试

B1 已从原 `tests/ros/` 迁入两份同名用例，没有新增场景：

- `test_native_motion_executor.py`：原实车/仿真入口的真实 DDS 元数据与原生接收调用。
- `test_receipt_process_shutdown.py`：原 raw/scan 接收子进程的 SIGINT/SIGTERM 与重复信号关停。

执行命令与环境限制见 [ROS 接收迁移记录](../README.md)。
ROS 工作区级的 Python 接口、实车/仿真装配和跨功能链回归归这里，
后续随对应实现从现有 `tests/ros/` 及 `tests/unit/`、`tests/integration/` 中的 ROS 用例迁入。

功能包自身的 C++/算法单元测试归各功能包 `test/`，由其构建声明注册；
管理与云端用例分别归根目录 `senior_control/tests/`、`cloud/tests/`。
原用例在迁移生产调用和测试路径的同一批移动，不先复制测试，也不在骨架批次移走现有覆盖。
ROS/ament 不可用时明确登记跳过，静态包元数据检查不替代实际 ROS 构建与端到端回归。
