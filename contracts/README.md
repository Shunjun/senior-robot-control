# senior-contracts

独立 Python 发行包，源码 `src/senior_contracts`。只依赖 Python ≥3.10 和 Pydantic，不能导入 ROS 或车辆执行/管理实现。

- `values.py`：Velocity（m/s、rad/s）、BodyTwist、Pose2D（m、rad）、MapRef、操作状态与结果。
- `errors.py`：稳定错误码与异常。
- `configuration.py`：原样迁移的不可变驾驶配置值；设置持久化/应用策略仍在管理程序。
- `navigation.py`：RoutePlan/RouteProgress、身份/代次/目标值验证；QualifiedRoute 执行不在此包。
- `wire.py`：既有版本化消息形状、校验和序列化；原数值/时钟/缺失语义不变。

通过根 `uv sync --frozen` 的本地路径依赖安装；独立交付时 `uv build contracts`，并先安装其 wheel 再安装 manager wheel。根 `senior_control` 的旧类型路径只显式转发同一对象，不维护另一份实现。跨程序消费者无需 ROS 生成环境。

验证：`tests/unit/test_protocol.py` 与 `tools/verify_batch_one_install.py`。本包不持有业务状态、不运行节点、不启动进程；输入不合法按原异常/校验语义拒绝，不转换成可执行运动。
