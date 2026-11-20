# 云端测试

C1 迁入原 test_cloud、test_cloud_epoch、test_agent_snapshots、test_heartbeat_wire、test_drive_challenge及唯一 heartbeat_test_support。
无新增场景；认证/配对/CSRF/Origin/epoch/撤销、快照过期和线上省略、心跳与安全消息保护来自原用例。

C2 的 test_cloud/test_cloud_epoch 改走 cloud.application、CloudSettings 和云 Database，覆盖独立设施的真实认证/SQLite/lifespan；旧 bootstrap 保留原设施并做仓库外兼容检查。
纯值与缓存/注册实现直接导入 cloud；需要旧默认技术设施的 Hub 构造仍使用显式Compatible子类。
时钟patch改在canonical模块；不是复制业务或复制夹具。

root pytest pythonpath 明确包括 cloud/src、cloud/tests 和 tests/integration。
管理侧 agent_fakes 和跨端消费者暂未归项目，测试依赖它们不代表生产云依赖车端；后续管理片再迁。
test_agent_pairing、manual_wire、settings_logs_wire、runtime_reporting继续复用这里的test_cloud夹具，
agent/manual/task_ownership继续复用这里的heartbeat_test_support，没有第二份支持模块。
CI新增cloud/tests发现，管理/ROS原测试路径继续有效。

[云包C1记录](../README.md#验证证据) 列出命令、版本、144项定向通过、四项基线复现失败及38项旧类型错误。
所有失败保留原保护；本机验证不等同真实部署、ROS或Linux运行。


C2新增test_schema只覆盖已选历史格式需求：0–5原起点、同路径重开、v5迁移、旧凭据和epoch不变。
原tests/integration/test_storage.py对vehicle/cloud后端参数化，原SQLite断言不变；
原tests/unit/test_runtime_diagnostics.py对队列/等待子集参数化，Age和车端环境用例仍留原owner；
原tests/unit/test_settings.py复用原cloud设置拒绝用例，覆盖旧加载器与新显式CloudSettings。
这些共用技术回归暂留原文件，不复制两份测试，不改假属性，不削弱保护。
C2执行结果为114项设施/设置定向通过、160项跨端选择通过（存在重复，不相加）；
四项旧控制夹具失败本轮仍复现；完整命令、版本、真实安装/lifespan证据及C3缺口见云包C2记录。
