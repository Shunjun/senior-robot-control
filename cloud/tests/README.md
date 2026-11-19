# 云端测试

C1 迁入原 test_cloud、test_cloud_epoch、test_agent_snapshots、test_heartbeat_wire、test_drive_challenge及唯一 heartbeat_test_support。
无新增场景；认证/配对/CSRF/Origin/epoch/撤销、快照过期和线上省略、心跳与安全消息保护来自原用例。

生产 factory 仍通过 `senior_control.cloud.app` 显式注入旧设施，相关原用例继续覆盖这个真实入口。
纯值与缓存/注册实现直接导入 cloud；需要旧默认技术设施的 Hub 构造仍使用显式Compatible子类。
时钟patch改在canonical模块；不是复制业务或复制夹具。

root pytest pythonpath 明确包括 cloud/src、cloud/tests 和 tests/integration。
管理侧 agent_fakes 和跨端消费者暂未归项目，测试依赖它们不代表生产云依赖车端；后续管理片再迁。
test_agent_pairing、manual_wire、settings_logs_wire、runtime_reporting继续复用这里的test_cloud夹具，
agent/manual/task_ownership继续复用这里的heartbeat_test_support，没有第二份支持模块。
CI新增cloud/tests发现，管理/ROS原测试路径继续有效。

[云包C1记录](../README.md#验证证据) 列出命令、版本、144项定向通过、四项基线复现失败及38项旧类型错误。
所有失败保留原保护；本机验证不等同真实部署、ROS或Linux运行。
