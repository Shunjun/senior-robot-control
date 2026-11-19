# 车端管理测试

M1a只迁入原tests/integration/test_vehicle_settings.py，62项原用例及原断言不变，仅改canonical导入。
继续使用原Root Database真实连接和写锁，覆盖持久/并发/幂等/commit取消、旧配置、损坏不可用、runtime gate、DEBUG恢复。
没有复制测试、拆假数据库或迁日志/控制夹具。

跨端test_settings_logs_wire、settings_logs_review和ROS手动命令相关用例保持原位置，
本片只在它们的真实旧兼容调用上做定向回归，不为改变目录增加无关场景。
独立manager wheel与完整旧SDK的安装/类型身份检查复用既有tools/verify_batch_one_install.py，
新进程确认单独导入公开值不加载service/store/Database，旧新包入口服务类仍同一对象。

[manager记录](../README.md#命令版本与结果) 给出实际命令/版本、75项定向通过、
基线同样复现的50项退役控制夹具失败及38项旧类型错误、真实ROS/安装环境限制。
其他管理/日志/监管测试在各自实现迁移时再归入，不先搬空覆盖。
