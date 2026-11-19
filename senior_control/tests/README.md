# 车端管理测试

M1a迁入原tests/integration/test_vehicle_settings.py，62项原用例及原断言不变，仅改canonical导入。
继续使用原Root Database真实连接和写锁，覆盖持久/并发/幂等/commit取消、旧配置、损坏不可用、runtime gate、DEBUG恢复。
没有复制测试或拆假数据库。

跨端test_settings_logs_wire、settings_logs_review和ROS手动命令相关用例保持原位置，
本片只在它们的真实旧兼容调用上做定向回归，不为改变目录增加无关场景。
独立manager wheel与完整旧SDK的安装/类型身份检查复用既有tools/verify_batch_one_install.py，
新进程确认单独导入公开值不加载service/store/Database，旧新包入口服务类仍同一对象。

[manager记录](../README.md#命令版本与结果) 给出实际命令/版本、75项定向通过、
基线同样复现的50项退役控制夹具失败及38项旧类型错误、真实ROS/安装环境限制。
其他管理/监管测试在各自实现迁移时再归入，不先搬空覆盖。

M1b将原`tests/integration/test_vehicle_logs.py`完整迁入本目录，38项原用例的输入、断言和时限不变。
models/store（含测试内局部prepare_entry）改从canonical导入；service仍通过生产实际使用的
旧Compatible构造入口装配原诊断owner，没有构造假诊断或假属性。
覆盖授权/撤销、过滤分页/游标重开、脱敏、损坏库拒绝、存储失败、队列溢出、订阅上界和控制隔离。
跨端wire/review及原runtime诊断用例仍归原位置，未为目录调整新增场景。

本轮完整选择集72项通过（3.76s），包含38项日志及原wire/review/诊断用例，不与子集相加。
之前的慢速执行出现flush/接收/恢复时限失败，原断言保留；基线对照和复跑详情见
[M1b记录](../README.md#m1b-车辆日志owner迁移)。独立manager与旧SDK安装检查通过，真实Linux/ROS未验证。
