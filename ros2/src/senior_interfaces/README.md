# senior_interfaces

统一 ROS 接口包：26 个消息与5个服务，通过 rosidl 生成类型。原 motion 与 SLAM 接口的字段、界限、常量及服务分段保持不变，仅嵌套类型包名改为 senior_interfaces。

不包含业务 Python 包或手写时间契约。时序头文件由 senior_sensor_ingress 导出。迁移时生产者与消费者需要整体干净重建，详见 [当前包归属](../../PACKAGE_OWNERSHIP.md)。
