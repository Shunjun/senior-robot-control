# 第9项生产端时间契约核查（原导航范围已审；系统扩展待审）

## 结论与证据

初次核查时仓库 `DedicatedController.warm()` 启动 `nav2_controller/controller_server`，并将裸 `geometry_msgs/Twist` 重映射到每次运行的私有 topic。消息没有生产单调时间、生产顺序或启动时钟域；`CommandNode._on_navigation_sample()` 依赖 DDS `source_timestamp` 与绑定时的 wall→steady offset。删除 offset 后不能由接收端推导原始样本年龄，接收时盖 monotonic 会使 DDS 队列中的积压指令续命。

核查的官方 [Humble controller_server.hpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/include/nav2_controller/controller_server.hpp) 中 `computeAndPublishVelocity()`、`publishVelocity()`、`publishZeroVelocity()` 都不是 virtual，因此继承一个子类不能拦截基类内部最终发布。官方 [controller_server.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp) 在控制插件返回后仍处理异常/失败容忍、反馈与最终发布，停止分支另外调用 `publishZeroVelocity()`；单独包装 MPPI 插件并提前发布不能完整覆盖最终生产边界。

下载到 `/tmp/nav2-controller-humble.cpp` 的源文件 SHA256：`c3bf93498d3be962975d3cd2749690acde4a422e3fb36c77c2d297a95aadcf0f`。Humble 分支为移动引用，以上是核查证据，不能作为发布固定版本。官方 [rosdistro Humble distribution.yaml](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) 核查时记录 navigation2 release `1.1.20-1`，不表示车辆实际安装版本。

当前环境 macOS / Python 3.12.15；无 `/opt/ros`，无 colcon，未能确认目标车现用 Nav2 包版本。本轮没有启动 ROS 或车辆。

## 最小同版方案建议

1. 固定官方 Nav2 Humble 源码 commit/发布版本，保留 Apache 许可证，以本仓受控 patch/overlay 构建 `nav2_controller`；不复制维护整个 navigation2，不以订阅桥接重新盖时间。构建必须验证源文件散列和 patch 上下文，版本不符明确失败。
2. 在最终生产函数的计算开始捕获本车 `CLOCK_MONOTONIC` 纳秒，随本轮计算直到最终发布；发布新的 `senior_motion_msgs/NavigationSample`，字段最少为 `clock_domain`（Linux boot_id）、`run_id`（私有 topic 相同 nonce）、`sequence`、`produced_steady_ns`、`vx`、`wz`。停止分支在产生零目标时独立捕获时间并共享相同发布函数。不能在最终 publish 才给已经耗时过久的非零计算盖新时间。
3. DedicatedController 为每个实际进程传入 run_id；接收端只接受绑定 topic/run、同车 boot 域、唯一 controller_server writer（固定 GID）、生产 sequence 严格递增、原始 produced_steady 在当前有效期内的有限速度。保留执行代次和真正过期、重放拒绝。绑定以单调时刻建立前沿，退役旧 offset、bound_at_ns、last_stamp 和 clock_offset_changed 原因。
4. 最终生产端控制循环/失败容忍等控制超时改用 steady；保留路径、TF、日志与计划日期语义的 ROS/system 时间。特别核查 `followPath()` 中 `rclcpp::Rate` 等待和 `last_valid_cmd_time_ = now()` 的失败容忍计算，避免仅替换入口。
5. CommandNode 继续通过 Arbiter/FinalCommand 的原始 steady_deadline 传递有效期，不把转发时间作为产生时间；定向覆盖校时前后身份不变、后续新指令可执行、积压/重放/旧代次拒绝、断流停车。

## 备选及边界

- 接收端在 Twist 回调盖单调时间：不能保留积压前的年龄，排除。
- MPPI 插件包装：不能覆盖 controller_server 最终过滤/异常/零输出，排除。
- 子类覆盖发布方法：官方 Humble 方法非 virtual，不能拦截基类内部调用，排除。
- 受控最终生产端 overlay：建议采用，需负责人确认固定版本及交付方式；在确认前不擅自替换 Nav2 依赖。

## 第8项已落地但未验证的增量

`command_node.py` 独立0.3/0.5整条拒绝改为非有限值校验；`ConstraintEngine._refine()` 按当前 RiskEnvelope 线/角上限同比缩放目标后，按既有 fractions 搜索并核验组合扫掠。未改既有导航 degraded_cap 策略，未改三份独立急停/诊断文件或第7项增量；待第9项生产端方案确认后一并迁移和运行受影响测试。


## 负责人决策与当前状态

负责人已亲审并认可最小producer patch/overlay，授权固定官方Nav2 Humble 1.1.20；不可变commit为`a097086719c88f781aa59788eca29ac6ca5e56db`。已按该固定源重新核查并完成5文件补丁，原始与最终散列见`tools/nav2_overlay/manifest.json`，完整构建说明见`docs/operations/navigation-monotonic-overlay.md`；先前“待确认/未验证”段落保留为核查过程记录，原导航范围现已335项定向验证通过。

用户随后扩大第9项至全系统实时控制（包含学习积分/运动估计），新增范围尚未实现；模块消费者清单、端到端时间契约及外部producer信息缺口见`docs/reviews/2026-11-16-control-clock-system-audit.md`，待负责人亲审。此报告的导航验证不能解释为扩大后的第9项完成。
