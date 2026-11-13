# T5 入口与 Humble DDS 元数据修正

2026-11-13，待负责人亲审；未提交、未动 index、未推送，未启动 Gazebo/驾驶场景或开发服务。范围仅四个 T5 ROS Python 文件、两个定向 unit 文件、新 DDS-only ROS 烟测与本报告；T6 producer/msg/protocol 无修改。

## 实际修正

- `motion_fusion.main`、`safety_node.main` 显式传 `executor_factory=MotionExecutor`；不改变共享 lifecycle 的默认策略。标记回调由实际 `_take_subscription/_execute_subscription` 交付 `(msg,info)`。
- 原生观测与 core reference 共用 `observation_origin`，真实非零逐样本 GID 存在时必须与唯一预期 endpoint 严格相等。实际缺失/全零 GID 仅在显式 simulation-only/unvalidated profile 中允许已知根命名空间唯一 publisher：wheel=`drive`、IMU=`senior_sim_bridge`、scan=`scan_motion_node`、内部reference=`motion_fusion`；对应名称来自当前 launch/驱动组合，不无限放宽。
- `PublisherOrigin` 分别保存 `sample_gid`、`topology_gid`、`basis`。Humble 路径明确 `sample_gid=None`、`basis=simulation_topology_only`，原始源 epoch 中保留该区别；不把 discovery GID 假称逐样本 GID。仍保留 producer epoch、原 ROS stamp/区间、steady 接收/截止、T3序号/时效/恢复及 reference boot/sequence/年龄检查；没有重盖原证据时间。代码标注 `//Compatible`。
- 来源歧义、错误名字/namespace、真实 GID 不匹配和非显式仿真缺 GID 都拒绝。此路径的身份依据弱于逐样本 GID，不能证明 DDS 换源瞬间队列中每条样本的原 publisher；仅用于已批准隔离仿真，不扩到实车。
- 消费 T6 最新 ESTOP 既有语义无需改 producer：idle/空authority 的显式原 request_id 可推进软件事件；同帧重发/无ID回声不反复增revision，新press仍推进；原 received 保持。新增精确回归，request_id 不产生硬件事件证据。

## 本地验证

先运行缺 GID 接纳反例，RED 为 callback 未收到样本（2 passed/1 failed）；再实现兼容。修复后：

```sh
PYTHONPATH=.:src:ros2/src/senior_robot .venv/bin/pytest -q \
  tests/unit/test_native_motion_observation_adapter.py tests/unit/test_motion_core_handoff.py \
  tests/unit/test_motion_core_chain.py tests/unit/test_motion_guard_to_owner.py \
  tests/ros/test_native_motion_executor.py
```

结果 **15 passed / 2 skipped**（本机无 rclpy）；Ruff限定7文件通过，两个适配模块严格 Mypy通过（follow-imports=silent/ignore-missing-imports，不冒充完整ROS类型证明）。共享 T3/T4/T5 数值模块未改，不重复无关旧矩阵。

## Linux 实际证据

证据目录 E：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5b-dds-review`。
VM 新目录：`/home/toby/senior-audit-20261031/t5-dds-smoke-20261113-r1`，未改 r3 source/install/venv。冻结 payload 逐文件 SHA 在 `payload/source-sha.json`；manifest SHA `12455663e15230c5fd55f5bc9f5c2358bb3f08e9388e4a3c6da06f6a1422b165`。

使用现有 r3 `.venv/bin/python`、`/opt/ros/humble/setup.bash`；实际 rclpy路径 `/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py`。PYTHONPATH 首选本次冻结源码。域173、LOCALHOST_ONLY=1、独占 `.vm-test.lock`，外70秒 watchdog、每次pytest30秒上限、独立进程组 finally TERM/KILL/reap。

`test_native_motion_executor.py` 从两个实际入口源码抽取 main，保留其真实 run/executor 参数，以小型 ROS factory 替代完整节点构造避免启动执行器/worker资源。真正创建 Odometry/String ROS publisher/subscriber，经真实 Humble MotionExecutor 的 take/execute 分派到生产 `NativeObservationRosBridge._callback` 与 `MotionCoreOutput._on_reference`；不直接调用回调、不mock executor。wheel publisher为 drive、reference为 motion_fusion，仅发布合成 Odometry/String，不发布 Twist。它证明入口选择与实际 DDS 回调通路，不是完整 SafetyGate/MotionFusion 启动或驾驶验收。

| 对照 | 结果与证据 |
|---|---|
| 原入口 main | 2 failed，明确 `entrypoint chose metadata-dropping executor`；`red-linux.log` SHA `89cc33dbe66860ea3614176850d1a88c24dca6252872d8c3f30158ce241a2dd7` |
| 修复入口 main | **2 passed in 0.35s**；两条 DDS_SMOKE均显示 executor=MotionExecutor，实际metadata keys只有 received_timestamp/source_timestamp；native wheel接纳，core reference sequence=1；`green-linux.log` SHA `4c2808326f7633fe95d32d7af1488543d595d07cab46daea11ca8122eb08d0f3` |
| 资源回收 | `linux-result.json`：两进程组269571/269603均reaped，remaining_owned_processes=[]，锁随runner退出释放 |

两次 Parallels 启动器失败发生在 Python runner 执行前：含空格路径被截断，以及过长 inline exec 参数被拒绝；最终用显式保留远端引号的短路径调用解决，未重跑任何驾驶场景。runner与完整日志保留在 E。

复现：新冻结目录（拒绝覆盖）复制同一源码/测试/manifest后，在上述隔离环境执行 `python -m pytest -s -q tests/ros/test_native_motion_executor.py`。对照以 `SENIOR_SMOKE_ENTRYPOINT_DIR=<冻结before-entrypoints>` 指向原 main。已有 E/run-vm.py 只适合记录中唯一新目录，勿直接重复覆盖；下一次须分配新run名。

## 未完成边界

完整节点启动与四个正常起停/转向驾驶仍等待本批复审后统一运行；没有本轮物理停止/5 km/h/实车通过声明。T1 acceptance null、候选profile、matcher门槛、T3/T4/T5数学与车辆输出策略均未改。本轮仅排除默认executor丢参和Humble缺GID全拒绝这两个启动阻断。
