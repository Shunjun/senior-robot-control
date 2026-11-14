# T5 同版正常运动链 Linux 调试：未通过

2026-11-13；task_423daf26a6ba。仅车端可信 manual IPC → CommandNode/T2 → SCF2 C++ guard → 唯一 T5 Twist → Fortress；跳过 cloud、Agent、SettingsService、网页及导航 runner。没有直接刺激执行器，没有使用 truth 作控制输入。未提交、未推送，T1 acceptance null 未修改。

## 结果与当前阻断

完整同版构建和实际节点构造成功，低档目标已通过真实生产者接纳，最终 r6 已产生真实运动，但正常起停未通过。低档前置条件失败，依授权没有执行高档起停、低档转向、高档转向。不能宣称四场通过、导航通过、网页整链通过或达到 5 km/h。

| 入口 | 实际结果 |
| --- | --- |
| low_start_stop | r1 启动失败；r2 接纳目标但没有运动；r4 未取得连续静止资格；r5/r6 有运动后因时效中断失败 |
| high_start_stop | 未运行；无实测稳态速度，无法评价与 25/18 m/s 的差距 |
| low_turn | 未运行 |
| high_turn | 未运行 |

r6 原生 bag：最终输出峰值 0.149668551 m/s、wheel 峰值 0.149416004 m/s、truth 峰值 0.149668551 m/s（均为瞬时峰值，非稳态）。目标 5/9 m/s，峰值也仅约 26.9%；测试观察者在异常前只收到 truth 峰值 0.107424028 m/s。原生 bag 比观察者更完整，两个数字不混用。

当前直接证据是正常请求/控制发布发生中断，并非速度资格门拒绝：

- `92350.365732` 的 drive.intent 成功，原 deadline `92350.565732`；之后下一请求直到 `92350.701588` 才发出，返回 unavailable。不能延长旧 deadline 掩盖这次真实断流。
- T5 sequence 118 sampled=`92350.341543`，119 sampled=`92350.690864`，间隔 **349.320 ms**；119 为 braking/`wheel_imu_limited:invalid_tick`，fault_revision 仍 0。150 ms 状态期限间出现约 199.320 ms 空隙。bag 同期输出/真值接收最大间隙约 339/341 ms。
- producer 发 generation 3/sequence 0、close_reason=expiry，反复重发仍保留原 received=`92350.365732`，deadline=0；没有用新时间复活旧授权。
- 后续原生输出从峰值下降，结束时仍为 0.069223462 m/s，truth 0.073464750 m/s，记录位移约 0.07424 m。测试已异常退出，finally 很快停止整个仿真；**这不是车体停止证明，也不是正常 release 验收通过**。
- r6 `/motion/runtime` median sampled 间隔 50.031 ms、max 349.320 ms；原生 body_reference 存在 28/130 帧，其余缺失如实保留。模式有 laser_reference 和 wheel_imu_limited，不伪造持续激光健康。

记录中的其他最大间隙包括启动/关闭阶段，不能一概归为行驶中丢样。ROS `/clock` 原始相邻 stamp 始终推进 1 ms，但 bag 接收有较大间隙；这证明需要区分仿真时钟与执行/采集调度，不证明某一个进程就是根因。VM 有 4 CPU；结束后的低 loadavg 不能代表运行中资源充足。

## 新冻结构建与运行隔离

VM：Parallels Ubuntu Linux 22.04、ARM64、ROS Humble、Fortress。新审计根：

```text
/home/toby/senior-audit-20261031/motion-normal-20261113-r1
```

宿主证据根 E：

```text
/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5-normal-r1
```

初始冻结 688 文件（含实际未提交源码），HEAD `3e225077b76773591c019f0efc22fc7fea44b860`；archive SHA `7fd4649113debefc0bbed52470a1d360836f3d6490075041cb45d211e996922c`。`freeze.json`、`working-status.txt`、`source-sha.json` 记录身份和脏文件。没有复制凭据/node_modules/旧构建，没有覆盖旧 r3 成果或私有 profile。

完整六包 colcon BUILD_TESTING=ON 成功，约 2min16s；实际编译 msg、C++ guard、ROS 包。`build-linux.log`、`build-result.json`、`identity-native-test.log` 保留日志。真实 FinalCommand 类型含全部新增来源字段；C++ frame roundtrip CTest 1 target 通过（内部五例）。guard 二进制 SHA `abd667f6bb96d2a6d311d53ec959ac9f697886901a0ba7452a6b5783aa9cf901`。

实际 ROS rclpy 来自 `/opt/ros/humble/local/lib/python3.10/dist-packages`；Python 依赖只读复用既有 venv，source/install 为本次新目录，不 overlay 旧 r3 install。后续修复每次复制前一冻结源码进入新的 source-rN、build-rN、install-rN，只重建 senior_robot Python 包；msg/C++ 保持上述完整构建同版文件。每版有 `source-rN-sha.json`、`patch-rN-sha.json`、安装摘要和构建日志；没有热改运行副本。

r3 及之后已纳入负责人批准的 T6 float 边界修复，command_node SHA `ba20f9db0a5cdc4a839625b83dac53e202c9690e2c25dfb33edcd5038ed2b14b`。其源码/协议/producer 未由本 worker 修改。

每场原独占 `.vm-test.lock`；domain173、LOCALHOST_ONLY=1、独立 Gazebo partition/socket/db/estop/日志。显式 `fortress-candidate-unvalidated-v1`、`qualified_reference=false`，因此只测手动链，不走无关地图 pin。candidate 保持已审无后附件几何。直道 world SHA `47cdf5be42009d10c317afdbbe9b64f14d11f26258230e611dcf6cc91deb13ab` 未改；转向 world 未运行。每场 systemd 独占 unit RuntimeMaxSec=175、stop=5s、KillMode=control-group，测试自身95s、IPC准备45s，并有外层210s watchdog。

`final-cleanup.json`：五个本次运行 unit 全部 inactive/dead，无 ControlGroup、无本审计路径残留进程；每场 run.json 还记录 reap 和 remaining_cgroup_pids=[]。unit 成功只表示包装器清理完成，测试的 `test_returncode=1` 才是功能失败，未混淆二者。中途个别 prlctl 返回 Invalid argument，先探查 unit/目录/锁，保留 probe；没有重启 VM 或盲目重复活跃运行。

## 按证据实施的小修复

1. `safety_node._emit_motion_core` 将内部整型零/速度统一转换为 ROS double。严格 setter RED→GREEN；不改变速度值。r1 中原真实 ROS setter 抛错；修复后不再出现。
2. `simulation_motion.launch.py` 将 candidate 的无后附件模型选择与旧 qualified_reference 地图分支分离；不改世界/模型物理参数或地图 pin。旧默认附件仍保留。
3. `motion_core_output` 的 revoke/event/close 使用一个已采样 steady now。r2 release 原先嵌套重采后再使用旧 now，触发假 clock_fault；回归复现后修复，r6 expiry/release 无新增 fault revision。
4. 原 T3 source steady deadline 与 ROS 当前预测域分开：反馈沿用原 received/deadline；当前投影按节点当前 ROS now 减原区间 end 检查 0.15s ROS horizon。删除把该 ROS horizon 再当 0.15s steady TTL 的错误。r4 的正常慢 RTF 会使连续静止恢复始终中断；定向测试覆盖慢 RTF、真正 ROS 过期与原 steady 过期，不延长源证据。
5. 仅独占 candidate perception adapter 识别经核实的 +Inf 无回波（下节）；shared self_filter 和 matcher 未改。
6. r6 candidate 控制调度由40Hz改为20Hz（旧普通默认也是20Hz，legacy qualified_reference 仍40Hz）。离线真实 scan 输入微剖析发现单次 prepare_perception 约37ms，超过40Hz的25ms周期；12 tick median6.52ms、max47.04ms。**这个调度调整没有解决 r6 的349ms间隙，不能宣称实时预算合格**。user TTL=.2、runtime validity=.15、预测域=.15、测量过期、刺激段/阈值均未放宽。

观察器测试只新增原请求/响应时序记录；原刺激与80%候选阈值未修改。旧失败原始曲线仍保留，不用新理想数据覆盖。

## +Inf 契约证据与限定适配

已安装包：ignition-sensors6 **6.9.0-1~jammy**、ignition-rendering6 **6.6.4-1~jammy**、ros-humble-ros-gz-bridge **0.244.26-1jammy.20260916.153147**。对应官方 tag 源码及 REP117 全文保存 E/lidar-contract，URL/SHA 见 `sources.json`：

- [GpuLidarSensor.cc，sensors6 6.9.0](https://raw.githubusercontent.com/gazebosim/gz-sensors/ignition-sensors6_6.9.0/src/GpuLidarSensor.cc)：FarClipPlane=RangeMax，SetClamp(false)，注释指明超出范围按 REP117 映射 ±Inf。
- [Ogre2GpuRays.cc，rendering6 6.6.4](https://raw.githubusercontent.com/gazebosim/gz-rendering/ignition-rendering6_6.6.4/ogre2/src/Ogre2GpuRays.cc)。
- [ros_gz_bridge 0.244.26](https://raw.githubusercontent.com/gazebosim/ros_gz/0.244.26/ros_gz_bridge/src/convert/sensor_msgs.cpp)：range_min/max 与 ranges 原样复制到 ROS LaserScan。
- [REP117](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0117.rst)：+Inf 为量程内无探测物语义，NaN/-Inf 不等价。

r2 实际有效 raw/bridge 帧：laser_link、720 束、range_max20、405 finite、315 +Inf、0 NaN，前方中央为 +Inf；默认 UNKNOWN 时前后(.45,0)不在已知自由区，目标虽被接受但输出全部0，truth峰值约1.45e-11。无 return 不是有限障碍，也不是无条件整片 free。

新策略仅在显式隔离 candidate、simulation_only/unvalidated、laser_link/range20 的已核实契约中启用。保留原始 ranges=+Inf、原 invalid rays 和 external_hits；另构造带来源 basis 的 **NO_RETURN** 自由 patch，最多到有效 range_max。有限 hit 不被造假；混合窗口也标 NO_RETURN。射线经过自遮挡则拒绝；NaN/-Inf/缺束/盲区仍 unknown。默认策略仍 UNKNOWN，shared self_filter.py 未改，没把 no_return_clear flag 当已支持。

该映射不是 matcher 改善，也不是实车激光泛化契约；真实硬件不得启用这些候选假设。r4/r5/r6 不再出现原 `sweep_unknown`，但仍有独立的时序失败。

## 测试、原始证据与复现

本地针对回归 **56 passed**：

```sh
PYTHONPATH=.:ros2/src/senior_robot .venv/bin/pytest -q \
  tests/unit/test_motion_core_perception.py tests/unit/test_motion_core_handoff.py \
  tests/unit/test_motion_core_chain.py tests/unit/test_motion_ros_twist_output.py \
  tests/unit/test_motion_launch_profile.py tests/unit/test_directional_constraints.py \
  tests/unit/test_self_filter.py
MYPYPATH=src:ros2/src/senior_robot .venv/bin/mypy --strict --follow-imports=silent \
  --ignore-missing-imports --explicit-package-bases \
  ros2/src/senior_robot/senior_robot/{motion_core_output,motion_core_perception,simulation_motion_profile}.py
```

r6之后宿主仅补 Compatible 标记及更正接缝注释，没有功能改动；安装文件仍按 r6 manifest 身份记录，未热改。

严格 Mypy 三模块成功；ROS 动态依赖按既有边界 silent/ignore，不声称完整 ROS 静态类型验证。限定11源码/测试 Ruff 通过。本阶段 scoped diff 检查通过；全仓 `git diff --check` 返回2，来自未改动的9个 T6/任务文件的 CRLF/trailing whitespace，详见 E/diff-check-global.txt，本 worker 未越权清理。RED 日志、初次未配 Python 路径导致的收集失败均保留 `/private/tmp/t5-normal-review`；修正运行环境后重跑，没有改断言来通过。

E/low_start_stop-r{1,2,4,5,6} 各自保存原生 bag、launch/test/junit、curve.csv、case/result.json、evidence-sha.json。r5/r6 另有原生 ROS 反序列化的 `motion-native.csv`、`targets.json`、`runtime-native.json`、`reference-native.json`、`native-summary.json`；truth 仅在这些评分/记录路径消费。`native-export.log` 与 export-native.py 可复查全部导出。最终 `review/evidence-sha.json` 补齐导出数据 SHA，`review/review.patch` 和 `files-sha.json` 记录本阶段增量及前后源码身份。

Linux 相同入口（需要下一次明确新 attempt，不能覆盖既有目录；本报告结束时不启动）：

```sh
prlctl exec 'Ubuntu Linux 22.04' --current-user \
  timeout --signal=TERM --kill-after=5s 210s python3 \
  '"/media/psf/Home/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5-normal-r1/run-vm-case.py"' \
  low_start_stop NEW_ATTEMPT
```

其余三个入口名为 high_start_stop、low_turn、high_turn；只有低档完成再顺序执行。本次 runner 绑定 source/install-r6，不能在没有新来源绑定时误称运行宿主最新源码。

## 最小下一步

暂停高档依赖是因为低档时序/正常起停尚不可行，不是等待全部29项。下步先在同一低档正常路径做一次**有界回调/线程耗时取证**：同时记录 SafetyGate tick各阶段、worker与 CommandNode/测试请求循环的 wall/CPU/调度间隔，定位349ms缺口；保留原TTL、原状态时效与证据期限，不增加健康门或宽限。当前仅证实 geometry preprocessing 曾超过25ms周期，**不足以解释全部349ms**，不能据此盲做异步重构或归罪VM。

按定位结果将已证实重活移出控制快路径/合并重复几何工作，并保留新scan危险立即收紧语义，再新冻结重复低档；正常连续制动和最终车体静止需要重新完整取证。T6 teardown 还记录 context invalid 时 shutdown barrier publish 异常，它发生在本次SIGINT回收时，非本轮行驶阻断；由其owner处理，本 worker 未改它。真实 CI 模型、激光移动误差、物理响应/地面适用范围仍未冻结，不以本轮峰值或合成 profile 声称安全资格。
