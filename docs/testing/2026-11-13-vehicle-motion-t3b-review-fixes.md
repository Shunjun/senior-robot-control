# T3b 审核修复（2026-11-13）

状态：本轮三项修复及定向验证完成，待负责人继续亲审完整T3b；未提交/推送、未接运动输出、未启动Gazebo或开发服务。29项实测边界/真实统计模型仍未冻结，不宣布T3b生产资格或5km/h闭环通过。

本轮仅修改 `motion/temporal.py`、`motion/fusion.py`、`tests/unit/test_motion_temporal.py`、`tests/unit/test_motion_fusion.py` 和本报告。未修改matcher评分、原控制出口、旧ROS测试或T3a元数据校验；本轮无需扩大观测类型，未顺带重写本地import/重复结构校验。所有健康/时效/恢复状态仍归AdmissionOwner。

证据根目录 E：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t3b-review-fixes`。

## 1. 辅助源准入从同一快照传播

`_segments/_held/_gyro` 显式接收本次SourceSnapshot；共享 `_ready_history` 检查该快照中的source存在、ready、producer epoch及序号范围，再读缓存。缓存有数据不能代替本次快照的资格。GPS及wheel的gyro主窗、GPS heading传播前缀窗、协方差数值扰动均使用同一快照，没有新的健康owner或恢复计数。

- 快照反例：GPS已ready，将IMU.ready置False后，IMU仍报recovering，GPS改报dependency_imu_recovering、无原keys，不再进入GPS/gyro CI；独立laser速率参考保留。
- 真实owner序列：先建立健康GPS/wheel/IMU；.22s新IMU epoch负面证据中断，.24/.26s两份有效gyro仍未满足.1s连续恢复跨度。GPS/wheel各自ready，但两者投影均报dependency_imu_recovering；重复查询不推进计数。.34s由owner恢复后两者才覆盖，使用的IMU keys全部属于新epoch。
- 身份反例：快照IMU已换代而新事件尚未到解析器，旧epoch缓存不能被借用，返回dependency_imu_history_missing。

这些保证基于输入提供的epoch；原记录IMU/wheel缺真实publisher身份的限制未改变，不能因此声称验证了硬件重启/GID。

## 2. 分量冲突与明确的传输依赖

纯 `conflict_decision` 比较所有共同可观测的独立分量：laser/GPS vx、vy使用原注入linear阈值，laser/IMU yaw使用原yaw阈值。严格 `abs(residual)>threshold` 才拒绝，边界相等仍可用。残差逐项记录；blocked按 `(source, body-component)` 表示，输出failures指明具体轴，不武断归罪源。wheel与独立body/IMU的代理残差继续只作诊断，不能用轮速不一致单独判定IMU或GPS坏。

ResolvedSource新增兼容默认的不可变dependencies：GPS x/y和wheel预测x依赖IMU yaw；laser Delta经SE2 log转换后的x/y依赖自身Delta yaw。冲突决策计算有限依赖闭包，不锁存、不改变AdmissionOwner。声明的规范化速率分量无本层额外传输依赖；真实原生laser走Delta保留耦合。

验证：仅GPS横向5m/s反例现在只拒绝两份vy，纵向CI仍约.34m/s；仅yaw冲突拒绝双方yaw及依赖它们的GPS/wheel投影，仍保留独立的laser速率x/y。Delta形式的x/y不能伪称与其yaw独立。冲突解除的下个一致窗口可恢复使用，不建永久锁存。争议gyro不能经wheel预测fallback绕回来；纵向冲突改用wheel参考时，即使其余轴健康也只给注入的退化cap，不能恢复normal cap。无wheel证据的部分分量不冒称wheel模式，独立分量仍保留在mask中，完整average可空；这层不授车辆许可。

原测试“vx冲突后所有方向都不能CI”改为只拒绝vx并保留正常vy/yaw，这是本轮明确要求的行为迁移；原测试未删除，未改旧ROS测试。

## 3. gap/hold为一个一致的有效覆盖上限

保留现有两个注入字段以兼容调用者，消除内部两套不一致判断：**所有ZOH支持的有效上限统一为min(max_gap_s,max_hold_s)**。`max_gap`不是只在已知右样本到来时才检查的例外。已知相邻段、开放尾段、单左样本及 `_held` 都调用同一个 `_check_span`；不新增参数/等待/时钟或恢复owner。

原因选择统一：gap严格较小时超限为sample_gap，否则为hold_expired；相等时以hold为诊断优先。这不会放宽任何覆盖，修复gap<hold时尾段/单左样本的漏检。测试 .04/.2 及反向 .2/.04，恰好到有效上限通过，.1s尾段和held均拒绝；没有用两值相等掩盖漏洞。

## 测试与审核包

修改前实际206项全部通过，日志 `baseline-206.log`。新增测试初次RED有7个实际失败，见 `red.log`（测试组织后将快照级集成测试移到fusion测试文件，避免测试fixture双向import）。GREEN为原206项（上述一项断言按要求迁移）+16项新增，共 **222 passed**。新例覆盖辅助源真实恢复/快照/身份、两种不同gap/hold、仅横向/仅yaw、每轴阈值相等及nextafter越界、gyro依赖fallback、退化cap和Delta传输耦合。原正常异步30/50Hz、CI解析值、去重、失锁/漂移恢复均继续通过。

```sh
PYTHONPATH=.:src .venv/bin/pytest -q tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py tests/unit/test_motion_fusion_replay.py
.venv/bin/ruff check src/senior_control/motion/temporal.py src/senior_control/motion/fusion.py tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py
MYPYPATH=src:. .venv/bin/mypy --strict --explicit-package-bases -p senior_control.motion -m tools.motion_fusion_replay -m tools.motion_replay_records -m tools.motion_fusion_synthetic
```

Ruff、严格Mypy（16模块）、git diff --check和限定文件的新旧diff空白检查通过。`review-fix.patch` 仅含本轮五文件增量，`files-manifest.json` 含前后SHA，`source-final/` 保存对应源码/测试，`checks.json` 记录验证命令与退出码；index为空。

## 仅受影响原始记录重放

先审查既有逐窗证据（`unaffected-audit.json`）：observable-1/2没有GPS，所有已覆盖wheel窗口的IMU均ready且epoch一致，laser/IMU yaw残差最大约1.49e-16/1.70e-16rad/s，无旧sample_gap分支，因此无需重放这两份。未采集未见H1/H2。

仅turn和弱走廊受诊断优先级影响；重放输出另存 `E/turn-final/`、`E/weak-corridor-final/`，均使用原固定策略/相同原文件，不覆盖先前证据。turn 307窗，laser/wheel/IMU覆盖305/294/305；弱场338窗，覆盖0/327/335。准入计数、覆盖、同窗误差、模式支持时间和CI计数均与原输出相同。唯一资格诊断标签变化：

| 记录 | 原wheel失败 | 新wheel失败 |
| --- | --- | --- |
| turn | sample_gap 6 + hold_expired 5 | hold_expired 11 |
| 弱走廊 | sample_gap 5 + hold_expired 1 | hold_expired 6 |

turn实际laser/IMU yaw最大残差.086542rad/s，低于既有注入.15阈值；本轮没有为了数据放宽标准。新残差字段laser_imu_yaw随窗保留，原wheel/IMU不一致诊断仍有29窗，不改变归因。两份真实记录CI仍 **0**；trusted_statistical_models仍空，保留原P并不意味着有验证过的统计模型，未将真实P偷偷登记为可信。真实记录无GPS/已校准误差模型、virtual steady和记录级epoch的限制继续保留，数据也不能验证生产TTL或确定性真速界。

`replay-comparison.json` 列出逐项不变指标及失败标签变化；最终summary保存input/source/原admissions/windows SHA，`evidence-sha256.json`另保存报告、patch、日志与证据摘要。输入原文件SHA：turn `070b88f9da2d172c7d920dbf3671ae8618a274af6e0a576bdc52c24961558897`，弱走廊 `f89247d5fe630f3dbf8e7df5372b14c288b5e65241fe74aa8820c3f4f13216a7`。

T3b完整审核仍未结束，本轮不继续T4/T5接线、不提交推送；下一步由负责人亲读本次小diff与原始证据后决定。真实模型缺口、受限轮速风险边界、预测/响应及同版闭环验收没有被本轮合成测试替代。
