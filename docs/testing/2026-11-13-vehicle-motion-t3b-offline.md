# T3b 异步多源融合纯离线交付（2026-11-13）

状态：T3a 已按授权独立本地提交；T3b 实现与离线证据待负责人亲审，未提交/推送，未接任何运动输出。T1 的29项正式边界、退化cap/响应模型仍未冻结；本报告不是5km/h、实车或产品闭环通过声明。未运行Gazebo、硬件、manual/nav，也未采集H1/H2。

## 提交与文件边界

T3a commit `0959041f3a4e82ac495b3a9ebc50b7456f358062`：仅 `motion/{contracts,observations,source_admission,source_modes,estimate_adapters}.py` 和 `tests/unit/{test_motion_sources,test_motion_contracts}.py` 共7文件。初始index为空，commit后及本轮结束检查仍为空；其他T1/诊断/docs脏文件未纳入。

本轮T3b：

- 新增 `src/senior_control/motion/{sample_types,admission_events,kinematics,temporal,fusion}.py`；必要修改 `motion/{observations,source_admission,source_modes}.py`。manual session/authority及contracts未继续修改。
- 新增 `tests/unit/test_motion_{temporal,fusion,fusion_replay}.py`、`tools/motion_{fusion_replay,replay_records,fusion_synthetic}.py`。
- 同步获批spec/plan前文依赖、Task3b实际接口及本报告。matcher/校准工具/旧ROS测试/safety输出/ROS/cloud/UI均未修改。

审核包根目录（下称 E）：
`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t3b-offline`。
`review-fix.patch` 是相对本轮开始快照的限定diff；`files-manifest.json` 保存原/现SHA与文件清单，`source-final/` 保存实际实现及测试。原始记录、调试版输出均保留，未覆盖冻结r3。

## 已实现的消费链

1. `PointObservation` 保留真实点时刻；`RateIntervalObservation` 为旧Observation的兼容别名；`DeltaObservation` 保留原SE(2)位移、协方差和T_body_sensor，不把位移P直接用作速度P。本轮旧速率区间桥用于laser；wheel/gyro/GPS走原生点，其他旧Observation仍由兼容assess消费，误送新解析器不会伪造覆盖。
2. AdmissionOwner仍是唯一健康、双时钟、epoch、顺序和连续恢复owner。可选有界事件journal/游标给出快照间全部接纳样本及中断；latest快照不会丢burst。畸形元数据不改变水印，fresh负面证据/换代会清历史。原assess保留旧规范化区间用途，原生点/Delta只走新链。
3. TemporalResolver只缓冲已接纳证据，容量/历史/hold/gap/预测域显式注入，使用因果ZOH，不等待未来、不要求30/50Hz精确同戳。过期判断复用原is_current；中断/换代/游标缺口清缓存。缺覆盖、恢复中、超log旋转域均返回明确原因。gyro仅yaw；wheel须声明非完整侧向运动假设才给SE2 **预测** delta，vy观测仍None，无该假设不伪装成区间body均值。
4. GPS ENU点需已知heading、同窗gyro和已知外参；各段先做ENU位移与杠杆臂修正，再compose/log。heading与非线性协方差传播为一阶近似（数值Jacobian），不声称余项已界定。完整常twist/转弯、90度heading、sensor外参、原位移P回归都有解析测试。
5. NumericalFusion对显式可信模型的同窗分量执行固定w=.5 CI；用方差比缩放，避免直接倒数溢出，不给零方差加epsilon。未知源间/时间相关不假独立；同均值/同方差不减半；`.3/.01 + .5/.04 => .34/.016`。联合统计对角包络按支持维数d取d*diag，原矩阵/交叉项另存。其依据仍依赖输入模型及一阶近似，不解决偏置、错误匹配或确定性停车保证（[CI原论文](https://ieeexplore.ieee.org/document/609105)）。
6. 分量mask、原source/epoch/seq和时间窗、源年龄/ready/deadline、残差/失败、目标cap均在FusionSnapshot；只有全部分量存在才构造average。upper始终None。CurrentReference保留原点时刻，predicted_at=None；无加速度模型不把区间均值重标当前。轮参考不进入独立统计融合。融合仅维护每源信息序号水印，不递归融合旧后验；重复tick/查询重编号/倒序查询不产生新信息或恢复计数。

显式GPS失锁/漂移、IMU异常、激光弱场经同一owner降级/连续恢复；重复tick不凑恢复数量。窗口冲突策略为注入阈值的本窗拒绝/诊断，不能据不足证据给GPS或滑移定责，也不另建健康锁存器。源健康恢复归owner，cap恢复平滑归T5，方向感知与车辆许可归T4。`average=None`/`upper=None`不自动将合法轮速退化变成永久禁行。

可直接消费的接口（仅示例策略，非生产配置）：

```python
from tools.motion_fusion_replay import ReplayChain, replay_policy

chain = ReplayChain(replay_policy())
# sample为保留原时间/epoch/协方差依据的Point/RateInterval/Delta，不含truth。
decision = chain.ingest(sample, now_ros, now_steady, heading=known_heading_or_none)
snapshot = chain.window(start_ros, end_ros, now_ros, now_steady)
vx_reference = snapshot.components[0].value  # 同时消费mask、mode、source_health与原因
# 不据这个值直接发车；upper可能None，当前参考另有原source_times_ros。
```

## 验证与RED证据

数据前的 `E/acceptance-before-run.md` 登记了CI abs1e-12、解析SE2 abs1e-9、异步覆盖/缺口/去重/模式断言与数据划分；真实精度未设通过线，也未事后冻结cap。合成传播CI端到端容差abs1e-9；不以有限差分误差宣称严格保守物理界。

RED后修复包括：新模块缺失；GPS各样本有效heading被误要求早于整个窗口；小键缓存淘汰使旧样本被重新算成新信息（改为每源序号水印）；同一ROS整数纳秒的两种浮点转换造成ULP级未来误拒（统一整数ns/1e9，无新增容差）；大于pi的sensor delta被SE2 compose静默wrap（明确拒绝log域）；不声明侧向假设时仍将wheel标量当SE2均值；原生适配器丢弃已有轮协方差交叉项（现保留原子矩阵、model仍未验证）。最后一个wheel模型RED原日志在 `red-wheel-assumption.log`，其余首次失败见本轮工具记录；正常负对照均保留。

实际结果：既有178项回归通过，新增28项通过；Ruff通过；严格Mypy检查16个模块通过；git diff --check通过。`checks.json` 和各同名log记录命令、退出码；无旧测试改写来造通过。

```sh
PYTHONPATH=.:src .venv/bin/python -m pytest -q tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py
PYTHONPATH=.:src .venv/bin/python -m pytest -q tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py tests/unit/test_motion_fusion_replay.py
.venv/bin/python -m ruff check src/senior_control/motion tools/motion_fusion_replay.py tools/motion_replay_records.py tools/motion_fusion_synthetic.py tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py tests/unit/test_motion_fusion_replay.py
MYPYPATH=src:. .venv/bin/python -m mypy --strict --explicit-package-bases -p senior_control.motion -m tools.motion_fusion_replay -m tools.motion_replay_records -m tools.motion_fusion_synthetic
```

## 真实记录离线结果（Gazebo记录，非实车）

开发集observable-1；observable-2、旧turn及弱走廊为已见固定回归。输入均为原生bag的无损JSON导出，不将capture缺流伪造成合格源。最终输出目录为 `E/{observable-1,observable-2,turn,weak-corridor}-retained-cov/`，每份含admissions.jsonl、windows.jsonl、curves.csv、summary.json。每窗保留原ordinal/keys、所有覆盖失败、原ScanMotion valid/reason、原协方差、同窗truth评分；truth只进入后置评分对象，不进入配准/融合/控制。

| 记录 | 查询窗/可truth评分 | laser/wheel/IMU覆盖窗 | laser参考/轮IMU受限/不可用支持秒 |
| --- | --- | --- | --- |
| observable-1 | 382 / 380 | 237 / 368 / 380 | 23.700 / 13.100 / 1.302 |
| observable-2 | 381 / 379 | 235 / 378 / 379 | 23.500 / 14.300 / .202 |
| turn | 307 / 306 | 305 / 294 / 305 | 30.500 / 0 / .200 |
| 弱走廊 | 338 / 335 | 0 / 327 / 335 | 0 / 32.700 / 1.100 |

支持秒为查询区间之和，不冒充整场wall时长或运行许可。四份原生输入均无GPS、无已验证统计模型；真实数值CI窗数均0。实际原轮vx/wz协方差子矩阵/IMU z方差保留（包括零），但model_id=None、trusted registry为空，不能因发布了数字就自动合格。激光valid及阈值完全沿用原节点。

整体低误差掩盖了静止占比，故另作描述性切片（abs truth vx>.05m/s或wz>.05rad/s；**事后描述分组，不是验收阈值**）：两个observable均有76个移动窗，laser覆盖均0；旧弱走廊71个移动窗，laser覆盖0。旧turn 47个移动窗laser均有覆盖，但vx MAE .108997/max .288458m/s，vy MAE .060467/max .237389m/s，yaw MAE .029372/max .088315rad/s；不能将valid当精度通过。移动窗wheel vx MAE：observable-1 .009224、observable-2 .009037、turn .010179、弱走廊 .009393m/s（分别76/76/46/69窗），仅适用于这批平地、既定摩擦/速度记录，不是轮速真速上界。wheel-IMU yaw不一致有29窗，使用预先注入.15rad/s阈值，只报告不一致、不武断归罪源或滑移。曲线中wheel_yaw_proxy与IMU/truth分列；真实车身与轮速的物理偏差不能靠轮速闭环自行消除。

剩余未来时间拒绝：observable-1 42条（header领先bag ROS receipt 1–2ms）、observable-2 60条（1ms）、turn 149条（1–4ms）、弱场93条（1–301ms，含启动）。无源时间回写或放宽门槛。回放now_ros取bag receipt；原接收steady及各节点实际clock状态不可恢复，virtual steady仅累计非负bag receipt差。因此这些是此保序离线时钟模型下的拒绝，**不能直接认定产品节点收到了未来消息或验证真实TTL/RTF**。原序列/缺口完整留存，clock receipt回退均0；不跨过期/无效/换代窗拼接。

宿主Python3.12并行离线链step（含日志序列化）P95约 .40–.48ms、max约5.8–8.7ms，仅开发成本记录，不冻结Linux实时预算。所有有界本地重放已退出，无VM/仿真进程需要回收。最初调试输出、统一ns后的r2、显式wheel模型守卫后的final、保留原协方差后的retained-cov目录均另存；后两次所有资格、覆盖、模式、误差指标与r2完全相同，不是调整算法/阈值取巧。

输入SHA256：

| 输入 | SHA256 |
| --- | --- |
| t1-observable-corridor-r2/vm-low_forward-1/samples.jsonl.gz | 08ac981765cd64112c3907d2768bd8a3de65120af3d51542f9d2152c2a4b96f1 |
| t1-observable-corridor-r2/vm-low_forward-2/samples.jsonl.gz | af2d1c541c8358bfe78b1a714be005ca362d698e1ecbd131932c9c03814f0b55 |
| t1-expanded-calibration/vm-turn-3/samples.jsonl.gz | 070b88f9da2d172c7d920dbf3671ae8618a274af6e0a576bdc52c24961558897 |
| t1-expanded-calibration/vm-low_forward-5/samples.jsonl.gz | f89247d5fe630f3dbf8e7df5372b14c288b5e65241fe74aa8820c3f4f13216a7 |

上述相对输入路径以E的父目录为根。输出summary/source/CSV等SHA见 `E/evidence-sha256.json`；summary内另记录实际源码和原admissions/windows SHA。wheel/IMU无DDS生产者身份，明确使用recording-scoped epoch；laser保留原producer source_epoch，bag未含GID，不能宣称还原所有发布者重启信息。

重现（NEW_OUTPUT须不存在；默认上限500000条）：

```sh
PYTHONPATH=.:src .venv/bin/python -m tools.motion_fusion_replay --input INPUT_SAMPLES_JSONL_GZ --output NEW_OUTPUT
PYTHONPATH=.:src .venv/bin/python -m tools.motion_fusion_synthetic --output NEW_SYNTHETIC_JSONL
```

## 合成输入单列

`tools/motion_fusion_synthetic.py` 可运行消费同一链，30/50Hz点+10Hz区间，标明synthetic-known统计模型、理想已知heading；不是硬件/GPS精度证据。`E/synthetic-final.jsonl` 八窗实际轨迹：.1s GPS+IMU参考，.2s CI（vx≈.34，P≈.016），.3s GPS显式漂移+laser弱场后wheel+IMU限速参考，.4/.5s组合IMU失效/恢复未满而不可用，.6/.7/.8s连续健康恢复后CI。upper全None，退化vy全None，不给未观测横向补0。重复tick、失锁与漂移各自的恢复、极小/零方差、时钟/缓存/源换代反例另有单测。

## 尚缺与可派发下一步

T3b的点/区间/Delta、有界异步解析、固定CI纯核及实际离线消费链已实现；尚未实现有依据的当前加速度预测/确定性真速界、响应模型或任何生产接线。所有演示cap/hold/恢复配置是注入的逻辑试验配置，不改五项驾驶设置、不冻结29项参数。

下一有限实现可消费FusionSnapshot的分量mask/source mode/age/目标约束，开始T4纯方向约束接口；不以average/upper为空永久禁止轮速退化。生产闭环仍缺：适用摩擦/滑移/传感器模型与降级风险边界；IMU实装姿态/零偏标定及GPS真实能力核验；T4感知/停车扫掠许可与T5轨迹/反馈唯一输出及平滑恢复；冻结后同版多场景/未见H1/H2/5kmh运动导航验收。弱特征/移动scan失败依旧真实存在，估计器修复须独立持出验证，不能为本链放松matcher或伪造P。
