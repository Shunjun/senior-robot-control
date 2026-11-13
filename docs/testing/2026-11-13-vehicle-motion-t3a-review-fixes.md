# T3a 审核修复与 T3b 数值方案（2026-11-13）

状态：两项阻断修复交付，待负责人亲审；T3b 仅为本文具体方案，未添加融合实现。没有新测量、Gazebo/四组matcher重跑、提交或推送；产品仍未接运动输出。cap/29项实测边界未冻结。

## 修复与证据

1. 已有有效、甚至已经恢复的样本之后，旧 ingest 会先索引 source、hash epoch、比较 sequence，再校验结构。字符串/列表/None序号、不可hash epoch/source等可抛 TypeError，或破坏健康计数。现将无状态 `metadata_error` 前移至任何 owner 访问及 clock/水印变化前：验证实际Source枚举、epoch/sequence类型、有限数值时间、区间顺序/未来、frame/status及数值容器结构；GPS姿态在它实际被消费时才检查，不给激光增加无关姿态门槛。畸形输入直接拒绝，已有参考、期限、水印不变；有效的新鲜“失锁/质量失败”等负面证据仍按原语义中断健康并隔离更早包，二者不混淆。
2. `.5s max_age，2份/.1s恢复` 下，第一份(end=10,receipt=20)，第二份(end=11,receipt=20.1)之间无assess，旧版仅检查steady期限，错误地直接恢复。现 `is_current` 是唯一已接纳样本时效判断（固定steady deadline + 原ROS年龄），ingest接纳前和assess均通过同一 `_expire` 使用；过期后先清计数，新样本作为第一份。没有第二套年龄、截止期、RTF限制或网页延迟资格。

owner 保持唯一，ingest 分为元数据、源顺序、payload质量、姿态/epoch接纳、接纳与恢复几个短阶段；拆出私有函数，不新增策略类或状态框架。所有配置/恢复参数均未改变。

本轮仅改 `motion/observations.py`、`motion/source_admission.py`、`tests/unit/test_motion_sources.py` 和本报告。RED日志显示旧版11个失败（10个已健康owner的类型反例及ROS过期断档），另保留数值畸形元数据和“未消费姿态不新增门槛”的失败对照；GREEN源测试48项全部通过。覆盖无中间assess、ROS或steady分别过期、断档后重新两份恢复，以及连续RTF=1、2、0.5的正常对照。非法输入在已有健康owner上重复三次，随后合法seq3仍可接纳，旧deadline不刷新。

```sh
.venv/bin/python -m pytest tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py -q
.venv/bin/python -m ruff check src/senior_control/motion/observations.py src/senior_control/motion/source_admission.py tests/unit/test_motion_sources.py
.venv/bin/python -m mypy --follow-imports=silent src/senior_control/motion/observations.py src/senior_control/motion/source_admission.py
```

实际最终结果与小diff、源前后SHA和RED/GREEN日志放在 `$E`：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t3a-review-fixes`。旧报告和原冻结证据不覆盖。

## T3b 提交审阅的最小接口

下述均为提案，代码尚未实施；保留 T3a admission 的时效/健康计数 owner，避免让后续解析器再维护一套资格。新增时间模型不能从“点时间戳”凭空补一个测量区间。

| 值/接口 | 内容与消费方 |
| --- | --- |
| `PointObservation` | stamp_ros、原received_steady、source/epoch/seq、frame、实际观测分量、covariance/basis/correlation；用于原生wheel/gyro/GPS点速率 |
| `RateIntervalObservation` | 实际支持区间及速率参考；兼容现T3a规范化Observation，不能把任意dt伪装为测量支持区间 |
| `DeltaObservation` | 原[a,b]、body/sensor SE(2) delta、位移协方差及外参依据；消费真实ScanMotion，不拿位移协方差直接当速度协方差 |
| `AdmissionOwner.snapshot(now_ros, now_steady) -> SourceSnapshot` | 每源当前接纳值、ready、原deadline/epoch，clock状态；与现assess共用状态和时效函数。assess保留纯同窗兼容用途，T3b不以它的“精确同窗组合成功”作为输入前置 |
| `TemporalResolver.push(snapshot)` / `resolve([a,b], snapshot) -> ResolvedWindow | CoverageGap` | 有界历史、唯一原sample keys、覆盖/gap/held年龄。仅缓冲已接纳的不同样本，不累计健康，不改原receipt/期限 |
| `NumericalFusion.update(window, snapshot) -> FusionSnapshot` | 同窗估计及统计协方差、分量支持mask、来源/原keys、mode/cap/recovery；保留raw依据和被拒原因 |
| `predict_current(t, fusion_snapshot, response_model) -> CurrentReference` | 将区间均值与当前参考分开；有依据时才给可选deterministic upper。缺模型/上界仍None，不能把统计P当确定性速度上限 |

具体消费值建议为 `ResolvedWindow(interval_ros, per_source_delta, covariance, coverage, held_age, original_keys)`；`FusionSnapshot` 至少含：

```text
average: BodyTwist | None                 # 完整同窗SE(2)参考；缺分量时不构造
component_reference: (vx|None, vy|None, wz|None)
statistical_covariance: Covariance | None
upper: BodyTwist | None
observed_components, original_keys, source_epochs
source_mode, target_cap, recovery_phase, computed_steady, valid_until_steady
```

缺横向观测的wheel退化保留vy=None的分量参考，不以0填成独立观测；CurrentReference与区间average分开。现BodyMotion只在完整参考且时间语义适配后构造，upper依旧可空。

resolver 从快照剔除已中断/过期/换代来源的旧缓存；snapshot新增ready不是让resolver推测计数。当前T3a只保留最近值，故T3b需要在接纳时保存历史；历史容量由最大观测窗、允许年龄和已测最高频率导出、显式注入并有界，不作为浏览器资格或等待挑战。

## 时间对齐：不要求30/50Hz时间戳相等

以激光新窗口[a,b]为一次校正窗口，或以配置控制栅格作**预测**时刻；切分点取[a,b]边界与其中IMU/wheel/GPS实际采样时刻的并集。边界用历史左样本的有界ZOH；若两侧真实样本已到，可分段线性插值。缺右样本时只用仍合格的左样本，不等未来包；超过现有源允许保持/gap范围则返回该分量CoverageGap，保留其他源/限速退化，绝不无限等待或延长年龄。

例如窗口[10.000,10.100]，wheel约10.013/10.047/10.080（30Hz），IMU约10.010/10.030/10.050/10.070/10.090（50Hz）；有各自前沿历史值即可覆盖边界，以原采样时刻分段积分，无须任何一对stamp精确相等。左持有值的“覆盖”不是新测量，健康计数只由原不同样本推进，deadline不刷新。

body速率沿各段通过 SE(2) exponential 积分，再compose为窗口delta，最后log/dt，和激光同一[a,b]的log/dt比较；不能简单把转弯时各帧坐标里的vx算术平均当相同SE(2)窗口量。点GPS先在它自己的时间取得合格heading/外参再转换，禁止无姿态猜方向。窗口输出保留区间均值；50Hz预测单独标识时间和输入年龄，不能把10Hz旧scan改戳反复“测量更新”。每个原source/epoch/seq及窗口支持键只校正一次；校正必须至少包含该通道新的原测量键，仅改变窗口编号或持有旧值不产生新测量。

## 最小数值融合：三分量保守CI，不上大型滤波框架

建议第一版只做vx、vy、wz三个同窗标量通道：vx/vy来自合格laser和GPS；wz来自合格laser与IMU积分。wheel先作运动预测代理和滑移对照，不以轮式vy=0伪装独立横向观测，也不在未有滑移误差模型时把轮速作为可信真速测量注入健康通道。

每个通道两项输入采用固定w=.5的 Covariance Intersection：

`p = 1 / (w/p1 + (1-w)/p2)`，`m = p * (w*m1/p1 + (1-w)*m2/p2)`。

它不假设源间独立；一致性依赖输入的均值/协方差本身可信，不解决未知偏置或错误输入。原方法见 [Julier/Uhlmann 1997](https://ieeexplore.ieee.org/document/609105)。此处w=.5是待审的算法基线选择，不是测得能力/放行阈值；不在holdout之后择权重凑指标。非正可逆方差不加任意epsilon“修好”，而保留单源/部分支持及原因。

可检查数值：两份完全相关的同均值、p=.04输入，输出仍p=.04而不是独立融合的.02；m1=.3,p1=.01和m2=.5,p2=.04时，输出m=.34,p=.016，实际改变参考但未伪造精度倍增。通道间相关未知时可用保守 `3*diag(pvx,pvy,pwz)` 作为联合统计协方差包络；不把原full covariance的交叉项默认为0，原矩阵仍留存。该包络来自相关矩阵特征值不超过trace的上界，不是新增物理真速界。

GPS ENU转换使用 `v_body=R(-heading)*v_ENU`。其Jacobian为R及 `J_heading=[vy,-vx]`；heading与速度相关未知时，各输出分量用 `sigma_out <= sum(abs(J_i)*sigma_i)` 的保守一阶标准差包络，不能按独立方差相加。时间积分的未知跨样本相关也用同样保守包络，不因重复持有/采样变密而人为缩小协方差。非线性余项/heading误差适用域须验证，不满足就不将GPS body量标为合格；此为统计模型提案，不声称高斯P提供确定性停车保证。

## 轮滑移、失效和恢复

- 比较同窗wheel与独立laser/GPS平移、wheel yaw与IMU yaw，残差的未知相关标准差可保守取两侧sigma之和；创新门限/持续窗/偏置余量必须在实施验收前从开发数据和正常负对照冻结，不能本报告填假数字。
- 有独立观测时，轮速异常降低wheel预测可信度，不让wheel覆盖独立反馈。只有wheel+IMU时保留限速运动参考，横向/真速界不支持即None；不得因为激光弱特征或GPS缺失永久锁死。T4方向感知、T5轨迹/物理响应监督仍决定可行运动；降级接口消费component_reference及mode/cap，不把average或upper为空一概解释为永久禁行。
- GPS与wheel冲突而无独立参考，不能武断区分GPS漂移和轮滑移；报告冲突并降级。自动漂移检测需要创新/连续性和外部状态证据，CI本身不是检测器。
- 恢复沿用修复后的连续不同原样本+ROS/steady有效性；预测tick、重采样网格、同一scan重算均不计为新健康样本。更高目标cap只给T5平滑恢复，普通源失效与用户急停分开。

## 下一项可有限派发的实现与验收

建议先实现 `SourceSnapshot + Point/Delta时间契约 + TemporalResolver` 和独立标量CI纯函数；然后接入一个无输出的FusionSnapshot离线驱动，不修改ROS/T4/T5。必须测试：30/50Hz错相在有限覆盖下有结果、缺边界不等包、ZOH超过年龄不放宽、窗口constant-twist与SE(2)积分解析一致、同一sample键反复tick不重复融合/恢复、未知相关相同输入不减半方差、源退休后旧缓存不可复用、GPS无heading/失锁/注入漂移及wheel滑移不冒充真速。

现有真实仿真输入可直接来自 `t1-observable-corridor-r2/vm-low_forward-{1,2}/samples.jsonl.gz`、`t1-expanded-calibration/vm-turn-3` 和弱走廊 `vm-low_forward-5`（均在既有chat-scratch根），保留原epoch/clock/gap；truth只用于评分。GPS在现有bag中缺失，只能显式注入带标签的ENU/heading点数据及失锁/漂移负例，不宣称GPS硬件验证。先冻结开发/固定回归及未见holdout划分、输出支持/误差判据，再实现与比较，不更换窗口或降低旧matcher门槛。真实硬件、5kmh闭环、cap/响应实测仍是后续独立验收。

最终检查：48项源测试、178项相关回归全部通过；Ruff通过，项目strict配置下Mypy所改2个模块通过，tracked及新增文件whitespace检查通过。数值/模式/默认配置未调整，没有修改其他模块或既有阶段报告。T3b上述类型与CI算法均仅为待审提案，不存在数值融合代码或运行出口接线。
