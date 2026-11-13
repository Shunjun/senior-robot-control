# 诊断审核修正与 T3a 多源基础（2026-11-13）

状态：本轮实现交付，待负责人亲审。T3a 是不接输出的源准入、适配、参考/组件选择与恢复策略；没有完成 T3b 数值融合/预测或生产控制资格。未运行新 Gazebo/manual/nav/高档，未提交/推送。T1 的29项实测边界和退化 cap 未冻结，合成配置不替代硬件/5kmh验收。

## 诊断审核修正

- 新 `diagnostic_isolation.hpp:emit_scan_diagnostics` 把 **整个**独立 DiagnosticArray 构造、序列化与发布包在 catch-all/noexcept 边界，返回观测是否发布；只读接收 motion，没有重置/修改有效性/epoch 的能力。node 的 `publish_diagnostics` 同为 noexcept，失败只允许一次有界警告，警告本身也隔离异常，仍 best-effort/keep-last-1，无可靠积压。
- `current_diagnostics_` 交接改为 move，定向测试 static_assert 其 move assignment 不抛异常，避免大 try 内诊断复制分配错误再次触发 invalidate。原 motion publisher、valid 判定、epoch/reset 与评分阈值未改。
- 故障注入 RED：无隔离的诊断构造/发布路径抛 `std::bad_alloc`，GTest 实际失败；GREEN：bad_alloc、非标准异常均被隔离，原 motion.valid/reason/epoch/body_delta 保持，正常与 initializing 发布身份正确。测试针对 node 实际调用的同一个 emitter，不增加测试专用 runtime 开关/虚拟类。
- 新增 C++ 的多语句行与长分支作局部整理；spread 先检查有效束数，再显式计算/存储局部 double 并判断，不再借 optional 短路解引用；旧算法不作大面积格式化。
- replay 核对实际 node：body.source_epoch 必须等于 owner.owner_epoch，孤立 body 重启不应直接准入。准备器现在对此执行 reset 并丢弃该帧；一致的 owner/body 换代仍按来源 key reset。新增反例先失败后通过。manifest 保留原始 epoch，工具仍不声称重演整个 DDS/TF/receipt admission。

Linux 独立路径：`/home/toby/senior-audit-20261031/t3a-diagnostic-isolation-20261113-r1/{red,green,final}-{source,build}`。原锁、ROS173/LOCALHOST_ONLY、build 240s 外层 timeout、测试15s/回放90s 上限、自身进程组清理。最终5个CTest target全部通过：23个C++用例 + 1个CLI契约。四组已有数据逐帧 replay 与旧 baseline 的 valid/reason/start/sensor_delta 无变化（382/381/307/338帧），没有再测场地。原冻结 r3 source/install 564/1004 文件零差异，本轮残留进程0；actual source SHA 与工作区对应 C++ 文件逐一一致。

## T3a 职责与接口

| 文件 | 职责 |
| --- | --- |
| `motion/observations.py` | 不可变 Observation/Covariance/Heading/配置，纯质量、帧、时效、PSD校验及坐标参考适配 |
| `motion/source_admission.py` | 唯一 clock/epoch/sequence/原始接收 deadline 与连续恢复状态 owner |
| `motion/source_modes.py` | 纯参考选择和模式目标约束，返回 MotionAssessment；不授运动许可 |
| `motion/estimate_adapters.py` | 复用现有 MotionEstimate.body_twist，要求显式速度协方差，不复制旧估计链 |
| `motion/contracts.py` | BodyMotion.upper 改 Optional；qualified 只表示参考质量，不表示确定性界或 T4许可 |

`AdmissionOwner(config).ingest(observation, *, now_ros, now_steady, heading=None) -> AdmissionDecision`；`assess(*, now_ros, now_steady) -> MotionAssessment`。所有源阈值、恢复数量/跨度、cap、姿态有效期/误差及 clock-pause/process-history 容量都显式注入，没有运行默认参数落盘。

输入保留 Source、Frame、原 ROS区间、原 steady接收、epoch/seq、quality/status、协方差种类/依据/相关组。laser/wheel 为 body vx/vy/wz，GPS 为 ENU east/north，IMU 仅 body yaw rate；不接加速度三轴积分车速。完整协方差检查有限、对称、对角和PSD；对角-only 明确相关未知，保留原值，不补独立交叉项。非法大有限交叉项不会在平方时溢出。GPS 需要有依据、方差合格、时效/顺序/来源健康的 Heading；保留完整 Heading context，旋转参考值后 body covariance 保持 None，不假设姿态与速度独立。

owner 拒绝 future/expired、乱序重放、退休 epoch、迟到的新 epoch、失锁/显式 drift、非有限/非法输入。新鲜但无效样本也推进合法时序水印，防止它之前的迟到好样本恢复健康；非法 future 不污染水印。ROS/steady 回退中断并退休来源，暂停中断健康；原样本不会换接收时间或 deadline。恢复数量及跨度只由不同的严格推进样本更新，重复 assess/重复样本不累计。时钟重建后旧来源继续拒绝，新 clock-generation/source/heading 可重新取得健康，不把旧姿态水印当永久封锁。

模式为 laser_reference / gps_imu_reference / wheel_imu_limited / recovering / unavailable。当前明确按顺序选参考；GPS与IMU、wheel平移与IMU yaw仅在 **同一原始区间** 做组件组合（仅允许浮点表示的一ulp差异），不对不同窗口插值/预测并伪称同窗平均。未能对齐时保留 available_sources、返回 interval_alignment_required，交 T3b。每个模式所需源均需配置的恢复样本/跨度；缺 IMU 时本阶段不构造 GPS/wheel 完整 body 参考。

MotionAssessment 包含 contributors/available_sources、原始质量/协方差/姿态依据、mode、recovery_phase、age_steady_s 和 target_linear_cap_mps。BodyMotion 保留 primary 原 ROS区间，valid_until 来自已固定的输入 deadline，**本阶段所有 upper 均 None，uncertainty_supported=False**。cap 只是给未来 T5 的目标约束，恢复上限不在此平滑，也不对测量值限幅；T5轨迹负责恢复不跃升。障碍感知与方向准入归 T4，本模块无 ROS/cloud/UI/许可/速度发布调用。

### BodyMotion.upper 消费点盘点

`rg -n 'BodyMotion' src tests --glob '*.py'`：产品侧只有 `motion/contracts.py` 声明与本次 `source_modes.py` 纯构造；现有 `test_motion_contracts.py` 对有界测试值增加非None断言，本次 `test_motion_sources.py` 验证无界为None。无 ROS/safety 输出节点消费此类型，未做运行链迁移。manual_session/manual_authority/package入口字节未变，相关现有回归通过。

旧 MotionEstimate 的 covariance 是位移量，不能简单除 dt² 声称完成 SE(2) twist/cross-covariance 转换。`from_motion_estimate(..., velocity_covariance=...)` 要求调用方提供有依据的速度协方差；没有完整 body_twist 时明确不准入，不用旧 Velocity 的缺失 vy 补0。该适配不修改旧 MotionEstimator。

## 可直接消费的合成示例

以下脚本已真实执行通过，只验证接口。cap=.2、恢复1份/0s 等均为示例，**不是车端冻结值**，不能连接运行出口。

```python
from senior_control.motion.observations import (
    AdmissionConfig, Covariance, CovarianceKind, Frame, Observation,
    Source, SourceRule, SourceStatus,
)
from senior_control.motion.source_admission import AdmissionOwner

# Synthetic values demonstrate the API; none is a frozen vehicle limit.
config = AdmissionConfig(
    rules=tuple(SourceRule(source, .5, .3, .3, 2.) for source in Source),
    recovery_samples=1, recovery_span_s=0., normal_cap_mps=1.,
    degraded_cap_mps=.2, heading_max_age_s=.2, alignment_max_skew_s=.1,
    heading_max_variance_rad2=.05, clock_pause_s=.3, epoch_history_limit=8,
)
owner = AdmissionOwner(config)
for source, values in [(Source.WHEEL, (.31, 0., .12)), (Source.IMU, (.1,))]:
    observation = Observation(
        source, Frame.BODY, 10., 10.1, 20., "demo-epoch", 1, .9,
        SourceStatus.VALID, values,
        Covariance((.01,)*len(values), CovarianceKind.DIAGONAL_ONLY,
                   "synthetic variance; correlation unknown", None),
    )
    assert owner.ingest(observation, now_ros=10.1, now_steady=20.).accepted
result = owner.assess(now_ros=10.1, now_steady=20.)
assert result.mode.value == "wheel_imu_limited"
assert result.motion is not None
assert result.motion.average.vx == .31       # Never clip the measured reference to the cap.
assert result.motion.average.wz == .1        # Same-window IMU yaw component.
assert result.motion.upper is None          # No invented true-speed bound.
assert result.target_linear_cap_mps == .2   # Target constraint only; not a permit/output.
```

## 验证与当前边界

本地：159项相关测试通过（新增T3a 29项、replay 3项及现有 contracts/manual session/authority/MotionEstimator回归）；所改6个Python模块严格Mypy和Ruff通过。测试覆盖有/无GPS、弱激光退化、IMU/轮非法值、失锁/注入drift、GPS姿态质量/换代、恢复计数/跨度、双时钟/时效/重放、epoch残留、同窗限制、相关协方差不假独立及遗留适配。RED证据包括先前被错误准入的迟到样本/姿态、溢出协方差和误拼区间等，再最小修正；同一有效样本 repeated tick 不变健康/期限。

```sh
.venv/bin/python -m pytest tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py -q
.venv/bin/python -m mypy --follow-imports=silent src/senior_control/motion/contracts.py src/senior_control/motion/observations.py src/senior_control/motion/source_modes.py src/senior_control/motion/source_admission.py src/senior_control/motion/estimate_adapters.py tools/scan_matcher_replay.py
```

Ruff 使用同一模块和3个定向Python测试文件；`git diff --check`及新增文件no-index whitespace检查通过。C++驱动与实际测试命令、完整日志、源SHA、回放输出在 `$E`：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t3a-foundation`。`red-tests.log` 为故障注入失败，`final-tests.log` 为最终完整CTest；`vm-final-verification.json` 为原r3身份/回收与各帧对照；`diagnostic-review.patch` 与 `t3a-review.patch` 分别供负责人独立审核。

尚未完成：自动GPS漂移/滑移创新检测（本测试用显式SourceStatus故障注入）、原始ROS驱动接线、厂家GPS点测量/IMU采样到规范化区间的时间模型、跨源统计融合/预测、姿态相关及SE(2)速度协方差变换、确定性真速/当前界、响应模型、真实cap标定。source admission不证明传感器实装标定正确。未运行新物理仿真，更未证明恢复平滑、障碍方向感知或5kmh闭环已通过。

T3b最小后续：先定义规范化点/区间和坐标参考、异步对齐/有限预测及未知相关时的保守协方差规则；用已保存数据和故障注入验证真实body yaw与wheel差异、GPS漂移/失锁、组合失效及恢复，不能用固定优先级选择充当融合。独立明确统计不确定性与有适用条件的确定性上界，缺依据仍None；随后按适用实测验证内部退化cap/响应，再接T4/T5。旧弱特征负例保留，未见holdout仍按上份报告固定布局/计划/预先判据登记，不使用本轮合成配置放行控制。
