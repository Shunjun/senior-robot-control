# T4a审核修正：物理制动与调度诊断、停车证据期限

2026-11-13，待负责人亲审。仅修改`motion/constraint_worker.py`、`tests/unit/test_constraint_scheduling.py`，新增本报告；T4仍未提交，index为空，未push、未运行Gazebo/硬件/服务。constraints数值/几何逻辑、contracts与风险模型未改。

## 复现与修复

1. 危险停车包络`current=.4, hits=((.5,0),)`由engine返回EMERGENCY/stop_envelope_blocked；原scheduler的submit/Future异常及超预算/旧fence处理会另造CONTROLLED制动，覆盖刚检查到的危险。现在worker_diagnostic只保存最后一个计算状态字符串，不改物理结果、不积压；物理结果仍由最新engine快检查给出。纯函数`_merge_refinement`让最新EMERGENCY或未经验证的停车请求优先于计算放宽，合法permit则可接纳，计算失败不清有效permit。诊断在成功接纳新permit时清除；没有新增急停锁存或恢复owner。

2. 原零目标分支关闭_active，随后poll返回永久缓存，30s后仍stop_verified。现在inactive poll仍对最新原快照进行零目标refine；该路径只执行一次有界停车检查，不进入候选搜索或新worker。只替换target.velocity，保留motion/perception/response/risk及target全部原期限，不重盖时间戳、不维护第二个证明deadline。cancel立即撤销运动授权和旧正向停车证明，保留已知紧急制动级别；后续poll重新证明条件停车，过期变为stop_verified=false。已完成的旧flight在inactive poll丢弃，运行中的flight仍占唯一槽位；新请求可在旧结果丢弃后推进。

调度停止不等于车体已静止。零目标/取消后仍可能因前进惯性碰撞而返回EMERGENCY；证据失效后只能输出未验证受控制动与原因，不能宣称输出0已安全。正常降速、紧急制动请求与用户ESTOP继续分开；T5必须保留这三者的语义。

## RED/GREEN与边界

证据目录E：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t4a-review2`。初次新增测试RED日志实际16失败；收窄最终测试时移除了直接改私有fence的有效permit样例，旧fence以实际config改变触发的危险轨迹测试保留。重新请求测试最初使用30s但fixture响应固定20s到期，按真实失效原因改用3s（超过原motion/perception2s期限、仍在原response/risk20s期限内），未放宽产品时效。额外保留零目标/取消且仍+.4m/s的危险停车测试。

最终新增16项，原314项全部保留：**330 passed**。scheduler共28项；新增覆盖：submit异常、Future异常、运行超预算、完成后超预算、实际旧fence、Future取消均不降级危险停车；submit/Future/运行及完成超预算不擦有效permit；零目标后仅poll分别越过motion/perception/response/risk原期限；cancel不接纳迟到结果且新请求恢复推进；零目标及cancel的真实当前惯性。已有持续10Hz/慢worker、危险立即收紧、原deadline不可续等继续通过。

```sh
PYTHONPATH=.:src .venv/bin/pytest -q tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py tests/unit/test_motion_fusion_replay.py tests/unit/test_geometry.py tests/unit/test_guard.py tests/unit/test_guard_hit_intersections.py tests/unit/test_async_guard.py tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py
MYPYPATH=src:. .venv/bin/mypy --strict --explicit-package-bases -p senior_control.motion -m tools.motion_fusion_replay -m tools.motion_replay_records -m tools.motion_fusion_synthetic
.venv/bin/ruff check src/senior_control/motion/{contracts,constraints,constraint_worker}.py tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py
```

严格Mypy18源文件、Ruff、git diff --check通过。原始red.log、green.log、types.log、ruff.log、仅本轮小diff与源码前后SHA保存在E。旧测试关于worker错误作为brake.reason的断言已定向迁移为worker_diagnostic，物理reason单独保留；未改失败标准掩盖危险。

## 职责拆分建议

本轮不移动已审数值逻辑，避免两项调度修复与大段搬迁混合，constraints.py仍749行。其职责可分为：观测预处理/不可变模型；纯S曲线及可达几何；候选与许可准入。后续独立机械拆分可将StopPhase/AxisStop/stop_axis、角度极值、坐标变换、可达域纯函数移到`constraint_geometry.py`，仅接受顶点/角域/距离及显式预算标量，不反向依赖ConstraintEngine/RiskEnvelope，不引入策略工厂；constraints保留原公开导入以维持API，原解析/几何测试不变。观测与资格仍留当前入口，避免过度分层；这次没有改变公式、采样步长或评分阈值。

所有synthetic模型、位置/侧滑/动态障碍假设、真实CI仍0、激光移动精度与Linux最坏调度成本未验证的限制沿用T4a assumptions页。逻辑测试通过不代表实物安全包络通过，本阶段没有生产运动输出资格。
