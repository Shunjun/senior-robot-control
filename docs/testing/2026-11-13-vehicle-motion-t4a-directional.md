# T4a：纯方向约束与有界异步调度（待审）

2026-11-13。当前交付可执行纯链，不是车辆许可、统计模型验收或5 km/h闭环通过。未启动Gazebo、硬件、ROS或开发服务，未改变matcher评分、旧safety输出、manual authority或任务状态。

## 提交与文件边界

已审T3b独立提交 `6e4adbe7db20e53f18a24564b91c88b15a872441`，恰为任务指定14文件；提交前后index为空，详见证据目录`t3b-commit.json`。T4a保持未提交：新增`motion/{constraints,constraint_worker}.py`、`tests/unit/test_{directional_constraints,constraint_scheduling}.py`，修改`motion/contracts.py`和总计划，新增本报告与assumptions。公共geometry只读复用，其他既有脏文件保持。

`BodyMotion.average`现在允许None，避免T3b只有轮/IMU分量时捏造vy；检索motion消费点只有source_modes构造，没有运行输出消费。既有构造仍兼容，新增BrakeRequest/MotionPermit均不可变。独立风险模型不是第二套ResponseBounds或时效owner：ResponseBounds仍持有响应系数，RiskEnvelope提供其适用模式/场地/当前状态范围、独立误差及有限有效期。没有证据时返回未验证制动请求，不以零值假装车体已停。

## 可运行接口

`prepare_perception`有限预处理凸free patches与障碍点，显式处理no-return；保留原始stamp/deadline、frame/epoch及预测有效域。`ConstraintEngine.refine(snapshot,now)`对固定比例候选生成反应+jerk加减速停车可达包络；`tighten(previous,snapshot,now)`仅保留旧离散候选，重基准到最新pose并检查最新危险。`admit`复用快检查。放宽只在worker做，快路径最多固定候选的解析粗包络，不搜索无限方向。

`ConstraintScheduler.update/poll/stop`是唯一调度状态owner，最多一个in-flight和一个latest pending。旧结果按最新几何/配置/来源/期限检查，普通10Hz scan或意图更新不按generation无条件清零；来源/配置变化及stop fence迟到结果。重复观察不延长原许可deadline；超预算、Future错误和submit错误不抛出控制tick，也不覆盖仍有效许可，不启动可靠积压。worker仅返回值，调用方拥有executor生命周期。

`MotionPermit.allows`只认精确离散速度。T5较小ramp输出须显式调用`validate_output`做新的有界几何证明，并自行满足jerk/加速度/反向状态及PI模型；几何验证不证明执行器可实现性。普通障碍hold/制动和紧急制动请求不等于ESTOP，不清authority、不取消任务。

完整消费例在证据目录`demo.py`，使用明确synthetic测试plant：

```sh
PYTHONPATH=.:src:tests/unit .venv/bin/python "$E/demo.py"
```

`demo.json`实跑：free前进许可[.6,.3,.15,.075]；前堵且后方free可退[-.3,-.15,-.075,-.0375]；后方unknown无倒车许可；仍+.4前进请求后退返回reverse_requires_stop；零目标但停车包络碰撞返回emergency_brake_request/stop_verified=false；缺独立模型返回missing_risk_envelope/false。另有真实T3b接口消费测试：ReplayChain的合成wheel+IMU→masked FusionSnapshot→average/upper均None的BodyMotion→独立已知plant风险域→候选与.008轨迹步证明，不把轮速当真实速度。

## RED/GREEN与验证

证据根目录E：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t4a-directional`。red*.log保存模块缺失及有意义失败：残余加速度、航向误差平移、模型时域、身份往返fence、超预算、旧scan、期限收短、离散外ramp、padding盲区、worker异常。新37项覆盖正常负对照与要求中的方向/惯性/停车/失效/异步条件；解析S曲线三角和平台积分绝对容差1e-12，只验证数学。没有用单位测试替代物理界冻结。

最终结果：**314 passed** = 原222基线 + 新37项 + geometry/guard相关55项。严格Mypy18个源文件通过、Ruff通过。最终命令及原始输出保存在regression.log、types.log、ruff.log：

```sh
PYTHONPATH=.:src .venv/bin/pytest -q tests/unit/test_motion_sources.py tests/unit/test_motion_contracts.py tests/unit/test_manual_session.py tests/unit/test_manual_authority.py tests/unit/test_motion_estimator.py tests/unit/test_scan_matcher_replay.py tests/unit/test_motion_temporal.py tests/unit/test_motion_fusion.py tests/unit/test_motion_fusion_replay.py tests/unit/test_geometry.py tests/unit/test_guard.py tests/unit/test_guard_hit_intersections.py tests/unit/test_async_guard.py tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py
MYPYPATH=src:. .venv/bin/mypy --strict --explicit-package-bases -p senior_control.motion -m tools.motion_fusion_replay -m tools.motion_replay_records -m tools.motion_fusion_synthetic
.venv/bin/ruff check src/senior_control/motion/{contracts,constraints,constraint_worker}.py tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py
```

## 自审、限制与T5接点

只读复用swept_footprint，有限角度步数；矩形车体长宽各乘1.05一次，传感误差在观测预处理一次，yaw误差进入整个平移方向域；分别验证physical新扫掠和padded新扫掠，旧padding不等于已占据的车体，不能吞掉传感盲条。停车按当前惯性而非目标符号；响应/风险/感知预测必须覆盖完整停车时域。横向漂移没有独立settling保证则不宣称停车已证明。

可达包络是保守旋转与平移Minkowski包络，可能拒绝实际可通狭角；不是精确最优弧线规划。快路径算法有输入/候选/角度规模界，Shapely/线程调度的Linux最坏耗时未测。合成响应/场地/障碍静态预测均未证实，真实CI仍0、激光移动估计质量问题与T1未冻结保持记录；没有用其当前值证明停车。

下一T5可实施轨迹/PI/状态机纯核心并按既定审查接线：保留原raw观测时效、self-filter/depth与unknown；提供经独立验证的仿真profile和状态相关RiskEnvelope；每tick更新轨迹加速度/jerk、换向先停、抗饱和与实际静止判据；对每次实际输出做validate_output；按制动原因持续处理，不把等待worker当急停；最后连接唯一车辆出口并有界测量真实停车和调度。详见一页`2026-11-13-vehicle-motion-t4a-assumptions.md`。T4a是否通过仍由负责人亲审实际diff，T4生产资格及T1冻结未完成。
