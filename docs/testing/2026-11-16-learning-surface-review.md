# 固定学习表面亲审返修与core可用性限制

任务 `task_dfab27221feb / ctx_977083177671`；分支fix/degraded-drive-speed，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。本轮明确bug及调查已交付待亲审；**不宣称非空学习的完整core运动周期通过**。未改停车风险、误差或未知空间规则，未启服务/ROS/仿真/车辆，未暂存、提交、推送。

## 三处明确bug：先失败后修复

本轮开始快照 `/tmp/frozen-learning-review-start` 含792份文件及sha256.json，保留前轮返修和其它工作区改动。初始定向测试4失败、2通过（`learning-surface-before.log/.xml`），对应下列实际问题；旧失败证据保留在本报告的[evidence目录](2026-11-16-learning-surface-review-evidence)。

| 问题 | 根因 | 本轮修复与直接证据 |
| --- | --- | --- |
| 密集长表面过滤空洞 | validate用stable[::stride]把1000个稳定端点压成125点，原点间距已超过1cm，未选点变成hit | 取消抽样，保存全部稳定参考点；上限与原扫描预算统一为2048。1000及2048束真实极坐标的约5m连续表面，所有原端点在重复运行及保存恢复后仍是self，参考点逐个相同 |
| 360°首尾断连 | connected_points只切线性run，未处理采样周期接缝 | 使用实际扫描束数/角步长识别完整一周，合并首尾有效run；周期及重复端点两种真实角度/距离fixture通过，局部扫描及无效束仍断连。仅数值ulp处理舍入，不增加连接距离/角阈值 |
| 不稳定桥剔除后外侧点仍保留 | 旧实现只检查原有效命中run的车内种子，独立剔除不稳定坐标后不再检查连接 | 在同帧剩余稳定支持上重新分段，迭代至候选固定；中间桥失稳后外侧断开部分删除，有另一条稳定车内连接的部分保留。原错误“外侧总保留”断言改为断连必须剔除，并新增另一路径正例 |

2048点规范记录可以超过原64KiB，故同步调整记录容量为256KiB、Complete和anchor id上限为2048、客户端接收上限使用统一常量；没有只扩模型却截断IPC的隐性失败。强断言包括2048点记录实际超过65536字节、真实owner保存恢复、parse_record与Complete JSON往返。原“超记录容量”负例改为MAX_RECORD_BYTES+1，保留实际大小约束，不再用失效的旧常量假覆盖。

固定目录的运行匹配使用缓存STRtree查找最近参考点，仍按该参考点原硬编码1cm距离判断；不把点连线变成新mask，不扩大过滤区域，不添加新的控制或模型资格门。学习候选也使用同样位置支持加速连接复核。完成模型仍只读，无运行撤销/持久化回写；旧已保存抽样目录不自动补点或替换，新学习结果才包含全目录。

## 原三个失败core探针的根因

完整原始失败报告 `frozen-learning-extra-raw-gap-probe.log/.xml` 保留。重新执行原输入：rear index0=0.15，index1及末束NaN，其余ranges=10；仅添加只读诊断，不修改输出/permit/风险。复现命令如下，退出1表示保留的“预期completed”诊断断言失败，不是通过：

```sh
PYTHONPATH=tests/unit .venv/bin/pytest -c pyproject.toml -q -s /tmp/test_review_gap_probe.py -k delay_pattern0 --maxfail=1
```

诊断源完整保留为 `evidence/original-gap-diagnostic.py.txt`；重新运行时复制到 `/tmp/test_review_gap_probe.py`，在仓库根目录执行上述命令。`learning-surface-exact-gap.log` 给出实际链：

`ConstraintEngine._stop_certificate` → 原 `_sweep(stop_only=True)` → `_clear`检查车体外新增扫掠是否被free_space覆盖 → `sweep_unknown` → stop_verified=False → 不发drive permit → 最终零输出 → 不预消耗动作。每个控制周期都会重新检查，不是学习记录会话、动作防重放或算法不返回的死循环；处于moving表示等待实际首次输出，3秒计时尚未开始。

原双侧缺束输入的padded+physical未覆盖扫掠面积为0.006171487959207388 m²，bounds=(-0.4476008504370064,-0.031070626477677436,-0.337,0.02715353786169763)。这部分实际在物理车体之外，不能因车内回波被标self就伪称自由空间。

按负责人要求补原非空回波的最小对照，每组20个真实core控制周期，均断言反复sweep_unknown、全零输出、动作未消费，未恢复空输入当作解决：

| 相邻扫描 | physical未知扫掠面积 m² | physical未知bounds |
| --- | --- | --- |
| 相邻有效原值不变 | 0.004844128980066649 | (-0.43005240222245733,-0.02684556532947094,-0.337,0.02684556532947113) |
| 仅index1缺束 | 0.0051564948672433684 | (-0.43005240222245733,-0.030609227188718776,-0.337,0.02684556532947113) |
| 仅末束缺束 | 0.004844128980066649 | 同无缺束 |
| 双侧缺束 | 0.0051564948672433684 | 同index1缺束 |

这里的角度、距离及NaN均由现有扫描接口接纳，失败不是无效消息或字段不合法；同角度重复端点的包容式扫描数据也没有在本轮擅自“修好”来改变原输入。即使不叠加缺束，self回波后的未知区域仍与停车支持相交，因此不能把根因只归于额外NaN。

原linear/yaw支持均为(-0.1,0.1)。来源明确是 `SimulationMotionProfile.linear_error/yaw_error=0.1` 与 `StoppingRisk._axis_domain` 的 `max(配置误差+prediction_error, axis.error)`；这些是现有明确标注unvalidated simulation的风险假设，不是物理实测界或学习资格门。wheel/IMU静止证据由MotionSensorRuntime单独提供，并不按当前风险规则自动把上述误差域归零。此处没有证实可在不改变风险政策下修复的实现错误，故未减误差、未假造自由区、未改stop证书或学习等待策略。

结论仅限定在已测输入和当前laser_only路径：**该非空完整运动学习场景仍未成功，存在尚未解决的可用性限制**；不外推所有非空传感器布局不可用，也不将合法未知观测等同已确认安全。协调者已将此缺口记入根交接，要求本轮保持原政策。原空模型完整周期、非空模型局部学习/保存/恢复、实际core只读过滤分别通过，不能拼成“非空完整core通过”。

## 集中验证与范围

产品修复后集中组 `/tmp/review-977083-final.log/.xml`：**147 passed、1 skipped，59.23秒**。

```sh
.venv/bin/pytest -q tests/unit/test_learning_surface_review.py tests/unit/test_learning_core_surface_review.py tests/unit/test_learning_frozen_reference.py tests/unit/test_learning_record_owner.py tests/unit/test_learning_client.py tests/unit/test_learning_service_linux.py tests/unit/test_learning_dual_clock.py tests/unit/test_startup_learning.py tests/unit/test_startup_learning_timed.py tests/unit/test_startup_learning_review.py tests/unit/test_body_return_geometry_only.py tests/unit/test_startup_learning_chain.py tests/unit/test_startup_learning_cycle.py tests/unit/test_startup_learning_latch.py tests/unit/test_learning_cleanup.py tests/unit/test_motion_core_handoff.py tests/unit/test_motion_core_chain.py --junitxml=/tmp/review-977083-final.xml
```

随后仅补强测试：改为真实360°极坐标及一致重复端点、补记录容量强断言，产品未再修改；定向 `test_learning_surface_review.py test_learning_record_owner.py` 为15通过（2.66秒），日志 `review-977083-physical-final.log/.xml`。这些是有原因定向补测，不伪称147项整套再次执行。原4失败负例已变成明确机制正例，原非空core场景则保留为安全拒绝/可用性未解决的负例。

严格mypy检查4个本轮产品文件通过；Ruff检查8个修改源码/测试通过，format --check为8 files already formatted。日志分别为 `review-977083-mypy.log`、`review-977083-ruff.log`、`review-977083-format.log`，均已复制到evidence目录。无ignore、错误预算放宽或断言弱化。

唯一跳过仍为目标Linux abstract SOCK_SEQPACKET/SO_PEERCRED+spawn文件I/O端到端测试。当前macOS/CPython3.12软件证据不代替Linux/aarch64/Python3.10运行、ROS、仿真、实车或硬实时上界，新的较大IPC载荷在目标Linux也仍待验证。

## 精确增量与保护

本轮实际增量相对 `/tmp/frozen-learning-review-start`，见 `2026-11-16-learning-surface-review-owned.diff`；逐文件启动/最终SHA及证据文件清单见 `2026-11-16-learning-surface-review-fingerprints.json`。旧固定参考报告、旧指纹与旧增量作为历史证据保留，不回写为本轮验收通过。原临时 `test_learning_gap_debug.py` 是任务前已有诊断文件，本轮未改动，不将其旧completed断言计入当前回归通过。

三保护文件与本轮快照字节一致；motion_core_output.py、stopping_input.py、simulation_motion_profile.py、motion_core_perception.py也均与本轮快照相同。共享工作区的test_learning_gap_debug.py仅作为诊断输入读取，本worker未编写或修改；它不在本轮启动快照内，不纳入本worker增量或“相对启动未变”校验。其它任务工作区改动保留。文件写owner、动作0.05m/s host3秒、防重放与原清理未改；未新增学习接管、等待超时、感知或安全放行条件。
