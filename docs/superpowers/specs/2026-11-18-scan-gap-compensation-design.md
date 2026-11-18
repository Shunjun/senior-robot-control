# 接收端短时雷达缺失补偿设计

状态：用户已批准行为契约；本文将源码接线与待负责人审阅的数值建议分开。当前仅设计，不代表实现或实车验收。

## 目标与边界

只补偿之前真实有回波、当前相邻真实回波支持同一局部连续表面的 `+Inf` 方向。当前真实点拟合直线，当前射线与该直线相交给出预测距离，不平均历史距离。真实回波立即替换；连续最多三个新原始采集帧，第四帧仍缺失则清除。预测不成为真实种子，不重置期限。重复快照不计轮次。

预测只给行驶避障，继续使用现有 `directional_clearance/limit_velocity`、配置尺寸乘 1.10、单次 sensor_error 和含 lag 的制动公式。启动学习、学习遮挡比例、雷达测速、原始消息均只用真实扫描。剩余 `+Inf` 沿用无障碍点行为，不新增缺 scan 门槛；不扩大外框、不延长 TTL、不改 stamp/deadline、source/authority、ROS 消息或频率。

前置提交独立保留：`6c37b2a` 修学习撤回终态，`e3adb0f` 补对应回归，`4bde9a6` 只改启动遮挡中 +Inf 的角度可用语义。补点不得混入前置问题。

## 当前源码与数据通路

- `safety_node.py:434` 的 `_on_scan` 已进行原始身份、代次、递增序号、消息时效与有限尺寸接纳，最后生成 `RawSafetyObservation`。`_clear_scan` 是现有失效/换源统一清理入口。
- `_scans` 目前是 `deque(maxlen=8)`，`_publish_inputs` 仅传 `tuple(_scans)[-1:]`；`ControlInputs.update` 清除旧扫描再放最新扫描。`directional_clearance` 只读最后一帧，学习也只读最后一帧。因此八帧缓存没有控制消费价值。
- `LatestSnapshot` 已用 RawArray，共享槽仍有 pickle.dumps/loads 与锁内缓冲复制；本任务沿用，不重构共享内存，不传历史图或估计器。
- 接收进程 `_input_bridge.core.poses` 已有最多 64 条标准 Odometry 平面位姿：x/y/yaw、epoch、sequence、原 SampleTime、received 时间。只有原接纳成功且 frame/四元数正确才追加。该数据是坐标关联参考，不是独立定位资格；无需新增定位订阅/融合/门槛。
- `_learning_observation` 读取 `raw.message`，真实观测交给 StartupLearning/SelfReturnModel。`scan_adapter` 与测速接线位于其上游，不应看到本接收端的预测。

## 职责和文件范围

| 类别 | 文件 | 责任 |
|---|---|---|
| 新产品 1 | `ros2/src/senior_robot/senior_robot/scan_gap_compensation.py` | 有界真实种子、位姿对齐、局部拟合、缺失预算；无 ROS I/O/输出权限 |
| 改产品 1 | `ros2/src/senior_robot/senior_robot/raw_safety_observation.py` | 当前原始扫描附稀疏预测及来源，不覆盖 message |
| 改产品 2 | `ros2/src/senior_robot/senior_robot/safety_node.py` | 接收端唯一补偿实例，接纳后处理、清理，八帧缓存改为一帧 |
| 改产品 3 | `ros2/src/senior_robot/senior_robot/motion_core_perception.py` | 真实/预测当前避障观测构造，沿用前后条带与边界 |
| 改产品 4 | `ros2/src/senior_robot/senior_robot/motion_core_output.py` | 真实观测继续学习；另对避障派生观测复用已有自体过滤 |
| 新测试 1 | `tests/unit/test_scan_gap_compensation.py` | 有界补点与原始身份直接回归，不新建公共夹具 |
| 可改测试 1 | `tests/unit/test_motion_core_perception.py` | 实际方向净距/限速消费与原始学习观测隔离 |
| 新文档 2 | 本设计及同名计划 | 行为、接线、执行与验收 |

产品合计新增 1、修改 4；测试新增 1、最多修改 1；文档新增 2；总计 8～9 文件，删除文件 0，配置/构建/工具/依赖变更 0。`ControlInputs`、`MotionSensorRuntime`、`scan_adapter`、学习和测速算法无需改。单一路径顺序实施，不派并行 helper。

## 冻结接口建议

在 `raw_safety_observation.py` 放一个小 frozen 数据项 `PredictedReturn`，字段为 `index`、`distance`、`missing_frames`、`seed_capture`；`seed_capture` 直接复用既有不可变 RawCaptureHeader，保留真实种子的原始来源/时间。`RawSafetyObservation` 新增默认空元组 `predictions: tuple[PredictedReturn, ...] = ()`，其 message 和五个现有接收/测量字段原样保留。

元数据解释：message 中合法有限量程回波为 real，其缺失轮数 0、来源是本帧 raw_capture；predictions 元组中的项目为 predicted，缺失轮数 1～3、真实种子来源是 seed_capture、当前消费帧来源仍是 message.raw_capture。不为每条真实射线复制来源。没有预测时不复制 ranges；共享快照只多当前稀疏结果，不带种子池、拟合树或位姿历史。

`ScanGapCompensator(profile)` 只保存既有 physical length/width 与 sensor_error；`update(raw, poses) -> RawSafetyObservation`；`clear() -> None`。每个通过接纳的新 raw 在 `_on_scan` 同步处理一次；相同原采集身份直接返回相同结果，不改计数。身份使用 raw_capture 的 bridge_boot_id/source_epoch/capture_generation/raw_gid/sequence，并保留原 SampleTime，不用 relay 序号或 `_scan_generation` 冒充新物理采集。换源、时钟断代或 `_clear_scan` 一并清补点状态。

`scan_observation` 保留原始语义。新增一个同模块的当前避障观测构造函数，只有 predictions 非空才复制 ranges 并放入预测距离，返回局部不可变 ScanObservation/FilteredObservation；绝不赋回 raw.message。core 先完成原始学习处理，再对派生避障观测调用同一个已完成 LearningFilterView 或现有 SelfReturnModel.filter(provisional=...)，只 filter、不 frame/observe/validate。最终 directional_clearance 使用这份避障观测，避免预测自体点绕过既有 1cm 已学过滤。无预测时复用当前 filtered 路径，不能无条件每 tick 多做一次过滤。

实现应让 directional_clearance 的射线枚举与 filtered 指向同一份 ranges/索引，不能从原始 ranges 读 +Inf 却拿派生过滤标签。新可选输入只表达当前观测，不提供第二套限速算法。

## 坐标关联与生命周期

1. 从既有 poses 中，以 scan 的原始 ROS stamp_ns 找同一 odom epoch 的左右包围样本；检查各自原时间有效、stamp 单调、区间属于连续时钟窗口。跨源 clock_generation 是各 owner 局部编号，不能拿数字相等冒充共享时钟身份。扫描换代清本地历史，odom epoch 改变也清历史。
2. 在两个真实位姿之间线性插值 x/y，yaw 用最短角差插值；只插值，不按目标速度、轮速或独立估计外推位姿，不用最新处理时刻替换测量时刻。没有包围样本时本帧原样发布且跳过补偿；不等待未来样本、不阻断扫描，不做运动资格判定。
3. 真实种子保存当帧真实 body 端点、当时 odom 位姿和采集身份，按 `T_body_now_from_odom * T_odom_from_body_seed` 投到当前车体，再反变换到当前 sensor。不是原角索引直接复用旧距离。
4. 当前角栅格改变时按投影后方位重新关联当前射线单元；无对应 FOV、两个不同表面落同单元、关联不唯一或换安装 TF 时丢弃该预测，不能沿用旧 index。负角增量和首尾环绕按实际 FOV 处理，未覆盖部分不补。
5. 缺失预算挂在真实种子身份上。每一新采集帧更新一次，格点改变不重置轮数；即使本轮因支持不足未产生预测，也消耗缺失预算，第四轮删除。后续真回波成为新的真实种子；预测绝不回写种子。
6. 每帧状态仅当前真实种子及最多三轮未过期种子，数量至多四倍接纳上限 2048；每个候选只看常数大小邻域，历史不过共享槽。旧 poses 只是借用现有本地有界记录，不新建另一份位姿历史。

位姿连续不等于环境同表面；下节当前真实几何必须同时支持。动态障碍/门打开与回波缺失无法单靠 Inf 完美区分，不能承诺识别所有门缝；严格短局部窗口、真实历史覆盖和三轮上限限定这类残留风险。

## 局部几何建议（供负责人审阅，未实现）

仅原始 +Inf 可以补；NaN、-Inf、超量程有限值、低于 range_min 不补。种子必须是曾真实合法量程回波；从来 +Inf 不造点，实际返回优先于任何历史。物理车体内部支持不用于外部表面拟合，最终仍按当前 learned/provisional 自体模型过滤。

建议常数大小窗口：一个缺口最多三个当前射线单元，两侧各至多三个连续真实点；内部拟合至少两侧各两个真实点，边缘拟合至少一侧三个真实点。缺口长于窗口不补；支持中遇异常、量程断层或已识别角点即截断，不跨越寻找更远的另一面。此处“最多三个射线单元”是空间窗口建议，与获批“最多三帧”时间期限不同，审阅时单独确认。

用已有 numpy 对当前真实支持点作二维正交最小二乘直线拟合（2x2 协方差的最小特征向量），求当前射线交点，不平均 ranges。建议拟合残差容限 `e = min(profile.sensor_error, s / 4)`，s 为支持集中相邻真实点间距的中位值；此值只用于接受/拒绝拟合，不膨胀坐标/边界或叠加雷达误差。所有支持点正交残差不超过 e；分母近零、退化点、交点不在真实量程内直接放弃。

连续性建议：先将上一真实小窗口按位姿转到当前坐标，要求其曾连续真实覆盖整个待补位置；支持相邻距离不能超过原连续段对应间距加 2e，且支持在同一拟合直线上。内部交点必须在当前两侧支持投影区间内；边缘交点只许沿单侧拟合方向延伸，延伸距离同时不超过该侧真实支持段长度、且位于投影后的历史真实覆盖区间内。已有历史空档/断层/角点不连接。当前有限真实点落在另一表面时立即取消旧候选，不拿预测当连接支撑。

这些建议的误差/窗口要用已录真实片段核对误补和支持率；如果不能同时满足不跨门缝/角点与可用性，先少补，不扩大范围，不自动调阈值，不改控制安全边界。需要改变已列条件时回报具体原因，不能作为开发者暗藏调参。

## 性能证据与证明范围

当前不跑 benchmark，不声称交换是瓶颈，不预言 30ms 控制周期。历史唯一在接收端，按新原始雷达帧执行；50ms重复发布/重复tick不重拟合、不增长轮数。期望复杂度 O(N) 加常数邻域小矩阵，须实测证明。

负责人后续只读获取源：`/home/wheeltec/ros2_shared_dir/senior-control/logs/obstruction-learning-70-1794974809034`，以及负责人已有修复前后控制 bag。保留原 raw capture、odom 位姿、TF、消息实际顺序及时间，worker不访问车辆。

同一硬件/解释器、同一输入同一帧对比旧/新 median/p95/max：接收处理（含位姿关联和拟合）、pickle 编码/解码、payload bytes、入接收至快照发布延迟。区分算法 CPU、50ms发布时隙等待、锁竞争跳过和调度等待；现有 input_update/guard/tick及callback间隔5字段复用，不能用RuntimeReports重复频率冒充控制tick。不增加新benchmark脚本或常驻诊断；若现有记录缺入口/发布时间，实施前单列临时最小时间字段和范围。

输入必须含：全有回波、内部短缺口、边缘缺口、从无回波、连续四帧缺失、重复raw、运动/角栅格改变、断层/角点/门缝。无预测帧的净距/限速与旧结果逐项相同；有预测帧只在元数据列出的缺失方向变化。学习/测速原始输入、stamp/deadline和身份保持相同；耗时结果标本机回放或车端实测，不能互相冒充。

## 真实验收由负责人串行执行

产品亲审、耗时核对、部署后，负责人先被动读雷达和停车状态，再按最新授权通过项目 CLI 重启一次真实学习；worker不SSH/启服务/运动。学习必须仍吃原始扫描。报告真实运动/停车、phase/reason、action consumed/stage/首提交/截止/累计输出、模型真实点数及车体坐标分布。候选点不是障碍物数量；completed但0点不能宣称车内/车上附件识别成功。若未完成，则明确真实终态与尚未得到模型，不盲目再次启动。
