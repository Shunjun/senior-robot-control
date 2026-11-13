# T1 matcher 结构化诊断与离线根因（2026-11-13）

状态：本轮 A/B 完成；没有证据足以认定局部配准实现 bug，C 不实施算法修复。已交付实际诊断与回放代码，不降低阈值、不替换估计器、不覆盖失败，不启动 Gazebo/manual/navigation/高档或硬件。T1 整体仍未冻结，不称 5 km/h 完成。

## 文件边界与诊断契约

仅修改 `ros2/src/senior_scan_matcher` 的 header/matcher/node/CMake/package 与定向 tests，新增 `tools/scan_matcher_replay.py`、其 unit tests、本报告，并在 spec/plan 定向同步安装/IMU事实与进度。其他 T2/core、校准工具和旧 ROS 测试均未修改，未提交/推送。

`MatchDiagnostics` 使用 optional：几何、匹配和 relative covariance 各阶段未计算时明确为空；独立 `/motion/scan_diagnostics` 输出 DiagnosticArray，数值用 17 位精度，缺值 `not_computed`，合法零仍为 `0`，非有限真实计算值保留 `nan/inf` 表达，不制造零。初始化由原 reason 表明，不冒充失败计算。数据包括有效束数、spread、normal windows、translation/determinant/yaw information，response、covariance、residual、inlier count/total/fraction、最终 anchor 和各失败子条件；relative covariance 单独保留。原有 ScanMotion 协议与其消费者不变，兼容新增主题处标 `//Compatible`。诊断为 best-effort/keep-last-1，不新增可靠积压通道。

`qualification_failures` 为唯一纯资格合取，保留原 inclusive response/residual/inlier/covariance 和严格 `.19` 搜索边界；relative covariance 也沿用原三个对角条件。诊断收集与原 valid/reason、clear/reset/new_epoch 分支分开：无效路径仍原样重置；只有此前发布过 qualified 的连续性断裂才旋转 producer epoch。粗后细流程保持：fine 平移 `.02 m`、yaw `.005 rad`，未增加重复 fine。

## 构建、回放与可复现证据

宿主 `$E`：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t1-matcher-diagnostics`。
VM `$R`：`/home/toby/senior-audit-20261031/matcher-diagnostics-20261113-r1`。

- 独占原 `.vm-test.lock`，新 baseline-source/build 与 diagnostic/final/final3-source/build；不覆盖原 r3。旧版与新版均使用相同 `/opt/ros/humble`、r3 消息依赖、GCC 11.4/Karto。首次独立 CMake 未找到 Karto，后绑定实际唯一现有库 `/opt/ros/humble/lib/libkartoSlamToolbox.so`；未安装/替换依赖。
- Karto 二进制 SHA `f70d96ea68439510b8cf4b4ba2b2a1b94479aa9eda1abc81cb41fc1f64bc66be`，Mapper.h SHA `22fa817360fc46284bdfc4239c4fcb723c081ce9f0b9e3e88c75a7a97bdd2b40`，ldd/编译器/包版本与源清单保存在 identity/runtime 文件。
- 原始样本路径：相对 `$E`，新地标为 `../t1-observable-corridor-r2/vm-low_forward-{1,2}/samples.jsonl.gz`；固定旧回归为 `../t1-expanded-calibration/vm-turn-3/samples.jsonl.gz` 和 `../t1-expanded-calibration/vm-low_forward-5/samples.jsonl.gz`。对应原生 bag 在各目录的 `bag/`；准备 manifest 记录每份原始样本和实际 profile/input SHA，历史目录未修改。
- replay 是无 ROS 的 stdin C++ 程序，不包含 truth/wheel/IMU，也无设备/发布接口。Python 准备器只把原 bag filtered beams、原顺序/stamp、来源改变/上游 invalid 对应的 reset 输入 matcher；manifest 另存原整数 ns、source epoch/GID 和原始样本/配置 SHA。ROS rollback 不排序、不偷偷重置抹掉证据；评估侧才载入 truth。输入不重采样、不调窗。
- 配置来自对应获审 fixture（max_interval=.4、max_variance=.05 等），不是 matcher 默认 `.3/.1`；每个 input manifest 冻结全部参数。每个 replay 外层 timeout 90 s、进程组最终回收；CTests 上限 240 s，ROS 构造/退出单测限定本机域173；无长期服务。
- 新工具曾被 Karto 注册/注销日志污染 stdout：旧输出逐行 JSON 解析真实失败，新增 CLI 回归先对旧 executable 观察 JSONDecodeError，再把库日志路由 stderr、JSON 独立 stdout，最后 CTest 验证。此修复仅针对新增回放工具，不冒称修复产品测速。

复现主命令（Linux；先按 build driver 绑定已有依赖、取得原锁）：

```sh
source /opt/ros/humble/setup.bash
source /home/toby/senior-audit-20261031/motion-20261112-r3/install/setup.bash
cmake -S "$R/final3-source" -B "$R/final3-build" -DKARTO_LIBRARY=/opt/ros/humble/lib/libkartoSlamToolbox.so -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
cmake --build "$R/final3-build" -j2
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ctest --test-dir "$R/final3-build" --output-on-failure
timeout --signal=TERM --kill-after=3 90 "$R/final3-build/scan_matcher_replay" < "$R/replay-inputs/observable-1/input.txt" > /tmp/matcher-review.jsonl
```

新准备器：`.venv/bin/python -m tools.scan_matcher_replay --samples <closed-samples.jsonl.gz> --profile <scan-motion.yaml> --output <new-dir>`。输出目录必须不存在。实际驱动脚本、编译/RED/GREEN日志、输入 manifest、原始输出与每帧诊断 CSV 全在 `$E`；`finish3-vm.py` 是最终有界构建/回放/审核入口，路径存在时拒绝覆盖，不重复执行。

## 序列复现与失败比例

最终诊断版与旧 baseline 的每帧 valid/reason/start/sensor_delta **四组全部完全一致**。与实际录制 ScanMotion 对比：新地标两次及弱场全部一致；旧 turn-3 的 bag 从 3.9 s 开始，缺初始化前缀，回放首帧 initializing、原录制已 qualified。除此一帧，后续 306 帧 reason/valid 相同，body delta 最大浮点差约 1.08e-17；不伪造缺失热启动，也不以录制结果喂算法。DDS/TF 发现和回调 receipt 时限不属于这个 matcher 离线实验，不能据此声称完整节点接线验收。

| 数据 | 总帧 | qualified | initializing | geometry.normal_windows | response | covariance/residual/inlier/search/relative 拒绝 |
| --- | --- | --- | --- | --- | --- | --- |
| observable-1 | 382 | 240 | 34 | 75 | 33 | 0 |
| observable-2 | 381 | 238 | 34 | 76 | 33 | 0 |
| old-turn-3 | 307 | 306 | 1 | 0 | 0 | 0 |
| old-weak-5 | 338 | 0 | 0 | 338 | 0 | 0 |

分母为该数据全部帧。两次新地标几何拒绝 19.6335%/19.9475%，response 拒绝 8.6387%/8.6614%；在“已进入匹配且被拒绝”的 33/33 帧中，response 是 **100% 唯一失败项**，其余四个初次匹配条件（covariance/residual/inlier/search）均实际计算通过；relative covariance 尚未进入计算，明确 not_computed，表中0表示拒绝次数为0，不表示已经通过。旧弱场 100% 在几何阶段失败，匹配指标明确未计算；旧转弯没有拒绝项也不代表精度通过。

### 可直接确定的因果与尚未确定的机制

- 新地标起点 18 个 normal windows<25，几何拒绝先于相关计算；此前理想 raycast 不能替代实际 beam。几何开始通过后，repeat1 ROS 7.6、repeat2 7.7 首次 match_unqualified，response 分别 .234111/.235111，低于原 .3。
- 所有 33 帧 response 范围：repeat1 .224694–.289347，repeat2 .226292–.286931；residual .02604–.04728<.09，inlier fraction .95413–1≥.55。协方差三个对角最大约 .013814/.005546/.000163，均在原 .05 内，搜索边界和 relative covariance 无拒绝。精确逐帧值（包括 off-diagonal）见 CSV，缺值不以零加入统计。
- `LocalMatcher::update` 的 response 判定拒绝 → 原 `m.clear()` → node `invalidate(reason,true)` → 原 matcher.reset；随后一帧 initializing。因此初始化/失败交替有具体资格首因，不归咎异步饥饿。曾 qualified 后在 14.1/14.2 s 再失败才经 published_qualified_ 旋转 epoch；其余持续失败不每帧伪造新 epoch。上次 bag 的启动 raw_source_changed 与这些运动中 response 重置是两回事。
- 候选机制：无效/超量程束占比会影响相关得分。上游 Humble Karto GetResponse 跳过 INVALID_SCAN 的累加，却按 lookup 总大小归一；[上游实现](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/humble/lib/karto_sdk/src/Mapper.cpp) 支持这种覆盖耦合解释。现场被拒帧有效束仅 209–251/720，且 residual/inlier 良好，与此一致。但本机二进制到该上游 branch 的完整 commit 对应和 lookup 内部数目未直接导出，故这是有依据的机制解释，不伪称已证本机局部代码 bug。
- 不压缩/重排有效束（会改变原射线角索引），不把 raw response 重标定后直接沿用 .3，也不取消几何检查。这些会改变质量契约，不是本轮允许的保行为诊断。若后续要改评分/连续配准，需独立架构决定、明确质量/协方差含义与未见留出验证；本轮算法盲试次数为 0，无行为修复 patch。

### 同源区间误差与计算成本

评估使用原 `interval_start→end`，sensor delta 按已知 laser 外参 (.15699,.00201,0) 转 body，再用 SE(2) log 与 truth 同窗比较；不跨 invalid/epoch 拼接。新地标运动窗仍只有 3/78 合格，最大 vx 误差 .140160/.143160 m/s；旧 turn-3 49 个可评估运动窗最大 vx .288458 m/s、yaw .088315 rad/s。弱场无合格窗，误差为空，不以 0 报告。已有 fine 后仍有这些误差；不能把粗步长 .02 rad 当最终分辨率，也不能将转弯 wheel .5 vs body/IMU≈.295 的物理差异说成轮速内环自行能消除的测量误差。

update 单次 steady 计时（不含 stdin、JSON/ROS diagnostics 序列化与发布；不是冻结 tick 预算）：

| 数据 | baseline p95 / max ms | 最终版 p95 / max ms |
| --- | --- | --- |
| observable-1 | 0.9114 / 1.8393 | 0.9071 / 1.4238 |
| observable-2 | 0.9488 / 1.5102 | 0.9214 / 1.2836 |
| old-turn-3 | 1.1492 / 3.7606 | 1.0176 / 1.4426 |
| old-weak-5 | 0.0078 / 0.0152 | 0.0074 / 0.0234 |

单次顺序回放有调度噪声，不能用快慢差声称优化或最坏耗时保证；原始每帧 seconds 保留。诊断可观测性改善已实证，产品测速准确性没有因此改善。

## IMU/安装事实与 T3 的可执行结论

用户确认激光已装，主要问题是无可识别场景特征；厂家 `wheeltec_robot.cpp:589–608` 解析串口三轴加速度/角速度并换算单位，:762 发布 `imu/data_raw`，这是已有能力证据，不再将 IMU 完全未知作为阻碍。实装坐标/零偏/比例/标定与时间戳仍待核验，IMU 积分不能长期保证无漂移或辨认恒定轮滑移。

已足以建议负责人派发 T3 多源质量/时效/协方差适配、IMU+轮速受限退化与恢复的纯模块及失败/正常负对照：边界与内部 cap 显式注入，unsupported 真速界不得伪造，GPS 可缺失；障碍感知健康才允许对应方向，缺感知受控停车。模块不接运动出口、不以扫描健康为唯一放行条件，不冻结未知 cap。生产接线、闭环与高档仍需要适用范围内的误差/响应/退化边界验证；本次不能替代那些测量。是否条件并行进入 T3 由负责人按该限定派发，非本轮自行越阶段。

## 未见 holdout 预登记建议（尚不执行）

当前 repeat2、旧 turn/弱场已看过，均为固定回归，不叫盲留出。不移动现有窗口凑指标。建议在任何评分/算法行为改动前由负责人将以下布局生成的完整 SDF、计划及数值判据冻结 SHA，原地面/摩擦/车辆/过滤模型保留：

- H1：20 m 半走廊，14 块 `.15×1.5×2 m` 静态板；k=0..6，x=-6+2k，y=±(4.5+.2(k mod 2))，yaw=±(.35+.1(k mod 3))。原点静止，固定低档 plan（2 s零、6 s +5/9、4 s零，a=.3、工作16/外限18 s），两次；这是新布局，不能沿用当前场地预测为合格。
- H2：12 块同尺寸板，θ=.17+kπ/6，r=5+.25(k mod 3)，位置 (r cosθ,r sinθ)，yaw=θ+.4；原点静止，固定 turn plan（2 s零、4 s +.25,+.5、3 s零，a=.3/yaw a=.5、工作13/外限15 s），两次。目标积分路径≤1 m，保留足够场地和外部停止；不是高档刺激。
- 两场先有限静止原生观测，不凭 DDS 匹配判就绪。若实际几何不可观测，保留为有效负例，不临时调板位/阈值重定义 holdout；需要新丰富场必须另编号并先登记。真实物理传感器、GPS/质量注入各自标明。
- 仪器保行为判据已明确：固定回归每帧 valid/reason/start/delta 不变，记录前缀缺失另列；所有输入/输出计数可追溯，缺流不判通过。算法精度判据 E_v/E_w、覆盖率、恢复时间/退化 cap 仍未冻结，须在行为实现和上述 holdout 采样前由负责人从开发集及独立响应测量冻结数值与来源，不能读 holdout 后调标准。弱特征要求正确退化，不要求虚假扫描合格；无方向感知必须受控停车。

## 验证与审核材料

本轮接口缺失先由 clang 编译证明；missing/真实零诊断增加前，本地检测程序退出42，补字段后退出0；stdout 回归实际 RED/GREEN 保存在 replay-contract-red.log 与最终 CTest 中。C++ 定向测试包含精确边界、多项并发失败、NaN/零/越界协方差、relative covariance、正常与缺值对照，并保留13项 matcher和2项shutdown回归。最终结果、SHA、完整增量及清理状态见末尾清单。

最终核验：4 个 CTest target 全通过（21 个 C++ 用例 + 1 CLI stdout contract）；本地 `.venv/bin/python -m pytest tests/unit/test_scan_matcher_replay.py -q` 2项通过，新增 Python 工具 Ruff/Mypy 通过。最终源与 VM `final3-source` 逐文件 SHA 一致，原 r3 source/install 564/1004 文件零差异；独占锁已重新取得，本次路径相关残留进程为零，详见 `final3-identity-cleanup.json`。每版构建/回放使用独立目录和输出，早期 Karto 查找失败及日志污染负例均保留。未见 holdout 尚未采集，不用旧数据充数。
