# 测试精简：负责人审阅与执行清单

基线：`5f59042dd9413cd50422b36c411ca9f2226c06f8`；分支：`test/prune-redundant-cases`。

用户要求负责人核对后明确告知开发删除哪些测试。本清单由负责人阅读候选测试实际正文后确定，开发仅执行 JSON 清单中的删除，不能自行扩大范围。此次只调整测试及其失去用途的专用辅助代码，不调整产品行为，不启动服务。

## 审阅范围和结论

已盘点 Python 的 217 个测试文件、1,921 个测试函数（未展开参数），以及前端测试与 ROS/C++ 测试目录。没有发现函数正文完全一致的 Python 测试；语义重复和异常场景需要结合正文判断。已逐项阅读本次删除项，并对照同文件或相关模块保留的正常路径。此次不是对全部保留函数的逐行重审，也不以名称或数量自动删除。

精确执行对象见 `2026-11-17-test-pruning-manifest.json`：删除 43 个 Python 测试函数，另删除只有 1 个测试函数的 `test_native_sensor_age_boundary.py`，删除 1 个前端测试及 1 个 C++ 测试。参数化展开和函数内循环场景数由执行报告另外列明，不能将函数数当成执行项数。

| 范围 | 删除理由 | 保留的主要覆盖 |
|---|---|---|
| independent_motion 的四个极值用例 | 1e308 / 10**1000 等不具实际传感器含义的数字 | 正常 PnP、运动变换、真实图像缺陷与来源替换 |
| independent_motion 的六个元数据类型用例 | 手工把 list、迭代器或错误标量塞入内部不可变对象，重复约束实现形态 | frame builder 输入快照、原始消息序列化及正常不可变副本 |
| independent_motion 的来源容量耗尽 | 连续制造 67 次新来源以命中内部退休表容量 | 正常来源重启、替换与旧数据不复活 |
| manual_session 的四个大整数用例 | 10**1000 与 10**308 的时钟、TTL、轴值 | 正常 TTL、原始接收时间、释放和一般输入越界 |
| motion_sources 的四个用例 | 极大协方差；向整数序号塞 NaN/list 等；混合重复非法内部结构 | 正常序号去重、来源健康、期限及数据质量拒绝 |
| cadence / manual_authority 的非有限时钟 | 把本机单调时钟注入 NaN/Inf | 实际调度超期、跳周期及正常权限生命周期 |
| rgbd_body_motion 的逆变换溢出 | 1.7e308 米平移 | 常规 SE2/SE3 变换和杆臂 |
| rear_geometry_equivalence 的三个用例 | 1e308、1e-16 与指定几何构造路径绑定；强制继续走旧 LineString 分支 | 实际后方场景几何等价与优化表现 |
| rear_fan_optimization 的随机组合用例 | 同时随机缺束、交换端点及改变射线原点，重复既有缺束场景等价比较 | 命名的 actual / masked / gap 等正常几何场景 |
| static_tf_binding / wire 的四个用例 | 对 ROS double/整数/固定内部结构注入字符串、list、bool | 真实 TF 组合、来源替换、图结构和生命周期 |
| protocol 的旁路构造用例 | 验证通过后改成 NaN，或用 model_construct 绕过验证 | 公共协议解析及非法外部输入检查 |
| websocket 的 NaN 服务时钟 | 特殊合成 NaN 回执 | 正常心跳、身份校验和五秒在线边界 |
| rgbd_spawn_worker 的六个用例 | 私有结果队列塞任意对象；十轮 SIGSTOP；线程启动精确失败；伪造线程未退出 | 一次实际超时/进程死亡、恢复、关闭及正常 IPC |
| rgbd_parent_ledger 的 source_epoch=[] | 将本应为字符串的内部标识改为列表 | 实际来源替换、重启和延迟结果 |
| rgbd_frame_builder 的 ROI 类型矩阵 | 对 uint32 ROS ROI 字段注入 bool、字符串、负数及溢出值 | 正常 numpy ROI、相机矩阵与真实图像布局 |
| directional_constraints 的极小减速度 | 1e-300 制动参数 | 实际制动距离、jerk、扫掠及障碍约束 |
| map_archive 的 syscall 间隙换符号链接 | 精确卡在 lstat/open 之间的恶意替换竞争 | 基本路径穿越/符号链接拒绝、正常地图导入导出 |
| Agent 的不响应取消任务 | 伪造主动吞掉每次 CancelledError 的永久任务 | 实际任务收束、正常关闭及进程故障隔离 |
| native_sensor_age_boundary 整文件 | 浮点最大值、285 年运行时长、uint64 计数饱和与 noexcept 实现约束混合矩阵 | shared_sensor_cpp 的正常 C++ 年龄策略、sensor_timing 的期限 |
| 前端 buffered socket 单测 | 与紧邻完整手动控制背压流程重复 | 保留 held input / drain / release 的完整流程 |
| C++ RetirementExhaustionRevokesActualProducerOnGraphLoss | 通过 TestPeer 强行填满退休表后叠加非法候选、图丢失 | producer 重启、真实发布者消失、正常所有者变更 |

正常传感器可能产生 NaN，因此没有笼统删除传感器无效值测试。保留正常断网、拥塞、重启、急停、障碍、任务生命周期、学习完整周期，以及现有真实功能链验证。前端功能和 C++ 感知算法未因目录不同而批量删除。测试删除不等于相关产品条件被撤销，未授权修改产品逻辑。

## 执行及验收

开发按 JSON 精确删除，清理因此未使用的 imports/helper；保留其他测试断言。报告须列出实际删除的函数及原参数化执行项、净删行数、保留回归命令与结果。先做收集差异检查，确保没有清单外用例消失；在变更文件上验证保留测试，前端使用现有 Vitest。C++ 如无 ROS 工具链，应明确未编译，不能记为通过。不得新增测试或借机改产品。

当前状态：负责人已核对实际 diff 与收集差异，清单内删除已完成；未执行的 OpenCV/ROS 验证仍按报告保留。
