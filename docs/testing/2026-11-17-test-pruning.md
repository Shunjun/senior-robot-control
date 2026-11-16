# 测试精简执行报告

基线 `5f59042dd9413cd50422b36c411ca9f2226c06f8`，分支 `test/prune-redundant-cases`。删除对象只来自 `docs/reviews/2026-11-17-test-pruning-manifest.json`。产品代码、配置和构建文件没有改动。未提交、未推送、未创建 PR。

代码 diff：21 个路径，859 行删除，0 行插入。`git diff` 的 SHA-256 为 `2ec4c07c87e012b50389b8d37051924bad4f60fcc2ec7d53e161774edf928c82`。

## 收集差异

unit 与 integration 分开收集。unit 与 integration 没有同名测试模块；同名模块是 `tests/unit/test_home.py` 与 `tests/e2e/test_home.py`。本机没有 `cv2`，`test_independent_motion.py` 在导入时 `importorskip`，`test_rgbd_spawn_worker.py` 又导入该模块，所以这两组在普通收集里整模块跳过。为核对它们的参数化节点，另外用仓库外的临时模块 `/tmp/prune-cv2-stub/cv2.py` 放在 `PYTHONPATH` 前面再收集。该桩没有装进 `.venv`，也没有拿来跑测试。

| 收集 | 删除前 | 删除后 | 消失 | 新增 | 退出码 |
|---|---:|---:|---:|---:|---:|
| `tests/unit` | 1875 | 1828 | 47 | 0 | 0 |
| `tests/integration` | 657 | 656 | 1 | 0 | 0 |
| 上述两个 cv2 模块（临时桩） | 101 | 80 | 21 | 0 | 0 |

三组消失节点没有交集，合计 69 个。其中 68 个是 43 个 Python 函数的参数化展开，1 个是整文件 `test_native_checked_age_input_matrix_does_not_throw`。消失名单与 manifest 一致，没有清单外节点消失，也没有新节点。

原始收集输出在 `docs/testing/2026-11-17-test-pruning-evidence/collect-*-before.txt` 与 `collect-*-after.txt`。

## 删除项

删除原因与保留覆盖摘自负责人审阅文档，没有另行取舍。节点数是 pytest 收集到的展开项；没有参数化的函数记为 1。函数内循环不算进节点数，单独写在备注里。

| 删除对象 | 节点 | 备注 | 删除原因 | 仍保留的覆盖 |
|---|---:|---|---|---|
| `test_extreme_finite_principal_point_returns_unavailable_without_numerical_exception` | 1 | | 1e308 / 10**1000 等不具实际传感器含义的数字 | 正常 PnP、运动变换、真实图像缺陷与来源替换 |
| `test_extreme_integer_intrinsics_are_rejected_without_overflow` | 1 | | 同上 | 同上 |
| `test_body_transform_overflow_is_rejected` | 1 | | 同上 | 同上 |
| `test_extreme_receipt_and_profile_numbers_do_not_escape_as_overflow` | 1 | | 同上 | 同上 |
| `test_malformed_capture_identity_is_rejected` | 1 | | 手工把 list、迭代器或错误标量塞入内部不可变对象 | frame builder 输入快照、原始消息序列化及正常不可变副本 |
| `test_mutable_dependencies_are_rejected_instead_of_aliasing_caller` | 1 | | 同上 | 同上 |
| `test_dependency_identities_have_immutable_bounded_shape` | 5 | `dependencies0`–`dependencies4` | 同上 | 同上 |
| `test_calibration_container_metadata_cannot_alias_mutable_values` | 1 | | 同上 | 同上 |
| `test_output_constructor_rejects_mutable_transform_or_pair` | 1 | | 同上 | 同上 |
| `test_metadata_scalars_cannot_hide_mutable_objects` | 1 | | 同上 | 同上 |
| `test_retired_source_capacity_exhaustion_fails_closed` | 1 | `range(1, 67)` 为 66 次，随后再供给第 67 次 | 连续制造 67 次新来源以命中内部退休表容量 | 正常来源重启、替换与旧数据不复活 |
| `test_unrepresentably_large_axis_is_rejected_without_raising` | 2 | `linear`、`turn` | 10**1000 与 10**308 的时钟、TTL、轴值 | 正常 TTL、原始接收时间、释放和一般输入越界 |
| `test_unrepresentably_large_configuration_is_a_value_error` | 2 | `ttl`、`now_steady` | 同上 | 同上 |
| `test_large_integer_deadline_sum_is_rejected_as_configuration_error` | 1 | | 同上 | 同上 |
| `test_large_integer_receipt_sum_rejects_without_raising_or_renewing` | 1 | | 同上 | 同上 |
| `test_huge_finite_off_diagonal_covariance_is_rejected_without_overflow` | 1 | | 极大协方差；向整数序号塞 NaN/list 等；混合重复非法内部结构 | 正常序号去重、来源健康、期限及数据质量拒绝 |
| `test_source_sequence_must_be_a_real_positive_integer` | 3 | `nan`、`1.5`、`True` | 同上 | 同上 |
| `test_invalid_sequence_cannot_poison_the_replay_watermark` | 1 | | 同上 | 同上 |
| `test_malformed_structure_after_valid_samples_is_pure_rejection` | 13 | `changes0`–`changes12`；每个节点内 `range(3)` 再拒绝 3 次 | 同上 | 同上 |
| `test_cadence_rejects_nonfinite_and_backwards_clock` | 4 | `nan`、`inf`、`-inf`、`-0.01` | 把本机单调时钟注入 NaN/Inf | 实际调度超期、跳周期及正常权限生命周期 |
| `test_nonfinite_clock_revokes_prepared_permission` | 3 | `nan`、`inf`、`-inf` | 同上 | 同上 |
| `test_nonfinite_inverse_result_is_rejected` | 1 | | 1.7e308 米平移 | 常规 SE2/SE3 变换和杆臂 |
| `test_nonfinite_geometry_extremes_preserve_legacy_filter_behavior` | 1 | | 1e308、1e-16 与指定几何构造路径绑定；强制继续走旧 LineString 分支 | 实际后方场景几何等价与优化表现 |
| `test_tiny_segment_and_exact_mask_boundary_keep_legacy_linestring_path` | 1 | 函数内 2 组距离/掩码 | 同上 | 同上 |
| `test_near_degenerate_triangle_keeps_legacy_mask_intersection_path` | 1 | | 同上 | 同上 |
| `test_seeded_gaps_and_unordered_rays_preserve_the_original_observed_set` | 1 | `range(16)`，16 个随机组合 | 同时随机缺束、交换端点及改变射线原点 | 命名的 actual / masked / gap 等正常几何场景 |
| `test_malformed_numbers_fail_closed` | 1 | | 对 ROS double/整数/固定内部结构注入字符串、list、bool | 真实 TF 组合、来源替换、图结构和生命周期 |
| `test_malformed_lifetime_metadata_fails_closed` | 1 | | 同上 | 同上 |
| `test_mutable_edge_storage_cannot_enter_binding` | 1 | | 同上 | 同上 |
| `test_transform_rejects_bool_strings_and_nonfinite_values` | 1 | 函数内 5 个非法标量 | 同上 | 同上 |
| `test_serialization_rejects_mutated_and_constructed_nonfinite_payloads` | 1 | | 验证通过后改成 NaN，或用 model_construct 绕过验证 | 公共协议解析及非法外部输入检查 |
| `test_reply_rejects_nonfinite_server_clock` | 1 | | 特殊合成 NaN 回执 | 正常心跳、身份校验和五秒在线边界 |
| `test_malformed_worker_output_fails_closed_instead_of_raising` | 1 | | 私有结果队列塞任意对象；十轮 SIGSTOP；线程启动精确失败；伪造线程未退出 | 一次实际超时/进程死亡、恢复、关闭及正常 IPC |
| `test_nonfinite_candidate_result_is_rejected` | 1 | | 同上 | 同上 |
| `test_repeated_hung_large_frames_reclaim_ipc_threads_and_descriptors` | 1 | `range(10)` | 同上 | 同上 |
| `test_reader_thread_start_failure_reaps_spawned_child_and_ipc` | 1 | | 同上 | 同上 |
| `test_reader_start_failure_on_restart_reaps_child_and_closes_admission` | 1 | | 同上 | 同上 |
| `test_shutdown_complete_stays_false_if_reader_or_feeder_did_not_stop` | 1 | `reader` 与 `feeder` 两组 | 同上 | 同上 |
| `test_current_malformed_header_revokes_without_raising` | 1 | | 将本应为字符串的内部标识改为列表 | 实际来源替换、重启和延迟结果 |
| `test_camera_roi_rejects_boolean_float_string_and_out_of_range` | 1 | 函数内 6 个非法 ROI 值 | 对 uint32 ROS ROI 字段注入 bool、字符串、负数及溢出值 | 正常 numpy ROI、相机矩阵与真实图像布局 |
| `test_unrepresentable_stop_model_is_a_brake_request_not_a_crash` | 1 | | 1e-300 制动参数 | 实际制动距离、jerk、扫掠及障碍约束 |
| `test_export_rejects_symlink_substituted_between_stat_and_open` | 1 | | 精确卡在 lstat/open 之间的恶意替换竞争 | 基本路径穿越/符号链接拒绝、正常地图导入导出 |
| `test_agent_release_is_bounded_for_cancellation_resistant_work` | 1 | | 伪造主动吞掉每次 CancelledError 的永久任务 | 实际任务收束、正常关闭及进程故障隔离；同文件 `test_agent_release_is_bounded_for_stubborn_cleanup` 仍在 |
| `tests/unit/test_native_sensor_age_boundary.py` 整文件 | 1 | 函数内断言矩阵：9 次非法时钟、2 次有限年龄、4 个软/硬阈值、抛出路径、uint64 饱和 | 浮点最大值、285 年运行时长、uint64 计数饱和与 noexcept 实现约束混合矩阵 | `test_shared_sensor_cpp.py` 的正常 C++ 年龄策略、`test_sensor_timing.py` 的期限 |
| `web/src/api/client.test.ts` 的 `does not append a motion sample behind a buffered socket` | 1 个 `it` | | 与紧邻完整手动控制背压流程重复 | 保留 held input / drain / release 的完整流程 |
| `RgbdCandidateReceipt.RetirementExhaustionRevokesActualProducerOnGraphLoss` | 1 个 `TEST` | | 通过 TestPeer 强行填满退休表后叠加非法候选、图丢失 | producer 重启、真实发布者消失、正常所有者变更仍留在同一 C++ 测试文件 |

前 11 项在 `tests/unit/test_independent_motion.py`。manual session 4 项、motion sources 4 项、cadence 与 manual authority 各 1 项、rear geometry 3 项、static TF binding 3 项、spawn worker 6 项，其余文件各 1 项。Python 函数展开合计 68，加整文件 1 个节点。

## 净删行与因此删掉的辅助代码

| 文件 | 净删行 | 辅助代码 |
|---|---:|---|
| `tests/unit/test_independent_motion.py` | 131 | 无。其余 helper 仍被保留测试使用 |
| `tests/unit/test_manual_session.py` | 30 | 无 |
| `tests/unit/test_motion_sources.py` | 57 | 无 |
| `tests/unit/test_control_process_resources.py` | 11 | 删除只被该参数化用例使用的 `import math`。autouse fixture `release_closed_future_owner_cycles` 仍被保留测试使用 |
| `tests/unit/test_manual_authority.py` | 9 | 无 |
| `tests/unit/test_rgbd_body_motion.py` | 11 | 无 |
| `tests/unit/test_rear_geometry_equivalence.py` | 94 | 删除只被已删用例使用的 `from shapely.errors import GEOSException` |
| `tests/unit/test_rear_fan_optimization.py` | 26 | 删除只被已删用例使用的 `import random` |
| `tests/unit/test_static_tf_binding.py` | 36 | 无 |
| `tests/unit/test_static_tf_wire.py` | 11 | 无 |
| `tests/unit/test_protocol.py` | 15 | 无 |
| `tests/unit/test_websocket_link.py` | 12 | 无 |
| `tests/unit/test_rgbd_spawn_worker.py` | 174 | 无 |
| `tests/unit/test_rgbd_parent_ledger.py` | 23 | 无 |
| `tests/unit/test_rgbd_frame_builder.py` | 22 | 无 |
| `tests/unit/test_directional_constraints.py` | 7 | 无 |
| `tests/unit/test_map_archive.py` | 23 | 无 |
| `tests/integration/test_agent.py` | 23 | 无 |
| `tests/unit/test_native_sensor_age_boundary.py` | 72 | 整文件删除 |
| `web/src/api/client.test.ts` | 19 | 无。`bufferedAmount` 仍被保留的背压流程使用 |
| `ros2/src/senior_sensor_ingress/test/test_rgbd_candidate_receipt.cpp` | 53 | 删除只被该 `TEST` 调用的 `RgbdCandidateReceiptTestPeer::fill_producer_retirement`。同文件其余 8 个 `TEST` 和 TestPeer 其他方法仍在。产品头文件里的 friend 声明未改 |

合计 859 行。`tests/unit/test_rear_geometry_equivalence.py` 里的 `_random_cases` 与 `_boundary_cases` 在删除前就没有调用方，本次没有动它们。带 `noqa: F401` 的 `synthetic_domain` 导入同样保持原样。

删除后 18 个改动过的 Python 测试文件 `ruff check` 与 `ruff format --check` 均通过。

## 保留测试的执行

命令：

```text
NO_COLOR=1 .venv/bin/pytest -q -p no:cacheprovider --tb=short -rs \
  --junitxml=docs/testing/2026-11-17-test-pruning-evidence/pytest-affected.xml \
  --basetemp=/tmp/prune-pytest-affected --durations=25 \
  tests/unit/test_independent_motion.py \
  tests/unit/test_manual_session.py \
  tests/unit/test_motion_sources.py \
  tests/unit/test_control_process_resources.py \
  tests/unit/test_manual_authority.py \
  tests/unit/test_rgbd_body_motion.py \
  tests/unit/test_rear_geometry_equivalence.py \
  tests/unit/test_rear_fan_optimization.py \
  tests/unit/test_static_tf_binding.py \
  tests/unit/test_static_tf_wire.py \
  tests/unit/test_protocol.py \
  tests/unit/test_websocket_link.py \
  tests/unit/test_rgbd_spawn_worker.py \
  tests/unit/test_rgbd_parent_ledger.py \
  tests/unit/test_rgbd_frame_builder.py \
  tests/unit/test_directional_constraints.py \
  tests/unit/test_map_archive.py \
  tests/integration/test_agent.py \
  tests/unit/test_shared_sensor_cpp.py \
  tests/unit/test_sensor_timing.py
```

结果：424 passed，2 skipped，0 failed，10.50s，退出码 0。两个 skipped 是模块收集跳过，不是单条用例跳过。

| 文件 | 结果 |
|---|---|
| `test_manual_session.py` | 69 passed |
| `test_motion_sources.py` | 28 passed |
| `test_control_process_resources.py` | 7 passed |
| `test_manual_authority.py` | 34 passed |
| `test_rgbd_body_motion.py` | 3 passed |
| `test_rear_geometry_equivalence.py` | 2 passed |
| `test_rear_fan_optimization.py` | 9 passed |
| `test_static_tf_binding.py` | 15 passed |
| `test_static_tf_wire.py` | 20 passed |
| `test_protocol.py` | 22 passed |
| `test_websocket_link.py` | 13 passed |
| `test_rgbd_parent_ledger.py` | 13 passed |
| `test_rgbd_frame_builder.py` | 13 passed |
| `test_directional_constraints.py` | 77 passed |
| `test_map_archive.py` | 27 passed |
| `tests/integration/test_agent.py` | 58 passed |
| `test_sensor_timing.py` | 13 passed |
| `test_shared_sensor_cpp.py` | 1 passed |
| `test_independent_motion.py` | 收集跳过 |
| `test_rgbd_spawn_worker.py` | 收集跳过 |

`test_shared_sensor_cpp.py::test_cpp_age_queue_boundaries_and_transient_crossing` 用本机 `c++` 编译并跑通。这只覆盖保留的年龄策略，不是 ROS 包里的 gtest。

前端：

```text
cd web && npx vitest run --reporter=dot
```

Vitest 5.0.5：13 个文件、168 个测试 passed，3.49s，退出码 0。`src/features/devices/devices.test.tsx` 有一条既有 `act(...)` stderr 提示，该文件测试仍通过。日志见 `docs/testing/2026-11-17-test-pruning-evidence/vitest.log`。

类型检查：

```text
cd web && npx tsc --noEmit
```

TypeScript 5.9.3，退出码 0。

```text
.venv/bin/mypy src/senior_control
```

mypy 2.4.0 退出码 1，170 个源文件里 6 个错误，位于 `tasks/lifecycle.py`、`motion/constraints.py`、`motion/controller.py`、`tasks/lifecycle_store.py`。这 4 个文件不在本次 diff 中，没有改产品代码。

## 未验证

- `test_independent_motion.py` 删除后仍有 39 个测试函数，`test_rgbd_spawn_worker.py` 仍有 22 个。本机 `.venv` 没有 `cv2`，两个模块在收集时跳过，保留用例没有实际运行。临时桩只用于列出节点，没有执行这些测试。
- `test_rgbd_candidate_receipt.cpp` 未编译。`/opt/ros` 不存在，`colcon` 不存在，`.venv` 中 `import rclpy` 失败。源码核对：`RetirementExhaustionRevokesActualProducerOnGraphLoss` 与 `fill_producer_retirement` 已不在；仍保留 `RealOwnersBindBothPixelBodiesAndRejectTampering`、`ProducerBootChangeOnSameGidRevokesImmediately`、`NewBridgeBootCanRequalifyAfterObservedGraphLoss`、`SameCurrentHeaderWithDifferentPixelsRevokes`、`InvalidTfGenerationRejectsDelayedOlderValid`、`ReconsidersSingleTransientStatusWhenOwnerBecomesUnique`、`ClockRollbackRequiresRestart`、`HeaderAgeBoundaryRejectsInputWithoutThrowingOrFakeAge`。这不是 gtest 通过。
- e2e、ros、deploy 目录没有改动，本次没有重跑。

## 指纹与起始快照

HEAD `5f59042dd9413cd50422b36c411ca9f2226c06f8`。起始工作区只有两份未跟踪的负责人审阅文件，`git diff` 为空。起始快照：

| 文件 | SHA-256 |
|---|---|
| `start-git-diff.patch` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `start-git-status.txt` | `0481540f61bf5277b79ce79857edd441acc34083c3b5bd03f58dc22d48a40751` |
| `start-head.txt` | `b4e7aa60c90ec46136e78c06193f25b136b40a038e15e839a80540bc055c86c4` |
| `start-source-inventory.txt` | `192c8582311e821a4a0d5c5dc27437d7a94289bfcdd0fef50ab790115abf0fea` |

环境：Python 3.12.15，pytest 8.4.2，Ruff 0.17.3，mypy 2.4.0，Node v24.14.1，Vitest 5.0.5，TypeScript 5.9.3。本机 `c++` 为 Apple clang 21.0.0。没有安装或更换依赖。

删除后的文件 SHA-256：

| 路径 | SHA-256 |
|---|---|
| `ros2/src/senior_sensor_ingress/test/test_rgbd_candidate_receipt.cpp` | `d9fb844e1c54e0eb2877ea3bf239741e7ab419d369ec778436c9245a1283c0af` |
| `tests/integration/test_agent.py` | `1bcc37faf9d34db7d55e122b070216d9b8d47f02857e95c297f82c7d1e2e905f` |
| `tests/unit/test_control_process_resources.py` | `7cd1b241c24dd9d1845d5b85776138f8a9ea6334ed43b0dfcb8649c40f4b74b0` |
| `tests/unit/test_directional_constraints.py` | `c453d9a4321e970d1207315c45d5b46ee48c3479c34f464185036867c633c0a8` |
| `tests/unit/test_independent_motion.py` | `649043a2e5c37cb645f2b6d9362a91fd2461a9f170048061e972d65b9e6e5def` |
| `tests/unit/test_manual_authority.py` | `77a28fa1701b8c545c481aea666ce28d8665353a0860f73fa14341eecc33f7d0` |
| `tests/unit/test_manual_session.py` | `13eadcfeb13c09fdc82c2c3a60c6c7d6bae924e9330dbad993a6087a8f38dafa` |
| `tests/unit/test_map_archive.py` | `fe42d8d72c33fbdaff45ae467e5229f74d629b12a026322aff7b54f2a0f9ae64` |
| `tests/unit/test_motion_sources.py` | `64a4f44237f154941bb0f2d014eac5c94105b4df6381cd757dfeb7c2ec0636ea` |
| `tests/unit/test_native_sensor_age_boundary.py` | 已删除 |
| `tests/unit/test_protocol.py` | `4c5b2d08bd6503df301d33c1e382b52d9b775037da94d5f62737449957268fa2` |
| `tests/unit/test_rear_fan_optimization.py` | `fd97646ed8fac1b316169ecdd78cec7aeefc6b568341eea5d5056b0a9db47c24` |
| `tests/unit/test_rear_geometry_equivalence.py` | `e5dc37288d054738ddee14da68b5db8ba42288d626974039534121ac8573a33c` |
| `tests/unit/test_rgbd_body_motion.py` | `4eb65912076d657c9c691808eb2988dc878fa1090c47458a8164d36219dca7c9` |
| `tests/unit/test_rgbd_frame_builder.py` | `01a3bf0c87fb122d91b7b67ef9175fd83c8e7ffcded57714663a53d35ea49eeb` |
| `tests/unit/test_rgbd_parent_ledger.py` | `2f9f0de4acd8f9bdbfe33031cc636391f0db2176816a4d157c46037d1ef504f8` |
| `tests/unit/test_rgbd_spawn_worker.py` | `72f4fba6f72cf7ee6a6c624dfeda7f1b55973996772ad9dac791afbc1ec9f935` |
| `tests/unit/test_static_tf_binding.py` | `e474ec7c1e411842e91adbc735bfd7c63c35ca58c376d093bd9f735159c28504` |
| `tests/unit/test_static_tf_wire.py` | `bc35b386aab768b6d506c446868e7ffd0f86ad2270371d867d3108499cf429de` |
| `tests/unit/test_websocket_link.py` | `2d08de227f121c5517be25cd293607ea9f8bdc58a7a68fb42e7beff4bacf561c` |
| `web/src/api/client.test.ts` | `e7d8300d6dd00785c3c80f8c54289bd3fabec30856861bd8e00aa6c2d9cf3c12` |

未触碰 `native_receipt.py`、`tests/test_native_receipt_diagnostics.py`、`docs/reviews/2026-11-16-software-estop-owner.md`。负责人审阅文档只把执行状态改为待亲审，清单仍在。

## 负责人最终核对

负责人亲自检查了实际 C++/前端 diff，并以 Python AST 对比全部 18 个变更文件：只删除清单列出的 43 个函数，所有保留函数和类的正文、装饰器及顶层参数均未变化。另核对整文件删除、收集前后 69 个节点差异及实际 diff SHA-256，与本报告一致；不存在清单外测试节点消失。产品、配置、构建文件没有变更，`git diff --check` 通过。上述结论不替代未运行的 OpenCV/ROS 验证，也不表示全部保留测试已逐行重审。
