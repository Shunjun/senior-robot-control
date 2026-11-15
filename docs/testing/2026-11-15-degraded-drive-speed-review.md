# 退化手动限速：负责人审阅后的增量修订

当前状态：总负责人已亲读累计与增量生产 diff、新增模块、测试及证据，代码审阅通过，授权提交本修复分支、推送及向 main 创建 **Draft PR**，不合并。仅认可模块验证；完整 Linux ROS 消息生成、guard 编译及实车验证仍缺证据。本轮只更新交付状态，不重复跑测试。

## 旧策略来源核查

基线仍为 `ad1c0c6e5764f436d0d25ec6622a9ff9d32afd09`，分支仍为 `fix/degraded-drive-speed`，工作区未变。前两阶段回报时未提交，当前按负责人明确授权完成分支提交和 Draft PR 交付。

上一版把 `degraded_cap=0.15` 解释为不可调高的物理边界不成立，现明确撤回。证据来自 main 已有内容，而非推测：

- 提交 `6fde6b0a0ea70f9d0d3cfb80ae7953ed7713c488` 新增的 `config/hardware-motion.json` 写明 `ASSUMED configuration requested by operator; no physical measurements or guarantees`，同时给出 0.15；该文件 `unvalidated=true`。
- 同提交原始 `docs/operations/hardware-motion-parameters.md` 首段写明没有猜填物理数值获实测保证，`degraded_cap` 行明确标“猜填”。`docs/testing/2026-11-14-vehicle-real-control-revision.md` 说明该 JSON 是用户要求凭空填写的完整数值草稿，没有实测响应/制动/误差保证。
- 仿真接线引入提交 `f06d046` 的 profile 也明确说明这些数值为 unvalidated hypotheses，不是测量或验收冻结。因此不能仅因字段存于 profile 就把同义旧策略升格为物理边界。
- 当前 `FusionSnapshot.target_cap_mps` 没有下游读取点；实际输出 cap 在 `MotionCoreOutput._snapshot()` 生成。无需改变融合数值/源质量判断即可修正手动权威来源。

## 本轮增量 diff

核心逻辑只有一处策略选择变化，归一化映射位置、原期限、轨迹控制、避障及最终核验保持上一版结构：

```diff
 if target.source == "manual" and target.manual_linear is not None:
+    # Manual degraded speed has one policy owner: the saved driving setting.
+    # p.degraded_cap is the legacy navigation policy, not measured capability.
     if degraded:
-        cap = min(cap, target.manual_linear.degraded_mps)
+        cap = min(p.cap, target.manual_linear.degraded_mps)
```

`degraded_linear_mps` 现在是唯一手动退化策略来源。默认满杆 0.3、半杆 0.15 m/s，自定义 0.6 在正常能力/档位允许时可以生效，不再被旧 0.15 或仿真旧 0.3 策略重复限制。正常能力 `p.cap` 仍由 `requested_linear_mps`、`wheel_speed_mps`、`assumed_body_cap` 取小；倒车、左右轮组合、方向许可和闭环控制继续约束输出。这里保留能力声明和响应模型，并未把其中未验证假设称作实测能力，也没有修改任何 profile JSON 数值或运行中的实车配置。

导航没有手动摇杆满杆概念，本轮只改用户批准的手动驾驶，因此导航继续使用原 `degraded_cap` 策略，不消费网页手动退化设置。保留该旧字段供导航及现有 profile 读取，不同时建立第二套手动退化设置；导航策略的后续调整应独立评审。

其余增量文件：

- `motion/manual_target.py`：说明更新为正常能力上限 + 单一手动退化策略。
- `VehicleSettings.tsx`：回读标签由“生效”改成“已保存上限”，文案明确手动范围以及档位、倒车、正常车体/轮能力和障碍仍会收紧；不把配置值说成实际车速。
- `test_degraded_manual_speed.py`：将上一版错误的“物理退化 cap”断言改为正常能力 cap 断言；新增旧策略 0.15 不覆盖默认的低/高档正反向百分比、自定义 0.6 超过旧策略、导航保留原策略测试。
- `settings.test.tsx`：编辑草稿时回读仍显示原已保存值，保存并回读后才显示新已保存上限。
- `hardware-motion-parameters.md` 和原交付报告：纠正旧策略归属与公式，记录手动/导航范围及 Draft 限制；新增本增量报告。

## 增量验证

遵循本轮指令，仅补测新改动和覆盖缺口，没有例行重跑上一轮同版同环境已通过的完整套件。

使用原仓库只读虚拟环境解释器，`PYTHONPATH=src` 导入本工作区：

1. 修复前运行 `pytest tests/unit/test_degraded_manual_speed.py -k 'normal_cap_still or old_strategy or custom_manual_limit' -q`：**16 failed / 42 deselected**，均因旧策略 cap 覆盖设置（例如默认半杆得到 0.075 而期望 0.15，自定义满杆得到 0.15 而期望 0.6）。另有正常能力 0.1 未被正确消费的断言也失败。
2. 修改策略选择后同一命令：**16 passed / 42 deselected**。
3. 加入导航范围测试后运行 `pytest tests/unit/test_degraded_manual_speed.py -k 'normal_cap_still or old_strategy or custom_manual_limit or navigation_retains' -q`：**17 passed / 42 deselected**。
4. `cd web && npm test -- --run src/features/settings/settings.test.tsx`：**1 file / 4 tests passed**，包括上述保存值与草稿分离的增量断言；`npm run typecheck`：通过。只有文案及测试变化，没有例行重跑前轮已通过的前端生产构建。
5. 本轮修改的两个 Python 源文件和对应增量测试 `ruff check`：通过；`git diff --check`：通过。

初次静态检查从 web 工作目录执行了仓库根相对路径，报找不到文件；改为根目录后完成上述有效检查。该路径错误不算 lint 或测试通过。

## ROS 消息生成 / guard 完整构建缺口

已查现有构建途径，未冒充完成：

- `.github/workflows/backend.yml` 只做 Python 检查/测试及 wheel 构建，没有 ROS underlay、colcon 或消息/guard 编译。
- `Jenkinsfile` 只构建网页并部署云端，不是 ROS 构建流程；本轮不触发部署。
- 本机没有 `ros2`、`colcon` 或 `/opt/ros`。
- Docker 的已选 context 为 `desktop-linux`；在解除只读查询的 sandbox 连接限制后，`docker info` / `docker image ls` 仍返回 **Cannot connect to the Docker daemon**，当前没有可用 Docker 构建环境；没有启动 Docker Desktop 或容器。
- `prlctl list -a` 只读确认 Ubuntu 22.04 VM 为 **suspended**，Ubuntu 26.04 为 **stopped**。未恢复虚拟机，避免恢复未知的旧 ROS/开发进程；既有统一构建根为 `/home/toby/senior-audit-20261031/`，须遵守 `.vm-test.lock` 串行和冻结源码核验约定。
- 既有实车编译记录仅证明旧版 `native_receipt.cpp` 的对象构建，明确不是本轮 SCF3 生成消息/guard 完整链接证据。本轮未连接实车、启动服务或改设备配置。

因此完整 ROS 构建缺口保留为 **Draft 限制**。在后续明确可用的隔离 Ubuntu 22.04/Humble 环境，应冻结本分支源码，运行 `colcon build --packages-select senior_motion_msgs` 完成所有生成接口及 `final_command_guard`/`native_receipt` 链接，再执行该包 CTest 并记录结果；本轮既有无 ROS 的 C++ frame roundtrip 不能替代这一步。SCF3/823 必须同版部署，不能混用旧节点。

历史增量修订阶段未提交、推送或创建 PR；当前负责人已完成上述增量与累计 diff 审阅并授权分支提交、推送和 Draft PR，保留工作区，不合并。不能将定向模块测试视为 ROS/Gazebo 或实车验收。
