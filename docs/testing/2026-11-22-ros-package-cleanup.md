# ROS 包清理验证记录

功能分支 `feat/vehicle-architecture-refactor`，基线 `da3ef99728544b566fd29adc55fdb2cac16c8833`。产品迁移分批提交 `93602d8`、`539a46e`、`5884870`；定位入口断言修正 `986a20d`。本记录不表示车端部署或完整重构验收完成。

按重命名前后路径分别计数，最终为366个路径（新增81、修改174、删除111）；Git合并重命名显示为305个逻辑文件。超过初报350上限16个路径，主要是收尾动态测试入口及迁移文档，已向用户更正。

## 范围与用例

获批八组验证范围见 [实施计划](../superpowers/plans/2026-11-22-ros-package-cleanup.md)。新增/实质改写的独立场景共7项：十包及 manifest DAG；31份接口字段完整性；仿真 transport 实际安装声明；定位实际安装入口；导航 canonical 安装验证；只读实车预检；隔离 wheel 导入与退役模块缺失。前两项为新增 pytest，其他复用既有验证或安装检查。其余批量修改仅迁移 import、patch target、资源路径和显式组合参数，保留原算法断言。

实际 bug 定向回归1项：相机包迁移后，可选包缺失触发 ModuleNotFoundError，旧异常判断仍检查 senior_robot。修复前 `test_preview_without_optional_ros_package_is_disabled` 失败；改为精确识别 senior_sensor_ingress 后通过。内部依赖缺失仍重新抛出，不扩大异常捕获。

退役断言：robot 的仿真 Include-only 转发、重复导航 share 数据及已移除 facade 的对象身份。移除这些断言与用户授权删除旧入口对应；canonical 配置、入口、算法和退出保护仍保留。

## 已执行

| 验证 | 结果 |
| --- | --- |
| 修改前广泛基线 | 2248 passed、200 failed、112 errors、115 skipped，519.05s |
| ownership、launch/profile、lifecycle、perception、native framing、camera regression 定向集 | 72 passed、43 skipped，3.71s；代码539a46e |
| wheel/IMU/stillness/recovery | 36 passed，0.31s；类型归属调整后 |
| 较大 startup/rearm 定向集 | 42 passed、2 failed、13 errors；失败/error节点均在基线 |
| unit + estimation/perception/mapping/localization 广泛集 | 1217 passed、116 failed、104 errors、11 skipped，962.85s；期间 asyncio 等待需 SIGINT 才继续；汇总后进程仍未退出，已终止本次进程，不作为全量通过 |
| 广泛集唯一新增失败修正 | localization 旧 setup 注册断言改为 canonical CMake + 可执行脚本；18 passed，0.10s；986a20d |
| 独立 point-index C++ | clang++ 构建运行，3个等价场景 PASS |
| Python wheel | core、contracts、manager、七个已有 ROS Python wheel 项目共10份构建成功；最终 estimation/control 重建并重新安装 |
| 隔离安装导入 | `python -I -S`，仅安装目录与第三方site-packages；canonical模块来自安装目录、退役 facade 不存在、类型身份、导航契约、只读预检 PASS |
| 实际视频回环及相机 IPC/lease 生命周期 | 48 passed，12.29s；允许本机 UDP/Unix socket 的测试执行，无产品服务启动 |
| 最终同范围 collection | 2672 tests collected、3个原有错误，1.69s |
| Ruff | check通过；修改的 Python 文件格式检查通过 |

广泛集与基线按精确 failing/error node 名称比对，唯一新增节点为定位注册断言，已单独修复并重跑对应18项。未删除已有失败，也未通过扩大 skip 或弱化算法断言取得结果。旧失败主要涉及已经变化的准入行为、旧 controller 属性和 ScanGeometry 等接口；本轮保留供后续迁移。

日志保存在本机 `/tmp/ros-cleanup/`：baseline.log、modules-after.log、checkpoint2-tests.log、recovery-tests.log、collect-comparison-final.log、media-unrestricted.log。默认全树收集还发现 manager/e2e 的 `test_home` 同名导入冲突；原比较范围不含 e2e/deploy，不以默认收集结果伪称无错误。

主要复现命令：

```sh
.venv/bin/ruff check .
.venv/bin/pytest -q tests/unit/test_ros_package_ownership.py
.venv/bin/pytest -q tests/integration/test_video.py senior_control/tests/test_camera_demand.py
.venv/bin/pytest -q tests/unit/test_unified_motion_runtime.py tests/unit/test_wheel_imu_stillness.py
.venv/bin/pytest -q ros2/src/senior_localization/test/test_localization.py
.venv/bin/pytest -q tests/unit ros2/src/senior_motion_estimation/test ros2/src/senior_perception/test ros2/src/senior_mapping/test ros2/src/senior_localization/test --continue-on-collection-errors --tb=line -o faulthandler_timeout=90
clang++ -std=c++17 -I ros2/src/senior_motion_estimation/src ros2/src/senior_motion_estimation/test/test_point_index.cpp -o /tmp/point-index-check
/tmp/point-index-check
```

## 未验证及边界

本机 macOS 无 ROS/ament/CMake，Docker daemon 未运行；没有启动 Docker 或开发服务。wheeltec.local 域名无法解析，未部署、未重启实车服务、未发运动命令。ROS 原生 typesupport、SLAM链接、已安装时序头 consumer、真实DDS和整车链路仍需 Linux 环境整体干净构建验证；参见 [当前包归属与重建步骤](../../ros2/PACKAGE_OWNERSHIP.md)。

manifest依赖声明无环，不等于所有 Python 功能模块完全独立。原感知/估计互引和感知学习 profile 类型对控制的引用仍在，本轮不扩展成算法分层重写。PR保持 Draft，不能把已有大量失败或缺少的车端验收说成全部任务完成。
