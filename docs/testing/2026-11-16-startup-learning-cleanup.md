# 启动学习旧文件清理小项

代码修改完成，定向39项通过，待负责人亲审；未提交、推送或暂存。分支fix/degraded-drive-speed，当前HEAD为593e8f13f385892f0bd62cf75c593f7b07efd2da，修改为其上未提交增量。

## 范围与并行边界

负责人msg_f1ada43e40c3已确认独占两个极小接线块：docker_start_helper.py:supervise的Agent创建后块、simulation_cli.py:supervise的READY前块，导航接线函数未改。实际新增hunk分别17行和11行。其余实现独立于runtime/learning_cleanup.py与tests/unit/test_learning_cleanup.py。

已冻结第7项1cm模型文件SHA256仍为df9460f4857d3d6afb13bd1e04c0b1af52cbca05182c9ab8cfdfd1b9fea05725，三份保护文件逐字节未变。未接手导航8—9，不修改后续共享学习记录/恢复设计，不把一次学习尝试规则标记成已删除。

## 一次启动边界

硬件：现有.agent-service.lock持有期间，adapter身份已通过_wait_for_adapter验证，Agent已创建，创建其wait任务并让出一次事件循环给退出通知，Agent/adapter/motion三个进程returncode仍为None且未收到停止请求后清理一次。无profile的任意单子进程兼容入口不清理。随后可选camera退出分支和主监督循环不再调用清理。

仿真：现有main持有域锁及.senior-simulator.lock，adapter验证返回、Agent创建后让出一次事件循环，ROS和Agent仍显示存活且未停止时，在READY通知前清理一次。

这是“adapter已验证，Agent已创建并在该事件循环检查时显示存活”的边界；当前没有更强的Agent应用层ready信号，不声称云连接或Agent应用层完全就绪，不增加固定等待延迟。检查之后退出仍由既有监督处理，异常残留留给下一次对应成功启动清理。

## 清理规则与失败处理

只精确匹配现有uuid4().hex生产器生成的小写32hex名称：.startup-learning-<32hex>、其.features.json与.features.json.pending。保留SENIOR_STARTUP_LEARNING_CLAIM对应本次run及全部上述配套；其他任意后缀也不会匹配，未来共享状态格式没有提前兼容壳。孤立旧pending配套可清理，不要求旧claim存在。

打开control root使用O_DIRECTORY|O_NOFOLLOW；用目录fd枚举、nofollow stat后只unlink普通文件，不递归、不删目录、不读取/跟随文件符号链接，不动其他任意文件。失败仅记录warning并继续或跳过，不触发主进程退出或云断连。

硬件与仿真根目录服务锁不同，因此清理额外非阻塞持有另一类既有服务锁；若它被占用则记录日志并跳过清理，防止删除同root另一活跃监督运行的标记。此锁只在清理期间持有，不改变服务启动/停止协议。现有锁由遵守服务入口的程序持有；不额外扫描进程或引入新的run登记/恢复协议。

## 同版集中验证

```sh
.venv/bin/pytest -q tests/unit/test_learning_cleanup.py tests/unit/test_startup_learning.py tests/unit/test_docker_start_helper.py::test_verified_adapter_precedes_agent_and_agent_stops_first tests/unit/test_simulation_reference.py tests/unit/test_simulation_video.py --junitxml=/tmp/learning-cleanup-tests.xml
```

最终39 passed in 0.73s，证据/tmp/learning-cleanup-tests.log及/tmp/learning-cleanup-tests.xml。首次沙箱执行37通过、两项既有测试被临时Unix socket bind权限阻止；同版通过权限机制重跑后39全部通过。新监督测试使用假子进程及假IPC server；两项既有监督测试使用临时Unix IPC及假子进程。未启开发服务、ROS或车辆。

覆盖当前claim及配套保护、旧claim/孤立配套删除、非项目名称/未知后缀保留、目录不递归、符号链接及根链接不跟随、另一活跃服务锁保护、删除/枚举失败非致命；硬件/仿真adapter失败及Agent已退出时不清理，活跃成功边界只调用一次，硬件清理枚举失败后仍正常监督退出。锁持有通过另一个fd的非阻塞flock失败实际核验。

Ruff check及format --check对4个修改Python文件通过；mypy对3个产品文件通过（Success: no issues found in 3 source files），日志/tmp/learning-cleanup-scope-mypy.log；Python3.10实际AST解析4文件通过。当前测试Python3.12.15、Ruff0.17.3、mypy2.4.0，macOS arm64。仅核查本项diff --check通过，未处理并行导航文件的CRLF/空白报告。

## 实际增量与指纹

补丁/tmp/learning-cleanup.patch包含两个监督入口hunk、新模块、新测试及计划，排除所有其他任务增量。SHA256：719f3903749c6266a622549de774b8b630790492b1c1486f573da223129a1ffd

```text
c23fc38de94b60f3d0c8ff8b0f3dc0934d03d2ffdcc4f53be252454f832feb3a  src/senior_control/docker_start_helper.py
9588ee53b7d3d3428a7c7b36e82289d8370f68b0b8aa1757cce8e7f7fc864607  src/senior_control/simulation_cli.py
d63675f8bcf491c3aded1ae3036c778733d14fb3e6e6eb97eb403b2bcf30806b  src/senior_control/runtime/learning_cleanup.py
02d583a97bd702684bd493d672c05f543c5c4584d0c3561435dca05f6b8b5688  tests/unit/test_learning_cleanup.py
cfc95f2d707ab5d80d7c3ea1ac706a77495b07a4a34acebf3b7c88ffc4233f23  docs/superpowers/plans/2026-11-16-startup-learning-cleanup.md
```

根方案第12项仅追加清理实施状态，保留用户原文及后续共享状态恢复待实施要求。硬件/ROS完整启动验收尚未执行；本小项不改变学习状态/结果仍由后续独立任务处理的事实。
