# 实车接线审阅修订与启动回归修复

当前功能分支 `Shunjun/vehicle-real-control-wiring`，HEAD 07b2d27；远端 main 已包含
该交付（9fda281，PR #18）。此次均为该提交后的未提交增量，不需要重复交前一批 PR。
本地 main 仍是 cca4999。已核对 origin/main 另含 7 个文档变更，未导入/覆盖它们；
代码基线包含先前 CLI 联合启动成果，不导入 navigation-validation 的未审成果。

## 首要启动回归

现场用户 journal 已确认：原 config.json 没有新增两字段，helper 在 IPC 创建前失败，
返回 1 导致 systemd 每约 6 s 重启。现修复为：

- 两字段均未选：Compatible 地启动原 Agent/IPC，新增 motion_control_configured=false，
  明确拒绝手动获取/意图/运动任务，CLI 和 IPC 报具体能力缺失；不启动其他控制器。
- 显式选任一字段：执行严格参数校验，不静默退回；配置错误返回 78。
- service 已声明 RestartPreventExitStatus=78，Docker exec 传播退出码；运行期 ROS/IPC
  故障仍返回可重试错误。可选空导航参数不再生成无效 ROS CLI 空 YAML override。

修改未部署到设备，未远程 stop/start 服务；负责人审阅/发布后，旧配置可用原有
senior-robot stop/start/status --control-dir 命令恢复基本 Agent，无需用户自己先造模型文件。

## Humble 真实来源通道

公共 native_receipt.cpp + native_receipt.py 每消费节点一个原生子进程；actual MessageInfo
GID 与唯一 writer 校验，经继承 socketpair 传入原生 receipt/RMW timestamps，不发 ROS relay，
不把 discovery GID 当 sample，不打开 simulation topology-only。覆盖 fusion wheel/IMU/scan、
core_reference、硬件 map/pose/readable ESTOP，以及 CommandNode runtime/执行回执。
来源/通道变化先清 pending、通过唯一 AdmissionOwner 中断 journal 和缓存；保留 seq/revision
frontier。观测 latest-only，执行回执至多 16 条、溢出失效。原接收年龄不被 IPC/TF 排队重计。

补 WHEELTEC 默认 base_footprint 到共用 base_link 的 stamped TF twist/pose 转换，
包括转动导致的速度原点偏移；缺 TF/非平面/无效四元数拒绝，不伪改 frame。

## 急停能力与参数

不再强制不存在的 Bool publisher。模式分 readable / independent_unreadable / unknown；
只有 readable 声明真实状态来源。不可读的独立切断必须给释放要求控制器重启的联锁依据，
否则无法保证不恢复旧速度。未知不虚报 clear，runtime 单独保留物理观察状态；
未核实能力不获运动/恢复资格，但不妨碍未选择模型的 Agent 启动。
厂家 cpp:582 标 Flag_Stop 为预留，chassis_security 是输入，不能作为物理回读或 watchdog 证明。

硬件 profile 派生 Controller/Admission/Temporal/Fusion，重复旧声明不一致即拒绝；
ROS 年龄/速度/静止重复项也核对唯一值。物理近静止阈值扣除测量误差，最终恢复再算预测年龄。
算法默认、名义尺寸、厂家话题/frames、产品请求上限已提供；安装确认保留 false，
实际制动/误差/滑移等只保留物理缺项，不把整表算法交给用户。

用户随后明确要求“凭空填一份”。已新增 config/hardware-motion.json 完整数值草稿，
保持 unvalidated=true，加载器明确拒绝将假设当作实测驾驶模型；不自动改车端 config。
逐变量解释和确定程度见 docs/operations/hardware-motion-parameters.md；
配套 config/hardware-safety.yaml 也已生成，含完整软件默认参数和仍为 false 的安装确认。

## 实际验证及限制

本地变更 Python Ruff、8 个源文件严格 Mypy、compileall 和 git diff --check 通过。
没有新增/运行 pytest/CTest/ROS 行驶测试。用户最新允许必要测试但要求统一目录；
实际只做一次 C++ 对象构建，所有车端产物在
`tests/vehicle_real/native_receipt_build/`，方便统一清除。

经用户授权 SSH，确认 hostname/user 为 wheeltec、车端代码 9fda281、ros2 容器运行，
Humble 和 /usr/bin/colcon 存在。初期两次读取连接关闭；改为在建 SSH 前内存输入密码后成功，
未将密码写入文件或命令参数。第一次编译包装命令有引用错误，未执行远端脚本；修正后：

- ROS setup: /home/wheeltec/ros2_shared_dir/senior-robot-control/ros2/install/setup.bash。
- aarch64，GNU 11.4，Python 3.10.12，rclcpp 16.0.8，rmw_fastrtps_cpp 6.2.6。
- CMake OBJECT target receipt_check，native_receipt.cpp [100%] Built target receipt_check，exit 0。

这只证明真实 Humble API 下模板/消息/序列化代码可编译，尚未做完整 package 链接/安装或
运行 socket/writer 撤销测量。未运行生成的程序、未启动驱动/开发服务、未发送速度、未动 VM。
仍没有车体响应/制动/误差的实测保证，不称可驾驶实车验收；下一步由负责人亲审累计 diff 后
确定同版安装与有界只读运行验证/后续受控物理测量。
