# 视频资源按需启动与 15 fps 优化交接

原始交接时间：2026-11-21。用户要求交接到可正常访问本机局域网、支持相应执行审批的 Codex 环境。本文件是接续记录，不代表代码已经实现或车端已经验收。

> 下方原始交接用于保留历史证据。当前需求、实现及验证状态见文末“接续实施记录”，其中包含用户后续调整。

## 接续任务

1. 先连接车端，测量当前实际图像帧率、视频发送帧率和 CPU 开销，读取 CPU/GPU 与编码设备信息，再判断硬件编码能否接入。
2. 网页视频预览最高保留 15 fps；在昂贵的像素转换/编码之前限制处理频率，继续保留最新帧，保持真实采集时间对应的 RTP 时间轴。
3. 只有在车辆控制页切换到视频预览时，车端才申请并启动相关视频组件；离开视频预览、离开控制页、隐藏页面或连接关闭时释放预览资源。
4. 相机深度、红外、点云、JPEG/深度压缩等处理将来会有消费者，用户要求按需求启动，而不是直接删除这些能力。预览只申请彩色图像；共享相机必须考虑多个消费者，退出视频预览不能关闭其他消费者仍在使用的资源。
5. 完成相关本地测试和构建，明确哪些车端数据实测过，哪些被环境限制。不要用本机编码数据替代实车结果。

用户已明确授权上述优化。尚未编写产品实现或测试，尚未改成 15 fps，也未启用硬件编码。

## 仓库与分支

- 项目：`/Users/zhangshunjun/Wheel 机器人/senior-robot-control`。
- 最近确认分支：`feat/vehicle-architecture-refactor`，跟踪同名 origin 分支，ahead 1。
- 最近确认 HEAD：`24e0ba7 fix: align mapping with deployed slam toolbox 2.6.8`。
- 前一提交：`f564273 refactor: finish functional package migration and remove root src`。
- 用户要求继续使用原重构分支，不要另起分支替代功能主分支。开始前重新核查工作区及 HEAD，保留用户随后发生的改动。
- 此交接文件新建前工作区干净。根 `src/` 已删除，代码在对应功能包及根 `vehicle_core/`。
- 按用户要求本人直接修改，不使用 Orca 派发。修改前先核查当前 AGENTS 和代码，汇报文件范围及数量。

## 车端连接与阻塞情况

- SSH：`wheeltec@wheeltec.local`。用户要求后续统一使用 hostname。
- 项目：`/home/wheeltec/ros2_shared_dir/senior-robot-control`。
- 运行配置：`/home/wheeltec/ros2_shared_dir/senior-control`。
- 用户在 Mac 上实测 hostname 解析到 `10.0.0.102`，ping 三次正常，约 3.5–4.8 ms。这个 IP 仅是本次观测，不应固化为连接记录。
- 用户此前从车端 ping 自己得到 `172.17.0.1`，不能拿它当 Mac 访问车端的局域网地址。
- 本会话 SSH 无法解析 `.local`；直接测试用户提供的 `10.0.0.102:22` 返回 `Operation not permitted`；Bonjour 查询返回 `Service Not Running`。沙箱外执行申请也被当前权限策略拒绝（sandbox_approval/request_permissions 禁用）。
- 因此本会话没有连接成功，尚未读取当前硬件、实际帧率或实际 CPU。接续环境先确认自己的真实权限，通过正常审批机制连接；若仍受限制，如实报告，不绕过限制。
- `/Users/zhangshunjun/Wheel 机器人/CURRENT_TASK.md` 第 153 行已更新 hostname。
- 检查以只读为先，不发送运动命令；不要为了测视频直接重启整车服务，已有启动流程可能触发学习前进。

## 当前视频实现及问题

实车：ROS raw Image → 最新帧槽 → AV 图像 → 软件编码 → WebRTC → 网页。仿真使用文件最新帧槽，不能把仿真文件读写算成实车视频开销。

- `senior_control/src/vehicle_manager/media/stream.py`
  - `SERVER_CODECS = ("VP8", "H264")`，默认优先 VP8。
  - `VideoLimits` 目前没有发送帧率上限。
  - `VideoSession._read_loop/_send_loop` 仅保留最新帧。
  - `_make_track().recv()` 把真实采集时间差换算为 90 kHz PTS；之前修过固定 30 fps 时间戳导致延迟的问题，不要回退。
  - `_to_av_frame()` 在 stride 对齐时整块更新 AV plane；640×480 rgb8 的正常 stride 为 1920，已走快路径，不应把逐行补齐当成当前主要瓶颈。
  - `_EncodeLoop` 是独立线程中的事件循环，不是硬件编码，也不是独立进程；factory 当前首次使用后保留到 Agent 退出。
- `senior_control/src/vehicle_manager/media/signaling.py`
  - 认证后的 video.offer 创建 session；video.close、连接退役等取消任务，并在 finally 关闭 session。
  - 需核对快速切换、协商中取消与下一次预览重开的资源所有权。
- `senior_control/src/vehicle_manager/agent/production.py::_media_binding`
  - 按会话创建 ROS camera source，session.close 时关闭订阅。
  - factory 懒创建，但目前在整个 media binding 关闭时才销毁。
- `ros2/src/senior_robot/senior_robot/ros_video.py`
  - ROS subscriber 在线程中按需求创建，QoS depth=1、BEST_EFFORT；只保留最新帧。
  - 处理 rgb8/bgr8、最大 640×480，复制 ROS 消息数据；next_frame 轮询间隔最高 25 ms。
- `senior_control/src/vehicle_manager/service.py::supervise/start_local`
  - 目前运动适配器就绪后，无论网页是否在看视频，都会启动相机进程。
  - 相机由 service 持有并参与退出清理；不能仅修改网页卸载 VideoPanel 就宣称相机已停止。
- `ros2/src/senior_robot/senior_robot/platform_runtime.py::_camera_command`
  - 调用厂家 `turn_on_wheeltec_robot/wheeltec_camera.launch.py`。
  - `_video_environment` 启动时探测 ROS Python 环境并设置 `/camera/color/image_raw`。
- `web/src/features/devices/ControlSurface.tsx`
  - 手动控制默认 view="video"，导航默认 map；当前视频 view 才挂载 VideoPanel。
  - 需要按用户要求核对默认视图，避免只是进入控制页就申请视频。
- `web/src/features/video/VideoPanel.tsx`
  - live + mount 时自动 createOffer；cleanup 关闭 peer 并发送 video.close。
  - 当前没有 document.visibilityState / pagehide 的资源管理。

## 厂家相机启动配置（仅本地资料，不代表车端实时版本）

资料文件：
`/Users/zhangshunjun/Wheel 机器人/1.WHEELTEC ROS机器人通用资料/7.ROS源码/ROS2源码/Humble源码/src/turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py`

- 按 camera_modes_yaml 中的 camera_mode 选择 Astra/大白/Gemini 或 usb_cam。
- Astra 类 include 时传入 `enable_d2c_viewer=True`。
- 另外启动 raw→compressed 彩色 JPEG（quality=85）和 raw→compressedDepth 两个 republish。
- 本地 Astra Pro/Pro Plus XML 默认彩色 640×480@30、depth/IR 30 fps，并有 enable_point_cloud 等开关。
- 这些只是默认配置。实际车端相机型号、驱动版本、启用参数及订阅者尚未核查；发布器存在不证明一直在计算（可能按订阅懒执行）。

## 本机编码小测试（不能作为车端结论）

本机会话安装的 aiortc 代码中：VP8 使用 libvpx，H264 使用 libx264。将协商顺序改成 H264 不会自动接入硬件编码。

640×480 合成移动纹理、约 1 Mbps、编码预热 3 帧后统计 20 帧：

| 环节 | 平均 wall ms/帧 | 平均 process CPU ms/帧 |
| --- | ---: | ---: |
| RGB→AV 拷贝 | 0.039 | 0.032 |
| VP8 编码及打包 | 3.089 | 3.389 |
| H264 编码及打包 | 2.862 | 11.727 |

测试仅说明在该本机样本中不能假定软件 H264 更省 CPU；不支持推算车端节省比例。

## 可复用的验证位置

- `tests/integration/test_video.py`：会话、信令、关闭、拥塞、真实 aiortc 测试。
- `web/src/features/video/video.test.tsx`：预览协商与连接状态。
- `web/src/features/devices/devices.test.tsx`：控制页视图与场景切换。
- `senior_control/tests/test_docker_start_helper.py`：实际相机/服务子进程生命周期。
- `ros2/tests/test_launch_recovery.py`：真实进程回收和退出行为。
- `web/package.json`：`test`、`typecheck`、`build`。

建议验证：预览 15 fps 且跳过帧不压缩时间轴；没有需求不启动；预览退出后释放；多个消费者共享时只释放自己的需求；协商中关闭/重开不泄漏进程；隐藏/恢复页面只建立有效的新会话；硬件编码若不可用保留明确的软件路径。优先复用已有测试，不机械增加测试数量。

本轮还没有修改以上产品代码。接续环境应先实测，再确定硬件编码方案，不能宣称已优化或已部署。

## 接续实施记录（代码已实现，尚未部署验收）

### 用户后续确定的行为

- 保留 `feat/vehicle-architecture-refactor`。工作区曾被外部切到 `main`，发现后停止写入；之后已回到原功能分支。接续代码基线为 `67e097dd48472f6a237b0b0374e51d414d56f9bb`，包含原 PATH 修复。
- 第一个消费者申请时，按车端厂家配置整体启动相机采集；有消费者期间保持同一采集进程，不随消费者变化重启。最后一个消费者释放后关闭。
- JPEG 和深度压缩独立按需求启动；网页 WebRTC 只申请彩色，不启动两路压缩。
- 用户进一步要求：车端 JPEG、深度压缩也最多 15 fps，在进入 image_transport 编码器前使用原始序列化 Image 的最新帧槽限流；不通过编码后丢帧限流。
- WebRTC 同样最多 15 fps，限流在 AV 转换前；保留采集时间对应的 90 kHz RTP 时间轴。
- 初始控制页显示地图；显式切到视频才申请预览。隐藏页面、pagehide、离开视频/控制页、断开连接均释放该预览；恢复页面建立新的会话。

### 车端已实际读取的事实

使用 hostname `wheeltec.local`，登录后在现有 `ros2` 容器内只读检查；未修改车端配置、未重启现有服务、未发送运动命令。

| 项目 | 实际观测 |
| --- | --- |
| 主机 | Raspberry Pi 5 Model B Rev 1.1；aarch64；四核 Cortex-A76，最高 2.4 GHz |
| 图形/视频设备 | DRM card0/card1/renderD128；video19 为 rpivid；其余 video20–37 为 pispbe 图像处理设备 |
| 相机配置 | 厂家 `wheeltec_param.yaml` 的 `camera_mode: astra` |
| 彩色、深度、红外参数 | 各 640×480、配置 30 fps；enable_color/depth/ir 均为 true |
| 点云 | enable_point_cloud=true；enable_colored_point_cloud=false |
| 其他实时参数 | use_uvc_camera=false；depth_registration=false；color_depth_synchronization=false；publish_tf=true，10 Hz；queue_size=5；use_sim_time=false |
| 通信 | rmw_fastrtps_cpp；当前内核 rmem/wmem max/default 均为 212992 字节 |

相机曝光、SDK 精确设备型号及红外实际无帧的原因尚未核实。参数启用不等于实际有输出：探针没有收到红外帧。车端安装的 `astra.launch.xml` 和 `wheeltec_camera.launch.py` 均已读取，不能把早先仅本地资料的结论冒充实时参数。

树莓派官方说明 Pi 5 使用软件视频编码；本轮不接入虚假的硬件 H264 路径、不因 H264 协商优先就宣称硬件加速。参考：[官方相机软件说明](https://www.raspberrypi.com/documentation/accessories/camera-software.html)。

### 实测帧率与 CPU：不同窗口不得混用

**重要更正：早先约 11.1 fps 是原始图像探针的接收帧率，不能称为相机采集能力上限。** 同一时段 JPEG 输出可接近 30 fps，说明原始订阅链路有明显的接收/调度差异。

- 首次 5 秒原图探针：55 帧，约 11.12 fps 接收，时间戳折算约 11.11 fps。
- 一次 6 秒无新增图像订阅的 CPU 采样：相机约 76.9%，彩色 JPEG 约 34.8%，深度压缩约 29.6%。这里 100% 为单核，四核总量为 400%；不能把单进程 30% 当成整机 30%。
- 另一同窗口多路采样：彩色原图约 8.34 fps、深度原图约 14.13 fps、JPEG 约 28.53 fps；相机约 76.8%、两路压缩约 33.3%/33.1%。探针自身约 13.6% 单核 CPU，属于测量干扰，不能算入产品开销。
- 两次顺序的 8 秒 QoS 对比：BEST_EFFORT 原图约 5.12 fps、JPEG 约 25.22 fps；RELIABLE 原图约 9.73 fps、JPEG 约 24.74 fps。原图最大时间戳间隔分别约 0.888 秒、0.385 秒。它们说明接收链路差异，不足以证明某一个内核参数就是唯一根因；本轮未擅自修改 sysctl 或 DDS 配置。
- 旧代码的车端同机 WebRTC 回环：6 秒内 source 收到 39 帧、解码 33 帧，首末帧折算约 5.83 fps。测量进程 CPU 约 24.97% 单核，包含 ROS、发送编码、接收解码，不能作为浏览器链路或纯编码 CPU。
- 旧链路的深度压缩进程确实在耗 CPU，但探针在标准 compressedDepth 话题上未发现其输出。此现象未确诊，不推定它有稳定 30 fps 输出。

### 实现与资源契约

- `vehicle_manager.media.camera_demand.CameraDemand` 在现有 service 私有 IPC 上管理消费者 token；每 3 秒续租，12 秒未续租时回收遗留需求。最后释放触发关闭，最终 SIGINT/KILL 使用原有的进程组回收边界。
- 请求类型为 `camera.acquire`、`camera.renew`、`camera.release`。acquire 带 token 和 capabilities 列表；支持 color/depth/ir/point_cloud/jpeg/depth_compressed。任意需求都共享 capture；只有显式 jpeg/depth_compressed 需求才启动对应限流及压缩节点。
- 后续应用消费者须使用此租约接口；任意外部 ROS 订阅不会自动登记为相机需求，不能通过相机内部点云订阅数来判断用户消费者是否仍存在。
- `senior_robot/launch/camera_demand.launch.py` 复用厂家相机型号选择及完整采集参数；分别启动 capture、jpeg、depth_compressed。两路压缩前各有 `camera_rate_limit`，原始序列化载荷原样转发，像素编码最多 15 fps。
- `production._media_binding` 按预览拥有 source、租约、编码 factory，退出时全部关闭；仿真仍使用文件源，不申请实车相机租约。
- 关闭中的旧会话与新会话使用各自候选队列，避免旧 finally 清掉新 viewer；新会话等待旧任务清理。编码事件循环也关闭其默认 executor。

### 验证范围和已知限制

批准的新增/实质改写需求场景共 **10 项**：预览限流与 RTP 时间戳、无需求/末次释放、多消费者与独立压缩、遗留租约到期、production source/租约/编码线程回收、编码前序列化限流、网页隐藏/恢复、默认地图/视频切换、急停 epoch 断言适配默认地图的额外请求、实际相机叶子恢复测试改为显式申请需求。

另有 **1 项实际 bug 定向回归**：旧会话清理尚未完成时，重新打开预览被拒绝为 `video.close`；已观察到原实现失败，修改后通过。保留原认证、拥塞、ICE、真实 aiortc 回环和进程组退出回归，不以 mock 替代所有链路验证。

已执行的命令及结果以本次代码最终检查记录为准：

- `npm test -- --run`（web）：180 passed。
- `npm run build`（web，含 TypeScript 检查）：通过。
- `.venv/bin/ruff check .`：通过。
- 本次改动 Python 文件的 `ruff format --check`：通过。全仓库格式检查另有原有 `docs/superpowers/plans/2026-11-21-safety-input-efficiency.md` 中代码块未格式化，不擅自改其他任务文档。
- 后端/进程/aiortc 定向集：`pytest senior_control/tests/test_camera_demand.py senior_control/tests/test_docker_start_helper.py tests/integration/test_video.py ros2/tests/test_launch_recovery.py -q`。最终解除 socket 限制后 61 passed、2 skipped、1 failed；失败为未修改的 `test_recovery_handoff_restores_original_estop_revision_without_reset`，其仍 monkeypatch 已不存在的 `senior_robot.vehicle_learning.LearningClient`。该最终结果包含新增压缩限流，不排除或隐瞒此旧失败。
- 新增三个独立模块的定向 mypy 通过；宽范围 mypy 暴露依赖包缺少 py.typed 及既有任务模块错误（66 个错误），未把它称为全仓库类型检查通过。
- 车端首轮隔离话题 + 30 fps 合成 640×480 RGB/16UC1 图像：实际跑通限流节点 → image_transport JPEG 与 compressedDepth 两条链路，输出分别收到 48/51 帧，输出时间戳折算约 5.42/9.15 fps。测试进程均在 finally 中回收，临时目录清理。它验证格式与 ROS API 能工作，不能证明整车在当前满载条件下稳定 15 fps，也没有证明真实画面 CPU 节省比例。
- 最终限流节点改为与厂家原始发布器/image_transport 一致的 RELIABLE 输入，并处理正常退出信号。复测已发出，但 SSH 随后 `No route to host` 断开，未取得结果，不能引用首轮结果当作最终版本验收。

尚未部署最终代码、未重启现有车端服务、未做真实浏览器→实际相机启停的现场验收。需要在车辆可连接时构建新的 senior_robot launch/console script，安排不会触发不受控启动学习运动的验收，再测匹配窗口的真实画面 fps 与 CPU。相机采集 30 fps 并不保证现有原始 ROS 订阅可收到 30 fps；这一接收瓶颈仍需单独诊断，不能靠“15 fps 上限”宣称已经解决。

补充构建验证：`uv build --cache-dir /tmp/video-uv-cache ros2/src/senior_robot --out-dir /tmp/video-wheels` 已成功生成 sdist 与 wheel；`uv build --offline --cache-dir /tmp/video-uv-cache senior_control --out-dir /tmp/video-wheels` 也已成功。已解包检查 ROS wheel 包含 camera_rate_limit.py，manager wheel 包含 camera_demand.py 与 camera_lease.py。

最终修改范围为 20 个文件：产品代码 12 个（新增 4、修改 8）、测试 5 个（新增 1、修改 4）、ROS 安装注册配置 1 个、文档 2 个。负责人已亲自阅读产品 diff 与新增文件，保留共享采集、独立压缩、租约清理及过期、会话隔离、编码前限流与时间轴的验证记录；因现场验收待完成，按阶段成果交付 Draft PR。


## 2026-11-22 ROS 归属清理后续

本轮已按获批归属迁移旧包及 robot 转发入口，采用分批提交。相机 subscriber 与压缩限帧器现位于 senior_sensor_ingress，感知节点归 senior_perception；共享采集与按需15fps压缩策略保持。实际视频回环和相机 lease/IPC 回归48项通过。当前包清单、干净重建方式见 [ROS 包归属](../../ros2/PACKAGE_OWNERSHIP.md)，验证与现存失败见 [清理测试记录](../testing/2026-11-22-ros-package-cleanup.md)。本机 ROS 原生构建和车端部署仍未验收，PR保持 Draft。
