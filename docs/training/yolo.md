# YOLO 语义检测：采集、训练与验收

仓库没有模型权重，也没有已完成的训练或召回率、延迟、负载测试。几何障碍必须独立于语义参与保护；人或车辆的检测结果不能擦除深度命中或把被遮挡区域变成空闲。

目前 `depth_ingress` 产生带原始时间和标定身份的深度采样，必需深度的扫掠体积覆盖尚未资格化，因此安全门保持停车。`semantic_ingress` 将经过显式配准资格检查的 ROS RGB/CameraInfo 与有界深度采样按原始时间配对，使用 `SpawnInferenceWorker` 独立 spawn 子进程、一个待处理帧和有界历史生成带原始身份的候选；权重缺失明确禁用。该接线已在 Linux ROS 中验证有界消息和受控处理器的正反路径；尚未连接激光匹配参考或稳定地图。线程版旧 `perception_node` 不是已验收的运行接线。异步结果只能用于其原始 RGB、深度和标定身份；缺失或空的识别批次不能证明场景静态。

## 采集和标注

在拟部署的相机、安装位置和机器人高度下采集原始 RGB、原始深度、CameraInfo、帧 ID、采集时间、首次接收时间和测量时 TF，并记录标定版本、场景/时间、光照、人员与障碍运动状态。保留实际的 `16UC1` 或 `32FC1` 编码、步长、字节序与单位；无效深度和遮挡必须保留为未知。覆盖近远距离、不同身高与姿态、遮挡、轮椅/推车、低箱、自车遮挡、强光/暗光，以及机器人移动但场景静止的负例。对可识别类逐帧标注框，并抽样复核漏标、标签一致性和 RGB/深度实际配准；任何个人数据按现场规则处理。

建议按采集批次保存数据：

```text
dataset/
  scenes/<scene_id>/
    rgb/<capture_ns>.jpg
    depth/<capture_ns>.*       # 保留原始编码和元数据，不强转单位
    labels/<capture_ns>.txt    # YOLO：class cx cy w h，归一化
    metadata.yaml              # 原始时间、帧、标定、场景与采集条件
  splits.yaml
  data.yaml
```

类别应与实际模型及待实现的语义关联一致。不要把一个类别未检出等同于目标不存在。

## 场景和时间切分

相邻帧高度相关，按帧随机划分会泄漏验证数据。按地点、采集批次和时间整体分组后分配 train、validation、test；同地点同一连续场景或同一时段不跨组。可以从约 70% / 15% / 15% 场景起步，检查各类距离、遮挡、光照、机器人运动和动态场景的代表性，再记录最终分配、场景清单与数据哈希。阈值只在 validation 上选择，最终 test 不参与调参；修改阈值后应使用新的独立测试集复验。

## 可选运行时与训练命令

当前 spawn 组件只接受 [Ultralytics 8.3.40](https://pypi.org/project/ultralytics/8.3.40/)；其他版本会报告 `runtime_version_unsupported`。该版本是明确的接口基线。独立 Linux CPU 环境已使用 Ultralytics 8.3.40、PyTorch 2.14.1+cpu、torchvision 0.29.2+cpu、OpenCV 4.11.0.86 和 NumPy 1.26.4，先配合本地随机未训练权重验证 RGB 与单通道图像的推理接口，再以官方预训练 YOLO11n 和公开街景验证到人员/车辆识别及独立推理进程；这些模型探针使用受控时间戳；子进程探针使用公开图像与受控采集时间、空几何，并未经过实时 ROS 相机或配准。它们不等同现场数据集上的训练质量、召回率、持续实时负载或目标机器人验收。核心与云端不需要此依赖。部署者自行核对 Ultralytics、PyTorch、模型及权重许可，准备本地权重文件；缺失权重不会触发组件下载。安装、训练和导出由独立环境的操作者执行：

```bash
python -m venv .venv-train
source .venv-train/bin/activate
# 先从官方 CPU wheel 索引安装，避免 ARM 环境默认解析到 NVIDIA 依赖。
python -m pip install 'torch==2.14.1+cpu' 'torchvision==0.29.2+cpu' \
  --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'ultralytics==8.3.40' 'numpy==1.26.4' 'opencv-python==4.11.0.86'
python -m pip freeze > requirements-training-lock.txt

# 使用已取得许可的本地初始权重；不要使用会自动下载的模型别名。
test -r /path/to/local/seed.pt
yolo detect train data=dataset/data.yaml model=/path/to/local/seed.pt imgsz=640 \
  epochs=100 batch=16 seed=0 project=runs name=senior-yolo

yolo detect val model=runs/senior-yolo/weights/best.pt data=dataset/data.yaml split=val
# 固定模型和阈值后才使用独立 test。
yolo detect val model=runs/senior-yolo/weights/best.pt data=dataset/data.yaml split=test
yolo export model=runs/senior-yolo/weights/best.pt format=onnx imgsz=640
```

训练参数只是起点，不代表适合目标设备。随权重保存训练配置、数据切分、依赖锁、许可证、版本和文件哈希。当前可选 Python 推理路径使用本地 PyTorch 权重；导出的 ONNX 是待评估产物，本项目尚未接入 ONNX/TensorRT 推理。语义节点支持通过 `weights_path` 和 `device` 参数选择本地权重及设备；空权重路径保持禁用。仿真配置的 `registered_profile` 只用于 Fortress 诊断，并非实车配准证明；现场必须先独立校验配准。候选的后续语义消费尚待接线。

## 阈值与验收计划

在 validation 集上按类别和场景计算 precision/recall 曲线，优先检查人员漏检、遮挡和小目标；记录所选置信度、IoU、距离、目标尺寸及每类漏检。可把人员 IoU 0.5 召回率至少 0.95 作为待验证目标，并分别报告夜间、逆光和遮挡子集；这不是现有性能声明。冻结参数后在独立 test 集报告完整结果和置信区间。

验收必须覆盖以下真实接线，而不是只检查模型输出：

- 同一来源的 RGB/depth 配准、原始时间/epoch 和 3-D 目标深度关联；合法迟到结果只作用于对应历史帧和相应激光参考，不能涂到当前帧或擦除后方墙面。
- 人/可移动物仍是实时几何障碍；空批次、漏检、过期、未配置权重和失败不构成已确认静态或自由空间。机器人移动但静态墙不被原始列差误判为动态。
- 在目标 CPU 和经配置的加速设备上，分别记录加载、单帧与尾部延迟、吞吐、超时、丢帧、内存、CPU/GPU 使用率和温度；模拟慢、卡住及崩溃推理，并同时测量几何和安全门频率与停车延迟。几何/控制不得等待识别进程。
- 现场复测真实相机盲区、深度无效、自车/人员遮挡、低矮障碍和覆盖不足停车；稳定占据地图、导航、远程控制与硬件验收需各自完成，不能由模型指标代替。
