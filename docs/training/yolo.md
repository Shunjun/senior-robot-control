# YOLO 语义检测：数据、训练与验收

本仓库没有提交任何训练权重，也没有训练过模型。`perception_node` 的 `weights_path` 为空或文件不存在时，检测器显式禁用（`weights_not_configured` / `weights_missing`），几何管线照常运行，动态掩码里所有射线类型为 `unknown_static`。下面的步骤用于在有数据和 GPU 的机器上产出权重，该机器不是机器人本体。

## 角色与边界

- 几何（`project_depth`）独立于检测器，以深度帧速率输出。检测器只给出 `person` / `moving` 语义。
- 检测器运行在有界、丢弃最旧帧的工作线程里。慢或失败的检测器不会阻塞几何，只会让语义过期（超过 `max_detection_age_s`，默认 0.3 s）。
- 检测结果不改写几何。人体框内与人距离差超过 `range_tolerance_m`（默认 0.5 m）的列仍是静态几何；被人遮挡的区域标为 `occluded`，不当作空闲。
- 实时安全使用全部几何点（人也是障碍）。语义只用来决定长期地图候选是否写入。

## 数据结构

```
dataset/
  scenes/
    <scene_id>/                # 一个场景 = 一次连续采集或一个地点与时段
      rgb/<timestamp_ns>.jpg
      depth/<timestamp_ns>.png # 16UC1，单位与标定里的 depth_unit_m 一致
      labels/<timestamp_ns>.txt  # YOLO 格式：class cx cy w h（归一化）
      meta.yaml                # 场景 id、日期、地点、光照、相机标定快照
  splits.yaml                  # 场景到 train / val / test 的分配
  data.yaml                    # Ultralytics 数据集描述
```

`meta.yaml` 至少记录：`scene_id`、`captured_at`、`location`、`lighting`（白天/黄昏/夜间/逆光）、`camera`（型号、分辨率、标定文件哈希）。

类别从最小集合开始：`person`、`bicycle`、`car`、`dog`。与 `TrackingConfig.person_labels` / `moving_labels` 保持一致；新增类别需要同步修改这两个集合。

## 按场景时间切分

同一场景的相邻帧高度相关，按帧随机切分会让验证集泄漏。必须按场景切分，并保证时间顺序：

1. 按 `captured_at` 对场景排序。
2. 最早的约 70% 场景进 `train`，接着约 15% 进 `val`，最晚约 15% 进 `test`。
3. 同一地点同一天的场景不拆开到不同分组。
4. `test` 只在定稿阈值后使用一次；之后再调参必须换新的 `test`。
5. 把切分写入 `splits.yaml`，并随权重一起存档。

## 训练与导出命令

训练环境独立安装检测运行时，不进入核心依赖：

```bash
python -m venv .venv-train && source .venv-train/bin/activate
pip install ultralytics numpy

yolo detect train data=dataset/data.yaml model=yolo11n.pt imgsz=640 epochs=100 \
  batch=16 seed=0 project=runs name=senior-yolo

yolo detect val model=runs/senior-yolo/weights/best.pt data=dataset/data.yaml split=test

# 机器人侧使用 PyTorch 权重；如需 ONNX/TensorRT，另行评估延迟后再启用
yolo export model=runs/senior-yolo/weights/best.pt format=onnx imgsz=640
```

机器人侧安装 `ultralytics` 与 `numpy` 后，把 `weights_path` 参数指向 `best.pt`。运行时缺失时检测器显式禁用并报告 `runtime_missing`，权重文件损坏时报告 `weights_unloadable`。

## 阈值

起始值，必须用验证集重新标定后再写入配置：

| 参数 | 起始值 | 含义 |
| --- | --- | --- |
| `min_confidence` | 0.5 | 低于该置信度的检测不进掩码 |
| `max_detection_age_s` | 0.3 | 检测与几何时间差上限，超过则语义视为不可用 |
| `range_tolerance_m` | 0.5 | 框内与目标参考距离差在此范围内的列才标为动态 |
| `max_object_speed_mps` | 2.0 | 检测框早于几何时，按“时间差 × 该速度”在角度和距离上外扩；外扩带内的列不确认为静态 |
| `motion_threshold_m` | 0.2 | 相邻观测间（基线 0.15–0.6 s）距离变化超过该值的列标为 `unknown_dynamic`，检测器禁用时同样生效 |
| `max_skew_s` | 0.03 | RGB 与深度帧时间差上限 |

标定依据：在验证集上画 precision-recall 曲线，选择在 `person` 类召回率达标前提下误检最少的置信度。漏检比误检代价高，因为漏检的人会被当作静态物写入长期地图候选。

## 验收标准

以下指标在 `test` 场景上测量，全部达标才允许启用语义；任何一项缺失则保持检测器禁用。没有实测数据之前这些项均为**未完成**。

漏检：

- `person` 召回率（IoU 0.5，距离 0.5–5 m）不低于 0.95，逆光与夜间场景单独报告，不低于 0.90。
- 连续漏检帧数：同一个人在可见范围内不得连续漏检超过 `max_detection_age_s` 对应的帧数。
- 漏检不影响实时安全：用关闭检测器的回放验证，几何点和安全停车结果与开启检测器时一致。

误检：

- `person` 精确率不低于 0.90。
- 静态物体（树、柱子、雕塑、海报人像）被标为动态的比例单独报告，并抽查其是否被长期地图丢弃。

负载（目标机器人计算平台上实测，不在开发机上推断）：

- 检测器持续运行时，几何输出频率相对检测器禁用时的下降不超过 5%。
- 检测器单帧耗时、丢弃帧数（`InferenceWorker.dropped`）、失败次数（`failures`）在 30 分钟运行中被记录；失败次数为 0 或有解释。
- CPU、内存与温度在 30 分钟内不触发降频。

Linux 与真实相机上的节点运行、`decode_depth` 对真实 `sensor_msgs/Image` 的解码、以及上述全部指标，需要在目标硬件上执行；本任务只在无 ROS 的单元测试里验证了管线逻辑。
