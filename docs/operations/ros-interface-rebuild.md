# ROS 接口删除后的定向重建

适用于 Humble 中删除或重命名消息后的构建产物恢复，不涉及 DDS 运行排障。构建成功不等于 Python 类型支持可加载；每次部署都应完成下方启动前检查。

## 故障与边界

`MotionEvidence.msg` 已在 `0f91003` 删除，当前 CMake 不再注册它。现场加载 `senior_motion_msgs_s__rosidl_typesupport_c` 时报告 `senior_motion_msgs__msg__motion_evidence__convert_from_py` 未定义，说明加载的包级注册入口仍引用已退休转换函数，而所加载转换库未提供它。`FinalCommand` 发布者也会失败，因为这些消息共用包级扩展模块。

Humble 为当前消息列表构建转换库，另为包级注册入口构建扩展模块；生成命令依赖当前 IDL/生成器，但未把生成参数 JSON 本身列入文件依赖。仅减少消息列表可能留下旧入口/产物；安装也不是对旧安装目录的同步删除。此机制可造成新旧产物混合，但现场尚未核对备份中的生成 C 文件和动态库解析路径，不能仅凭异常断定是哪个文件残留。另一个可能来源是 `PYTHONPATH`、`LD_LIBRARY_PATH` 或 underlay 选中了不同版本。依据：[Humble 生成与链接规则](https://github.com/ros2/rosidl_python/blob/humble/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake)、[生成命令依赖](https://github.com/ros2/rosidl_python/blob/humble/rosidl_generator_py/cmake/custom_command.cmake)。

## 默认 isolated install 的恢复

先由操作者停止实际管理该实例的服务并确认退出；保持停止直到检查通过。本页不提供自动启动命令。命令在实际运行 ROS 的环境执行：Docker 部署应在原容器内执行，不在宿主机另建一份。

使用未加载本项目 overlay 的干净 shell，只加载 `/opt/ros/humble/setup.bash` 和现场必需的厂商 underlay。旧 shell 中再次 source 不会移除旧 `PYTHONPATH`/`LD_LIBRARY_PATH` 条目；不要加载备份的 setup 文件。确认源码已更新到待部署版本，复用现有 `.ros-venv`，无需再次 pip 安装。

下列命令要求默认 `build/`、`install/` 路径及 isolated 布局；自定义 base 路径需对应调整。执行前，将临时 shell 变量 `ROS_INTERFACE_BACKUP_DIR` 设为操作者选定的工作空间外备份目录；这不是产品配置。命令不移动或删除任何既有备份。

```bash
cd /home/wheeltec/ros2_shared_dir/senior-robot-control/ros2
(
  set -e
  test "$(cat install/.colcon_install_layout)" = isolated
  test -d build/senior_motion_msgs
  test -d install/senior_motion_msgs
  : "${ROS_INTERFACE_BACKUP_DIR:?请先指定工作空间外的备份目录}"
  mkdir -p "$ROS_INTERFACE_BACKUP_DIR"
  msgs_backup=$(mktemp -d "$ROS_INTERFACE_BACKUP_DIR/senior-msgs-rebuild.XXXXXX")
  printf '备份位置：%s\n' "$msgs_backup"
  mv build/senior_motion_msgs "$msgs_backup/build-senior_motion_msgs"
  mv install/senior_motion_msgs "$msgs_backup/install-senior_motion_msgs"
  ../.ros-venv/bin/python -m colcon build --packages-up-to senior_robot \
    --cmake-force-configure --cmake-args -DBUILD_TESTING=OFF -Usenior_motion_msgs_DIR
)
```

只有上述命令成功才继续。两个目录必须一起移走：只清安装目录可能重新安装旧生成物，只清 build 会保留已不再安装的旧消息文件。重新配置下游并移除其缓存的 `senior_motion_msgs_DIR` 可避免沿用错误包位置；其余包无需全量清理。若是 `--merge-install`，不能套用单包 install 目录移动法，应选新的 build/install base 做同版重建并审阅切换路径，不在共享 install 下按模糊匹配删除文件。

## 启动前检查（不创建 ROS 节点）

在同一 ROS 工作目录、使用与服务相同的解释器和 underlay 执行：

```bash
source install/local_setup.bash
../.ros-venv/bin/python - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from rosidl_generator_py import import_type_support
from rclpy.type_support import check_for_type_support
import senior_motion_msgs.msg as messages

expected = Path('install/senior_motion_msgs').resolve()
actual = Path(get_package_prefix('senior_motion_msgs')).resolve()
if actual != expected:
    raise RuntimeError(f'包索引来源错误: {actual}, 预期 {expected}')
support = import_type_support('senior_motion_msgs')
print('Python 消息路径:', messages.__file__)
print('类型支持扩展路径:', support.__file__)
for module in (messages, support):
    # 保留安装入口路径；symlink-install 的真实目标可能位于 build 中。
    if expected not in Path(module.__file__).absolute().parents:
        raise RuntimeError(f'Python 模块来自其他 overlay: {module.__file__}')
sources = sorted(Path('src/senior_motion_msgs/msg').glob('*.msg'))
if not sources:
    raise RuntimeError('当前目录没有消息源码，请在 ROS 工作空间执行')
for source in sources:
    check_for_type_support(getattr(messages, source.stem))
if hasattr(messages, 'MotionEvidence'):
    raise RuntimeError('仍导出已退休 MotionEvidence')
print('当前消息类型支持加载通过；未创建节点或发布者')
PY
```

普通 `from senior_motion_msgs.msg import FinalCommand` 可能只加载 Python 类，不能替代扩展模块加载检查。上述检查覆盖当前包所有消息的类型支持，不调用 `rclpy.init()`，也不代表节点、DDS 或车辆运行验收。

若仍报同名符号错误，保留异常与备份，不重复安装 Python 依赖。核对扩展路径，并对异常中实际加载的 `.so` 执行 `ldd <实际扩展路径>`，检查 `libsenior_motion_msgs__rosidl_generator_py.so` 等是否解析到当前 `install/senior_motion_msgs/lib`（symlink-install 可指向当前 build），不得来自其他 overlay 或备份。检查备份生成入口 `build-senior_motion_msgs/rosidl_generator_py/senior_motion_msgs/_senior_motion_msgs_s.ep.rosidl_typesupport_c.c` 是否仍有 `motion_evidence`，可区分旧生成入口与库路径混用；无需启动 ROS。

## 其他安装残留

本次双目录重建也会移除消息包中的旧 Python 模块、IDL、头文件、typesupport 库以及前次头文件修复遗留的平铺 include 文件。其他包的旧文件不会自动移除：例如 `senior_robot` 曾删除的 `simulation_motion_flat_lateral.yaml`、`simulation_motion_rear_mask.yaml` 可能仍在其安装目录，但它们不提供本次缺失符号。若部署还要求清除已退休资源，应单独审阅对应包的删除清单，再定向备份该包 build/install 后重建，勿把此次符号恢复扩大成全量清理。

今后接口列表删除/重命名时，发布步骤应包含该接口包双目录备份重建、下游重建和上述加载检查；不以 `--cmake-clean-cache` 或单次编译成功替代。当前说明基于上游源码与现场异常分析，未在本机执行 Humble 构建或类型支持加载。
