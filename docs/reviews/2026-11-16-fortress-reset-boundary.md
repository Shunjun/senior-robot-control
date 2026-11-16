# 固定Fortress reset执行边界：已实现，待目标构建核验

负责人已批准仅为真实已执行reset通知扩展固定Gazebo core。本次没有修改物理算法、调度或控制模型，没有启动仿真、ROS或车辆。

官方 `gazebosim/gz-sim` tag `ignition-gazebo6_6.17.0`、commit `d9b18d85a17400d32494bca4c3589f519aca7e81` 的 `SimulationRunner::UpdateCurrentInfo()`，在 `requestedRewind` 分支把dt设为负的原simTime，并将simTime/iterations归零。原薄插件无法识别0→0 reset，也没有公共reset执行回调；不能订阅控制请求猜执行成功。

现在仅在上述已执行分支、后续producer回调之前调用共享库 `reset_simulation_epoch()`。库在同一mutex临界区更新该world generation并设置seen=false；第一份0 tick不会再次递增，连续reset各递增一次，暂停同tick不换代。

源码核对还证实，`EntityComponentManager` 的entityCount是各ECM实例成员，多世界可同时具有world entity #1。因此core和所有生产插件统一调用 `simulation_world_key(ECM地址, world entity)`，由同一个共享库返回该ECM生命周期内一致的进程内key；diff-drive沿现有状态接口的 `source_clock/world_key` 透传。没有新增wire身份、第二套producer epoch或接收端猜测。measurement_epoch仍由共享world时钟产生，source_epoch/sequence仍由各producer独立维护。固定Server为每个world创建ECM，registry按实际ECM实例隔离；不拿跨进程地址作为消息时钟域。

## 同一个动态库实例

`simulation_timing` 是 `senior_motion_msgs` 0.2.0 的独立SHARED目标，Linux SONAME为 `libsimulation_timing.so.0`，状态仅定义在cpp匿名namespace；头文件没有inline状态副本。公开world/reset/epoch函数有显式符号可见性，库默认hidden。core与三个插件均链接同一导出target；CMake拒绝把它当静态库，生成实际目标路径供构建后核查。

Linux构建脚本在完成后对core和三个插件运行ldd，要求它们解析到CMake记录的同一真实共享库路径，并检查公开符号。这个静态链接解析证据不等于Gazebo实际加载验证，目标部署仍须核验loader路径、SDK/插件ABI以及没有另一份旧同名库抢先加载。

本机真实C++机制验证分别编译共享状态库、一个core适配DSO、两个producer适配DSO，再由真实动态加载器加载；测试通过跨DSO一致key/epoch、重复entity数字的多ECM隔离、0→0、非零→0只换一次、连续reset、并发32次reset无丢失、暂停、producer适配重新引用共享库。另一个实际DriverClock harness验证producer source_epoch独立且不改变world域。适配DSO仅验证链接边界，**没有冒称编译完整Gazebo engine**。

## 可复现外部构建

补丁仍只有 `SimulationRunner.cc` 的一处include和一处reset调用；`core.CMakeLists.txt` 使用固定上游core的34份源文件和必要私有头/protobuf闭包（59份原始文件，包含原始构建声明），公开/生成SDK头和声明依赖来自确切6.17.0安装。保留原protobuf的msgs子目录生成路径，不构建GUI、工具、测试或其他system。仓库只维护小patch、构建适配与不可变SHA manifest，不维护整份engine源码。

prepare脚本先校验所有原始文件、patch及两个构建适配SHA，零fuzz应用后再次校验；新overlay依次构建sensor库、必要core、三个system、ROS桥及控制插件。未在本macOS进行完整Humble构建、SDK/ament消息生成、真实DDS、Gazebo运行或目标ABI验收；这些明确列为原Draft PR26部署前置。
