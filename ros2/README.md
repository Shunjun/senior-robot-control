# Humble / Fortress simulation (runtime validation pending)

| OS | ROS | Gazebo | integration |
| --- | --- | --- | --- |
| Ubuntu 22.04 | Humble | Fortress | `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` Humble binaries; not yet tested here |
| macOS | none | none | portable tests only; not a simulation validation |

After installing ROS Humble and Fortress from their official repositories on Ubuntu 22.04, start in the repository root:

```sh
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
rosdep install --from-paths ros2/src --ignore-src -r -y --rosdistro humble
cd ros2
colcon build --packages-select senior_sim senior_robot
source install/setup.bash
ros2 launch senior_sim simulation.launch.py headless:=true use_sim_time:=true
```

`world:=/absolute/path/to/world.sdf` and `headless:=false` select another world or GUI. Run the following in another sourced terminal at the repository root, with the isolated simulation already running:

```sh
cd /path/to/senior-robot-control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
/usr/bin/python3.10 -m pytest tests/ros/test_sim_topics.py -q
```

The tests move the simulated robot and temporarily relocate it through the Fortress UserCommands service, restoring its pose afterward; run them only in an isolated baseline simulation. They require the `ign` executable and its `/world/baseline/set_pose` service. A skipped test, a successful static test, or a build alone does not establish the ROS runtime contract. Install the core Python wheel separately into the ROS interpreter when using it; `rclpy` is supplied by ROS, not PyPI.

The chassis is a conservative primitive using vendor senior_4wd_dl wheel positions, 0.076 m wheel radius, body bounding box 0.550 × 0.449 × 0.15425 m and navigation footprint ±0.337 × ±0.27 m. A rear attachment makes lidar occlusion physical. The collision envelope retains the full conservative footprint; visual boxes include a front camera aperture so the forward ray is unobstructed. Camera position is left at the supplied vendor reference, whose parent frame still requires verification. A static wall gives sensors an observable target. Vendor sensor transform values are approximations requiring validation against actual frame parentage. Original vendor assets are not copied. The truth publisher reads simulator pose independently; its TF topic is deliberately not bridged. Do not use truth for production localization or present simulated data as hardware data.

The official `diff_drive_controller` uses feedback (`open_loop: false`) and `cmd_vel_timeout: 0.3` independent of the later safety process. Its clock follows simulation time: a pause can prevent timeout and permit stale commands on resume. A monotonic safety gate must prevent this before motion is relied upon. Controller loading, ROS frame IDs, RGB-D topic names, graphics support in headless mode, geometry contact, stop timing, and installed binary compatibility require the Linux runtime gate. Hardware profile is illustrative only and must be verified on the robot; it grants no motion permission.

Sources: [Humble/Fortress control compatibility](https://github.com/ros-controls/gz_ros2_control/tree/humble), [controller timeout and feedback](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), [Fortress ROS bridge](https://gazebosim.org/docs/fortress/ros2_integration/), [simulator odometry publisher](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1OdometryPublisher.html).

`python3 tools/detect_environment.py` reports sourced ament package manifests and versions, missing prerequisites, and passive topic discovery. Its matrix does not certify runtime readiness or the Fortress version; check the installed `ignition-gazebo6` packages on the Linux host. The Humble plugin `<ros><remapping>` forwards controller topic remaps as documented in [the Humble integration documentation](https://github.com/ros-controls/gz_ros2_control/blob/humble/doc/index.rst). Runtime tests must confirm the actual `/drive` publisher and subscriber mappings.
