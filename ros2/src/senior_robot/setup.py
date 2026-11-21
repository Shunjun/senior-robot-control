from setuptools import setup

setup(
    name="senior_robot",
    version="0.1.0",
    packages=["senior_robot"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/senior_robot"]),
        ("share/senior_robot", ["package.xml"]),
        (
            "share/senior_robot/config",
            [
                "config/self_filter.yaml",
                # //Compatible: frozen original resources retain the historical share paths.
                "config/nav2.yaml",
                "config/lattice_diff.json",
                "config/slam.yaml",
                "config/simulation_motion.yaml",
                "config/simulation_rgbd.yaml",
                "config/rgbd_transport.xml",
            ],
        ),
        (
            "share/senior_robot/launch",
            [
                "launch/navigation.launch.py",
                "launch/mapping.launch.py",
                "launch/simulation_motion.launch.py",
                "launch/simulation_rgbd.launch.py",
                "launch/rgbd_diagnostic.launch.py",
                "launch/safety_motion.launch.py",
                "launch/camera_demand.launch.py",
            ],
        ),
        (
            "share/senior_robot/behavior_trees",
            # //Compatible: original byte-identical trees; production reads senior_navigation.
            ["behavior_trees/navigate.xml", "behavior_trees/navigate_through_poses.xml"],
        ),
    ],
    entry_points={
        "console_scripts": [
            "camera_rate_limit = senior_robot.camera_rate_limit:main",
            "scan_adapter = senior_robot.scan_adapter:main",
            "safety_scan_ingress = senior_robot.safety_scan_ingress:main",
            "depth_ingress = senior_robot.depth_ingress:main",
            "rgbd_candidate_producer = senior_robot.rgbd_candidate_producer:main",
            "semantic_ingress = senior_robot.semantic_ingress:main",
            # //Compatible: retain historical executable names as direct functional aliases.
            "safety_node = senior_motion_control.safety_node:main",
            "motion_fusion = senior_robot.motion_fusion:main",
            "command_node = senior_motion_control.command_node:main",
            # //Compatible: preserve ros2 run senior_robot navigation_adapter.
            "navigation_adapter = senior_navigation.navigation_adapter:main",
            "slam_adapter = senior_mapping.slam_adapter:main",
            "localization_node = senior_localization.localization_node:main",
            "agent_adapter = senior_robot.agent_adapter:main",
        ]
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Senior Robot",
    maintainer_email="maintainer@example.com",
    description="Robot profile resources",
    license="Apache-2.0",
)
