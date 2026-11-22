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
            ],
        ),
        (
            "share/senior_robot/launch",
            [
                "launch/rgbd_diagnostic.launch.py",
                "launch/safety_motion.launch.py",
                "launch/camera_demand.launch.py",
            ],
        ),
    ],
    entry_points={
        "console_scripts": [
            "motion_fusion = senior_robot.node_composition:motion_fusion_main",
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
