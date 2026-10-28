from setuptools import setup

setup(
    name="senior_robot",
    version="0.1.0",
    packages=["senior_robot"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/senior_robot"]),
        ("share/senior_robot", ["package.xml"]),
        ("share/senior_robot/config", ["config/hardware.yaml", "config/self_filter.yaml"]),
    ],
    entry_points={
        "console_scripts": [
            "scan_adapter = senior_robot.scan_adapter:main",
            "safety_node = senior_robot.safety_node:main",
            "command_node = senior_robot.command_node:main",
        ]
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Senior Robot",
    maintainer_email="maintainer@example.com",
    description="Robot profile resources",
    license="Apache-2.0",
)
