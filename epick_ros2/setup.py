from glob import glob

from setuptools import find_packages, setup


package_name = "epick_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial", "matplotlib"],
    zip_safe=True,
    maintainer="ePick ROS 2 Workspace",
    maintainer_email="maintainer@example.com",
    description="ROS 2 control and telemetry recording for ePick vacuum grippers.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "epick_driver = epick_ros2.driver:main",
            "epick_gui = epick_ros2.gui:main",
        ],
    },
)
