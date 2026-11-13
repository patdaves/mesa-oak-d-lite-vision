from glob import glob
from setuptools import setup

package_name = "mesa_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MESA vision team",
    maintainer_email="team@example.org",
    description="OAK-D Lite perception for MESA",
    license="MIT",
    entry_points={
        "console_scripts": [
            "oak_camera_node = mesa_vision.oak_camera_node:main",
            "obstacle_confirm_node = mesa_vision.obstacle_confirm_node:main",
            "person_tracker_node = mesa_vision.person_tracker_node:main",
        ],
    },
)
