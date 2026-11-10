from glob import glob
import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = "x650_sim"
package_source = Path(__file__).resolve().parent
repo_root = package_source.parents[2]
model_files = [
    os.path.relpath(path, package_source)
    for path in (repo_root / "models" / "x650").glob("*")
]
world_files = [
    os.path.relpath(path, package_source)
    for path in (repo_root / "worlds").glob("*")
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models", "x650"), model_files),
        (os.path.join("share", package_name, "worlds"), world_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="APS490 simulation",
    maintainer_email="maintainer@example.com",
    description="Holybro X650 PX4 Gazebo simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "offboard_hover = x650_sim.offboard_hover:main",
            "manipulator_controller = x650_sim.manipulator_controller:main",
            "camera_bridge = x650_sim.camera_bridge:main",
        ]
    },
)

