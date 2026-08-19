from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_nav2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "maps"),
            glob("maps/*.json") + glob("maps/*.pgm") + glob("maps/*.yaml") + glob("maps/*.md"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bkuang",
    maintainer_email="kuangbrian71@gmail.com",
    description="Lean Nav2 bringup using simulator ground-truth odometry.",
    license="Apache-2.0",
    url="https://github.com/bkuangs/target-intercept",
    entry_points={
        "console_scripts": [
            "ground_truth_tf = robot_nav2.ground_truth_tf:main",
        ],
    },
)
