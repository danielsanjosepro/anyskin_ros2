from setuptools import find_packages, setup
import os
from glob import glob


package_name = "anyskin_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel San Jose Pro",
    maintainer_email="sanjose.daniel@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tactile_sensor_broadcaster = anyskin_bringup.tactile_sensor_broadcaster:main"
        ],
    },
)
