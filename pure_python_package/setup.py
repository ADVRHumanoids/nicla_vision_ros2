import os
from glob import glob
from setuptools import setup

package_name = "nicla_vision_ros2"
author_email_str = (
    "toridebraus@gmail.com, "
    "damiano.gasperini98@gmail.com, "
    "delbianco.edoardo@gmail.com, "
    "rollo.f96@gmail.com"
)

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        # Include our package.xml file
        (os.path.join("share", package_name), ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config/rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.xacro")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf/meshes", "*.stl")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf/meshes", "*.dae")),
        ),
    ],
    # This is important as well
    install_requires=["setuptools"],
    zip_safe=True,
    author="Davide Torielli, Damiano Gasperini, Edoardo Del Bianco, Federico Rollo",
    author_email=author_email_str,
    keywords=[
        "nicla_vision",
        "driver",
        "arduino_integration",
        "nicla_integration",
    ],
    classifiers=[
        "Intended Audience :: Users",
        "License :: Apache 2.0 Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="ROS2 integration of the Arduino Nicla Vision",
    license="Apache 2.0 Software License",
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        "console_scripts": [
            "nicla_receiver = nicla_vision_ros2.nicla_receiver:main"
        ],
    },
)
