# Copyright 2024 HHCM, Istituto Italiano di Tecnologia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # External args
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    gazebo_arg = DeclareLaunchArgument("gazebo", default_value="true")

    # External args for xacro
    nicla_tof_arg = DeclareLaunchArgument("nicla_tof", default_value="true")
    nicla_tof_visualize_arg = DeclareLaunchArgument(
        "nicla_tof_visualize", default_value="true"
    )
    nicla_camera_arg = DeclareLaunchArgument(
        "nicla_camera", default_value="true"
    )
    nicla_mic_arg = DeclareLaunchArgument("nicla_mic", default_value="true")
    nicla_imu_arg = DeclareLaunchArgument("nicla_imu", default_value="true")
    nicla_use_mesh_arg = DeclareLaunchArgument(
        "nicla_use_mesh", default_value="false"
    )

    # This project
    pkg_project = get_package_share_directory("nicla_vision_ros2")

    # Offical ROS-gazebo bridge repo
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Opaque function to use the launch argument inside here
    def create_robot_description(context):
        robot_xacro = os.path.join(pkg_project, "urdf/", "nicla.urdf.xacro")
        assert os.path.exists(
            robot_xacro
        ), "The nicla.urdf.xacro doesnt exist in " + str(robot_xacro)
        robot_description_config = xacro.process_file(
            robot_xacro,
            mappings={
                "nicla_camera": context.launch_configurations["nicla_camera"],
                "nicla_tof": context.launch_configurations["nicla_tof"],
                "nicla_tof_visualize": context.launch_configurations[
                    "nicla_tof_visualize"
                ],
                "nicla_mic": context.launch_configurations["nicla_mic"],
                "nicla_imu": context.launch_configurations["nicla_imu"],
                "nicla_use_mesh": context.launch_configurations[
                    "nicla_use_mesh"
                ],
            },
        )
        robot_desc = robot_description_config.toxml()

        return [SetLaunchConfiguration("robot_desc", robot_desc)]

    create_robot_description_arg = OpaqueFunction(
        function=create_robot_description
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": LaunchConfiguration("robot_desc")},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project, "config", "rviz", "nicla.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    # )
    # Or this?
    # gz_sim = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
    #     launch_arguments={
    #         'gui': LaunchConfiguration('gui'),
    #         'pause': 'false',
    #     }.items(),
    # )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
        condition=IfCondition(LaunchConfiguration("gazebo")),
    )

    # Spawn robot
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(LaunchConfiguration("gazebo")),
        arguments=[
            "-name",
            "nicla",
            "-entity",
            "model" "-z",
            "0.1",
            "-topic",
            "/robot_description",
            # '-unpause'
        ],
        output="screen",
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(LaunchConfiguration("gazebo")),
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project, "config", "gz_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        remappings=[
            ("/world/empty/model/nicla/joint_state", "joint_states"),
        ],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["nicla/camera/image_raw"],
        output="screen",
    )

    return LaunchDescription(
        [
            nicla_tof_arg,
            nicla_tof_visualize_arg,
            nicla_camera_arg,
            nicla_mic_arg,
            nicla_imu_arg,
            nicla_use_mesh_arg,
            create_robot_description_arg,
            rviz_arg,
            gazebo_arg,
            use_sim_time_arg,
            gz_sim,
            robot_state_publisher,
            rviz,
            spawn,
            bridge,
            image_bridge,
        ]
    )
