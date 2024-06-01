# Copyright 2023 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    #This project
    pkg_project = get_package_share_directory('nicla_vision_ros2')

    #Offical ROS-gazebo bridge repo
    pkg_ros_gz_sim = get_package_share_directory('ros_ign_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_xacro = os.path.join(pkg_project, 'urdf/', 'nicla.urdf.xacro')
    assert os.path.exists(robot_xacro), "The nicla.urdf.xacro doesnt exist in "+str(robot_xacro)

    robot_description_config = xacro.process_file(robot_xacro)
    robot_desc = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project, 'config', 'rviz', 'nicla.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'ign_gazebo.launch.py')),
    )

    # Spawn robot
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'nicla',
                    '-z', '0.1',
                    '-topic', '/robot_description'],
                 output='screen')

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        robot_state_publisher,
        rviz,
        gz_sim,
        spawn,
        bridge
    ])