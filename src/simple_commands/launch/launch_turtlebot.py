import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import  SetEnvironmentVariable, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Loading URDF file into a parameter
    bringup_dir = get_package_share_directory('simple_commands')

    declared_arguments = []

    # Nodes Initialization
    gazebo_ros_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3', '-topic', 'robot_description'],
        output='screen',
    )

    nodes = [
        gazebo_ros_node,
    ]
    return LaunchDescription(declared_arguments + nodes)