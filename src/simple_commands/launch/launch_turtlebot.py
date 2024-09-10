import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    aruco_publisher = Node(
        package='simple_commands',
        executable='aruco_publisher',
        name='aruco_publisher',
    )   

    aruco_subscriber = Node(
        package='simple_commands',
        executable='aruco_subscriber',
        name='aruco_subscriber'
    )

    robots_formation = Node(
        package='simple_commands',
        executable='robots_formation',
        name='robots_formation'
    )
    
    # Add nodes to the launch description
    return launch.LaunchDescription([        
        robots_formation
    ])

if __name__ == "__main__":
    launch.launch(generate_launch_description())
