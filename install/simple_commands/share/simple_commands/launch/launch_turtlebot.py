import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



import xacro


def generate_launch_description():
    # Define the URDF path
    bringup_dir = get_package_share_directory('simple_commands')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'turtlebot3_burger.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_path).toxml()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_second_delta"),
                    "urdf",
                    "delta.urdf.xacro",
                ]
            ),
            " ",
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Create nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description]
    )
    
    # Add nodes to the launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument('gui', default_value='False',
                               description='Whether to start the Joint State Publisher GUI'),
        
        robot_state_publisher,
        joint_state_publisher
    ])
