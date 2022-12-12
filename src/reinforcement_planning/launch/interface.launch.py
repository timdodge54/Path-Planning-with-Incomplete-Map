import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    interface = Node(
        package='reinforcement_planning',
        executable='rein_interface.py',
        name='reinforcement_interface',
        output='screen',
    )
    ld.add_action(interface)

    return ld
    
    
