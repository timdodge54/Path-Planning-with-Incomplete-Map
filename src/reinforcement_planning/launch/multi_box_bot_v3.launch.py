#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_box_bot_gazebo = get_package_share_directory("reinforcement_planning")
    pkg_box_bot_description = get_package_share_directory("box_bot_description")

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_box_bot_gazebo, "launch", "launch_robots.launch.py")
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_box_bot_description, "launch", "multi_spawn_robot_v3.launch.py"
            )
        )
    )

    return LaunchDescription([start_world, spawn_robot_world])
