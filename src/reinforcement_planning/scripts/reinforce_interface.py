#! /usr/bin/env python3
import os
from rclpy.node import Node
import rclpy
from rclpy.node import Node
import rclpy.client
import rclpy.subscription
import rclpy.callback_groups
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import typing

import threading
from ament_index_python import get_package_share_directory
import yaml

class Reinforcement_Interface(Node):
    """Class that gets plans from the plan server

    Attributes:
        plan_sub: The subscription to the plan topic
        plan_array: The array of poses in the plan in the form 
            [x, y, z, roll, pitch, yaw]
        _lock: The lock for limiting access to plan array
        goal: The goal of the plan
        current_pose: The current pose of the robot
        
    """
    def __init__(self):
        super().__init__('reinforcement_interface')
        self.plan_sub = self.create_subscription("nav_msgs/msg/Path", "/plan", self.plan_callback, 10)
        self.plan_array: typing.List[typing.List[float]] = []
        self._lock = threading.Lock()
        self.goal: typing.Optional[typing.List[float]] = None
        self.current_pose: typing.Optional[typing.List[float]] = None
    def plan_callback(self, msg):
        """Callback for the plan topic

        Args:
            msg: The message from the plan topic
        """
        with self._lock:
            self.plan_array.clear()
            poses: typing.List[PoseStamped] = msg.poses
            for i in len(range(poses)):
                pose: PoseStamped = poses[i]
                x = pose.pose.position.x
                y = pose.pose.position.y
                z = pose.pose.position.z
                orient = pose.pose.orientation
                orientation_list = [orient.x, orient.y, orient.z, orient.w]
                roll, pitch, yaw = euler_from_quaternion(orientation_list)
                pose_array = [x, y, z, roll, pitch, yaw]
                self.plan_array.append(pose_array)
            self.goal = [poses[self.plan_array[-1]]]
            
