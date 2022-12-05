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
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
        super().__init__("reinforcement_interface")
        self.plan_sub = self.create_subscription(
            "nav_msgs/msg/Path", "/plan", self.plan_callback, 10
        )
        self.plan_array: typing.List[typing.List[float]] = []
        self.plan_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.goal: typing.Optional[typing.List[float]] = None
        self.target_frame = (
            self.declare_parameter("target_frame", "/map")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_pose: typing.Optional[typing.List[float]] = None

    def plan_callback(self, msg) -> None:
        """Callback for the plan topic

        Args:
            msg: The message from the plan topic
        """
        with self.plan_lock:
            self.plan_array.clear()
            poses: typing.List[PoseStamped] = msg.poses
            for i in range(len(poses)):
                pose: PoseStamped = poses[i]
                x = pose.pose.position.x
                y = pose.pose.position.y
                z = pose.pose.position.z
                orient = pose.pose.orientation
                orientation_list = [orient.x, orient.y, orient.z, orient.w]
                roll, pitch, yaw = euler_from_quaternion(orientation_list)
                pose_array = [x, y, z, roll, pitch, yaw]
                self.plan_array.append(pose_array)

    def get_current_pose(self) -> None:
        """Listens to the tf tree to get the current pose of the robot."""
        with self.pose_lock:
            from_frame_rel: str = self.target_frame
            to_frame_rel = "base_footprint"

            try:
                current_pose = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, rclpy.time.Time()
                )
                x = current_pose.transform.translation.x
                y = current_pose.transform.translation.y
                z = current_pose.transform.translation.z
                orien_x = current_pose.transform.rotation.x
                orien_y = current_pose.transform.rotation.y
                orien_z = current_pose.transform.rotation.z
                orien_w = current_pose.transform.rotation.w
                roll, pitch, yaw = euler_from_quaternion(
                    [orien_x, orien_y, orien_z, orien_w]
                )
                self.current_pose = [x, y, z, roll, pitch, yaw]

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
                )
                return
