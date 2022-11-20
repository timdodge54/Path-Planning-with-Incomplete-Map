#! /usr/bin/env python3
import threading
import typing

import rclpy
import rclpy.callback_groups
import rclpy.client
import rclpy.subscription
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion


class Reinforcement_Interface(Node):
    """Class that gets plans from the plan server

    Attributes:
        plan_sub: The subscription to the plan topic
        plan_array: The array of poses in the plan in the form 
            [x, y, z, roll, pitch, yaw]
        plan_lock: The lock for limiting access to plan array
        pose_lock: The lock for limiting access to the current pose
        goal: The goal of the plan
        current_pose: The current pose of the robot
        target_frame: The frame that houses the map 
        
    """

    def __init__(self):
        """Initialize."""
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

    def plan_callback(self, msg: Path) -> None:
        """Clear plan array and replace with current path.

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
        """Listen to the tf tree and get the current pose of the robot."""
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
