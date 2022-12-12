#! /usr/bin/env python3
from reinforcement_planning.reinforcement_planning.Agent import Agent
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
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
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
        self.agent = Agent(
            alpha=0.000025,
            beta=0.00025,
            input_dims=[12],
            tau=0.001,
            batch_size=64,
            fc1_dims=300,
            fc2_dims=200,
            fc3_dims=100,
            n_actions=2,
            action_range=1,
            run_name=sys.argv[1]
        )
        self.agent.load_models()
        self.agent.eval()
        self._timer = self.create_timer(0.1, self.reinforcement_loop)

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
        self.goal = self.plan_array[-1]
        
                
    def calc_closest(self, current_pose: typing.List[float]) -> Twist:
        """Calculates the closest point to the robot"""
        current_distance = 0
        current_closest = []
        current_next = [] 
        for i in range(len(self.plan_array) - 1):
            element = self.plan_array[i]
            x = element[0]
            y = element[1]
            calc = np.sqrt(
                (x - current_pose[0]) ** 2 + (y - current_pose[1]) ** 2
                )
            if calc < current_distance:
                current_closest = element
                current_distance = calc
                current_next = self.plan_array[i+1]
        return (current_closest, current_next)

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
                return (x, y, yaw)

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
                )
                raise
    def calc_heading_dif(
        self,
        current_next_plan: typing.List[float],
        current_position: typing.List[float],
        current_plan: typing.List[float]) -> float:
        """Calculates the difference in heading between the robot and the path"""
        ## x, y, theta

        path_vector = [current_next_plan[0] - current_plan[0], current_next_plan[1] - current_plan[1]]
        robot_vector = [math.sin(current_position[2]), math.cos(current_position[2])]
        
        dot = path_vector[0] * robot_vector[0] + path_vector[1] * robot_vector[1]
        det = path_vector[0] * robot_vector[1] - path_vector[1] * robot_vector[0]
        
        d_theta = math.atan2(det, dot)
        
        if d_theta <= math.pi:
            d_theta *= -1
        else:
            d_theta = 2 * math.pi - d_theta
            
        return d_theta 
        
    def reinforcement_loop(self) -> None:
        pose = self.get_current_pose()
        next_point, closest_point = self.calc_closest(pose)
        d_theta = self.calc_heading_dif(next_point, pose, closest_point)




            
            # obs needs to be array 
                # obs = [
                    # unit vector to path (x,y), unit vector to goal (x,y),
                    # forward ultrasonic sensor, left ultrasonic sensor, 
                    # right ultrasonic sensor, back ultrasonic sensor,
                    # difference in heading between robot and path,
                    # current x, current y, current theta
                    # ]
            #act = agent.choose_action(obs)
