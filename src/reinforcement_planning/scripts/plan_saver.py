#! /usr/bin/env python3
from ddpg_planning.Agent import Agent
import time
import os
from rclpy.node import Node
import rclpy
import rclpy.executors
from rclpy.node import Node
import rclpy.client
import rclpy.subscription
import rclpy.callback_groups
from tf_transformations import euler_from_quaternion
import sys
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import typing

import threading
from ament_index_python import get_package_share_directory
import yaml
import math

from geometry_msgs.msg import Twist
from plan_msg.srv import Plan


import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PlanSaver(Node):
    def __init__(self):
        super().__init__("plan_saver")
        self.plan_sub = self.create_subscription(
            Path, "/plan", self.plan_callback, 10
        )
        self.plan: typing.Optional(typing.List[PoseStamped]) = None
        self.plan_server = self.create_service(Plan, "/paths" ,self.plan_cb)

    def plan_callback(self, msg: Path):
        self.plan = msg.poses
        self.get_logger().info("Plan received")
        self.get_logger().info(f"{self.plan}")
    
    def plan_cb(self, req: Plan.Request, res: Plan.Response):
        res.poses = self.plan
        return res

def main(args=None):
    rclpy.init(args=args)
    node = PlanSaver()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
        
