#! /usr/bin/env python3
import threading
import typing

import rclpy
import rclpy.callback_groups
import rclpy.client
import rclpy.executors
import rclpy.subscription
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from plan_msg.srv import Plan
from rclpy.node import Node


class PlanSaver(Node):
    """A node meant to save plans from the plan server.

    Attributes:
        plan_sub: The subscription to the plan topic
        plan: The plan received from the plan server
        plan_server: The service that saves the plan
    """

    def __init__(self):
        """Initialize.
        
        Args:
            None
        """
        super().__init__("plan_saver")
        self._lock = threading.Lock()
        self.plan_sub = self.create_subscription(Path, "/plan", self.plan_callback, 10)
        self.plan: typing.Optional(typing.List[PoseStamped]) = None
        self.plan_server = self.create_service(Plan, "/paths", self.plan_cb)

    def plan_callback(self, msg: Path):
        """Callback for the plan subscription.

        Args:
            msg: The message received from the plan topic
        """
        with self._lock:
            self.plan = msg.poses
            self.get_logger().info("Plan received")
            self.get_logger().info(f"{self.plan}")

    def plan_cb(self, req: Plan.Request, res: Plan.Response):
        """Callback to send the saved plan.
        
        Args:
            req: a dummy request that is not utilized
            res: the plan to be sent to the client
        """
        with self._lock:
            res.poses = self.plan
            return res


def main(args=None):
    rclpy.init(args=args)
    node = PlanSaver()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
