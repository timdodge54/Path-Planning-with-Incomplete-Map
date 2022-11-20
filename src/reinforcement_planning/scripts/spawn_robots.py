#! /usr/bin/env python3
import os
import threading

import yaml

import rclpy
from ament_index_python import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class RobotSpawner(Node):
    """Class that spawns robots"""

    def __init__(self) -> None:
        super().__init__("robot_spawner")
        self.declare_parameter("robot_count")
        self.robot_count = self.get_parameter("robot_count").value
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        while self.client.wait_for_service(1.0):
            self.get_logger().warn("service not available, waiting again...")

    def spawn_robots(self) -> None:
        """Spawns a robot with a given id"""
        base_robot_name = "turtlebot"
        urdf_file_name = "turtlebot3_" + "burger" + ".urdf"
        urdf = os.path.join(
            get_package_share_directory("turtlebot3_description"),
            "urdf",
            urdf_file_name,
        )
        for i in range(self.robot_count):
            self.get_logger().warn(f"Spawning robot {i}")
            robot_name = base_robot_name + str(i)
            req = SpawnEntity.Request()
            req.robot_namespace = robot_name
            req.name = robot_name
            req.initial_pose.position.x = i * 2.0
            req.initial_pose.position.y = i * 2.0
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                print("response: %r" % future.result())
            else:
                raise RuntimeError(
                    "exception while calling service: %r" % future.exception()
                )


def main(args=None):
    rclpy.init(args=args)
    node = Node("robot_spawner")
    node.declare_parameter("robot_count")
    robot_count = node.get_parameter("robot_count").value
    node.get_logger().warn(f"Spawning {robot_count} robots")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    while client.wait_for_service(1.0):
        node.get_logger().warn("service not available, waiting again...")
    base_robot_name = "turtlebot"
    urdf_file_name = "turtlebot3_" + "burger" + ".urdf"
    urdf = os.path.join(
        get_package_share_directory("turtlebot3_description"), "urdf", urdf_file_name
    )
    for i in range(robot_count):
        node.get_logger().warn(f"Spawning robot {i}")
        robot_name = base_robot_name + str(i)
        req = SpawnEntity.Request()
        req.xml = open(urdf, "r").read()
        req.robot_namespace = robot_name
        req.name = robot_name
        req.initial_pose.position.x = i * 2.0
        req.initial_pose.position.y = i * 2.0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print("response: %r" % future.result())
        else:
            raise RuntimeError(
                "exception while calling service: %r" % future.exception()
            )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
