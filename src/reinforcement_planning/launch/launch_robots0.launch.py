import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


def generate_launch_description():
    ld = LaunchDescription()
    NUM_ROBOTS = 3
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    world_file_name = "turtlebot3_worlds/" + TURTLEBOT3_MODEL + ".model"
    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "worlds", world_file_name
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory("turtlebot3_description"), "urdf", urdf_file_name
    )

    xml = open(urdf, "r").read()

    xml = xml.replace('"', '\\"')
    ld.add_action(
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world, "-s", "libgazebo_ros_factory.so"],
            output="screen",
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "param",
                "set",
                "/gazebo",
                "use_sim_time",
                use_sim_time,
            ],
            output="screen",
        )
    )

    for i in range(NUM_ROBOTS):
        pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"robot_state_publisher_{i}",
            namespace=f"turtlebot{i}",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", f"turtlebot{i}/cmd_vel")],
            arguments=[urdf],
        )
        ld.add_action(pub)
    spawn = Node(
        package="reinforcement_planning",
        executable="spawn_robots.py",
        name="spawn_robots",
        parameters=[{"robot_count": NUM_ROBOTS}],
    )
    ld.add_action(spawn)
    for entity in ld.entities:
        print(f"{entity}")
    return ld
