import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    world_file_name = "empty_world.model"
    world = os.path.join(
        get_package_share_directory("reinforcement_planning"), "worlds", world_file_name
    )
    print(world)
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory("turtlebot3_description"), "urdf", urdf_file_name
    )

    use_sim_time = True
    list_of_robots = []

    pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"robot_state_publisher",
        namespace=f"turtlebot",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[urdf],
    )
    list_of_robots.append(pub)

    ld0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    print("made it 68")

    ld1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )
    list_of_robots.append(ld0)
    print("made it 76")
    list_of_robots.append(ld1)
    print("made it 78")
    return LaunchDescription(list_of_robots)
