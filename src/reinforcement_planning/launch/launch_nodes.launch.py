from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server']
    map_file = get_package_share_directory('reinforcement_planning') + '/config/map.yaml'
    use_sim_time = True
    autostart = True
    save_map_timeout = 2000
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65

    # Creating Map server 
    start_map_saver_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'yaml_filename': get_package_share_directory('reinforcement_planning') + '/config/map.yaml'}],
    )
    # Create lifecycle manager to manage the map server
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    # Creating map loader that reads the map from the yaml file and publishes 
        # it as an OccupancyGrid
    map_loader = launch_ros.actions.Node(
            package="reinforcement_planning",
            executable="map_loader.py",
            output="screen",
            emulate_tty=True,
            parameters=[{'map_file': map_file}],
            )
    ld = LaunchDescription()
    
    ld.add_action(map_loader)
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
