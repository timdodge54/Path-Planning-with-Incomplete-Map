from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_saver', 'map_server']
    use_sim_time = True
    autostart = True
    save_map_timeout = 2000
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65
    share = get_package_share_directory('reinforcement_planning')
    file_loc = f"{share}/map.yaml"

    # Nodes launching commands
    map_loader_node = launch_ros.actions.Node(
        package='reinforcement_planning',
        executable='map_loader_cmd',
        name='map_loader',
        parameters=[{'map_path': file_loc}],
    )

    ld = LaunchDescription()
    ld.add_action(map_loader_node)


    return ld
