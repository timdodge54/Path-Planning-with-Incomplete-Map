import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "turtlbot"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})


    return robots


def generate_launch_description():
    NUM_ROBOTS = 5
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    
    use_sim_time = True
    list_of_robots = []

    for i in range(NUM_ROBOTS):
        pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'robot_state_publisher_{i}',
                namespace=f'turtlbot{i}',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[urdf]
                )
        list_of_robots.append(pub) 

    ld0 =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    print("made it 68")

    ld1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
    list_of_robots.append(ld0)
    print("made it 76")
    list_of_robots.append(ld1)
    print("made it 78")
    return LaunchDescription(list_of_robots)
        
   
   


