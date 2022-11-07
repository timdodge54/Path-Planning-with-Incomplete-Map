import rclpy
import rclpy.publisher
import rclpy.parameter
import rclpy.callback_groups
from rclpy.node import Node
import yaml
import threading
import rl

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap
from ament_index_python import get_package_share_directory


class MapLoader(Node):
    """Class that loads a map from a yaml file and publishes an OccupancyGrid.

    Attributes:
        file_loc: The location of the yaml file.

    """

    def __init__(self):
        """Initialize.
        
        Args:
            None
            
        Parameters:
            map_file: The location of the yaml file.
        """
        super().__init__('map_loader')
        share = get_package_share_directory('reinforcement_planning')
        file_loc = f"{share}/map.yaml"
        param = rclpy.parameter.Parameter(
            'map_file',
            rclpy.Parameter.Type.STRING,
            file_loc
            )
        self.declare_parameter('map_file')
        self.file_loc: str = self.get_parameter_or('map_file', file_loc).value
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(
            LoadMap,
            'map_server/load_map',
            callback_group=self.cbg,
            )
        self._lock = threading.Lock()
        while not self.client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')
        

    async def query_map(self):
        """Query the map server for the map.

        Args:
            None

        Returns:
            None

        """
        self.get_logger().debug('Requesting map...')
        req = LoadMap.Request()
        req.map_url = self.file_loc
        with self._lock:
            self.future = await self.client.call_async(req)
        self.get_logger().debug('Map received.')
        self.map: LoadMap.Response = self.future.result().map
    
    def publish_grid(self):
        """Publish the map as an OccupancyGrid.

        Args:
            None

        Returns:
            None

        """
        self.get_logger().debug('Publishing map...')
        self.pub = self.create_publisher(
            OccupancyGrid,
            'map',
            1,
            )
        self.pub.publish(self.map)
        self.get_logger().debug('Map published.')
        