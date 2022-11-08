#! /usr/bin/env python3
from rclpy.node import Node
import rclpy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap 

import threading
from ament_index_python import get_package_share_directory
import yaml


class MapLoader(Node):
    """Class that loads a map from a yaml file and publishes an OccupancyGrid.

    Attributes:
        file_loc: The location of the yaml file.
        client: The client that queries the map server.
        pub: The publisher that publishes the map as an OccupancyGrid.
        _map_q_timer: The timer that queries the map server at given frequency.
        _publish_timer: The timer that publishes the map at given frequency.

    """

    def __init__(self):
        """Initialize.
        
        Args:
            None
            
        Parameters:
            map_file: The location of the yaml file.
        """
        super().__init__('map_loader')
        # setting log level
        self.log_level = self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # setting default values for parameters
        share = get_package_share_directory('reinforcement_planning')
        file_loc = f"{share}/map.yaml"
        param = rclpy.parameter.Parameter(
            'map_file',
            rclpy.Parameter.Type.STRING,
            file_loc
            )
        # declare and get parameter
        self.declare_parameter('map_file')
        self.file_loc: str = self.get_parameter_or('map_file', file_loc).value
        self.get_logger().debug(f"Map file: {self.file_loc} {type(self.file_loc)}")
        # Setting callback groups
        self._cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._timer1_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._timer2_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # Creating client and publisher
        self.client = self.create_client(
            LoadMap,
            '/map_server/load_map',
            callback_group=self._cbg,
            )
        while not self.client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')
        self.pub = self.create_publisher(
            OccupancyGrid,
            'map',
            1,
        )
        # creating multi-threading lock
        self._lock = threading.Lock()
        # creating timers
        self._map_q_timer = self.create_timer(
            3,
            self.query_map,
            callback_group=self._timer1_cbg,
        )
        self._publish_timer = self.create_timer(
            4,
            self.publish_grid,
            callback_group=self._timer2_cbg,
        )
            
        

    async def query_map(self):
        """Query the map server for the map.

        Args:
            None

        Returns:
            None

        """
        # Create request
        req = LoadMap.Request()
        # Lock map data so publishing thread can't access it
        with self._lock:
            req.map_url= self.file_loc
            res: LoadMap.Response = await self.client.call_async(req)
            # save map data
            self.map: LoadMap.Response = res.map
        self.get_logger().debug('Map received.')
    
    def publish_grid(self):
        """Publish the map as an OccupancyGrid.

        Args:
            None

        Returns:
            None

        """
        self.get_logger().debug('Publishing map...')
       # Lock map data so query thread can't access it 
        with self._lock:
            self.pub.publish(self.map)
        self.get_logger().debug('Map published.')
        
def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
    
