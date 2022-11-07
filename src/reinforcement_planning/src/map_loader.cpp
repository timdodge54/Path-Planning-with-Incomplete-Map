#include "reinforcment_planning/map_loader.hpp"
using namespace reinforcement_planning;
using namespace std::chrono;

MapLoader::MapLoader() : Node("map_loader"), map_path_("a"), timer_res_(12.0)
{
  // Create a publisher for the map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  // Get the path to the map config file
  this->declare_parameter("map_path");
  this->declare_parameter("timer_res", this->timer_res_);
  this->get_parameter("map_path", this->map_path_);
  RCLCPP_INFO(this->get_logger(), "Map path: %s", this->map_path_.c_str());
  this->get_parameter("timer_res", this->timer_res_);
  // Create a timer to periodically publish the map
  this->timer_ = rclcpp::create_timer(this, this->get_clock(), duration<double>(this->timer_res_),
                                      std::bind(&MapLoader::publishGrid, this));
}

void MapLoader::publishGrid()
{
  nav_msgs::msg::OccupancyGrid map;
  nav2_map_server::loadMapFromYaml(map_path_, map);
  map_pub_->publish(map);
}

int main(int argc, char** argv)
{
  // Initialize ros and node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapLoader>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
