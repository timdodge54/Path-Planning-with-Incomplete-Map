#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_map_server/map_io.hpp"

namespace reinforcement_planning
{
class MapLoader : public rclcpp::Node
{
public:
  MapLoader();

  void publishGrid();

private:
  std::string map_path_;
  double timer_res_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace reinforcement_planning