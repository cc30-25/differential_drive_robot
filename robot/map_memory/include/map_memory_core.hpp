#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{
class MapMemoryCore {
 public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configureMapMemory(
        double resolution, 
        int grid_width, 
        int grid_height, 
        const geometry_msgs::msg::Pose& origin_pose
    );

    void refreshMap(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map,
        double x_position, double y_position, double orientation
    );

    bool convertToMapCoordinates(
        double robot_x, double robot_y, int& map_x, int& map_y
    ) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr retrieveMap() const;

 private:
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Logger logger_;
};
}  

#endif  // MAP_MEMORY_core_HPP_
