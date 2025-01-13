#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{
class CostmapCore {
public:
    // Constructor accepting logger for terminal output
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Initialize the costmap with configuration parameters
    void configureCostmap(
        double resolution,
        int width,
        int height,
        geometry_msgs::msg::Pose origin,
        double obstacle_inflation_radius
    );

    // Update the costmap based on current laser scan data
    void refreshCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_data) const;

    // Inflate obstacles on the costmap to account for safety margins
    void expandObstacleArea(int origin_x, int origin_y) const;

    // Retrieve the current costmap data
    nav_msgs::msg::OccupancyGrid::SharedPtr fetchCostmapData() const;

private:
    // Shared pointer to the costmap data
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;

    // Logger for output
    rclcpp::Logger logger_;

    // Inflation radius for obstacle expansion
    double inflation_radius_;
    // Number of cells to inflate around obstacles
    int inflation_cells_;
};

}
