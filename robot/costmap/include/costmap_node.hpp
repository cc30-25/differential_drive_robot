#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void publishMessage();
    
    void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

  private:
    robot::CostmapCore costmap_processor_;
    std::string scan_topic_;
    std::string map_topic_;
    double map_resolution_;
    int map_width_;
    int map_height_;
    geometry_msgs::msg::Pose map_origin_;
    double cost_inflation_radius_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
};

#endif
