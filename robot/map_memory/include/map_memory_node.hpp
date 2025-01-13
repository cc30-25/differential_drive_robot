#ifndef MEMORY_MAP_NODE_HPP_
#define MEMORY_MAP_NODE_HPP_

#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "map_memory_core.hpp"

class MemoryMapNode : public rclcpp::Node {
 public:
    MemoryMapNode();

    void handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);
    void onTimerEvent();

    // Utility function for quaternion to yaw conversion
    double convertQuaternionToYaw(double qx, double qy, double qz, double qw) const;

    void loadParameters();

 private:
    // Core processing unit for global map updates
    robot_system::MapMemoryCore memory_core_;
    
    // ROS2 communication interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // ROS2 configuration parameters
    std::string local_map_topic_;
    std::string odometry_topic_;
    std::string global_map_topic_;
    int publish_rate_;
    double grid_resolution_;
    int grid_width_;
    int grid_height_;
    geometry_msgs::msg::Pose map_origin_;
    double min_update_distance_;

    // Robot state in simulation frame
    double current_robot_x_;  
    double current_robot_y_;  
    double current_robot_yaw_;  

    // Tracking last known robot position for update checks
    double previous_robot_x_;
    double previous_robot_y_;
};

#endif  // MEMORY_MAP_NODE_HPP_
