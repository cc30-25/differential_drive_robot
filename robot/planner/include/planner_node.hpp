#ifndef PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER_NODE_HPP_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "planner_core.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode();

    void initializeParams();
    void onMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void onGoalReceived(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg);
    void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void onTimerEvent();
    void planAndPublishPath();
    void clearActiveGoal();

private:
    robot::PlannerCore path_planner_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr periodic_timer_;

    // Map and thread safety
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    std::mutex costmap_mutex_;

    // Goal-related data
    geometry_msgs::msg::PointStamped goal_point_;
    bool goal_active_;
    rclcpp::Time planning_start_time_;

    // Odometry
    bool odom_received_;
    double robot_odom_x_;
    double robot_odom_y_;

    // Configuration parameters
    std::string topic_map_;
    std::string topic_goal_;
    std::string topic_odom_;
    std::string topic_path_;
    double path_smoothing_factor_;
    int max_iterations_;
    double tolerance_to_goal_;
    double planning_timeout_seconds_;
};

#endif  // PATH_PLANNER_NODE_HPP_
