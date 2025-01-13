#ifndef CONTROL_ENGINE_NODE_HPP_
#define CONTROL_ENGINE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();


    
    void configureParameters();
 
    double quaternionToYaw(double x, double y, double z, double w);
    // callback
    void pathUpdateCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // update callback
    void odometryUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Follow the path by generating control commands
    void followTrajectory();
    // regular path following
    void timerCallback();
  
  private:
   robot::ControlCore control_;
    // ROS2 subscriber and publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Timer for periodic control updates
    rclcpp::TimerBase::SharedPtr timer_;
    // Robot state tracking
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    // ROS2 parameters for configuration
    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    
    int control_period_ms_;
    double lookahead_distance_;
    double steering_gain_;
    double max_steering_angle_;
    double linear_velocity_;
};

#endif
