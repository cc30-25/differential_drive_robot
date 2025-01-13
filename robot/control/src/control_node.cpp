#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "control_node.hpp"

ControlNode::ControlNode() : Node("robot_control"), control_system_(robot::ControlCore(this->get_logger())) {
  loadParameters();
  // Initialize subscribers and publisher
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    navigation_path_topic_, 10, std::bind(&ControlNode::pathUpdateCallback, this, std::placeholders::_1));
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 10, std::bind(&ControlNode::odomUpdateCallback, this, std::placeholders::_1));
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(velocity_command_topic_, 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_loop_period_ms_),
    std::bind(&ControlNode::controlLoopCallback, this)
  );
  control_system_.initializeControlSystem(
    lookahead_distance_, max_steering_angle_, steering_gain_, linear_velocity_);
}

void ControlNode::loadParameters() {
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("navigation_path_topic", "/navigation_path");
  this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
  this->declare_parameter<std::string>("velocity_command_topic", "/velocity_command");
  this->declare_parameter<int>("control_loop_period_ms", 100);
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("steering_gain", 1.5);
  this->declare_parameter<double>("max_steering_angle", 1.5);
  this->declare_parameter<double>("linear_velocity", 1.5);

  // Retrieve parameters and store them in member variables
  navigation_path_topic_ = this->get_parameter("navigation_path_topic").as_string();
  odometry_topic_ = this->get_parameter("odometry_topic").as_string();
  velocity_command_topic_ = this->get_parameter("velocity_command_topic").as_string();
  control_loop_period_ms_ = this->get_parameter("control_loop_period_ms").as_int();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  steering_gain_ = this->get_parameter("steering_gain").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();
}

void ControlNode::pathUpdateCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {
  control_system_.updatePath(*path_msg);
}

void ControlNode::odomUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  robot_x_ = odom_msg->pose.pose.position.x;
  robot_y_ = odom_msg->pose.pose.position.y;
  // Get robot's orientation (yaw) from quaternion using utility function
  robot_theta_ = quaternionToYaw(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w
  );
}

void ControlNode::executePathFollowing() {
  if (control_system_.isPathEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Waiting for a new path...");
  }
  // Calculate control commands
  geometry_msgs::msg::Twist velocity_command = control_system_.computeControlCommand(robot_x_, robot_y_, robot_theta_);
  velocity_publisher_->publish(velocity_command);
}

void ControlNode::controlLoopCallback() {
  executePathFollowing();
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) {
    // Using tf2 to convert to RPY
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

ControlNode::ControlNode() : Node("robot_control"), control_system_(robot::ControlCore(this->get_logger())) {}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

