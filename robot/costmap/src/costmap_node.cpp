#include "costmap_node.hpp"
#include <chrono>
#include <memory>

CostmapNode::CostmapNode() : Node("costmap_node"), costmap_core_(robot::CostmapCore(this->get_logger())) {
  // Load configuration parameters
  loadParameters();
  laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_, 10, 
    std::bind(
      &CostmapNode::laserScanCallback, this, 
      std::placeholders::_1));
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 10);
  RCLCPP_INFO(this->get_logger(), "ROS2 constructs initialized");
  costmap_core_.initializeMap(
    map_resolution_, 
    map_width_, 
    map_height_, 
    map_origin_,
    inflation_radius_
  );
  RCLCPP_INFO(this->get_logger(), "Costmap core initialized");
}

void CostmapNode::loadParameters() {
  // Declare parameters in ROS2
  this->declare_parameter<std::string>("laserscan_topic", "/lidar_scan");
  this->declare_parameter<std::string>("costmap_topic", "/robot_costmap");
  this->declare_parameter<double>("costmap.resolution", 0.1);
  this->declare_parameter<int>("costmap.width", 100);
  this->declare_parameter<int>("costmap.height", 100);
  this->declare_parameter<double>("costmap.origin.position.x", -5.0);
  this->declare_parameter<double>("costmap.origin.position.y", -5.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.0);
  
  // Retrieve parameters from ROS2 server and store them locally
  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  map_resolution_ = this->get_parameter("costmap.resolution").as_double();
  map_width_ = this->get_parameter("costmap.width").as_int();
  map_height_ = this->get_parameter("costmap.height").as_int();
  map_origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  map_origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  map_origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) const {
  // Update the costmap based on the received laser scan
  costmap_core_.processLaserScan(laser_msg);
  
  // Publish the updated costmap message
  nav_msgs::msg::OccupancyGrid costmap_message = *costmap_core_.retrieveCostmap();
  costmap_message.header = laser_msg->header;
  costmap_publisher_->publish(costmap_message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
