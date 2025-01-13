#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory_node"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    // Load parameters from ROS2 YAML
    loadParameters();

    // Subscribe to the local costmap topic
    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        local_costmap_topic_,
        10,
        std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1)
    );

    // Subscribe to the odometry topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
    );

    // Publish the global costmap
    global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        10
    );

    // Set up a timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(map_pub_rate_),
        std::bind(&MapMemoryNode::timerCallback, this)
    );

    map_memory_.initMapMemory(resolution_, width_, height_, origin_);
    RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized successfully.");
}

void MapMemoryNode::loadParameters() {
    // Declare and retrieve all ROS2 parameters
    this->declare_parameter<std::string>("local_costmap_topic", "/local_costmap");
    this->declare_parameter<std::string>("odom_topic", "/odometry");
    this->declare_parameter<std::string>("map_topic", "/global_map");
    this->declare_parameter<int>("map_publish_rate", 500);
    this->declare_parameter<double>("min_update_distance", 1.0);
    this->declare_parameter<double>("global_map.resolution", 0.05);
    this->declare_parameter<int>("global_map.width", 200);
    this->declare_parameter<int>("global_map.height", 200);
    this->declare_parameter<double>("global_map.origin.x", -10.0);
    this->declare_parameter<double>("global_map.origin.y", -10.0);
    this->declare_parameter<double>("global_map.origin.orientation", 1.0);

    local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    map_pub_rate_ = this->get_parameter("map_publish_rate").as_int();
    update_distance_ = this->get_parameter("min_update_distance").as_double();
    resolution_ = this->get_parameter("global_map.resolution").as_double();
    width_ = this->get_parameter("global_map.width").as_int();
    height_ = this->get_parameter("global_map.height").as_int();
    origin_.position.x = this->get_parameter("global_map.origin.x").as_double();
    origin_.position.y = this->get_parameter("global_map.origin.y").as_double();
    origin_.orientation.w = this->get_parameter("global_map.origin.orientation").as_double();
}

void MapMemoryNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    bool is_empty = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t value) { return value == 0; });
    if (is_empty) {
        RCLCPP_INFO(this->get_logger(), "Received an empty local costmap. Skipping update.");
        return;
    }

    if (!std::isnan(last_robot_x_)) {
        double distance_moved = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
        if (distance_moved < update_distance_) {
            // Not enough movement to warrant an update
            return;
        }
    }

    // Update robot's last position
    last_robot_x_ = robot_x_;
    last_robot_y_ = robot_y_;

    // Merge local costmap into global map
    map_memory_.updateMap(msg, robot_x_, robot_y_, robot_theta_);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    robot_theta_ = quaternionToYaw(qx, qy, qz, qw);
}

void MapMemoryNode::timerCallback() {
    auto global_map = map_memory_.getMapData();
    global_map->header.stamp = this->now();
    global_map->header.frame_id = "map_frame";
    global_costmap_pub_->publish(*global_map);
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
    tf2::Quaternion quat(x, y, z, w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}

