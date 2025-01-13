#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(robot::PlannerCore(this->get_logger())) {
    initializeParameters();
    setupSubscribers();
    setupPublishers();
    setupTimer();
    planner_.initialize(smoothing_factor_, iterations_);
}

void PlannerNode::initializeParameters() {
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<double>("smoothing_factor", 0.2);
    this->declare_parameter<int>("iterations", 20);
    this->declare_parameter<double>("goal_tolerance", 0.3);
    this->declare_parameter<double>("plan_timeout_seconds", 10.0);

    map_topic_ = this->get_parameter("map_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    iterations_ = this->get_parameter("iterations").as_int();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::setupSubscribers() {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10, std::bind(&PlannerNode::onMapUpdate, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, 10, std::bind(&PlannerNode::onGoalReceived, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&PlannerNode::onOdomUpdate, this, std::placeholders::_1));
}

void PlannerNode::setupPublishers() {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
}

void PlannerNode::setupTimer() {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::onTimer, this));
}

void PlannerNode::onMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = msg;
    }
    if (active_goal_) {
        double elapsed_time = (now() - plan_start_time_).seconds();
        if (elapsed_time <= plan_timeout_) {
            RCLCPP_INFO(this->get_logger(),
                        "Replanning due to map update (elapsed time: %.2f).", elapsed_time);
            executePathPlanning();
        }
    }
}

void PlannerNode::onGoalReceived(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg) {
    if (active_goal_) {
        RCLCPP_WARN(this->get_logger(), "New goal ignored; active goal in progress.");
        return;
    }
    if (!map_) {
        RCLCPP_WARN(this->get_logger(), "Cannot set goal; no map data available.");
        return;
    }
    current_goal_ = *goal_msg;
    active_goal_ = true;
    plan_start_time_ = now();
    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                goal_msg->point.x, goal_msg->point.y);
    executePathPlanning();
}

void PlannerNode::onOdomUpdate(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    odom_x_ = odom_msg->pose.pose.position.x;
    odom_y_ = odom_msg->pose.pose.position.y;
    have_odom_ = true;
}

void PlannerNode::onTimer() {
    if (!active_goal_) {
        return;
    }
    double elapsed_time = (now() - plan_start_time_).seconds();
    if (elapsed_time > plan_timeout_) {
        RCLCPP_WARN(this->get_logger(), "Goal timed out after %.2f seconds.", elapsed_time);
        clearGoal();
        return;
    }
    double distance_to_goal = std::hypot(odom_x_ - current_goal_.point.x,
                                         odom_y_ - current_goal_.point.y);
    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached in %.2f seconds.", elapsed_time);
        clearGoal();
    }
}

void PlannerNode::executePathPlanning() {
    if (!have_odom_) {
        RCLCPP_WARN(this->get_logger(), "No odometry data available; cannot plan path.");
        clearGoal();
        return;
    }

    double start_x = odom_x_;
    double start_y = odom_y_;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!planner_.generatePath(start_x, start_y, current_goal_.point.x, current_goal_.point.y, map_)) {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed.");
            clearGoal();
            return;
        }
    }

    auto path_msg = planner_.getPathMessage();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_->header.frame_id;
    path_pub_->publish(path_msg);
}

void PlannerNode::clearGoal() {
    active_goal_ = false;
    RCLCPP_INFO(this->get_logger(), "Clearing active goal and publishing empty path.");
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        empty_path.header.frame_id = map_ ? map_->header.frame_id : "world";
    }
    path_pub_->publish(empty_path);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
