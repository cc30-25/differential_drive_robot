#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // Initializes core control parameters
    void initializeControlCore(
      double lookahead_distance,
      double max_steering_angle, 
      double steering_gain,
      double linear_velocity
    );

    // Updates the current path
    void updatePath(nav_msgs::msg::Path path);

    // Checks if the path is empty
    bool isPathEmpty();

    // Finds the next lookahead point in the path
    unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);

    // Calculates control commands based on the current robot state
    geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta);

  private:
    nav_msgs::msg::Path path_;  // Current path for robot to follow
    rclcpp::Logger logger_;     // Logger for debugging and terminal outputs

    // Control parameters
    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
};

}  // namespace robot

#endif
