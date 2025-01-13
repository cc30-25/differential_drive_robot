#include "control_core.hpp"
#include <cmath>

namespace robot {

ControlEngine::ControlEngine(const rclcpp::Logger& logger) 
  : trajectory_(nav_msgs::msg::Path()), logger_(logger) {}

void ControlEngine::initializeControlSystem(
  double lookahead_distance,
  double max_steering_angle,
  double steering_gain,
  double linear_velocity
) {
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  steering_gain_ = steering_gain;
  linear_velocity_ = linear_velocity;
}

void ControlEngine::updateTrajectory(nav_msgs::msg::Path new_path) {
  RCLCPP_INFO(logger_, "Trajectory Updated");
  trajectory_ = new_path;
}

bool ControlEngine::isTrajectoryEmpty() {
  return trajectory_.poses.empty();
}

geometry_msgs::msg::Twist ControlEngine::computeControlCommand(double robot_x, double robot_y, double robot_theta) {
  geometry_msgs::msg::Twist cmd_vel;
  unsigned int target_index = findTargetPoint(robot_x, robot_y, robot_theta);
  
  if (target_index >= trajectory_.poses.size()) {
    return cmd_vel;
  }

  // Get the target point coordinates
  double target_x = trajectory_.poses[target_index].pose.position.x;
  double target_y = trajectory_.poses[target_index].pose.position.y;
  
  // Calculate the distance to the target point
  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  // Calculate the angle to the target point
  double angle_to_target = std::atan2(dy, dx);

  // Calculate the steering angle (difference between robot's heading and target direction)
  double steering_angle = angle_to_target - robot_theta;

  // Normalize the steering angle to the range [-pi, pi] for sharp turns
  if (steering_angle > M_PI) {
    steering_angle -= 2 * M_PI;
  } else if (steering_angle < -M_PI) {
    steering_angle += 2 * M_PI;
  }

  // If the steering angle exceeds the max, we don't move forward
  if (std::abs(steering_angle) > std::abs(max_steering_angle_)) {
    cmd_vel.linear.x = 0;
  } else {
    cmd_vel.linear.x = linear_velocity_;
  }

  // Limit the steering angle
  steering_angle = std::max(-max_steering_angle_, std::min(steering_angle, max_steering_angle_));

  // Set the angular velocity according to steering gain
  cmd_vel.angular.z = steering_angle * steering_gain_;
  
  return cmd_vel;
}

unsigned int ControlEngine::findTargetPoint(double robot_x, double robot_y, double robot_theta) {
  double closest_distance = std::numeric_limits<double>::max();
  int target_index = 0;
  bool forward_target_found = false;

  // Loop through all path points to find the closest target point
  for (size_t i = 0; i < trajectory_.poses.size(); ++i) {
    double dx = trajectory_.poses[i].pose.position.x - robot_x;
    double dy = trajectory_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Skip points that are too close
    if (distance < lookahead_distance_) {
      continue;
    }

    // Calculate the angle to the target point
    double angle_to_target = std::atan2(dy, dx);

    // Calculate the difference between the robot's heading and the angle to the target point
    double angle_diff = angle_to_target - robot_theta;
    
    // Normalize the angle difference to be within [-pi, pi]
    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Check if the angle to the point is within the forward-facing direction range (-π/2 to π/2)
    if (std::abs(angle_diff) < M_PI / 2) {
      // We found a valid forward point, update the closest point
      if (distance < closest_distance) {
        closest_distance = distance;
        target_index = i;
        forward_target_found = true;
      }
    }
  }

  // If no forward point was found, allow reverse direction
  if (!forward_target_found) {
    // Find the closest point regardless of direction
    for (size_t i = 0; i < trajectory_.poses.size(); ++i) {
      double dx = trajectory_.poses[i].pose.position.x - robot_x;
      double dy = trajectory_.poses[i].pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);

      // Skip points that are too close
      if (distance < lookahead_distance_) {
        continue;
      }

      // Always select the closest point if no valid forward point was found
      if (distance < closest_distance) {
        closest_distance = distance;
        target_index = i;
      }
    }
  }

  return target_index;
}

}  // namespace robot
