#include <chrono>
#include <memory>
#include <algorithm>
#include <queue>
#include <cmath>
#include <utility>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap") {
  costmap_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  
  origin_.position.x = -24.0; 
  origin_.position.y = -24.0; 
  origin_.orientation.w = 1.0;
  
  setupCommunication();
  initializeCostmap();
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  updateCostmap(msg);
  costmap_data_->header = msg->header;
  costmap_pub_->publish(*costmap_data_);
}

void CostmapNode::setupCommunication() {
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, 10, 
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, 10);
    
  RCLCPP_INFO(this->get_logger(), "ROS communication setup completed");
}
 
void CostmapNode::initializeCostmap() {
  costmap_data_->info.resolution = resolution_;
  costmap_data_->info.width = width_;
  costmap_data_->info.height = height_;
  costmap_data_->info.origin = origin_;
  costmap_data_->data.assign(width_ * height_, -1);
  
  inflation_cells_ = static_cast<int>(inflation_radius_ / resolution_);
  
  RCLCPP_INFO(this->get_logger(), "Costmap initialized with resolution: %.2f, width: %d, height: %d", 
              resolution_, width_, height_);
}

void CostmapNode::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) {
  auto& grid_data = costmap_data_->data;
  std::fill(grid_data.begin(), grid_data.end(), 0);
  const double min_valid_range = laserscan->range_min;
  const double max_valid_range = laserscan->range_max;
  const int grid_width = static_cast<int>(costmap_data_->info.width);
  const int grid_height = static_cast<int>(costmap_data_->info.height);
  const auto& grid_origin = costmap_data_->info.origin.position;
  const double cell_size = costmap_data_->info.resolution;
  const std::vector<float>& ranges = laserscan->ranges;
  const size_t num_rays = ranges.size();
  for (size_t i = 0; i < num_rays; ++i) {
    const double beam_angle = laserscan->angle_min + i * laserscan->angle_increment;
    const double range = ranges[i];
    
    if (range < min_valid_range || range > max_valid_range) {
      continue;
    }
    const double world_x = range * std::cos(beam_angle);
    const double world_y = range * std::sin(beam_angle);
    const int cell_x = static_cast<int>((world_x - grid_origin.x) / cell_size);
    const int cell_y = static_cast<int>((world_y - grid_origin.y) / cell_size);
    if (cell_x >= 0 && cell_x < grid_width && cell_y >= 0 && cell_y < grid_height) {
      const int cell_index = cell_y * grid_width + cell_x;
      grid_data[cell_index] = 100;
      inflateObstacle(cell_x, cell_y);
    }
  }
}

void CostmapNode::inflateObstacle(int origin_x, int origin_y) {
  const int max_x = static_cast<int>(costmap_data_->info.width);
  const int max_y = static_cast<int>(costmap_data_->info.height);
  auto& grid_data = costmap_data_->data;
  std::vector<std::vector<bool>> processed(max_x, std::vector<bool>(max_y, false));
  processed[origin_x][origin_y] = true;
  std::queue<std::pair<int, int>> flood_queue;
  flood_queue.push({origin_x, origin_y});
  
  const std::vector<std::pair<int, int>> directions = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},           {0, 1},
    {1, -1},  {1, 0},  {1, 1}
  };
  while (!flood_queue.empty()) {
    const auto [current_x, current_y] = flood_queue.front();
    flood_queue.pop();
    
    for (const auto& [dx, dy] : directions) {
      const int neighbor_x = current_x + dx;
      const int neighbor_y = current_y + dy;
      
      if (neighbor_x < 0 || neighbor_x >= max_x || 
          neighbor_y < 0 || neighbor_y >= max_y ||
          processed[neighbor_x][neighbor_y]) {
        continue;
      }
      
      processed[neighbor_x][neighbor_y] = true;
      
      const double dist_squared = 
          (neighbor_x - origin_x) * (neighbor_x - origin_x) + 
          (neighbor_y - origin_y) * (neighbor_y - origin_y);
      const double distance = std::sqrt(dist_squared) * resolution_;
      if (distance <= inflation_radius_) {
        const int cell_index = neighbor_y * max_x + neighbor_x;
        const double normalized_distance = distance / inflation_radius_;
        const int cost_value = static_cast<int>((1.0 - normalized_distance) * 100);
        
        if (grid_data[cell_index] < cost_value) {
          grid_data[cell_index] = cost_value;
        }
        
        flood_queue.push({neighbor_x, neighbor_y});
      }
    }
  }
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
