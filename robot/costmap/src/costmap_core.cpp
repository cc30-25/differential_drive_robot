#include <algorithm>
#include <queue>
#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_ptr_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void CostmapCore::initializeMap(double map_resolution, int map_width, int map_height, 
    geometry_msgs::msg::Pose map_origin, double inflation_radius) {
  costmap_ptr_->info.resolution = map_resolution;
  costmap_ptr_->info.width = map_width;
  costmap_ptr_->info.height = map_height;
  costmap_ptr_->info.origin = map_origin;
  costmap_ptr_->data.assign(map_width * map_height, -1);
  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(inflation_radius / map_resolution);
  RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", map_resolution, map_width, map_height);
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) const {
  // Reset all cells to free space
  std::fill(costmap_ptr_->data.begin(), costmap_ptr_->data.end(), 0);
  double angle = laser_msg->angle_min;
  
  for (size_t i = 0; i < laser_msg->ranges.size(); ++i, angle += laser_msg->angle_increment) {
    double range = laser_msg->ranges[i];
    // Check if the range value is valid
    if (range >= laser_msg->range_min && range <= laser_msg->range_max) {
      // Calculate the position of the obstacle in map coordinates
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);
      // Convert to grid coordinates
      int grid_x = static_cast<int>((x - costmap_ptr_->info.origin.position.x) / costmap_ptr_->info.resolution);
      int grid_y = static_cast<int>((y - costmap_ptr_->info.origin.position.y) / costmap_ptr_->info.resolution);
      
      if (grid_x >= 0 && grid_x < static_cast<int>(costmap_ptr_->info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(costmap_ptr_->info.height)) {
        // Mark the cell as occupied
        int index = grid_y * costmap_ptr_->info.width + grid_x;
        costmap_ptr_->data[index] = 100;  // 100 indicates an occupied cell
        // Inflate around the obstacle
        applyInflation(grid_x, grid_y);
      }
    }
  }
}

void CostmapCore::applyInflation(int origin_x, int origin_y) const {
  // Use breadth-first search (BFS) to mark cells within the inflation radius
  std::queue<std::pair<int, int>> bfs_queue;
  bfs_queue.emplace(origin_x, origin_y);
  std::vector<std::vector<bool>> visited(costmap_ptr_->info.width, std::vector<bool>(costmap_ptr_->info.height, false));
  visited[origin_x][origin_y] = true;
  
  while (!bfs_queue.empty()) {
    auto [x, y] = bfs_queue.front();
    bfs_queue.pop();
    
    // Iterate over neighboring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // Skip the center cell
        int nx = x + dx;
        int ny = y + dy;
        
        // Ensure the neighbor cell is within bounds
        if (nx >= 0 && nx < static_cast<int>(costmap_ptr_->info.width) &&
            ny >= 0 && ny < static_cast<int>(costmap_ptr_->info.height) &&
            !visited[nx][ny]) {
          // Calculate the distance to the original obstacle
          double dist = std::hypot(nx - origin_x, ny - origin_y) * costmap_ptr_->info.resolution;
          // If within inflation radius, mark as inflated and add to BFS queue
          if (dist <= inflation_radius_) {
            int index = ny * costmap_ptr_->info.width + nx;
            if (costmap_ptr_->data[index] < (1 - (dist / inflation_radius_)) * 100) {
              costmap_ptr_->data[index] = (1 - (dist / inflation_radius_)) * 100;
            }
            bfs_queue.emplace(nx, ny);
          }
          visited[nx][ny] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::retrieveCostmap() const {
  return costmap_ptr_;
}

}  // namespace robot
