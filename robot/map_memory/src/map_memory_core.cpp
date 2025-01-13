#include "map_memory_core.hpp"

namespace robot 
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
    : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void MapMemoryCore::configureMapMemory(
    double grid_resolution, 
    int grid_width, 
    int grid_height, 
    const geometry_msgs::msg::Pose& origin_pose) {
    global_map_->info.resolution = grid_resolution;
    global_map_->info.width = grid_width;
    global_map_->info.height = grid_height;
    global_map_->info.origin = origin_pose;
    global_map_->data.assign(grid_width * grid_height, 0);
    RCLCPP_INFO(logger_, "Map initialized with resolution: %.2f, width: %d, height: %d", 
                grid_resolution, grid_width, grid_height);
}

void MapMemoryCore::refreshMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map,
    double robot_x, double robot_y, double robot_orientation) {
    // Retrieve local map attributes
    double local_res = local_map->info.resolution;
    double local_origin_x = local_map->info.origin.position.x;
    double local_origin_y = local_map->info.origin.position.y;
    unsigned int local_width = local_map->info.width;
    unsigned int local_height = local_map->info.height;
    const auto& local_data = local_map->data;

    for (unsigned int row = 0; row < local_height; ++row) {
        for (unsigned int col = 0; col < local_width; ++col) {
            int8_t local_cell_value = local_data[row * local_width + col];
            if (local_cell_value < 0) {
                // Skip unknown values
                continue;
            }
            // Convert local grid cell to metric coordinates
            double local_x = local_origin_x + (col + 0.5) * local_res;
            double local_y = local_origin_y + (row + 0.5) * local_res;

            // Transform local coordinates to global coordinates
            double cos_theta = std::cos(robot_orientation);
            double sin_theta = std::sin(robot_orientation);
            double global_x = robot_x + (local_x * cos_theta - local_y * sin_theta);
            double global_y = robot_y + (local_x * sin_theta + local_y * cos_theta);

            // Convert global coordinates to global map indices
            int global_idx_x, global_idx_y;
            if (!convertToMapIndex(global_x, global_y, global_idx_x, global_idx_y)) {
                // Out of bounds
                continue;
            }

            // Update the global map using max cost strategy
            int8_t& global_cell_value = global_map_->data[global_idx_y * global_map_->info.width + global_idx_x];
            int current_global_cost = (global_cell_value < 0) ? 0 : global_cell_value;
            int merged_cost = std::max(current_global_cost, static_cast<int>(local_cell_value));
            global_cell_value = static_cast<int8_t>(merged_cost);
        }
    }
}

bool MapMemoryCore::convertToMapIndex(double wx, double wy, int& mx, int& my) const {
    double origin_x = global_map_->info.origin.position.x;
    double origin_y = global_map_->info.origin.position.y;
    double resolution = global_map_->info.resolution;

    if (wx < origin_x || wy < origin_y) {
        return false;
    }

    mx = static_cast<int>((wx - origin_x) / resolution);
    my = static_cast<int>((wy - origin_y) / resolution);

    return mx >= 0 && mx < static_cast<int>(global_map_->info.width) &&
           my >= 0 && my < static_cast<int>(global_map_->info.height);
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::retrieveMap() const {
    return global_map_;
}

}  // namespace robot_system
