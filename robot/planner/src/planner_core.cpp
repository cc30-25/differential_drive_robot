#include "planner_core.hpp"

namespace robot
{

PathPlannerCore::PathPlannerCore(const rclcpp::Logger &node_logger)
    : logger_(node_logger),
      path_(std::make_shared<nav_msgs::msg::Path>()),
      map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void PathPlannerCore::configurePlanner(double smoothing_weight, int max_iterations)
{
    smoothing_weight_ = smoothing_weight;
    max_iterations_ = max_iterations;
}

bool PathPlannerCore::calculatePath(
    double start_x, double start_y, double goal_x, double goal_y,
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map)
{
    map_ = occupancy_map;

    GridCell goal_cell;
    if (!convertWorldToGrid(goal_x, goal_y, goal_cell))
    {
        RCLCPP_WARN(logger_, "Goal is outside the map bounds. Path planning aborted.");
        return false;
    }

    GridCell start_cell;
    if (!convertWorldToGrid(start_x, start_y, start_cell))
    {
        RCLCPP_WARN(logger_, "Start position is outside the map bounds. Path planning aborted.");
        return false;
    }

    RCLCPP_INFO(logger_,
                "Path planning initiated: Start (%.2f, %.2f) mapped to cell (%d, %d); "
                "Goal mapped to cell (%d, %d).",
                start_x, start_y, start_cell.row, start_cell.col,
                goal_cell.row, goal_cell.col);

    // Perform A* search
    std::vector<GridCell> planned_path_cells;
    if (!executeAStar(start_cell, goal_cell, planned_path_cells))
    {
        RCLCPP_WARN(logger_, "A* algorithm failed to compute a valid path.");
        return false;
    }

    // Clear and convert to ROS Path message
    path_->poses.clear();
    for (const auto &cell : planned_path_cells)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map_->header;
        double world_x, world_y;
        convertGridToWorld(cell, world_x, world_y);
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.orientation.w = 1.0; // Default orientation
        path_->poses.push_back(pose);
    }

    return true;
}

bool PathPlannerCore::executeAStar(
    const GridCell &start_cell, const GridCell &goal_cell,
    std::vector<GridCell> &result_path)
{
    const int map_width = map_->info.width;
    const int map_height = map_->info.height;

    std::unordered_map<GridCell, double, GridCellHash> g_costs;
    std::unordered_map<GridCell, GridCell, GridCellHash> parent_map;
    std::unordered_map<GridCell, double, GridCellHash> f_costs;

    auto setCost = [](auto &storage, const GridCell &cell, double value)
    { storage[cell] = value; };

    auto getCost = [](const auto &storage, const GridCell &cell)
    {
        auto it = storage.find(cell);
        return (it != storage.end()) ? it->second : std::numeric_limits<double>::infinity();
    };

    auto gridCellCost = [&](const GridCell &cell)
    {
        if (cell.row < 0 || cell.row >= map_width || cell.col < 0 || cell.col >= map_height)
        {
            return 127; // Treat out-of-bounds cells as high-cost
        }
        int index = cell.col * map_width + cell.row;
        int8_t cost = map_->data[index];
        return (cost < 0) ? 100 : static_cast<int>(cost); // Default cost for unknown cells
    };

    setCost(g_costs, start_cell, 0.0);
    double initial_heuristic = calculateEuclideanDistance(start_cell, goal_cell);
    setCost(f_costs, start_cell, initial_heuristic);

    std::priority_queue<SearchNode, std::vector<SearchNode>, CompareCost> open_set;
    open_set.emplace(start_cell, initial_heuristic);

    while (!open_set.empty())
    {
        SearchNode current_node = open_set.top();
        open_set.pop();
        GridCell current_cell = current_node.cell;

        if (current_cell == goal_cell)
        {
            generatePathFromMap(parent_map, current_cell, result_path);
            return true;
        }

        double current_g = getCost(g_costs, current_cell);
        for (const auto &neighbor : find8Neighbors(current_cell))
        {
            if (neighbor.row < 0 || neighbor.row >= map_width || neighbor.col < 0 || neighbor.col >= map_height)
                continue;

            int cell_cost = gridCellCost(neighbor);
            if (cell_cost > 90)
                continue; // Treat as obstacle

            double step_cost = computeStepCost(current_cell, neighbor);
            double penalty = static_cast<double>(cell_cost) / 25.0;
            double tentative_g = current_g + step_cost + penalty;

            if (tentative_g < getCost(g_costs, neighbor))
            {
                setCost(g_costs, neighbor, tentative_g);
                double heuristic = calculateEuclideanDistance(neighbor, goal_cell);
                double total_cost = tentative_g + heuristic;
                setCost(f_costs, neighbor, total_cost);
                parent_map[neighbor] = current_cell;
                open_set.emplace(neighbor, total_cost);
            }
        }
    }

    return false; // No path found
}

void PathPlannerCore::generatePathFromMap(
    const std::unordered_map<GridCell, GridCell, GridCellHash> &parent_map,
    const GridCell &current_cell, std::vector<GridCell> &result_path)
{
    result_path.clear();
    GridCell cell = current_cell;
    result_path.push_back(cell);

    auto it = parent_map.find(cell);
    while (it != parent_map.end())
    {
        cell = it->second;
        result_path.push_back(cell);
        it = parent_map.find(cell);
    }
    std::reverse(result_path.begin(), result_path.end());
}

std::vector<GridCell> PathPlannerCore::find8Neighbors(const GridCell &cell)
{
    std::vector<GridCell> neighbors;
    neighbors.reserve(8);
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx != 0 || dy != 0)
            {
                neighbors.emplace_back(cell.row + dx, cell.col + dy);
            }
        }
    }
    return neighbors;
}

double PathPlannerCore::calculateEuclideanDistance(const GridCell &a, const GridCell &b)
{
    double dx = static_cast<double>(a.row - b.row);
    double dy = static_cast<double>(a.col - b.col);
    return std::sqrt(dx * dx + dy * dy);
}

double PathPlannerCore::computeStepCost(const GridCell &start, const GridCell &end)
{
    int dx = std::abs(start.row - end.row);
    int dy = std::abs(start.col - end.col);
    return (dx + dy == 2) ? std::sqrt(2.0) : 1.0; // Diagonal or orthogonal step
}

bool PathPlannerCore::convertWorldToGrid(double world_x, double world_y, GridCell &grid_cell)
{
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;
    int grid_x = static_cast<int>((world_x - origin_x) / resolution);
    int grid_y = static_cast<int>((world_y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= static_cast<int>(map_->info.width) ||
        grid_y < 0 || grid_y >= static_cast<int>(map_->info.height))
    {
        return false;
    }

    grid_cell.row = grid_x;
    grid_cell.col = grid_y;
    return true;
}

void PathPlannerCore::convertGridToWorld(const GridCell &grid_cell, double &world_x, double &world_y)
{
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;
    world_x = origin_x + (grid_cell.row + 0.5) * resolution;
    world_y = origin_y + (grid_cell.col + 0.5) * resolution;
}

nav_msgs::msg::Path::SharedPtr PathPlannerCore::retrievePath() const
{
    return path_;
}

} 
