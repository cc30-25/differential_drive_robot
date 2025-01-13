#ifndef PATH_PLANNER_CORE_HPP_
#define PATH_PLANNER_CORE_HPP_

#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <unordered_map>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace path_planner
{

// Represents a 2D cell in a grid
struct GridCell
{
    int row;
    int col;
    GridCell(int r, int c) : row(r), col(c) {}
    GridCell() : row(0), col(0) {}
    bool operator==(const GridCell &other) const
    {
        return (row == other.row && col == other.col);
    }
    bool operator!=(const GridCell &other) const
    {
        return (row != other.row || col != other.col);
    }
};

// Custom hash function 
struct GridCellHash
{
    std::size_t operator()(const GridCell &cell) const
    {
        return std::hash<int>()(cell.row) ^ (std::hash<int>()(cell.col) << 1);
    }
};

// Represents a node in the open set for A* 
struct SearchNode
{
    GridCell cell;
    double total_cost;  // g + h
    SearchNode(GridCell c, double cost) : cell(c), total_cost(cost) {}
};

// Custom comparator for A* priority queue 
struct CompareCost
{
    bool operator()(const SearchNode &a, const SearchNode &b)
    {
        return a.total_cost > b.total_cost;
    }
};

class PathPlannerCore {
public:
    explicit PathPlannerCore(const rclcpp::Logger &node_logger);

    void configurePlanner(double smoothing_factor, int max_iterations);
    bool calculatePath(
        double start_x, 
        double start_y, 
        double goal_x, 
        double goal_y, 
        nav_msgs::msg::OccupancyGrid::SharedPtr map
    );
    bool executeAStar(
        const GridCell &start, 
        const GridCell &goal, 
        std::vector<GridCell> &result_path
    );
    void generatePathFromMap(
        const std::unordered_map<GridCell, GridCell, GridCellHash> &parent_map, 
        const GridCell &current, 
        std::vector<GridCell> &result_path
    );
    std::vector<GridCell> find8Neighbors(const GridCell &cell);
    double calculateEuclideanDistance(const GridCell &a, const GridCell &b);
    double computeStepCost(const GridCell &start, const GridCell &end);
    void applyPathSmoothing(std::vector<GridCell> &path_points);
    bool convertWorldToGrid(double world_x, double world_y, GridCell &grid_cell);
    void convertGridToWorld(const GridCell &grid_cell, double &world_x, double &world_y);
    bool checkLineOfSight(const GridCell &start, const GridCell &end);
    bool isGridCellFree(int x, int y);
    nav_msgs::msg::Path::SharedPtr retrievePath() const;

private:
    double smoothing_weight_;
    int max_iterations_;
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
    nav_msgs::msg::Path::SharedPtr planned_path_;
};

} 

#endif 
