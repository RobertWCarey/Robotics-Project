// An undirected graph

#ifndef brick_search_OCCUPANCY_GRID_H
#define brick_search_OCCUPANCY_GRID_H

#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/OccupancyGrid.h>

namespace brick_search
{
struct WorldPosition
{
  double x = 0.;
  double y = 0.;
};

struct GridPosition
{
  int x = 0;
  int y = 0;
};

struct Cell
{
  int id = -1;
  bool occupied = true;
  GridPosition grid_position{};
  WorldPosition world_position{};  // The world position of the centre of the cell
};

struct AdjacentCell
{
  int id = -1;
  double cost = 0.;                // Cost of moving from the parent in metres
  WorldPosition world_position{};  // The world position of the centre of the cell
};

class OccupancyGrid
{
public:
  // Constructors
  OccupancyGrid() = default;
  OccupancyGrid(const nav_msgs::OccupancyGrid& map, double inflation_radius);

  // Public methods
  bool isOutOfBounds(WorldPosition world_position);
  bool isOccupied(WorldPosition world_position);
  bool isOccupied(GridPosition grid_position);
  WorldPosition getWorldPosition(int id);  // World position of the centre of the cell
  Cell getCell(WorldPosition world_position);
  Cell getCell(int id);
  nav_msgs::OccupancyGrid getOccupancyGridMsg();  // This is for publishing the inflated map

  // Return the unoccupied cells adjacent to "id"
  std::vector<AdjacentCell> getAdjacentCells(int id, bool diagonal_movement);

private:
  nav_msgs::OccupancyGrid map_{};
  cv::Mat map_image_{};

  // Limits of the map in metres
  double map_x_min_ = 0., map_x_max_ = 0., map_y_min_ = 0., map_y_max_ = 0.;

  // Private methods
  bool isOutOfBounds(GridPosition grid_position);
  bool isOccupied(int id);
  
  GridPosition getGridPosition(WorldPosition world_position);
  GridPosition getGridPosition(int id);
  WorldPosition getWorldPosition(GridPosition grid_position);  // World position of the centre of the cell
  int getCellId(GridPosition grid_position);
  Cell getCell(GridPosition grid_position);
};

}  // namespace brick_search

#endif  // brick_search_OCCUPANCY_GRID_H
