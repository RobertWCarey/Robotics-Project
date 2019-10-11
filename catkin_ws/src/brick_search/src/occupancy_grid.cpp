#include "brick_search/occupancy_grid.h"

namespace astar_path_planner
{
OccupancyGrid::OccupancyGrid(const nav_msgs::OccupancyGrid& map, const double inflation_radius)
{
  // Copy the occupancy grid message
  map_ = map;

  // Access occupancy grid message data with an image
  map_image_ = cv::Mat(map.info.height, map.info.width, CV_8U, &map_.data.front());

  // Dilate the image
  int element_diameter =
      2 * static_cast<int>(std::round(inflation_radius / map.info.resolution)) + 1;  // element_diameter is always odd

  int offset = (element_diameter - 1) / 2;  // Centre of the element

  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(element_diameter, element_diameter), cv::Size(offset, offset));

  cv::dilate(map_image_, map_image_, element);

  // Map geometry for particle filter
  map_x_min_ = map_.info.origin.position.x;
  map_x_max_ = map_.info.width * map_.info.resolution + map_.info.origin.position.x;

  map_y_min_ = map_.info.origin.position.y;
  map_y_max_ = map_.info.height * map_.info.resolution + map_.info.origin.position.y;
}

bool OccupancyGrid::isOutOfBounds(GridPosition grid_position)
{
  return grid_position.x < 0 || grid_position.x > map_image_.cols ||  //
         grid_position.y < 0 || grid_position.y > map_image_.rows;
}

bool OccupancyGrid::isOutOfBounds(WorldPosition world_position)
{
  return world_position.x < map_x_min_ || world_position.x > map_x_max_ ||  //
         world_position.y < map_y_min_ || world_position.y > map_y_max_;
}

bool OccupancyGrid::isOccupied(int id)
{
  return map_.data[id] != 0;
}

bool OccupancyGrid::isOccupied(GridPosition grid_position)
{
  return isOccupied(getCellId(grid_position));
}

bool OccupancyGrid::isOccupied(WorldPosition world_position)
{
  return isOccupied(getGridPosition(world_position));
}

GridPosition OccupancyGrid::getGridPosition(int id)
{
  return { id % static_cast<int>(map_.info.width), id / static_cast<int>(map_.info.width) };
}

GridPosition OccupancyGrid::getGridPosition(WorldPosition world_position)
{
  GridPosition grid_position{};

  grid_position.x =
      static_cast<int>(std::floor((world_position.x - map_.info.origin.position.x) / map_.info.resolution));

  grid_position.y =
      static_cast<int>(std::floor((world_position.y - map_.info.origin.position.y) / map_.info.resolution));

  return grid_position;
}

WorldPosition OccupancyGrid::getWorldPosition(GridPosition grid_position)
{
  WorldPosition world_position{};

  world_position.x = static_cast<double>(grid_position.x) * map_.info.resolution + map_.info.origin.position.x +
                     (map_.info.resolution / 2.);
  world_position.y = static_cast<double>(grid_position.y) * map_.info.resolution + map_.info.origin.position.y +
                     (map_.info.resolution / 2.);

  return world_position;
}

WorldPosition OccupancyGrid::getWorldPosition(int id)
{
  return getWorldPosition(getGridPosition(id));
}

int OccupancyGrid::getCellId(GridPosition grid_position)
{
  return grid_position.y * static_cast<int>(map_.info.width) + grid_position.x;
}

Cell OccupancyGrid::getCell(int id)
{
  Cell cell{};

  cell.id = id;
  cell.occupied = isOccupied(id);
  cell.grid_position = getGridPosition(id);
  cell.world_position = getWorldPosition(cell.grid_position);

  return cell;
}

Cell OccupancyGrid::getCell(GridPosition grid_position)
{
  return getCell(getCellId(grid_position));
}

Cell OccupancyGrid::getCell(WorldPosition world_position)
{
  return getCell(getGridPosition(world_position));
}

nav_msgs::OccupancyGrid OccupancyGrid::getOccupancyGridMsg()
{
  return map_;
}

std::vector<AdjacentCell> OccupancyGrid::getAdjacentCells(int id, bool diagonal_movement)
{
  // Return the unoccupied cells adjacent to "id"

  // Grid position of the given cell, use this to get adjacent cell grid positions
  GridPosition grid_position = getGridPosition(id);

  // Fill this with adjacent cells
  std::vector<AdjacentCell> adjacent_cells{};

  // Use "isOutOfBounds" and "isOccupied" to check if the adjacent cells are out of bounds or occupied
  // The "AdjacentCell" structure has three fields: "id", "cost", and "world_position"
  // Use "getCellId" and "getWorldPosition" to get a cell ID and world position from a grid position
  // "cost" is the cost of moving from the parent to the adjacent cell in metres
  // "map_.info.resolution" is the distance between cells
  // Only return diagonal cells if "diagonal_movement" is true
  // Keep in mind that the distance between diagonal cells is larger than horizontal/vertical cells

  GridPosition gridpos;
  int adj_id;
  AdjacentCell adjcell;

  if (diagonal_movement == true)
  {
    for (int x = grid_position.x - 1; x <= grid_position.x + 1; x++)
    {
      for (int y = grid_position.y - 1; y <= grid_position.y + 1; y++)
      {
        if ( !(y == grid_position.y && x == grid_position.x))
        {
          gridpos.x = x;
          gridpos.y = y;

          adj_id = getCellId(gridpos);

          if (!isOccupied(adj_id) && !isOutOfBounds(gridpos))
          {
            adjcell.id = adj_id;

            adjcell.world_position = getWorldPosition(adjcell.id);

            adjcell.cost = std::sqrt(std::pow((grid_position.x - gridpos.x) * map_.info.resolution, 2) + std::pow((grid_position.y - gridpos.y) * map_.info.resolution, 2));

            adjacent_cells.push_back(adjcell);
          }
        }
      }
    }
  }

  else
  {
    int x = 0;
    int y = 0;
    for (x = grid_position.x - 1; x <= grid_position.x + 1; x += 2)
    {
      gridpos.y = grid_position.y;
      gridpos.x = x;
      adj_id = getCellId(gridpos);
      if (!isOccupied(adj_id) && !isOutOfBounds(gridpos))
        {
          adjcell.id = adj_id;

          adjcell.world_position = getWorldPosition(adjcell.id);

          adjcell.cost = map_.info.resolution;

          adjacent_cells.push_back(adjcell);
        }
    }

    for (y = grid_position.y - 1; y <= grid_position.y + 1; y += 2)
    {
      gridpos.x = grid_position.x;
      gridpos.y = y;

      adj_id = getCellId(gridpos);
      if (!isOccupied(adj_id) && !isOutOfBounds(gridpos))
      {
        adjcell.id = adj_id;

        adjcell.world_position = getWorldPosition(adjcell.id);

        adjcell.cost = map_.info.resolution;

        adjacent_cells.push_back(adjcell);
      }
    }

  }

  return adjacent_cells;
}

}  // namespace astar_path_planner
