#include "brick_search/occupancy_grid.h"

namespace brick_search
{
OccupancyGrid::OccupancyGrid(const nav_msgs::OccupancyGrid& map, const double inflation_radius)
{
  // Copy the occupancy grid message
  map_ = map;

  // Access occupancy grid message data with an image
  map_image_ = cv::Mat(map.info.height, map.info.width, CV_8U, &map_.data.front());

  // Dilate the image
  int element_diameter =
      2 * static_cast<int>(std::round(inflation_radius / map.info.resolution)) + 1; // element_diameter is always odd

  int offset = (element_diameter - 1) / 2; // Centre of the element

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
  return grid_position.x < 0 || grid_position.x > map_image_.cols || //
         grid_position.y < 0 || grid_position.y > map_image_.rows;
}

bool OccupancyGrid::isOccupied(int id)
{
  return map_.data[id] != 0;
}

bool OccupancyGrid::isOccupied(GridPosition grid_position)
{
  return isOccupied(getCellId(grid_position));
}

GridPosition OccupancyGrid::getGridPosition(int id)
{
  return {id % static_cast<int>(map_.info.width), id / static_cast<int>(map_.info.width)};
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

  return cell;
}

Cell OccupancyGrid::getCell(GridPosition grid_position)
{
  return getCell(getCellId(grid_position));
}

nav_msgs::OccupancyGrid OccupancyGrid::getOccupancyGridMsg()
{
  return map_;
}
}  // namespace brick_search
