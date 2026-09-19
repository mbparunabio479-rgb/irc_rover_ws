#include "irc_rover_terrain_costmap/terrain_costmap_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace irc_rover_terrain_costmap
{

TerrainCostmapLayer::TerrainCostmapLayer()
: enabled_(true),
  current_(false),
  min_x_(0.0),
  min_y_(0.0),
  max_x_(0.0),
  max_y_(0.0)
{
}

void TerrainCostmapLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "input_topic",
    rclcpp::ParameterValue("/terrain/traversability"));

  declareParameter(
    "unknown_cost",
    rclcpp::ParameterValue(0.0));

  declareParameter(
    "difficult_cost",
    rclcpp::ParameterValue(150.0));

  declareParameter(
    "non_traversable_cost",
    rclcpp::ParameterValue(254.0));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".input_topic", input_topic_);
  node->get_parameter(name_ + ".unknown_cost", unknown_cost_);
  node->get_parameter(name_ + ".difficult_cost", difficult_cost_);
  node->get_parameter(name_ + ".non_traversable_cost", non_traversable_cost_);

  terrain_sub_ =
    node->create_subscription<grid_map_msgs::msg::GridMap>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &TerrainCostmapLayer::terrainCallback,
      this,
      std::placeholders::_1));

  current_ = false;

  RCLCPP_INFO(
    node->get_logger(),
    "Terrain costmap layer initialized");

  RCLCPP_INFO(
    node->get_logger(),
    "Input: %s",
    input_topic_.c_str());
}

void TerrainCostmapLayer::terrainCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  latest_map_ = std::make_shared<grid_map_msgs::msg::GridMap>(*msg);

  if (msg->info.length_x > 0.0 && msg->info.length_y > 0.0) {
    const double cx = msg->info.pose.position.x;
    const double cy = msg->info.pose.position.y;

    min_x_ = cx - msg->info.length_x * 0.5;
    max_x_ = cx + msg->info.length_x * 0.5;
    min_y_ = cy - msg->info.length_y * 0.5;
    max_y_ = cy + msg->info.length_y * 0.5;
  }

  current_ = true;
}

void TerrainCostmapLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!latest_map_) {
    return;
  }

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
}

void TerrainCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!latest_map_) {
    return;
  }

  /*
   * Phase 12 initial implementation:
   *
   * The GridMap contains:
   *   terrain_class = 0  UNKNOWN
   *   terrain_class = 1  TRAVERSABLE
   *   terrain_class = 2  DIFFICULT
   *   terrain_class = 3  NON_TRAVERSABLE
   *
   * We currently use the GridMap's terrain_class layer.
   *
   * This first implementation intentionally does NOT modify the
   * terrain classification thresholds. Those remain part of the
   * terrain/traversability pipeline and will be calibrated later
   * on the actual rover.
   */

  int terrain_class_index = -1;

  for (size_t i = 0; i < latest_map_->layers.size(); ++i) {
    if (latest_map_->layers[i] == "terrain_class") {
      terrain_class_index = static_cast<int>(i);
      break;
    }
  }

  if (terrain_class_index < 0) {
    return;
  }

  if (latest_map_->data.empty()) {
    return;
  }

  const auto & array = latest_map_->data[terrain_class_index];

  const unsigned int rows = latest_map_->info.resolution > 0.0
    ? static_cast<unsigned int>(
        std::round(latest_map_->info.length_y /
                   latest_map_->info.resolution))
    : 0;

  const unsigned int cols = latest_map_->info.resolution > 0.0
    ? static_cast<unsigned int>(
        std::round(latest_map_->info.length_x /
                   latest_map_->info.resolution))
    : 0;

  if (rows == 0 || cols == 0) {
    return;
  }

  if (array.data.size() < rows * cols) {
    return;
  }

  const double resolution = latest_map_->info.resolution;
  const double center_x = latest_map_->info.pose.position.x;
  const double center_y = latest_map_->info.pose.position.y;

  const double origin_x =
    center_x - latest_map_->info.length_x * 0.5;

  const double origin_y =
    center_y - latest_map_->info.length_y * 0.5;

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(max_i, size_x);
  max_j = std::min(max_j, size_y);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {

      double wx;
      double wy;

      master_grid.mapToWorld(
        static_cast<unsigned int>(i),
        static_cast<unsigned int>(j),
        wx,
        wy);

      const int gx =
        static_cast<int>(std::floor((wx - origin_x) / resolution));

      const int gy =
        static_cast<int>(std::floor((wy - origin_y) / resolution));

      if (gx < 0 || gx >= static_cast<int>(cols) ||
          gy < 0 || gy >= static_cast<int>(rows))
      {
        continue;
      }

      const size_t index =
        static_cast<size_t>(gy) * cols +
        static_cast<size_t>(gx);

      const float class_value = array.data[index];

      if (!std::isfinite(class_value)) {
        continue;
      }

      const int terrain_class =
        static_cast<int>(std::round(class_value));

      unsigned char cost = nav2_costmap_2d::FREE_SPACE;

      switch (terrain_class) {

        case 0:
          /*
           * Unknown terrain is NOT automatically made an obstacle.
           * This is important because sparse LiDAR terrain must not
           * create false obstacles.
           */
          continue;

        case 1:
          cost = nav2_costmap_2d::FREE_SPACE;
          break;

        case 2:
          cost = static_cast<unsigned char>(
            std::clamp(
              difficult_cost_,
              0.0,
              254.0));
          break;

        case 3:
          cost = static_cast<unsigned char>(
            std::clamp(
              non_traversable_cost_,
              0.0,
              254.0));
          break;

        default:
          continue;
      }

      const unsigned char old_cost =
        master_grid.getCost(
          static_cast<unsigned int>(i),
          static_cast<unsigned int>(j));

      if (cost > old_cost) {
        master_grid.setCost(
          static_cast<unsigned int>(i),
          static_cast<unsigned int>(j),
          cost);
      }
    }
  }

  current_ = false;
}

void TerrainCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);

  latest_map_.reset();
  current_ = false;
}

bool TerrainCostmapLayer::isClearable()
{
  return true;
}

void TerrainCostmapLayer::activate()
{
  enabled_ = true;
}

void TerrainCostmapLayer::deactivate()
{
  enabled_ = false;
}

}  // namespace irc_rover_terrain_costmap

PLUGINLIB_EXPORT_CLASS(
  irc_rover_terrain_costmap::TerrainCostmapLayer,
  nav2_costmap_2d::Layer)
