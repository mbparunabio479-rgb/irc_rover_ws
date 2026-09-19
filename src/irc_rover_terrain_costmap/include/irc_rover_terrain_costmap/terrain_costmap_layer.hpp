#ifndef IRC_ROVER_TERRAIN_COSTMAP__TERRAIN_COSTMAP_LAYER_HPP_
#define IRC_ROVER_TERRAIN_COSTMAP__TERRAIN_COSTMAP_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "grid_map_msgs/msg/grid_map.hpp"

namespace irc_rover_terrain_costmap
{

class TerrainCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  TerrainCostmapLayer();
  ~TerrainCostmapLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;

  bool isClearable() override;

  void activate() override;
  void deactivate() override;

private:
  void terrainCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg);

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;

  std::mutex mutex_;

  std::shared_ptr<grid_map_msgs::msg::GridMap> latest_map_;

  std::string input_topic_;

  double unknown_cost_;
  double difficult_cost_;
  double non_traversable_cost_;

  bool enabled_;
  bool current_;

  double min_x_;
  double min_y_;
  double max_x_;
  double max_y_;
};

}  // namespace irc_rover_terrain_costmap

#endif  // IRC_ROVER_TERRAIN_COSTMAP__TERRAIN_COSTMAP_LAYER_HPP_
