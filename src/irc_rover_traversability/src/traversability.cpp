#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <cmath>
#include <limits>
#include <string>

class Traversability : public rclcpp::Node
{
public:
  Traversability()
  : Node("traversability")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/terrain/features");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/terrain/traversability");

    max_slope_deg_ = declare_parameter<double>(
      "max_slope_deg", 25.0);

    max_roughness_m_ = declare_parameter<double>(
      "max_roughness_m", 0.10);

    min_confidence_ = declare_parameter<double>(
      "min_confidence", 0.20);

    slope_weight_ = declare_parameter<double>(
      "slope_weight", 0.5);

    roughness_weight_ = declare_parameter<double>(
      "roughness_weight", 0.5);

    publisher_ =
      create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_,
        rclcpp::SensorDataQoS());

    subscription_ =
      create_subscription<grid_map_msgs::msg::GridMap>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &Traversability::mapCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Traversability mapping started");

    RCLCPP_INFO(
      get_logger(),
      "Input: %s",
      input_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Output: %s",
      output_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Thresholds: slope %.1f deg | roughness %.3f m | confidence %.2f",
      max_slope_deg_,
      max_roughness_m_,
      min_confidence_);
  }

private:

  void mapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap input_map;

    if (!grid_map::GridMapRosConverter::fromMessage(
        *msg, input_map))
    {
      RCLCPP_WARN(
        get_logger(),
        "Failed to convert terrain feature GridMap");

      return;
    }

    if (!input_map.exists("elevation") ||
        !input_map.exists("confidence") ||
        !input_map.exists("slope") ||
        !input_map.exists("roughness"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Required terrain feature layers are missing");

      return;
    }

    grid_map::GridMap output_map = input_map;

    if (!output_map.exists("traversability"))
    {
      output_map.add("traversability");
    }

    if (!output_map.exists("terrain_class"))
    {
      output_map.add("terrain_class");
    }

    output_map["traversability"].setConstant(
      std::numeric_limits<float>::quiet_NaN());

    output_map["terrain_class"].setConstant(
      std::numeric_limits<float>::quiet_NaN());

    /*
     * terrain_class:
     *
     * 0 = UNKNOWN
     * 1 = TRAVERSABLE
     * 2 = DIFFICULT / HIGH COST
     * 3 = NON_TRAVERSABLE
     */

    int unknown_count = 0;
    int traversable_count = 0;
    int difficult_count = 0;
    int non_traversable_count = 0;

    for (grid_map::GridMapIterator iterator(input_map);
         !iterator.isPastEnd();
         ++iterator)
    {
      const grid_map::Index index(*iterator);

      const float elevation =
        input_map.at("elevation", index);

      const float confidence =
        input_map.at("confidence", index);

      const float slope =
        input_map.at("slope", index);

      const float roughness =
        input_map.at("roughness", index);

      /*
       * Unknown terrain.
       */
      if (!std::isfinite(elevation) ||
          !std::isfinite(confidence))
      {
        unknown_count++;
        continue;
      }

      /*
       * Low-confidence terrain.
       */
      if (static_cast<double>(confidence) < min_confidence_)
      {
        unknown_count++;
        continue;
      }

      /*
       * Slope/roughness may be unavailable in sparse regions.
       */
      if (!std::isfinite(slope) ||
          !std::isfinite(roughness))
      {
        unknown_count++;
        continue;
      }

      const double slope_ratio =
        std::clamp(
          static_cast<double>(slope) /
          max_slope_deg_,
          0.0,
          1.0);

      const double roughness_ratio =
        std::clamp(
          static_cast<double>(roughness) /
          max_roughness_m_,
          0.0,
          1.0);

      /*
       * Higher value = easier terrain.
       */
      const double slope_score =
        1.0 - slope_ratio;

      const double roughness_score =
        1.0 - roughness_ratio;

      double score =
        slope_weight_ * slope_score +
        roughness_weight_ * roughness_score;

      score = std::clamp(score, 0.0, 1.0);

      /*
       * Hard limits.
       */
      if (slope > max_slope_deg_ ||
          roughness > max_roughness_m_)
      {
        output_map.at(
          "terrain_class",
          index) = 3.0f;

        output_map.at(
          "traversability",
          index) = 0.0f;

        non_traversable_count++;
      }
      else if (score < 0.5)
      {
        output_map.at(
          "terrain_class",
          index) = 2.0f;

        output_map.at(
          "traversability",
          index) =
          static_cast<float>(score);

        difficult_count++;
      }
      else
      {
        output_map.at(
          "terrain_class",
          index) = 1.0f;

        output_map.at(
          "traversability",
          index) =
          static_cast<float>(score);

        traversable_count++;
      }
    }

    auto output_msg =
      grid_map::GridMapRosConverter::toMessage(
        output_map);

    if (output_msg)
    {
      publisher_->publish(*output_msg);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Traversability | unknown: %d | traversable: %d | difficult: %d | non-traversable: %d",
      unknown_count,
      traversable_count,
      difficult_count,
      non_traversable_count);
  }

  std::string input_topic_;
  std::string output_topic_;

  double max_slope_deg_;
  double max_roughness_m_;
  double min_confidence_;

  double slope_weight_;
  double roughness_weight_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr
    subscription_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
    publisher_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<Traversability>());

  rclcpp::shutdown();

  return 0;
}
