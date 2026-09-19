#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

class TerrainFeatures : public rclcpp::Node
{
public:
  TerrainFeatures()
  : Node("terrain_features")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/terrain/elevation");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/terrain/features");

    roughness_radius_ = declare_parameter<int>(
      "roughness_radius", 1);

    slope_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
      output_topic_, rclcpp::SensorDataQoS());

    subscription_ = create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &TerrainFeatures::mapCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Terrain feature extraction started");

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
      "Roughness radius: %d cells",
      roughness_radius_);
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
        "Failed to convert GridMap message");

      return;
    }

    if (!input_map.exists("elevation"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Input GridMap has no elevation layer");

      return;
    }

    const double resolution = input_map.getResolution();

    if (resolution <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid GridMap resolution");

      return;
    }

    grid_map::GridMap output_map = input_map;

    if (!output_map.exists("slope"))
    {
      output_map.add("slope");
    }

    if (!output_map.exists("roughness"))
    {
      output_map.add("roughness");
    }

    /*
     * Initialize output layers to NaN.
     */
    output_map["slope"].setConstant(
      std::numeric_limits<float>::quiet_NaN());

    output_map["roughness"].setConstant(
      std::numeric_limits<float>::quiet_NaN());

    /*
     * ------------------------------------------------------------
     * SLOPE
     *
     * Estimate local slope using available neighboring cells.
     *
     * If both sides are available, use central difference.
     * If only one side is available, use forward/backward difference.
     *
     * This is important for sparse LiDAR terrain maps.
     * ------------------------------------------------------------
     */

    for (grid_map::GridMapIterator iterator(input_map);
         !iterator.isPastEnd();
         ++iterator)
    {
      const grid_map::Index index(*iterator);

      const float center =
        input_map.at("elevation", index);

      if (!std::isfinite(center))
      {
        continue;
      }

      grid_map::Position center_position;
      input_map.getPosition(index, center_position);

      const double zc =
        static_cast<double>(center);

      /*
       * X direction.
       */
      double dzdx = 0.0;
      bool valid_x = false;

      grid_map::Index left_index;
      grid_map::Index right_index;

      grid_map::Position left_position =
        center_position;

      grid_map::Position right_position =
        center_position;

      left_position.x() -= resolution;
      right_position.x() += resolution;

      bool have_left =
        input_map.getIndex(left_position, left_index);

      bool have_right =
        input_map.getIndex(right_position, right_index);

      bool left_valid = false;
      bool right_valid = false;

      double zl = 0.0;
      double zr = 0.0;

      if (have_left)
      {
        const float value =
          input_map.at("elevation", left_index);

        if (std::isfinite(value))
        {
          zl = static_cast<double>(value);
          left_valid = true;
        }
      }

      if (have_right)
      {
        const float value =
          input_map.at("elevation", right_index);

        if (std::isfinite(value))
        {
          zr = static_cast<double>(value);
          right_valid = true;
        }
      }

      if (left_valid && right_valid)
      {
        dzdx = (zr - zl) / (2.0 * resolution);
        valid_x = true;
      }
      else if (right_valid)
      {
        dzdx = (zr - zc) / resolution;
        valid_x = true;
      }
      else if (left_valid)
      {
        dzdx = (zc - zl) / resolution;
        valid_x = true;
      }

      /*
       * Y direction.
       */
      double dzdy = 0.0;
      bool valid_y = false;

      grid_map::Index down_index;
      grid_map::Index up_index;

      grid_map::Position down_position =
        center_position;

      grid_map::Position up_position =
        center_position;

      down_position.y() -= resolution;
      up_position.y() += resolution;

      bool have_down =
        input_map.getIndex(down_position, down_index);

      bool have_up =
        input_map.getIndex(up_position, up_index);

      bool down_valid = false;
      bool up_valid = false;

      double zd = 0.0;
      double zu = 0.0;

      if (have_down)
      {
        const float value =
          input_map.at("elevation", down_index);

        if (std::isfinite(value))
        {
          zd = static_cast<double>(value);
          down_valid = true;
        }
      }

      if (have_up)
      {
        const float value =
          input_map.at("elevation", up_index);

        if (std::isfinite(value))
        {
          zu = static_cast<double>(value);
          up_valid = true;
        }
      }

      if (down_valid && up_valid)
      {
        dzdy = (zu - zd) / (2.0 * resolution);
        valid_y = true;
      }
      else if (up_valid)
      {
        dzdy = (zu - zc) / resolution;
        valid_y = true;
      }
      else if (down_valid)
      {
        dzdy = (zc - zd) / resolution;
        valid_y = true;
      }

      /*
       * Calculate slope if at least one horizontal gradient
       * direction is available.
       */
      if (valid_x || valid_y)
      {
        const double gradient =
          std::sqrt(
            dzdx * dzdx +
            dzdy * dzdy);

        const double slope_rad =
          std::atan(gradient);

        const double slope_deg =
          slope_rad * 180.0 / M_PI;

        output_map.at("slope", index) =
          static_cast<float>(slope_deg);
      }
    }

    /*
     * ------------------------------------------------------------
     * ROUGHNESS
     *
     * Local elevation standard deviation.
     *
     * roughness = sqrt(
     *   mean((z - mean(z))^2)
     * )
     *
     * This is an initial terrain roughness metric.
     * It can later be replaced with plane-fit residuals.
     * ------------------------------------------------------------
     */

    for (grid_map::GridMapIterator iterator(input_map);
         !iterator.isPastEnd();
         ++iterator)
    {
      const grid_map::Index index(*iterator);

      const float center =
        input_map.at("elevation", index);

      if (!std::isfinite(center))
      {
        continue;
      }

      grid_map::Position center_position;
      input_map.getPosition(index, center_position);

      std::vector<double> heights;

      for (int dx = -roughness_radius_;
           dx <= roughness_radius_;
           ++dx)
      {
        for (int dy = -roughness_radius_;
             dy <= roughness_radius_;
             ++dy)
        {
          grid_map::Position sample_position =
            center_position;

          sample_position.x() +=
            static_cast<double>(dx) * resolution;

          sample_position.y() +=
            static_cast<double>(dy) * resolution;

          grid_map::Index sample_index;

          if (!input_map.getIndex(
                sample_position,
                sample_index))
          {
            continue;
          }

          const float z =
            input_map.at(
              "elevation",
              sample_index);

          if (std::isfinite(z))
          {
            heights.push_back(
              static_cast<double>(z));
          }
        }
      }

      if (heights.size() < 2)
      {
        continue;
      }

      double mean = 0.0;

      for (const double z : heights)
      {
        mean += z;
      }

      mean /= static_cast<double>(heights.size());

      double variance = 0.0;

      for (const double z : heights)
      {
        const double difference =
          z - mean;

        variance +=
          difference * difference;
      }

      variance /=
        static_cast<double>(heights.size());

      const double roughness =
        std::sqrt(variance);

      output_map.at(
        "roughness",
        index) =
        static_cast<float>(roughness);
    }

    /*
     * Publish.
     */
    auto output_msg =
      grid_map::GridMapRosConverter::toMessage(
        output_map);

    if (output_msg)
    {
      slope_pub_->publish(*output_msg);
    }

    static int counter = 0;
    counter++;

    if (counter % 10 == 0)
    {
      RCLCPP_INFO(
        get_logger(),
        "Published terrain features | "
        "resolution: %.3f m | "
        "layers: %zu",
        resolution,
        output_map.getLayers().size());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  int roughness_radius_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr
    subscription_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
    slope_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TerrainFeatures>());

  rclcpp::shutdown();

  return 0;
}
