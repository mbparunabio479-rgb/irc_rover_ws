#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class ObstacleFilter : public rclcpp::Node
{
public:
  ObstacleFilter()
  : Node("obstacle_filter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cloud_registered");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/terrain/live_obstacles");

    min_height_ = declare_parameter<double>("min_height", 0.30);
    max_height_ = declare_parameter<double>("max_height", 2.00);
    max_range_ = declare_parameter<double>("max_range", 10.0);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ObstacleFilter::cloudCallback,
        this,
        std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Live obstacle filter started");

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
      "Height: %.2f to %.2f m | Range: %.2f m",
      min_height_,
      max_height_,
      max_range_);
  }

private:
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*msg, input);

    pcl::PointCloud<pcl::PointXYZ> obstacles;
    obstacles.header = input.header;

    const double max_range_sq = max_range_ * max_range_;

    for (const auto & p : input.points)
    {
      if (!std::isfinite(p.x) ||
          !std::isfinite(p.y) ||
          !std::isfinite(p.z))
      {
        continue;
      }

      const double range_sq =
        static_cast<double>(p.x) * p.x +
        static_cast<double>(p.y) * p.y;

      if (range_sq > max_range_sq)
      {
        continue;
      }

      if (p.z < min_height_ || p.z > max_height_)
      {
        continue;
      }

      obstacles.points.push_back(p);
    }

    obstacles.width = obstacles.points.size();
    obstacles.height = 1;
    obstacles.is_dense = false;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(obstacles, output);

    output.header = msg->header;

    pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;

  double min_height_;
  double max_height_;
  double max_range_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleFilter>());
  rclcpp::shutdown();

  return 0;
}
