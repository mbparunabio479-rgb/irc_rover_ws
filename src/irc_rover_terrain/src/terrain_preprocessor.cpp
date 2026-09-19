#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>

class TerrainPreprocessor : public rclcpp::Node
{
public:
  TerrainPreprocessor()
  : Node("terrain_preprocessor")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cloud_registered");

    elevation_cloud_topic_ = declare_parameter<std::string>(
      "elevation_cloud_topic", "/terrain/elevation_cloud");

    elevation_topic_ = declare_parameter<std::string>(
      "elevation_topic", "/terrain/elevation");

    confidence_topic_ = declare_parameter<std::string>(
      "confidence_topic", "/terrain/confidence");

    voxel_size_ = declare_parameter<double>(
      "voxel_size", 0.05);

    grid_resolution_ = declare_parameter<double>(
      "grid_resolution", 0.10);

    roi_x_min_ = declare_parameter<double>("roi_x_min", -10.0);
    roi_x_max_ = declare_parameter<double>("roi_x_max", 10.0);
    roi_y_min_ = declare_parameter<double>("roi_y_min", -10.0);
    roi_y_max_ = declare_parameter<double>("roi_y_max", 10.0);
    roi_z_min_ = declare_parameter<double>("roi_z_min", -3.0);
    roi_z_max_ = declare_parameter<double>("roi_z_max", 3.0);

    elevation_percentile_ = declare_parameter<double>(
      "elevation_percentile", 20.0);

    min_points_per_cell_ = declare_parameter<int>(
      "min_points_per_cell", 2);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &TerrainPreprocessor::cloudCallback,
        this,
        std::placeholders::_1));

    elevation_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        elevation_cloud_topic_,
        rclcpp::SensorDataQoS());

    elevation_pub_ =
      create_publisher<grid_map_msgs::msg::GridMap>(
        elevation_topic_,
        rclcpp::SensorDataQoS());

    confidence_pub_ =
      create_publisher<grid_map_msgs::msg::GridMap>(
        confidence_topic_,
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Terrain elevation mapping started");

    RCLCPP_INFO(
      get_logger(),
      "Input: %s",
      input_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Elevation cloud: %s",
      elevation_cloud_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Elevation grid: %s",
      elevation_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Confidence grid: %s",
      confidence_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Voxel: %.3f m | Grid: %.3f m",
      voxel_size_,
      grid_resolution_);

    RCLCPP_INFO(
      get_logger(),
      "ROI: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
      roi_x_min_,
      roi_x_max_,
      roi_y_min_,
      roi_y_max_,
      roi_z_min_,
      roi_z_max_);
  }

private:

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *input);

    if (input->empty()) {
      return;
    }

    /*
     * ------------------------------------------------------------
     * 1. ROI filtering
     * ------------------------------------------------------------
     */

    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(
      new pcl::PointCloud<pcl::PointXYZ>);

    roi->reserve(input->size());

    for (const auto & p : input->points)
    {
      if (!std::isfinite(p.x) ||
          !std::isfinite(p.y) ||
          !std::isfinite(p.z))
      {
        continue;
      }

      if (p.x < roi_x_min_ || p.x > roi_x_max_)
        continue;

      if (p.y < roi_y_min_ || p.y > roi_y_max_)
        continue;

      if (p.z < roi_z_min_ || p.z > roi_z_max_)
        continue;

      roi->points.push_back(p);
    }

    roi->width = roi->points.size();
    roi->height = 1;
    roi->is_dense = true;

    if (roi->empty()) {
      return;
    }

    /*
     * ------------------------------------------------------------
     * 2. Voxel downsampling
     * ------------------------------------------------------------
     */

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;

    voxel.setInputCloud(roi);

    voxel.setLeafSize(
      voxel_size_,
      voxel_size_,
      voxel_size_);

    voxel.filter(*filtered);

    if (filtered->empty()) {
      return;
    }

    /*
     * ------------------------------------------------------------
     * 3. Create elevation grid
     * ------------------------------------------------------------
     */

    const int cells_x = static_cast<int>(
      std::ceil(
        (roi_x_max_ - roi_x_min_) /
        grid_resolution_));

    const int cells_y = static_cast<int>(
      std::ceil(
        (roi_y_max_ - roi_y_min_) /
        grid_resolution_));

    const size_t total_cells =
      static_cast<size_t>(cells_x) *
      static_cast<size_t>(cells_y);

    std::vector<std::vector<float>> cell_heights(total_cells);

    for (const auto & p : filtered->points)
    {
      const int ix = static_cast<int>(
        std::floor(
          (p.x - roi_x_min_) /
          grid_resolution_));

      const int iy = static_cast<int>(
        std::floor(
          (p.y - roi_y_min_) /
          grid_resolution_));

      if (ix < 0 || ix >= cells_x ||
          iy < 0 || iy >= cells_y)
      {
        continue;
      }

      const size_t index =
        static_cast<size_t>(iy) *
        static_cast<size_t>(cells_x) +
        static_cast<size_t>(ix);

      cell_heights[index].push_back(p.z);
    }

    /*
     * ------------------------------------------------------------
     * 4. Grid Map
     * ------------------------------------------------------------
     */

    grid_map::GridMap elevation_map;

    elevation_map.setFrameId(msg->header.frame_id);

    elevation_map.setGeometry(
      grid_map::Length(
        roi_x_max_ - roi_x_min_,
        roi_y_max_ - roi_y_min_),
      grid_resolution_,
      grid_map::Position(
        (roi_x_min_ + roi_x_max_) / 2.0,
        (roi_y_min_ + roi_y_max_) / 2.0));

    elevation_map.add(
      "elevation",
      std::numeric_limits<float>::quiet_NaN());

    elevation_map.add(
      "confidence",
      std::numeric_limits<float>::quiet_NaN());

    /*
     * Cloud containing one representative terrain point
     * per valid grid cell.
     */

    pcl::PointCloud<pcl::PointXYZ>::Ptr elevation_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    /*
     * ------------------------------------------------------------
     * 5. Estimate elevation
     *
     * We use a low percentile instead of the absolute minimum.
     *
     * This prevents one isolated low outlier from defining
     * the terrain elevation.
     * ------------------------------------------------------------
     */

    for (int iy = 0; iy < cells_y; ++iy)
    {
      for (int ix = 0; ix < cells_x; ++ix)
      {
        const size_t index =
          static_cast<size_t>(iy) *
          static_cast<size_t>(cells_x) +
          static_cast<size_t>(ix);

        auto & heights = cell_heights[index];

        if (static_cast<int>(heights.size()) <
            min_points_per_cell_)
        {
          continue;
        }

        std::sort(
          heights.begin(),
          heights.end());

        const double percentile =
          std::clamp(
            elevation_percentile_,
            0.0,
            100.0);

        const double position =
          (percentile / 100.0) *
          static_cast<double>(heights.size() - 1);

        const size_t lower =
          static_cast<size_t>(std::floor(position));

        const size_t upper =
          static_cast<size_t>(std::ceil(position));

        const double alpha =
          position - static_cast<double>(lower);

        const double elevation =
          static_cast<double>(heights[lower]) *
          (1.0 - alpha) +
          static_cast<double>(heights[upper]) *
          alpha;

        const double x =
          roi_x_min_ +
          (static_cast<double>(ix) + 0.5) *
          grid_resolution_;

        const double y =
          roi_y_min_ +
          (static_cast<double>(iy) + 0.5) *
          grid_resolution_;

        grid_map::Index cell_index;
        elevation_map.getIndex(
          grid_map::Position(x, y),
          cell_index);

        elevation_map.at("elevation", cell_index) =
          static_cast<float>(elevation);

        /*
         * Confidence is normalized point density.
         *
         * 1.0 = 10 or more points in a cell.
         */

        const float confidence =
          std::min(
            1.0f,
            static_cast<float>(heights.size()) / 10.0f);

        elevation_map.at(
          "confidence",
          cell_index) =
          confidence;

        pcl::PointXYZ point;

        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(elevation);

        elevation_cloud->points.push_back(point);
      }
    }

    elevation_cloud->width =
      elevation_cloud->points.size();

    elevation_cloud->height = 1;

    elevation_cloud->is_dense = false;

    /*
     * ------------------------------------------------------------
     * 6. Publish representative terrain cloud
     * ------------------------------------------------------------
     */

    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(
      *elevation_cloud,
      cloud_msg);

    cloud_msg.header = msg->header;

    elevation_cloud_pub_->publish(cloud_msg);

    /*
     * ------------------------------------------------------------
     * 7. Publish elevation GridMap
     * ------------------------------------------------------------
     */

    grid_map_msgs::msg::GridMap elevation_msg;

    elevation_msg =
      *grid_map::GridMapRosConverter::toMessage(elevation_map);

    elevation_msg.header.stamp =
      msg->header.stamp;

    elevation_msg.header.frame_id =
      msg->header.frame_id;

    elevation_pub_->publish(elevation_msg);

    /*
     * ------------------------------------------------------------
     * 8. Publish confidence GridMap
     * ------------------------------------------------------------
     */

    grid_map_msgs::msg::GridMap confidence_msg;

    confidence_msg =
      *grid_map::GridMapRosConverter::toMessage(elevation_map);

    confidence_msg.header.stamp =
      msg->header.stamp;

    confidence_msg.header.frame_id =
      msg->header.frame_id;

    /*
     * Keep both layers in the same GridMap message.
     * The confidence layer can be selected in RViz.
     */

    confidence_pub_->publish(confidence_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Input: %zu | ROI: %zu | Filtered: %zu | Elevation cells: %zu",
      input->size(),
      roi->size(),
      filtered->size(),
      elevation_cloud->size());
  }

  std::string input_topic_;
  std::string elevation_cloud_topic_;
  std::string elevation_topic_;
  std::string confidence_topic_;

  double voxel_size_;
  double grid_resolution_;

  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;

  double elevation_percentile_;
  int min_points_per_cell_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    elevation_cloud_pub_;

  rclcpp::Publisher<
    grid_map_msgs::msg::GridMap>::SharedPtr
    elevation_pub_;

  rclcpp::Publisher<
    grid_map_msgs::msg::GridMap>::SharedPtr
    confidence_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<TerrainPreprocessor>());

  rclcpp::shutdown();

  return 0;
}
