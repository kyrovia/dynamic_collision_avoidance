#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>

#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Axis
{
  float min;
  float max;
};

Axis read_axis(rclcpp::Node & node, const std::string & name)
{
  const std::vector<double> values =
    node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
  if (values.size() != 2 || !(values[0] < values[1])) {
    throw std::runtime_error(name + " must be [min, max]");
  }
  return Axis{static_cast<float>(values[0]), static_cast<float>(values[1])};
}

pcl::PointCloud<pcl::PointXYZ>::Ptr remove_outliers(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  int mean_k,
  double stddev_mul)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setMeanK(mean_k);
  filter.setStddevMulThresh(stddev_mul);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
  filter.filter(*inliers);
  return inliers;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_aabb(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const Axis & x,
  const Axis & y,
  const Axis & z)
{
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(x.min, y.min, z.min, 1.0f));
  crop.setMax(Eigen::Vector4f(x.max, y.max, z.max, 1.0f));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  crop.filter(*cropped);
  return cropped;
}

}  // namespace

class ObstacleCloudNode : public rclcpp::Node
{
public:
  ObstacleCloudNode()
  : Node("obstacle_cloud_node")
  {
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "/camera/camera/depth/color/points");
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "obstacle_cloud");
    mean_k_ = declare_parameter<int>("mean_k", 50);
    stddev_mul_ = declare_parameter<double>("stddev_mul", 1.0);
    if (mean_k_ < 1) {
      throw std::runtime_error("mean_k must be at least 1");
    }
    if (!(stddev_mul_ > 0.0)) {
      throw std::runtime_error("stddev_mul must be positive");
    }
    x_ = read_axis(*this, "x");
    y_ = read_axis(*this, "y");
    z_ = read_axis(*this, "z");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        on_cloud(cloud);
      });
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    RCLCPP_INFO(
      get_logger(), "listening on %s, publishing %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*message, *cloud);
    if (cloud->size() <= static_cast<std::size_t>(mean_k_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cloud has %zu points, fewer than mean_k %d", cloud->size(), mean_k_);
      return;
    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr inliers =
      remove_outliers(cloud, mean_k_, stddev_mul_);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cropped = inliers->empty() ?
      inliers : crop_aabb(inliers, x_, y_, z_);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cropped, output);
    output.header = message->header;
    cloud_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "input %zu, inliers %zu, cropped %zu",
      cloud->size(), inliers->size(), cropped->size());
  }

  int mean_k_{50};
  double stddev_mul_{1.0};
  Axis x_{};
  Axis y_{};
  Axis z_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleCloudNode>());
  rclcpp::shutdown();
  return 0;
}
