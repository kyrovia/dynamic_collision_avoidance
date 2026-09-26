#include <rclcpp/rclcpp.hpp>

#include <camera_perception/msg/closest_obstacle.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <limits>
#include <string>

namespace
{

struct ClosestPoint
{
  bool found{false};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float distance{0.0f};
};

ClosestPoint find_closest_to_origin(const sensor_msgs::msg::PointCloud2 & cloud)
{
  ClosestPoint closest;
  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (count == 0) {
    return closest;
  }

  float best_distance_sq = std::numeric_limits<float>::infinity();
  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");
  for (std::size_t index = 0; index < count; ++index, ++x_it, ++y_it, ++z_it) {
    const float x = *x_it;
    const float y = *y_it;
    const float z = *z_it;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const float distance_sq = x * x + y * y + z * z;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      closest.x = x;
      closest.y = y;
      closest.z = z;
      closest.found = true;
    }
  }
  if (closest.found) {
    closest.distance = std::sqrt(best_distance_sq);
  }
  return closest;
}

}  // namespace

class ClosestObstacleNode : public rclcpp::Node
{
public:
  ClosestObstacleNode()
  : Node("closest_obstacle_node")
  {
    const std::string cloud_topic = declare_parameter<std::string>(
      "cloud_topic", "obstacle_cloud");
    const std::string closest_topic = declare_parameter<std::string>(
      "closest_topic", "closest_obstacle");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        on_cloud(cloud);
      });
    closest_pub_ = create_publisher<camera_perception::msg::ClosestObstacle>(
      closest_topic, rclcpp::QoS(1));
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    ClosestPoint closest;
    try {
      closest = find_closest_to_origin(*cloud);
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "obstacle cloud: %s", error.what());
      return;
    }
    if (!closest.found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "obstacle cloud has no finite points");
      return;
    }

    camera_perception::msg::ClosestObstacle message;
    message.header = cloud->header;
    if (!frame_id_.empty()) {
      message.header.frame_id = frame_id_;
    }
    message.point.x = closest.x;
    message.point.y = closest.y;
    message.point.z = closest.z;
    message.distance = closest.distance;
    closest_pub_->publish(message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "closest point [%.4f, %.4f, %.4f] distance %.4f m",
      closest.x, closest.y, closest.z, closest.distance);
  }

  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<camera_perception::msg::ClosestObstacle>::SharedPtr closest_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
