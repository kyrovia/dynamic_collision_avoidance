#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

struct Axis
{
  float min;
  float max;
};

struct Config
{
  std::filesystem::path input_pcd;
  std::filesystem::path output_pcd;
  std::filesystem::path filtered_pcd;
  int mean_k;
  float stddev_mul;
  Axis x;
  Axis y;
  Axis z;
};

std::filesystem::path relative_to_config(
  const std::filesystem::path & config_dir,
  const YAML::Node & node,
  const char * key)
{
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error(std::string(key) + " is required");
  }
  const std::string text = node[key].as<std::string>();
  const std::filesystem::path path(text);
  if (path.empty() || path.is_absolute()) {
    throw std::runtime_error(std::string(key) + " must be a relative path, got: " + text);
  }
  return (config_dir / path).lexically_normal();
}

int read_mean_k(const YAML::Node & root)
{
  if (!root["mean_k"] || !root["mean_k"].IsScalar()) {
    throw std::runtime_error("mean_k is required");
  }
  const int mean_k = root["mean_k"].as<int>();
  if (mean_k < 1) {
    throw std::runtime_error("mean_k must be at least 1");
  }
  return mean_k;
}

float read_stddev_mul(const YAML::Node & root)
{
  if (!root["stddev_mul"] || !root["stddev_mul"].IsScalar()) {
    throw std::runtime_error("stddev_mul is required");
  }
  const float stddev_mul = root["stddev_mul"].as<float>();
  if (!(stddev_mul > 0.0f)) {
    throw std::runtime_error("stddev_mul must be positive");
  }
  return stddev_mul;
}

Axis read_axis(const YAML::Node & root, const char * key)
{
  const YAML::Node node = root[key];
  if (!node || !node.IsSequence() || node.size() != 2) {
    throw std::runtime_error(std::string(key) + " must be [min, max]");
  }
  Axis axis{node[0].as<float>(), node[1].as<float>()};
  if (!(axis.min < axis.max)) {
    throw std::runtime_error(std::string(key) + " min must be smaller than max");
  }
  return axis;
}

Config load_config(const std::filesystem::path & config_path)
{
  if (!std::filesystem::is_regular_file(config_path)) {
    throw std::runtime_error("config not found: " + config_path.string());
  }
  // Follow the symlink from the install space back to the source file,
  // so paths in the yaml stay next to that file.
  const std::filesystem::path config_real = std::filesystem::canonical(config_path);
  const YAML::Node root = YAML::LoadFile(config_real.string());
  const std::filesystem::path config_dir = config_real.parent_path();
  Config config;
  config.input_pcd = relative_to_config(config_dir, root, "input_pcd");
  config.output_pcd = relative_to_config(config_dir, root, "output_pcd");
  config.filtered_pcd = relative_to_config(config_dir, root, "filtered_pcd");
  config.mean_k = read_mean_k(root);
  config.stddev_mul = read_stddev_mul(root);
  config.x = read_axis(root, "x");
  config.y = read_axis(root, "y");
  config.z = read_axis(root, "z");
  return config;
}

std::filesystem::path default_config_path()
{
  const std::string share =
    ament_index_cpp::get_package_share_directory("camera_perception");
  return std::filesystem::path(share) / "config" / "aabb_crop.yaml";
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_outliers(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  int mean_k,
  float stddev_mul)
{
  if (cloud->size() <= static_cast<std::size_t>(mean_k)) {
    throw std::runtime_error("point cloud has fewer points than mean_k");
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
  filter.setInputCloud(cloud);
  filter.setMeanK(mean_k);
  filter.setStddevMulThresh(stddev_mul);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
  filter.filter(*inliers);
  return inliers;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop_aabb(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const Config & config)
{
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(config.x.min, config.y.min, config.z.min, 1.0f));
  crop.setMax(Eigen::Vector4f(config.x.max, config.y.max, config.z.max, 1.0f));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>);
  crop.filter(*cropped);
  return cropped;
}

void save_cloud(
  const std::filesystem::path & path,
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
  const std::filesystem::path parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  if (pcl::io::savePCDFileBinary(path.string(), cloud) < 0) {
    throw std::runtime_error("failed to write " + path.string());
  }
}

void filter_cloud(const Config & config)
{
  if (!std::filesystem::is_regular_file(config.input_pcd)) {
    throw std::runtime_error("input point cloud not found: " + config.input_pcd.string());
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile(config.input_pcd.string(), *cloud) < 0) {
    throw std::runtime_error("failed to read " + config.input_pcd.string());
  }

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers =
    remove_outliers(cloud, config.mean_k, config.stddev_mul);
  save_cloud(config.filtered_pcd, *inliers);
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped = crop_aabb(inliers, config);
  save_cloud(config.output_pcd, *cropped);

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "input " << config.input_pcd << " (" << cloud->size() << " points)\n"
            << "statistical outlier mean_k=" << config.mean_k
            << " stddev_mul=" << config.stddev_mul
            << " (" << inliers->size() << " points)\n"
            << "filtered " << config.filtered_pcd << "\n"
            << "aabb x[" << config.x.min << ", " << config.x.max << "] "
            << "y[" << config.y.min << ", " << config.y.max << "] "
            << "z[" << config.z.min << ", " << config.z.max << "]\n"
            << "output " << config.output_pcd << " (" << cropped->size() << " points)\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc > 2) {
      std::cerr << "usage: aabb_crop [config.yaml]\n";
      return 2;
    }
    const std::filesystem::path config_path =
      argc == 2 ? std::filesystem::path(argv[1]) : default_config_path();
    filter_cloud(load_config(config_path));
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "aabb_crop: " << error.what() << '\n';
    return 1;
  }
}
