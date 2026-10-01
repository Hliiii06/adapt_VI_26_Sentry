#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher() : Node("map_pub")
  {
    const std::string file_name = declare_parameter<std::string>("file_name", "");
    if (file_name.empty()) throw std::runtime_error("file_name parameter must reference a PCD file");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double publish_rate = declare_parameter<double>("publish_rate", 0.2);
    const double downsample_resolution = declare_parameter<double>("downsample_res", 0.0);
    const double offset_x = declare_parameter<double>("map_offset_x", 0.0);
    const double offset_y = declare_parameter<double>("map_offset_y", 0.0);
    const double offset_z = declare_parameter<double>("map_offset_z", 0.0);
    // 高度带过滤：只保留 z_min <= z <= z_max 的点。
    //
    // 注意语义：这是"按绝对高度切掉地图的一部分"，**不是地面识别**。
    // 它对有噪声/坡度的地面有效，但会连带删除矮于阈值的真实障碍，
    // 也会改变机器人与原始地图的高度关系。因此仅作为演示地图的预处理，
    // 参数名与日志都显式标注；正式方案需要局部地面高度或地面分割。
    // 默认不过滤（保持上游行为）。
    const double z_min = declare_parameter<double>("keep_z_min",
                                                   -std::numeric_limits<double>::infinity());
    const double z_max = declare_parameter<double>("keep_z_max",
                                                   std::numeric_limits<double>::infinity());
    // 是否额外发布未过滤的原始点云，供 RViz 对照查看被删掉了什么。
    const bool publish_raw = declare_parameter<bool>("publish_raw_cloud", false);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(file_name, cloud) != 0)
      throw std::runtime_error("failed to load PCD file: " + file_name);
    const size_t loaded_points = cloud.size();
    for (auto &point : cloud)
    {
      point.x += offset_x;
      point.y += offset_y;
      point.z += offset_z;
    }

    pcl::PointCloud<pcl::PointXYZ> raw_cloud = cloud;  // 过滤前的对照副本

    if (std::isfinite(z_min) || std::isfinite(z_max))
    {
      pcl::PointCloud<pcl::PointXYZ> filtered;
      filtered.reserve(cloud.size());
      for (const auto &point : cloud)
        if (point.z >= z_min && point.z <= z_max)
          filtered.push_back(point);
      const size_t removed = cloud.size() - filtered.size();
      RCLCPP_WARN(get_logger(),
                  "DEMO MAP: cut points outside z=[%.3f, %.3f] m (after map_offset_z=%.3f) -> "
                  "removed %zu of %zu points (%.1f%%). This is an absolute-height cut, NOT ground "
                  "segmentation: everything below %.3f m in the ORIGINAL PCD is gone, including "
                  "real low structures. Not for real-robot use.",
                  z_min, z_max, offset_z, removed, loaded_points,
                  100.0 * static_cast<double>(removed) / std::max<size_t>(1, loaded_points),
                  z_min - offset_z);
      cloud.swap(filtered);
    }
    if (cloud.empty())
      throw std::runtime_error("PCD has no points left after height filtering: " + file_name);
    if (downsample_resolution > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      pcl::PointCloud<pcl::PointXYZ> downsampled;
      filter.setInputCloud(cloud.makeShared());
      filter.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      filter.filter(downsampled);
      cloud.swap(downsampled);

      // 对照云按同样分辨率降采样，但保留所有高度，便于在同一视角下对比。
      pcl::PointCloud<pcl::PointXYZ> raw_downsampled;
      filter.setInputCloud(raw_cloud.makeShared());
      filter.filter(raw_downsampled);
      raw_cloud.swap(raw_downsampled);
    }
    pcl::toROSMsg(cloud, message_);
    message_.header.frame_id = frame_id_;
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "global_cloud", rclcpp::QoS(1).reliable().transient_local());
    if (publish_raw)
    {
      pcl::toROSMsg(raw_cloud, raw_message_);
      raw_message_.header.frame_id = frame_id_;
      raw_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "global_cloud_raw", rclcpp::QoS(1).reliable().transient_local());
      RCLCPP_INFO(get_logger(),
                  "Publishing unfiltered reference cloud on global_cloud_raw (%zu points)",
                  raw_cloud.size());
    }
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate)),
        std::bind(&MapPublisher::publishMap, this));
    publishMap();
    RCLCPP_INFO(get_logger(), "Loaded %zu PCD points from %s", cloud.size(), file_name.c_str());
  }

private:
  void publishMap()
  {
    message_.header.stamp = now();
    publisher_->publish(message_);
    if (raw_publisher_)
    {
      raw_message_.header.stamp = message_.header.stamp;
      raw_publisher_->publish(raw_message_);
    }
  }
  std::string frame_id_;
  sensor_msgs::msg::PointCloud2 message_;
  sensor_msgs::msg::PointCloud2 raw_message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<MapPublisher>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_pub"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
