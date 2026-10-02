#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
    // 地形表面点云（prepare_terrain_map.py 产出的 *_surface.pcd）。
    // **仅供 RViz 显示地形起伏**：地面点绝不能喂给 SCAN 的占据栅格，否则地面自身
    // 会被判成障碍。机器人的高度来自 ground_grid_file 查询，不走这条话题。
    const std::string ground_file = declare_parameter<std::string>("ground_file", "");
    ground_offset_z_ = declare_parameter<double>("ground_offset_z", 0.0);

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
      const double removed_pct =
          100.0 * static_cast<double>(removed) / std::max<size_t>(1, loaded_points);
      // 只有真的删了点才告警；地形剖面的宽阈值只是为了不删任何东西。
      if (removed > 0)
        RCLCPP_WARN(get_logger(),
                    "DEMO MAP: cut points outside z=[%.3f, %.3f] m (after map_offset_z=%.3f) -> "
                    "removed %zu of %zu points (%.1f%%). This is an absolute-height cut, NOT "
                    "ground segmentation: everything below %.3f m in the ORIGINAL PCD is gone, "
                    "including real low structures. Not for real-robot use.",
                    z_min, z_max, offset_z, removed, loaded_points, removed_pct, z_min - offset_z);
      else
        RCLCPP_INFO(get_logger(),
                    "Height window z=[%.3f, %.3f] m removed no points", z_min, z_max);
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
    if (!ground_file.empty())
    {
      pcl::PointCloud<pcl::PointXYZ> ground_cloud;
      if (pcl::io::loadPCDFile(ground_file, ground_cloud) != 0)
        throw std::runtime_error("failed to load ground PCD file: " + ground_file);
      for (auto &point : ground_cloud)
        point.z += ground_offset_z_;
      pcl::toROSMsg(ground_cloud, ground_message_);
      ground_message_.header.frame_id = frame_id_;
      ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "ground_surface", rclcpp::QoS(1).reliable().transient_local());
      RCLCPP_INFO(get_logger(),
                  "Publishing terrain surface on ground_surface (%zu points) — "
                  "visualisation only, NOT fed to the planner",
                  ground_cloud.size());
    }
    // 地形实体网格（按高度着色），用于在 RViz 里看清坡道与洞口
    const std::string mesh_grid =
        declare_parameter<std::string>("ground_mesh_grid_file", "");
    if (!mesh_grid.empty())
    {
      std::ifstream grid_file(mesh_grid);
      if (!grid_file.is_open())
        throw std::runtime_error("failed to load ground mesh grid: " + mesh_grid);
      std::string line;
      double cell = 0.0, gx0 = 0.0, gy0 = 0.0;
      int gnx = 0, gny = 0;
      bool header_done = false;
      std::vector<double> heights;
      while (std::getline(grid_file, line))
      {
        if (line.empty()) continue;
        if (line[0] == '#')
        {
          if (!header_done && line.find("cell") != std::string::npos)
          {
            std::istringstream stream(line);
            std::string token;
            while (stream >> token)
            {
              if (token == "cell") stream >> cell;
              else if (token == "x0") stream >> gx0;
              else if (token == "y0") stream >> gy0;
              else if (token == "nx") stream >> gnx;
              else if (token == "ny") stream >> gny;
            }
            header_done = (gnx > 1 && gny > 1 && cell > 0.0);
          }
          continue;
        }
        std::istringstream stream(line);
        double value = 0.0;
        while (stream >> value) heights.push_back(value);
      }
      if (!header_done || heights.size() != static_cast<size_t>(gnx) * static_cast<size_t>(gny))
        throw std::runtime_error("ground mesh grid malformed: " + mesh_grid);

      ground_mesh_.header.frame_id = frame_id_;
      ground_mesh_.ns = "terrain_surface";
      ground_mesh_.id = 0;
      ground_mesh_.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      ground_mesh_.action = visualization_msgs::msg::Marker::ADD;
      ground_mesh_.pose.orientation.w = 1.0;
      ground_mesh_.scale.x = 1.0;
      ground_mesh_.scale.y = 1.0;
      ground_mesh_.scale.z = 1.0;
      // 半透明：地形面是**辅助层**，不能盖住真正的 PCD 点云。
      ground_mesh_.color.a = 0.45;

      double hmin = heights.front(), hmax = heights.front();
      for (double h : heights) { hmin = std::min(hmin, h); hmax = std::max(hmax, h); }
      const double span = std::max(1e-6, hmax - hmin);
      const auto shade = [&](double h) {
        const double t = (h - hmin) / span;
        // 蓝(低) -> 青 -> 绿 -> 黄 -> 红(高)
        geometry_msgs::msg::Point c;
        c.x = std::min(1.0, std::max(0.0, 1.5 - std::abs(4.0 * t - 3.0)));
        c.y = std::min(1.0, std::max(0.0, 1.5 - std::abs(4.0 * t - 2.0)));
        c.z = std::min(1.0, std::max(0.0, 1.5 - std::abs(4.0 * t - 1.0)));
        return c;
      };
      const auto vertex = [&](int i, int j) {
        geometry_msgs::msg::Point p;
        p.x = gx0 + i * cell;
        p.y = gy0 + j * cell;
        p.z = heights[static_cast<size_t>(j) * gnx + i];
        return p;
      };
      for (int j = 0; j + 1 < gny; ++j)
      {
        for (int i = 0; i + 1 < gnx; ++i)
        {
          const geometry_msgs::msg::Point p00 = vertex(i, j);
          const geometry_msgs::msg::Point p10 = vertex(i + 1, j);
          const geometry_msgs::msg::Point p01 = vertex(i, j + 1);
          const geometry_msgs::msg::Point p11 = vertex(i + 1, j + 1);
          const geometry_msgs::msg::Point tri[6] = {p00, p10, p11, p00, p11, p01};
          for (const auto &p : tri)
          {
            ground_mesh_.points.push_back(p);
            const auto c = shade(p.z);
            std_msgs::msg::ColorRGBA col;
            col.r = static_cast<float>(c.x);
            col.g = static_cast<float>(c.y);
            col.b = static_cast<float>(c.z);
            col.a = 0.45f;
            ground_mesh_.colors.push_back(col);
          }
        }
      }
      ground_mesh_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
          "terrain_surface_mesh", rclcpp::QoS(1).reliable().transient_local());
      RCLCPP_INFO(get_logger(),
                  "Publishing terrain surface mesh on terrain_surface_mesh "
                  "(%zu triangles, z %.3f..%.3f m) — visualisation only",
                  ground_mesh_.points.size() / 3, hmin, hmax);
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
    if (ground_mesh_publisher_)
    {
      ground_mesh_.header.stamp = message_.header.stamp;
      ground_mesh_publisher_->publish(ground_mesh_);
    }
    if (ground_publisher_)
    {
      ground_message_.header.stamp = message_.header.stamp;
      ground_publisher_->publish(ground_message_);
    }
  }
  std::string frame_id_;
  double ground_offset_z_{0.0};
  sensor_msgs::msg::PointCloud2 message_;
  sensor_msgs::msg::PointCloud2 raw_message_;
  sensor_msgs::msg::PointCloud2 ground_message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  // 地形实体网格：坡道/洞口看"面"比看"点云"直观得多。仅用于显示，不进规划。
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ground_mesh_publisher_;
  visualization_msgs::msg::Marker ground_mesh_;
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
