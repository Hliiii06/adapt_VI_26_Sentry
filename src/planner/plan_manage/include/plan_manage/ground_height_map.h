// 地面高度网格：从 prepare_terrain_map.py 生成的文本网格读取，提供双线性查询。
//
// 用途：让地面机器人的 z 跟随地形。地图里的地面点不能喂给 SCAN（占据栅格会把地面
// 本身当成障碍），所以地面以这个独立网格的形式提供给运动模拟器和规划器。
//
// 文件格式（见 scripts/prepare_terrain_map.py）：
//     # 注释行若干
//     <z00> <z10> ... （共 ny 行，每行 nx 个值，x 递增、y 递增）
// 网格原点与尺寸从注释里的 "cell <c> x0 <x> y0 <y> nx <nx> ny <ny>" 读取。

#ifndef PLAN_MANAGE_GROUND_HEIGHT_MAP_H
#define PLAN_MANAGE_GROUND_HEIGHT_MAP_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace scan_planner
{
class GroundHeightMap
{
public:
  bool load(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open())
      return false;

    values_.clear();
    std::string line;
    bool header_parsed = false;
    while (std::getline(file, line))
    {
      if (line.empty())
        continue;
      if (line[0] == '#')
      {
        // 只在第一行含 "cell" 的注释里解析头部。其它注释（例如
        // "each row nx values"）同样含 nx/ny 这样的词，参与解析会把 nx 读成 0。
        if (!header_parsed && line.find("cell") != std::string::npos)
        {
          std::istringstream stream(line);
          std::string token;
          while (stream >> token)
          {
            if (token == "cell") { if (!(stream >> cell_)) cell_ = 0.0; }
            else if (token == "x0") { if (!(stream >> x0_)) x0_ = 0.0; }
            else if (token == "y0") { if (!(stream >> y0_)) y0_ = 0.0; }
            else if (token == "nx") { if (!(stream >> nx_)) nx_ = 0; }
            else if (token == "ny") { if (!(stream >> ny_)) ny_ = 0; }
          }
          header_parsed = (nx_ > 0 && ny_ > 0 && cell_ > 0.0);
        }
        continue;
      }
      std::istringstream stream(line);
      double value = 0.0;
      while (stream >> value)
        values_.push_back(value);
    }

    if (!header_parsed)
      return false;
    if (values_.size() != static_cast<size_t>(nx_) * static_cast<size_t>(ny_))
      throw std::runtime_error("ground grid size mismatch in " + path + ": got " +
                               std::to_string(values_.size()) + ", expected " +
                               std::to_string(nx_ * ny_));
    loaded_ = true;
    return true;
  }

  bool loaded() const { return loaded_; }
  double cell() const { return cell_; }

  /// 双线性插值查询；超出网格范围时夹取到边界，保证不会返回 NaN。
  double heightAt(double x, double y) const
  {
    if (!loaded_)
      return 0.0;
    const double fx = (x - x0_) / cell_ - 0.5;
    const double fy = (y - y0_) / cell_ - 0.5;
    const int i0 = clampIndex(static_cast<int>(std::floor(fx)), nx_);
    const int j0 = clampIndex(static_cast<int>(std::floor(fy)), ny_);
    const int i1 = std::min(i0 + 1, nx_ - 1);
    const int j1 = std::min(j0 + 1, ny_ - 1);
    const double tx = std::clamp(fx - std::floor(fx), 0.0, 1.0);
    const double ty = std::clamp(fy - std::floor(fy), 0.0, 1.0);

    const double v00 = at(i0, j0);
    const double v10 = at(i1, j0);
    const double v01 = at(i0, j1);
    const double v11 = at(i1, j1);
    const double a = v00 * (1.0 - tx) + v10 * tx;
    const double b = v01 * (1.0 - tx) + v11 * tx;
    return a * (1.0 - ty) + b * ty;
  }

  double minHeight() const
  {
    return values_.empty() ? 0.0 : *std::min_element(values_.begin(), values_.end());
  }
  double maxHeight() const
  {
    return values_.empty() ? 0.0 : *std::max_element(values_.begin(), values_.end());
  }

private:
  static int clampIndex(int index, int size)
  {
    return std::max(0, std::min(index, size - 1));
  }
  double at(int i, int j) const
  {
    return values_[static_cast<size_t>(j) * static_cast<size_t>(nx_) + static_cast<size_t>(i)];
  }

  std::vector<double> values_;
  double cell_{0.0}, x0_{0.0}, y0_{0.0};
  int nx_{0}, ny_{0};
  bool loaded_{false};
};
}  // namespace scan_planner

#endif  // PLAN_MANAGE_GROUND_HEIGHT_MAP_H
