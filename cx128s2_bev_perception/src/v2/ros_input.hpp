#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <cmath>
namespace crane_v2 {

inline void decode(const sensor_msgs::msg::PointCloud2&m, const tf2::Transform&t, std::vector<Point>&out)
{
  if (m.is_bigendian)
    throw std::runtime_error("V2: big-endian cloud unsupported; no reinterpretation");
  int offsets[3] = {-1, -1, -1};
  for (auto&f:m.fields)
    for (int a = 0; a < 3; ++a)
      if (f.name == std::string(1, "xyz"[a]))
      {
        if (f.datatype != sensor_msgs::msg::PointField::FLOAT32||f.count != 1)
          throw std::runtime_error("V2: XYZ must be float32");
        offsets[a] = int(f.offset);
      }
  for (int a:offsets)
    if (a < 0||uint32_t(a + 4) > m.point_step)
      throw std::runtime_error("V2: invalid XYZ layout");
  if (uint64_t(m.width) * m.point_step > m.row_step||uint64_t(m.row_step) * m.height > m.data.size())
    throw std::runtime_error("V2: truncated cloud");
  out.clear();
  out.reserve(uint64_t(m.width) * m.height);
  for (uint32_t row = 0; row < m.height; ++row)
    for (uint32_t col = 0; col < m.width; ++col)
    {
      const auto*p = m.data.data() + size_t(row) * m.row_step + size_t(col) * m.point_step;
      float v[3];
      for (int a = 0; a < 3; ++a)
        std::memcpy(v + a, p + offsets[a], 4);
      if (!std::isfinite(v[0])||!std::isfinite(v[1])||!std::isfinite(v[2]))
        continue;
      auto w = t * tf2::Vector3(v[0], v[1], v[2]);
      out.push_back({float(w.x()), float(w.y()), float(w.z()), row * m.width + col});
    }
}
}
