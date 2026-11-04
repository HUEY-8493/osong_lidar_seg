#pragma once

#include <cstdint>

namespace cx128s2_bev_perception {

struct PointXYZIRT {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
  uint16_t ring = 0;
  float time = 0.0f;
};

}  // namespace cx128s2_bev_perception

