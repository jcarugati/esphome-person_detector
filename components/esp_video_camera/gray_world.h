#pragma once
// Gray-world white balance step (pure, host-testable). Takes mean R/G/B of the
// current (already balanced) frame and nudges the ISP red/blue gains so R and B
// match G. Square-root damping + clamps keep one odd frame from swinging it.
// ponytail: whole-frame gray-world; a scene dominated by one color tints it.
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_video_camera {

inline bool gray_world_step(uint64_t sum_r, uint64_t sum_g, uint64_t sum_b, float &red_gain, float &blue_gain) {
  if (sum_r == 0 || sum_g == 0 || sum_b == 0)
    return false;
  float r = red_gain * std::sqrt(static_cast<float>(sum_g) / sum_r);
  float b = blue_gain * std::sqrt(static_cast<float>(sum_g) / sum_b);
  r = std::clamp(r, 0.25f, 4.0f);
  b = std::clamp(b, 0.25f, 4.0f);
  bool changed = std::fabs(r - red_gain) > 0.01f || std::fabs(b - blue_gain) > 0.01f;
  red_gain = r;
  blue_gain = b;
  return changed;
}

}  // namespace esp_video_camera
}  // namespace esphome
