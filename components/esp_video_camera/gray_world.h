#pragma once
// Gray-world white balance for the JPEG copy only (pure, host-testable).
// Output-space R/B gains from the frame mean; never touches the ISP/CCM the
// detector shares (on P4 rev<3 ISP WB folds into the CCM and breaks it).
// ponytail: whole-frame gray-world; a scene dominated by one color tints it.
#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace esp_video_camera {

// BGR888 in memory (ESP-IDF RGB888 convention). Writes the balanced copy to dst.
inline void gray_world_copy(const uint8_t *src, uint8_t *dst, size_t pixels) {
  uint64_t sb = 0, sg = 0, sr = 0;
  for (size_t i = 0; i < pixels; i += 16) {
    sb += src[i * 3];
    sg += src[i * 3 + 1];
    sr += src[i * 3 + 2];
  }
  // 8.8 fixed-point gains, clamped to 0.25..4.
  uint32_t kr = sr ? std::clamp<uint64_t>(sg * 256 / sr, 64, 1024) : 256;
  uint32_t kb = sb ? std::clamp<uint64_t>(sg * 256 / sb, 64, 1024) : 256;
  for (size_t i = 0; i < pixels * 3; i += 3) {
    dst[i] = std::min<uint32_t>(255, (src[i] * kb) >> 8);
    dst[i + 1] = src[i + 1];
    dst[i + 2] = std::min<uint32_t>(255, (src[i + 2] * kr) >> 8);
  }
}

}  // namespace esp_video_camera
}  // namespace esphome
