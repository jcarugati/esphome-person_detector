#pragma once
// Gray-world white balance for JPEG output only (pure, host-testable).
// Output-space R/B gains from mid-luma pixels with EMA smoothing and 256-entry
// LUTs; never touches the ISP/CCM or the detector's rotated_ frame (on P4 rev<3
// ISP WB folds into the CCM and breaks it).
// ponytail: mid-luma gray-world with fixed alpha=1/8 EMA; a scene dominated by
// one saturated color still tints it.

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace esp_video_camera {

static constexpr uint32_t GRAY_WORLD_UNITY_GAIN = 256;
static constexpr uint32_t GRAY_WORLD_MIN_GAIN = 64;    // 0.25x in 8.8 fixed-point
static constexpr uint32_t GRAY_WORLD_MAX_GAIN = 1024;  // 4.00x in 8.8 fixed-point
static constexpr uint32_t GRAY_WORLD_LUMA_MIN = 40;
static constexpr uint32_t GRAY_WORLD_LUMA_MAX = 220;
static constexpr size_t GRAY_WORLD_SAMPLE_STRIDE = 16;

struct GrayWorldState {
  uint32_t kr{GRAY_WORLD_UNITY_GAIN};
  uint32_t kb{GRAY_WORLD_UNITY_GAIN};
  bool initialized{false};

  void reset() {
    this->kr = GRAY_WORLD_UNITY_GAIN;
    this->kb = GRAY_WORLD_UNITY_GAIN;
    this->initialized = false;
  }
};

// EMA step with alpha = 1/8 (0.125) and +-1 minimum step so fixed-point gains
// converge cleanly to the target without a rounding deadband.
inline uint32_t gray_world_ema_step(uint32_t current, uint32_t target) {
  const int32_t diff = static_cast<int32_t>(target) - static_cast<int32_t>(current);
  if (diff == 0)
    return current;
  int32_t delta = diff / 8;
  if (delta == 0)
    delta = diff > 0 ? 1 : -1;
  return static_cast<uint32_t>(static_cast<int32_t>(current) + delta);
}

// Estimate target 8.8 fixed-point R/B gains from BGR888 pixels (ESP-IDF RGB888
// memory order) and update EMA state. Prefers mid-luma pixels Y in [40, 220]
// so clipped highlights and deep shadows do not skew white balance, falling
// back to all non-black pixels when few mid-luma samples exist.
inline void gray_world_update_gains(const uint8_t *bgr, size_t pixels, GrayWorldState &state) {
  if (bgr == nullptr || pixels == 0)
    return;

  uint64_t mid_b = 0, mid_g = 0, mid_r = 0;
  size_t mid_count = 0;
  uint64_t all_b = 0, all_g = 0, all_r = 0;
  size_t all_count = 0;

  for (size_t i = 0; i < pixels; i += GRAY_WORLD_SAMPLE_STRIDE) {
    const uint32_t b = bgr[i * 3];
    const uint32_t g = bgr[i * 3 + 1];
    const uint32_t r = bgr[i * 3 + 2];
    if ((b | g | r) == 0)
      continue;

    all_b += b;
    all_g += g;
    all_r += r;
    ++all_count;

    // Rec.601 luma for BGR888: Y = 0.114*B + 0.587*G + 0.299*R
    const uint32_t y = (29u * b + 150u * g + 77u * r) >> 8;
    if (y >= GRAY_WORLD_LUMA_MIN && y <= GRAY_WORLD_LUMA_MAX) {
      mid_b += b;
      mid_g += g;
      mid_r += r;
      ++mid_count;
    }
  }

  if (all_count == 0)
    return;

  const bool use_mid = (mid_count >= 16) || (mid_count * 16 >= all_count);
  const uint64_t sb = use_mid ? mid_b : all_b;
  const uint64_t sg = use_mid ? mid_g : all_g;
  const uint64_t sr = use_mid ? mid_r : all_r;

  const uint32_t target_kr =
      sr ? static_cast<uint32_t>(std::clamp<uint64_t>(sg * 256 / sr, GRAY_WORLD_MIN_GAIN, GRAY_WORLD_MAX_GAIN))
         : GRAY_WORLD_UNITY_GAIN;
  const uint32_t target_kb =
      sb ? static_cast<uint32_t>(std::clamp<uint64_t>(sg * 256 / sb, GRAY_WORLD_MIN_GAIN, GRAY_WORLD_MAX_GAIN))
         : GRAY_WORLD_UNITY_GAIN;

  if (!state.initialized) {
    state.kr = target_kr;
    state.kb = target_kb;
    state.initialized = true;
  } else {
    state.kr = gray_world_ema_step(state.kr, target_kr);
    state.kb = gray_world_ema_step(state.kb, target_kb);
  }
}

inline void gray_world_build_lut(uint32_t gain_q8, uint8_t lut[256]) {
  for (uint32_t v = 0; v < 256; ++v) {
    const uint32_t scaled = (v * gain_q8) >> 8;
    lut[v] = static_cast<uint8_t>(scaled > 255u ? 255u : scaled);
  }
}

// Fast in-place LUT white balance for a dedicated streaming BGR888 buffer
// (zero extra buffer allocation).
inline void gray_world_apply_inplace(uint8_t *bgr, size_t pixels, GrayWorldState &state) {
  if (bgr == nullptr || pixels == 0)
    return;
  gray_world_update_gains(bgr, pixels, state);
  uint8_t lut_b[256];
  uint8_t lut_r[256];
  gray_world_build_lut(state.kb, lut_b);
  gray_world_build_lut(state.kr, lut_r);
  const size_t bytes = pixels * 3;
  for (size_t i = 0; i < bytes; i += 3) {
    bgr[i] = lut_b[bgr[i]];
    bgr[i + 2] = lut_r[bgr[i + 2]];
  }
}

// Out-of-place LUT white balance for full-res snapshots so the detector's
// rotated_ buffer is never mutated.
inline void gray_world_copy(const uint8_t *src, uint8_t *dst, size_t pixels, GrayWorldState &state) {
  if (src == nullptr || dst == nullptr || pixels == 0)
    return;
  gray_world_update_gains(src, pixels, state);
  uint8_t lut_b[256];
  uint8_t lut_r[256];
  gray_world_build_lut(state.kb, lut_b);
  gray_world_build_lut(state.kr, lut_r);
  const size_t bytes = pixels * 3;
  for (size_t i = 0; i < bytes; i += 3) {
    dst[i] = lut_b[src[i]];
    dst[i + 1] = src[i + 1];
    dst[i + 2] = lut_r[src[i + 2]];
  }
}

// Stateless single-frame overload for on-demand snapshots.
inline void gray_world_copy(const uint8_t *src, uint8_t *dst, size_t pixels) {
  GrayWorldState oneshot{};
  gray_world_copy(src, dst, pixels, oneshot);
}

}  // namespace esp_video_camera
}  // namespace esphome
