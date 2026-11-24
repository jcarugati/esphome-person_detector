#pragma once
// Gray-world white balance for streaming/snapshot output only (pure, host-testable).
// Output-space R/B gains (BGR888) and U/V chroma offsets (YUV420 O_UYY_E_VYY)
// from mid-luma pixels with EMA smoothing and 256-entry LUTs; never touches the
// ISP/CCM or the detector's rotated_ frame (on P4 rev<3 ISP WB folds into the
// CCM and breaks it).
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

// Stateless out-of-place LUT white balance for on-demand full-res snapshots so
// the detector's rotated_ buffer is never mutated.
inline void gray_world_copy(const uint8_t *src, uint8_t *dst, size_t pixels) {
  if (src == nullptr || dst == nullptr || pixels == 0)
    return;
  GrayWorldState oneshot{};
  gray_world_update_gains(src, pixels, oneshot);
  uint8_t lut_b[256];
  uint8_t lut_r[256];
  gray_world_build_lut(oneshot.kb, lut_b);
  gray_world_build_lut(oneshot.kr, lut_r);
  const size_t bytes = pixels * 3;
  for (size_t i = 0; i < bytes; i += 3) {
    dst[i] = lut_b[src[i]];
    dst[i + 1] = src[i + 1];
    dst[i + 2] = lut_r[src[i + 2]];
  }
}

// Maximum chroma correction in BT.601 U/V units so a single-color saturated
// scene cannot over-shift white balance.
static constexpr int32_t YUV_AWB_MAX_BIAS = 48;
static constexpr uint8_t YUV_AWB_CHROMA_MIN = 16;
static constexpr uint8_t YUV_AWB_CHROMA_MAX = 240;

struct YuvChromaAwbState {
  int32_t bias_u{0};
  int32_t bias_v{0};
  bool initialized{false};

  void reset() {
    this->bias_u = 0;
    this->bias_v = 0;
    this->initialized = false;
  }
};

// Signed EMA step with alpha = 1/8 (0.125) and +-1 minimum step so integer
// chroma biases converge cleanly without a rounding deadband.
inline int32_t gray_world_ema_step_signed(int32_t current, int32_t target) {
  const int32_t diff = target - current;
  if (diff == 0)
    return current;
  int32_t delta = diff / 8;
  if (delta == 0)
    delta = diff > 0 ? 1 : -1;
  return current + delta;
}

inline void yuv_chroma_build_lut(int32_t bias, uint8_t lut[256]) {
  for (int32_t v = 0; v < 256; ++v) {
    const int32_t shifted = v + bias;
    if (shifted < YUV_AWB_CHROMA_MIN)
      lut[v] = YUV_AWB_CHROMA_MIN;
    else if (shifted > YUV_AWB_CHROMA_MAX)
      lut[v] = YUV_AWB_CHROMA_MAX;
    else
      lut[v] = static_cast<uint8_t>(shifted);
  }
}

// Fast in-place chroma LUT white balance for ESP32-P4 PPA/H.264 packed YUV420
// (`ESP_H264_RAW_FMT_O_UYY_E_VYY`):
//   even row y   (1-indexed odd):  [U0, Y00, Y01, U1, Y02, Y03, ...]
//   odd  row y+1 (1-indexed even): [V0, Y10, Y11, V1, Y12, Y13, ...]
// Each row is (width * 3) / 2 bytes; width and height must be even.
// Remaps only chroma bytes (1/3 of buffer), never touches luma or ISP.
inline void gray_world_yuv420_oyy_inplace(uint8_t *buf, uint16_t width, uint16_t height,
                                          YuvChromaAwbState &state) {
  if (buf == nullptr || width < 2 || height < 2 || (width & 1u) != 0 || (height & 1u) != 0)
    return;

  const size_t row_stride = (static_cast<size_t>(width) * 3u) / 2u;

  uint64_t mid_u = 0, mid_v = 0;
  size_t mid_count = 0;
  uint64_t all_u = 0, all_v = 0;
  size_t all_count = 0;

  // Subsample every 2nd row pair (4 rows) and every 4th 2x2 block (12 bytes).
  for (size_t y = 0; y < height; y += 4) {
    const uint8_t *row_u = buf + y * row_stride;
    const uint8_t *row_v = row_u + row_stride;
    for (size_t k = 0; k < row_stride; k += 12) {
      const uint32_t y_avg =
          (static_cast<uint32_t>(row_u[k + 1]) + static_cast<uint32_t>(row_u[k + 2]) +
           static_cast<uint32_t>(row_v[k + 1]) + static_cast<uint32_t>(row_v[k + 2])) >>
          2;
      if (y_avg <= 16u)
        continue;

      const uint32_t u = row_u[k];
      const uint32_t v = row_v[k];
      all_u += u;
      all_v += v;
      ++all_count;

      if (y_avg >= GRAY_WORLD_LUMA_MIN && y_avg <= GRAY_WORLD_LUMA_MAX) {
        mid_u += u;
        mid_v += v;
        ++mid_count;
      }
    }
  }

  if (all_count == 0)
    return;

  const bool use_mid = (mid_count >= 16) || (mid_count * 16 >= all_count);
  const size_t count = use_mid ? mid_count : all_count;
  const uint64_t su = use_mid ? mid_u : all_u;
  const uint64_t sv = use_mid ? mid_v : all_v;

  const int32_t mean_u = static_cast<int32_t>((su + count / 2u) / count);
  const int32_t mean_v = static_cast<int32_t>((sv + count / 2u) / count);
  const int32_t target_u =
      std::clamp<int32_t>(128 - mean_u, -YUV_AWB_MAX_BIAS, YUV_AWB_MAX_BIAS);
  const int32_t target_v =
      std::clamp<int32_t>(128 - mean_v, -YUV_AWB_MAX_BIAS, YUV_AWB_MAX_BIAS);

  if (!state.initialized) {
    state.bias_u = target_u;
    state.bias_v = target_v;
    state.initialized = true;
  } else {
    state.bias_u = gray_world_ema_step_signed(state.bias_u, target_u);
    state.bias_v = gray_world_ema_step_signed(state.bias_v, target_v);
  }

  if (state.bias_u == 0 && state.bias_v == 0)
    return;

  uint8_t lut_u[256];
  uint8_t lut_v[256];
  yuv_chroma_build_lut(state.bias_u, lut_u);
  yuv_chroma_build_lut(state.bias_v, lut_v);

  for (size_t y = 0; y < height; y += 2) {
    uint8_t *row_u = buf + y * row_stride;
    for (size_t k = 0; k < row_stride; k += 3)
      row_u[k] = lut_u[row_u[k]];
    uint8_t *row_v = row_u + row_stride;
    for (size_t k = 0; k < row_stride; k += 3)
      row_v[k] = lut_v[row_v[k]];
  }
}

}  // namespace esp_video_camera
}  // namespace esphome
