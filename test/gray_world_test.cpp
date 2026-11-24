// g++ -std=c++17 -Wall -Wextra -Werror -I components test/gray_world_test.cpp -o /tmp/gw && /tmp/gw
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "esp_video_camera/gray_world.h"

using esphome::esp_video_camera::GRAY_WORLD_MAX_GAIN;
using esphome::esp_video_camera::GRAY_WORLD_MIN_GAIN;
using esphome::esp_video_camera::GRAY_WORLD_UNITY_GAIN;
using esphome::esp_video_camera::GrayWorldState;
using esphome::esp_video_camera::YUV_AWB_CHROMA_MAX;
using esphome::esp_video_camera::YUV_AWB_CHROMA_MIN;
using esphome::esp_video_camera::YUV_AWB_MAX_BIAS;
using esphome::esp_video_camera::YuvChromaAwbState;
using esphome::esp_video_camera::gray_world_apply_inplace;
using esphome::esp_video_camera::gray_world_build_lut;
using esphome::esp_video_camera::gray_world_copy;
using esphome::esp_video_camera::gray_world_update_gains;
using esphome::esp_video_camera::gray_world_yuv420_oyy_inplace;

static void fill_bgr(std::vector<uint8_t> &buf, uint8_t b, uint8_t g, uint8_t r) {
  for (size_t i = 0; i < buf.size(); i += 3) {
    buf[i] = b;
    buf[i + 1] = g;
    buf[i + 2] = r;
  }
}

// Fill an ESP_H264_RAW_FMT_O_UYY_E_VYY buffer with uniform (y, u, v):
//   even row: U Y Y U Y Y ...
//   odd  row: V Y Y V Y Y ...
static void fill_oyy(std::vector<uint8_t> &buf, uint16_t w, uint16_t h,
                     uint8_t y_val, uint8_t u_val, uint8_t v_val) {
  const size_t stride = (static_cast<size_t>(w) * 3u) / 2u;
  buf.resize(stride * h);
  for (size_t row = 0; row < h; row += 2) {
    uint8_t *ru = buf.data() + row * stride;
    uint8_t *rv = ru + stride;
    for (size_t k = 0; k < stride; k += 3) {
      ru[k] = u_val;
      ru[k + 1] = y_val;
      ru[k + 2] = y_val;
      rv[k] = v_val;
      rv[k + 1] = y_val;
      rv[k + 2] = y_val;
    }
  }
}

int main() {
  const size_t n = 64 * 64;
  std::vector<uint8_t> src(n * 3), dst(n * 3);

  // 1. Stateless out-of-place copy balances magenta tint without touching src.
  fill_bgr(src, 135, 101, 120);
  gray_world_copy(src.data(), dst.data(), n);
  assert(std::abs(dst[0] - 101) <= 1 && dst[1] == 101 && std::abs(dst[2] - 101) <= 1);
  assert(src[0] == 135 && src[1] == 101 && src[2] == 120);

  // 2. Pure black frame: no div-by-zero, stays black, does not seed state.
  fill_bgr(src, 0, 0, 0);
  gray_world_copy(src.data(), dst.data(), n);
  for (auto v : dst)
    assert(v == 0);
  GrayWorldState state{};
  gray_world_update_gains(src.data(), n, state);
  assert(!state.initialized);
  assert(state.kr == GRAY_WORLD_UNITY_GAIN && state.kb == GRAY_WORLD_UNITY_GAIN);

  // 3. First valid lit frame seeds immediately; subsequent scene change smooths via EMA.
  fill_bgr(src, 80, 120, 160);  // warm indoor cast: kb=1.5x (384), kr=0.75x (192)
  std::vector<uint8_t> inplace = src;
  gray_world_apply_inplace(inplace.data(), n, state);
  assert(state.initialized);
  assert(state.kb == 384 && state.kr == 192);
  assert(std::abs(inplace[0] - 120) <= 1 && inplace[1] == 120 && std::abs(inplace[2] - 120) <= 1);

  // Shift to a cool scene: target kb=0.75x (192), kr=1.5x (384).
  // After one EMA step (alpha=1/8), gains move 1/8 of the way, not jumping immediately.
  fill_bgr(src, 160, 120, 80);
  gray_world_update_gains(src.data(), n, state);
  assert(state.kb == 360);  // 384 + (192 - 384) / 8 = 360
  assert(state.kr == 216);  // 192 + (384 - 192) / 8 = 216

  // Convergence over repeated frames reaches the exact target without deadband.
  for (int f = 0; f < 64; ++f)
    gray_world_update_gains(src.data(), n, state);
  assert(state.kb == 192 && state.kr == 384);

  // 4. Mid-luma filtering: clipped bright highlights and dark shadows are ignored
  //    when mid-luma samples exist.
  fill_bgr(src, 120, 120, 120);  // neutral mid-luma background (Y = 120)
  // Paint top quarter with bright blue-tinted window (Y > 220) and bottom quarter
  // with deep red-tinted shadow (Y < 40).
  for (size_t i = 0; i < n / 4; ++i) {
    src[i * 3] = 255;
    src[i * 3 + 1] = 240;
    src[i * 3 + 2] = 200;  // Y = 229 > 220
  }
  for (size_t i = (3 * n) / 4; i < n; ++i) {
    src[i * 3] = 5;
    src[i * 3 + 1] = 10;
    src[i * 3 + 2] = 60;  // Y = 24 < 40
  }
  state.reset();
  gray_world_update_gains(src.data(), n, state);
  assert(state.initialized);
  assert(state.kr == GRAY_WORLD_UNITY_GAIN && state.kb == GRAY_WORLD_UNITY_GAIN);

  // 5. Fallback when no mid-luma samples exist (all dim non-black pixels, Y < 40).
  fill_bgr(src, 20, 30, 40);  // Y = 31 < 40
  state.reset();
  gray_world_update_gains(src.data(), n, state);
  assert(state.initialized);
  assert(state.kb == 384 && state.kr == 192);

  // 6. Gain clamping to [0.25x, 4.0x] and LUT saturation at 255.
  fill_bgr(src, 10, 180, 250);  // raw kb=18x -> clamped to 4x; raw kr=0.72x
  state.reset();
  gray_world_update_gains(src.data(), n, state);
  assert(state.kb == GRAY_WORLD_MAX_GAIN);
  fill_bgr(src, 240, 20, 10);  // raw kb=0.083x -> clamped to 0.25x
  state.reset();
  gray_world_update_gains(src.data(), n, state);
  assert(state.kb == GRAY_WORLD_MIN_GAIN);

  uint8_t lut[256];
  gray_world_build_lut(512, lut);  // 2.0x
  assert(lut[0] == 0 && lut[64] == 128 && lut[128] == 255 && lut[255] == 255);

  // 7. In-place matches out-of-place byte-for-byte on fresh initial state.
  fill_bgr(src, 95, 110, 150);
  GrayWorldState s2{};
  std::vector<uint8_t> ip = src;
  gray_world_copy(src.data(), dst.data(), n);
  gray_world_apply_inplace(ip.data(), n, s2);
  assert(s2.initialized && dst == ip);

  // 8. YUV420 O_UYY_E_VYY chroma LUT AWB: warm/magenta tint shifts U/V to 128
  //    without modifying any Y luma byte.
  const uint16_t yw = 64, yh = 32;
  const size_t ystride = (static_cast<size_t>(yw) * 3u) / 2u;
  std::vector<uint8_t> yuv;
  YuvChromaAwbState ystate{};

  fill_oyy(yuv, yw, yh, 120, 108, 152);  // warm cast: U=-20, V=+24
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.initialized);
  assert(ystate.bias_u == 20 && ystate.bias_v == -24);
  for (size_t row = 0; row < yh; row += 2) {
    const uint8_t *ru = yuv.data() + row * ystride;
    const uint8_t *rv = ru + ystride;
    for (size_t k = 0; k < ystride; k += 3) {
      assert(ru[k] == 128);      // U balanced to neutral
      assert(rv[k] == 128);      // V balanced to neutral
      assert(ru[k + 1] == 120 && ru[k + 2] == 120);  // luma untouched
      assert(rv[k + 1] == 120 && rv[k + 2] == 120);  // luma untouched
    }
  }

  // 9. Neutral scene preserves every byte (exact no-op).
  ystate.reset();
  fill_oyy(yuv, yw, yh, 140, 128, 128);
  const std::vector<uint8_t> neutral_copy = yuv;
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.initialized && ystate.bias_u == 0 && ystate.bias_v == 0);
  assert(yuv == neutral_copy);

  // 10. Black frame (Y <= 16): does not seed state and leaves frame untouched.
  ystate.reset();
  fill_oyy(yuv, yw, yh, 16, 90, 180);
  const std::vector<uint8_t> black_copy = yuv;
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(!ystate.initialized && ystate.bias_u == 0 && ystate.bias_v == 0);
  assert(yuv == black_copy);

  // 11. Strongly saturated scene clamps bias to [-YUV_AWB_MAX_BIAS, +YUV_AWB_MAX_BIAS]
  //     and clamps remapped chroma to [16, 240].
  ystate.reset();
  fill_oyy(yuv, yw, yh, 120, 40, 220);  // raw diff: U=+88, V=-92 -> clamped +-48
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.bias_u == YUV_AWB_MAX_BIAS && ystate.bias_v == -YUV_AWB_MAX_BIAS);
  assert(yuv[0] == static_cast<uint8_t>(40 + YUV_AWB_MAX_BIAS));
  assert(yuv[ystride] == static_cast<uint8_t>(220 - YUV_AWB_MAX_BIAS));

  // Verify saturation at BT.601 chroma bounds [16, 240] when bias pushes past limits.
  yuv[0] = 230;        // +48 -> clamped to 240
  yuv[ystride] = 20;   // -48 -> clamped to 16
  // Keep bulk of frame at (40, 220) so bias stays (+48, -48), check corner block (0,0)
  // after re-applying on an already-initialized state.
  fill_oyy(yuv, yw, yh, 120, 40, 220);
  yuv[3] = 230;               // non-sampled block (k=3)
  yuv[ystride + 3] = 20;
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(yuv[3] == YUV_AWB_CHROMA_MAX);
  assert(yuv[ystride + 3] == YUV_AWB_CHROMA_MIN);

  // 12. EMA convergence: step smoothly on scene shift without deadband.
  ystate.reset();
  fill_oyy(yuv, yw, yh, 120, 104, 152);  // seeds bias_u=+24, bias_v=-24
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.bias_u == 24 && ystate.bias_v == -24);

  // Shift to cool cast: target bias_u=-16, bias_v=+16.
  fill_oyy(yuv, yw, yh, 120, 144, 112);
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.bias_u == 19 && ystate.bias_v == -19);  // 24 + (-40)/8 = 19

  for (int f = 0; f < 64; ++f) {
    fill_oyy(yuv, yw, yh, 120, 144, 112);
    gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  }
  assert(ystate.bias_u == -16 && ystate.bias_v == 16);

  // 13. Mid-luma filtering ignores clipped highlights (Y > 220) and deep shadows (Y < 40).
  ystate.reset();
  fill_oyy(yuv, yw, yh, 120, 128, 128);  // neutral mid-luma bulk
  for (size_t row = 0; row < 8; row += 2) {
    uint8_t *ru = yuv.data() + row * ystride;
    uint8_t *rv = ru + ystride;
    for (size_t k = 0; k < ystride; k += 3) {
      ru[k] = 200; ru[k + 1] = 235; ru[k + 2] = 235;
      rv[k] = 60;  rv[k + 1] = 235; rv[k + 2] = 235;
    }
  }
  for (size_t row = yh - 8; row < yh; row += 2) {
    uint8_t *ru = yuv.data() + row * ystride;
    uint8_t *rv = ru + ystride;
    for (size_t k = 0; k < ystride; k += 3) {
      ru[k] = 60;  ru[k + 1] = 25; ru[k + 2] = 25;
      rv[k] = 200; rv[k + 1] = 25; rv[k + 2] = 25;
    }
  }
  gray_world_yuv420_oyy_inplace(yuv.data(), yw, yh, ystate);
  assert(ystate.initialized && ystate.bias_u == 0 && ystate.bias_v == 0);

  return 0;
}
