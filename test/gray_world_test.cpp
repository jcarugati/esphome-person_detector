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
using esphome::esp_video_camera::gray_world_apply_inplace;
using esphome::esp_video_camera::gray_world_build_lut;
using esphome::esp_video_camera::gray_world_copy;
using esphome::esp_video_camera::gray_world_update_gains;

static void fill_bgr(std::vector<uint8_t> &buf, uint8_t b, uint8_t g, uint8_t r) {
  for (size_t i = 0; i < buf.size(); i += 3) {
    buf[i] = b;
    buf[i + 1] = g;
    buf[i + 2] = r;
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
  GrayWorldState state{};
  gray_world_copy(src.data(), dst.data(), n, state);
  for (auto v : dst)
    assert(v == 0);
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

  // 7. In-place matches out-of-place byte-for-byte on identical initial state.
  fill_bgr(src, 95, 110, 150);
  GrayWorldState s1{}, s2{};
  std::vector<uint8_t> ip = src;
  gray_world_copy(src.data(), dst.data(), n, s1);
  gray_world_apply_inplace(ip.data(), n, s2);
  assert(s1.kr == s2.kr && s1.kb == s2.kb && dst == ip);

  return 0;
}
