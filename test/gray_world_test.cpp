// g++ -std=c++17 -I components test/gray_world_test.cpp -o /tmp/gw && /tmp/gw
#include <cassert>
#include "esp_video_camera/gray_world.h"
using esphome::esp_video_camera::gray_world_step;
int main() {
  // Magenta frame (R,B > G) converges to neutral.
  float rg = 1, bg = 1, R = 120, G = 101, B = 135;
  for (int i = 0; i < 20; i++) gray_world_step(R * rg * 1000, G * 1000, B * bg * 1000, rg, bg);
  assert(std::fabs(R * rg - G) < 1 && std::fabs(B * bg - G) < 1);
  // Neutral frame: no change.
  float a = 1, b = 1;
  assert(!gray_world_step(100, 100, 100, a, b) && a == 1 && b == 1);
  // Degenerate / extreme input: guarded and clamped.
  assert(!gray_world_step(0, 100, 100, a, b));
  for (int i = 0; i < 50; i++) gray_world_step(1, 1000000, 1, a, b);
  assert(a == 4.0f && b == 4.0f);
  return 0;
}
