// g++ -std=c++17 -I components test/gray_world_test.cpp -o /tmp/gw && /tmp/gw
#include <cassert>
#include <cstdlib>
#include <vector>
#include "esp_video_camera/gray_world.h"
using esphome::esp_video_camera::gray_world_copy;
int main() {
  const size_t n = 64 * 64;
  std::vector<uint8_t> src(n * 3), dst(n * 3);
  for (size_t i = 0; i < n; i++) { src[i * 3] = 135; src[i * 3 + 1] = 101; src[i * 3 + 2] = 120; }  // magenta, BGR
  gray_world_copy(src.data(), dst.data(), n);
  assert(std::abs(dst[0] - 101) <= 1 && dst[1] == 101 && std::abs(dst[2] - 101) <= 1);
  assert(src[0] == 135 && src[2] == 120);  // detector frame untouched
  for (auto &v : src) v = 0;  // black frame: no div-by-zero, stays black
  gray_world_copy(src.data(), dst.data(), n);
  for (auto v : dst) assert(v == 0);
  return 0;
}
