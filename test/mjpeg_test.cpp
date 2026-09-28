// g++ -std=c++17 -I components test/mjpeg_test.cpp -o /tmp/mjpeg && /tmp/mjpeg
#include <cassert>
#include <cstring>
#include "esp_video_camera/mjpeg.h"
using esphome::esp_video_camera::format_mjpeg_part;
int main() {
  char part[96];
  int len = format_mjpeg_part(part, sizeof(part), 12345);
  const char expected[] =
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 12345\r\n\r\n";
  assert(len == static_cast<int>(std::strlen(expected)));
  assert(std::strcmp(part, expected) == 0);
  char short_part[8];
  assert(format_mjpeg_part(short_part, sizeof(short_part), 12345) >=
         static_cast<int>(sizeof(short_part)));
  return 0;
}
