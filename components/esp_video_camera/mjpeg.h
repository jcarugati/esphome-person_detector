#pragma once

#include <cstddef>
#include <cstdio>

namespace esphome {
namespace esp_video_camera {

// Minimum spacing between MJPEG parts (~10 fps). Tune if Wi-Fi/CPU allows.
static constexpr long long MJPEG_MIN_FRAME_US = 100000;

static constexpr const char *MJPEG_CONTENT_TYPE =
    "multipart/x-mixed-replace; boundary=frame";

inline int format_mjpeg_part(char *out, size_t capacity, size_t jpeg_len) {
  return snprintf(out, capacity,
                  "--frame\r\nContent-Type: image/jpeg\r\n"
                  "Content-Length: %u\r\n\r\n",
                  static_cast<unsigned>(jpeg_len));
}

}  // namespace esp_video_camera
}  // namespace esphome
