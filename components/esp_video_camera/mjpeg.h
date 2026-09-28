#pragma once

#include <cstddef>
#include <cstdio>

namespace esphome {
namespace esp_video_camera {

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
