#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace esp_video_camera {

// Minimum spacing between MJPEG parts (~10 fps). Tune if Wi-Fi/CPU allows.
static constexpr long long MJPEG_MIN_FRAME_US = 100000;

// Cache-line-aligned headroom/tailroom around the HW JPEG output buffer so
// multipart header + JPEG + CRLF can be framed in place and sent in one chunk.
static constexpr size_t MJPEG_PART_HEADROOM = 128;
static constexpr size_t MJPEG_PART_TAILROOM = 128;
static constexpr uint8_t MJPEG_JPEG_QUALITY = 70;

static constexpr const char *MJPEG_CONTENT_TYPE =
    "multipart/x-mixed-replace; boundary=frame";

inline int format_mjpeg_part(char *out, size_t capacity, size_t jpeg_len) {
  return snprintf(out, capacity,
                  "--frame\r\nContent-Type: image/jpeg\r\n"
                  "Content-Length: %u\r\n\r\n",
                  static_cast<unsigned>(jpeg_len));
}

// Compute even half-resolution stream dimensions from the rotated output size
// (e.g. 1280x720 -> 640x360, 720x1280 -> 360x640).
inline void mjpeg_half_resolution(uint16_t out_w, uint16_t out_h, uint16_t &stream_w, uint16_t &stream_h) {
  stream_w = static_cast<uint16_t>((out_w / 2u) & ~1u);
  stream_h = static_cast<uint16_t>((out_h / 2u) & ~1u);
}

// Prepend the multipart part header in [jpeg_buf - headroom, jpeg_buf) and
// append "\r\n" at [jpeg_buf + jpeg_len, jpeg_buf + jpeg_len + 2) so one
// contiguous write carries the complete MJPEG part.
inline bool frame_mjpeg_in_place(uint8_t *jpeg_buf, size_t headroom, size_t jpeg_len, size_t tailroom,
                                 const uint8_t *&frame_out, size_t &frame_len) {
  if (jpeg_buf == nullptr || jpeg_len == 0 || tailroom < 2)
    return false;
  char part[96];
  const int part_len = format_mjpeg_part(part, sizeof(part), jpeg_len);
  if (part_len <= 0 || static_cast<size_t>(part_len) >= sizeof(part) ||
      static_cast<size_t>(part_len) > headroom) {
    return false;
  }
  uint8_t *start = jpeg_buf - part_len;
  std::memcpy(start, part, static_cast<size_t>(part_len));
  jpeg_buf[jpeg_len] = '\r';
  jpeg_buf[jpeg_len + 1] = '\n';
  frame_out = start;
  frame_len = static_cast<size_t>(part_len) + jpeg_len + 2;
  return true;
}

// Mutex-protected handshake between one HTTP consumer and the frame producer.
// Generation tokens ensure stale completions or privacy-off wakeups never
// return a previous or half-written JPEG buffer.
struct JpegRequestState {
  uint32_t req_gen{0};
  uint32_t done_gen{0};
  size_t jpeg_len{0};
  bool pending{false};

  uint32_t begin_request() {
    ++this->req_gen;
    this->pending = true;
    return this->req_gen;
  }

  bool complete_request(size_t len) {
    if (!this->pending)
      return false;
    this->jpeg_len = len;
    this->done_gen = this->req_gen;
    this->pending = false;
    return true;
  }

  bool finish_wait(uint32_t token, bool waited_ok, bool camera_streaming, size_t &out_len) {
    this->pending = false;
    if (!waited_ok || !camera_streaming || this->done_gen != token || this->jpeg_len == 0) {
      out_len = 0;
      return false;
    }
    out_len = this->jpeg_len;
    return true;
  }
};

}  // namespace esp_video_camera
}  // namespace esphome
