// g++ -std=c++17 -Wall -Wextra -Werror -I components test/mjpeg_test.cpp -o /tmp/mjpeg && /tmp/mjpeg
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#include "esp_video_camera/mjpeg.h"

using esphome::esp_video_camera::JpegRequestState;
using esphome::esp_video_camera::MJPEG_PART_HEADROOM;
using esphome::esp_video_camera::MJPEG_PART_TAILROOM;
using esphome::esp_video_camera::format_mjpeg_part;
using esphome::esp_video_camera::frame_mjpeg_in_place;
using esphome::esp_video_camera::mjpeg_half_resolution;

int main() {
  // 1. Multipart part header formatting and truncation detection.
  char part[96];
  int len = format_mjpeg_part(part, sizeof(part), 12345);
  const char expected[] =
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 12345\r\n\r\n";
  assert(len == static_cast<int>(std::strlen(expected)));
  assert(std::strcmp(part, expected) == 0);
  char short_part[8];
  assert(format_mjpeg_part(short_part, sizeof(short_part), 12345) >=
         static_cast<int>(sizeof(short_part)));

  // 2. Half-resolution downscaling for landscape, portrait, and odd dimensions.
  uint16_t sw = 0, sh = 0;
  mjpeg_half_resolution(1280, 720, sw, sh);
  assert(sw == 640 && sh == 360);
  mjpeg_half_resolution(720, 1280, sw, sh);
  assert(sw == 360 && sh == 640);
  mjpeg_half_resolution(642, 362, sw, sh);
  assert(sw == 320 && sh == 180);

  // 3. In-place contiguous MJPEG framing (part header + JPEG + CRLF).
  const uint8_t fake_jpeg[] = {0xFF, 0xD8, 0x11, 0x22, 0x33, 0xFF, 0xD9};
  std::vector<uint8_t> raw(MJPEG_PART_HEADROOM + sizeof(fake_jpeg) + MJPEG_PART_TAILROOM, 0xAA);
  uint8_t *jpeg_buf = raw.data() + MJPEG_PART_HEADROOM;
  std::memcpy(jpeg_buf, fake_jpeg, sizeof(fake_jpeg));

  const uint8_t *framed = nullptr;
  size_t framed_len = 0;
  assert(frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, sizeof(fake_jpeg),
                              MJPEG_PART_TAILROOM, framed, framed_len));
  const char expected_hdr[] =
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 7\r\n\r\n";
  const size_t hdr_len = std::strlen(expected_hdr);
  assert(framed == jpeg_buf - hdr_len);
  assert(framed_len == hdr_len + sizeof(fake_jpeg) + 2);
  assert(std::memcmp(framed, expected_hdr, hdr_len) == 0);
  assert(std::memcmp(framed + hdr_len, fake_jpeg, sizeof(fake_jpeg)) == 0);
  assert(framed[framed_len - 2] == '\r' && framed[framed_len - 1] == '\n');

  // Invalid framing cases fail closed.
  assert(!frame_mjpeg_in_place(jpeg_buf, 16, sizeof(fake_jpeg), MJPEG_PART_TAILROOM, framed, framed_len));
  assert(!frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, 0, MJPEG_PART_TAILROOM, framed, framed_len));
  assert(!frame_mjpeg_in_place(jpeg_buf, MJPEG_PART_HEADROOM, sizeof(fake_jpeg), 1, framed, framed_len));

  // 4. JpegRequestState handshake: happy path, stale wakeup, privacy-off abort,
  //    timeout/late-producer race, and encode failure (0 bytes).
  JpegRequestState st{};
  // Producer cannot complete when nothing is pending.
  assert(!st.complete_request(1000));

  // Happy path.
  uint32_t tok1 = st.begin_request();
  assert(st.pending);
  assert(st.complete_request(24500));
  assert(!st.pending);
  size_t out_len = 0;
  assert(st.finish_wait(tok1, true, true, out_len));
  assert(out_len == 24500);

  // Privacy switch OFF wakes waiter before producer encodes -> rejected even if
  // old jpeg_len is non-zero (generation mismatch + camera_streaming false).
  uint32_t tok2 = st.begin_request();
  assert(!st.finish_wait(tok2, true, false, out_len));
  assert(out_len == 0 && !st.pending);

  // Privacy OFF -> rapid ON wakeup without an encode -> generation mismatch still rejects!
  uint32_t tok3 = st.begin_request();
  assert(!st.finish_wait(tok3, true, true, out_len));
  assert(out_len == 0 && !st.pending);

  // Consumer timeout clears pending so a late producer aborts cleanly.
  uint32_t tok4 = st.begin_request();
  assert(!st.finish_wait(tok4, false, true, out_len));
  assert(out_len == 0 && !st.pending);
  assert(!st.complete_request(12345));

  // Encode error (len == 0) wakes waiter and fails cleanly.
  uint32_t tok5 = st.begin_request();
  assert(st.complete_request(0));
  assert(!st.finish_wait(tok5, true, true, out_len));
  assert(out_len == 0);

  return 0;
}
