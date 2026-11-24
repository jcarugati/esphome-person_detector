// g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -I components test/h264_rtp_test.cpp -o /tmp/h264_rtp && /tmp/h264_rtp
#include <array>
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "esp_video_camera/h264_rtp.h"

using esphome::esp_video_camera::H264Geometry;
using esphome::esp_video_camera::H264SpropParams;
using esphome::esp_video_camera::H264_RTP_INTERLEAVED_BUF_SIZE;
using esphome::esp_video_camera::base64_encode;
using esphome::esp_video_camera::compute_h264_geometry;
using esphome::esp_video_camera::constant_time_equals;
using esphome::esp_video_camera::extract_h264_sps_pps;
using esphome::esp_video_camera::format_rtsp_sdp;
using esphome::esp_video_camera::packetize_h264_annex_b;
using esphome::esp_video_camera::packetize_h264_annex_b_interleaved;
using esphome::esp_video_camera::rtp_timestamp_90khz;

struct Packet {
  std::vector<uint8_t> bytes;
  bool marker;
};

int main() {
  // 1. Bare Annex-B RTP packetizer (SPS, PPS, fragmented IDR).
  const uint8_t access_unit[] = {
      0, 0, 0, 1, 0x67, 0x42, 0xC0, 0x1E, 0x99,
      0, 0, 1, 0x68, 0xCE, 0x3C, 0x80,
      0, 0, 1, 0x65, 1, 2, 3, 4, 5, 6, 7,
  };
  uint16_t seq = 41;
  std::vector<Packet> packets;
  packetize_h264_annex_b(access_unit, sizeof(access_unit), 9000, seq, 5,
                         [&](const uint8_t *data, size_t len, bool marker) {
                           packets.push_back({std::vector<uint8_t>(data, data + len), marker});
                         });

  assert(seq == 46);
  assert(packets.size() == 5);
  assert((packets[0].bytes[1] & 0x7F) == 96);
  assert(packets[0].bytes[12] == 0x67);          // SPS stays a single NAL packet
  assert((packets[2].bytes[12] & 0x1F) == 28);   // IDR fragmented as FU-A
  assert((packets[2].bytes[13] & 0x80) != 0);    // first fragment
  assert((packets[4].bytes[13] & 0x40) != 0);    // last fragment
  for (size_t i = 0; i + 1 < packets.size(); ++i)
    assert(!packets[i].marker);
  assert(packets.back().marker);
  assert((packets.back().bytes[1] & 0x80) != 0);

  // 2. Interleaved TCP packetizer: exact `$` + ch + 16-bit BE len framing,
  //    zero-copy into caller scratch, and immediate abort on emit()==false.
  std::array<uint8_t, H264_RTP_INTERLEAVED_BUF_SIZE> scratch{};
  uint16_t iseq = 41;
  std::vector<Packet> ipackets;
  const bool ok_all = packetize_h264_annex_b_interleaved(
      access_unit, sizeof(access_unit), 9000, iseq, 5, 0, scratch.data(), scratch.size(),
      [&](const uint8_t *framed, size_t len, bool marker) {
        assert(framed == scratch.data());
        assert(len >= 4);
        assert(framed[0] == '$');
        assert(framed[1] == 0);
        const size_t rtp_len =
            (static_cast<size_t>(framed[2]) << 8) | static_cast<size_t>(framed[3]);
        assert(rtp_len + 4 == len);
        ipackets.push_back({std::vector<uint8_t>(framed + 4, framed + len), marker});
        return true;
      });
  assert(ok_all);
  assert(iseq == seq);
  assert(ipackets.size() == packets.size());
  for (size_t i = 0; i < packets.size(); ++i) {
    assert(ipackets[i].bytes == packets[i].bytes);
    assert(ipackets[i].marker == packets[i].marker);
  }

  // Abort on 2nd packet stops immediately without emitting remaining packets.
  size_t emitted = 0;
  uint16_t aseq = 100;
  const bool aborted = packetize_h264_annex_b_interleaved(
      access_unit, sizeof(access_unit), 9000, aseq, 5, 0, scratch.data(), scratch.size(),
      [&](const uint8_t *, size_t, bool) {
        ++emitted;
        return emitted < 2;
      });
  assert(!aborted);
  assert(emitted == 2);
  assert(aseq == 102);

  // Undersized scratch fails closed.
  assert(!packetize_h264_annex_b_interleaved(access_unit, sizeof(access_unit), 0, aseq, 100, 0,
                                             scratch.data(), 100,
                                             [](const uint8_t *, size_t, bool) { return true; }));

  // 3. 90 kHz wall-clock RTP timestamp math (including multi-year uptime without overflow).
  assert(rtp_timestamp_90khz(-5000) == 0);
  assert(rtp_timestamp_90khz(0) == 0);
  assert(rtp_timestamp_90khz(66667) == 6000);     // 15 fps = 6000 ticks
  assert(rtp_timestamp_90khz(100000) == 9000);    // 10 fps = 9000 ticks
  assert(rtp_timestamp_90khz(1000000) == 90000);  // 1 s = 90000 ticks
  // 10 years in microseconds (~3.1536e14 us): naive (us * 90000) overflows int64_t!
  const int64_t ten_years_us = 10LL * 365LL * 86400LL * 1000000LL;
  const uint32_t ts_10y = rtp_timestamp_90khz(ten_years_us);
  const uint32_t ts_10y_plus_1s = rtp_timestamp_90khz(ten_years_us + 1000000LL);
  assert(static_cast<uint32_t>(ts_10y_plus_1s - ts_10y) == 90000u);
  const uint32_t ts_10y_plus_frame = rtp_timestamp_90khz(ten_years_us + 66667LL);
  assert(static_cast<uint32_t>(ts_10y_plus_frame - ts_10y) == 6000u);

  // 4. H.264 16px-aligned geometry + PPA crop box (landscape & portrait).
  const H264Geometry land = compute_h264_geometry(1280, 720, 0);
  assert(land.crop_w == 1280 && land.crop_h == 704);
  assert(land.crop_x == 0 && land.crop_y == 8);
  assert(land.out_w == 640 && land.out_h == 352);
  assert(land.out_w % 16 == 0 && land.out_h % 16 == 0);

  const H264Geometry port = compute_h264_geometry(1280, 720, 90);
  assert(port.crop_w == 1280 && port.crop_h == 704);
  assert(port.crop_x == 0 && port.crop_y == 8);
  assert(port.out_w == 352 && port.out_h == 640);
  assert(port.out_w % 16 == 0 && port.out_h % 16 == 0);

  // 5. Base64 RFC 4648 vectors + SPS/PPS extraction + SDP formatting.
  assert(base64_encode(reinterpret_cast<const uint8_t *>("f"), 1) == "Zg==");
  assert(base64_encode(reinterpret_cast<const uint8_t *>("fo"), 2) == "Zm8=");
  assert(base64_encode(reinterpret_cast<const uint8_t *>("foo"), 3) == "Zm9v");
  assert(base64_encode(reinterpret_cast<const uint8_t *>("foobar"), 6) == "Zm9vYmFy");

  H264SpropParams sprop{};
  assert(extract_h264_sps_pps(access_unit, sizeof(access_unit), sprop));
  assert(sprop.valid());
  assert(sprop.profile_level_id == "42C01E");
  assert(sprop.sprop_parameter_sets == "Z0LAHpk=,aM48gA==");

  // P-frame without SPS/PPS fails extraction cleanly.
  const uint8_t p_frame[] = {0, 0, 0, 1, 0x41, 0x9A, 0x01, 0x02};
  H264SpropParams missing{};
  assert(!extract_h264_sps_pps(p_frame, sizeof(p_frame), missing));
  assert(!missing.valid());

  // SDP without cached sprop still declares framerate:15 and framesize.
  const std::string sdp_plain = format_rtsp_sdp(352, 640, 15, nullptr);
  assert(sdp_plain.find("a=framerate:15\r\n") != std::string::npos);
  assert(sdp_plain.find("a=framesize:96 352-640\r\n") != std::string::npos);
  assert(sdp_plain.find("a=fmtp:96 packetization-mode=1\r\n") != std::string::npos);

  // SDP with cached sprop includes profile-level-id and sprop-parameter-sets.
  const std::string sdp_full = format_rtsp_sdp(352, 640, 15, &sprop);
  assert(sdp_full.find("a=framerate:15\r\n") != std::string::npos);
  assert(sdp_full.find("a=framesize:96 352-640\r\n") != std::string::npos);
  assert(sdp_full.find("a=fmtp:96 packetization-mode=1;profile-level-id=42C01E;"
                       "sprop-parameter-sets=Z0LAHpk=,aM48gA==\r\n") != std::string::npos);

  // 6. Constant-time auth comparison.
  assert(constant_time_equals("Basic dXNlcjpwYXNz", "Basic dXNlcjpwYXNz"));
  assert(!constant_time_equals("Basic dXNlcjpwYXNz", "Basic dXNlcjpwYXN0"));
  assert(!constant_time_equals("Basic dXNlcjpwYXNz", "Basic dXNlcjpwYXN"));
  assert(!constant_time_equals("", "Basic dXNlcjpwYXNz"));
  assert(constant_time_equals("", ""));

  return 0;
}
