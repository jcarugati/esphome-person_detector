#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace esphome {
namespace esp_video_camera {

static constexpr uint8_t H264_FPS = 15;
static constexpr uint8_t H264_GOP = 30;  // 2 s keyframes at 15 fps
static constexpr uint32_t H264_BITRATE = 750000;
static constexpr uint8_t H264_QP_MIN = 20;
static constexpr uint8_t H264_QP_MAX = 40;
// 60 ms spacing hits every 2nd 30-fps CSI frame (~15 fps steady) without
// jittering down to 10 fps when two CSI frames arrive 66.5 ms apart.
static constexpr int64_t H264_MIN_FRAME_US = 60000;

static constexpr size_t H264_RTP_MAX_PAYLOAD = 1400;
static constexpr size_t H264_RTP_INTERLEAVED_HEADER = 4;
static constexpr size_t H264_RTP_HEADER = 12;
static constexpr size_t H264_RTP_INTERLEAVED_BUF_SIZE =
    H264_RTP_INTERLEAVED_HEADER + H264_RTP_HEADER + H264_RTP_MAX_PAYLOAD;

struct H264Geometry {
  uint16_t crop_w{0};
  uint16_t crop_h{0};
  uint16_t crop_x{0};
  uint16_t crop_y{0};
  uint16_t out_w{0};
  uint16_t out_h{0};
};

// Compute centered 16px-aligned half-resolution H.264 dimensions and PPA input
// crop window from sensor capture dimensions and rotation.
// Uses exact 0.5x scaling (ESP32-P4 PPA YUV420 requires scale_frag to be a
// multiple of 2/16, so 8/16 = 0.5 is exact in hardware) and even crop offsets.
// For 1280x720:
//   rot 0/180  -> crop 1280x704 @ (0, 8) -> 640x352 (40x22 MBs)
//   rot 90/270 -> crop 1280x704 @ (0, 8) -> 352x640 (22x40 MBs)
inline H264Geometry compute_h264_geometry(uint16_t cap_w, uint16_t cap_h, uint16_t rotation) {
  H264Geometry g{};
  g.crop_w = static_cast<uint16_t>(cap_w & ~31u);
  g.crop_h = static_cast<uint16_t>(cap_h & ~31u);
  g.crop_x = static_cast<uint16_t>(((cap_w - g.crop_w) / 2u) & ~1u);
  g.crop_y = static_cast<uint16_t>(((cap_h - g.crop_h) / 2u) & ~1u);
  const uint16_t half_w = static_cast<uint16_t>(g.crop_w / 2u);
  const uint16_t half_h = static_cast<uint16_t>(g.crop_h / 2u);
  if (rotation == 90 || rotation == 270) {
    g.out_w = half_h;
    g.out_h = half_w;
  } else {
    g.out_w = half_w;
    g.out_h = half_h;
  }
  return g;
}

// Convert elapsed microseconds into a 90 kHz RFC 3550 RTP timestamp without
// intermediate 64-bit overflow even across multi-year uptimes.
inline uint32_t rtp_timestamp_90khz(int64_t elapsed_us) {
  if (elapsed_us <= 0)
    return 0;
  const uint64_t u = static_cast<uint64_t>(elapsed_us);
  const uint64_t secs = u / 1000000ULL;
  const uint64_t rem_us = u % 1000000ULL;
  return static_cast<uint32_t>(secs * 90000ULL + (rem_us * 9ULL) / 100ULL);
}

// Constant-time equality check for RTSP Basic Authorization header comparison.
inline bool constant_time_equals(const std::string &a, const std::string &b) {
  const size_t max_len = a.size() > b.size() ? a.size() : b.size();
  volatile uint8_t diff = (a.size() == b.size()) ? 0u : 1u;
  for (size_t i = 0; i < max_len; ++i) {
    const uint8_t ca = i < a.size() ? static_cast<uint8_t>(a[i]) : 0u;
    const uint8_t cb = i < b.size() ? static_cast<uint8_t>(b[i]) : 0u;
    diff = static_cast<uint8_t>(diff | static_cast<uint8_t>(ca ^ cb));
  }
  return diff == 0u;
}

inline std::string base64_encode(const uint8_t *data, size_t len) {
  static constexpr char TABLE[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  if (data == nullptr || len == 0)
    return {};
  std::string out;
  out.reserve(((len + 2u) / 3u) * 4u);
  for (size_t i = 0; i < len; i += 3) {
    const uint32_t b0 = data[i];
    const uint32_t b1 = (i + 1 < len) ? data[i + 1] : 0u;
    const uint32_t b2 = (i + 2 < len) ? data[i + 2] : 0u;
    const uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
    out.push_back(TABLE[(triple >> 18) & 0x3Fu]);
    out.push_back(TABLE[(triple >> 12) & 0x3Fu]);
    out.push_back((i + 1 < len) ? TABLE[(triple >> 6) & 0x3Fu] : '=');
    out.push_back((i + 2 < len) ? TABLE[triple & 0x3Fu] : '=');
  }
  return out;
}

struct H264SpropParams {
  std::string profile_level_id;       // 6 uppercase hex digits from SPS[1..3]
  std::string sprop_parameter_sets;   // "<base64(SPS)>,<base64(PPS)>"

  bool valid() const {
    return this->profile_level_id.size() == 6 && !this->sprop_parameter_sets.empty();
  }
  void clear() {
    this->profile_level_id.clear();
    this->sprop_parameter_sets.clear();
  }
};

namespace detail {
inline size_t annex_b_start_code(const uint8_t *data, size_t len, size_t from, size_t &prefix) {
  for (size_t i = from; i + 3 <= len; ++i) {
    if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
      prefix = 3;
      return i;
    }
    if (i + 4 <= len && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
      prefix = 4;
      return i;
    }
  }
  return len;
}

inline void rtp_header(uint8_t *out, uint16_t seq, uint32_t timestamp, bool marker) {
  out[0] = 0x80;
  out[1] = static_cast<uint8_t>((marker ? 0x80 : 0) | 96);
  out[2] = static_cast<uint8_t>(seq >> 8);
  out[3] = static_cast<uint8_t>(seq);
  out[4] = static_cast<uint8_t>(timestamp >> 24);
  out[5] = static_cast<uint8_t>(timestamp >> 16);
  out[6] = static_cast<uint8_t>(timestamp >> 8);
  out[7] = static_cast<uint8_t>(timestamp);
  out[8] = 0x45;  // fixed SSRC: "ESPH"
  out[9] = 0x53;
  out[10] = 0x50;
  out[11] = 0x48;
}
}  // namespace detail

// Extract SPS (NAL type 7) and PPS (NAL type 8) from an Annex-B access unit
// into RFC 6184 `profile-level-id` and `sprop-parameter-sets`.
inline bool extract_h264_sps_pps(const uint8_t *data, size_t len, H264SpropParams &out) {
  if (data == nullptr || len == 0)
    return false;

  const uint8_t *sps = nullptr;
  size_t sps_len = 0;
  const uint8_t *pps = nullptr;
  size_t pps_len = 0;

  size_t prefix = 0;
  size_t start = detail::annex_b_start_code(data, len, 0, prefix);
  while (start < len) {
    const size_t nal_start = start + prefix;
    size_t next_prefix = 0;
    const size_t next = detail::annex_b_start_code(data, len, nal_start, next_prefix);
    size_t nal_end = next;
    while (nal_end > nal_start && data[nal_end - 1] == 0)
      --nal_end;
    const size_t nal_len = nal_end - nal_start;
    if (nal_len > 0) {
      const uint8_t nal_type = static_cast<uint8_t>(data[nal_start] & 0x1Fu);
      if (nal_type == 7 && sps == nullptr && nal_len >= 4) {
        sps = data + nal_start;
        sps_len = nal_len;
      } else if (nal_type == 8 && pps == nullptr && nal_len >= 1) {
        pps = data + nal_start;
        pps_len = nal_len;
      }
    }
    if (sps != nullptr && pps != nullptr)
      break;
    start = next;
    prefix = next_prefix;
  }

  if (sps == nullptr || pps == nullptr)
    return false;

  char pli[8];
  snprintf(pli, sizeof(pli), "%02X%02X%02X", sps[1], sps[2], sps[3]);
  out.profile_level_id.assign(pli, 6);
  out.sprop_parameter_sets = base64_encode(sps, sps_len);
  out.sprop_parameter_sets.push_back(',');
  out.sprop_parameter_sets += base64_encode(pps, pps_len);
  return true;
}

// Format the RTSP DESCRIBE SDP body with framerate, framesize, and optional
// cached RFC 6184 profile-level-id + sprop-parameter-sets.
inline std::string format_rtsp_sdp(uint16_t width, uint16_t height, uint8_t fps,
                                   const H264SpropParams *sprop = nullptr) {
  char buf[512];
  int n = 0;
  if (sprop != nullptr && sprop->valid()) {
    n = snprintf(buf, sizeof(buf),
                 "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=ESP32-P4 Camera\r\nt=0 0\r\n"
                 "a=control:*\r\nm=video 0 RTP/AVP 96\r\nc=IN IP4 0.0.0.0\r\n"
                 "a=rtpmap:96 H264/90000\r\na=framerate:%u\r\na=framesize:96 %u-%u\r\n"
                 "a=fmtp:96 packetization-mode=1;profile-level-id=%s;sprop-parameter-sets=%s\r\n"
                 "a=control:trackID=0\r\n",
                 static_cast<unsigned>(fps), static_cast<unsigned>(width),
                 static_cast<unsigned>(height), sprop->profile_level_id.c_str(),
                 sprop->sprop_parameter_sets.c_str());
  } else {
    n = snprintf(buf, sizeof(buf),
                 "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=ESP32-P4 Camera\r\nt=0 0\r\n"
                 "a=control:*\r\nm=video 0 RTP/AVP 96\r\nc=IN IP4 0.0.0.0\r\n"
                 "a=rtpmap:96 H264/90000\r\na=framerate:%u\r\na=framesize:96 %u-%u\r\n"
                 "a=fmtp:96 packetization-mode=1\r\na=control:trackID=0\r\n",
                 static_cast<unsigned>(fps), static_cast<unsigned>(width),
                 static_cast<unsigned>(height));
  }
  if (n <= 0)
    return {};
  const size_t used = static_cast<size_t>(n) < sizeof(buf) ? static_cast<size_t>(n) : sizeof(buf) - 1;
  return std::string(buf, used);
}

// RFC 6184 packetization-mode=1 directly into a caller-owned interleaved TCP
// scratch buffer (`$` + channel + 2-byte big-endian RTP length + RTP header +
// payload) with zero stack arrays and a single memcpy per packet.
// `emit(const uint8_t *framed, size_t framed_len, bool marker)` returns true
// to continue or false to abort packetization immediately.
template<typename Emit>
bool packetize_h264_annex_b_interleaved(const uint8_t *data, size_t len, uint32_t timestamp,
                                        uint16_t &sequence, size_t max_nal_payload, uint8_t channel,
                                        uint8_t *scratch, size_t scratch_cap, Emit emit) {
  if (data == nullptr || scratch == nullptr || max_nal_payload < 3 ||
      max_nal_payload > H264_RTP_MAX_PAYLOAD ||
      scratch_cap < H264_RTP_INTERLEAVED_HEADER + H264_RTP_HEADER + max_nal_payload) {
    return false;
  }

  size_t prefix = 0;
  size_t start = detail::annex_b_start_code(data, len, 0, prefix);
  while (start < len) {
    const size_t nal_start = start + prefix;
    size_t next_prefix = 0;
    const size_t next = detail::annex_b_start_code(data, len, nal_start, next_prefix);
    size_t nal_end = next;
    while (nal_end > nal_start && data[nal_end - 1] == 0)
      --nal_end;
    const size_t nal_len = nal_end - nal_start;
    const bool final_nal = next == len;

    if (nal_len != 0 && nal_len <= max_nal_payload) {
      const size_t rtp_len = H264_RTP_HEADER + nal_len;
      scratch[0] = '$';
      scratch[1] = channel;
      scratch[2] = static_cast<uint8_t>(rtp_len >> 8);
      scratch[3] = static_cast<uint8_t>(rtp_len);
      detail::rtp_header(scratch + H264_RTP_INTERLEAVED_HEADER, sequence++, timestamp, final_nal);
      std::memcpy(scratch + H264_RTP_INTERLEAVED_HEADER + H264_RTP_HEADER, data + nal_start,
                  nal_len);
      if (!emit(scratch, H264_RTP_INTERLEAVED_HEADER + rtp_len, final_nal))
        return false;
    } else if (nal_len > 1) {
      const uint8_t nal_header = data[nal_start];
      const size_t chunk_max = max_nal_payload - 2;
      size_t offset = 1;
      while (offset < nal_len) {
        const size_t remaining = nal_len - offset;
        const size_t chunk = remaining < chunk_max ? remaining : chunk_max;
        const bool first = offset == 1;
        const bool last = offset + chunk == nal_len;
        const bool marker = final_nal && last;
        const size_t rtp_len = H264_RTP_HEADER + 2 + chunk;
        scratch[0] = '$';
        scratch[1] = channel;
        scratch[2] = static_cast<uint8_t>(rtp_len >> 8);
        scratch[3] = static_cast<uint8_t>(rtp_len);
        detail::rtp_header(scratch + H264_RTP_INTERLEAVED_HEADER, sequence++, timestamp, marker);
        scratch[H264_RTP_INTERLEAVED_HEADER + 12] =
            static_cast<uint8_t>((nal_header & 0xE0) | 28);  // FU-A indicator
        scratch[H264_RTP_INTERLEAVED_HEADER + 13] =
            static_cast<uint8_t>((first ? 0x80 : 0) | (last ? 0x40 : 0) | (nal_header & 0x1F));
        std::memcpy(scratch + H264_RTP_INTERLEAVED_HEADER + 14, data + nal_start + offset, chunk);
        if (!emit(scratch, H264_RTP_INTERLEAVED_HEADER + rtp_len, marker))
          return false;
        offset += chunk;
      }
    }

    start = next;
    prefix = next_prefix;
  }
  return true;
}

// RFC 6184 packetization-mode=1 returning bare RTP packets (host/test helper).
template<typename Emit>
void packetize_h264_annex_b(const uint8_t *data, size_t len, uint32_t timestamp,
                            uint16_t &sequence, size_t max_nal_payload, Emit emit) {
  std::array<uint8_t, H264_RTP_INTERLEAVED_BUF_SIZE> scratch{};
  packetize_h264_annex_b_interleaved(
      data, len, timestamp, sequence, max_nal_payload, 0, scratch.data(), scratch.size(),
      [&](const uint8_t *framed, size_t framed_len, bool marker) {
        emit(framed + H264_RTP_INTERLEAVED_HEADER, framed_len - H264_RTP_INTERLEAVED_HEADER,
             marker);
        return true;
      });
}

}  // namespace esp_video_camera
}  // namespace esphome
