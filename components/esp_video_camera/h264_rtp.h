#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace esp_video_camera {

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

// RFC 6184 packetization-mode=1. Input is one Annex-B access unit. The callback
// receives complete RTP packets; only the final packet carries the marker bit.
template<typename Emit>
void packetize_h264_annex_b(const uint8_t *data, size_t len, uint32_t timestamp,
                            uint16_t &sequence, size_t max_nal_payload, Emit emit) {
  if (data == nullptr || max_nal_payload < 3 || max_nal_payload > 1400)
    return;

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
      std::array<uint8_t, 1412> packet{};
      detail::rtp_header(packet.data(), sequence++, timestamp, final_nal);
      for (size_t i = 0; i < nal_len; ++i)
        packet[12 + i] = data[nal_start + i];
      emit(packet.data(), 12 + nal_len, final_nal);
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
        std::array<uint8_t, 1412> packet{};
        detail::rtp_header(packet.data(), sequence++, timestamp, marker);
        packet[12] = static_cast<uint8_t>((nal_header & 0xE0) | 28);  // FU-A indicator
        packet[13] = static_cast<uint8_t>((first ? 0x80 : 0) | (last ? 0x40 : 0) |
                                          (nal_header & 0x1F));
        for (size_t i = 0; i < chunk; ++i)
          packet[14 + i] = data[nal_start + offset + i];
        emit(packet.data(), 14 + chunk, marker);
        offset += chunk;
      }
    }

    start = next;
    prefix = next_prefix;
  }
}

}  // namespace esp_video_camera
}  // namespace esphome
