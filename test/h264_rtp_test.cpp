// g++ -std=c++17 -I components test/h264_rtp_test.cpp -o /tmp/h264_rtp && /tmp/h264_rtp
#include <cassert>
#include <cstdint>
#include <vector>

#include "esp_video_camera/h264_rtp.h"

using esphome::esp_video_camera::packetize_h264_annex_b;

struct Packet {
  std::vector<uint8_t> bytes;
  bool marker;
};

int main() {
  const uint8_t access_unit[] = {
      0, 0, 0, 1, 0x67, 0x11, 0x22,
      0, 0, 1, 0x68, 0x33,
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
  assert(packets[0].bytes[12] == 0x67);  // SPS stays a single NAL packet
  assert((packets[2].bytes[12] & 0x1F) == 28);  // IDR fragmented as FU-A
  assert((packets[2].bytes[13] & 0x80) != 0);   // first fragment
  assert((packets[4].bytes[13] & 0x40) != 0);   // last fragment
  for (size_t i = 0; i + 1 < packets.size(); ++i) assert(!packets[i].marker);
  assert(packets.back().marker);
  assert((packets.back().bytes[1] & 0x80) != 0);
  return 0;
}
