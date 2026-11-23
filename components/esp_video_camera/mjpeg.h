#pragma once

#include <atomic>
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
// Generation tokens ensure stale completions, timeout/re-request races, or
// privacy-off wakeups never return a previous or half-written JPEG buffer.
struct JpegRequestState {
  uint32_t req_gen{0};
  uint32_t done_gen{0};
  size_t jpeg_len{0};
  bool pending{false};

  uint32_t begin_request() {
    ++this->req_gen;
    if (this->req_gen == 0)
      ++this->req_gen;  // 0 is reserved for "no request claimed"
    this->pending = true;
    return this->req_gen;
  }

  uint32_t claim_request() const { return this->pending ? this->req_gen : 0; }

  bool complete_request(uint32_t claimed_gen, size_t len) {
    if (claimed_gen == 0 || !this->pending || this->req_gen != claimed_gen)
      return false;
    this->jpeg_len = len;
    this->done_gen = claimed_gen;
    this->pending = false;
    return true;
  }

  bool finish_wait(uint32_t token, bool waited_ok, bool camera_streaming, size_t &out_len) {
    if (this->req_gen == token)
      this->pending = false;
    if (!waited_ok || !camera_streaming || this->done_gen != token || this->jpeg_len == 0) {
      out_len = 0;
      return false;
    }
    out_len = this->jpeg_len;
    return true;
  }
};

// Action decided by capture_task_() on each loop iteration from the stop epoch,
// stream_requested flag, and current V4L2 camera_on state.
enum class StreamLifecycleAction : uint8_t {
  IDLE,
  START,
  STOP,
  RUN,
};

// Latches every unobserved stop_epoch increment so a rapid privacy OFF -> ON
// toggle before capture_task_() observes OFF still forces a full V4L2 STOP
// before restarting and invalidates any active pre-stop HTTP viewer session.
struct StopEpochLatch {
  uint32_t handled_epoch{0};

  StreamLifecycleAction step(uint32_t observed_epoch, bool stream_requested, bool camera_on) {
    if (observed_epoch != this->handled_epoch) {
      if (camera_on)
        return StreamLifecycleAction::STOP;
      this->handled_epoch = observed_epoch;
    }
    if (!stream_requested)
      return camera_on ? StreamLifecycleAction::STOP : StreamLifecycleAction::IDLE;
    if (!camera_on)
      return StreamLifecycleAction::START;
    return StreamLifecycleAction::RUN;
  }

  void mark_stopped(uint32_t stopped_epoch) { this->handled_epoch = stopped_epoch; }

  static bool session_valid(uint32_t session_epoch, uint32_t current_epoch, bool streaming) {
    return streaming && current_epoch == session_epoch;
  }
};

// Lock-free 3-state handoff between one detector consumer and camera_stream so
// timeout or privacy-off at any interleaving can never strand IN_USE.
class FrameHandoff {
 public:
  enum State : uint8_t {
    IDLE = 0,
    REQUESTED = 1,
    IN_USE = 2,
  };

  State state() const { return this->state_.load(); }
  bool is_requested() const { return this->state_.load() == REQUESTED; }
  bool is_in_use() const { return this->state_.load() == IN_USE; }

  bool request() {
    State expected = IDLE;
    return this->state_.compare_exchange_strong(expected, REQUESTED);
  }

  bool deliver() {
    State expected = REQUESTED;
    return this->state_.compare_exchange_strong(expected, IN_USE);
  }

  // Consumer abort (timeout or privacy-off): cancel REQUESTED -> IDLE, or if
  // producer already transitioned REQUESTED -> IN_USE, immediately release
  // IN_USE -> IDLE.
  void cancel_or_release() {
    State expected = REQUESTED;
    if (this->state_.compare_exchange_strong(expected, IDLE))
      return;
    expected = IN_USE;
    this->state_.compare_exchange_strong(expected, IDLE);
  }

  bool accept(bool stream_valid) {
    if (stream_valid && this->state_.load() == IN_USE)
      return true;
    this->cancel_or_release();
    return false;
  }

  void release() {
    State expected = IN_USE;
    this->state_.compare_exchange_strong(expected, IDLE);
  }

  // Stop helper: cancel an unclaimed REQUESTED -> IDLE without touching IN_USE
  // (which belongs to an active detector inference pass until release()).
  void cancel_request() {
    State expected = REQUESTED;
    this->state_.compare_exchange_strong(expected, IDLE);
  }

 private:
  std::atomic<State> state_{IDLE};
};

}  // namespace esp_video_camera
}  // namespace esphome
