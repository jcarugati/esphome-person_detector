#pragma once

// EspVideoCamera — a raw (typed-RGB) FrameSource for ESP32-P4 MIPI-CSI cameras.
//
// Drives the sensor over the P4 CSI + ISP via Espressif's esp_video (V4L2)
// stack, then uses the PPA to rotate (portrait mounting) and convert to RGB888,
// which it hands to person_detect directly — no JPEG encode/decode. This is the
// "raw" backend DESIGN.md §2/§7 anticipated behind the FrameSource seam.
//
// Sensor power/reset lines on the reTerminal D1001 sit on an XL9535 I2C GPIO
// expander, so those are ESPHome GPIOPins (from an `xl9535:` hub) that we drive
// before esp_video_init; the sensor's own SCCB/I2C is handled by esp_video.

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/defines.h"
#include "esphome/components/person_detect/frame_source.h"
#ifdef USE_I2C
#include "esphome/components/i2c/i2c_bus.h"
#endif

#include <atomic>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/ppa.h"
#ifdef USE_WEBSERVER
#include <esp_http_server.h>
#endif

#include "auto_exposure.h"
#include "gray_world.h"
#include "h264_rtp.h"
#include "mjpeg.h"

namespace esphome {
namespace esp_video_camera {

enum Rotation : uint16_t {
  ROTATION_0 = 0,
  ROTATION_90 = 90,
  ROTATION_180 = 180,
  ROTATION_270 = 270,
};

class EspVideoCamera : public Component, public person_detect::FrameSource {
 public:
  // Component: bring up the CSI/ISP pipeline before the detector's setup runs.
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // person_detect::FrameSource
  bool init() override;
  bool start() override;
  void stop() override;
  bool acquire(person_detect::FrameView &out, uint32_t timeout_ms) override;
  void release() override;
  void report_luma(uint8_t mean_luma, uint32_t capture_epoch) override;

  // Codegen setters
  void set_sccb_pins(int sda, int scl) {
    this->sccb_sda_ = sda;
    this->sccb_scl_ = scl;
  }
  void set_sccb_port(int port) { this->sccb_port_ = port; }
  void set_sccb_freq(uint32_t freq) { this->sccb_freq_ = freq; }
#ifdef USE_I2C
  // Share this ESPHome i2c bus for the SCCB instead of installing our own
  // master (display+touch boards where both P4 I2C controllers are taken).
  void set_sccb_bus(i2c::I2CBus *bus) { this->sccb_bus_ = bus; }
#endif
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_powerdown_pin(GPIOPin *pin) { this->powerdown_pin_ = pin; }
  void set_enable_pin(GPIOPin *pin) { this->enable_pin_ = pin; }
  void set_capture_size(uint16_t w, uint16_t h) {
    this->cap_w_ = w;
    this->cap_h_ = h;
  }
  void set_rotation(uint16_t deg) { this->rotation_ = deg; }
  // rotation: auto — pick the rotation once at boot from an accelerometer, unless
  // an explicit rotation was given. Low cost: a single read, no polling.
  void set_auto_rotation(bool a) { this->auto_rotation_ = a; }
#ifdef USE_I2C
  void set_imu(i2c::I2CBus *bus, uint8_t address) {
    this->imu_bus_ = bus;
    this->imu_addr_ = address;
  }
#endif
  void set_swap_rgb(bool swap) { this->swap_rgb_ = swap; }
  void set_frame_buffer_count(uint8_t n) { this->fb_count_ = n; }
  void set_exposure(int e) { this->exposure_ = e; }
  void set_gain(int g) { this->gain_ = g; }
  void set_snapshot(bool enabled) { this->snapshot_enabled_ = enabled; }
  void set_mjpeg_stream(bool enabled) { this->mjpeg_stream_enabled_ = enabled; }
  void set_auto_white_balance(bool enabled) { this->awb_enabled_ = enabled; }
  void set_auto_exposure(bool enabled) { this->auto_exposure_ = enabled; }
  void set_h264_stream(bool enabled) { this->h264_enabled_ = enabled; }
  void set_h264_auth(const std::string &auth) { this->h264_auth_ = auth; }

#ifdef USE_WEBSERVER
  // HTTP endpoint handlers invoked from ESP-IDF's web_server_idf after auth.
  // Snapshot responds synchronously under snapshot_buf_mutex_; MJPEG hands off
  // an async httpd_req_t copy to a dedicated worker task so port 80 stays free.
  void handle_snapshot_http(httpd_req_t *req);
  void handle_mjpeg_http(httpd_req_t *req);
#endif

 protected:
  bool power_on_sensor_();
  bool open_and_configure_();
  bool setup_ppa_();
  bool set_dqbuf_timeout_(int fd, uint32_t timeout_ms);
  bool requeue_all_buffers_();
  bool start_v4l2_stream_(uint32_t expected_epoch);
  bool stop_v4l2_stream_();
  void wake_all_waiters_();
  // One-time gravity read → pick rotation so people are upright in the current
  // device orientation. Falls back to 0 (landscape) if no/unknown IMU.
  void detect_rotation_from_imu_();
  // Drive the sensor's exposure/gain via V4L2 controls. The SC-family sensors
  // power up at their minimum exposure (a near-black frame) and only auto-expose
  // if esp_ipa has a per-sensor tuning config, so set a usable level explicitly.
  void apply_sensor_controls_();
  // Re-apply the chosen exposure/gain. A set right at STREAMON occasionally
  // doesn't take (and some ISP paths drift exposure), so acquire() calls this
  // periodically so a missed/drifted value self-corrects. Silent.
  bool reassert_controls_();
  // Consume any luma sample reported by person_detect (or retry unknown sensor
  // controls after an ioctl failure) on the V4L2-owning thread after the
  // current frame has been dequeued and stamped.
  void step_auto_exposure_(uint32_t current_epoch);

  // Optional H.264/RTSP and MJPEG paths. A dedicated camera task is the sole
  // V4L2 reader whenever a live stream is enabled, and lazily owns the direct
  // esp_h264_enc_hw instance only while an RTSP client is actively in PLAY.
  bool open_h264_();
  void teardown_h264_();
  void start_rtsp_listener_();
  bool convert_frame_(const void *input, void *output, size_t output_size,
                      uint16_t width, uint16_t height, ppa_srm_color_mode_t mode,
                      float scale_x, float scale_y, uint16_t block_w = 0,
                      uint16_t block_h = 0, uint16_t offset_x = 0,
                      uint16_t offset_y = 0);
  void capture_task_();
  static void capture_task_entry_(void *arg);
  void rtsp_task_();
  static void rtsp_task_entry_(void *arg);
  bool encode_h264_(uint32_t capture_epoch, uint32_t session_gen, int64_t frame_us);
  bool send_rtsp_(int fd, const char *data, size_t len);
  bool send_h264_rtp_(const uint8_t *data, size_t len, uint32_t rtp_timestamp,
                      uint32_t capture_epoch, uint32_t session_gen);

  bool ensure_jpeg_encoder_();
  bool ensure_stream_rgb_();
  uint32_t claim_snapshot_request_();
  void encode_snapshot_(uint32_t claimed_gen, uint32_t expected_epoch);
  void fail_snapshot_(uint32_t claimed_gen);
  uint32_t claim_mjpeg_request_();
  void encode_mjpeg_(uint32_t claimed_gen, uint32_t expected_epoch);
  void fail_mjpeg_(uint32_t claimed_gen);

#ifdef USE_WEBSERVER
  struct MjpegQueuedRequest {
    httpd_req_t *req;
    uint32_t session_epoch;
  };
  void mjpeg_worker_task_();
  static void mjpeg_worker_entry_(void *arg);
  void serve_mjpeg_client_(httpd_req_t *req, uint32_t session_epoch);
#endif

  // Config
  int sccb_sda_{-1};
  int sccb_scl_{-1};
  int sccb_port_{1};  // dedicated I2C controller for the sensor SCCB (must not
                      // collide with the ESPHome i2c bus that owns the expander)
  uint32_t sccb_freq_{100000};
#ifdef USE_I2C
  i2c::I2CBus *sccb_bus_{nullptr};  // when set: share this bus, don't install one
#endif
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *powerdown_pin_{nullptr};
  GPIOPin *enable_pin_{nullptr};
  uint16_t cap_w_{1280};
  uint16_t cap_h_{720};
  uint16_t rotation_{0};
  bool auto_rotation_{false};
#ifdef USE_I2C
  i2c::I2CBus *imu_bus_{nullptr};
  uint8_t imu_addr_{0x6A};
#endif
  bool swap_rgb_{false};
  uint8_t fb_count_{2};
  int exposure_{-1};  // raw sensor exposure; -1 = auto-pick a bright default
  int gain_{-1};      // raw sensor gain;     -1 = auto-pick a moderate default
  bool auto_exposure_{false};
  bool h264_enabled_{false};
  std::string h264_auth_;

  // The exposure/gain we actually applied, re-asserted periodically by acquire().
  uint32_t applied_exp_cid_{0};
  int applied_exp_val_{-1};
  uint32_t applied_gain_cid_{0};
  int applied_gain_val_{-1};
  int64_t last_ctrl_us_{0};
  AutoExposureController ae_{};
  std::atomic<uint64_t> pending_ae_feedback_{0};
  // Written ONLY alongside rotated_ inside the frame handoff (before deliver()
  // in task mode, or under v4l2_mutex_ in non-task mode).
  float rotated_exposure_scale_{1.0f};
  uint32_t rotated_capture_epoch_{0};

  // Runtime
  bool ready_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<uint32_t> stop_epoch_{0};
  bool v4l2_active_{false};            // guarded by v4l2_mutex_ in non-task mode
  bool non_task_stop_pending_{false};  // guarded by v4l2_mutex_ in non-task mode
  uint32_t streamoff_failures_{0};
  uint32_t queued_mask_{0};      // bitmask of MMAP buffers currently queued via QBUF
  SemaphoreHandle_t v4l2_mutex_{nullptr};
  SemaphoreHandle_t sock_mutex_{nullptr};  // guards active HTTP/RTSP sockfds for shutdown()
  int fd_{-1};

  struct MappedBuffer {
    void *start{nullptr};
    size_t length{0};
  };
  std::vector<MappedBuffer> buffers_;

  ppa_client_handle_t ppa_{nullptr};
  void *rotated_{nullptr};   // PPA output: full-res upright RGB888 (PSRAM, cache-aligned)
  size_t rotated_size_{0};
  uint16_t out_w_{0};        // dims after rotation
  uint16_t out_h_{0};

  // Dedicated half-resolution RGB888 buffer for /stream.mjpg (lazy PSRAM alloc),
  // decoupled from person_detect's rotated_ buffer.
  uint8_t *stream_rgb_{nullptr};
  size_t stream_rgb_size_{0};
  uint16_t stream_w_{0};
  uint16_t stream_h_{0};

  uint32_t capture_failures_{0};
  uint32_t last_ppa_us_{0};

  // Camera task + lazy direct H.264/RTSP state.
  bool task_mode_{false};
  std::atomic<uint32_t> task_frames_{0};  // camera task owns V4L2 (any live stream enabled)
  std::atomic<bool> h264_ready_{false};   // RTSP listener bound & task started
  bool rtsp_listener_attempted_{false};   // main-loop one-shot guard (after lwIP init)
  int rtsp_server_fd_{-1};
  H264Geometry h264_geom_{};
  void *h264_enc_{nullptr};               // esp_h264_enc_handle_t; owned by capture_task_
  uint8_t *h264_yuv_{nullptr};            // lazy PSRAM O_UYY_E_VYY input buffer
  size_t h264_yuv_size_{0};
  uint8_t *h264_output_{nullptr};         // lazy PSRAM Annex-B output buffer
  size_t h264_output_size_{0};
  uint8_t *rtp_packet_buf_{nullptr};      // lazy PSRAM interleaved RTP scratch (zero stack)
  YuvChromaAwbState h264_awb_state_{};
  H264SpropParams h264_sprop_{};          // cached SPS/PPS for SDP; guarded by sock_mutex_
  TaskHandle_t capture_task_handle_{nullptr};
  TaskHandle_t rtsp_task_handle_{nullptr};
  SemaphoreHandle_t frame_done_{nullptr};
  SemaphoreHandle_t rtsp_send_mutex_{nullptr};
  bool rtsp_partial_active_{false};       // guarded by rtsp_send_mutex_
  uint32_t rtsp_partial_session_{0};      // guarded by rtsp_send_mutex_
  std::atomic<bool> stream_requested_{false};
  FrameHandoff frame_handoff_{};
  std::atomic<bool> rtsp_playing_{false};
  std::atomic<int> rtsp_client_fd_{-1};
  std::atomic<uint32_t> rtsp_session_gen_{0};
  std::atomic<int64_t> rtsp_last_media_us_{0};
  uint16_t rtp_sequence_{0};
  int64_t rtsp_play_start_us_{0};
  int64_t last_h264_us_{0};
  uint32_t h264_encode_failures_{0};

  // Gray-world white balance: out-of-place scratch copy for full-res snapshots,
  // EMA state for in-place balancing on stream_rgb_.
  bool awb_enabled_{false};
  uint8_t *balanced_{nullptr};
  GrayWorldState mjpeg_awb_state_{};
  std::atomic<bool> mjpeg_awb_reset_{true};

  // Shared HW JPEG encoder + independent mutex-protected snapshot and MJPEG buffers.
  void *jpeg_encoder_{nullptr};
  bool snapshot_enabled_{false};
  bool mjpeg_stream_enabled_{false};

  std::atomic<bool> snapshot_requested_{false};
  SemaphoreHandle_t snapshot_done_{nullptr};
  SemaphoreHandle_t snapshot_buf_mutex_{nullptr};
  JpegRequestState snapshot_state_{};
  uint8_t *snapshot_jpeg_buf_{nullptr};
  size_t snapshot_jpeg_buf_size_{0};

  std::atomic<bool> mjpeg_requested_{false};
  SemaphoreHandle_t mjpeg_done_{nullptr};
  SemaphoreHandle_t mjpeg_buf_mutex_{nullptr};
  JpegRequestState mjpeg_state_{};
  uint8_t *mjpeg_jpeg_raw_{nullptr};   // raw jpeg_alloc_encoder_mem pointer
  uint8_t *mjpeg_jpeg_buf_{nullptr};   // raw + MJPEG_PART_HEADROOM (128B aligned)
  size_t mjpeg_jpeg_buf_size_{0};

#ifdef USE_WEBSERVER
  QueueHandle_t mjpeg_req_queue_{nullptr};
  TaskHandle_t mjpeg_worker_handle_{nullptr};
  std::atomic<bool> mjpeg_client_active_{false};
  int mjpeg_active_sockfd_{-1};
  int snapshot_active_sockfd_{-1};
#endif
};

}  // namespace esp_video_camera
}  // namespace esphome
