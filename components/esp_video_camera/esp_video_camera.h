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
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/ppa.h"

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
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // person_detect::FrameSource
  bool init() override;
  bool start() override;
  void stop() override;
  bool acquire(person_detect::FrameView &out, uint32_t timeout_ms) override;
  void release() override;

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
  void set_h264_stream(bool enabled) { this->h264_enabled_ = enabled; }
  void set_h264_auth(const std::string &auth) { this->h264_auth_ = auth; }

  // Blocking, for the HTTP task: waits for the detector's next acquire() to
  // JPEG-encode its frame. The buffer stays valid until the next request.
  // False when the camera is idle (privacy switch off) or on timeout.
  bool capture_jpeg(const uint8_t *&data, size_t &len, uint32_t timeout_ms);

 protected:
  bool power_on_sensor_();
  bool open_and_configure_();
  bool setup_ppa_();
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
  void reassert_controls_();

  // Optional H.264/RTSP path. A low-priority camera task is the sole V4L2
  // reader; detector requests are serviced before stream frames.
  bool setup_h264_();
  bool convert_frame_(const void *input, void *output, size_t output_size,
                      uint16_t width, uint16_t height, ppa_srm_color_mode_t mode,
                      float scale_x, float scale_y);
  void capture_task_();
  static void capture_task_entry_(void *arg);
  void rtsp_task_();
  static void rtsp_task_entry_(void *arg);
  bool encode_h264_();
  bool send_rtsp_(int fd, const char *data, size_t len);
  void send_h264_rtp_(const uint8_t *data, size_t len);

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
  bool h264_enabled_{false};
  std::string h264_auth_;

  // The exposure/gain we actually applied, re-asserted periodically by acquire().
  uint32_t applied_exp_cid_{0};
  int applied_exp_val_{-1};
  uint32_t applied_gain_cid_{0};
  int applied_gain_val_{-1};
  int64_t last_ctrl_us_{0};

  // Runtime
  bool ready_{false};
  std::atomic<bool> streaming_{false};
  int fd_{-1};

  struct MappedBuffer {
    void *start{nullptr};
    size_t length{0};
  };
  std::vector<MappedBuffer> buffers_;
  int dq_index_{-1};  // index of the buffer currently held by acquire()

  ppa_client_handle_t ppa_{nullptr};
  void *rotated_{nullptr};   // PPA output: upright RGB888 (PSRAM, cache-aligned)
  size_t rotated_size_{0};
  uint16_t out_w_{0};        // dims after rotation
  uint16_t out_h_{0};

  uint32_t capture_failures_{0};
  uint32_t last_ppa_us_{0};

  // H.264 encoder and single-client RTSP server (all dormant unless opted in).
  bool h264_ready_{false};
  int h264_fd_{-1};
  uint8_t *h264_yuv_{nullptr};
  size_t h264_yuv_size_{0};
  uint8_t *h264_output_{nullptr};
  size_t h264_output_size_{0};
  uint16_t h264_w_{0};
  uint16_t h264_h_{0};
  TaskHandle_t capture_task_handle_{nullptr};
  TaskHandle_t rtsp_task_handle_{nullptr};
  SemaphoreHandle_t frame_done_{nullptr};
  SemaphoreHandle_t rtsp_send_mutex_{nullptr};
  std::atomic<bool> stream_requested_{false};
  std::atomic<bool> frame_requested_{false};
  std::atomic<bool> frame_in_use_{false};
  std::atomic<bool> rtsp_playing_{false};
  std::atomic<int> rtsp_client_fd_{-1};
  uint16_t rtp_sequence_{0};
  uint32_t rtp_timestamp_{0};
  int64_t last_h264_us_{0};

  // Gray-world white balance applied to a JPEG-only copy.
  bool awb_enabled_{false};
  uint8_t *balanced_{nullptr};

  // JPEG: the frame owner (detector task) encodes on request, so the sensor
  // keeps a single reader. ponytail: one waiter; httpd is single-task.
  void encode_snapshot_();
  bool snapshot_enabled_{false};
  bool mjpeg_stream_enabled_{false};
  std::atomic<bool> snapshot_requested_{false};
  SemaphoreHandle_t snapshot_done_{nullptr};
  void *jpeg_encoder_{nullptr};
  uint8_t *jpeg_buf_{nullptr};
  size_t jpeg_buf_size_{0};
  size_t jpeg_len_{0};
};

}  // namespace esp_video_camera
}  // namespace esphome
