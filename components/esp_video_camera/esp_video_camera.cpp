#include "esp_video_camera.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <array>
#include <cctype>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "lwip/inet.h"
#include "linux/videodev2.h"

// ESP-IDF's <sys/mman.h> compat shim doesn't always define MAP_FAILED.
#ifndef MAP_FAILED
#define MAP_FAILED (reinterpret_cast<void *>(-1))
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"

#include "driver/i2c_master.h"  // i2c_master_get_bus_handle for shared-bus SCCB
#include "driver/jpeg_encode.h"
#include "gray_world.h"
#include "h264_rtp.h"
#include "mjpeg.h"

#ifdef USE_WEBSERVER
#include "esphome/components/web_server_base/web_server_base.h"
#endif

namespace esphome {
namespace esp_video_camera {

static const char *const TAG = "esp_video_camera";
static const char *const VIDEO_DEVICE = "/dev/video0";
static constexpr uint32_t DQBUF_POLL_TIMEOUT_MS = 100;
static constexpr uint32_t HTTP_CAPTURE_TIMEOUT_MS = 4000;

#ifdef USE_WEBSERVER
static void send_http_503(httpd_req_t *req, const char *msg) {
  httpd_resp_set_status(req, "503 Service Unavailable");
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
}

// GET /snapshot.jpg, behind the web server's own auth (add_handler wraps it).
class SnapshotHandler : public AsyncWebHandler {
 public:
  explicit SnapshotHandler(EspVideoCamera *cam) : cam_(cam) {}
  bool canHandle(AsyncWebServerRequest *request) const override {
    char buf[AsyncWebServerRequest::URL_BUF_SIZE];
    return request->method() == HTTP_GET && request->url_to(buf) == "/snapshot.jpg";
  }
  void handleRequest(AsyncWebServerRequest *request) override {
    this->cam_->handle_snapshot_http(static_cast<httpd_req_t *>(*request));
  }

 protected:
  EspVideoCamera *cam_;
};

// GET /stream.mjpg, behind the same auth wrapper as /snapshot.jpg. Hands off
// an async httpd_req_t copy to a dedicated worker task so ESP-IDF's single
// httpd task stays free for all other port-80 traffic.
class MjpegHandler : public AsyncWebHandler {
 public:
  explicit MjpegHandler(EspVideoCamera *cam) : cam_(cam) {}
  bool canHandle(AsyncWebServerRequest *request) const override {
    char buf[AsyncWebServerRequest::URL_BUF_SIZE];
    return request->method() == HTTP_GET && request->url_to(buf) == "/stream.mjpg";
  }
  void handleRequest(AsyncWebServerRequest *request) override {
    this->cam_->handle_mjpeg_http(static_cast<httpd_req_t *>(*request));
  }

 protected:
  EspVideoCamera *cam_;
};
#endif

static int xioctl(int fd, unsigned long req, void *arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

// Set a single V4L2 control through the *extended* API — esp_video wires up
// sensor controls there, not through the legacy VIDIOC_S_CTRL.
static bool set_ext_ctrl(int fd, uint32_t cid, int value) {
  struct v4l2_ext_control c = {};
  c.id = cid;
  c.value = value;
  struct v4l2_ext_controls cs = {};
  cs.which = V4L2_CTRL_WHICH_CUR_VAL;  // set current value regardless of class
  cs.count = 1;
  cs.controls = &c;
  return xioctl(fd, VIDIOC_S_EXT_CTRLS, &cs) == 0;
}

static std::string rtsp_header(const std::string &request, const char *name) {
  const size_t wanted = strlen(name);
  size_t line = request.find("\r\n") + 2;
  while (line >= 2 && line < request.size()) {
    size_t end = request.find("\r\n", line);
    if (end == std::string::npos || end == line)
      break;
    size_t colon = request.find(':', line);
    if (colon < end && colon - line == wanted) {
      bool match = true;
      for (size_t i = 0; i < wanted; ++i) {
        if (std::tolower(static_cast<unsigned char>(request[line + i])) !=
            std::tolower(static_cast<unsigned char>(name[i]))) {
          match = false;
          break;
        }
      }
      if (match) {
        size_t value = colon + 1;
        while (value < end && (request[value] == ' ' || request[value] == '\t'))
          ++value;
        return request.substr(value, end - value);
      }
    }
    line = end + 2;
  }
  return {};
}

void EspVideoCamera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up esp_video_camera (MIPI-CSI)...");

  if (!this->power_on_sensor_()) {
    this->mark_failed();
    return;
  }

  // Initialize the CSI + ISP pipeline and the sensor's SCCB (I2C). Power/reset
  // are handled by us via the expander, so tell esp_video there are none.
  esp_video_init_csi_config_t csi = {};
  csi.sccb_config.freq = this->sccb_freq_;
  csi.reset_pin = static_cast<gpio_num_t>(-1);   // handled via expander
  csi.pwdn_pin = static_cast<gpio_num_t>(-1);

#ifdef USE_I2C
  if (this->sccb_bus_ != nullptr) {
    // Share an existing ESPHome i2c bus: don't install a master, hand esp_video
    // the underlying i2c_master_bus_handle_t so the sensor joins that bus as
    // another device (alongside e.g. a touch controller on the same wires). The
    // new i2c-master API is per-device speed, so our sccb freq is independent of
    // the other devices' speeds. Retrieve the handle from the bus's port.
    int port = static_cast<i2c::InternalI2CBus *>(this->sccb_bus_)->get_port();
    i2c_master_bus_handle_t handle = nullptr;
    esp_err_t herr = i2c_master_get_bus_handle(static_cast<i2c_port_num_t>(port), &handle);
    if (herr != ESP_OK || handle == nullptr) {
      ESP_LOGE(TAG, "Could not get i2c bus handle for shared SCCB (port %d): %s",
               port, esp_err_to_name(herr));
      this->mark_failed();
      return;
    }
    csi.sccb_config.init_sccb = false;
    csi.sccb_config.i2c_handle = handle;
    ESP_LOGCONFIG(TAG, "esp_video_init: SCCB shares ESPHome i2c bus (port %d) @ %uHz",
                  port, (unsigned) this->sccb_freq_);
  } else
#endif
  {
    // Standalone: install our own SCCB master on sccb_port_. It must not be the
    // port an ESPHome `i2c:` bus already owns, or the install collides and the
    // sensor never probes (no /dev/video0).
    csi.sccb_config.init_sccb = true;
    csi.sccb_config.i2c_config.port = this->sccb_port_;
    csi.sccb_config.i2c_config.scl_pin = static_cast<gpio_num_t>(this->sccb_scl_);
    csi.sccb_config.i2c_config.sda_pin = static_cast<gpio_num_t>(this->sccb_sda_);
    ESP_LOGCONFIG(TAG, "esp_video_init: SCCB on I2C%d (SDA=%d SCL=%d @ %uHz)",
                  this->sccb_port_, this->sccb_sda_, this->sccb_scl_,
                  (unsigned) this->sccb_freq_);
  }

  esp_video_init_config_t cfg = {};
  cfg.csi = &csi;
  esp_err_t err = esp_video_init(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s — check SCCB wiring/port and that "
                  "the sensor is powered (enable/reset lines)",
             esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  if (!this->open_and_configure_()) {
    this->mark_failed();
    return;
  }
  // Pick the rotation from gravity before sizing the PPA output (rotation:auto).
  if (this->auto_rotation_)
    this->detect_rotation_from_imu_();
  if (!this->setup_ppa_()) {
    this->mark_failed();
    return;
  }

  this->v4l2_mutex_ = xSemaphoreCreateMutex();
  this->sock_mutex_ = xSemaphoreCreateMutex();
  if (this->v4l2_mutex_ == nullptr || this->sock_mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create camera synchronization mutexes");
    this->mark_failed();
    return;
  }

  this->ready_ = true;
#ifdef USE_WEBSERVER
  if (this->snapshot_enabled_) {
    this->snapshot_done_ = xSemaphoreCreateBinary();
    this->snapshot_buf_mutex_ = xSemaphoreCreateMutex();
    if (this->snapshot_done_ != nullptr && this->snapshot_buf_mutex_ != nullptr) {
      web_server_base::global_web_server_base->add_handler(new SnapshotHandler(this));  // NOLINT
    } else {
      ESP_LOGW(TAG, "Snapshot primitives unavailable; /snapshot.jpg disabled");
    }
  }
  if (this->mjpeg_stream_enabled_) {
    this->mjpeg_done_ = xSemaphoreCreateBinary();
    this->mjpeg_buf_mutex_ = xSemaphoreCreateMutex();
    this->mjpeg_req_queue_ = xQueueCreate(1, sizeof(MjpegQueuedRequest));
    if (this->mjpeg_done_ != nullptr && this->mjpeg_buf_mutex_ != nullptr &&
        this->mjpeg_req_queue_ != nullptr) {
      web_server_base::global_web_server_base->add_handler(new MjpegHandler(this));  // NOLINT
    } else {
      ESP_LOGW(TAG, "MJPEG primitives unavailable; /stream.mjpg disabled");
    }
  }
#endif
  // A dedicated camera task owns V4L2 whenever a live stream is enabled, so the
  // stream gets fresh frames instead of the detector's interval. Priority 3 sits
  // above person_detect (2) on Core 1 so streaming is not stalled by inference,
  // while remaining below Wi-Fi/LVGL/system tasks.
  if (this->h264_enabled_ || this->mjpeg_stream_enabled_) {
    this->frame_done_ = xSemaphoreCreateBinary();
    // Set before the task starts: it may run immediately on the other core and
    // exits if it sees task_mode_ false.
    this->task_mode_ = this->frame_done_ != nullptr;
    if (this->task_mode_)
      this->task_mode_ = xTaskCreatePinnedToCore(&EspVideoCamera::capture_task_entry_, "camera_stream",
                                                 8192, this, 3, &this->capture_task_handle_, 1) == pdPASS;
    if (!this->task_mode_)
      ESP_LOGW(TAG, "camera task unavailable; streams follow the detector interval");
  }
  // H.264 is brought up later by the camera task (see start_h264_).
  ESP_LOGCONFIG(TAG, "esp_video_camera ready: capture %ux%u -> rotate %u -> "
                     "RGB888 %ux%u (stream %ux%u)",
                this->cap_w_, this->cap_h_, this->rotation_, this->out_w_,
                this->out_h_, this->stream_w_, this->stream_h_);
}

void EspVideoCamera::detect_rotation_from_imu_() {
#ifdef USE_I2C
  if (this->imu_bus_ == nullptr) {
    ESP_LOGW(TAG, "rotation: auto but no imu configured — using 0 (landscape)");
    this->rotation_ = 0;
    return;
  }
  // Minimal LSM6DS-family accelerometer read (e.g. the D1001's LSM6DS3TR @ 0x6A).
  auto read_regs = [this](uint8_t reg, uint8_t *buf, size_t n) -> bool {
    return this->imu_bus_->write(this->imu_addr_, &reg, 1, false) == i2c::ERROR_OK &&
           this->imu_bus_->read(this->imu_addr_, buf, n) == i2c::ERROR_OK;
  };
  auto write_reg = [this](uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    this->imu_bus_->write(this->imu_addr_, b, 2, true);
  };

  uint8_t who = 0;
  if (!read_regs(0x0F, &who, 1) || who != 0x6A) {  // WHO_AM_I
    ESP_LOGW(TAG, "IMU WHO_AM_I=0x%02X (expected 0x6A) — using rotation 0", who);
    this->rotation_ = 0;
    return;
  }
  write_reg(0x10, 0x40);  // CTRL1_XL: 104 Hz, ±2g
  delay(30);
  uint8_t d[6] = {};
  if (!read_regs(0x28, d, 6)) {  // OUTX_L_A..OUTZ_H_A
    ESP_LOGW(TAG, "IMU accel read failed — using rotation 0");
    this->rotation_ = 0;
    return;
  }
  int16_t ax = static_cast<int16_t>(d[0] | (d[1] << 8));
  int16_t ay = static_cast<int16_t>(d[2] | (d[3] << 8));
  int16_t az = static_cast<int16_t>(d[4] | (d[5] << 8));
  int abs_x = ax < 0 ? -ax : ax;
  int abs_y = ay < 0 ? -ay : ay;

  // Choose the rotation that puts world-up at the top of the frame. Mapping
  // calibrated on the D1001 (raw values logged so other boards can recheck):
  //   gravity -Y -> 0 (landscape), +X -> 90 (portrait), +Y -> 180, -X -> 270.
  uint16_t rot;
  if (abs_x >= abs_y)
    rot = ax > 0 ? 90 : 270;
  else
    rot = ay > 0 ? 180 : 0;
  ESP_LOGCONFIG(TAG, "IMU accel ax=%d ay=%d az=%d -> auto rotation %u deg", ax, ay,
                az, rot);
  this->rotation_ = rot;
#else
  this->rotation_ = 0;
#endif
}

bool EspVideoCamera::power_on_sensor_() {
  // Drive the expander control lines. Polarity is expressed in YAML via the
  // pin's `inverted:` flag, so here we just assert the logical state:
  //   enable = on, power-down = off, then pulse reset.
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(true);
  }
  if (this->powerdown_pin_ != nullptr) {
    this->powerdown_pin_->setup();
    this->powerdown_pin_->digital_write(false);
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  return true;
}

bool EspVideoCamera::set_dqbuf_timeout_(int fd, uint32_t timeout_ms) {
  struct timeval tv = {};
  tv.tv_sec = static_cast<time_t>(timeout_ms / 1000u);
  tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000u) * 1000u);
  return xioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &tv) == 0;
}

bool EspVideoCamera::requeue_all_buffers_() {
  for (uint32_t i = 0; i < this->buffers_.size(); i++) {
    if ((this->queued_mask_ & (1u << i)) != 0)
      continue;
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF[%u] failed: %s", (unsigned) i, strerror(errno));
      return false;
    }
    this->queued_mask_ |= (1u << i);
  }
  return true;
}

bool EspVideoCamera::start_v4l2_stream_(uint32_t expected_epoch) {
  if (this->task_mode_ &&
      (!this->stream_requested_.load() || this->stop_epoch_.load() != expected_epoch)) {
    return false;
  }
  // esp_video 2.2.0 empties queued_list/done_list and resets all elements on
  // VIDIOC_STREAMOFF, so re-queue any unqueued buffers before every STREAMON.
  if (!this->requeue_all_buffers_())
    return false;
  this->set_dqbuf_timeout_(this->fd_, DQBUF_POLL_TIMEOUT_MS);
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(this->fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "VIDIOC_STREAMON failed: %s", strerror(errno));
    return false;
  }
  if (!this->task_mode_) {
    this->streaming_.store(true);
    ESP_LOGD(TAG, "Streaming started");
    // Controls take effect once the sensor is streaming.
    this->apply_sensor_controls_();
  } else if (this->stream_requested_.load() && this->stop_epoch_.load() == expected_epoch) {
    this->streaming_.store(true);
    if (!this->stream_requested_.load() || this->stop_epoch_.load() != expected_epoch) {
      // Privacy stop raced with STREAMON; caller sets camera_on=true and loops to STREAMOFF.
      this->streaming_.store(false);
    } else {
      ESP_LOGD(TAG, "Streaming started");
      this->apply_sensor_controls_();
    }
  }
  return true;
}

bool EspVideoCamera::stop_v4l2_stream_() {
  this->streaming_.store(false);
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const int rc = xioctl(this->fd_, VIDIOC_STREAMOFF, &type);
  if (rc != 0) {
    this->streamoff_failures_++;
    if (this->streamoff_failures_ == 1 || this->streamoff_failures_ % 20 == 0) {
      ESP_LOGW(TAG, "VIDIOC_STREAMOFF failed (%u consecutive): %s",
               (unsigned) this->streamoff_failures_, strerror(errno));
    }
    return false;
  }
  this->streamoff_failures_ = 0;
  this->queued_mask_ = 0;
  ESP_LOGD(TAG, "Streaming stopped");
  return true;
}

void EspVideoCamera::wake_all_waiters_() {
  this->frame_handoff_.cancel_request();
  this->snapshot_requested_.store(false);
  this->mjpeg_requested_.store(false);
  if (this->frame_done_ != nullptr)
    xSemaphoreGive(this->frame_done_);
  if (this->snapshot_done_ != nullptr)
    xSemaphoreGive(this->snapshot_done_);
  if (this->mjpeg_done_ != nullptr)
    xSemaphoreGive(this->mjpeg_done_);
}

bool EspVideoCamera::open_and_configure_() {
  // esp_video 2.2.0 ignores O_NONBLOCK and controls DQBUF blocking via
  // VIDIOC_S_DQBUF_TIMEOUT (defaulting to portMAX_DELAY on open).
  this->fd_ = open(VIDEO_DEVICE, O_RDWR);
  if (this->fd_ < 0) {
    ESP_LOGE(TAG, "open(%s) failed: %s", VIDEO_DEVICE, strerror(errno));
    ESP_LOGE(TAG, "  esp_video_init returned OK but registered no capture device "
                  "— the CSI sensor was not detected over SCCB. Verify the "
                  "SCCB port/pins, that the sensor Kconfig matches the module, "
                  "and that enable/power-down/reset put it in a running state.");
    return false;
  }

  // Ask the ISP to output RGB565 at the capture resolution (2 bytes/px keeps
  // the capture buffers small; the PPA converts to RGB888 while rotating).
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = this->cap_w_;
  fmt.fmt.pix.height = this->cap_h_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(this->fd_, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_S_FMT failed: %s", strerror(errno));
    return false;
  }
  // The driver may adjust the geometry/format; honor what it gave us and flag a
  // format it couldn't satisfy (the PPA pass below assumes RGB565 input).
  this->cap_w_ = fmt.fmt.pix.width;
  this->cap_h_ = fmt.fmt.pix.height;
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
    ESP_LOGW(TAG, "Requested RGB565 but driver set pixelformat 0x%08" PRIx32
                  " (%ux%u) — capture may not match PPA input",
             (uint32_t) fmt.fmt.pix.pixelformat, this->cap_w_, this->cap_h_);
  }

  struct v4l2_requestbuffers req = {};
  req.count = this->fb_count_;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(this->fd_, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: %s", strerror(errno));
    return false;
  }

  this->buffers_.resize(req.count);
  this->queued_mask_ = 0;
  for (uint32_t i = 0; i < req.count; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(this->fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%u] failed: %s", (unsigned) i, strerror(errno));
      return false;
    }
    this->buffers_[i].length = buf.length;
    this->buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, this->fd_, buf.m.offset);
    if (this->buffers_[i].start == MAP_FAILED) {
      ESP_LOGE(TAG, "mmap[%u] failed: %s", (unsigned) i, strerror(errno));
      return false;
    }
  }
  if (!this->requeue_all_buffers_())
    return false;
  this->set_dqbuf_timeout_(this->fd_, DQBUF_POLL_TIMEOUT_MS);
  return true;
}

bool EspVideoCamera::setup_ppa_() {
  // Output geometry: 90/270 swap width and height.
  if (this->rotation_ == 90 || this->rotation_ == 270) {
    this->out_w_ = this->cap_h_;
    this->out_h_ = this->cap_w_;
  } else {
    this->out_w_ = this->cap_w_;
    this->out_h_ = this->cap_h_;
  }
  mjpeg_half_resolution(this->out_w_, this->out_h_, this->stream_w_, this->stream_h_);
  this->stream_rgb_size_ =
      (static_cast<size_t>(this->stream_w_) * this->stream_h_ * 3 + 127) & ~static_cast<size_t>(127);

  ppa_client_config_t pc = {};
  pc.oper_type = PPA_OPERATION_SRM;
  esp_err_t err = ppa_register_client(&pc, &this->ppa_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ppa_register_client failed: %s", esp_err_to_name(err));
    return false;
  }

  // PPA output buffer in PSRAM, aligned for L1/L2 cache as the PPA requires.
  this->rotated_size_ =
      (static_cast<size_t>(this->out_w_) * this->out_h_ * 3 + 127) & ~static_cast<size_t>(127);
  this->rotated_ =
      heap_caps_aligned_alloc(128, this->rotated_size_, MALLOC_CAP_SPIRAM);
  if (this->rotated_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u-byte PPA output buffer in PSRAM",
             (unsigned) this->rotated_size_);
    return false;
  }
  return true;
}

bool EspVideoCamera::ensure_stream_rgb_() {
  if (this->stream_rgb_ != nullptr)
    return true;
  if (this->stream_w_ == 0 || this->stream_h_ == 0 || this->stream_rgb_size_ == 0)
    return false;
  this->stream_rgb_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(128, this->stream_rgb_size_, MALLOC_CAP_SPIRAM));
  if (this->stream_rgb_ == nullptr) {
    ESP_LOGW(TAG, "Failed to allocate %u-byte MJPEG RGB buffer in PSRAM",
             (unsigned) this->stream_rgb_size_);
    return false;
  }
  return true;
}

bool EspVideoCamera::setup_h264_() {
  mjpeg_half_resolution(this->out_w_, this->out_h_, this->h264_w_, this->h264_h_);
  this->h264_yuv_size_ = static_cast<size_t>(this->h264_w_) * this->h264_h_ * 3 / 2;
  this->h264_yuv_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(128, this->h264_yuv_size_, MALLOC_CAP_SPIRAM));
  if (this->h264_yuv_ == nullptr)
    return false;

  bool cap_streaming = false;
  auto cleanup = [&]() {
    if (cap_streaming && this->h264_fd_ >= 0) {
      int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(this->h264_fd_, VIDIOC_STREAMOFF, &t);
    }
    if (this->h264_output_ != nullptr && this->h264_output_ != MAP_FAILED)
      munmap(this->h264_output_, this->h264_output_size_);
    this->h264_output_ = nullptr;
    this->h264_output_size_ = 0;
    if (this->h264_fd_ >= 0) {
      close(this->h264_fd_);
      this->h264_fd_ = -1;
    }
    if (this->h264_yuv_ != nullptr) {
      heap_caps_free(this->h264_yuv_);
      this->h264_yuv_ = nullptr;
      this->h264_yuv_size_ = 0;
    }
    this->h264_buf_state_.reset();
    return false;
  };

  this->h264_fd_ = open(ESP_VIDEO_H264_DEVICE_NAME, O_RDWR);
  if (this->h264_fd_ < 0) {
    ESP_LOGW(TAG, "open(%s) failed: %s", ESP_VIDEO_H264_DEVICE_NAME, strerror(errno));
    return cleanup();
  }

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = this->h264_w_;
  fmt.fmt.pix.height = this->h264_h_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
  if (xioctl(this->h264_fd_, VIDIOC_S_FMT, &fmt) != 0)
    return cleanup();

  struct v4l2_requestbuffers req = {};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  req.memory = V4L2_MEMORY_USERPTR;
  if (xioctl(this->h264_fd_, VIDIOC_REQBUFS, &req) != 0)
    return cleanup();

  fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = this->h264_w_;
  fmt.fmt.pix.height = this->h264_h_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
  fmt.fmt.pix.sizeimage = static_cast<uint32_t>(this->h264_w_) * this->h264_h_;
  if (xioctl(this->h264_fd_, VIDIOC_S_FMT, &fmt) != 0)
    return cleanup();

  req = {};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(this->h264_fd_, VIDIOC_REQBUFS, &req) != 0)
    return cleanup();

  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if (xioctl(this->h264_fd_, VIDIOC_QUERYBUF, &buf) != 0)
    return cleanup();
  this->h264_output_size_ = buf.length;
  this->h264_output_ = static_cast<uint8_t *>(mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                                    MAP_SHARED, this->h264_fd_, buf.m.offset));
  if (this->h264_output_ == MAP_FAILED)
    return cleanup();
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &buf) != 0)
    return cleanup();
  this->h264_buf_state_.reset();
  this->h264_buf_state_.on_cap_queued();
  if (!this->set_dqbuf_timeout_(this->h264_fd_, 200)) {
    ESP_LOGW(TAG, "H.264 VIDIOC_S_DQBUF_TIMEOUT failed: %s; refusing unbounded DQBUF",
             strerror(errno));
    return cleanup();
  }

  set_ext_ctrl(this->h264_fd_, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 15);
  set_ext_ctrl(this->h264_fd_, V4L2_CID_MPEG_VIDEO_BITRATE, 1000000);
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(this->h264_fd_, VIDIOC_STREAMON, &type) != 0)
    return cleanup();
  cap_streaming = true;
  type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (xioctl(this->h264_fd_, VIDIOC_STREAMON, &type) != 0)
    return cleanup();

  ESP_LOGCONFIG(TAG, "H.264 RTSP: %ux%u @ 10 fps, 1 Mbps, TCP port 8554",
                this->h264_w_, this->h264_h_);
  return true;
}

bool EspVideoCamera::convert_frame_(const void *input, void *output, size_t output_size,
                                    uint16_t width, uint16_t height,
                                    ppa_srm_color_mode_t mode, float scale_x,
                                    float scale_y) {
  ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
  if (this->rotation_ == 90)
    angle = PPA_SRM_ROTATION_ANGLE_90;
  else if (this->rotation_ == 180)
    angle = PPA_SRM_ROTATION_ANGLE_180;
  else if (this->rotation_ == 270)
    angle = PPA_SRM_ROTATION_ANGLE_270;

  ppa_srm_oper_config_t op = {};
  op.in.buffer = input;
  op.in.pic_w = this->cap_w_;
  op.in.pic_h = this->cap_h_;
  op.in.block_w = this->cap_w_;
  op.in.block_h = this->cap_h_;
  op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  op.out.buffer = output;
  op.out.buffer_size = output_size;
  op.out.pic_w = width;
  op.out.pic_h = height;
  op.out.srm_cm = mode;
  op.out.yuv_range = PPA_COLOR_RANGE_LIMIT;
  op.out.yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601;
  op.rotation_angle = angle;
  op.scale_x = scale_x;
  op.scale_y = scale_y;
  op.rgb_swap = this->swap_rgb_;
  op.mode = PPA_TRANS_MODE_BLOCKING;
  int64_t t0 = esp_timer_get_time();
  esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &op);
  this->last_ppa_us_ = static_cast<uint32_t>(esp_timer_get_time() - t0);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "PPA rotate/convert failed: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

bool EspVideoCamera::init() { return this->ready_; }

bool EspVideoCamera::start() {
  if (!this->ready_)
    return false;
  if (this->task_mode_) {
    this->stream_requested_.store(true);
    return true;
  }
  if (this->v4l2_mutex_ == nullptr)
    return false;
  xSemaphoreTake(this->v4l2_mutex_, portMAX_DELAY);
  if (this->non_task_stop_pending_) {
    if (!this->stop_v4l2_stream_()) {
      xSemaphoreGive(this->v4l2_mutex_);
      return false;
    }
    this->non_task_stop_pending_ = false;
    this->v4l2_active_ = false;
  }
  if (this->v4l2_active_) {
    this->streaming_.store(true);
    xSemaphoreGive(this->v4l2_mutex_);
    return true;
  }
  const uint32_t epoch = this->stop_epoch_.load();
  const bool ok = this->start_v4l2_stream_(epoch);
  this->v4l2_active_ = ok;
  xSemaphoreGive(this->v4l2_mutex_);
  return ok;
}

void EspVideoCamera::apply_sensor_controls_() {
  // esp_video exposes sensor controls through the extended control API
  // (set_ext_ctrl); the legacy VIDIOC_S_CTRL path isn't wired up. Query the
  // range best-effort (for logging + fractional targets), falling back to a
  // known range when the query isn't supported.
  auto query_range = [this](uint32_t cid, int fb_lo, int fb_hi, int &lo, int &hi,
                            int &def) {
    lo = fb_lo;
    hi = fb_hi;
    def = -1;
    struct v4l2_queryctrl q = {};
    q.id = cid;
    if (xioctl(this->fd_, VIDIOC_QUERYCTRL, &q) == 0 &&
        !(q.flags & V4L2_CTRL_FLAG_DISABLED)) {
      lo = (int) q.minimum;
      hi = (int) q.maximum;
      def = (int) q.default_value;
    }
  };
  auto pick = [](int explicit_val, int lo, int hi, float frac) {
    int v = explicit_val >= 0 ? explicit_val : (int) (lo + (hi - lo) * frac);
    return v < lo ? lo : (v > hi ? hi : v);
  };

  int lo, hi, def;

  // Exposure is the dominant lever: the SC202CS powers up at its minimum (~8 of
  // a ~1244 max = a near-black frame), so lift it well up by default. Remember
  // the intent even if this first set fails, so reassert_controls_ retries it.
  query_range(V4L2_CID_EXPOSURE, 8, 1244, lo, hi, def);
  this->applied_exp_cid_ = V4L2_CID_EXPOSURE;
  this->applied_exp_val_ = pick(this->exposure_, lo, hi, 0.65f);
  if (set_ext_ctrl(this->fd_, this->applied_exp_cid_, this->applied_exp_val_))
    ESP_LOGCONFIG(TAG, "sensor exposure=%d (range %d..%d, default %d)",
                  this->applied_exp_val_, lo, hi, def);
  else
    ESP_LOGW(TAG, "set exposure=%d failed: %s (will retry)",
             this->applied_exp_val_, strerror(errno));

  // Gain: a modest lift on top. Prefer the generic gain control (which the
  // SC202CS accepts); only fall back to analogue gain if that's rejected.
  query_range(V4L2_CID_GAIN, 0, 255, lo, hi, def);
  int gval = pick(this->gain_, lo, hi, 0.25f);
  if (set_ext_ctrl(this->fd_, V4L2_CID_GAIN, gval)) {
    this->applied_gain_cid_ = V4L2_CID_GAIN;
    this->applied_gain_val_ = gval;
    ESP_LOGCONFIG(TAG, "sensor gain=%d (range %d..%d, default %d)", gval, lo, hi,
                  def);
  } else {
    query_range(V4L2_CID_ANALOGUE_GAIN, 0, 255, lo, hi, def);
    gval = pick(this->gain_, lo, hi, 0.25f);
    if (set_ext_ctrl(this->fd_, V4L2_CID_ANALOGUE_GAIN, gval)) {
      this->applied_gain_cid_ = V4L2_CID_ANALOGUE_GAIN;
      this->applied_gain_val_ = gval;
      ESP_LOGCONFIG(TAG, "sensor analogue_gain=%d (range %d..%d, default %d)",
                    gval, lo, hi, def);
    }
  }
  this->last_ctrl_us_ = esp_timer_get_time();
}

void EspVideoCamera::reassert_controls_() {
  if (this->applied_exp_val_ >= 0)
    set_ext_ctrl(this->fd_, this->applied_exp_cid_, this->applied_exp_val_);
  if (this->applied_gain_val_ >= 0 && this->applied_gain_cid_ != 0)
    set_ext_ctrl(this->fd_, this->applied_gain_cid_, this->applied_gain_val_);
}

void EspVideoCamera::stop() {
  // Immediately mark idle, bump stop epoch, cut RTSP playback, reset streaming
  // AWB seed, and wake every waiter so privacy-off unblocks all consumers.
  this->stream_requested_.store(false);
  this->stop_epoch_.fetch_add(1);
  this->streaming_.store(false);
  this->rtsp_playing_.store(false);
  this->mjpeg_awb_reset_.store(true);
  this->wake_all_waiters_();
  // sock_mutex_ is held only for fd registration/clear and non-blocking
  // shutdown() (never across send/recv/httpd_resp_send) so shutdown() cannot
  // race with close()/fd reuse.
  if (this->sock_mutex_ != nullptr) {
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
#ifdef USE_WEBSERVER
    if (this->mjpeg_active_sockfd_ >= 0)
      shutdown(this->mjpeg_active_sockfd_, SHUT_RDWR);
    if (this->snapshot_active_sockfd_ >= 0)
      shutdown(this->snapshot_active_sockfd_, SHUT_RDWR);
#endif
    const int rtsp_fd = this->rtsp_client_fd_.load();
    if (rtsp_fd >= 0)
      shutdown(rtsp_fd, SHUT_RDWR);
    xSemaphoreGive(this->sock_mutex_);
  }
  if (this->task_mode_ || !this->ready_ || this->v4l2_mutex_ == nullptr)
    return;

  xSemaphoreTake(this->v4l2_mutex_, portMAX_DELAY);
  if (this->v4l2_active_) {
    if (this->stop_v4l2_stream_()) {
      this->v4l2_active_ = false;
      this->non_task_stop_pending_ = false;
    } else {
      this->non_task_stop_pending_ = true;
    }
  }
  xSemaphoreGive(this->v4l2_mutex_);
}

bool EspVideoCamera::acquire(person_detect::FrameView &out, uint32_t timeout_ms) {
  if (!this->ready_)
    return false;
  if (this->task_mode_) {
    const uint32_t req_epoch = this->stop_epoch_.load();
    if (this->frame_done_ == nullptr || !this->stream_requested_.load() ||
        this->stop_epoch_.load() != req_epoch) {
      return false;
    }
    xSemaphoreTake(this->frame_done_, 0);
    if (!this->frame_handoff_.request(req_epoch))
      return false;
    if (!this->stream_requested_.load() || this->stop_epoch_.load() != req_epoch) {
      this->frame_handoff_.cancel_or_release(req_epoch);
      return false;
    }
    if (xSemaphoreTake(this->frame_done_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
      this->frame_handoff_.cancel_or_release(req_epoch);
      this->capture_failures_++;
      return false;
    }
    const bool valid =
        this->stream_requested_.load() && this->stop_epoch_.load() == req_epoch;
    if (!this->frame_handoff_.accept(req_epoch, valid))
      return false;
    out.data = static_cast<const uint8_t *>(this->rotated_);
    out.width = this->out_w_;
    out.height = this->out_h_;
    out.format = person_detect::FRAME_FORMAT_RGB888;
    return true;
  }
  if (this->v4l2_mutex_ == nullptr || !this->streaming_.load())
    return false;

  const uint32_t start_epoch = this->stop_epoch_.load();
  const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
  bool flushed_stale = false;
  bool converted_full = false;
  bool converted_mjpeg = false;
  uint32_t snap_gen = 0;
  uint32_t mjpeg_gen = 0;

  while (this->streaming_.load() && this->stop_epoch_.load() == start_epoch) {
    if (xSemaphoreTake(this->v4l2_mutex_, pdMS_TO_TICKS(DQBUF_POLL_TIMEOUT_MS)) != pdTRUE) {
      if (esp_timer_get_time() >= deadline_us) {
        this->capture_failures_++;
        return false;
      }
      continue;
    }
    if (!this->streaming_.load() || !this->v4l2_active_ || this->non_task_stop_pending_ ||
        this->stop_epoch_.load() != start_epoch) {
      xSemaphoreGive(this->v4l2_mutex_);
      return false;
    }

    // Re-assert exposure/gain periodically: a set right at STREAMON occasionally
    // doesn't take (frame stays near-black) and some ISP paths drift it, so this
    // self-corrects within a couple seconds regardless.
    if (esp_timer_get_time() - this->last_ctrl_us_ > 2000000) {
      this->reassert_controls_();
      this->last_ctrl_us_ = esp_timer_get_time();
    }

    // Drop already-captured frames with a 0-tick non-blocking timeout so
    // inference runs on a LIVE frame without risking an unbounded wait when
    // esp_video ignores O_NONBLOCK.
    if (!flushed_stale) {
      flushed_stale = true;
      this->set_dqbuf_timeout_(this->fd_, 0);
      for (size_t i = 0; i < this->buffers_.size(); i++) {
        struct v4l2_buffer stale = {};
        stale.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        stale.memory = V4L2_MEMORY_MMAP;
        if (xioctl(this->fd_, VIDIOC_DQBUF, &stale) != 0)
          break;  // nothing (more) ready
        if (stale.index < 32)
          this->queued_mask_ &= ~(1u << stale.index);
        if (xioctl(this->fd_, VIDIOC_QBUF, &stale) == 0 && stale.index < 32)
          this->queued_mask_ |= (1u << stale.index);
      }
      this->set_dqbuf_timeout_(this->fd_, DQBUF_POLL_TIMEOUT_MS);
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    const int64_t dq_t0 = esp_timer_get_time();
    const int rc = xioctl(this->fd_, VIDIOC_DQBUF, &buf);
    if (rc == 0) {
      if (buf.index < 32)
        this->queued_mask_ &= ~(1u << buf.index);
      if (this->streaming_.load() && this->stop_epoch_.load() == start_epoch) {
        void *raw = this->buffers_[buf.index].start;
        snap_gen = this->claim_snapshot_request_();
        mjpeg_gen = this->claim_mjpeg_request_();
        // Rotate (portrait mount) + RGB565->RGB888 in one PPA pass.
        converted_full = this->convert_frame_(raw, this->rotated_, this->rotated_size_,
                                              this->out_w_, this->out_h_,
                                              PPA_SRM_COLOR_MODE_RGB888, 1.0f, 1.0f);
        if (mjpeg_gen != 0 && this->ensure_stream_rgb_()) {
          converted_mjpeg = this->convert_frame_(raw, this->stream_rgb_, this->stream_rgb_size_,
                                                 this->stream_w_, this->stream_h_,
                                                 PPA_SRM_COLOR_MODE_RGB888, 0.5f, 0.5f);
        }
      }
      // Immediately re-queue the CSI buffer before AWB, JPEG, or inference.
      if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGW(TAG, "VIDIOC_QBUF failed: %s", strerror(errno));
      } else if (buf.index < 32) {
        this->queued_mask_ |= (1u << buf.index);
      }
      xSemaphoreGive(this->v4l2_mutex_);
      break;
    }
    xSemaphoreGive(this->v4l2_mutex_);

    if (!this->streaming_.load() || this->stop_epoch_.load() != start_epoch)
      return false;
    if (esp_timer_get_time() >= deadline_us) {
      this->capture_failures_++;
      return false;
    }
    if (esp_timer_get_time() - dq_t0 < 5000)
      vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (!this->streaming_.load() || this->stop_epoch_.load() != start_epoch)
    return false;

  if (!converted_full) {
    if (snap_gen != 0)
      this->fail_snapshot_(snap_gen);
    if (mjpeg_gen != 0) {
      if (converted_mjpeg)
        this->encode_mjpeg_(mjpeg_gen, start_epoch);
      else
        this->fail_mjpeg_(mjpeg_gen);
    }
    return false;
  }

  if (snap_gen != 0)
    this->encode_snapshot_(snap_gen, start_epoch);
  if (mjpeg_gen != 0) {
    if (converted_mjpeg)
      this->encode_mjpeg_(mjpeg_gen, start_epoch);
    else
      this->fail_mjpeg_(mjpeg_gen);
  }

  out.data = static_cast<const uint8_t *>(this->rotated_);
  out.width = this->out_w_;
  out.height = this->out_h_;
  out.format = person_detect::FRAME_FORMAT_RGB888;
  return true;
}

void EspVideoCamera::capture_task_entry_(void *arg) {
  static_cast<EspVideoCamera *>(arg)->capture_task_();
}

// Brought up from the camera task well after boot so a failure here can't
// block boot (OTA rollback) and its log lines reach the API log stream.
void EspVideoCamera::start_h264_() {
  ESP_LOGI(TAG, "H.264: setup (internal free %u, psram free %u)",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  this->rtsp_send_mutex_ = xSemaphoreCreateMutex();
  if (this->rtsp_send_mutex_ == nullptr || !this->setup_h264_()) {
    ESP_LOGW(TAG, "H.264: encoder setup failed; RTSP disabled");
    return;
  }
  ESP_LOGI(TAG, "H.264: encoder ready (internal free %u)",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (xTaskCreatePinnedToCore(&EspVideoCamera::rtsp_task_entry_, "camera_rtsp", 8192, this, 1,
                              &this->rtsp_task_handle_, 0) != pdPASS) {
    ESP_LOGW(TAG, "H.264: RTSP task create failed");
    return;
  }
  this->h264_ready_ = true;
  ESP_LOGI(TAG, "H.264: RTSP listening on :8554");
}

void EspVideoCamera::capture_task_() {
  ESP_LOGI(TAG, "camera task running (task_mode=%s)", YESNO(this->task_mode_));
  StopEpochLatch stop_latch{};
  bool camera_on = false;
  bool h264_tried = false;
  uint32_t consecutive_dq_misses = 0;

  for (;;) {
    if (!this->task_mode_)
      break;
    // ponytail: fixed 30 s boot delay; lets ESPHome mark the app valid first.
    if (this->h264_enabled_ && !h264_tried && esp_timer_get_time() > 30000000) {
      h264_tried = true;
      this->start_h264_();
    }

    const uint32_t observed_epoch = this->stop_epoch_.load();
    const bool wanted = this->stream_requested_.load();
    const StreamLifecycleAction action =
        stop_latch.step(observed_epoch, wanted, camera_on);
    if (action == StreamLifecycleAction::STOP) {
      if (this->stop_v4l2_stream_()) {
        camera_on = false;
        stop_latch.mark_stopped(observed_epoch);
        this->wake_all_waiters_();
        continue;
      }
      this->wake_all_waiters_();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (action == StreamLifecycleAction::START) {
      if (this->start_v4l2_stream_(observed_epoch)) {
        camera_on = true;
        consecutive_dq_misses = 0;
        // Re-check stream_requested_ and stop_epoch_ before dequeuing a frame.
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    if (action == StreamLifecycleAction::IDLE) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (esp_timer_get_time() - this->last_ctrl_us_ > 2000000) {
      this->reassert_controls_();
      this->last_ctrl_us_ = esp_timer_get_time();
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    const int64_t dq_t0 = esp_timer_get_time();
    if (xioctl(this->fd_, VIDIOC_DQBUF, &buf) != 0) {
      if (this->stream_requested_.load() && this->stop_epoch_.load() == observed_epoch) {
        consecutive_dq_misses++;
        if (consecutive_dq_misses == 20 || (consecutive_dq_misses > 20 && consecutive_dq_misses % 50 == 0)) {
          ESP_LOGW(TAG, "camera DQBUF timed out/failed (%u consecutive, errno=%d: %s)",
                   (unsigned) consecutive_dq_misses, errno, strerror(errno));
        }
      }
      // Yield briefly only if ioctl returned immediately instead of sleeping in ready_sem.
      if (esp_timer_get_time() - dq_t0 < 5000)
        vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (buf.index < 32)
      this->queued_mask_ &= ~(1u << buf.index);

    if (this->stop_epoch_.load() != observed_epoch || !this->stream_requested_.load()) {
      if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGW(TAG, "camera QBUF failed: %s", strerror(errno));
      } else if (buf.index < 32) {
        this->queued_mask_ |= (1u << buf.index);
      }
      continue;
    }

    consecutive_dq_misses = 0;
    this->task_frames_++;
    void *raw = this->buffers_[buf.index].start;

    // 1. Full-res PPA pass for person_detect and /snapshot.jpg (shares rotated_,
    //    gated on !frame_handoff_.is_in_use() so inference always sees an untouched frame).
    const uint32_t snap_gen = this->claim_snapshot_request_();
    bool full_ready = false;
    bool full_failed = false;
    const bool need_full = !this->frame_handoff_.is_in_use() &&
                           (this->frame_handoff_.is_requested_for(observed_epoch) || snap_gen != 0);
    if (need_full) {
      full_ready = this->convert_frame_(raw, this->rotated_, this->rotated_size_,
                                        this->out_w_, this->out_h_,
                                        PPA_SRM_COLOR_MODE_RGB888, 1.0f, 1.0f);
      full_failed = !full_ready;
    }

    // 2. Half-res PPA pass for /stream.mjpg — decoupled from frame_handoff_ so
    //    streaming continues smoothly while person_detect runs inference.
    const uint32_t mjpeg_gen = this->claim_mjpeg_request_();
    bool do_mjpeg = false;
    bool mjpeg_failed = false;
    if (mjpeg_gen != 0) {
      if (this->ensure_stream_rgb_() &&
          this->convert_frame_(raw, this->stream_rgb_, this->stream_rgb_size_,
                               this->stream_w_, this->stream_h_,
                               PPA_SRM_COLOR_MODE_RGB888, 0.5f, 0.5f)) {
        do_mjpeg = true;
      } else {
        mjpeg_failed = true;
      }
    }

    // 3. H.264 PPA pass (converts raw -> h264_yuv_ while CSI buffer is valid).
    //    If a previous USERPTR/CAPTURE buffer is still pending recovery, defer
    //    PPA so h264_yuv_ is not overwritten while the encoder owns it.
    bool do_h264 = false;
    const int64_t now = esp_timer_get_time();
    if (this->h264_ready_ && this->rtsp_playing_.load() && now - this->last_h264_us_ >= 100000) {
      if (this->h264_buf_state_.needs_recovery()) {
        this->last_h264_us_ = now;
        do_h264 = true;
      } else if (this->convert_frame_(raw, this->h264_yuv_, this->h264_yuv_size_,
                                      this->h264_w_, this->h264_h_, PPA_SRM_COLOR_MODE_YUV420,
                                      0.5f, 0.5f)) {
        this->last_h264_us_ = now;
        do_h264 = true;
      }
    }

    // 4. Re-queue the CSI V4L2 buffer immediately before AWB or HW encoding.
    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGW(TAG, "camera QBUF failed: %s", strerror(errno));
    } else if (buf.index < 32) {
      this->queued_mask_ |= (1u << buf.index);
    }

    if (this->stop_epoch_.load() != observed_epoch || !this->stream_requested_.load())
      continue;

    // 5. Post-QBUF: snapshot/detector dispatch, MJPEG AWB+JPEG, H.264 encode.
    if (full_ready) {
      if (snap_gen != 0)
        this->encode_snapshot_(snap_gen, observed_epoch);
      if (this->frame_handoff_.deliver(observed_epoch))
        xSemaphoreGive(this->frame_done_);
    } else if (full_failed && snap_gen != 0) {
      this->fail_snapshot_(snap_gen);
    }

    if (do_mjpeg) {
      this->encode_mjpeg_(mjpeg_gen, observed_epoch);
    } else if (mjpeg_failed) {
      this->fail_mjpeg_(mjpeg_gen);
    }

    if (do_h264 && this->streaming_.load() && this->stop_epoch_.load() == observed_epoch)
      this->encode_h264_(observed_epoch);
  }
  this->capture_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

bool EspVideoCamera::encode_h264_(uint32_t capture_epoch) {
  if (this->h264_buf_state_.needs_recovery()) {
    // Reclaim stale H.264 buffers left behind by a previous partial DQBUF timeout
    // (esp_video 2.2.0 returns EPERM on timeout; QBUF fails with EINVAL until
    // both CAPTURE and OUTPUT return to the ready state).
    if (this->h264_buf_state_.cap_pending && this->h264_buf_state_.cap_queued) {
      struct v4l2_buffer stale = {};
      stale.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      stale.memory = V4L2_MEMORY_MMAP;
      if (xioctl(this->h264_fd_, VIDIOC_DQBUF, &stale) == 0) {
        this->h264_buf_state_.on_cap_dequeued();
        if (xioctl(this->h264_fd_, VIDIOC_QBUF, &stale) == 0)
          this->h264_buf_state_.on_cap_queued();
      }
    } else if (!this->h264_buf_state_.cap_queued) {
      struct v4l2_buffer cap = {};
      cap.index = 0;
      cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      cap.memory = V4L2_MEMORY_MMAP;
      if (xioctl(this->h264_fd_, VIDIOC_QBUF, &cap) == 0)
        this->h264_buf_state_.on_cap_queued();
    }
    if (this->h264_buf_state_.out_queued) {
      struct v4l2_buffer done_out = {};
      done_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
      done_out.memory = V4L2_MEMORY_USERPTR;
      if (xioctl(this->h264_fd_, VIDIOC_DQBUF, &done_out) == 0)
        this->h264_buf_state_.on_out_dequeued();
    }
    // h264_yuv_ was not refreshed while recovery was pending.
    return false;
  }

  if (!this->streaming_.load() || this->stop_epoch_.load() != capture_epoch)
    return false;

  struct v4l2_buffer out = {};
  out.index = 0;
  out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out.memory = V4L2_MEMORY_USERPTR;
  out.m.userptr = reinterpret_cast<unsigned long>(this->h264_yuv_);
  out.length = this->h264_yuv_size_;
  out.bytesused = this->h264_yuv_size_;
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &out) != 0)
    return false;
  this->h264_buf_state_.on_out_queued();

  // VIDIOC_S_DQBUF_TIMEOUT (200 ms) bounds the V4L2 semaphore wait; note that
  // esp_video 2.2.0 runs ESP_VIDEO_M2M_TRIGGER synchronously before that wait
  // (up to the hardware driver's internal 1000-tick encode bound), which Phase 2
  // removes when replacing V4L2 H.264 with direct esp_h264_enc_hw.
  struct v4l2_buffer encoded = {};
  encoded.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  encoded.memory = V4L2_MEMORY_MMAP;
  const bool got_encoded = xioctl(this->h264_fd_, VIDIOC_DQBUF, &encoded) == 0;
  if (got_encoded)
    this->h264_buf_state_.on_cap_dequeued();

  struct v4l2_buffer done_out = {};
  done_out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  done_out.memory = V4L2_MEMORY_USERPTR;
  if (xioctl(this->h264_fd_, VIDIOC_DQBUF, &done_out) == 0)
    this->h264_buf_state_.on_out_dequeued();

  if (!got_encoded)
    return false;

  if (encoded.bytesused != 0 && this->streaming_.load() &&
      this->stop_epoch_.load() == capture_epoch) {
    this->send_h264_rtp_(this->h264_output_, encoded.bytesused, capture_epoch);
  }
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &encoded) != 0)
    return false;
  this->h264_buf_state_.on_cap_queued();
  return true;
}

bool EspVideoCamera::send_rtsp_(int fd, const char *data, size_t len) {
  if (xSemaphoreTake(this->rtsp_send_mutex_, pdMS_TO_TICKS(200)) != pdTRUE)
    return false;
  size_t sent = 0;
  while (sent < len) {
    int n = send(fd, data + sent, len - sent, 0);
    if (n <= 0) {
      xSemaphoreGive(this->rtsp_send_mutex_);
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  xSemaphoreGive(this->rtsp_send_mutex_);
  return true;
}

void EspVideoCamera::send_h264_rtp_(const uint8_t *data, size_t len, uint32_t capture_epoch) {
  int fd = -1;
  uint32_t session_gen = 0;
  if (this->sock_mutex_ == nullptr)
    return;
  xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
  fd = this->rtsp_client_fd_.load();
  session_gen = this->rtsp_session_gen_.load();
  xSemaphoreGive(this->sock_mutex_);
  if (fd < 0 || !this->rtsp_playing_.load() ||
      !this->streaming_.load() || this->stop_epoch_.load() != capture_epoch) {
    return;
  }

  bool ok = true;
  packetize_h264_annex_b(data, len, this->rtp_timestamp_, this->rtp_sequence_, 1400,
      [&](const uint8_t *packet, size_t packet_len, bool) {
        if (!ok)
          return;
        if (this->rtsp_session_gen_.load() != session_gen || !this->rtsp_playing_.load() ||
            !this->streaming_.load() || this->stop_epoch_.load() != capture_epoch) {
          ok = false;
          return;
        }
        std::array<uint8_t, 1416> framed{};
        framed[0] = '$';
        framed[1] = 0;
        framed[2] = static_cast<uint8_t>(packet_len >> 8);
        framed[3] = static_cast<uint8_t>(packet_len);
        memcpy(framed.data() + 4, packet, packet_len);
        if (xSemaphoreTake(this->rtsp_send_mutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
          ok = false;
          return;
        }
        if (this->rtsp_session_gen_.load() != session_gen ||
            this->rtsp_client_fd_.load() != fd || !this->rtsp_playing_.load() ||
            !this->streaming_.load() || this->stop_epoch_.load() != capture_epoch) {
          xSemaphoreGive(this->rtsp_send_mutex_);
          ok = false;
          return;
        }
        int n = send(fd, framed.data(), packet_len + 4, MSG_DONTWAIT);
        xSemaphoreGive(this->rtsp_send_mutex_);
        ok = n == static_cast<int>(packet_len + 4);
      });
  this->rtp_timestamp_ += 9000;
  if (!ok) {
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    if (this->rtsp_session_gen_.load() == session_gen && this->rtsp_client_fd_.load() == fd) {
      this->rtsp_playing_.store(false);
      shutdown(fd, SHUT_RDWR);
    }
    xSemaphoreGive(this->sock_mutex_);
  }
}

void EspVideoCamera::rtsp_task_entry_(void *arg) {
  static_cast<EspVideoCamera *>(arg)->rtsp_task_();
}

void EspVideoCamera::rtsp_task_() {
  int server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (server < 0) {
    ESP_LOGW(TAG, "RTSP socket failed: %s", strerror(errno));
    vTaskDelete(nullptr);
    return;
  }
  int one = 1;
  setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_port = htons(8554);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      listen(server, 1) != 0) {
    ESP_LOGW(TAG, "RTSP bind/listen failed: %s", strerror(errno));
    close(server);
    vTaskDelete(nullptr);
    return;
  }

  for (;;) {
    int client = accept(server, nullptr, nullptr);
    if (client < 0)
      continue;
    timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    this->rtsp_session_gen_.fetch_add(1);
    this->rtsp_client_fd_.store(client);
    xSemaphoreGive(this->sock_mutex_);
    this->rtsp_playing_.store(false);
    bool setup = false;
    std::string pending;

    while (this->rtsp_client_fd_.load() == client) {
      char chunk[1024];
      int n = recv(client, chunk, sizeof(chunk), 0);
      if (n == 0)
        break;
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          continue;
        break;
      }
      pending.append(chunk, n);
      if (pending.size() > 4096)
        break;

      for (;;) {
        if (!pending.empty() && pending[0] == '$') {
          if (pending.size() < 4)
            break;
          size_t interleaved_len =
              (static_cast<uint8_t>(pending[2]) << 8) | static_cast<uint8_t>(pending[3]);
          if (pending.size() < interleaved_len + 4)
            break;
          pending.erase(0, interleaved_len + 4);  // ignore client RTCP
          continue;
        }
        size_t request_end = pending.find("\r\n\r\n");
        if (request_end == std::string::npos)
          break;
        std::string request = pending.substr(0, request_end + 4);
        pending.erase(0, request_end + 4);
        const std::string cseq = rtsp_header(request, "CSeq");
        const std::string auth = rtsp_header(request, "Authorization");
        char response[1024];
        bool play_after_response = false;
        uint32_t play_epoch = 0;

        if (auth != this->h264_auth_) {
          snprintf(response, sizeof(response),
                   "RTSP/1.0 401 Unauthorized\r\nCSeq: %s\r\n"
                   "WWW-Authenticate: Basic realm=\"ESP32-P4 Camera\"\r\n\r\n",
                   cseq.c_str());
        } else if (request.rfind("OPTIONS ", 0) == 0) {
          snprintf(response, sizeof(response),
                   "RTSP/1.0 200 OK\r\nCSeq: %s\r\n"
                   "Public: OPTIONS, DESCRIBE, SETUP, PLAY, GET_PARAMETER, TEARDOWN\r\n\r\n",
                   cseq.c_str());
        } else if (request.rfind("DESCRIBE ", 0) == 0) {
          char sdp[384];
          int sdp_len = snprintf(sdp, sizeof(sdp),
              "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=ESP32-P4 Camera\r\nt=0 0\r\n"
              "a=control:*\r\nm=video 0 RTP/AVP 96\r\nc=IN IP4 0.0.0.0\r\n"
              "a=rtpmap:96 H264/90000\r\na=framerate:10\r\na=framesize:96 %u-%u\r\n"
              "a=fmtp:96 packetization-mode=1\r\na=control:trackID=0\r\n",
              this->h264_w_, this->h264_h_);
          int head = snprintf(response, sizeof(response),
              "RTSP/1.0 200 OK\r\nCSeq: %s\r\nContent-Type: application/sdp\r\n"
              "Content-Length: %d\r\n\r\n%s", cseq.c_str(), sdp_len, sdp);
          if (head < 0)
            break;
        } else if (request.rfind("SETUP ", 0) == 0) {
          const std::string transport = rtsp_header(request, "Transport");
          if (transport.find("RTP/AVP/TCP") == std::string::npos) {
            snprintf(response, sizeof(response), "RTSP/1.0 461 Unsupported Transport\r\nCSeq: %s\r\n\r\n",
                     cseq.c_str());
          } else {
            setup = true;
            snprintf(response, sizeof(response),
                     "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n"
                     "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n",
                     cseq.c_str());
          }
        } else if (request.rfind("PLAY ", 0) == 0 && setup) {
          this->rtp_sequence_ = 0;
          this->rtp_timestamp_ = 0;
          play_epoch = this->stop_epoch_.load();
          play_after_response = true;
          snprintf(response, sizeof(response),
                   "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n"
                   "RTP-Info: url=trackID=0;seq=0;rtptime=0\r\n\r\n", cseq.c_str());
        } else if (request.rfind("GET_PARAMETER ", 0) == 0) {
          snprintf(response, sizeof(response), "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n\r\n",
                   cseq.c_str());
        } else if (request.rfind("TEARDOWN ", 0) == 0) {
          snprintf(response, sizeof(response), "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n\r\n",
                   cseq.c_str());
          this->send_rtsp_(client, response, strlen(response));
          this->rtsp_playing_.store(false);
          goto client_done;
        } else {
          snprintf(response, sizeof(response), "RTSP/1.0 405 Method Not Allowed\r\nCSeq: %s\r\n\r\n",
                   cseq.c_str());
        }
        if (!this->send_rtsp_(client, response, strlen(response)))
          goto client_done;
        if (play_after_response) {
          this->rtsp_playing_.store(true);
          if (this->stop_epoch_.load() != play_epoch) {
            this->rtsp_playing_.store(false);
            goto client_done;
          }
        }
      }
    }
client_done:
    this->rtsp_playing_.store(false);
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    this->rtsp_client_fd_.store(-1);
    xSemaphoreGive(this->sock_mutex_);
    shutdown(client, SHUT_RDWR);
    // Wait for any in-flight send_h264_rtp_ packet on this fd before close().
    if (this->rtsp_send_mutex_ != nullptr) {
      xSemaphoreTake(this->rtsp_send_mutex_, portMAX_DELAY);
      xSemaphoreGive(this->rtsp_send_mutex_);
    }
    close(client);
  }
}

bool EspVideoCamera::ensure_jpeg_encoder_() {
  if (this->jpeg_encoder_ != nullptr)
    return true;
  jpeg_encode_engine_cfg_t eng = {};
  eng.timeout_ms = 500;
  jpeg_encoder_handle_t h = nullptr;
  if (jpeg_new_encoder_engine(&eng, &h) != ESP_OK) {
    ESP_LOGW(TAG, "JPEG encoder init failed");
    return false;
  }
  this->jpeg_encoder_ = h;
  return true;
}

uint32_t EspVideoCamera::claim_snapshot_request_() {
  if (this->snapshot_buf_mutex_ == nullptr || !this->snapshot_requested_.load())
    return 0;
  xSemaphoreTake(this->snapshot_buf_mutex_, portMAX_DELAY);
  const uint32_t gen = this->snapshot_state_.claim_request();
  xSemaphoreGive(this->snapshot_buf_mutex_);
  return gen;
}

void EspVideoCamera::fail_snapshot_(uint32_t claimed_gen) {
  if (claimed_gen == 0 || this->snapshot_buf_mutex_ == nullptr)
    return;
  xSemaphoreTake(this->snapshot_buf_mutex_, portMAX_DELAY);
  const bool notify = this->snapshot_state_.complete_request(claimed_gen, 0);
  if (!this->snapshot_state_.pending)
    this->snapshot_requested_.store(false);
  xSemaphoreGive(this->snapshot_buf_mutex_);
  if (notify && this->snapshot_done_ != nullptr)
    xSemaphoreGive(this->snapshot_done_);
}

void EspVideoCamera::encode_snapshot_(uint32_t claimed_gen, uint32_t expected_epoch) {
  if (claimed_gen == 0 || this->snapshot_buf_mutex_ == nullptr)
    return;

  // Out-of-place white balance runs into balanced_ before locking the JPEG
  // buffer; rotated_ is strictly read-only so person_detect never sees AWB.
  const uint8_t *src = static_cast<const uint8_t *>(this->rotated_);
  if (this->awb_enabled_) {
    if (this->balanced_ == nullptr)
      this->balanced_ = static_cast<uint8_t *>(
          heap_caps_aligned_alloc(128, this->rotated_size_, MALLOC_CAP_SPIRAM));
    if (this->balanced_ != nullptr) {
      gray_world_copy(src, this->balanced_, static_cast<size_t>(this->out_w_) * this->out_h_);
      src = this->balanced_;
    }
  }

  xSemaphoreTake(this->snapshot_buf_mutex_, portMAX_DELAY);
  if (!this->snapshot_state_.pending || this->snapshot_state_.req_gen != claimed_gen ||
      !this->streaming_.load() || this->stop_epoch_.load() != expected_epoch) {
    if (!this->snapshot_state_.pending)
      this->snapshot_requested_.store(false);
    xSemaphoreGive(this->snapshot_buf_mutex_);
    return;
  }

  uint32_t out = 0;
  if (this->ensure_jpeg_encoder_()) {
    if (this->snapshot_jpeg_buf_ == nullptr) {
      jpeg_encode_memory_alloc_cfg_t mem = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
      // ponytail: fixed 1/4 of raw RGB888; ample for q70 indoor frames.
      this->snapshot_jpeg_buf_ = static_cast<uint8_t *>(
          jpeg_alloc_encoder_mem(this->rotated_size_ / 4, &mem, &this->snapshot_jpeg_buf_size_));
    }
    if (this->snapshot_jpeg_buf_ != nullptr) {
      jpeg_encode_cfg_t cfg = {};
      cfg.width = this->out_w_;
      cfg.height = this->out_h_;
      cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
      cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
      cfg.image_quality = MJPEG_JPEG_QUALITY;
      esp_err_t err = jpeg_encoder_process(
          static_cast<jpeg_encoder_handle_t>(this->jpeg_encoder_), &cfg, src,
          static_cast<uint32_t>(this->out_w_) * this->out_h_ * 3,
          this->snapshot_jpeg_buf_, this->snapshot_jpeg_buf_size_, &out);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Snapshot JPEG encode failed: %s", esp_err_to_name(err));
        out = 0;
      }
    }
  }

  const uint32_t final_len =
      (this->streaming_.load() && this->stop_epoch_.load() == expected_epoch) ? out : 0;
  const bool notify = this->snapshot_state_.complete_request(claimed_gen, final_len);
  if (!this->snapshot_state_.pending)
    this->snapshot_requested_.store(false);
  xSemaphoreGive(this->snapshot_buf_mutex_);
  if (notify && this->snapshot_done_ != nullptr)
    xSemaphoreGive(this->snapshot_done_);
}

uint32_t EspVideoCamera::claim_mjpeg_request_() {
  if (this->mjpeg_buf_mutex_ == nullptr || !this->mjpeg_requested_.load())
    return 0;
  xSemaphoreTake(this->mjpeg_buf_mutex_, portMAX_DELAY);
  const uint32_t gen = this->mjpeg_state_.claim_request();
  xSemaphoreGive(this->mjpeg_buf_mutex_);
  return gen;
}

void EspVideoCamera::fail_mjpeg_(uint32_t claimed_gen) {
  if (claimed_gen == 0 || this->mjpeg_buf_mutex_ == nullptr)
    return;
  xSemaphoreTake(this->mjpeg_buf_mutex_, portMAX_DELAY);
  const bool notify = this->mjpeg_state_.complete_request(claimed_gen, 0);
  if (!this->mjpeg_state_.pending)
    this->mjpeg_requested_.store(false);
  xSemaphoreGive(this->mjpeg_buf_mutex_);
  if (notify && this->mjpeg_done_ != nullptr)
    xSemaphoreGive(this->mjpeg_done_);
}

void EspVideoCamera::encode_mjpeg_(uint32_t claimed_gen, uint32_t expected_epoch) {
  if (claimed_gen == 0 || this->mjpeg_buf_mutex_ == nullptr || this->stream_rgb_ == nullptr)
    return;

  if (this->awb_enabled_) {
    if (this->mjpeg_awb_reset_.exchange(false))
      this->mjpeg_awb_state_.reset();
    gray_world_apply_inplace(this->stream_rgb_,
                             static_cast<size_t>(this->stream_w_) * this->stream_h_,
                             this->mjpeg_awb_state_);
  }

  xSemaphoreTake(this->mjpeg_buf_mutex_, portMAX_DELAY);
  if (!this->mjpeg_state_.pending || this->mjpeg_state_.req_gen != claimed_gen ||
      !this->streaming_.load() || this->stop_epoch_.load() != expected_epoch) {
    if (!this->mjpeg_state_.pending)
      this->mjpeg_requested_.store(false);
    xSemaphoreGive(this->mjpeg_buf_mutex_);
    return;
  }

  uint32_t out = 0;
  if (this->ensure_jpeg_encoder_()) {
    if (this->mjpeg_jpeg_raw_ == nullptr) {
      jpeg_encode_memory_alloc_cfg_t mem = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
      const size_t payload_target = this->stream_rgb_size_ / 4;
      size_t alloc_size = 0;
      this->mjpeg_jpeg_raw_ = static_cast<uint8_t *>(
          jpeg_alloc_encoder_mem(payload_target + MJPEG_PART_HEADROOM + MJPEG_PART_TAILROOM,
                                 &mem, &alloc_size));
      if (this->mjpeg_jpeg_raw_ != nullptr &&
          alloc_size > MJPEG_PART_HEADROOM + MJPEG_PART_TAILROOM) {
        this->mjpeg_jpeg_buf_ = this->mjpeg_jpeg_raw_ + MJPEG_PART_HEADROOM;
        this->mjpeg_jpeg_buf_size_ = alloc_size - MJPEG_PART_HEADROOM - MJPEG_PART_TAILROOM;
      }
    }
    if (this->mjpeg_jpeg_buf_ != nullptr && this->mjpeg_jpeg_buf_size_ > 0) {
      jpeg_encode_cfg_t cfg = {};
      cfg.width = this->stream_w_;
      cfg.height = this->stream_h_;
      cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
      cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
      cfg.image_quality = MJPEG_JPEG_QUALITY;
      esp_err_t err = jpeg_encoder_process(
          static_cast<jpeg_encoder_handle_t>(this->jpeg_encoder_), &cfg,
          this->stream_rgb_, static_cast<uint32_t>(this->stream_w_) * this->stream_h_ * 3,
          this->mjpeg_jpeg_buf_, this->mjpeg_jpeg_buf_size_, &out);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "MJPEG encode failed: %s", esp_err_to_name(err));
        out = 0;
      }
    }
  }

  const uint32_t final_len =
      (this->streaming_.load() && this->stop_epoch_.load() == expected_epoch) ? out : 0;
  const bool notify = this->mjpeg_state_.complete_request(claimed_gen, final_len);
  if (!this->mjpeg_state_.pending)
    this->mjpeg_requested_.store(false);
  xSemaphoreGive(this->mjpeg_buf_mutex_);
  if (notify && this->mjpeg_done_ != nullptr)
    xSemaphoreGive(this->mjpeg_done_);
}

#ifdef USE_WEBSERVER
void EspVideoCamera::handle_snapshot_http(httpd_req_t *req) {
  const uint32_t session_epoch = this->stop_epoch_.load();
  if (this->snapshot_done_ == nullptr || this->snapshot_buf_mutex_ == nullptr ||
      this->sock_mutex_ == nullptr ||
      !StopEpochLatch::session_valid(session_epoch, this->stop_epoch_.load(),
                                     this->streaming_.load())) {
    send_http_503(req, "camera idle or busy");
    return;
  }

  if (xSemaphoreTake(this->snapshot_buf_mutex_, pdMS_TO_TICKS(HTTP_CAPTURE_TIMEOUT_MS)) != pdTRUE) {
    send_http_503(req, "camera idle or busy");
    return;
  }
  if (!StopEpochLatch::session_valid(session_epoch, this->stop_epoch_.load(),
                                     this->streaming_.load())) {
    xSemaphoreGive(this->snapshot_buf_mutex_);
    send_http_503(req, "camera idle or busy");
    return;
  }
  const uint32_t token = this->snapshot_state_.begin_request();
  xSemaphoreTake(this->snapshot_done_, 0);
  this->snapshot_requested_.store(true);
  xSemaphoreGive(this->snapshot_buf_mutex_);

  const bool waited = xSemaphoreTake(this->snapshot_done_, pdMS_TO_TICKS(HTTP_CAPTURE_TIMEOUT_MS)) == pdTRUE;

  xSemaphoreTake(this->snapshot_buf_mutex_, portMAX_DELAY);
  size_t len = 0;
  const bool session_ok = StopEpochLatch::session_valid(
      session_epoch, this->stop_epoch_.load(), this->streaming_.load());
  const bool ok = this->snapshot_state_.finish_wait(token, waited, session_ok, len);
  if (!this->snapshot_state_.pending)
    this->snapshot_requested_.store(false);
  bool sent = false;
  if (ok) {
    const int sockfd = httpd_req_to_sockfd(req);
    bool registered = false;
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    const bool still_valid = StopEpochLatch::session_valid(
        session_epoch, this->stop_epoch_.load(), this->streaming_.load());
    if (still_valid && sockfd >= 0) {
      this->snapshot_active_sockfd_ = sockfd;
      registered = true;
    }
    xSemaphoreGive(this->sock_mutex_);
    if (still_valid) {
      httpd_resp_set_status(req, HTTPD_200);
      httpd_resp_set_type(req, "image/jpeg");
      httpd_resp_set_hdr(req, "Cache-Control", "no-store");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      httpd_resp_send(req, reinterpret_cast<const char *>(this->snapshot_jpeg_buf_), len);
      sent = true;
    }
    if (registered) {
      xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
      this->snapshot_active_sockfd_ = -1;
      xSemaphoreGive(this->sock_mutex_);
    }
  }
  xSemaphoreGive(this->snapshot_buf_mutex_);

  if (!sent)
    send_http_503(req, "camera idle or busy");
}

void EspVideoCamera::handle_mjpeg_http(httpd_req_t *raw) {
  const uint32_t session_epoch = this->stop_epoch_.load();
  if (this->mjpeg_req_queue_ == nullptr || this->mjpeg_done_ == nullptr ||
      this->mjpeg_buf_mutex_ == nullptr || this->sock_mutex_ == nullptr ||
      !StopEpochLatch::session_valid(session_epoch, this->stop_epoch_.load(),
                                     this->streaming_.load())) {
    send_http_503(raw, "camera idle");
    return;
  }

  // Single active MJPEG viewer: reject a second viewer immediately without blocking httpd.
  bool expected = false;
  if (!this->mjpeg_client_active_.compare_exchange_strong(expected, true)) {
    send_http_503(raw, "stream busy");
    return;
  }

  if (!StopEpochLatch::session_valid(session_epoch, this->stop_epoch_.load(),
                                     this->streaming_.load())) {
    this->mjpeg_client_active_.store(false);
    send_http_503(raw, "camera idle");
    return;
  }

  if (this->mjpeg_worker_handle_ == nullptr) {
    if (xTaskCreatePinnedToCore(&EspVideoCamera::mjpeg_worker_entry_, "camera_mjpeg",
                                6144, this, 1, &this->mjpeg_worker_handle_, 0) != pdPASS) {
      this->mjpeg_worker_handle_ = nullptr;
      this->mjpeg_client_active_.store(false);
      send_http_503(raw, "stream worker unavailable");
      return;
    }
  }

  httpd_req_t *async_req = nullptr;
  if (httpd_req_async_handler_begin(raw, &async_req) != ESP_OK) {
    this->mjpeg_client_active_.store(false);
    send_http_503(raw, "async stream unavailable");
    return;
  }

  const MjpegQueuedRequest queued{async_req, session_epoch};
  if (xQueueSend(this->mjpeg_req_queue_, &queued, 0) != pdTRUE) {
    send_http_503(async_req, "stream busy");
    httpd_req_async_handler_complete(async_req);
    this->mjpeg_client_active_.store(false);
    return;
  }
}

void EspVideoCamera::mjpeg_worker_entry_(void *arg) {
  static_cast<EspVideoCamera *>(arg)->mjpeg_worker_task_();
}

void EspVideoCamera::mjpeg_worker_task_() {
  for (;;) {
    MjpegQueuedRequest queued{nullptr, 0};
    if (xQueueReceive(this->mjpeg_req_queue_, &queued, portMAX_DELAY) != pdTRUE ||
        queued.req == nullptr) {
      continue;
    }
    this->serve_mjpeg_client_(queued.req, queued.session_epoch);
    httpd_req_async_handler_complete(queued.req);
    this->mjpeg_client_active_.store(false);
  }
}

void EspVideoCamera::serve_mjpeg_client_(httpd_req_t *req, uint32_t session_epoch) {
  auto session_active = [this, session_epoch]() {
    return StopEpochLatch::session_valid(session_epoch, this->stop_epoch_.load(),
                                         this->streaming_.load());
  };

  if (!session_active()) {
    send_http_503(req, "camera idle or busy");
    return;
  }

  const int sockfd = httpd_req_to_sockfd(req);
  if (sockfd >= 0) {
    timeval tv = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    this->mjpeg_active_sockfd_ = sockfd;
    xSemaphoreGive(this->sock_mutex_);
  }

  // Seed EMA AWB immediately on the first frame of a new stream.
  this->mjpeg_awb_reset_.store(true);

  bool headers_sent = false;
  bool send_failed = false;
  int64_t next_us = 0;

  while (session_active()) {
    const int64_t wait_us = next_us - esp_timer_get_time();
    if (wait_us > 0) {
      int64_t rem_ms = (wait_us + 999) / 1000;
      while (rem_ms > 0 && session_active()) {
        const int64_t step = rem_ms > 20 ? 20 : rem_ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        rem_ms -= step;
      }
      if (!session_active())
        break;
    }
    next_us = esp_timer_get_time() + MJPEG_MIN_FRAME_US;

    xSemaphoreTake(this->mjpeg_buf_mutex_, portMAX_DELAY);
    if (!session_active()) {
      xSemaphoreGive(this->mjpeg_buf_mutex_);
      break;
    }
    const uint32_t token = this->mjpeg_state_.begin_request();
    xSemaphoreTake(this->mjpeg_done_, 0);
    this->mjpeg_requested_.store(true);
    xSemaphoreGive(this->mjpeg_buf_mutex_);

    const bool waited = xSemaphoreTake(this->mjpeg_done_, pdMS_TO_TICKS(HTTP_CAPTURE_TIMEOUT_MS)) == pdTRUE;

    xSemaphoreTake(this->mjpeg_buf_mutex_, portMAX_DELAY);
    size_t jpeg_len = 0;
    const bool ok = this->mjpeg_state_.finish_wait(token, waited, session_active(), jpeg_len);
    if (!this->mjpeg_state_.pending)
      this->mjpeg_requested_.store(false);
    if (!ok) {
      xSemaphoreGive(this->mjpeg_buf_mutex_);
      break;
    }

    const uint8_t *frame_ptr = nullptr;
    size_t frame_len = 0;
    if (!frame_mjpeg_in_place(this->mjpeg_jpeg_buf_, MJPEG_PART_HEADROOM, jpeg_len,
                              MJPEG_PART_TAILROOM, frame_ptr, frame_len)) {
      xSemaphoreGive(this->mjpeg_buf_mutex_);
      break;
    }
    // Flush the dirty cache line from the trailing "\r\n" write before any
    // exit path releases the buffer back to HW JPEG DMA2D (jpeg_encoder_process
    // only invalidates M2C after DMA, so a dirty line could evict over the next frame).
    esp_cache_msync(this->mjpeg_jpeg_buf_ + jpeg_len, 2,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    if (!session_active()) {
      xSemaphoreGive(this->mjpeg_buf_mutex_);
      break;
    }

    if (!headers_sent) {
      if (httpd_resp_set_status(req, HTTPD_200) != ESP_OK ||
          httpd_resp_set_type(req, MJPEG_CONTENT_TYPE) != ESP_OK ||
          httpd_resp_set_hdr(req, "Cache-Control", "no-store") != ESP_OK ||
          httpd_resp_set_hdr(req, "Connection", "close") != ESP_OK ||
          httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*") != ESP_OK) {
        xSemaphoreGive(this->mjpeg_buf_mutex_);
        send_failed = true;
        break;
      }
      headers_sent = true;
    }

    // mjpeg_requested_ is false while we hold mjpeg_buf_mutex_, so camera_stream
    // never blocks on this socket write.
    const esp_err_t err =
        httpd_resp_send_chunk(req, reinterpret_cast<const char *>(frame_ptr), frame_len);
    xSemaphoreGive(this->mjpeg_buf_mutex_);
    if (err != ESP_OK) {
      send_failed = true;
      break;
    }
  }

  if (!headers_sent && !send_failed) {
    if (session_active()) {
      send_http_503(req, "camera idle or busy");
    } else if (sockfd >= 0) {
      shutdown(sockfd, SHUT_RDWR);
    }
  } else if (headers_sent) {
    const bool aborted = send_failed || !session_active();
    if (!aborted)
      httpd_resp_send_chunk(req, nullptr, 0);
    if (sockfd >= 0) {
      // Mark the stream socket ready for httpd session teardown once async
      // ownership returns: SHUT_RDWR on stalled/broken/privacy-stopped send,
      // SHUT_RD after a clean terminating chunk so TX finishes flushing.
      shutdown(sockfd, aborted ? SHUT_RDWR : SHUT_RD);
    }
  }

  // Clear active sockfd BEFORE returning to httpd_req_async_handler_complete()
  // so stop() can never call shutdown() on a recycled fd.
  xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
  this->mjpeg_active_sockfd_ = -1;
  xSemaphoreGive(this->sock_mutex_);
}
#endif

void EspVideoCamera::release() {
  if (this->task_mode_)
    this->frame_handoff_.release();
}

void EspVideoCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "esp_video_camera (MIPI-CSI):");
#ifdef USE_I2C
  if (this->sccb_bus_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  SCCB (sensor I2C): shared ESPHome i2c bus @ %uHz",
                  (unsigned) this->sccb_freq_);
  } else
#endif
  {
    ESP_LOGCONFIG(TAG, "  SCCB (sensor I2C): own master I2C%d SDA=%d SCL=%d @ %uHz",
                  this->sccb_port_, this->sccb_sda_, this->sccb_scl_,
                  (unsigned) this->sccb_freq_);
  }
  LOG_PIN("  Enable pin: ", this->enable_pin_);
  LOG_PIN("  Power-down pin: ", this->powerdown_pin_);
  LOG_PIN("  Reset pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  Capture: %ux%u RGB565, %u buffers", this->cap_w_,
                this->cap_h_, this->fb_count_);
  ESP_LOGCONFIG(TAG, "  Rotation: %u deg -> output %ux%u RGB888 (swap_rgb=%s)",
                this->rotation_, this->out_w_, this->out_h_,
                YESNO(this->swap_rgb_));
  ESP_LOGCONFIG(TAG, "  PPA output buffer: %u bytes PSRAM",
                (unsigned) this->rotated_size_);
  if (this->mjpeg_stream_enabled_) {
    ESP_LOGCONFIG(TAG, "  MJPEG stream: %ux%u q%u (lazy %u-byte PSRAM buffer)",
                  this->stream_w_, this->stream_h_, (unsigned) MJPEG_JPEG_QUALITY,
                  (unsigned) this->stream_rgb_size_);
  }
  if (this->last_ppa_us_ != 0) {
    ESP_LOGCONFIG(TAG, "  Last PPA rotate: %u us", (unsigned) this->last_ppa_us_);
  }
  ESP_LOGCONFIG(TAG, "  Capture failures: %u", (unsigned) this->capture_failures_);
  if (this->task_mode_)
    ESP_LOGCONFIG(TAG, "  Camera task: %s, %u frames", this->capture_task_handle_ ? "running" : "EXITED",
                  (unsigned) this->task_frames_.load());
  if (this->h264_enabled_) {
    ESP_LOGCONFIG(TAG, "  H.264 RTSP: %s, %ux%u, port 8554, Basic auth",
                  this->h264_ready_ ? "ready" : "unavailable", this->h264_w_, this->h264_h_);
  }
  if (this->is_failed())
    ESP_LOGE(TAG, "  Component failed to set up");
}

}  // namespace esp_video_camera
}  // namespace esphome
