#include "esp_video_camera.h"

#include "esphome/core/application.h"
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
#include <sys/select.h>
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
#include "esp_video_init.h"
#include "esp_video_ioctl.h"

#ifdef USE_ESP_VIDEO_CAMERA_H264
#include "esp_h264_enc_single_hw.h"
#endif

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
static constexpr int64_t H264_RTP_FRAME_BUDGET_US = 100000;       // 100 ms per access unit
static constexpr int64_t RTSP_CTRL_SNDTIMEO_US = 200000;          // 200 ms control send timeout
static constexpr int64_t RTSP_CTRL_SEND_DEADLINE_US = 250000;     // 250 ms mutex + framing wait
static constexpr size_t H264_RTP_SCRATCH_ALLOC_SIZE = 1536;       // 128B-aligned >= 1416B

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
    // exits if it sees task_mode_ false. Only clear on creation failure so we
    // never concurrently overwrite task_mode_ while the running task reads it.
    if (this->frame_done_ != nullptr) {
      this->task_mode_ = true;
      if (xTaskCreatePinnedToCore(&EspVideoCamera::capture_task_entry_, "camera_stream",
                                  8192, this, 3, &this->capture_task_handle_, 1) != pdPASS) {
        this->task_mode_ = false;
      }
    }
    if (!this->task_mode_)
      ESP_LOGW(TAG, "camera task unavailable; streams follow the detector interval");
  }
#ifdef USE_ESP_VIDEO_CAMERA_H264
  // Note: RTSP socket bind/listen is deferred to loop() after App.is_setup_complete()
  // (see start_rtsp_listener_()) because setup() runs at priority DATA (600), before
  // ESPHome's NetworkComponent initializes lwIP / esp_netif_init() at AFTER_BLUETOOTH
  // (300), and earlier loops can still be polled while a mid-priority component blocks
  // in can_proceed().
#endif
  ESP_LOGCONFIG(TAG, "esp_video_camera ready: capture %ux%u -> rotate %u -> "
                     "RGB888 %ux%u (stream %ux%u)",
                this->cap_w_, this->cap_h_, this->rotation_, this->out_w_,
                this->out_h_, this->stream_w_, this->stream_h_);
}

void EspVideoCamera::loop() {
#ifdef USE_ESP_VIDEO_CAMERA_H264
  if (!this->h264_enabled_ || !this->task_mode_) {
    this->disable_loop();
    return;
  }
  if (!App.is_setup_complete())
    return;
  if (!this->rtsp_listener_attempted_) {
    this->rtsp_listener_attempted_ = true;
    this->start_rtsp_listener_();
  }
#endif
  this->disable_loop();
}

void EspVideoCamera::start_rtsp_listener_() {
#ifdef USE_ESP_VIDEO_CAMERA_H264
  // Bind the lightweight RTSP listener synchronously once lwIP is up (after all
  // setup() priorities have completed) before publishing h264_ready_. The direct
  // esp_h264 hardware encoder and its buffers remain lazily allocated on PLAY
  // and freed immediately on TEARDOWN/disconnect/stop (0 B encoder SRAM idle).
  bool rtsp_ok = false;
  this->rtsp_send_mutex_ = xSemaphoreCreateMutex();
  if (this->rtsp_send_mutex_ == nullptr) {
    ESP_LOGW(TAG, "H.264: RTSP send mutex allocation failed; RTSP disabled");
  } else {
    int server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server < 0) {
      ESP_LOGW(TAG, "H.264: RTSP socket failed: %s; RTSP disabled", strerror(errno));
    } else {
      int one = 1;
      setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
      sockaddr_in address = {};
      address.sin_family = AF_INET;
      address.sin_port = htons(8554);
      address.sin_addr.s_addr = htonl(INADDR_ANY);
      if (bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
          listen(server, 2) != 0) {
        ESP_LOGW(TAG, "H.264: RTSP bind/listen on :8554 failed: %s; RTSP disabled",
                 strerror(errno));
        close(server);
      } else {
        this->rtsp_server_fd_ = server;
        if (xTaskCreatePinnedToCore(&EspVideoCamera::rtsp_task_entry_, "camera_rtsp", 6144,
                                    this, 1, &this->rtsp_task_handle_, 0) == pdPASS) {
          rtsp_ok = true;
        } else {
          ESP_LOGW(TAG, "H.264: RTSP task creation failed; RTSP disabled");
        }
      }
    }
  }
  if (rtsp_ok) {
    this->h264_ready_.store(true);
    ESP_LOGI(TAG, "H.264 RTSP listening on :8554 (%ux%u @ %u fps, lazy encoder)",
             this->h264_geom_.out_w, this->h264_geom_.out_h, (unsigned) H264_FPS);
  } else {
    if (this->rtsp_server_fd_ >= 0) {
      close(this->rtsp_server_fd_);
      this->rtsp_server_fd_ = -1;
    }
    if (this->rtsp_send_mutex_ != nullptr) {
      vSemaphoreDelete(this->rtsp_send_mutex_);
      this->rtsp_send_mutex_ = nullptr;
    }
    this->h264_ready_.store(false);
  }
#endif
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
  this->h264_geom_ = compute_h264_geometry(this->cap_w_, this->cap_h_, this->rotation_);

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

bool EspVideoCamera::open_h264_() {
#ifdef USE_ESP_VIDEO_CAMERA_H264
  if (this->h264_enc_ != nullptr)
    return true;
  if (this->h264_geom_.out_w < 80 || this->h264_geom_.out_h < 80) {
    ESP_LOGW(TAG, "H.264: invalid geometry %ux%u", this->h264_geom_.out_w, this->h264_geom_.out_h);
    return false;
  }

  const size_t yuv_bytes =
      ((static_cast<size_t>(this->h264_geom_.out_w) * this->h264_geom_.out_h * 3u / 2u) + 127u) &
      ~static_cast<size_t>(127u);
  this->h264_yuv_size_ = yuv_bytes;
  // esp_h264_enc_single.h requires out_frame.raw_data.len >= in_frame.raw_data.len.
  this->h264_output_size_ = yuv_bytes;
  this->h264_yuv_ =
      static_cast<uint8_t *>(heap_caps_aligned_alloc(128, this->h264_yuv_size_, MALLOC_CAP_SPIRAM));
  this->h264_output_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(128, this->h264_output_size_, MALLOC_CAP_SPIRAM));
  this->rtp_packet_buf_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(128, H264_RTP_SCRATCH_ALLOC_SIZE, MALLOC_CAP_SPIRAM));
  if (this->h264_yuv_ == nullptr || this->h264_output_ == nullptr ||
      this->rtp_packet_buf_ == nullptr) {
    ESP_LOGW(TAG, "H.264: PSRAM buffer alloc failed (psram free %u)",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    this->teardown_h264_();
    return false;
  }

  static constexpr size_t H264_MIN_PRE_NEW_INTERNAL_FREE = 96 * 1024;
  static constexpr size_t H264_MIN_PRE_NEW_INTERNAL_BLOCK = 64 * 1024;
  static constexpr size_t H264_MIN_PRE_OPEN_INTERNAL_FREE = 16 * 1024;
  static constexpr size_t H264_MIN_PRE_OPEN_INTERNAL_BLOCK = 4 * 1024;

  const size_t pre_new_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t pre_new_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  if (pre_new_free < H264_MIN_PRE_NEW_INTERNAL_FREE ||
      pre_new_largest < H264_MIN_PRE_NEW_INTERNAL_BLOCK) {
    ESP_LOGW(TAG, "H.264: low internal SRAM before enc_hw_new (free %u, largest %u)",
             (unsigned) pre_new_free, (unsigned) pre_new_largest);
    this->teardown_h264_();
    return false;
  }

  esp_h264_enc_cfg_hw_t cfg = {};
  cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;
  cfg.gop = H264_GOP;
  cfg.fps = H264_FPS;
  cfg.res.width = this->h264_geom_.out_w;
  cfg.res.height = this->h264_geom_.out_h;
  cfg.rc.bitrate = H264_BITRATE;
  cfg.rc.qp_min = H264_QP_MIN;
  cfg.rc.qp_max = H264_QP_MAX;

  esp_h264_enc_handle_t enc = nullptr;
  esp_h264_err_t err = esp_h264_enc_hw_new(&cfg, &enc);
  if (err != ESP_H264_ERR_OK || enc == nullptr) {
    ESP_LOGW(TAG, "H.264: esp_h264_enc_hw_new failed (%d, internal free %u, psram free %u)",
             static_cast<int>(err), (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    this->teardown_h264_();
    return false;
  }
  // espressif/esp_h264 1.3.0 (exact version hard-pinned by espressif/esp_video 2.2.0)
  // omits a NULL check on its internal frame_done semaphore inside enc_open(). This
  // pre-open internal SRAM headroom check is a best-effort mitigation (not an atomic
  // reservation) so enc_open() is skipped under obvious internal-heap pressure.
  const size_t pre_open_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t pre_open_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  if (pre_open_free < H264_MIN_PRE_OPEN_INTERNAL_FREE ||
      pre_open_largest < H264_MIN_PRE_OPEN_INTERNAL_BLOCK) {
    ESP_LOGW(TAG, "H.264: low internal SRAM before enc_open (free %u, largest %u)",
             (unsigned) pre_open_free, (unsigned) pre_open_largest);
    esp_h264_enc_del(enc);
    this->teardown_h264_();
    return false;
  }
  err = esp_h264_enc_open(enc);
  if (err != ESP_H264_ERR_OK) {
    ESP_LOGW(TAG, "H.264: esp_h264_enc_open failed (%d)", static_cast<int>(err));
    esp_h264_enc_del(enc);
    this->teardown_h264_();
    return false;
  }

  this->h264_enc_ = enc;
  this->h264_awb_state_.reset();
  this->rtp_sequence_ = 0;
  this->rtsp_play_start_us_ = 0;
  this->last_h264_us_ = 0;
  this->h264_encode_failures_ = 0;
  ESP_LOGI(TAG, "H.264: encoder opened %ux%u @ %u fps, %u kbps (internal free %u, psram free %u)",
           this->h264_geom_.out_w, this->h264_geom_.out_h, (unsigned) H264_FPS,
           (unsigned) (H264_BITRATE / 1000u),
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  return true;
#else
  return false;
#endif
}

void EspVideoCamera::teardown_h264_() {
  const bool had_resources = this->h264_enc_ != nullptr || this->h264_yuv_ != nullptr ||
                             this->h264_output_ != nullptr || this->rtp_packet_buf_ != nullptr;
#ifdef USE_ESP_VIDEO_CAMERA_H264
  if (this->h264_enc_ != nullptr) {
    auto enc = static_cast<esp_h264_enc_handle_t>(this->h264_enc_);
    esp_h264_enc_close(enc);
    esp_h264_enc_del(enc);
    this->h264_enc_ = nullptr;
  }
#endif
  if (this->h264_yuv_ != nullptr) {
    heap_caps_free(this->h264_yuv_);
    this->h264_yuv_ = nullptr;
  }
  this->h264_yuv_size_ = 0;
  if (this->h264_output_ != nullptr) {
    heap_caps_free(this->h264_output_);
    this->h264_output_ = nullptr;
  }
  this->h264_output_size_ = 0;
  if (this->rtp_packet_buf_ != nullptr) {
    heap_caps_free(this->rtp_packet_buf_);
    this->rtp_packet_buf_ = nullptr;
  }
  this->h264_awb_state_.reset();
  this->rtsp_play_start_us_ = 0;
  this->last_h264_us_ = 0;
  if (had_resources) {
    ESP_LOGI(TAG, "H.264: encoder closed (internal free %u, psram free %u)",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  }
}

bool EspVideoCamera::convert_frame_(const void *input, void *output, size_t output_size,
                                    uint16_t width, uint16_t height,
                                    ppa_srm_color_mode_t mode, float scale_x,
                                    float scale_y, uint16_t block_w, uint16_t block_h,
                                    uint16_t offset_x, uint16_t offset_y) {
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
  op.in.block_w = block_w != 0 ? block_w : this->cap_w_;
  op.in.block_h = block_h != 0 ? block_h : this->cap_h_;
  op.in.block_offset_x = offset_x;
  op.in.block_offset_y = offset_y;
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
  auto pick = [](int explicit_val, int lo, int hi, float frac) {
    int v = explicit_val >= 0 ? explicit_val : (int) (lo + (hi - lo) * frac);
    return v < lo ? lo : (v > hi ? hi : v);
  };

  if (!this->auto_exposure_) {
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
    this->rotated_exposure_scale_ = 1.0f;
    this->last_ctrl_us_ = esp_timer_get_time();
    return;
  }

  // Auto-exposure path: esp_video 2.2.0 implements VIDIOC_QUERY_EXT_CTRL (not
  // legacy VIDIOC_QUERYCTRL), returning SC202CS's true bounds (exposure 8..1244,
  // gain index 0..191). Fall back to VIDIOC_QUERYCTRL then known SC202CS bounds.
  auto query_ext_range = [this](uint32_t cid, int fb_lo, int fb_hi, int &lo, int &hi,
                                int &def) -> bool {
    lo = fb_lo;
    hi = fb_hi;
    def = -1;
    struct v4l2_query_ext_ctrl qe = {};
    qe.id = cid;
    if (xioctl(this->fd_, VIDIOC_QUERY_EXT_CTRL, &qe) == 0 &&
        !(qe.flags & V4L2_CTRL_FLAG_DISABLED)) {
      lo = static_cast<int>(qe.minimum);
      hi = static_cast<int>(qe.maximum);
      def = static_cast<int>(qe.default_value);
      return true;
    }
    struct v4l2_queryctrl q = {};
    q.id = cid;
    if (xioctl(this->fd_, VIDIOC_QUERYCTRL, &q) == 0 &&
        !(q.flags & V4L2_CTRL_FLAG_DISABLED)) {
      lo = static_cast<int>(q.minimum);
      hi = static_cast<int>(q.maximum);
      def = static_cast<int>(q.default_value);
      return true;
    }
    return false;
  };

  int exp_lo, exp_hi, exp_def;
  query_ext_range(V4L2_CID_EXPOSURE, 8, 1244, exp_lo, exp_hi, exp_def);
  const int max_exp = pick(this->exposure_, exp_lo, exp_hi, 0.65f);

  int gain_lo, gain_hi, gain_def;
  const bool gain_queried = query_ext_range(V4L2_CID_GAIN, 0, 191, gain_lo, gain_hi, gain_def);
  int max_gain = pick(this->gain_, gain_lo, gain_hi, 0.25f);

  this->applied_exp_cid_ = V4L2_CID_EXPOSURE;
  this->applied_exp_val_ = max_exp;
  if (set_ext_ctrl(this->fd_, this->applied_exp_cid_, this->applied_exp_val_)) {
    ESP_LOGCONFIG(TAG, "sensor exposure=%d (AE range %d..%d, sensor %d..%d, default %d)",
                  this->applied_exp_val_, exp_lo, max_exp, exp_lo, exp_hi, exp_def);
  } else {
    ESP_LOGW(TAG, "set exposure=%d failed: %s (will retry)",
             this->applied_exp_val_, strerror(errno));
  }

  this->applied_gain_cid_ = 0;
  this->applied_gain_val_ = -1;
  if (set_ext_ctrl(this->fd_, V4L2_CID_GAIN, max_gain)) {
    this->applied_gain_cid_ = V4L2_CID_GAIN;
    this->applied_gain_val_ = max_gain;
    ESP_LOGCONFIG(TAG, "sensor gain=%d (AE range %d..%d, sensor %d..%d, default %d)",
                  max_gain, gain_lo, max_gain, gain_lo, gain_hi, gain_def);
  } else if (gain_queried) {
    // V4L2_CID_GAIN is confirmed supported by QUERY_EXT_CTRL; keep it armed so
    // the post-STREAMON reassert retries this transient write failure.
    this->applied_gain_cid_ = V4L2_CID_GAIN;
    this->applied_gain_val_ = max_gain;
    ESP_LOGW(TAG, "set gain=%d failed: %s (will retry)", max_gain, strerror(errno));
  } else {
    query_ext_range(V4L2_CID_ANALOGUE_GAIN, 0, 191, gain_lo, gain_hi, gain_def);
    max_gain = pick(this->gain_, gain_lo, gain_hi, 0.25f);
    if (set_ext_ctrl(this->fd_, V4L2_CID_ANALOGUE_GAIN, max_gain)) {
      this->applied_gain_cid_ = V4L2_CID_ANALOGUE_GAIN;
      this->applied_gain_val_ = max_gain;
      ESP_LOGCONFIG(TAG, "sensor analogue_gain=%d (AE range %d..%d, sensor %d..%d, default %d)",
                    max_gain, gain_lo, max_gain, gain_lo, gain_hi, gain_def);
    } else {
      gain_lo = 0;
      max_gain = 0;
    }
  }

  // Configure and reset AE to its ceiling (exposure_scale == 1.0f) on every
  // STREAMON so privacy OFF/ON recovers cleanly.
  this->ae_.configure(exp_lo, max_exp, gain_lo, max_gain);
  this->applied_exp_val_ = this->ae_.exposure();
  if (this->applied_gain_cid_ != 0)
    this->applied_gain_val_ = this->ae_.gain();
  this->pending_ae_luma_.store(-1, std::memory_order_relaxed);
  this->rotated_exposure_scale_ = this->ae_.exposure_scale();
  this->ae_reassert_pending_ = true;
  this->last_ctrl_us_ = esp_timer_get_time();
}

bool EspVideoCamera::reassert_controls_() {
  bool ok = true;
  if (this->applied_exp_val_ >= 0 && this->applied_exp_cid_ != 0)
    ok = set_ext_ctrl(this->fd_, this->applied_exp_cid_, this->applied_exp_val_) && ok;
  if (this->applied_gain_val_ >= 0 && this->applied_gain_cid_ != 0)
    ok = set_ext_ctrl(this->fd_, this->applied_gain_cid_, this->applied_gain_val_) && ok;
  return ok;
}

void EspVideoCamera::report_luma(uint8_t mean_luma) {
  if (this->auto_exposure_)
    this->pending_ae_luma_.store(static_cast<int32_t>(mean_luma), std::memory_order_relaxed);
}

void EspVideoCamera::step_auto_exposure_() {
  if (!this->auto_exposure_)
    return;
  const int32_t sample = this->pending_ae_luma_.exchange(-1, std::memory_order_relaxed);
  if (sample < 0)
    return;

  const int64_t now_us = esp_timer_get_time();
  const int32_t prev_exp = this->ae_.exposure();
  const int32_t prev_gain = this->ae_.gain();
  const AutoExposureDecision d = this->ae_.update(static_cast<uint8_t>(sample), now_us);
  if (!d.changed)
    return;

  int32_t actual_exp = prev_exp;
  int32_t actual_gain = prev_gain;
  bool any_written = false;
  if (d.exposure_changed && this->applied_exp_cid_ != 0) {
    if (set_ext_ctrl(this->fd_, this->applied_exp_cid_, d.exposure)) {
      actual_exp = d.exposure;
      any_written = true;
    } else {
      ESP_LOGW(TAG, "AE set exposure=%" PRId32 " failed: %s", d.exposure, strerror(errno));
    }
  }
  if (d.gain_changed && this->applied_gain_cid_ != 0) {
    if (set_ext_ctrl(this->fd_, this->applied_gain_cid_, d.gain)) {
      actual_gain = d.gain;
      any_written = true;
    } else {
      ESP_LOGW(TAG, "AE set gain=%" PRId32 " failed: %s", d.gain, strerror(errno));
    }
  }
  if (actual_exp != d.exposure || actual_gain != d.gain) {
    this->ae_.sync_applied(actual_exp, actual_gain, any_written);
  }
  this->applied_exp_val_ = this->ae_.exposure();
  if (this->applied_gain_cid_ != 0)
    this->applied_gain_val_ = this->ae_.gain();
  if (any_written) {
    ESP_LOGD(TAG, "AE luma=%d -> exp=%" PRId32 " gain=%" PRId32 " scale=%.3f",
             (int) sample, this->ae_.exposure(), this->ae_.gain(),
             this->ae_.exposure_scale());
  }
}

void EspVideoCamera::stop() {
  // Immediately mark idle, bump stop epoch, cut RTSP playback, reset streaming
  // AWB seed, and wake every waiter so privacy-off unblocks all consumers.
  this->stream_requested_.store(false);
  this->stop_epoch_.fetch_add(1);
  this->streaming_.store(false);
  this->rtsp_playing_.store(false);
  this->mjpeg_awb_reset_.store(true);
  this->pending_ae_luma_.store(-1, std::memory_order_relaxed);
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
    out.exposure_scale = this->rotated_exposure_scale_;
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

    // Re-assert exposure/gain periodically when fixed; with auto_exposure,
    // reassert active controls once after STREAMON (retrying until ok) so
    // steady-state AE never wastes shared I2C0 bandwidth or clobbers AE state.
    if (esp_timer_get_time() - this->last_ctrl_us_ > 2000000) {
      if (!this->auto_exposure_) {
        this->reassert_controls_();
        this->last_ctrl_us_ = esp_timer_get_time();
      } else if (this->ae_reassert_pending_) {
        if (this->reassert_controls_())
          this->ae_reassert_pending_ = false;
        this->last_ctrl_us_ = esp_timer_get_time();
      }
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
        if (converted_full)
          this->rotated_exposure_scale_ = this->auto_exposure_ ? this->ae_.exposure_scale() : 1.0f;
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
      // Step closed-loop AE after capturing rotated_exposure_scale_ so the
      // current frame's scale matches its exposure and any SCCB write has the
      // full detector interval to settle before the next acquire().
      if (this->streaming_.load() && this->stop_epoch_.load() == start_epoch)
        this->step_auto_exposure_();
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
  out.exposure_scale = this->rotated_exposure_scale_;
  return true;
}

void EspVideoCamera::capture_task_entry_(void *arg) {
  static_cast<EspVideoCamera *>(arg)->capture_task_();
}

void EspVideoCamera::capture_task_() {
  ESP_LOGI(TAG, "camera task running (task_mode=%s)", YESNO(this->task_mode_));
  StopEpochLatch stop_latch{};
  bool camera_on = false;
  uint32_t active_h264_session = 0;
  uint32_t consecutive_dq_misses = 0;

  auto teardown_h264_if_open = [&]() {
    if (this->h264_enc_ != nullptr) {
      this->teardown_h264_();
      active_h264_session = 0;
    }
  };
  auto disconnect_rtsp_session = [&](uint32_t session_gen) {
    if (this->sock_mutex_ == nullptr)
      return;
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    if (this->rtsp_session_gen_.load() == session_gen) {
      this->rtsp_playing_.store(false);
      const int fd = this->rtsp_client_fd_.load();
      if (fd >= 0)
        shutdown(fd, SHUT_RDWR);
    }
    xSemaphoreGive(this->sock_mutex_);
  };

  for (;;) {
    if (!this->task_mode_)
      break;

    const uint32_t observed_epoch = this->stop_epoch_.load();
    const bool wanted = this->stream_requested_.load();
    const StreamLifecycleAction action =
        stop_latch.step(observed_epoch, wanted, camera_on);
    if (action == StreamLifecycleAction::STOP) {
      teardown_h264_if_open();
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
      teardown_h264_if_open();
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
      teardown_h264_if_open();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Lazy H.264 encoder lifecycle (sole owner = capture_task_): open on PLAY
    // so frame 0 is guaranteed to be an IDR with SPS+PPS, and close immediately
    // when idle or when a new RTSP session preempts the old one.
    if (this->h264_ready_.load()) {
      const bool want_h264 = this->rtsp_playing_.load();
      const uint32_t cur_gen = this->rtsp_session_gen_.load();
      if (!want_h264) {
        teardown_h264_if_open();
      } else {
        if (this->h264_enc_ != nullptr && active_h264_session != cur_gen)
          teardown_h264_if_open();
        if (this->h264_enc_ == nullptr) {
          if (this->open_h264_()) {
            active_h264_session = cur_gen;
          } else {
            disconnect_rtsp_session(cur_gen);
          }
        }
      }
    }

    if (esp_timer_get_time() - this->last_ctrl_us_ > 2000000) {
      if (!this->auto_exposure_) {
        this->reassert_controls_();
        this->last_ctrl_us_ = esp_timer_get_time();
      } else if (this->ae_reassert_pending_) {
        if (this->reassert_controls_())
          this->ae_reassert_pending_ = false;
        this->last_ctrl_us_ = esp_timer_get_time();
      }
    }
    this->step_auto_exposure_();

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
      if (full_ready)
        this->rotated_exposure_scale_ = this->auto_exposure_ ? this->ae_.exposure_scale() : 1.0f;
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

    // 3. H.264 PPA pass: centered 16px-aligned 0.5x crop into O_UYY_E_VYY YUV420
    //    while the CSI V4L2 buffer is valid.
    bool do_h264 = false;
    int64_t h264_frame_us = 0;
    if (this->h264_enc_ != nullptr && this->rtsp_playing_.load() &&
        this->rtsp_session_gen_.load() == active_h264_session) {
      const int64_t now = esp_timer_get_time();
      if (this->last_h264_us_ == 0 || now - this->last_h264_us_ >= H264_MIN_FRAME_US) {
        if (this->convert_frame_(raw, this->h264_yuv_, this->h264_yuv_size_,
                                 this->h264_geom_.out_w, this->h264_geom_.out_h,
                                 PPA_SRM_COLOR_MODE_YUV420, 0.5f, 0.5f,
                                 this->h264_geom_.crop_w, this->h264_geom_.crop_h,
                                 this->h264_geom_.crop_x, this->h264_geom_.crop_y)) {
          h264_frame_us = now;
          this->last_h264_us_ = now;
          do_h264 = true;
        }
      }
    }

    // 4. Re-queue the CSI V4L2 buffer immediately before AWB, HW encode, or RTP send.
    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGW(TAG, "camera QBUF failed: %s", strerror(errno));
    } else if (buf.index < 32) {
      this->queued_mask_ |= (1u << buf.index);
    }

    if (this->stop_epoch_.load() != observed_epoch || !this->stream_requested_.load())
      continue;

    // 5. Post-QBUF: snapshot/detector dispatch, MJPEG AWB+JPEG, H.264 AWB+encode+RTP.
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

    if (do_h264 && this->streaming_.load() && this->stop_epoch_.load() == observed_epoch &&
        this->rtsp_playing_.load() && this->rtsp_session_gen_.load() == active_h264_session) {
      this->encode_h264_(observed_epoch, active_h264_session, h264_frame_us);
    }
  }
  teardown_h264_if_open();
  this->capture_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

bool EspVideoCamera::encode_h264_(uint32_t capture_epoch, uint32_t session_gen,
                                  int64_t frame_us) {
#ifdef USE_ESP_VIDEO_CAMERA_H264
  if (this->h264_enc_ == nullptr || this->h264_yuv_ == nullptr || this->h264_output_ == nullptr)
    return false;
  if (!this->streaming_.load() || this->stop_epoch_.load() != capture_epoch ||
      !this->rtsp_playing_.load() || this->rtsp_session_gen_.load() != session_gen) {
    return false;
  }

  // Fast in-place chroma LUT white balance on the H.264 YUV420 copy only
  // (esp_h264_enc_process writes back the input cache before H.264 DMA2D).
  if (this->awb_enabled_) {
    gray_world_yuv420_oyy_inplace(this->h264_yuv_, this->h264_geom_.out_w,
                                  this->h264_geom_.out_h, this->h264_awb_state_);
  }

  if (!this->streaming_.load() || this->stop_epoch_.load() != capture_epoch ||
      !this->rtsp_playing_.load() || this->rtsp_session_gen_.load() != session_gen) {
    return false;
  }

  esp_h264_enc_in_frame_t in_frame = {};
  in_frame.raw_data.buffer = this->h264_yuv_;
  in_frame.raw_data.len = static_cast<uint32_t>(this->h264_yuv_size_);
  esp_h264_enc_out_frame_t out_frame = {};
  out_frame.raw_data.buffer = this->h264_output_;
  out_frame.raw_data.len = static_cast<uint32_t>(this->h264_output_size_);

  auto enc = static_cast<esp_h264_enc_handle_t>(this->h264_enc_);
  const esp_h264_err_t err = esp_h264_enc_process(enc, &in_frame, &out_frame);
  if (err != ESP_H264_ERR_OK || out_frame.length == 0) {
    this->h264_encode_failures_++;
    if (this->h264_encode_failures_ == 1 || this->h264_encode_failures_ % 30 == 0) {
      ESP_LOGW(TAG, "H.264: esp_h264_enc_process failed (%d, len=%u, count=%u)",
               static_cast<int>(err), (unsigned) out_frame.length,
               (unsigned) this->h264_encode_failures_);
    }
    return false;
  }
  this->h264_encode_failures_ = 0;

  // Cache SPS/PPS + profile-level-id once from an IDR access unit for SDP.
  if (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR && this->sock_mutex_ != nullptr) {
    H264SpropParams extracted{};
    if (extract_h264_sps_pps(this->h264_output_, out_frame.length, extracted)) {
      xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
      this->h264_sprop_ = extracted;
      xSemaphoreGive(this->sock_mutex_);
    }
  }

  if (!this->streaming_.load() || this->stop_epoch_.load() != capture_epoch ||
      !this->rtsp_playing_.load() || this->rtsp_session_gen_.load() != session_gen) {
    return false;
  }

  if (this->rtsp_play_start_us_ == 0)
    this->rtsp_play_start_us_ = frame_us;
  const uint32_t rtp_ts = rtp_timestamp_90khz(frame_us - this->rtsp_play_start_us_);
  if (!this->send_h264_rtp_(this->h264_output_, out_frame.length, rtp_ts, capture_epoch,
                            session_gen)) {
    return false;
  }
  this->rtsp_last_media_us_.store(esp_timer_get_time());
  return true;
#else
  (void) capture_epoch;
  (void) session_gen;
  (void) frame_us;
  return false;
#endif
}

bool EspVideoCamera::send_rtsp_(int fd, const char *data, size_t len) {
  if (this->rtsp_send_mutex_ == nullptr)
    return false;
  const int64_t deadline_us = esp_timer_get_time() + RTSP_CTRL_SEND_DEADLINE_US;
  for (;;) {
    const int64_t rem_us = deadline_us - esp_timer_get_time();
    if (rem_us <= 0)
      return false;
    TickType_t wait_ticks = pdMS_TO_TICKS(static_cast<uint32_t>((rem_us + 999) / 1000));
    if (wait_ticks == 0)
      wait_ticks = 1;
    if (xSemaphoreTake(this->rtsp_send_mutex_, wait_ticks) != pdTRUE)
      return false;
    const uint32_t cur_gen = this->rtsp_session_gen_.load();
    if (!this->rtsp_partial_active_ || this->rtsp_partial_session_ != cur_gen)
      break;
    // Wait for an in-flight partial interleaved RTP packet to finish before
    // writing an RTSP control response onto the shared TCP stream.
    const bool still_playing = this->rtsp_playing_.load();
    xSemaphoreGive(this->rtsp_send_mutex_);
    if (!still_playing)
      return false;
    vTaskDelay(1);
  }
  size_t sent = 0;
  while (sent < len) {
    int n = send(fd, data + sent, len - sent, 0);
    if (n < 0 && errno == EINTR)
      continue;
    if (n <= 0) {
      xSemaphoreGive(this->rtsp_send_mutex_);
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  xSemaphoreGive(this->rtsp_send_mutex_);
  return true;
}

bool EspVideoCamera::send_h264_rtp_(const uint8_t *data, size_t len, uint32_t rtp_timestamp,
                                    uint32_t capture_epoch, uint32_t session_gen) {
  if (this->sock_mutex_ == nullptr || this->rtsp_send_mutex_ == nullptr ||
      this->rtp_packet_buf_ == nullptr) {
    return false;
  }
  int fd = -1;
  xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
  if (this->rtsp_session_gen_.load() == session_gen)
    fd = this->rtsp_client_fd_.load();
  xSemaphoreGive(this->sock_mutex_);
  if (fd < 0 || !this->rtsp_playing_.load() || !this->streaming_.load() ||
      this->stop_epoch_.load() != capture_epoch) {
    return false;
  }

  // Single 100 ms total budget for all RTP packets of this access unit so a
  // stalled TCP peer can never stall capture_task_().
  const int64_t deadline_us = esp_timer_get_time() + H264_RTP_FRAME_BUDGET_US;

  auto session_live = [&]() {
    return this->rtsp_session_gen_.load() == session_gen &&
           this->rtsp_client_fd_.load() == fd && this->rtsp_playing_.load() &&
           this->streaming_.load() && this->stop_epoch_.load() == capture_epoch;
  };
  auto take_send_mutex = [&]() -> bool {
    const int64_t rem_us = deadline_us - esp_timer_get_time();
    if (rem_us <= 0)
      return false;
    TickType_t wait_ticks = pdMS_TO_TICKS(static_cast<uint32_t>((rem_us + 999) / 1000));
    if (wait_ticks == 0)
      wait_ticks = 1;
    return xSemaphoreTake(this->rtsp_send_mutex_, wait_ticks) == pdTRUE;
  };

  size_t packets_sent = 0;
  const bool ok = packetize_h264_annex_b_interleaved(
      data, len, rtp_timestamp, this->rtp_sequence_, H264_RTP_MAX_PAYLOAD, 0,
      this->rtp_packet_buf_, H264_RTP_SCRATCH_ALLOC_SIZE,
      [&](const uint8_t *framed, size_t framed_len, bool) -> bool {
        if (!session_live() || !take_send_mutex())
          return false;
        bool mutex_held = true;
        bool packet_ok = true;
        size_t sent = 0;
        while (sent < framed_len) {
          if (esp_timer_get_time() >= deadline_us || !session_live()) {
            packet_ok = false;
            break;
          }
          const int n = send(fd, framed + sent, framed_len - sent, MSG_DONTWAIT);
          if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
          }
          if (n < 0 && errno == EINTR)
            continue;
          if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (esp_timer_get_time() >= deadline_us || !session_live()) {
              packet_ok = false;
              break;
            }
            if (sent > 0) {
              this->rtsp_partial_active_ = true;
              this->rtsp_partial_session_ = session_gen;
            }
            xSemaphoreGive(this->rtsp_send_mutex_);
            mutex_held = false;
            vTaskDelay(1);
            if (!session_live() || esp_timer_get_time() >= deadline_us || !take_send_mutex()) {
              packet_ok = false;
              break;
            }
            mutex_held = true;
            continue;
          }
          packet_ok = false;
          break;
        }
        if (mutex_held) {
          if (packet_ok) {
            if (this->rtsp_partial_active_ && this->rtsp_partial_session_ == session_gen)
              this->rtsp_partial_active_ = false;
          } else if (sent > 0) {
            // Keep the partial-frame guard active until the socket is shut down
            // so a waiting send_rtsp_() cannot append a control reply mid-packet.
            this->rtsp_partial_active_ = true;
            this->rtsp_partial_session_ = session_gen;
          }
          xSemaphoreGive(this->rtsp_send_mutex_);
        }
        if (packet_ok)
          ++packets_sent;
        return packet_ok;
      });
  if (!ok) {
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    if (this->rtsp_client_fd_.load() == fd &&
        this->rtsp_session_gen_.load() == session_gen) {
      this->rtsp_playing_.store(false);
      shutdown(fd, SHUT_RDWR);
    }
    xSemaphoreGive(this->sock_mutex_);
    return false;
  }
  return packets_sent > 0;
}

void EspVideoCamera::rtsp_task_entry_(void *arg) {
  static_cast<EspVideoCamera *>(arg)->rtsp_task_();
}

void EspVideoCamera::rtsp_task_() {
  const int server = this->rtsp_server_fd_;
  if (server < 0) {
    this->rtsp_task_handle_ = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  int client = -1;
  bool setup = false;
  bool client_authed = false;
  std::string pending;
  int64_t last_rx_us = 0;

  auto close_client = [&]() {
    if (client < 0)
      return;
    const int fd = client;
    client = -1;
    setup = false;
    client_authed = false;
    pending.clear();
    this->rtsp_playing_.store(false);
    xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
    this->rtsp_session_gen_.fetch_add(1);
    if (this->rtsp_client_fd_.load() == fd)
      this->rtsp_client_fd_.store(-1);
    shutdown(fd, SHUT_RDWR);
    xSemaphoreGive(this->sock_mutex_);
    // Wait for any in-flight send_h264_rtp_ packet on this fd before close().
    if (this->rtsp_send_mutex_ != nullptr) {
      xSemaphoreTake(this->rtsp_send_mutex_, portMAX_DELAY);
      this->rtsp_partial_active_ = false;
      xSemaphoreGive(this->rtsp_send_mutex_);
    }
    close(fd);
  };

  for (;;) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(server, &rfds);
    int max_fd = server;
    if (client >= 0) {
      FD_SET(client, &rfds);
      if (client > max_fd)
        max_fd = client;
    }
    timeval tv = {.tv_sec = 1, .tv_usec = 0};
    const int ready = ::select(max_fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ready < 0) {
      if (errno == EINTR)
        continue;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Never allow an unauthenticated accept() to evict an authenticated or
    // actively playing viewer; only an empty slot or idle unauthenticated
    // connection may be replaced.
    bool accepted_new = false;
    if (ready > 0 && FD_ISSET(server, &rfds)) {
      int next_client = accept(server, nullptr, nullptr);
      if (next_client >= 0) {
        if (!rtsp_should_accept_replacement(client >= 0, this->rtsp_playing_.load(),
                                            client_authed)) {
          ESP_LOGD(TAG, "RTSP: rejecting connection while authenticated viewer is active");
          close(next_client);
        } else {
          if (client >= 0) {
            ESP_LOGI(TAG, "RTSP: preempting idle unauthenticated client");
            close_client();
          }
          int nodelay = 1;
          setsockopt(next_client, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
          timeval rcv_tv = {.tv_sec = 1, .tv_usec = 0};
          timeval snd_tv = {.tv_sec = 0, .tv_usec = static_cast<suseconds_t>(RTSP_CTRL_SNDTIMEO_US)};
          setsockopt(next_client, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));
          setsockopt(next_client, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));
          xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
          this->rtsp_session_gen_.fetch_add(1);
          this->rtsp_client_fd_.store(next_client);
          xSemaphoreGive(this->sock_mutex_);
          this->rtsp_playing_.store(false);
          client = next_client;
          setup = false;
          client_authed = false;
          pending.clear();
          last_rx_us = esp_timer_get_time();
          accepted_new = true;
        }
      }
    }
    if (accepted_new)
      continue;

    if (client < 0)
      continue;

    const bool playing = this->rtsp_playing_.load();
    if (rtsp_session_timed_out(playing, esp_timer_get_time(), last_rx_us,
                               this->rtsp_last_media_us_.load())) {
      if (playing) {
        ESP_LOGW(TAG, "RTSP: disconnecting stalled playback session (no media for >5 s)");
      } else {
        ESP_LOGD(TAG, "RTSP: disconnecting idle client");
      }
      close_client();
      continue;
    }

    if (ready == 0 || !FD_ISSET(client, &rfds))
      continue;

    char chunk[512];
    const int n = recv(client, chunk, sizeof(chunk), 0);
    if (n == 0) {
      close_client();
      continue;
    }
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        continue;
      close_client();
      continue;
    }
    last_rx_us = esp_timer_get_time();
    pending.append(chunk, static_cast<size_t>(n));
    if (pending.size() > 4096) {
      close_client();
      continue;
    }

    bool disconnect_client = false;
    while (client >= 0 && !disconnect_client) {
      if (!pending.empty() && pending[0] == '$') {
        if (pending.size() < 4)
          break;
        const size_t interleaved_len =
            (static_cast<uint8_t>(pending[2]) << 8) | static_cast<uint8_t>(pending[3]);
        if (pending.size() < interleaved_len + 4)
          break;
        pending.erase(0, interleaved_len + 4);  // ignore client RTCP
        continue;
      }
      const size_t request_end = pending.find("\r\n\r\n");
      if (request_end == std::string::npos)
        break;
      std::string request = pending.substr(0, request_end + 4);
      pending.erase(0, request_end + 4);
      const std::string cseq = rtsp_header(request, "CSeq");
      const std::string auth = rtsp_header(request, "Authorization");
      char response[1024];
      bool play_after_response = false;
      uint32_t play_epoch = 0;

      if (this->h264_auth_.empty() || !constant_time_equals(auth, this->h264_auth_)) {
        snprintf(response, sizeof(response),
                 "RTSP/1.0 401 Unauthorized\r\nCSeq: %s\r\n"
                 "WWW-Authenticate: Basic realm=\"ESP32-P4 Camera\"\r\n\r\n",
                 cseq.c_str());
      } else {
        client_authed = true;
        if (request.rfind("OPTIONS ", 0) == 0) {
          snprintf(response, sizeof(response),
                   "RTSP/1.0 200 OK\r\nCSeq: %s\r\n"
                   "Public: OPTIONS, DESCRIBE, SETUP, PLAY, GET_PARAMETER, TEARDOWN\r\n\r\n",
                   cseq.c_str());
        } else if (request.rfind("DESCRIBE ", 0) == 0) {
          H264SpropParams sprop{};
          xSemaphoreTake(this->sock_mutex_, portMAX_DELAY);
          sprop = this->h264_sprop_;
          xSemaphoreGive(this->sock_mutex_);
          const std::string sdp = format_rtsp_sdp(this->h264_geom_.out_w, this->h264_geom_.out_h,
                                                  H264_FPS, sprop.valid() ? &sprop : nullptr);
          const int head = snprintf(response, sizeof(response),
                                    "RTSP/1.0 200 OK\r\nCSeq: %s\r\nContent-Type: application/sdp\r\n"
                                    "Content-Length: %u\r\n\r\n%s",
                                    cseq.c_str(), static_cast<unsigned>(sdp.size()), sdp.c_str());
          if (head < 0 || static_cast<size_t>(head) >= sizeof(response)) {
            disconnect_client = true;
            break;
          }
        } else if (request.rfind("SETUP ", 0) == 0) {
          const std::string transport = rtsp_header(request, "Transport");
          if (transport.find("RTP/AVP/TCP") == std::string::npos) {
            snprintf(response, sizeof(response),
                     "RTSP/1.0 461 Unsupported Transport\r\nCSeq: %s\r\n\r\n", cseq.c_str());
          } else {
            setup = true;
            snprintf(response, sizeof(response),
                     "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n"
                     "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n",
                     cseq.c_str());
          }
        } else if (request.rfind("PLAY ", 0) == 0 && setup) {
          play_epoch = this->stop_epoch_.load();
          if (this->rtsp_playing_.load()) {
            // Reject duplicate PLAY on an active session so it cannot bump
            // session_gen mid-packet or reset the 5 s media-stall watchdog.
            snprintf(response, sizeof(response),
                     "RTSP/1.0 455 Method Not Valid in This State\r\nCSeq: %s\r\nSession: 1\r\n\r\n",
                     cseq.c_str());
          } else if (!this->stream_requested_.load() || !this->streaming_.load()) {
            snprintf(response, sizeof(response),
                     "RTSP/1.0 503 Service Unavailable\r\nCSeq: %s\r\n\r\n", cseq.c_str());
          } else {
            play_after_response = true;
            snprintf(response, sizeof(response),
                     "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n"
                     "RTP-Info: url=trackID=0;seq=0;rtptime=0\r\n\r\n",
                     cseq.c_str());
          }
        } else if (request.rfind("GET_PARAMETER ", 0) == 0) {
          snprintf(response, sizeof(response),
                   "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n\r\n", cseq.c_str());
        } else if (request.rfind("TEARDOWN ", 0) == 0) {
          this->rtsp_playing_.store(false);
          snprintf(response, sizeof(response),
                   "RTSP/1.0 200 OK\r\nCSeq: %s\r\nSession: 1\r\n\r\n", cseq.c_str());
          this->send_rtsp_(client, response, strlen(response));
          disconnect_client = true;
          break;
        } else {
          snprintf(response, sizeof(response),
                   "RTSP/1.0 405 Method Not Allowed\r\nCSeq: %s\r\n\r\n", cseq.c_str());
        }
      }

      if (!this->send_rtsp_(client, response, strlen(response))) {
        disconnect_client = true;
        break;
      }
      if (play_after_response) {
        if (!this->stream_requested_.load() || !this->streaming_.load() ||
            this->stop_epoch_.load() != play_epoch) {
          disconnect_client = true;
          break;
        }
        this->rtsp_last_media_us_.store(esp_timer_get_time());
        this->rtsp_playing_.store(true);
        if (!this->stream_requested_.load() || !this->streaming_.load() ||
            this->stop_epoch_.load() != play_epoch) {
          this->rtsp_playing_.store(false);
          disconnect_client = true;
          break;
        }
      }
    }
    if (disconnect_client)
      close_client();
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
  ESP_LOGCONFIG(TAG, "  Auto-exposure: %s", this->auto_exposure_ ? "enabled" : "disabled");
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
    ESP_LOGCONFIG(TAG, "  H.264 RTSP: %s, %ux%u @ %u fps, %u kbps, port 8554, Basic auth (lazy)",
                  this->h264_ready_.load() ? "ready" : "unavailable", this->h264_geom_.out_w,
                  this->h264_geom_.out_h, (unsigned) H264_FPS, (unsigned) (H264_BITRATE / 1000u));
  }
  if (this->is_failed())
    ESP_LOGE(TAG, "  Component failed to set up");
}

}  // namespace esp_video_camera
}  // namespace esphome
