#include "esp_video_camera.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <array>
#include <cerrno>
#include <cinttypes>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
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

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

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

#ifdef USE_WEBSERVER
// GET /snapshot.jpg, behind the web server's own auth (add_handler wraps it).
class SnapshotHandler : public AsyncWebHandler {
 public:
  explicit SnapshotHandler(EspVideoCamera *cam) : cam_(cam) {}
  bool canHandle(AsyncWebServerRequest *request) const override {
    char buf[AsyncWebServerRequest::URL_BUF_SIZE];
    return request->method() == HTTP_GET && request->url_to(buf) == "/snapshot.jpg";
  }
  void handleRequest(AsyncWebServerRequest *request) override {
    const uint8_t *data = nullptr;
    size_t len = 0;
    if (!this->cam_->capture_jpeg(data, len, 4000)) {
      request->send(503, "text/plain", "camera idle or busy");
      return;
    }
    auto *rsp = request->beginResponse(200, "image/jpeg", data, len);
    rsp->addHeader("Cache-Control", "no-store");
    request->send(rsp);
  }

 protected:
  EspVideoCamera *cam_;
};

// GET /stream.mjpg, behind the same auth wrapper as /snapshot.jpg. This runs
// in ESP-IDF's HTTP task, not ESPHome's main/LVGL loop, and exits on timeout or
// the first failed socket write. Frames come from the camera task when enabled.
class MjpegHandler : public AsyncWebHandler {
 public:
  explicit MjpegHandler(EspVideoCamera *cam) : cam_(cam) {}
  bool canHandle(AsyncWebServerRequest *request) const override {
    char buf[AsyncWebServerRequest::URL_BUF_SIZE];
    return request->method() == HTTP_GET && request->url_to(buf) == "/stream.mjpg";
  }
  void handleRequest(AsyncWebServerRequest *request) override {
    auto *raw = static_cast<httpd_req_t *>(*request);
    if (httpd_resp_set_type(raw, MJPEG_CONTENT_TYPE) != ESP_OK)
      return;
    httpd_resp_set_hdr(raw, "Cache-Control", "no-store");

    int64_t next_us = 0;
    while (true) {
      // ponytail: fixed ~10 fps cap; keeps HW JPEG + gray-world off the UI's back.
      const int64_t wait_us = next_us - esp_timer_get_time();
      if (wait_us > 0)
        vTaskDelay(pdMS_TO_TICKS(wait_us / 1000 + 1));
      next_us = esp_timer_get_time() + MJPEG_MIN_FRAME_US;
      const uint8_t *data = nullptr;
      size_t len = 0;
      if (!this->cam_->capture_jpeg(data, len, 4000))
        break;
      char part[96];
      int part_len = format_mjpeg_part(part, sizeof(part), len);
      if (part_len <= 0 || static_cast<size_t>(part_len) >= sizeof(part) ||
          httpd_resp_send_chunk(raw, part, part_len) != ESP_OK ||
          httpd_resp_send_chunk(raw, reinterpret_cast<const char *>(data), len) != ESP_OK ||
          httpd_resp_send_chunk(raw, "\r\n", 2) != ESP_OK)
        break;
    }
    httpd_resp_send_chunk(raw, nullptr, 0);
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

  this->ready_ = true;
#ifdef USE_WEBSERVER
  if (this->snapshot_enabled_ || this->mjpeg_stream_enabled_)
    this->snapshot_done_ = xSemaphoreCreateBinary();
  if (this->snapshot_enabled_)
    web_server_base::global_web_server_base->add_handler(new SnapshotHandler(this));  // NOLINT
  if (this->mjpeg_stream_enabled_)
    web_server_base::global_web_server_base->add_handler(new MjpegHandler(this));  // NOLINT
#endif
  // A dedicated camera task owns V4L2 whenever a live stream is enabled, so the
  // stream gets fresh frames instead of the detector's interval.
  if (this->h264_enabled_ || this->mjpeg_stream_enabled_) {
    this->frame_done_ = xSemaphoreCreateBinary();
    // Set before the task starts: it may run immediately on the other core and
    // exits if it sees task_mode_ false.
    this->task_mode_ = this->frame_done_ != nullptr;
    if (this->task_mode_)
      this->task_mode_ = xTaskCreatePinnedToCore(&EspVideoCamera::capture_task_entry_, "camera_stream",
                                                 8192, this, 1, &this->capture_task_handle_, 1) == pdPASS;
    if (!this->task_mode_)
      ESP_LOGW(TAG, "camera task unavailable; streams follow the detector interval");
  }
  // H.264 is brought up later by the camera task (see start_h264_).
  ESP_LOGCONFIG(TAG, "esp_video_camera ready: capture %ux%u -> rotate %u -> "
                     "RGB888 %ux%u",
                this->cap_w_, this->cap_h_, this->rotation_, this->out_w_,
                this->out_h_);
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

bool EspVideoCamera::open_and_configure_() {
  // Non-blocking so acquire() can bound its wait with a DQBUF poll loop.
  // esp_video's V4L2 devices don't hook ESP-IDF's VFS select()/poll(), so a
  // select()-gated DQBUF never wakes (the canonical capture_stream example uses
  // a plain blocking DQBUF for exactly this reason).
  this->fd_ = open(VIDEO_DEVICE, O_RDWR | O_NONBLOCK);
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
    // Queue it for capture.
    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF[%u] failed: %s", (unsigned) i, strerror(errno));
      return false;
    }
  }
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

bool EspVideoCamera::setup_h264_() {
  this->h264_w_ = this->out_w_ / 2;
  this->h264_h_ = this->out_h_ / 2;
  this->h264_w_ &= ~1U;
  this->h264_h_ &= ~1U;
  this->h264_yuv_size_ = static_cast<size_t>(this->h264_w_) * this->h264_h_ * 3 / 2;
  this->h264_yuv_ = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(128, this->h264_yuv_size_, MALLOC_CAP_SPIRAM));
  if (this->h264_yuv_ == nullptr)
    return false;

  this->h264_fd_ = open(ESP_VIDEO_H264_DEVICE_NAME, O_RDWR | O_NONBLOCK);
  if (this->h264_fd_ < 0) {
    ESP_LOGW(TAG, "open(%s) failed: %s", ESP_VIDEO_H264_DEVICE_NAME, strerror(errno));
    return false;
  }

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = this->h264_w_;
  fmt.fmt.pix.height = this->h264_h_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
  if (xioctl(this->h264_fd_, VIDIOC_S_FMT, &fmt) != 0)
    return false;

  struct v4l2_requestbuffers req = {};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  req.memory = V4L2_MEMORY_USERPTR;
  if (xioctl(this->h264_fd_, VIDIOC_REQBUFS, &req) != 0)
    return false;

  fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = this->h264_w_;
  fmt.fmt.pix.height = this->h264_h_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
  fmt.fmt.pix.sizeimage = static_cast<uint32_t>(this->h264_w_) * this->h264_h_;
  if (xioctl(this->h264_fd_, VIDIOC_S_FMT, &fmt) != 0)
    return false;

  req = {};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(this->h264_fd_, VIDIOC_REQBUFS, &req) != 0)
    return false;

  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if (xioctl(this->h264_fd_, VIDIOC_QUERYBUF, &buf) != 0)
    return false;
  this->h264_output_size_ = buf.length;
  this->h264_output_ = static_cast<uint8_t *>(mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                                    MAP_SHARED, this->h264_fd_, buf.m.offset));
  if (this->h264_output_ == MAP_FAILED)
    return false;
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &buf) != 0)
    return false;

  set_ext_ctrl(this->h264_fd_, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 15);
  set_ext_ctrl(this->h264_fd_, V4L2_CID_MPEG_VIDEO_BITRATE, 1000000);
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(this->h264_fd_, VIDIOC_STREAMON, &type) != 0)
    return false;
  type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  if (xioctl(this->h264_fd_, VIDIOC_STREAMON, &type) != 0)
    return false;

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
  if (this->streaming_.load())
    return true;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(this->fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "VIDIOC_STREAMON failed: %s", strerror(errno));
    return false;
  }
  this->streaming_ = true;
  ESP_LOGD(TAG, "Streaming started");
  // Controls take effect once the sensor is streaming.
  this->apply_sensor_controls_();
  return true;
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
  if (this->task_mode_) {
    this->stream_requested_.store(false);
    this->frame_requested_.store(false);
    return;
  }
  if (!this->streaming_.load())
    return;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(this->fd_, VIDIOC_STREAMOFF, &type);
  this->streaming_ = false;
  ESP_LOGD(TAG, "Streaming stopped");
}

bool EspVideoCamera::acquire(person_detect::FrameView &out, uint32_t timeout_ms) {
  if (this->task_mode_) {
    if (!this->stream_requested_.load())
      return false;
    xSemaphoreTake(this->frame_done_, 0);
    this->frame_requested_.store(true);
    if (xSemaphoreTake(this->frame_done_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
      this->frame_requested_.store(false);
      this->capture_failures_++;
      return false;
    }
    out.data = static_cast<const uint8_t *>(this->rotated_);
    out.width = this->out_w_;
    out.height = this->out_h_;
    out.format = person_detect::FRAME_FORMAT_RGB888;
    return true;
  }
  if (!this->streaming_.load())
    return false;

  // Re-assert exposure/gain periodically: a set right at STREAMON occasionally
  // doesn't take (frame stays near-black) and some ISP paths drift it, so this
  // self-corrects within a couple seconds regardless.
  if (esp_timer_get_time() - this->last_ctrl_us_ > 2000000) {
    this->reassert_controls_();
    this->last_ctrl_us_ = esp_timer_get_time();
  }

  // Drop already-captured frames so inference runs on a LIVE one. We grab only
  // once per interval, but V4L2's DQBUF is FIFO, so without this it hands back
  // the oldest queued buffer — a frame captured a couple of cycles ago. Requeue
  // any ready buffer, then wait below for the sensor to fill a fresh one.
  //
  // BOUND this to the ring size: esp_video can hand a buffer straight back as
  // fast as we requeue it (frames queued internally), so an unbounded
  // "while (DQBUF succeeds)" spins forever and hangs the inference task. There
  // are at most fb_count_ buffers to drain.
  for (uint8_t i = 0; i < this->fb_count_; i++) {
    struct v4l2_buffer stale = {};
    stale.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stale.memory = V4L2_MEMORY_MMAP;
    if (xioctl(this->fd_, VIDIOC_DQBUF, &stale) != 0)
      break;  // nothing (more) ready
    xioctl(this->fd_, VIDIOC_QBUF, &stale);
  }

  // The fd is non-blocking (esp_video doesn't support select/poll), so poll
  // DQBUF until a filled buffer is ready or the timeout elapses. At 30 fps a
  // frame lands within ~33 ms; the short vTaskDelay yields to other tasks.
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  const int64_t deadline_us = esp_timer_get_time() + (int64_t) timeout_ms * 1000;
  int rc;
  while ((rc = xioctl(this->fd_, VIDIOC_DQBUF, &buf)) != 0 && errno == EAGAIN) {
    if (esp_timer_get_time() >= deadline_us) {
      this->capture_failures_++;
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (rc != 0) {
    ESP_LOGW(TAG, "VIDIOC_DQBUF failed: %s", strerror(errno));
    this->capture_failures_++;
    return false;
  }
  this->dq_index_ = buf.index;

  // Rotate (portrait mount) + RGB565->RGB888 in one PPA pass.
  if (!this->convert_frame_(this->buffers_[this->dq_index_].start, this->rotated_,
                            this->rotated_size_, this->out_w_, this->out_h_,
                            PPA_SRM_COLOR_MODE_RGB888, 1.0f, 1.0f)) {
    this->release();
    return false;
  }

  if (this->snapshot_requested_.load())
    this->encode_snapshot_();

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
  bool camera_on = false;
  bool h264_tried = false;
  for (;;) {
    if (!this->task_mode_)
      break;
    // ponytail: fixed 30 s boot delay; lets ESPHome mark the app valid first.
    if (this->h264_enabled_ && !h264_tried && esp_timer_get_time() > 30000000) {
      h264_tried = true;
      this->start_h264_();
    }
    const bool wanted = this->stream_requested_.load();
    if (wanted && !camera_on) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(this->fd_, VIDIOC_STREAMON, &type) == 0) {
        camera_on = true;
        this->streaming_.store(true);
        this->apply_sensor_controls_();
      } else {
        ESP_LOGW(TAG, "camera STREAMON failed: %s", strerror(errno));
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
      }
    } else if (!wanted && camera_on) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(this->fd_, VIDIOC_STREAMOFF, &type);
      camera_on = false;
      this->streaming_.store(false);
    }
    if (!camera_on) {
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
    if (xioctl(this->fd_, VIDIOC_DQBUF, &buf) != 0) {
      if (errno != EAGAIN)
        ESP_LOGW(TAG, "camera DQBUF failed: %s", strerror(errno));
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    this->task_frames_++;
    void *raw = this->buffers_[buf.index].start;
    const bool need_rgb = !this->frame_in_use_.load() &&
                          (this->frame_requested_.load() || this->snapshot_requested_.load());
    if (need_rgb && this->convert_frame_(raw, this->rotated_, this->rotated_size_,
                                         this->out_w_, this->out_h_,
                                         PPA_SRM_COLOR_MODE_RGB888, 1.0f, 1.0f)) {
      if (this->frame_requested_.exchange(false)) {
        this->frame_in_use_.store(true);
        xSemaphoreGive(this->frame_done_);
      }
      if (this->snapshot_requested_.load())
        this->encode_snapshot_();
    }

    const int64_t now = esp_timer_get_time();
    if (this->rtsp_playing_.load() && now - this->last_h264_us_ >= 100000 &&
        this->convert_frame_(raw, this->h264_yuv_, this->h264_yuv_size_,
                             this->h264_w_, this->h264_h_, PPA_SRM_COLOR_MODE_YUV420,
                             0.5f, 0.5f)) {
      this->last_h264_us_ = now;
      this->encode_h264_();
    }

    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0)
      ESP_LOGW(TAG, "camera QBUF failed: %s", strerror(errno));
  }
  this->capture_task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

bool EspVideoCamera::encode_h264_() {
  struct v4l2_buffer out = {};
  out.index = 0;
  out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out.memory = V4L2_MEMORY_USERPTR;
  out.m.userptr = reinterpret_cast<unsigned long>(this->h264_yuv_);
  out.length = this->h264_yuv_size_;
  out.bytesused = this->h264_yuv_size_;
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &out) != 0)
    return false;

  struct v4l2_buffer encoded = {};
  encoded.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  encoded.memory = V4L2_MEMORY_MMAP;
  const int64_t deadline = esp_timer_get_time() + 200000;
  while (xioctl(this->h264_fd_, VIDIOC_DQBUF, &encoded) != 0) {
    if (errno != EAGAIN || esp_timer_get_time() >= deadline)
      return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  while (xioctl(this->h264_fd_, VIDIOC_DQBUF, &out) != 0) {
    if (errno != EAGAIN || esp_timer_get_time() >= deadline)
      break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (encoded.bytesused != 0)
    this->send_h264_rtp_(this->h264_output_, encoded.bytesused);
  if (xioctl(this->h264_fd_, VIDIOC_QBUF, &encoded) != 0)
    return false;
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

void EspVideoCamera::send_h264_rtp_(const uint8_t *data, size_t len) {
  const int fd = this->rtsp_client_fd_.load();
  if (fd < 0 || !this->rtsp_playing_.load())
    return;

  bool ok = true;
  packetize_h264_annex_b(data, len, this->rtp_timestamp_, this->rtp_sequence_, 1400,
      [&](const uint8_t *packet, size_t packet_len, bool) {
        if (!ok)
          return;
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
        int n = send(fd, framed.data(), packet_len + 4, MSG_DONTWAIT);
        xSemaphoreGive(this->rtsp_send_mutex_);
        ok = n == static_cast<int>(packet_len + 4);
      });
  this->rtp_timestamp_ += 9000;
  if (!ok) {
    this->rtsp_playing_.store(false);
    shutdown(fd, SHUT_RDWR);
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
    this->rtsp_client_fd_.store(client);
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
        if (play_after_response)
          this->rtsp_playing_.store(true);
      }
    }
client_done:
    this->rtsp_playing_.store(false);
    this->rtsp_client_fd_.store(-1);
    shutdown(client, SHUT_RDWR);
    close(client);
  }
}

bool EspVideoCamera::capture_jpeg(const uint8_t *&data, size_t &len, uint32_t timeout_ms) {
  if (this->snapshot_done_ == nullptr || !this->streaming_.load())
    return false;
  xSemaphoreTake(this->snapshot_done_, 0);  // drop a stale completion
  this->snapshot_requested_.store(true);
  if (xSemaphoreTake(this->snapshot_done_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    this->snapshot_requested_.store(false);
    return false;
  }
  if (this->jpeg_len_ == 0)
    return false;
  data = this->jpeg_buf_;
  len = this->jpeg_len_;
  return true;
}

void EspVideoCamera::encode_snapshot_() {
  this->jpeg_len_ = 0;
  if (this->jpeg_encoder_ == nullptr) {
    jpeg_encode_engine_cfg_t eng = {};
    eng.timeout_ms = 500;
    jpeg_encoder_handle_t h = nullptr;
    if (jpeg_new_encoder_engine(&eng, &h) != ESP_OK) {
      ESP_LOGW(TAG, "JPEG encoder init failed");
    } else {
      this->jpeg_encoder_ = h;
      jpeg_encode_memory_alloc_cfg_t mem = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
      // ponytail: fixed 1/4 of raw RGB888; ample for q70 indoor frames.
      this->jpeg_buf_ = static_cast<uint8_t *>(
          jpeg_alloc_encoder_mem(this->rotated_size_ / 4, &mem, &this->jpeg_buf_size_));
    }
  }
  const uint8_t *src = static_cast<const uint8_t *>(this->rotated_);
  if (this->awb_enabled_) {
    if (this->balanced_ == nullptr)
      this->balanced_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, this->rotated_size_, MALLOC_CAP_SPIRAM));
    if (this->balanced_ != nullptr) {
      gray_world_copy(src, this->balanced_, static_cast<size_t>(this->out_w_) * this->out_h_);
      src = this->balanced_;
    }
  }
  if (this->jpeg_encoder_ != nullptr && this->jpeg_buf_ != nullptr) {
    jpeg_encode_cfg_t cfg = {};
    cfg.width = this->out_w_;
    cfg.height = this->out_h_;
    cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
    cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
    cfg.image_quality = 70;
    uint32_t out = 0;
    esp_err_t err = jpeg_encoder_process(static_cast<jpeg_encoder_handle_t>(this->jpeg_encoder_), &cfg,
                                         src, (uint32_t) this->out_w_ * this->out_h_ * 3, this->jpeg_buf_,
                                         this->jpeg_buf_size_, &out);
    if (err == ESP_OK)
      this->jpeg_len_ = out;
    else
      ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
  }
  this->snapshot_requested_.store(false);
  xSemaphoreGive(this->snapshot_done_);
}

void EspVideoCamera::release() {
  if (this->task_mode_) {
    this->frame_in_use_.store(false);
    return;
  }
  if (this->dq_index_ < 0)
    return;
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = this->dq_index_;
  xioctl(this->fd_, VIDIOC_QBUF, &buf);
  this->dq_index_ = -1;
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
