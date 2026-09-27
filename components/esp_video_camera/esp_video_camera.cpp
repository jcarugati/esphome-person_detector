#include "esp_video_camera.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cerrno>
#include <cinttypes>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

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
#include "esp_video_init.h"

#include "driver/i2c_master.h"  // i2c_master_get_bus_handle for shared-bus SCCB
#include "driver/jpeg_encode.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"
#include "gray_world.h"

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
  if (this->snapshot_enabled_) {
    this->snapshot_done_ = xSemaphoreCreateBinary();
    web_server_base::global_web_server_base->add_handler(new SnapshotHandler(this));  // NOLINT
  }
#endif
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
      ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%u] failed: %s", i, strerror(errno));
      return false;
    }
    this->buffers_[i].length = buf.length;
    this->buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, this->fd_, buf.m.offset);
    if (this->buffers_[i].start == MAP_FAILED) {
      ESP_LOGE(TAG, "mmap[%u] failed: %s", i, strerror(errno));
      return false;
    }
    // Queue it for capture.
    if (xioctl(this->fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF[%u] failed: %s", i, strerror(errno));
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

bool EspVideoCamera::init() { return this->ready_; }

bool EspVideoCamera::start() {
  if (!this->ready_ || this->streaming_)
    return this->ready_;
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
  if (!this->streaming_)
    return;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(this->fd_, VIDIOC_STREAMOFF, &type);
  this->streaming_ = false;
  ESP_LOGD(TAG, "Streaming stopped");
}

bool EspVideoCamera::acquire(person_detect::FrameView &out, uint32_t timeout_ms) {
  if (!this->streaming_)
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
  ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
  switch (this->rotation_) {
    case 90:
      angle = PPA_SRM_ROTATION_ANGLE_90;
      break;
    case 180:
      angle = PPA_SRM_ROTATION_ANGLE_180;
      break;
    case 270:
      angle = PPA_SRM_ROTATION_ANGLE_270;
      break;
    default:
      angle = PPA_SRM_ROTATION_ANGLE_0;
      break;
  }

  ppa_srm_oper_config_t op = {};
  op.in.buffer = this->buffers_[this->dq_index_].start;
  op.in.pic_w = this->cap_w_;
  op.in.pic_h = this->cap_h_;
  op.in.block_w = this->cap_w_;
  op.in.block_h = this->cap_h_;
  op.in.block_offset_x = 0;
  op.in.block_offset_y = 0;
  op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  op.out.buffer = this->rotated_;
  op.out.buffer_size = this->rotated_size_;
  op.out.pic_w = this->out_w_;
  op.out.pic_h = this->out_h_;
  op.out.block_offset_x = 0;
  op.out.block_offset_y = 0;
  op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB888;
  op.rotation_angle = angle;
  op.scale_x = 1.0f;
  op.scale_y = 1.0f;
  op.rgb_swap = this->swap_rgb_;
  op.byte_swap = false;
  op.mode = PPA_TRANS_MODE_BLOCKING;

  int64_t t0 = esp_timer_get_time();
  esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &op);
  this->last_ppa_us_ = static_cast<uint32_t>(esp_timer_get_time() - t0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PPA rotate/convert failed: %s", esp_err_to_name(err));
    this->release();  // requeue the buffer
    return false;
  }

  if (this->awb_enabled_ && esp_timer_get_time() - this->last_awb_us_ > 2000000) {
    this->last_awb_us_ = esp_timer_get_time();
    this->white_balance_step_();
  }
  if (this->snapshot_requested_.load())
    this->encode_snapshot_();

  out.data = static_cast<const uint8_t *>(this->rotated_);
  out.width = this->out_w_;
  out.height = this->out_h_;
  out.format = person_detect::FRAME_FORMAT_RGB888;
  return true;
}

void EspVideoCamera::white_balance_step_() {
  // PPA RGB888 in memory is B,G,R (ESP-IDF little-endian convention, same as
  // the JPEG encoder input). Sample every 16th pixel.
  const uint8_t *px = static_cast<const uint8_t *>(this->rotated_);
  const size_t n = static_cast<size_t>(this->out_w_) * this->out_h_;
  uint64_t sb = 0, sg = 0, sr = 0;
  for (size_t i = 0; i < n; i += 16) {
    sb += px[i * 3];
    sg += px[i * 3 + 1];
    sr += px[i * 3 + 2];
  }
  if (!gray_world_step(sr, sg, sb, this->red_gain_, this->blue_gain_))
    return;
  if (this->isp_fd_ < 0)
    this->isp_fd_ = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
  if (this->isp_fd_ < 0)
    return;
  esp_video_isp_wb_t wb = {.enable = true, .red_gain = this->red_gain_, .blue_gain = this->blue_gain_};
  struct v4l2_ext_control c = {};
  c.id = V4L2_CID_USER_ESP_ISP_WB;
  c.size = sizeof(wb);
  c.p_u8 = reinterpret_cast<uint8_t *>(&wb);
  struct v4l2_ext_controls cs = {};
  cs.ctrl_class = V4L2_CTRL_CLASS_USER;
  cs.count = 1;
  cs.controls = &c;
  if (xioctl(this->isp_fd_, VIDIOC_S_EXT_CTRLS, &cs) == 0)
    ESP_LOGD(TAG, "AWB mean rgb=%u/%u/%u -> gains r=%.2f b=%.2f", (unsigned) (sr * 16 / n),
             (unsigned) (sg * 16 / n), (unsigned) (sb * 16 / n), this->red_gain_, this->blue_gain_);
}

bool EspVideoCamera::capture_jpeg(const uint8_t *&data, size_t &len, uint32_t timeout_ms) {
  if (this->snapshot_done_ == nullptr || !this->streaming_)
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
  if (this->jpeg_encoder_ != nullptr && this->jpeg_buf_ != nullptr) {
    jpeg_encode_cfg_t cfg = {};
    cfg.width = this->out_w_;
    cfg.height = this->out_h_;
    cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
    cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
    cfg.image_quality = 70;
    uint32_t out = 0;
    esp_err_t err = jpeg_encoder_process(static_cast<jpeg_encoder_handle_t>(this->jpeg_encoder_), &cfg,
                                         static_cast<const uint8_t *>(this->rotated_),
                                         (uint32_t) this->out_w_ * this->out_h_ * 3, this->jpeg_buf_,
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
  if (this->last_ppa_us_ != 0)
    ESP_LOGCONFIG(TAG, "  Last PPA rotate: %u us", (unsigned) this->last_ppa_us_);
  ESP_LOGCONFIG(TAG, "  Capture failures: %u", (unsigned) this->capture_failures_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Component failed to set up");
}

}  // namespace esp_video_camera
}  // namespace esphome
