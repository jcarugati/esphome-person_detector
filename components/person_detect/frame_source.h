#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace person_detect {

// Pixel layout of a FrameView. Kept ESP-DL-free so backends don't need ESP-DL
// headers; the detector maps this to dl::image::pix_type_t in one place.
enum FrameFormat : int {
  FRAME_FORMAT_RGB888 = 0,  // 24-bit, R,G,B byte order
  FRAME_FORMAT_RGB565 = 1,  // 16-bit little-endian
  FRAME_FORMAT_GRAYSCALE = 2,
};

// A single frame handed to the detector, valid only until FrameSource::release().
struct FrameView {
  const uint8_t *data{nullptr};
  uint16_t width{0};
  uint16_t height{0};
  FrameFormat format{FRAME_FORMAT_RGB888};
  // Relative exposure*gain ratio anchored at 1.0f for fixed exposure (or AE ceiling).
  // NaN indicates the sensor exposure/gain is settling or unknown (skip illuminance/AE).
  float exposure_scale{1.0f};
  // Stream session epoch matching the camera's stop_epoch_ when this frame was captured.
  uint32_t capture_epoch{0};
};

// Normalize mean luma (0..255) by exposure_scale into relative ambient light [0, 100%].
// Falls back to 1.0f if exposure_scale is non-finite or non-positive.
inline float normalize_illuminance_pct(uint8_t mean_luma, float exposure_scale) {
  const float safe_scale =
      (std::isfinite(exposure_scale) && exposure_scale > 0.0f) ? exposure_scale : 1.0f;
  const float pct = (static_cast<float>(mean_luma) / safe_scale) * 100.0f / 255.0f;
  if (!std::isfinite(pct) || pct <= 0.0f)
    return 0.0f;
  return pct > 100.0f ? 100.0f : pct;
}

// Abstraction over "where frames come from". v1 ships EsphomeCameraSource, which
// consumes an ESPHome camera by ID and owns no hardware. A direct esp_video /
// esp_cam_ctlr backend (owning the sensor) can implement this same interface
// later for boards ESPHome's camera framework does not yet support — see
// DESIGN.md §2 / §7. The detector only ever talks to this interface.
class FrameSource {
 public:
  virtual ~FrameSource() = default;

  // One-time wiring (register listeners, allocate sync primitives). Returns
  // false if the source cannot be used (e.g. missing camera).
  virtual bool init() = 0;

  // Begin / end frame delivery. stop() must fully idle the camera so the device
  // does no capture work while detection is disabled (privacy).
  virtual bool start() = 0;
  virtual void stop() = 0;

  // Block up to timeout_ms for the next frame. On success fills `out` with a
  // buffer valid until release() and returns true; returns false on timeout.
  virtual bool acquire(FrameView &out, uint32_t timeout_ms) = 0;

  // Release the frame returned by the last successful acquire(). Exactly one
  // frame may be in flight at a time.
  virtual void release() = 0;

  // Optional feedback hook: report whole-frame mean luma (0..255) and capture
  // session epoch from the detector's raw frame for closed-loop auto-exposure
  // backends. Default no-op.
  virtual void report_luma(uint8_t /*mean_luma*/, uint32_t /*capture_epoch*/) {}
};

}  // namespace person_detect
}  // namespace esphome
