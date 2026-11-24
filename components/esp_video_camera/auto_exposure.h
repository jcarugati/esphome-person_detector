#pragma once

// Pure, ESP-IDF-free slow closed-loop auto-exposure controller for SC202CS.
//
// Controls sensor exposure (linear integration lines) and gain (SC202CS V4L2
// enumeration index 0..191) along a single monotonic total-exposure axis
// E = exposure * gain_factor(gain), anchored at exposure_scale() == 1.0f at
// the configured ceiling (max_exposure, max_gain).
//
// In bright scenes, gain drops first (less noise) before exposure decreases;
// in dark scenes, exposure rises first before gain increases. Updates are
// rate-limited and skip a settle frame after every applied write to keep the
// shared I2C0/SCCB bus quiet and avoid sensor-pipeline oscillation.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace esp_video_camera {

// SC202CS physical gain multiplier for a V4L2_CID_GAIN enumeration index.
// Matches espressif/esp_cam_sensor sc202cs_gain_map (ANA_GAIN_PRIORITY):
// 6 octaves x 32 fine steps (indices 0..191 -> 1.0x..63.0x). Within each
// octave, dgain_fine steps linearly by 1/32 from 1.0x (0x80) to 63/32x (0xfc),
// and each octave doubles coarse analog/digital gain (2^octave).
inline float sc202cs_gain_factor(int32_t gain_index) {
  if (gain_index <= 0)
    return 1.0f;
  int32_t octave = gain_index / 32;
  int32_t fine = gain_index % 32;
  if (octave > 10) {
    octave = 10;
    fine = 31;
  }
  return std::ldexp(1.0f + static_cast<float>(fine) / 32.0f, octave);
}

// Inverse of sc202cs_gain_factor(), clamped to [min_gain, max_gain].
inline int32_t sc202cs_gain_index_for_factor(float factor, int32_t min_gain, int32_t max_gain) {
  if (min_gain > max_gain)
    min_gain = max_gain;
  if (!std::isfinite(factor) || factor <= 1.0f)
    return min_gain;
  const float min_f = sc202cs_gain_factor(min_gain);
  if (factor <= min_f)
    return min_gain;
  const float max_f = sc202cs_gain_factor(max_gain);
  if (factor >= max_f)
    return max_gain;

  int exp2 = 0;
  const float mant = std::frexp(factor, &exp2);  // factor = mant * 2^exp2, mant in [0.5, 1.0)
  int32_t octave = exp2 - 1;
  float ratio = mant * 2.0f;  // in [1.0, 2.0)
  if (octave < 0) {
    octave = 0;
    ratio = 1.0f;
  }
  const int32_t fine = static_cast<int32_t>(std::lround((ratio - 1.0f) * 32.0f));
  const int32_t idx = octave * 32 + fine;  // fine == 32 carries cleanly into next octave
  return std::clamp(idx, min_gain, max_gain);
}

// Exposure-normalized ambient light percentage in [0, 100]. Falls back to
// exposure_scale == 1.0f if scale is non-finite or non-positive.
inline float normalize_illuminance_pct(uint8_t mean_luma, float exposure_scale) {
  const float safe_scale =
      (std::isfinite(exposure_scale) && exposure_scale > 0.0f) ? exposure_scale : 1.0f;
  const float pct = (static_cast<float>(mean_luma) / safe_scale) * 100.0f / 255.0f;
  if (!std::isfinite(pct) || pct <= 0.0f)
    return 0.0f;
  return pct > 100.0f ? 100.0f : pct;
}

struct AutoExposureConfig {
  uint8_t target_luma{115};          // ~45% full scale (matches Nocturne light threshold)
  uint8_t deadband{18};              // hold steady for mean_luma in [97..133]
  int64_t min_interval_us{3000000};  // >= 3 s between SCCB writes to protect shared I2C0
  uint8_t settle_frames{1};          // skip 1 report after any write so sensor pipeline settles
  float damping{0.6f};               // damped proportional exponent in (0, 1] (no overshoot)
  float max_step_ratio{2.0f};        // clamp each adjustment to [0.5x, 2.0x] (+-1 EV)
};

struct AutoExposureDecision {
  bool changed{false};
  bool exposure_changed{false};
  bool gain_changed{false};
  int32_t exposure{0};
  int32_t gain{0};
};

class AutoExposureController {
 public:
  explicit AutoExposureController(AutoExposureConfig cfg = {}) : cfg_(cfg) {
    this->configure(8, 1244, 0, 191);
  }

  void configure(int32_t min_exp, int32_t max_exp, int32_t min_gain, int32_t max_gain) {
    this->min_exp_ = min_exp < 1 ? 1 : min_exp;
    this->max_exp_ = max_exp < this->min_exp_ ? this->min_exp_ : max_exp;
    this->min_gain_ = min_gain < 0 ? 0 : min_gain;
    this->max_gain_ = max_gain < this->min_gain_ ? this->min_gain_ : max_gain;
    this->ref_product_ =
        static_cast<float>(this->max_exp_) * sc202cs_gain_factor(this->max_gain_);
    this->reset();
  }

  void reset() {
    this->exposure_ = this->max_exp_;
    this->gain_ = this->max_gain_;
    this->has_updated_ = false;
    this->last_update_us_ = 0;
    this->settle_remaining_ = 0;
  }

  AutoExposureDecision update(uint8_t mean_luma, int64_t now_us) {
    AutoExposureDecision decision{};
    decision.exposure = this->exposure_;
    decision.gain = this->gain_;

    // Skip settle frame(s) after any applied SCCB write so sensor pipeline
    // latency cannot trigger oscillation.
    if (this->settle_remaining_ > 0) {
      this->settle_remaining_--;
      return decision;
    }

    // Rate-limit SCCB updates and guard against backwards/wrapped timestamps.
    if (this->has_updated_) {
      if (now_us < this->last_update_us_) {
        this->last_update_us_ = now_us;
        return decision;
      }
      if (now_us - this->last_update_us_ < this->cfg_.min_interval_us)
        return decision;
    }

    const int32_t target = static_cast<int32_t>(this->cfg_.target_luma);
    const int32_t deadband = static_cast<int32_t>(this->cfg_.deadband);
    const int32_t luma = static_cast<int32_t>(mean_luma);
    const int32_t lo = target - deadband;
    const int32_t hi = target + deadband;
    if (luma >= lo && luma <= hi)
      return decision;

    const bool too_bright = luma > hi;
    if (too_bright && this->exposure_ <= this->min_exp_ && this->gain_ <= this->min_gain_)
      return decision;
    if (!too_bright && this->exposure_ >= this->max_exp_ && this->gain_ >= this->max_gain_)
      return decision;

    const float effective_luma = mean_luma > 0 ? static_cast<float>(mean_luma) : 1.0f;
    const float raw_ratio = static_cast<float>(target > 0 ? target : 1) / effective_luma;
    const float damping = std::clamp(this->cfg_.damping, 0.1f, 1.0f);
    const float max_step = this->cfg_.max_step_ratio > 1.0f ? this->cfg_.max_step_ratio : 2.0f;
    const float step_ratio =
        std::clamp(std::pow(raw_ratio, damping), 1.0f / max_step, max_step);

    const float cur_product =
        static_cast<float>(this->exposure_) * sc202cs_gain_factor(this->gain_);
    const float desired_product = cur_product * step_ratio;
    const float min_gain_f = sc202cs_gain_factor(this->min_gain_);
    const float knee_product = static_cast<float>(this->max_exp_) * min_gain_f;

    int32_t next_exp = this->exposure_;
    int32_t next_gain = this->gain_;
    if (desired_product >= knee_product) {
      next_exp = this->max_exp_;
      const float desired_gain_f = desired_product / static_cast<float>(this->max_exp_);
      next_gain = sc202cs_gain_index_for_factor(desired_gain_f, this->min_gain_, this->max_gain_);
    } else {
      next_gain = this->min_gain_;
      const float desired_exp_f = desired_product / min_gain_f;
      next_exp = std::clamp(static_cast<int32_t>(std::lround(desired_exp_f)),
                            this->min_exp_, this->max_exp_);
    }

    // Avoid stalling just outside the deadband due to integer quantization.
    if (next_exp == this->exposure_ && next_gain == this->gain_) {
      if (too_bright) {
        if (next_gain > this->min_gain_)
          next_gain--;
        else if (next_exp > this->min_exp_)
          next_exp--;
      } else {
        if (next_exp < this->max_exp_)
          next_exp++;
        else if (next_gain < this->max_gain_)
          next_gain++;
      }
    }

    if (next_exp == this->exposure_ && next_gain == this->gain_)
      return decision;

    decision.changed = true;
    decision.exposure_changed = (next_exp != this->exposure_);
    decision.gain_changed = (next_gain != this->gain_);
    decision.exposure = next_exp;
    decision.gain = next_gain;

    this->exposure_ = next_exp;
    this->gain_ = next_gain;
    this->has_updated_ = true;
    this->last_update_us_ = now_us;
    this->settle_remaining_ = this->cfg_.settle_frames;
    return decision;
  }

  // Reconcile state after partial or complete V4L2 ioctl failure so
  // exposure_scale() never desyncs from the sensor's actual registers.
  // Keeps the rate-limit timestamp so a failing I2C bus is not spammed.
  void sync_applied(int32_t actual_exp, int32_t actual_gain, bool hardware_changed) {
    this->exposure_ = std::clamp(actual_exp, this->min_exp_, this->max_exp_);
    this->gain_ = std::clamp(actual_gain, this->min_gain_, this->max_gain_);
    if (!hardware_changed)
      this->settle_remaining_ = 0;
  }

  float exposure_scale() const {
    if (this->exposure_ == this->max_exp_ && this->gain_ == this->max_gain_)
      return 1.0f;
    if (!std::isfinite(this->ref_product_) || this->ref_product_ <= 0.0f)
      return 1.0f;
    const float cur =
        static_cast<float>(this->exposure_) * sc202cs_gain_factor(this->gain_);
    const float scale = cur / this->ref_product_;
    if (!std::isfinite(scale) || scale <= 0.0f)
      return 1.0f;
    return scale;
  }

  int32_t exposure() const { return this->exposure_; }
  int32_t gain() const { return this->gain_; }
  int32_t min_exposure() const { return this->min_exp_; }
  int32_t max_exposure() const { return this->max_exp_; }
  int32_t min_gain() const { return this->min_gain_; }
  int32_t max_gain() const { return this->max_gain_; }

 private:
  AutoExposureConfig cfg_{};
  int32_t min_exp_{8};
  int32_t max_exp_{1244};
  int32_t min_gain_{0};
  int32_t max_gain_{191};
  int32_t exposure_{1244};
  int32_t gain_{191};
  float ref_product_{1244.0f * 63.0f};
  bool has_updated_{false};
  int64_t last_update_us_{0};
  uint8_t settle_remaining_{0};
};

}  // namespace esp_video_camera
}  // namespace esphome
