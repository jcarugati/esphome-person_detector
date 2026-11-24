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
#include <limits>

namespace esphome {
namespace esp_video_camera {

// Matches fixed-mode's omitted-gain default (25% of the legacy 0..255 fallback = 63)
// so enabling auto_exposure with omitted `gain` keeps the exact same ceiling and
// Ambient Light (%) reference as fixed mode.
static constexpr int32_t DEFAULT_AE_GAIN_CEILING = 63;

// SC202CS sensor pipeline delay (exposure_frame_delay = gain_frame_delay = 3 in
// Espressif's s_ipa_agc_SC202CS_config).
static constexpr uint8_t SC202CS_AE_PIPELINE_DELAY_FRAMES = 3;

inline int32_t default_ae_gain_ceiling(int32_t min_gain, int32_t max_gain) {
  if (min_gain > max_gain)
    min_gain = max_gain;
  return std::clamp<int32_t>(DEFAULT_AE_GAIN_CEILING, min_gain, max_gain);
}

// Number of post-write dequeued frames that must be marked settling (NaN scale):
// - Task mode: all queued V4L2 buffers plus SC202CS's 3-frame register delay.
// - Non-task mode: acquire() flushes stale buffers at the detector interval;
//   mark 1 settling frame so the following frame reflects the new setting.
inline uint8_t ae_settling_frame_count(uint8_t fb_count, bool task_mode) {
  return task_mode ? static_cast<uint8_t>(fb_count + SC202CS_AE_PIPELINE_DELAY_FRAMES)
                   : uint8_t{1};
}

// Single-word atomic packing for (mean_luma, capture_epoch) feedback from
// person_detect to EspVideoCamera. 0 denotes empty; bit 40 marks valid.
inline uint64_t pack_ae_feedback(uint8_t mean_luma, uint32_t capture_epoch) {
  return (1ULL << 40) | (static_cast<uint64_t>(capture_epoch) << 8) |
         static_cast<uint64_t>(mean_luma);
}

inline bool unpack_ae_feedback(uint64_t packed, uint8_t &mean_luma, uint32_t &capture_epoch) {
  if (packed == 0)
    return false;
  mean_luma = static_cast<uint8_t>(packed & 0xFFULL);
  capture_epoch = static_cast<uint32_t>((packed >> 8) & 0xFFFFFFFFULL);
  return true;
}

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
    this->controls_known_ = false;
    this->has_updated_ = false;
    this->last_update_us_ = 0;
    this->settle_remaining_ = 0;
    this->settling_frames_ = 0;
  }

  bool is_known() const { return this->controls_known_; }
  bool is_settling() const { return this->settling_frames_ > 0; }
  uint8_t settling_frames() const { return this->settling_frames_; }

  // Record a complete, verified write of BOTH exposure and gain to the sensor.
  void mark_applied(int32_t exp, int32_t gain, int64_t now_us, uint8_t settling_frames = 0) {
    this->exposure_ = std::clamp(exp, this->min_exp_, this->max_exp_);
    this->gain_ = std::clamp(gain, this->min_gain_, this->max_gain_);
    this->controls_known_ = true;
    this->has_updated_ = true;
    this->last_update_us_ = now_us;
    this->settling_frames_ = settling_frames;
  }

  // Record that an exposure or gain ioctl failed so physical sensor registers
  // are unknown. Keeps target (exposure_, gain_) and rate-limit timestamp so
  // the caller retries BOTH controls together once min_interval_us elapses.
  void mark_unknown(int64_t now_us) {
    this->controls_known_ = false;
    this->has_updated_ = true;
    this->last_update_us_ = now_us;
    this->settle_remaining_ = 0;
    this->settling_frames_ = 0;
  }

  // True when sensor registers are unknown and a full (exposure + gain) retry
  // is allowed (immediately after reset, or once min_interval_us has elapsed).
  bool should_retry_unknown(int64_t now_us) const {
    if (this->controls_known_)
      return false;
    if (!this->has_updated_)
      return true;
    if (now_us < this->last_update_us_)
      return true;
    return (now_us - this->last_update_us_) >= this->cfg_.min_interval_us;
  }

  // Compute the exposure_scale stamp for a newly dequeued frame and advance the
  // settling countdown once per dequeued frame. Returns NaN while settling or
  // while physical sensor registers are unknown.
  float stamp_dequeued_frame() {
    if (this->settling_frames_ > 0) {
      this->settling_frames_--;
      return std::numeric_limits<float>::quiet_NaN();
    }
    return this->exposure_scale();
  }

  AutoExposureDecision update(uint8_t mean_luma, int64_t now_us) {
    AutoExposureDecision decision{};
    decision.exposure = this->exposure_;
    decision.gain = this->gain_;

    // Never step from an unknown physical register state or while frames from a
    // prior write are still draining through V4L2/sensor pipeline latency.
    if (!this->controls_known_ || this->settling_frames_ > 0)
      return decision;

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

  float exposure_scale() const {
    if (!this->controls_known_)
      return std::numeric_limits<float>::quiet_NaN();
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
  bool controls_known_{false};
  bool has_updated_{false};
  int64_t last_update_us_{0};
  uint8_t settle_remaining_{0};
  uint8_t settling_frames_{0};
};

}  // namespace esp_video_camera
}  // namespace esphome
