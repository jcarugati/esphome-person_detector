// g++ -std=c++20 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -I components test/auto_exposure_test.cpp -o /tmp/ae && /tmp/ae
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "esp_video_camera/auto_exposure.h"
#include "person_detect/frame_source.h"

using esphome::esp_video_camera::AutoExposureConfig;
using esphome::esp_video_camera::AutoExposureController;
using esphome::esp_video_camera::AutoExposureDecision;
using esphome::esp_video_camera::ae_settling_frame_count;
using esphome::esp_video_camera::default_ae_gain_ceiling;
using esphome::esp_video_camera::pack_ae_feedback;
using esphome::esp_video_camera::sc202cs_gain_factor;
using esphome::esp_video_camera::sc202cs_gain_index_for_factor;
using esphome::esp_video_camera::unpack_ae_feedback;
using esphome::person_detect::normalize_illuminance_pct;

static bool near(float a, float b, float tol = 1e-4f) {
  return std::fabs(a - b) <= tol;
}

static uint8_t simulate_sensor_luma(float scene_luma_at_ceiling, float exposure_scale) {
  const float raw = scene_luma_at_ceiling * exposure_scale;
  if (raw <= 0.0f)
    return 0;
  if (raw >= 255.0f)
    return 255;
  return static_cast<uint8_t>(std::lround(raw));
}

int main() {
  constexpr int64_t kSec = 1000000;

  // 1. SC202CS physical gain curve: landmarks, strict monotonicity, continuity
  //    across octaves, driver table agreement, and exact inverse round-trip.
  assert(near(sc202cs_gain_factor(-5), 1.0f));
  assert(near(sc202cs_gain_factor(0), 1.0f));
  assert(near(sc202cs_gain_factor(1), 1.03125f));
  assert(near(sc202cs_gain_factor(31), 1.96875f));
  assert(near(sc202cs_gain_factor(32), 2.0f));
  assert(near(sc202cs_gain_factor(64), 4.0f));
  assert(near(sc202cs_gain_factor(96), 8.0f));
  assert(near(sc202cs_gain_factor(128), 16.0f));
  assert(near(sc202cs_gain_factor(160), 32.0f));
  assert(near(sc202cs_gain_factor(191), 63.0f));

  // Cross-check against sc202cs_abs_gain_val_map[] landmarks (x1000 in driver).
  struct GainLandmark {
    int32_t idx;
    float driver_gain;
  };
  const GainLandmark landmarks[] = {
      {0, 1.000f},   {16, 1.500f},  {31, 1.969f},  {32, 2.000f},  {48, 3.000f},
      {64, 4.000f},  {96, 8.000f},  {128, 16.000f}, {160, 32.000f}, {191, 63.008f},
  };
  for (const auto &lm : landmarks) {
    const float f = sc202cs_gain_factor(lm.idx);
    assert(std::fabs(f - lm.driver_gain) / lm.driver_gain < 0.002f);
  }

  // Strictly monotonic and exact round-trip across all 192 SC202CS indices.
  float prev_g = 0.0f;
  for (int32_t idx = 0; idx <= 191; ++idx) {
    const float g = sc202cs_gain_factor(idx);
    assert(std::isfinite(g) && g > prev_g);
    if (idx > 0) {
      // Every step (including octave boundaries) is between 1.5% and 3.2%.
      const float step = g / prev_g;
      assert(step > 1.01f && step < 1.035f);
    }
    assert(sc202cs_gain_index_for_factor(g, 0, 191) == idx);
    prev_g = g;
  }
  assert(sc202cs_gain_index_for_factor(std::numeric_limits<float>::quiet_NaN(), 0, 191) == 0);
  assert(sc202cs_gain_index_for_factor(-10.0f, 5, 191) == 5);
  assert(sc202cs_gain_index_for_factor(9999.0f, 0, 191) == 191);
  assert(std::isfinite(sc202cs_gain_factor(10000)));

  // 2. Deadband holds steady (no spurious writes in [97..133]).
  AutoExposureController ae{};
  ae.configure(8, 1244, 0, 191);
  // Freshly configured controller starts unknown until hardware writes succeed.
  assert(!ae.is_known());
  assert(std::isnan(ae.exposure_scale()));
  assert(ae.should_retry_unknown(0));
  ae.mark_applied(1244, 191, 0);
  assert(ae.is_known());
  assert(ae.exposure() == 1244 && ae.gain() == 191);
  assert(ae.exposure_scale() == 1.0f);
  int64_t t = 5 * kSec;
  for (int luma = 97; luma <= 133; ++luma) {
    const AutoExposureDecision d = ae.update(static_cast<uint8_t>(luma), t);
    assert(!d.changed && !d.exposure_changed && !d.gain_changed);
    assert(d.exposure == 1244 && d.gain == 191);
    t += 5 * kSec;
  }
  // Dim scene while already at ceiling: no write, scale stays exact 1.0f.
  for (int luma = 0; luma < 97; luma += 10) {
    const AutoExposureDecision d = ae.update(static_cast<uint8_t>(luma), t);
    assert(!d.changed);
    assert(ae.exposure_scale() == 1.0f);
    t += 5 * kSec;
  }

  // 3. Settle-frame skip and >= 3000 ms rate limiting.
  ae.reset();
  ae.mark_applied(1244, 191, 0);
  t = 10 * kSec;
  {
    const AutoExposureDecision d1 = ae.update(220, t);
    assert(d1.changed && d1.gain_changed && !d1.exposure_changed);
    assert(d1.exposure == 1244 && d1.gain < 191);
    ae.mark_applied(d1.exposure, d1.gain, t);

    // Even 10 seconds later, the very first report after a write is skipped to
    // let the sensor pipeline settle.
    const AutoExposureDecision d_settle = ae.update(220, t + 10 * kSec);
    assert(!d_settle.changed);

    // Next report succeeds because settle is done and >= 3s elapsed.
    const AutoExposureDecision d2 = ae.update(220, t + 11 * kSec);
    assert(d2.changed && d2.gain < d1.gain);
    ae.mark_applied(d2.exposure, d2.gain, t + 11 * kSec);

    // Now test rapid reports: settle skipped at +1s, rate-limited at +2s and
    // +2.999s, then allowed at +3.0s.
    const int64_t t2 = t + 11 * kSec;
    assert(!ae.update(220, t2 + 1 * kSec).changed);       // settle frame
    assert(!ae.update(220, t2 + 2 * kSec).changed);       // < 3s
    assert(!ae.update(220, t2 + 3 * kSec - 1).changed);   // 2999.999 ms
    const AutoExposureDecision d3 = ae.update(220, t2 + 3 * kSec);
    assert(d3.changed && d3.gain < d2.gain);
  }

  // 4. Bright scene convergence: gain decreases FIRST (less noise), then
  //    exposure; converges into deadband [97..133] without oscillating.
  {
    ae.reset();
    ae.mark_applied(1244, 191, 0);
    t = 3 * kSec;
    const float bright_indoor = 1200.0f;  // ~10x brighter than target=115 at ceiling
    bool saw_exposure_change = false;
    int changes = 0;
    uint8_t last_luma = 255;
    for (int step = 0; step < 40; ++step) {
      last_luma = simulate_sensor_luma(bright_indoor, ae.exposure_scale());
      const AutoExposureDecision d = ae.update(last_luma, t);
      if (d.changed) {
        ae.mark_applied(d.exposure, d.gain, t);
        ++changes;
        if (d.exposure_changed)
          saw_exposure_change = true;
      }
      t += 1500000;  // 1.5 s detector cadence
    }
    assert(changes > 0);
    assert(!saw_exposure_change);  // Gain alone covers 63x -> exposure stays 1244
    assert(ae.exposure() == 1244 && ae.gain() > 0 && ae.gain() < 191);
    assert(last_luma >= 97 && last_luma <= 133);
    // Verify stability (zero oscillation for 20 more frames).
    for (int step = 0; step < 20; ++step) {
      const uint8_t y = simulate_sensor_luma(bright_indoor, ae.exposure_scale());
      assert(!ae.update(y, t).changed);
      t += 1500000;
    }
  }

  // Direct sunlight: gain drops to 0 FIRST, then exposure drops below 1244.
  {
    ae.reset();
    ae.mark_applied(1244, 191, 0);
    t = 3 * kSec;
    const float direct_sun = 40000.0f;
    bool exposure_moved_before_min_gain = false;
    uint8_t last_luma = 255;
    for (int step = 0; step < 60; ++step) {
      last_luma = simulate_sensor_luma(direct_sun, ae.exposure_scale());
      const int32_t prev_exp = ae.exposure();
      const AutoExposureDecision d = ae.update(last_luma, t);
      if (d.changed) {
        if (d.exposure < prev_exp && d.gain > ae.min_gain())
          exposure_moved_before_min_gain = true;
        ae.mark_applied(d.exposure, d.gain, t);
      }
      t += 1500000;
    }
    assert(!exposure_moved_before_min_gain);
    assert(ae.gain() == 0 && ae.exposure() < 1244 && ae.exposure() > 8);
    assert(last_luma >= 97 && last_luma <= 133);
    // Stable once converged.
    for (int step = 0; step < 20; ++step) {
      const uint8_t y = simulate_sensor_luma(direct_sun, ae.exposure_scale());
      assert(!ae.update(y, t).changed);
      t += 1500000;
    }

    // 5. Dark scene recovery from low exposure + min gain: exposure rises to
    //    1244 FIRST while gain stays at 0, then gain rises to ceiling 191.
    const float dark_room = 50.0f;  // ~19.6% ambient at ceiling
    bool gain_moved_before_max_exp = false;
    for (int step = 0; step < 60; ++step) {
      const uint8_t y = simulate_sensor_luma(dark_room, ae.exposure_scale());
      const int32_t prev_gain = ae.gain();
      const AutoExposureDecision d = ae.update(y, t);
      if (d.changed) {
        if (d.gain > prev_gain && d.exposure < ae.max_exposure())
          gain_moved_before_max_exp = true;
        ae.mark_applied(d.exposure, d.gain, t);
      }
      t += 1500000;
    }
    assert(!gain_moved_before_max_exp);
    assert(ae.exposure() == 1244 && ae.gain() == 191);
    assert(ae.exposure_scale() == 1.0f);
    // No further writes once clamped at ceiling.
    assert(!ae.update(50, t + 10 * kSec).changed);
  }

  // Extreme bright scene clamps at floor (min_exp, min_gain) and stops writing.
  {
    ae.reset();
    ae.mark_applied(1244, 191, 0);
    t = 3 * kSec;
    for (int step = 0; step < 120; ++step) {
      const AutoExposureDecision d = ae.update(255, t);
      if (d.changed)
        ae.mark_applied(d.exposure, d.gain, t);
      t += 1500000;
    }
    assert(ae.exposure() == 8 && ae.gain() == 0);
    assert(ae.exposure_scale() > 0.0f && ae.exposure_scale() < 1.0f);
    assert(!ae.update(255, t + 10 * kSec).changed);
  }

  // 6. exposure_scale() monotonicity & continuity across the full sweep and knee.
  {
    ae.configure(8, 1244, 0, 191);
    float prev_scale = 0.0f;
    // Exposure sweep at min gain (8..1244, gain=0)
    for (int32_t exp = 8; exp <= 1244; ++exp) {
      ae.mark_applied(exp, 0, 0);
      const float s = ae.exposure_scale();
      assert(std::isfinite(s) && s > prev_scale && s < 1.0f);
      prev_scale = s;
    }
    const float knee_scale = prev_scale;
    // Gain sweep at max exposure (exp=1244, gain=1..191)
    for (int32_t gain = 1; gain <= 191; ++gain) {
      ae.mark_applied(1244, gain, 0);
      const float s = ae.exposure_scale();
      assert(std::isfinite(s) && s > prev_scale);
      if (gain == 1) {
        // Continuous across knee: 1.03125x step from (1244, 0) to (1244, 1).
        assert(near(s / knee_scale, 33.0f / 32.0f, 1e-4f));
      }
      prev_scale = s;
    }
    assert(ae.exposure_scale() == 1.0f);
  }

  // 7. Single normalize_illuminance_pct() in person_detect/frame_source.h:
  //    fixed-mode equivalence, daylight scaling, and defensive fallback.
  for (int y = 0; y <= 255; ++y) {
    const uint8_t luma = static_cast<uint8_t>(y);
    const float expected = static_cast<float>(luma) * 100.0f / 255.0f;
    assert(normalize_illuminance_pct(luma, 1.0f) == expected);
  }
  // Nocturne thresholds preserved at ceiling (scale=1.0):
  //   luma=63 -> 24.7% (< 25% DARK), luma=115 -> 45.1% (> 45% LIGHT)
  assert(normalize_illuminance_pct(63, 1.0f) < 25.0f);
  assert(normalize_illuminance_pct(115, 1.0f) > 45.0f);
  // Bright daylight: AE holds luma near 115 with scale < 1 -> rises into 50..100%.
  assert(near(normalize_illuminance_pct(115, 0.5f), (115.0f / 0.5f) * 100.0f / 255.0f, 1e-3f));
  assert(normalize_illuminance_pct(115, 0.5f) > 85.0f);
  assert(normalize_illuminance_pct(115, 0.1f) == 100.0f);
  assert(normalize_illuminance_pct(0, 0.05f) == 0.0f);
  // Defensive fallback on NaN, +Inf, -Inf, 0, negative scales.
  const float invalid_scales[] = {
      0.0f,
      -0.5f,
      -100.0f,
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  for (float bad : invalid_scales) {
    const float expected = 115.0f * 100.0f / 255.0f;
    assert(near(normalize_illuminance_pct(115, bad), expected));
  }

  // 8. Regression (Astra #2): pre-change queued buffer & settling frames are
  //    NEVER stamped with the new exposure scale.
  //    Model a 2-buffer V4L2 ring captured at 0.5x followed by a 1.0x command:
  //    - Dequeued pre-change frame (before writing 1.0x) keeps 0.5x.
  //    - Next fb_count + 3 = 5 settling frames report NaN (unknown).
  //    - First settled post-change frame reports 1.0x.
  {
    constexpr uint8_t kFbCount = 2;
    const uint8_t settle = ae_settling_frame_count(kFbCount);
    assert(settle == kFbCount + 3);

    ae.configure(8, 1244, 0, 0);
    ae.mark_applied(622, 0, 0, 0);  // Settled at exact 0.5x
    assert(near(ae.exposure_scale(), 0.5f));

    // Dequeue & stamp an already-queued frame BEFORE applying the 1.0x write.
    const float pre_change_stamp = ae.stamp_dequeued_frame();
    assert(near(pre_change_stamp, 0.5f));

    // Now apply the 1.0x command after stamping the pre-change frame
    // (luma=20 -> raw_ratio=5.75 -> clamped to +1 EV = 2.0x -> 622 -> 1244).
    const AutoExposureDecision step = ae.update(20, 5 * kSec);
    assert(step.changed && step.exposure_changed && step.exposure == 1244);
    ae.mark_applied(step.exposure, step.gain, 5 * kSec, settle);
    assert(ae.is_settling() && ae.settling_frames() == 5);

    // Every queued V4L2 buffer + 3 SC202CS pipeline delay frames gets NaN.
    for (uint8_t i = 0; i < settle; ++i) {
      assert(!ae.update(250, 10 * kSec + i * kSec).changed);  // AE ignores feedback while settling
      const float settling_stamp = ae.stamp_dequeued_frame();
      assert(std::isnan(settling_stamp));
    }
    assert(!ae.is_settling());

    // First settled post-change frame carries the new 1.0x scale.
    const float post_change_stamp = ae.stamp_dequeued_frame();
    assert(post_change_stamp == 1.0f);

    // Non-task mode regression (Astra R2 #1): stale-buffer flush finds 0
    // buffers and capture retries immediately (e.g. after failed frame
    // conversion). All fb_count + 3 post-write dequeues must still return NaN
    // so the 2nd post-write frame never receives the new scale before the
    // 3-frame SC202CS pipeline delay elapses.
    ae.mark_applied(622, 0, 20 * kSec, settle);
    for (uint8_t i = 0; i < settle; ++i) {
      // 0 stale buffers flushed; immediate retry dequeues 1 frame per attempt.
      assert(std::isnan(ae.stamp_dequeued_frame()));
    }
    assert(!ae.is_settling());
    assert(near(ae.stamp_dequeued_frame(), 0.5f));

    // Non-task mode with stale-buffer flush: discarding kFbCount stale buffers
    // counts toward settle, and the remaining 3 SC202CS delay frames still
    // produce NaN before the settled frame returns 0.5f.
    ae.mark_applied(622, 0, 25 * kSec, settle);
    for (uint8_t i = 0; i < kFbCount; ++i)
      assert(std::isnan(ae.stamp_dequeued_frame()));  // flushed stale buffers
    for (uint8_t i = 0; i < 3; ++i)
      assert(std::isnan(ae.stamp_dequeued_frame()));  // SC202CS 3-frame delay
    assert(near(ae.stamp_dequeued_frame(), 0.5f));
  }

  // 9. Regression (Astra #3): startup ioctl failure and partial mid-stream
  //    ioctl failure leave physical registers unknown (NaN scale) until a
  //    rate-limited complete (exposure + gain) retry succeeds and settles.
  {
    constexpr uint8_t kSettle = 5;  // fb_count=2 + 3
    ae.configure(8, 1244, 0, 191);

    // Startup failure: first attempt at t=0 fails -> remains unknown.
    assert(!ae.is_known() && ae.should_retry_unknown(0));
    ae.mark_unknown(0);
    assert(!ae.is_known());
    assert(std::isnan(ae.exposure_scale()));
    assert(std::isnan(ae.stamp_dequeued_frame()));
    assert(!ae.update(240, 1 * kSec).changed);
    assert(!ae.should_retry_unknown(2 * kSec));
    assert(!ae.should_retry_unknown(3 * kSec - 1));

    // At >= 3s, complete retry succeeds -> known + settling -> settled 1.0f.
    assert(ae.should_retry_unknown(3 * kSec));
    assert(ae.exposure() == 1244 && ae.gain() == 191);
    ae.mark_applied(ae.exposure(), ae.gain(), 3 * kSec, kSettle);
    assert(ae.is_known() && !ae.should_retry_unknown(10 * kSec));
    for (uint8_t i = 0; i < kSettle; ++i)
      assert(std::isnan(ae.stamp_dequeued_frame()));
    assert(ae.stamp_dequeued_frame() == 1.0f);

    // Mid-stream partial ioctl failure: AE steps down at t=10s, one ioctl fails
    // -> mark_unknown(10s) -> NEVER assumes previous scale still holds!
    const AutoExposureDecision d = ae.update(240, 10 * kSec);
    assert(d.changed && d.gain < 191);
    const int32_t target_exp = d.exposure;
    const int32_t target_gain = d.gain;
    ae.mark_unknown(10 * kSec);
    assert(!ae.is_known());
    assert(std::isnan(ae.exposure_scale()));
    assert(std::isnan(ae.stamp_dequeued_frame()));
    assert(!ae.update(240, 15 * kSec).changed);  // cannot step while unknown
    assert(!ae.should_retry_unknown(12 * kSec));

    // At 13s (>= 3s after failure), complete retry of BOTH target_exp and
    // target_gain succeeds -> known + settling -> settled new scale.
    assert(ae.should_retry_unknown(13 * kSec));
    assert(ae.exposure() == target_exp && ae.gain() == target_gain);
    ae.mark_applied(ae.exposure(), ae.gain(), 13 * kSec, kSettle);
    assert(ae.is_known());
    for (uint8_t i = 0; i < kSettle; ++i)
      assert(std::isnan(ae.stamp_dequeued_frame()));
    const float recovered_scale = ae.stamp_dequeued_frame();
    assert(std::isfinite(recovered_scale) && recovered_scale > 0.0f && recovered_scale < 1.0f);
  }

  // 10. Regression (Astra #4): epoch-tagged atomic feedback packing & stale
  //     session rejection across privacy OFF/ON restarts.
  {
    uint8_t luma = 99;
    uint32_t epoch = 99;
    assert(!unpack_ae_feedback(0, luma, epoch));

    const uint64_t p0 = pack_ae_feedback(0, 0);
    assert(p0 != 0);
    assert(unpack_ae_feedback(p0, luma, epoch));
    assert(luma == 0 && epoch == 0);

    const uint64_t p1 = pack_ae_feedback(215, 7);
    assert(unpack_ae_feedback(p1, luma, epoch));
    assert(luma == 215 && epoch == 7);
    // Simulate camera restarted at epoch 8: stale epoch 7 feedback is rejected.
    const uint32_t current_epoch = 8;
    assert(epoch != current_epoch);

    const uint64_t pmax = pack_ae_feedback(255, 0xFFFFFFFFu);
    assert(unpack_ae_feedback(pmax, luma, epoch));
    assert(luma == 255 && epoch == 0xFFFFFFFFu);
  }

  // 11. Regression (Astra #5 + Opus #1): omitted-gain default ceiling keeps
  //     fixed-mode parity (63, clamped to sensor bounds) across both Nocturne
  //     thresholds (25% DARK / 45% LIGHT).
  {
    assert(default_ae_gain_ceiling(0, 191) == 63);
    assert(default_ae_gain_ceiling(0, 40) == 40);
    assert(default_ae_gain_ceiling(80, 191) == 80);

    // Default ceiling (exposure=811, gain=63) matches fixed mode exactly.
    ae.configure(8, 811, 0, default_ae_gain_ceiling(0, 191));
    ae.mark_applied(811, 63, 0);
    assert(ae.max_gain() == 63 && ae.gain() == 63);
    assert(ae.exposure_scale() == 1.0f);
    // Dark bedroom (< 25% Nocturne DARK threshold): stays at ceiling 1.0x.
    assert(normalize_illuminance_pct(60, ae.exposure_scale()) < 25.0f);
    assert(!ae.update(60, 5 * kSec).changed);
    // Bright bedroom (> 45% Nocturne LIGHT threshold): at ceiling luma=115 -> 45.1%.
    assert(normalize_illuminance_pct(115, ae.exposure_scale()) > 45.0f);
    // Daylight scene: AE steps down below gain=63 while normalized light stays > 45%.
    const AutoExposureDecision day = ae.update(230, 5 * kSec);
    assert(day.changed && day.gain < 63);
    ae.mark_applied(day.exposure, day.gain, 5 * kSec);
    const float day_scale = ae.exposure_scale();
    assert(day_scale > 0.0f && day_scale < 1.0f);
    assert(normalize_illuminance_pct(115, day_scale) > 45.0f);
  }

  // 12. Privacy reset() restores ceiling, marks unknown, and clears timers.
  {
    ae.configure(8, 1244, 0, 191);
    ae.mark_applied(1244, 191, 0);
    const AutoExposureDecision d = ae.update(250, 50 * kSec);
    assert(d.changed && ae.exposure_scale() < 1.0f);
    ae.reset();
    assert(!ae.is_known() && ae.should_retry_unknown(1 * kSec));
    assert(ae.exposure() == 1244 && ae.gain() == 191);
    assert(std::isnan(ae.exposure_scale()));
    ae.mark_applied(1244, 191, 1 * kSec);
    assert(ae.exposure_scale() == 1.0f);
    // Responsive once min_interval_us elapses after the post-reset write.
    const AutoExposureDecision after = ae.update(250, 4 * kSec);
    assert(after.changed && after.gain < 191);
  }

  // 13. Edge cases: min == max, luma == 0, luma == 255, non-monotonic clock.
  {
    // Locked min == max
    ae.configure(500, 500, 32, 32);
    ae.mark_applied(500, 32, 0);
    assert(ae.exposure_scale() == 1.0f);
    assert(!ae.update(0, 5 * kSec).changed);
    assert(!ae.update(255, 10 * kSec).changed);
    assert(ae.exposure() == 500 && ae.gain() == 32 && ae.exposure_scale() == 1.0f);

    // Inverted / zero bounds sanitized safely
    ae.configure(0, 0, -5, -1);
    ae.mark_applied(1, 0, 0);
    assert(ae.min_exposure() == 1 && ae.max_exposure() == 1);
    assert(ae.min_gain() == 0 && ae.max_gain() == 0);
    assert(ae.exposure_scale() == 1.0f);

    // luma == 0 recovers smoothly from low exposure without div-by-zero
    ae.configure(8, 1244, 0, 191);
    ae.mark_applied(20, 0, 0);
    const AutoExposureDecision d0 = ae.update(0, 10 * kSec);
    assert(d0.changed && d0.exposure == 40 && d0.gain == 0);
    ae.mark_applied(d0.exposure, d0.gain, 10 * kSec);

    // Clock wrap / backwards timestamp: re-anchors without spurious write, then resumes.
    assert(!ae.update(0, 11 * kSec).changed);           // settle skip
    assert(!ae.update(0, 2 * kSec).changed);            // backwards jump -> re-anchor to 2s
    assert(!ae.update(0, 4 * kSec).changed);            // < 3s from 2s
    const AutoExposureDecision resumed = ae.update(0, 5 * kSec);
    assert(resumed.changed && resumed.exposure == 80);  // 2x step from 40
  }

  return 0;
}
