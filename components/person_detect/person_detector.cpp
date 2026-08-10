#include <cmath>
#include <list>

#include "person_detector.h"
#include "esphome_camera_source.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esp_heap_caps.h"

// ESP-DL pedestrian model + detection result type. Provided by the
// `espressif/pedestrian_detect` registry component (see __init__.py), which
// pulls esp-dl transitively.
#include "pedestrian_detect.hpp"
#include "dl_detect_define.hpp"
#include "dl_image_define.hpp"

namespace esphome {
namespace person_detect {

static const char *const TAG = "person_detect";

static const char *model_to_string(Model model) {
  switch (model) {
    case Model::PEDESTRIAN:
      return "pedestrian (ESP-DL, 224x224 INT8)";
    default:
      return "unknown";
  }
}

void PersonDetector::setup() {
  ESP_LOGCONFIG(TAG, "Setting up person_detect...");

  // Frame source selection: a raw source injected via set_frame_source() (e.g.
  // the esp_video CSI backend) wins; otherwise fall back to wrapping an ESPHome
  // camera in the JPEG-decoding source.
  if (this->source_ == nullptr) {
    if (this->camera_ == nullptr) {
      ESP_LOGE(TAG, "No frame source: set either camera_id or a raw source");
      this->mark_failed();
      return;
    }
    this->source_ = new EsphomeCameraSource(this->camera_);  // NOLINT
  }
  if (!this->source_->init()) {
    ESP_LOGE(TAG, "Frame source init failed");
    this->mark_failed();
    return;
  }

  if (!this->create_model_()) {
    this->mark_failed();
    return;
  }

  this->wake_ = xSemaphoreCreateBinary();
  this->result_mutex_ = xSemaphoreCreateMutex();
  if (this->wake_ == nullptr || this->result_mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate task primitives");
    this->mark_failed();
    return;
  }

  // Honor the privacy toggle's restored state: only stream if enabled.
  if (this->enabled_.load()) {
    this->source_->start();
  } else {
    ESP_LOGCONFIG(TAG, "Detection disabled at boot (privacy switch); camera idle");
#ifdef USE_BINARY_SENSOR
    // Publish a definite "not occupied" so the entity isn't left unknown in
    // Home Assistant while the camera stays idle. Done directly (not via
    // publish_present_) to avoid firing the on_cleared automation at boot.
    if (this->binary_sensor_ != nullptr)
      this->binary_sensor_->publish_state(false);
#endif
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
      &PersonDetector::task_entry, "person_detect", this->task_stack_size_, this,
      this->task_priority_, &this->task_handle_, this->task_core_);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create inference task");
    this->mark_failed();
    return;
  }
}

bool PersonDetector::create_model_() {
  size_t before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  switch (this->model_kind_) {
    case Model::PEDESTRIAN:
    default:
      this->model_ = new PedestrianDetect();  // NOLINT
      break;
  }
  if (this->model_ == nullptr) {
    ESP_LOGE(TAG, "Model allocation failed");
    return false;
  }
  size_t after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  this->model_psram_cost_ = (before > after) ? (before - after) : 0;
  ESP_LOGI(TAG, "Loaded %s; model runtime PSRAM cost ~%u bytes",
           model_to_string(this->model_kind_), (unsigned) this->model_psram_cost_);
  return true;
}

void PersonDetector::task_entry(void *param) {
  static_cast<PersonDetector *>(param)->task_loop_();
}

void PersonDetector::task_loop_() {
  // Give a slow sensor a chance to deliver the first frame after enabling.
  const uint32_t capture_timeout = this->interval_ms_ + 1500;
  for (;;) {
    if (!this->enabled_.load()) {
      // Idle until re-enabled (privacy). Camera is already stopped.
      xSemaphoreTake(this->wake_, portMAX_DELAY);
      continue;
    }

    FrameView frame;
    if (this->source_->acquire(frame, capture_timeout)) {
      this->run_inference_(frame);
      this->source_->release();
    } else {
      this->capture_failures_++;
      ESP_LOGW(TAG, "Frame capture timed out (%u total)",
               (unsigned) this->capture_failures_);
    }

    vTaskDelay(pdMS_TO_TICKS(this->interval_ms_));
  }
}

void PersonDetector::run_inference_(const FrameView &frame) {
  auto *model = static_cast<PedestrianDetect *>(this->model_);
  if (model == nullptr)
    return;

  dl::image::img_t img{};
  img.data = const_cast<uint8_t *>(frame.data);
  img.width = frame.width;
  img.height = frame.height;
  switch (frame.format) {
    case FRAME_FORMAT_RGB565:
      img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
      break;
    case FRAME_FORMAT_GRAYSCALE:
      img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_GRAY;
      break;
    case FRAME_FORMAT_RGB888:
    default:
      img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
      break;
  }

  // Whole-frame brightness probe. Serves two purposes: a blank-frame sanity
  // check (an ISP not exposing the RAW sensor reads ~uniform, distinguishing it
  // from a valid image where the model simply found no person) and the ambient-
  // light reading. Per-pixel Rec.601 luma sampled across the frame; because the
  // sensor runs at fixed exposure/gain, mean luma tracks scene illumination.
  // Cheap: a few thousand samples at the detection interval.
  uint32_t px_sum = 0;  // reused as mean luma (0..255)
  uint8_t px_min = 255, px_max = 0;
  {
    size_t bytes_per_px = (frame.format == FRAME_FORMAT_RGB888) ? 3
                          : (frame.format == FRAME_FORMAT_RGB565) ? 2
                                                                  : 1;
    size_t npx = static_cast<size_t>(frame.width) * frame.height;
    size_t step = npx > 4096 ? npx / 4096 : 1;
    size_t samples = 0;
    for (size_t p = 0; p < npx; p += step) {
      const uint8_t *px = frame.data + p * bytes_per_px;
      uint8_t y;
      switch (frame.format) {
        case FRAME_FORMAT_RGB565: {
          uint16_t v =
              static_cast<uint16_t>(px[0]) | (static_cast<uint16_t>(px[1]) << 8);
          uint8_t r = ((v >> 11) & 0x1F) << 3;
          uint8_t g = ((v >> 5) & 0x3F) << 2;
          uint8_t b = (v & 0x1F) << 3;
          y = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
          break;
        }
        case FRAME_FORMAT_GRAYSCALE:
          y = px[0];
          break;
        case FRAME_FORMAT_RGB888:
        default:
          y = static_cast<uint8_t>((77 * px[0] + 150 * px[1] + 29 * px[2]) >> 8);
          break;
      }
      px_sum += y;
      if (y < px_min) px_min = y;
      if (y > px_max) px_max = y;
      samples++;
    }
    px_sum = samples ? px_sum / samples : 0;  // reuse as mean luma
  }
  float illuminance_pct = px_sum * 100.0f / 255.0f;

  uint32_t t0 = millis();
  // ESP-DL resizes the frame to the model's 224x224 input internally; on P4 that
  // resize/color-convert is hardware-accelerated (PPA / 2D-DMA) inside dl::image.
  std::list<dl::detect::result_t> &results = model->run(img);
  uint32_t dt = millis() - t0;

  float best = 0.0f;
  uint8_t count = 0;
  for (const auto &r : results) {
    if (r.score >= this->threshold_)
      count++;
    if (r.score > best)
      best = r.score;
  }
  bool person = best >= this->threshold_;

  size_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  if (psram < this->psram_free_low_)
    this->psram_free_low_ = psram;

  ESP_LOGD(TAG,
           "inference %ums: %ux%u boxes=%u best=%.2f present=%d "
           "luma[min=%u max=%u mean=%u] light=%.0f%% psram_free=%u",
           (unsigned) dt, frame.width, frame.height, (unsigned) results.size(),
           best, person, (unsigned) px_min, (unsigned) px_max, (unsigned) px_sum,
           illuminance_pct, (unsigned) psram);

  xSemaphoreTake(this->result_mutex_, portMAX_DELAY);
  this->last_infer_ms_ = dt;
  // Discard a result whose frame was captured for a now-disabled detector, so a
  // late-completing inference can't re-assert presence after the privacy switch
  // is toggled off. Paired with the mutex-guarded clear in set_enabled() this is
  // robust even when disable races the write: enabled_ is set false before that
  // clear runs, so whichever acquires the mutex last, no stale result survives.
  if (this->enabled_.load()) {
    this->pending_.person_raw = person;
    this->pending_.best_score = best;
    this->pending_.count = count;
    this->pending_.infer_ms = dt;
    this->pending_.illuminance = illuminance_pct;
    this->has_new_result_ = true;
  }
  xSemaphoreGive(this->result_mutex_);
}

void PersonDetector::loop() {
  // All entity/state publishing and debounce happen here on the main loop.
  if (this->force_clear_.exchange(false)) {
    this->miss_streak_ = 0;
    if (this->present_state_)
      this->publish_present_(false);
    // Privacy off: reset the numeric sensors too. Otherwise they stay frozen at
    // their last reading (e.g. 92% confidence, count 2) while presence reads
    // clear, showing contradictory state in Home Assistant.
#ifdef USE_SENSOR
    if (this->confidence_sensor_ != nullptr)
      this->confidence_sensor_->publish_state(0.0f);
    if (this->count_sensor_ != nullptr)
      this->count_sensor_->publish_state(0);
    // The camera is idled, so ambient light is genuinely unknown (not zero):
    // publish NaN, which Home Assistant renders as "unknown".
    if (this->illuminance_sensor_ != nullptr)
      this->illuminance_sensor_->publish_state(NAN);
#endif
  }

  // Don't act on a late in-flight inference while detection is disabled.
  if (!this->enabled_.load())
    return;

  InferenceResult r;
  bool have = false;
  if (this->result_mutex_ != nullptr &&
      xSemaphoreTake(this->result_mutex_, 0) == pdTRUE) {
    if (this->has_new_result_) {
      r = this->pending_;
      this->has_new_result_ = false;
      have = true;
    }
    xSemaphoreGive(this->result_mutex_);
  }
  if (!have)
    return;

#ifdef USE_SENSOR
  if (this->confidence_sensor_ != nullptr)
    this->confidence_sensor_->publish_state(r.best_score * 100.0f);
  if (this->count_sensor_ != nullptr)
    this->count_sensor_->publish_state(r.count);
  if (this->illuminance_sensor_ != nullptr)
    this->illuminance_sensor_->publish_state(r.illuminance);
#endif

  // Debounce: assert immediately, clear only after N consecutive misses.
  if (r.person_raw) {
    this->miss_streak_ = 0;
    if (!this->present_state_)
      this->publish_present_(true);
  } else if (this->present_state_) {
    if (++this->miss_streak_ >= this->clear_after_) {
      this->miss_streak_ = 0;
      this->publish_present_(false);
    }
  } else {
#ifdef USE_BINARY_SENSOR
    // The first valid negative inference initializes Occupancy without firing
    // on_cleared, which is reserved for clearing an asserted presence state.
    if (this->binary_sensor_ != nullptr)
      this->binary_sensor_->publish_state(false);
#endif
  }
}

void PersonDetector::publish_present_(bool present) {
  this->present_state_ = present;
#ifdef USE_BINARY_SENSOR
  if (this->binary_sensor_ != nullptr)
    this->binary_sensor_->publish_state(present);
#endif
  if (present) {
    this->on_detected_.call();
  } else {
    this->on_cleared_.call();
  }
}

void PersonDetector::set_enabled(bool enabled) {
  bool was = this->enabled_.exchange(enabled);
  if (was == enabled)
    return;
  ESP_LOGD(TAG, "Detection %s", enabled ? "enabled" : "disabled");
  if (this->source_ == nullptr)
    return;  // called before setup(); setup() will honor enabled_
  if (enabled) {
    this->source_->start();
    if (this->wake_ != nullptr)
      xSemaphoreGive(this->wake_);
  } else {
    this->source_->stop();
    // Clear presence promptly when the user cuts the camera.
    this->force_clear_.store(true);
    // Drop any result the task already queued so it can't be published on
    // re-enable. enabled_ was set false above, so the guard in run_inference_
    // handles a result finalized concurrently with this disable.
    if (this->result_mutex_ != nullptr) {
      xSemaphoreTake(this->result_mutex_, portMAX_DELAY);
      this->has_new_result_ = false;
      xSemaphoreGive(this->result_mutex_);
    }
  }
}

void PersonDetector::dump_config() {
  ESP_LOGCONFIG(TAG, "Person Detect:");
  ESP_LOGCONFIG(TAG, "  Model: %s", model_to_string(this->model_kind_));
  ESP_LOGCONFIG(TAG, "  Interval: %u ms", (unsigned) this->interval_ms_);
  ESP_LOGCONFIG(TAG, "  Confidence threshold: %.0f%%", this->threshold_ * 100.0f);
  ESP_LOGCONFIG(TAG, "  Clear after: %u consecutive misses", this->clear_after_);
  ESP_LOGCONFIG(TAG, "  Inference task: priority=%u core=%u stack=%u bytes",
                this->task_priority_, this->task_core_,
                (unsigned) this->task_stack_size_);
  ESP_LOGCONFIG(TAG, "  Model runtime PSRAM cost: ~%u bytes",
                (unsigned) this->model_psram_cost_);
  if (this->psram_free_low_ != SIZE_MAX)
    ESP_LOGCONFIG(TAG, "  PSRAM free low-water: %u bytes",
                  (unsigned) this->psram_free_low_);
  if (this->last_infer_ms_ != 0)
    ESP_LOGCONFIG(TAG, "  Last inference: %u ms", (unsigned) this->last_infer_ms_);
  ESP_LOGCONFIG(TAG, "  Capture failures: %u", (unsigned) this->capture_failures_);
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Presence", this->binary_sensor_);
#endif
  if (this->is_failed())
    ESP_LOGE(TAG, "  Component failed to set up");
}

}  // namespace person_detect
}  // namespace esphome
