#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/camera/camera.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#include "frame_source.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace esphome {
namespace person_detect {

enum class Model : uint8_t {
  PEDESTRIAN = 0,
};

// Result produced by one inference, handed from the task to the main loop.
struct InferenceResult {
  bool person_raw{false};   // best-box score >= threshold
  float best_score{0.0f};   // 0..1
  uint8_t count{0};         // boxes >= threshold
  uint32_t infer_ms{0};     // inference wall time
  float illuminance{0.0f};  // whole-frame brightness, 0..100%
};

class PersonDetector : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // Run after the camera has set itself up.
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_camera(camera::Camera *camera) { this->camera_ = camera; }
  // Inject a raw frame source (e.g. the esp_video CSI backend). Takes priority
  // over camera_id: when set, the detector consumes typed RGB frames directly
  // and no ESPHome camera / JPEG decode is involved.
  void set_frame_source(FrameSource *source) { this->source_ = source; }
  void set_interval(uint32_t interval_ms) { this->interval_ms_ = interval_ms; }
  void set_confidence_threshold(float threshold) { this->threshold_ = threshold; }
  void set_clear_after(uint8_t clear_after) { this->clear_after_ = clear_after; }
  void set_model(Model model) { this->model_kind_ = model; }
  void set_task_priority(uint8_t priority) { this->task_priority_ = priority; }
  void set_task_stack_size(uint32_t stack) { this->task_stack_size_ = stack; }
  void set_task_core(uint8_t core) { this->task_core_ = core; }

#ifdef USE_BINARY_SENSOR
  void set_binary_sensor(binary_sensor::BinarySensor *bs) { this->binary_sensor_ = bs; }
#endif
#ifdef USE_SENSOR
  void set_confidence_sensor(sensor::Sensor *s) { this->confidence_sensor_ = s; }
  void set_count_sensor(sensor::Sensor *s) { this->count_sensor_ = s; }
  // Ambient light estimated from mean frame brightness (relative, 0..100%).
  void set_illuminance_sensor(sensor::Sensor *s) { this->illuminance_sensor_ = s; }
#endif

  // Privacy toggle. When disabled the camera is idled and no inference runs.
  void set_enabled(bool enabled);
  bool is_enabled() const { return this->enabled_.load(); }

  void add_on_detected_callback(std::function<void()> &&cb) {
    this->on_detected_.add(std::move(cb));
  }
  void add_on_cleared_callback(std::function<void()> &&cb) {
    this->on_cleared_.add(std::move(cb));
  }

 protected:
  static void task_entry(void *param);
  void task_loop_();
  bool create_model_();
  void run_inference_(const FrameView &frame);
  void publish_present_(bool present);  // main loop only

  // Config
  camera::Camera *camera_{nullptr};
  uint32_t interval_ms_{1500};
  float threshold_{0.6f};
  uint8_t clear_after_{3};
  Model model_kind_{Model::PEDESTRIAN};
  uint8_t task_priority_{2};
  uint32_t task_stack_size_{8192};
  uint8_t task_core_{1};

#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *binary_sensor_{nullptr};
#endif
#ifdef USE_SENSOR
  sensor::Sensor *confidence_sensor_{nullptr};
  sensor::Sensor *count_sensor_{nullptr};
  sensor::Sensor *illuminance_sensor_{nullptr};
#endif

  // Runtime
  FrameSource *source_{nullptr};
  void *model_{nullptr};  // PedestrianDetect* (kept opaque; cast in .cpp)
  TaskHandle_t task_handle_{nullptr};
  SemaphoreHandle_t wake_{nullptr};       // released when (re)enabled
  SemaphoreHandle_t result_mutex_{nullptr};
  std::atomic<bool> enabled_{true};

  // Task -> main-loop handoff (guarded by result_mutex_)
  bool has_new_result_{false};
  InferenceResult pending_{};

  // Debounce / publish state (main loop only)
  bool present_state_{false};
  bool presence_initialized_{false};
  uint8_t miss_streak_{0};
  std::atomic<bool> force_clear_{false};

  // Diagnostics
  size_t psram_free_low_{SIZE_MAX};  // low-water of PSRAM free (arena proxy)
  size_t model_psram_cost_{0};
  uint32_t last_infer_ms_{0};
  uint32_t capture_failures_{0};

  CallbackManager<void()> on_detected_;
  CallbackManager<void()> on_cleared_;
};

}  // namespace person_detect
}  // namespace esphome
