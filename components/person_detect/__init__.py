"""person_detect — on-device human presence detection for ESPHome.

Consumes frames from a pluggable FrameSource (by ID), runs a quantized
pedestrian-detection model (Espressif ESP-DL) entirely on-device, and drives a
binary_sensor / sensors / triggers. No frame ever leaves the device. Verified on
ESP32-P4; other ESP-DL targets are allowed but experimental (see README).
"""

import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components.esp32 import (
    VARIANT_ESP32P4,
    add_idf_component,
    get_esp32_variant,
)

_LOGGER = logging.getLogger(__name__)
from esphome.const import CONF_ID, CONF_MODEL, CONF_TRIGGER_ID

CODEOWNERS = ["@zacs"]
DEPENDENCIES = ["esp32"]
# person_detector.h / esphome_camera_source.* always include the camera base
# (esphome/components/camera/camera.h) for the JPEG path, so ensure the camera
# component's sources are compiled even in a CSI-only (frame_source_id) config.
AUTO_LOAD = ["camera"]
MULTI_CONF = True

person_detect_ns = cg.esphome_ns.namespace("person_detect")
PersonDetector = person_detect_ns.class_("PersonDetector", cg.Component)

# ESPHome's modular `camera` framework currently ships a C++-only base
# (esphome::camera::Camera); its Python module does not export a `Camera`
# codegen class to import. Reference the C++ base by namespace so we can
# use_id() a camera source without depending on a non-existent Python symbol.
camera_ns = cg.esphome_ns.namespace("camera")
Camera = camera_ns.class_("Camera")

# Marker base for a raw (typed-RGB) frame source. A backend component (e.g.
# esp_video_camera) subclasses esphome::person_detect::FrameSource; person_detect
# references it by id and consumes frames directly, bypassing the JPEG path.
FrameSource = person_detect_ns.class_("FrameSource")

PersonDetectedTrigger = person_detect_ns.class_(
    "PersonDetectedTrigger", automation.Trigger.template()
)
PersonClearedTrigger = person_detect_ns.class_(
    "PersonClearedTrigger", automation.Trigger.template()
)

# Selectable detection model. One entry today; adding another ESP-DL registry
# model is a single row here plus a case in PersonDetector::create_model().
Model = person_detect_ns.enum("Model", is_class=True)
MODELS = {
    "pedestrian": Model.PEDESTRIAN,
}

# ESP-DL model package on the ESP Component Registry. Pulling it also brings
# esp-dl transitively via its manifest. Pinned for reproducible builds.
PEDESTRIAN_DETECT_COMPONENT = "espressif/pedestrian_detect"
PEDESTRIAN_DETECT_REF = "0.3.2"

CONF_CAMERA_ID = "camera_id"
CONF_FRAME_SOURCE_ID = "frame_source_id"
CONF_INTERVAL = "interval"
CONF_CONFIDENCE_THRESHOLD = "confidence_threshold"
CONF_CLEAR_AFTER = "clear_after"
CONF_TASK_PRIORITY = "task_priority"
CONF_TASK_STACK_SIZE = "task_stack_size"
CONF_TASK_CORE = "task_core"
CONF_ON_PERSON_DETECTED = "on_person_detected"
CONF_ON_PERSON_CLEARED = "on_person_cleared"

# Shared with the binary_sensor / sensor / switch platform files.
CONF_PERSON_DETECT_ID = "person_detect_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PersonDetector),
        # Exactly one frame source (enforced below):
        #   camera_id        -> an ESPHome camera (JPEG frames, decoded on-device)
        #   frame_source_id  -> a raw RGB backend, e.g. esp_video_camera (CSI/ISP)
        cv.Optional(CONF_CAMERA_ID): cv.use_id(Camera),
        cv.Optional(CONF_FRAME_SOURCE_ID): cv.use_id(FrameSource),
        cv.Optional(
            CONF_INTERVAL, default="1500ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_CONFIDENCE_THRESHOLD, default="60%"): cv.percentage,
        cv.Optional(CONF_CLEAR_AFTER, default=3): cv.int_range(min=1, max=255),
        cv.Optional(CONF_MODEL, default="pedestrian"): cv.enum(MODELS, lower=True),
        # Inference task tuning. Priority is deliberately low so the model task
        # can never starve the ESPHome main loop or an LVGL/UI task.
        cv.Optional(CONF_TASK_PRIORITY, default=2): cv.int_range(min=1, max=24),
        cv.Optional(CONF_TASK_STACK_SIZE, default=8192): cv.int_range(
            min=4096, max=65536
        ),
        cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(min=0, max=1),
        cv.Optional(CONF_ON_PERSON_DETECTED): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PersonDetectedTrigger)}
        ),
        cv.Optional(CONF_ON_PERSON_CLEARED): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PersonClearedTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def _require_one_source(config):
    has_camera = CONF_CAMERA_ID in config
    has_raw = CONF_FRAME_SOURCE_ID in config
    if has_camera == has_raw:  # both or neither
        raise cv.Invalid(
            "person_detect needs exactly one frame source: set either "
            "'camera_id' (an ESPHome JPEG camera) or 'frame_source_id' (a raw "
            "backend such as esp_video_camera), not both."
        )
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _require_one_source)


def _final_validate(config):
    # The detector itself is SoC-agnostic (ESP-DL runs on the S3 as well as the
    # P4), so don't hard-fail on the variant. Only the ESP32-P4 path is verified;
    # the built-in esp_video_camera backend is P4-only silicon (CSI/ISP/PPA), so
    # other targets must supply frames via `camera_id` (a JPEG ESPHome camera,
    # e.g. an OV2640 on the S3) — that path is unverified. Warn, don't block.
    variant = get_esp32_variant()
    if variant != VARIANT_ESP32P4:
        _LOGGER.warning(
            "person_detect is verified on ESP32-P4; '%s' is experimental. The "
            "esp_video_camera (MIPI-CSI) backend is P4-only, so on this target "
            "feed frames via camera_id (a JPEG camera such as an OV2640). Expect "
            "slower inference and ensure enough PSRAM for the model + frames.",
            variant,
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_CAMERA_ID in config:
        cam = await cg.get_variable(config[CONF_CAMERA_ID])
        cg.add(var.set_camera(cam))
    if CONF_FRAME_SOURCE_ID in config:
        src = await cg.get_variable(config[CONF_FRAME_SOURCE_ID])
        cg.add(var.set_frame_source(src))

    cg.add(var.set_interval(config[CONF_INTERVAL]))
    cg.add(var.set_confidence_threshold(config[CONF_CONFIDENCE_THRESHOLD]))
    cg.add(var.set_clear_after(config[CONF_CLEAR_AFTER]))
    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_task_priority(config[CONF_TASK_PRIORITY]))
    cg.add(var.set_task_stack_size(config[CONF_TASK_STACK_SIZE]))
    cg.add(var.set_task_core(config[CONF_TASK_CORE]))

    for conf in config.get(CONF_ON_PERSON_DETECTED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_PERSON_CLEARED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    # Pull the ESP-DL pedestrian model (and, transitively, esp-dl) from the
    # ESP Component Registry. Weights land in flash rodata by default.
    add_idf_component(name=PEDESTRIAN_DETECT_COMPONENT, ref=PEDESTRIAN_DETECT_REF)

    cg.add_define("USE_PERSON_DETECT")
