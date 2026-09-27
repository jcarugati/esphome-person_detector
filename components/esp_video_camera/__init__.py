"""esp_video_camera — ESP32-P4 MIPI-CSI raw frame source for person_detect.

Drives a CSI sensor (SC2356 on the reTerminal D1001) through Espressif's
esp_video (V4L2) + ISP stack and the PPA, and exposes an
esphome::person_detect::FrameSource that yields upright RGB888 frames. Wire it to
person_detect via `frame_source_id`.

This owns the camera hardware, unlike the ESPHome-camera JPEG path — it exists
because stock ESPHome has no configurable MIPI-CSI camera platform yet.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.components.esp32 import (
    VARIANT_ESP32P4,
    add_idf_component,
    add_idf_sdkconfig_option,
    get_esp32_variant,
)
from esphome.components.person_detect import FrameSource
from esphome.const import (
    CONF_ADDRESS,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_I2C_ID,
    CONF_RESET_PIN,
    CONF_RESOLUTION,
    CONF_ROTATION,
    CONF_SCL,
    CONF_SDA,
)

CODEOWNERS = ["@zacs"]
DEPENDENCIES = ["person_detect", "esp32"]

esp_video_camera_ns = cg.esphome_ns.namespace("esp_video_camera")
# Inherits person_detect::FrameSource so person_detect's use_id(FrameSource)
# accepts it.
EspVideoCamera = esp_video_camera_ns.class_(
    "EspVideoCamera", cg.Component, FrameSource
)

# esp_video (pulls esp_cam_sensor / esp_cam_ctlr / esp_driver_isp transitively).
# Pin for reproducibility; bump if the esp_video_init API changes. 2.x tracks
# current ESP-IDF (5.5.x) and ships the SC202CS driver; the old 0.9.0 pin bundled
# an esp_cam_sensor that still used the SC2356 name and predates IDF 5.5.4.
ESP_VIDEO_COMPONENT = "espressif/esp_video"
ESP_VIDEO_REF = "2.2.0"

CONF_POWERDOWN_PIN = "powerdown_pin"
CONF_ENABLE_PIN = "enable_pin"
CONF_I2C_PORT = "i2c_port"
CONF_EXPOSURE = "exposure"
CONF_GAIN = "gain"
CONF_SENSOR = "sensor"
CONF_SWAP_RGB = "swap_rgb"
CONF_FRAME_BUFFER_COUNT = "frame_buffer_count"
CONF_IMU = "imu"
CONF_SNAPSHOT = "snapshot"
CONF_AUTO_WHITE_BALANCE = "auto_white_balance"

# rotation: auto reads an accelerometer once at boot to keep people upright.
ROTATION_AUTO = "auto"

# The reTerminal D1001's camera is marketed as "SC2356"; the esp_cam_sensor
# driver for that part is named SC202CS (SC2356 is the same die).
SENSOR_SC2356 = "sc2356"

# resolution -> (width, height, esp_cam_sensor default-format Kconfig choice).
# The SC202CS RAW8 1280x720@30 mode is the sensor driver's default; we select it
# explicitly so the capture geometry is deterministic.
RESOLUTIONS = {
    "1280x720": (
        1280,
        720,
        "CONFIG_CAMERA_SC202CS_MIPI_DEFAULT_FMT_RAW8_1280X720_30FPS",
    ),
}

ROTATIONS = [0, 90, 180, 270]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EspVideoCamera),
        # --- Sensor SCCB (I2C). Two mutually-exclusive ways to reach it: ---
        # (a) Share an existing ESPHome `i2c:` bus by id. Use this on a board that
        #     already uses both P4 I2C controllers (e.g. a display + touch build
        #     where the SCCB shares the touch bus wires): no new master is
        #     installed; the sensor joins the referenced bus as another device.
        #     When set, `sda`/`scl`/`i2c_port` below are ignored.
        cv.Optional(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
        # (b) Install our own SCCB master on raw pins/controller (standalone).
        #     reTerminal D1001: SDA=GPIO37, SCL=GPIO38.
        cv.Optional(CONF_SDA, default=37): cv.int_range(min=0, max=56),
        cv.Optional(CONF_SCL, default=38): cv.int_range(min=0, max=56),
        # Dedicated I2C controller for the (a-off) standalone master. MUST differ
        # from the port an ESPHome `i2c:` bus uses (that one is port 0 when
        # declared first, e.g. the D1001 expander bus), else the install collides
        # and the sensor is never detected. P4 has controllers 0 and 1.
        cv.Optional(CONF_I2C_PORT, default=1): cv.int_range(min=0, max=1),
        # SCCB clock. With a shared bus (a) the new i2c-master API sets this
        # per-device, so it's independent of other devices' speeds (e.g. a 400kHz
        # touch controller on the same wires).
        cv.Optional(CONF_FREQUENCY, default="100kHz"): cv.All(
            cv.frequency, cv.int_range(min=1)
        ),
        # Power/reset control lines. On the D1001 these are on the XL9535 I2C GPIO
        # expander — declare an `xl9535:` hub and pass its pins here. Use
        # `inverted:` to match each line's active level.
        cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_POWERDOWN_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_SENSOR, default=SENSOR_SC2356): cv.enum(
            {SENSOR_SC2356: SENSOR_SC2356}, lower=True
        ),
        cv.Optional(CONF_RESOLUTION, default="1280x720"): cv.enum(
            {k: k for k in RESOLUTIONS}, lower=True
        ),
        # Explicit 0/90/180/270, or "auto" to pick once at boot from an IMU.
        cv.Optional(CONF_ROTATION, default=0): cv.Any(
            cv.one_of(ROTATION_AUTO, lower=True),
            cv.one_of(*ROTATIONS, int=True),
        ),
        # Optional accelerometer for rotation: auto (D1001: LSM6DS3TR @ 0x6A on
        # the expander I2C bus). One read at boot; no polling.
        cv.Optional(CONF_IMU): cv.Schema(
            {
                cv.Required(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
                cv.Optional(CONF_ADDRESS, default=0x6A): cv.int_range(
                    min=0, max=0x7F
                ),
            }
        ),
        cv.Optional(CONF_SWAP_RGB, default=False): cv.boolean,
        # Serve GET /snapshot.jpg (HW JPEG of the detector's frame) through the
        # ESPHome web_server, inheriting its auth. Needs `web_server:`.
        cv.Optional(CONF_SNAPSHOT, default=False): cv.boolean,
        # Gray-world white balance driven from the frame mean into the ISP
        # red/blue gains (for sensors whose IPA AWB finds no white patches).
        cv.Optional(CONF_AUTO_WHITE_BALANCE, default=False): cv.boolean,
        cv.Optional(CONF_FRAME_BUFFER_COUNT, default=2): cv.int_range(min=2, max=4),
        # Sensor exposure/gain in raw sensor units. Omit (or "auto") to let the
        # driver pick a bright default — the SC202CS powers up at its minimum
        # exposure, which is a near-black frame. Raise/lower to suit the room.
        cv.Optional(CONF_EXPOSURE, default="auto"): cv.Any(
            cv.one_of("auto", lower=True), cv.int_range(min=0)
        ),
        cv.Optional(CONF_GAIN, default="auto"): cv.Any(
            cv.one_of("auto", lower=True), cv.int_range(min=0)
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def _final_validate(config):
    # Unlike person_detect (SoC-agnostic), this backend is P4-only silicon: it
    # uses the MIPI-CSI controller, the ISP, and the PPA, none of which exist on
    # other ESP32s. Fail with a clear message instead of an obscure build error.
    variant = get_esp32_variant()
    if variant != VARIANT_ESP32P4:
        raise cv.Invalid(
            f"esp_video_camera needs an ESP32-P4 (MIPI-CSI + ISP + PPA), but this "
            f"device is '{variant}'. On other targets, drive person_detect with a "
            f"JPEG camera via camera_id instead of this backend."
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sccb_pins(config[CONF_SDA], config[CONF_SCL]))
    cg.add(var.set_sccb_port(config[CONF_I2C_PORT]))
    cg.add(var.set_sccb_freq(int(config[CONF_FREQUENCY])))
    # Share an existing ESPHome i2c bus for the SCCB instead of installing a new
    # master (display+touch boards where both P4 controllers are already used).
    if CONF_I2C_ID in config:
        cg.add(var.set_sccb_bus(await cg.get_variable(config[CONF_I2C_ID])))

    if CONF_ENABLE_PIN in config:
        cg.add(var.set_enable_pin(await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])))
    if CONF_POWERDOWN_PIN in config:
        cg.add(
            var.set_powerdown_pin(
                await cg.gpio_pin_expression(config[CONF_POWERDOWN_PIN])
            )
        )
    if CONF_RESET_PIN in config:
        cg.add(var.set_reset_pin(await cg.gpio_pin_expression(config[CONF_RESET_PIN])))

    width, height, mode_kconfig = RESOLUTIONS[config[CONF_RESOLUTION]]
    cg.add(var.set_capture_size(width, height))
    if config[CONF_ROTATION] == ROTATION_AUTO:
        cg.add(var.set_auto_rotation(True))  # rotation picked from the IMU at boot
    else:
        cg.add(var.set_rotation(config[CONF_ROTATION]))
    if CONF_IMU in config:
        imu = config[CONF_IMU]
        imu_bus = await cg.get_variable(imu[CONF_I2C_ID])
        cg.add(var.set_imu(imu_bus, imu[CONF_ADDRESS]))
    cg.add(var.set_swap_rgb(config[CONF_SWAP_RGB]))
    cg.add(var.set_frame_buffer_count(config[CONF_FRAME_BUFFER_COUNT]))
    cg.add(var.set_snapshot(config[CONF_SNAPSHOT]))
    cg.add(var.set_auto_white_balance(config[CONF_AUTO_WHITE_BALANCE]))
    # -1 => auto (driver picks a bright default) for exposure/gain.
    exposure = config[CONF_EXPOSURE]
    cg.add(var.set_exposure(-1 if exposure == "auto" else exposure))
    gain = config[CONF_GAIN]
    cg.add(var.set_gain(-1 if gain == "auto" else gain))

    # esp_video managed component + build-time selection of the CSI/ISP path and
    # the sensor. Kconfig symbol names verified against espressif/esp-video-components
    # (esp_video/Kconfig and esp_cam_sensor/sensors/sc202cs/Kconfig.sc202cs).
    add_idf_component(name=ESP_VIDEO_COMPONENT, ref=ESP_VIDEO_REF)
    # MIPI-CSI + ISP video devices default to y on the P4, but the ISP *pipeline
    # controller* — needed to demosaic the sensor's RAW8 Bayer output into RGB —
    # does not, so enable it (and the ISP device it depends on) explicitly.
    add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE", True)
    add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE", True)
    add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER", True)
    if config[CONF_SENSOR] == SENSOR_SC2356:
        # Enable the SC202CS driver AND its MIPI auto-detect: esp_video_init only
        # probes sensors whose auto-detect fn is registered, so without this no
        # sensor is found and no /dev/video0 is created (the v0.1.5 failure).
        add_idf_sdkconfig_option("CONFIG_CAMERA_SC202CS", True)
        add_idf_sdkconfig_option(
            "CONFIG_CAMERA_SC202CS_AUTO_DETECT_MIPI_INTERFACE_SENSOR", True
        )
        add_idf_sdkconfig_option(mode_kconfig, True)

    cg.add_define("USE_ESP_VIDEO_CAMERA")
