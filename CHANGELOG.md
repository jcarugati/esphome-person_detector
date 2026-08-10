# Changelog

## Unreleased

- Update `espressif/pedestrian_detect` to `0.3.2` (ESP-DL 3.3.x) so builds with
  ESPHome 2026.7.0's `runtime_image` dependency resolve without the incompatible
  ESP-DSP 1.7.0 pin.

## v0.4.0 — ambient light sensor + privacy-switch boot-gate fix

**Ambient light sensor.** The device has no dedicated light sensor, but the
detector already computes a whole-frame brightness value each cycle (the
blank-frame sanity probe). Expose it as a new optional `type: illuminance` on the
`person_detect` sensor platform: a relative 0–100 % ambient-light reading derived
from the mean frame luma. Because the sensor runs at fixed exposure/gain, mean
brightness tracks room light monotonically, so it's useful for "is it dark?"
automations even though it isn't calibrated lux. The probe was upgraded from a raw
byte mean to a proper per-pixel Rec.601 luma (handling RGB888/RGB565/grayscale),
which also makes the debug `luma[min max mean]` line meaningful. It's a
whole-frame average (a bright lamp in view skews it) and clips toward 100 % in
bright scenes. While the privacy switch idles the camera the sensor publishes NaN,
so Home Assistant shows *unknown* rather than a misleading 0 %.

**Privacy-switch boot gate.** The `person_detect` switch only overrode
`write_state()` — it never applied its `restore_mode` at boot, so on a fresh boot
the detector's `enabled_` stayed on regardless of what the switch was restored to:
a switch restored to "off" didn't actually idle the camera or stop inference
(toggling it by hand worked; only the boot-time restore was skipped). The switch
now has a `setup()` that reads `get_initial_state_with_restore_mode()` and applies
it via `write_state()`, mirroring stock ESPHome switches. The detector runs at
`LATE` priority (after the switch), so its `setup()` sees the correct `enabled_`
before deciding whether to start the camera; when restored to "off" it publishes a
definite occupancy=false instead of leaving the binary sensor unknown. (This fix
was slated for a v0.3.2 tag whose build failed on a bad API name; it ships here.)

## v0.3.1 — fix inference hang from the frame flush

The v0.2.1 frame flush used an unbounded `while (VIDIOC_DQBUF succeeds)` loop to
drop stale buffers. On esp_video the driver hands a buffer straight back as fast
as we requeue it, so that loop never exits — the inference task spun forever
inside `acquire()` and produced no detections (no `inference` lines, no capture
timeouts, `Capture failures: 0`). The flush shipped compile-only in v0.2.1, so
this was its first run on hardware.

Bound the flush to the ring size (`frame_buffer_count`, at most a handful of
`DQBUF`s), keeping the live-frame benefit without the hang. `acquire()` is now
fully bounded (the flush by count, the frame wait by timeout).

## v0.3.0 — share an existing I2C bus for the SCCB

For integrating into a full board (display + touchscreen), where both ESP32-P4
I2C controllers are already used and the camera SCCB physically shares the touch
bus wires (GPIO37/38): a new optional `i2c_id:` on `esp_video_camera` points the
SCCB at an existing ESPHome `i2c:` bus instead of installing a second master
(which would collide at runtime and the sensor would never probe). When set, the
sensor joins that bus as another device via its `i2c_master_bus_handle_t`
(`init_sccb=false`), alongside e.g. the touch controller. `i2c_id` and the raw
`sda`/`scl`/`i2c_port` path are mutually exclusive; the sensor keeps its own
per-device `frequency` on the shared bus, so it coexists with a 400 kHz touch
controller. Standalone (own-master) behavior is unchanged.

Verify on hardware with both the camera and the touch controller active.

## v0.2.1 — run inference on live frames

`acquire()` did a single FIFO `VIDIOC_DQBUF`, so with only two capture buffers
and one grab per `interval` it handed back a frame captured a couple of cycles
(~2–3 s) earlier rather than the live one. Flush every already-queued buffer
before grabbing, so inference always runs on the freshest frame. (This bounds
staleness to one frame period; it does not change the model's behavior on
out-of-distribution scenes — a pedestrian detector aimed at a blank ceiling can
still produce spurious low-confidence boxes.)

## v0.2.0 — first hardware-verified release (reTerminal D1001)

The full pipeline now works end-to-end on real D1001 hardware: MIPI-CSI capture
through esp_video / ISP / PPA, sensor exposure and gain, one-time IMU
auto-rotation, and ESP-DL pedestrian detection driving the occupancy
`binary_sensor`. This milestone rolls up the v0.1.x bring-up series.

Highlights since v0.1.0:

- Camera backend verified on hardware — the SC202CS is detected over SCCB on its
  own I2C controller, captured with a non-blocking DQBUF loop, and exposure/gain
  are set (and periodically re-asserted) over V4L2, since the sensor otherwise
  powers up near-black.
- `rotation: auto` — a single accelerometer read at boot (the on-board
  LSM6DS3TR) picks the orientation, calibrated so people are upright in both
  landscape and portrait. An explicit `rotation:` still wins, and the IMU is
  optional.
- General-purpose framing — the detector is SoC-agnostic and only warns off-P4;
  `esp_video_camera` is the P4-only MIPI-CSI backend. Model, sensor, frame-source
  backend, board, and target SoC are all extension points.
- Docs — README, DESIGN, and BRINGUP updated to the verified state, with the
  ESP-DL model and its alternatives (e.g. face detection for close range) linked.

See the v0.1.x entries below for the detailed bring-up history.

## v0.1.13 — calibrate auto-rotation Y axis

The v0.1.12 auto-rotation was right in portrait (gravity +X → 90°) but flipped in
landscape (gravity −Y picked 180° when it needed 0°). Fix the inverted Y-axis
branch; the mapping is now calibrated against both orientations on the D1001:
gravity **−Y → 0** (landscape), **+X → 90** (portrait), **+Y → 180**, **−X → 270**.

## v0.1.12 — optional auto-rotation from an accelerometer

`rotation:` now also accepts **`auto`**: a single accelerometer read at boot
picks 0/90/180/270 so a standing person is upright in whatever orientation the
device is in — no polling, and an explicit `rotation:` always wins. Add an
optional `imu:` block (`i2c_id` + `address`, default `0x6A`) pointing at a
LSM6DS-family accelerometer; the D1001's onboard LSM6DS3TR sits on the expander
I2C bus. Gravity-based orientation needs the device roughly upright (wall/desk
mount); lying flat it holds the last orientation. The raw accel values and the
chosen rotation are logged so the axis/sign mapping can be calibrated per board.

## v0.1.11 — keep exposure applied (consistent detection)

v0.1.10 proved the fix works when it lands (`sensor exposure=811`, frame
`mean≈130`, detections at 0.85–0.89), but the set right at `STREAMON` didn't
always take — some boots stayed near-black, needing the subject in bright light.

- **Re-assert exposure/gain periodically** from `acquire()` (~every 2 s) so a
  missed initial set or any ISP drift self-corrects within a couple seconds —
  the intended value is remembered even if the first set fails.
- **Try the generic `V4L2_CID_GAIN` first** (which the SC202CS accepts) instead
  of `V4L2_CID_ANALOGUE_GAIN`, removing the `ctrl id=9e0903 is not supported`
  error the sensor logged on every boot.

Tune with `exposure:` / `gain:` on `esp_video_camera` if your room needs it.

## v0.1.10 — exposure via extended controls + landscape default

The v0.1.9 exposure code silently did nothing — no `sensor exposure=` line ever
logged and the frame stayed at the sensor minimum (`mean≈11`), because esp_video
exposes sensor controls through the **extended** control API, not the legacy one
this code used.

- Set exposure/gain via **`VIDIOC_S_EXT_CTRLS`** (matching esp_video's own
  examples) instead of `VIDIOC_S_CTRL`; range query is best-effort with a known
  fallback (SC202CS exposure ~8..1244). Confirmed on hardware that the pipeline
  is otherwise healthy — a flashlight drove the frame to a real image
  (`max=248 mean=52`), so this is purely an exposure fix.
- Default the reference board to **`rotation: 0` (landscape)** — with the D1001
  in its default landscape orientation, the previous `rotation: 90` rotated a
  standing person onto their side, which the pedestrian model won't detect. Use
  90/270 only for a portrait mount.

## v0.1.9 — set sensor exposure/gain (fix near-black frames)

The v0.1.8 probe showed `frame[min=8 max=16 mean=11]` — the pipeline works but
the image is near-black: the SC202CS powers up at its **minimum exposure**
(`0x08`) and only auto-exposes if `esp_ipa` has a per-sensor tuning config, which
isn't wired up, so exposure never rises.

- After `STREAMON`, set the sensor **exposure/gain via V4L2 controls**
  (`V4L2_CID_EXPOSURE`/`_ABSOLUTE`, `V4L2_CID_ANALOGUE_GAIN`/`GAIN`) — default to
  ~70% of the exposure range and a moderate gain, and log each control's range.
- New `esp_video_camera` options `exposure:` / `gain:` (raw sensor units, or
  `auto`) to tune for the room.

## v0.1.8 — frame-content probe (diagnose blank vs valid frames)

v0.1.7 got the pipeline fully running — frames captured, PPA rotate, inference
at ~76 ms — but detection was always `boxes=0 best=0.00`, which typically means
the analyzed frame is blank rather than "no person present." Add a cheap
min/max/mean subsample of each frame to the inference log so a black frame (ISP
not exposing the RAW sensor) is distinguishable from a valid image the model
found no person in. Diagnostic only; no behavior change.

## v0.1.7 — capture frames with a blocking DQBUF (no select/poll)

v0.1.6 detected the SC202CS, opened `/dev/video0`, and started streaming, but
every capture timed out: `esp_video`'s V4L2 devices don't register ESP-IDF's VFS
`select()`/`poll()` hooks, so the `select()`-gated `DQBUF` in `acquire()` never
saw the fd become readable and always hit the timeout — even though frames were
being produced. (Espressif's own `capture_stream` example uses a plain blocking
`DQBUF` for exactly this reason.)

- Open the device **non-blocking** and poll `VIDIOC_DQBUF` (EAGAIN → short
  `vTaskDelay`) until a filled buffer is ready or the timeout elapses. Drops the
  `select()` path entirely.
- Warn if the driver couldn't honor the requested RGB565 capture format (the PPA
  rotate/convert pass assumes RGB565 input).

## v0.1.6 — correct sensor driver (SC202CS) + current esp_video

v0.1.5 put the SCCB on its own controller but the SC2356 still wasn't detected:
`esp_video_init` returned OK, logged nothing, and created no `/dev/video0`. Root
cause was the sensor driver never being compiled/registered — two version issues,
verified against `espressif/esp-video-components` rather than guessed:

- **Driver rename.** esp_cam_sensor renamed this part **SC2356 → SC202CS** (same
  die). Our `CONFIG_CAMERA_SC2356*` keys didn't exist in the resolved driver, and
  ESP-IDF silently ignores unknown sdkconfig keys, so no sensor driver was built.
  Now sets `CONFIG_CAMERA_SC202CS` + the `SC202CS` RAW8 1280×720 default-format.
- **Auto-detect.** `esp_video_init` only probes sensors whose auto-detect fn is
  registered — now enables `CONFIG_CAMERA_SC202CS_AUTO_DETECT_MIPI_INTERFACE_SENSOR`.
- **Bumped `espressif/esp_video` 0.9.0 → 2.2.0.** The old pin predated ESP-IDF
  5.5.4 (what ESPHome 2026.6 uses) and the SC202CS naming. The `esp_video_init`
  CSI/SCCB API is unchanged across the bump, so no component C++ changed.
- Also enables `CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE` (the ISP pipeline
  controller, which demosaics the sensor's RAW8 Bayer to RGB, depends on it).

## v0.1.5 — camera SCCB on its own I2C controller (sensor now detectable)

First hardware boot of v0.1.4 got the whole ESPHome app up (Wi-Fi AP, xl9535,
PSRAM all fine) but `esp_video_camera` failed with
`open(/dev/video0): No such file or directory` — `esp_video_init` returned OK yet
registered no capture device, i.e. the SC2356 was never detected over SCCB.

- **Fix:** the sensor SCCB was hard-coded to **I2C port 0**, the same controller
  ESPHome assigns to the first `i2c:` bus — on the D1001 that's the GPIO20/21
  expander bus. Two masters on one controller collide, the sensor never probes,
  and no `/dev/video0` is created. The SCCB now uses a **dedicated controller**
  via a new `i2c_port:` option (default **1**), independent of the expander bus.
- Clearer diagnostics: log the SCCB port at setup, and on an `open()` miss say
  explicitly that the sensor wasn't detected (with what to check).

## v0.1.4 — camera pinout verified against the D1001 schematic

Cross-checked the camera wiring against the official reTerminal D1001 schematic
(Seeed, KiCad `LCD_CAM` sheet) instead of trusting derived values:

- **Confirmed:** SCCB on **GPIO37 (SDA) / GPIO38 (SCL)** (shared peripheral I2C,
  camera addr 0x10); expander controls **CAM_EN=EXP_GPO1 (xl9535 pin 1),
  CAM_PWDN=EXP_GPO3 (pin 3), CAM_RST=EXP_GPO9=P1.1 (pin 11)**; MCLK is a board
  24 MHz source (no xclk driven by the SoC).
- **Fix:** the reset line is active-low at the sensor and the driver pulses the
  expander pin low→high to release, so the expander pin must **not** be
  `inverted:` — removing it (v0.1.3 would have held the sensor in reset).

## v0.1.3 — adopt canonical reTerminal D1001 board config

Replace hand-derived board bring-up with the known-good reTerminal D1001 config
from [zacs/espcontrol](https://github.com/zacs/espcontrol)
(`devices/seeed-esp32-p4-reterminal-d1001/device/device.yaml`), fixing several
guessed values at once:

- `cpu_frequency: 360MHZ`, and framework `advanced: execute_from_psram: true` +
  experimental features + PSRAM/hosted sdkconfig (`ESP_HOSTED_DFLT_TASK_FROM_SPIRAM`,
  `MBEDTLS_EXTERNAL_MEM_ALLOC`, `SDMMC_HOST_DEFAULT`, 60 s task WDT).
- `esp32_hosted: sdio_frequency: 20MHz`.
- Camera power/reset expander corrected to **`xl9535` @ 0x20 on I2C GPIO20/GPIO21**
  (was a guessed `pca9554` on pins that collided with the ESP-Hosted SDIO bus).
- **Drop the custom partition CSV** — `flash_size: 16MB` lets ESPHome generate a
  self-consistent table + factory image (removes the offset class of bug in v0.1.2).

All device-config; no component code changed.

## v0.1.2 — partition-offset fix (no bootable app)

- **Fix `No bootable app partitions` / `invalid magic byte` on boot.** The
  two-OTA partition tables (firmware + example) started the first app slot at
  0x20000, but ESPHome's factory image writes the app at the standard 0x10000,
  so the bootloader found an empty slot. Realign to the standard ESP-IDF two-OTA
  layout (app at 0x10000). Device-config detail, not a component change.

## v0.1.1 — pre-rev3 ESP32-P4 boot fix

- **Fix boot-loop / `Illegal instruction` at bootloader entry** on pre-rev3
  ESP32-P4 silicon (ROM `esp32p4-eco2`; the reTerminal D1001 ships this): the
  build defaulted to rev3 production silicon. The board configs now set
  `esp32: engineering_sample: true` (engineering board + safe 360 MHz CPU). This
  is a per-board device-config detail, not a component change.

## v0.1.0 — initial release

`person_detect`: general-purpose on-device human-presence detection for ESPHome,
exposed to Home Assistant as an occupancy `binary_sensor`. Verified to **compile
and link for ESP32-P4** in CI (ESPHome 2026.6.0 / ESP-IDF 5.5.4). The reTerminal
D1001 is the first supported board. Not yet hardware-verified — see `BRINGUP.md`.

### Added
- `person_detect` component: ESP-DL `PedestrianDetect` (224×224 INT8) on a
  low-priority pinned FreeRTOS task; debounced occupancy `binary_sensor`,
  optional confidence/count `sensor`s, privacy `switch`, and
  `on_person_detected` / `on_person_cleared` triggers.
- `esp_video_camera` component: generic ESP32-P4 MIPI-CSI capture backend
  (`esp_video`/V4L2 + ISP → PPA hardware rotate + RGB565→RGB888 → ESP-DL);
  sensor selectable via Kconfig (first sensor: SC2356), `rotation:`/`swap_rgb:`.
- Pluggable `FrameSource` seam: `frame_source_id` (raw RGB, `esp_video_camera`)
  and `camera_id` (ESPHome JPEG camera), mutually exclusive.
- Prebuilt reference image for the first supported board (reTerminal D1001,
  `firmware/reterminal_d1001.yaml`) with Wi-Fi AP + captive portal / Improv
  provisioning, published via the Firmware workflow.
- CI: validate (both frame-source paths) + best-effort P4 compile; Firmware
  workflow builds a flashable image and attaches it to tagged releases.

### Measured (CI link, CSI build)
- Image ~2.29 MB (Flash 13.6% of 16 MB; `.rodata`/model 596 KB); internal RAM
  5.8%. Capture/PPA/tensor-arena buffers are PSRAM at runtime.

### Known / verify on hardware
- `rotation:` direction, `swap_rgb:` channel order, expander pin polarity + I2C
  bus pins, and the `espressif/esp_video` version pin (`0.9.0`).
