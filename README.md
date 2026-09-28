# esphome-person_detector

On-device human-presence detection for ESPHome.

`person_detect` is a general-purpose ESPHome external component that uses a
camera to decide whether a person is in frame — presence only, not identity —
entirely on-device, and exposes it to Home Assistant as an occupancy
`binary_sensor`.

It's built to be portable and extended: the detection model, the camera sensor,
the frame-source backend, and the target SoC are all swappable (see
[Extending and portability](#extending-and-portability)). The Seeed Studio
reTerminal D1001 (ESP32-P4 + MIPI-CSI SC2356) is simply the first board that's
been verified end-to-end on hardware — a starting point, not the purpose. Other
ESP-DL targets, such as an ESP32-S3 with a DVP camera, fit the same design; they
work today and just need verifying.

## Privacy

Camera inference stays on-device; there is no cloud API, companion server, face
recognition, or biometric storage. Optional authenticated `snapshot` and
`mjpeg_stream` endpoints can expose frames on the local network when explicitly
enabled. An optional `switch` turns detection off and releases the camera, so
flipping it off gives you a hard privacy cut.

## The model

Detection uses Espressif's
[`pedestrian_detect`](https://components.espressif.com/components/espressif/pedestrian_detect)
model (v0.3.0), built on [ESP-DL](https://github.com/espressif/esp-dl),
Espressif's on-device deep-learning library. It's an INT8-quantized full-body
pedestrian detector with a 224×224×3 input, pulled in automatically by the
ESP-IDF component manager — you don't fetch or embed anything by hand.

Two practical consequences worth knowing:

- It looks for whole standing or walking people, not faces. A close-up of your
  face may not trigger it, but a person standing in the room will — so aim the
  camera to see bodies, not head-and-shoulders.
- Inference takes about 75 ms on the ESP32-P4 (it also runs on the S3, slower).

ESP-DL ships a whole [catalog of models](https://github.com/espressif/esp-dl/tree/master/models),
and the right one depends on framing. `pedestrian_detect` (the default) suits
room-scale presence, where you see whole bodies. For close range — a desk, a
doorway, a doorbell, where the camera mostly sees a head and shoulders —
[`human_face_detect`](https://github.com/espressif/esp-dl/tree/master/models/human_face_detect)
tends to work better. It detects that a face is *present*, not whose it is
(that's face *detection*, not *recognition*), so it stays within the
presence-only, no-identity design. Others like the 80-class `coco_detect` also
include a person class.

Today only `pedestrian` is wired up as a `model:` value; adding another ESP-DL
detector (face, COCO, hand, …) is the small extension described in
[Extending and portability](#extending-and-portability) — any model that returns
`dl::detect::result_t` boxes drops into the existing pipeline.

## Requirements

The detector runs anywhere ESP-DL does; these are the practical constraints:

| | |
|---|---|
| SoC | ESP32-P4 is verified. Other ESP-DL targets (e.g. ESP32-S3) are allowed but experimental — a warning, not a build error. See [Extending and portability](#extending-and-portability) |
| Framework | ESP-IDF (no Arduino core on these targets) |
| PSRAM | Required — the model runtime and frame buffers live here |
| Flash | `flash_size: 16MB` — the embedded model makes the app image ~2.3 MB; 16 MB lets ESPHome auto-size an app partition that fits, with no custom partition CSV |

Tested against ESPHome 2026.6.0 / ESP-IDF v5.5.4.

## Quick start

These paths use the reTerminal D1001 because it's the verified board, but the
shape is the same for any board: pull the component, give it a frame source, and
expose a sensor. For a different ESP32-P4 camera board, point `esp_video_camera`
at your sensor and pins; for another SoC, see [Extending and portability](#extending-and-portability).

### Option A — flash the prebuilt D1001 image (fastest)

Grab `reterminal-presence.factory.bin` from the [latest
release](https://github.com/zacs/esphome-person_detector/releases), flash it to
offset `0x0` (the [ESPHome web flasher](https://web.esphome.io) or
`esptool.py --chip esp32p4 write_flash 0x0 …`), then join the "reTerminal
Presence" Wi-Fi AP / captive portal (or Improv over USB) to set your network.

### Option B — start from the example config

[`example/reterminal_d1001.yaml`](example/reterminal_d1001.yaml) is a complete,
compilable D1001 config — board bring-up, camera wiring, and detector. Copy it,
drop in your Wi-Fi, and build:

```bash
pip install "esphome==2026.6.0"
esphome run example/reterminal_d1001.yaml
```

### Option C — add it to an existing ESP32-P4 config

Pull the component in, point the detector at a camera backend, and expose a
sensor. On the D1001 the camera backend is `esp_video_camera` (CSI + ISP →
RGB888); its power and reset lines sit on an XL9535 I²C expander, so copy the
`i2c:`, `xl9535:`, and `esp_video_camera:` blocks from the example for the full
wiring. The detector part is just:

```yaml
external_components:
  - source: github://zacs/esphome-person_detector@v0.4.0   # pin a released tag
    components: [person_detect, esp_video_camera]

person_detect:
  id: presence
  frame_source_id: d1001_cam   # id of your esp_video_camera (see below)

binary_sensor:
  - platform: person_detect
    person_detect_id: presence
    name: "Room Occupied"       # device_class defaults to "occupancy"
```

A quick note on `frame_source_id` vs `camera_id`: use `frame_source_id` to point
at an `esp_video_camera` (the raw MIPI-CSI path, which is what the D1001 uses),
and `camera_id` only if you're feeding a stock ESPHome JPEG `camera:` instead.
Set exactly one — see [Frame sources](#frame-sources).

## Configuration

### `esp_video_camera:` — the ESP32-P4 MIPI-CSI backend

Owns the sensor and hands the detector upright RGB888 frames.

```yaml
esp_video_camera:
  id: d1001_cam
  sda: 37                # sensor SCCB (I²C) pins — D1001: 37/38
  scl: 38
  i2c_port: 1            # SCCB I²C controller; must differ from your `i2c:` bus
  # i2c_id: touch_bus    # OR: share an existing i2c bus instead of sda/scl/i2c_port
  resolution: 1280x720
  rotation: auto         # IMU-picked at boot; or pin 0 / 90 / 180 / 270
  imu:                   # only needed for rotation: auto
    i2c_id: bus_expander
    address: 0x6A        # D1001 LSM6DS3TR
  # snapshot: true        # authenticated GET /snapshot.jpg (needs web_server)
  # mjpeg_stream: true    # authenticated GET /stream.mjpg, lazy while connected
  # auto_white_balance: true  # gray-world correction on JPEGs only
  # exposure: 811        # raw sensor units; omit for a bright auto default
  # gain: 63             # raw sensor units; omit for auto
  # power/reset lines (D1001 wires these to the XL9535 expander):
  enable_pin: { xl9535: cam_expander, number: 1 }
  powerdown_pin: { xl9535: cam_expander, number: 3 }
  reset_pin: { xl9535: cam_expander, number: 11 }
```

- `rotation` accepts `auto` or an explicit `0` / `90` / `180` / `270`. Rotation
  is counter-clockwise, so `90` turns the image 90° CCW (confirmed on the D1001).
  `auto` reads the accelerometer once at boot and picks the orientation that
  keeps a standing person upright, which is handy for a device you reposition; an
  explicit value always wins and is best for a fixed mount. Auto needs the device
  roughly upright — lying flat, gravity is straight down and it holds the last
  orientation.
- `exposure` and `gain` default to a bright value because the SC2356 powers up
  near-black. If a room comes out too dark or too bright, set them (raw sensor
  units; the valid range is logged at boot).
- SCCB bus — by default the camera installs its own I²C master on `sda`/`scl` /
  `i2c_port` (`i2c_port` must differ from every ESPHome `i2c:` bus). On a board
  that already uses both P4 I²C controllers — e.g. a display + touch build where
  the SCCB physically shares the touch bus wires — set `i2c_id:` to an existing
  `i2c:` bus instead: the camera then joins that bus as another device rather
  than installing a second master. `i2c_id` and `sda`/`scl`/`i2c_port` are
  mutually exclusive; the sensor keeps its own `frequency` (per-device on the
  shared bus, so it's independent of e.g. a 400 kHz touch controller on the same
  wires).

### `person_detect:` — the detector

```yaml
person_detect:
  id: presence
  frame_source_id: d1001_cam  # an esp_video_camera; OR camera_id: for a JPEG camera
  interval: 1500ms            # inference cadence (default 1500ms, min 100ms)
  confidence_threshold: 60%   # min score to count a detection (default 60%)
  clear_after: 3              # consecutive misses before clearing (default 3)
  model: pedestrian           # selectable model (default: pedestrian)
  # advanced task tuning (defaults shown):
  task_priority: 2            # keep below a UI/LVGL task
  task_stack_size: 8192
  task_core: 1
  on_person_detected:
    - logger.log: "Person detected"
  on_person_cleared:
    - logger.log: "Room empty"
```

### `binary_sensor` / `sensor` / `switch`

```yaml
binary_sensor:
  - platform: person_detect
    person_detect_id: presence
    name: "Room Occupied"
    filters:
      - delayed_off: 15s        # optional hold-open on top of clear_after

sensor:                          # optional
  - platform: person_detect
    person_detect_id: presence
    type: confidence             # best-box score, 0–100 %
    name: "Presence Confidence"
  - platform: person_detect
    person_detect_id: presence
    type: count                  # boxes over threshold
    name: "People Count"
  - platform: person_detect
    person_detect_id: presence
    type: illuminance            # relative ambient light, 0–100 % (see below)
    name: "Ambient Light"

switch:                          # optional detection on/off (privacy)
  - platform: person_detect
    person_detect_id: presence
    name: "Presence Detection"   # on = detecting (default); off = camera idle, no inference
```

### Ambient light

The `illuminance` sensor is a free by-product of the frames the detector already
pulls — the device has no dedicated light sensor. It reports the mean frame
brightness as a **relative** 0–100 %, not calibrated lux. Because the camera runs
at a fixed exposure and gain, that value tracks room light monotonically, so it's
useful for "is it dark?" automations even though it isn't an absolute reading. Two
things to keep in mind: it's a whole-frame average, so a bright lamp in view skews
it, and it clips toward 100 % in very bright scenes (the fixed exposure is tuned
for indoor light). While the privacy switch idles the camera it reads *unknown*
rather than a misleading 0 %.

## Frame sources

`person_detect` takes frames from exactly one backend:

| Key | Backend | Frames | Use when |
|---|---|---|---|
| `frame_source_id` | `esp_video_camera` (this repo) | raw RGB888 (CSI + ISP + PPA) | ESP32-P4 MIPI-CSI sensors (SC2356 / D1001) — the path the D1001 uses |
| `camera_id` | a stock ESPHome `camera:` | JPEG, decoded on-device | a board that already exposes a JPEG camera to ESPHome |

Both sit behind one `FrameSource` seam ([`DESIGN.md`](DESIGN.md) §2). The
`camera_id` path exists for composability, but ESPHome (as of 2026.6.0) ships no
user-configurable MIPI-CSI camera platform, so on the P4 you use
`esp_video_camera`.

## Coexisting with an LVGL / display config

Inference runs on its own low-priority, pinned FreeRTOS task (`task_priority`,
`task_core`), publishes only from the main loop, and keeps every large buffer in
PSRAM, so it stays out of the way of an 800×1280 MIPI-DSI LVGL UI. If your UI
task is pinned to a core, pin detection to the other with `task_core`.

## Tuning

- `confidence_threshold` — raise it (70–80 %) to cut false positives from
  clutter, or lower it (45–55 %) if real people are missed.
- `interval` — presence doesn't need video framerates; 1–2 s is a good default.
  Inference is ~75 ms on the P4, so the task idles most of the interval.
- `clear_after` + `delayed_off` — `clear_after` debounces single-frame misses; a
  `delayed_off:` filter adds occupancy-style hold-open.
- Exposure and lighting — the SC2356 is a small 2 MP sensor. If confidence sags
  in dim rooms, raise `exposure`/`gain`, and avoid strong backlight (a window
  behind the subject) that silhouettes people.

## Diagnostics

With `logger: level: DEBUG`, each inference logs its time, frame dimensions, box
count, best score, a min/max/mean frame-brightness probe, and free PSRAM.
`dump_config` prints the model, cadence, threshold, task placement, PSRAM
cost/low-water, last inference time, and capture-failure count. On boot,
`esp_video_camera` logs the SCCB port, the applied exposure/gain with their
ranges, and — for `rotation: auto` — the raw accelerometer reading and the
rotation it chose.

## Memory / flash budget

All large buffers live in PSRAM; internal SRAM use is limited to the inference
task stack. Measured static footprint (CI `esphome compile`, ESP32-P4, ESPHome
2026.6.0 / ESP-IDF 5.5.4, `esp_video_camera` + `person_detect` + ESP-DL +
pedestrian model):

| Metric | Value |
|---|---|
| Total image | ~2.29 MB (Flash 13.6 % of 16 MB) |
| `.rodata` (model weights + const) | 596 KB |
| Internal RAM (static) | ~5.8 % of 512 KB |
| PPA / capture / tensor-arena buffers | PSRAM, allocated at runtime (not in the image) |

Runtime PSRAM depends on resolution and rotation; read it on your build from the
`model runtime PSRAM cost` (startup) and `PSRAM free low-water` (`dump_config`)
log lines. Roughly: model weights go in flash; the ESP-DL tensor arena, the
RGB888 frame (1280×720 ≈ 2.6 MB), and camera buffers go in PSRAM; the task stack
is internal SRAM.

## How it works

`PersonDetector` pulls one typed RGB frame per `interval` from a `FrameSource`
(on a dedicated low-priority FreeRTOS task), runs the ESP-DL model on it (resized
to the model input via the P4's hardware image path), and marshals the result to
the main loop for debouncing, state publishing, and triggers. See
[`DESIGN.md`](DESIGN.md) for the full design and [`BRINGUP.md`](BRINGUP.md) for a
first-flash checklist.

## Extending and portability

The component is layered so you can add support along several axes without
touching the core — including the target SoC, which is really just another axis:

- Detection model — pick from [ESP-DL's model catalog](https://github.com/espressif/esp-dl/tree/master/models)
  (pedestrian, face, hand, the 80-class COCO detector, and more). Add one with a
  row in the `MODELS` table plus a `case` in `PersonDetector::create_model_()`
  (`components/person_detect/`); any ESP-DL detector returning
  `dl::detect::result_t` boxes fits.
- Camera sensor — a `sensor:` enum entry and its `esp_cam_sensor` Kconfig in
  `components/esp_video_camera/__init__.py`; the capture/PPA path is
  sensor-independent.
- Frame-source backend — implement `person_detect::FrameSource`
  (`frame_source.h`) and yield `FrameView`s; the detector consumes them unchanged.
- Board — a device YAML with its `esp32:` details, camera wiring, and a
  `person_detect` block. The D1001 files under `example/` and `firmware/` are the
  first; others sit alongside them.
- Target SoC — the detector is SoC-agnostic (ESP-DL runs on the S3 as well as
  the P4), so `person_detect` only warns off-P4 rather than blocking. The
  P4-specific piece is `esp_video_camera`, which uses the P4's MIPI-CSI + ISP +
  PPA and hard-fails elsewhere. To run on an S3, add a frame-source backend fed
  by a DVP JPEG camera — ESPHome's `esp32_camera` (e.g. an OV2640) via
  `camera_id`. Only the P4 path is hardware-verified; the S3 route exists behind
  the same seam but needs bring-up (slower inference, no hardware rotation).

Contributions for new models, sensors, boards, and the S3 path are welcome.

## License

C++ is licensed GPL-3.0-only to match the ESPHome ecosystem (see `LICENSE`).
The bundled Espressif ESP-DL and pedestrian model are Apache-2.0 / MIT — see
`NOTICE`.
