# Warden — SCADA-Driven Autonomous Inspection & Repair Robot

> A human-in-the-loop patrol robot. A differential-drive bot roams an open arena,
> drives up to a set of target boxes, reads each box's fault/OK state from an onboard
> camera, and — only after a human operator authorizes it — performs a (mocked) physical
> repair. All perception, planning, and decision-making run on a laptop SCADA hub;
> the bot itself is a dumb actuator.

---

## Table of Contents

1. [What the system does](#1-what-the-system-does)
2. [System architecture](#2-system-architecture)
3. [Hardware](#3-hardware)
   - [3.1 Bill of materials](#31-bill-of-materials)
   - [3.2 Controller (ESP32-S3 "bot brain") pin map](#32-controller-esp32-s3-bot-brain-pin-map)
   - [3.3 Inspector (ESP32-S3 camera) pin map](#33-inspector-esp32-s3-camera-pin-map)
   - [3.4 Power rail](#34-power-rail)
4. [Network, ports & wire protocols](#4-network-ports--wire-protocols)
5. [ArUco marker scheme](#5-aruco-marker-scheme)
6. [Repository layout](#6-repository-layout)
7. [SCADA software (the laptop)](#7-scada-software-the-laptop)
   - [7.1 main.py — process & thread model](#71-mainpy--process--thread-model)
   - [7.2 vision_node.py — the overhead eye](#72-vision_nodepy--the-overhead-eye)
   - [7.3 control_node.py — the mission FSM](#73-control_nodepy--the-mission-fsm)
   - [7.4 bot_cam.py — the inspection decoder](#74-bot_campy--the-inspection-decoder)
   - [7.5 udp_bridge_node.py — ROS2 ↔ UDP](#75-udp_bridge_nodepy--ros2--udp)
   - [7.6 gui_node.py — the dashboard](#76-gui_nodepy--the-dashboard)
   - [7.7 arena_config.py — single source of truth](#77-arena_configpy--single-source-of-truth)
   - [7.8 RGB status LED (status colours)](#78-rgb-status-led-status-colours)
8. [The navigation algorithm (pixel-space visual servoing)](#8-the-navigation-algorithm-pixel-space-visual-servoing)
9. [Encoder motion primitives (TURN / FWD)](#9-encoder-motion-primitives-turn--fwd)
10. [Odom fusion (latency compensation)](#10-odom-fusion-latency-compensation)
11. [The inspection read (distance back-off)](#11-the-inspection-read-distance-back-off)
12. [The repair cycle](#12-the-repair-cycle)
13. [Firmware](#13-firmware)
    - [13.1 Controller firmware (bot_code/esp_bot)](#131-controller-firmware-bot_codeesp_bot)
    - [13.2 Camera firmware (bot_code/esp_cam)](#132-camera-firmware-bot_codeesp_cam)
14. [ROS2 topic reference](#14-ros2-topic-reference)
15. [Key constants](#15-key-constants)
16. [How to run](#16-how-to-run)
17. [Debug & calibration scripts](#17-debug--calibration-scripts)
18. [Simulation](#18-simulation)
19. [Build / flash workflow](#19-build--flash-workflow)

---

## 1. What the system does

**The bot is a dumb actuator. All intelligence lives on the laptop.** The two
microcontrollers execute motor commands, stream a camera, and report raw sensors. Every
decision — where to go, when to stop, whether a box is faulty, whether to repair — is made
on the SCADA hub in ROS2.

The mission, start to finish:

1. An **overhead phone camera** watches the whole arena from above. It sees the **bot**
   (ArUco marker ID 5) and a **top beacon marker** on each box (IDs 0–3).
2. The operator presses **START**. The controller locks in which boxes to patrol and orders
   them nearest-first.
3. For each box the bot **homes** on the beacon in raw image pixels, **aligns** square-on and
   centred in front of the inspection face, and **docks** using its forward time-of-flight
   sensor.
4. At the dock the bot's **inspector camera** reads the box's **front marker** (IDs 10–13).
   That ID is looked up in `fault_config` → **FAULT** or **OK**.
5. On **OK** the bot moves to the next box. On **FAULT** the GUI raises a **modal** and the
   bot holds still until the operator chooses **Repair Now**, **Queue**, or **Skip**.
6. **Repair** (mocked) noses the bot into the box, pauses, and reverses the exact same
   distance back to the docking stand-off.
7. When every box has been visited and every queued repair done, the run completes and the
   dashboard shows the per-box result summary.

Two radios feed the laptop independently over the phone hotspot: the **overhead phone
camera** (navigation — it sees the bot and the box tops) and the **inspector camera** on the
bot (fault verdict only — it reads the box fronts).

Every section below is cross-referenced to the source file that implements it.

---

## 2. System architecture

┌──────────────────────────────────────────────────────────────────────────┐
│                SCADA HUB  (laptop, Ubuntu + ROS2 Humble)                 │
│                   one process: `uv run scada/main.py`                    │
│                                                                          │
│  vision_node        control_node          bot_cam_node      gui_node     │
│  ───────────        ────────────          ────────────      ────────     │
│  overhead phone →    mission FSM,          S3 MJPEG →        PyQt5       │
│  ArUco detect:      pixel-servo homing,   front marker      4-panel      │
│   • bot ID 5        primitive align/dock, IDs 10-13 →       dashboard,   │
│   • box tops 0-3    back-off inspect,     verdict           operator     │
│  → /bot_px          mock repair                             modals       │
│  → /box_px          → /cmd_vel, TURN/FWD                                 │
│                     → /bot_command                                       │
│         │                    │                      ▲             ▲      │
│         └── /bot_px ────────┤ /box_aruco_detected ─┘             │      │
│             /box_px         │ /bot_cam_status         /fsm_state │      │
│         udp_bridge_node ◄───┘ /cmd_vel, /bot_command /inspection─┘      │
│                └──────────► /odom, /tof_range, /bot_status               │
└─────────────────────────────┬────────────────────────────────────────────┘
          phone MJPEG         │ Wi-Fi UDP (SSID "kafka")
          (overhead cam)      │ laptop :4211  ◄──►  esp-bot.local :4210
                   ┌──────────▼────────────┐      UART1 115200       ┌────────────────────────┐
                   │ ESP32-S3  (bot brain) │ TX GPIO2 ──────────────►│ ESP32-S3 (inspector)   │
                   │ esp-bot.local         │        "SCAN\n"         │ esp-cam.local          │
                   │ • TB6612FNG motors    │                         │ • GC2145 camera        │
                   │ • quad encoders       │                         │ • MJPEG HTTP :81       │
                   │ • VL53L0X TOF (I2C)   │                         │   /stream              │
                   │ • TURN/FWD primitives │                         │ • mDNS                 │
                   │ • UDP + mDNS          │                         │                        │
                   │ • 500 ms watchdog     │                         │                        │
                   └───────────────────────┘                         └────────────────────────┘
                                                              MJPEG :81 to laptop (bot_cam_node)



## 3. Hardware

### 3.1 Bill of materials

| # | Component | Spec | Role |
|---|---|---|---|
| 1 | ESP32-S3-WROOM (bot brain) | N16R8 class | Motor control, encoders, VL53L0X, UDP, watchdog, mDNS, TURN/FWD primitives |
| 2 | ESP32-S3-WROOM (inspector) | N16R8, + camera | MJPEG stream server on :81, UART SCAN handler |
| 3 | Camera module | GC2145 (YUV422 out, software JPEG) | Reads box-front ArUco at the dock |
| 4 | TB6612FNG | Dual H-bridge, VM 4.5–13.5 V | Drives both N20 motors |
| 5–6 | N20 encoder motors ×2 | 6 V, quadrature, 2 channels | Left / right drive |
| 7–8 | 43 mm wheels ×2 | D-shaft | Drive wheels |
| 9 | Ball caster | — | Rear contact |
| 10 | VL53L0X TOF | 0–2 m laser, I2C | Docking distance |
| 11 | 2S 18650 pack | 7.4 V nominal | Power source |
| 12 | Mini-360 buck | → 5.0 V | Logic rail |

### 3.2 Controller (ESP32-S3 "bot brain") pin map

From `bot_code/esp_bot/main/main.c`.

| Signal | GPIO | Notes |
|---|---|---|
| TB6612FNG `STBY` | 4 | HIGH = driver enabled |
| `AIN1` / `AIN2` (left dir) | 8 / 9 | plain GPIO direction |
| `BIN1` / `BIN2` (right dir) | 40 / 41 | — |
| `PWMA` (left speed) | 38 | LEDC channel 0, 20 kHz, 8-bit |
| `PWMB` (right speed) | 39 | LEDC channel 1 |
| Left encoder C1 / C2 | 21 / 12 | C1 = interrupt (rising edge), C2 = direction sample |
| Right encoder C1 / C2 | 18 / 15 | " |
| VL53L0X `SDA` / `SCL` | 6 / 7 | I2C0 @ 10 kHz, internal pull-ups enabled |
| UART1 TX → inspector | 2 | forwards `SCAN\n` to the camera board |

Motor direction logic (`set_motor_side`): `speed>0` → IN1=1, IN2=0; `speed<0` → IN1=0,
IN2=1; `speed==0` → IN1=1, IN2=1 (brake). Duty is `abs(speed)` clamped to 255.

### 3.3 Inspector (ESP32-S3 camera) pin map

From `bot_code/esp_cam/main/camera_core.c` and `main.c`:

| Signal | GPIO |
|---|---|
| XCLK | 15 |
| SIOD / SIOC (SCCB) | 4 / 5 |
| D7..D0 | 16, 17, 18, 12, 10, 8, 9, 11 |
| VSYNC / HREF / PCLK | 6 / 7 / 13 |
| UART1 RX (← controller SCAN) | 2 |
| LED (SCAN flash) | 48 |

The camera outputs **YUV422** (the GC2145 has no hardware JPEG encoder), so frames are
encoded to JPEG in software (`frame2jpg`, quality 80) before the MJPEG send. HTTP server runs
on **port 81**, path **`/stream`**, `multipart/x-mixed-replace`. Capture is QVGA, 2 frame
buffers in PSRAM, `CAMERA_GRAB_LATEST`.

### 3.4 Power rail

2S 18650 (7.4 V) → slide switch → TB6612FNG `VM` and Mini-360 buck input. Buck trimmed to
**exactly 5.00 V** before any board is connected → both ESP32-S3 `VIN` pins. All grounds
(battery−, buck, driver, both S3s, VL53L0X) tie to one common rail. 100 nF caps from each
encoder channel to ground suppress tick noise.

---

## 4. Network, ports & wire protocols

All devices join the phone's Wi-Fi hotspot (SSID **`kafka`**, pass `kafka000` — set in both
firmwares' `WIFI_SSID`/`WIFI_PASS`). Addressing is by **mDNS**, so no static IPs are needed:

| Device | Address | Port | Protocol |
|---|---|---|---|
| Phone IP Webcam (overhead) | `PHONE_STREAM_URL` | 8080 | HTTP MJPEG (`/video`) |
| Controller (bot brain) | `esp-bot.local` | 4210 | UDP (commands in) |
| Laptop SCADA | (learned) | 4211 | UDP (telemetry in) |
| Inspector camera | `esp-cam.local` | 81 | HTTP MJPEG (`/stream`) |

The controller advertises `esp-bot.local` via `mdns_hostname_set("esp-bot")`; the inspector
advertises `esp-cam.local`. The three URLs/hosts are overridable by environment variable
(`PHONE_STREAM_URL`, `S3_STREAM_URL`, `BOT_IP`, `BOT_PORT`) — see `arena_config.py`.

**Dynamic IP learning:** the controller initializes its SCADA destination to `0.0.0.0` and
only starts sending telemetry once it has received a packet, latching the sender's address
(`s_scada_addr`). Symmetrically, `udp_bridge_node._rx_loop` re-learns the bot's address from
any incoming packet, so mDNS resolution is a bootstrap only.

### UDP packet formats

**SCADA → controller** (`udp_bridge_node` encodes, `udp_rx_task` parses):

```
LEFT:<int> RIGHT:<int>\n   Per-wheel PWM, −255..255 (clamped). Posted to the motor queue.
STOP\n                     Zero both wheels.
TURN:<deg>\n               Closed-loop in-place rotation by <deg> (encoder primitive).
FWD:<mm>\n                 Closed-loop straight drive by <mm>, signed (encoder primitive).
SCAN\n                     Forwarded over UART1 to the inspector camera.
REPAIR_MODE\n              Notifies the firmware repair task.
REPAIR_STOP\n              Ends the firmware repair forward drive.
LED:<r>,<g>,<b>\n          Set persistent status colour (0-255 each; firmware caps brightness).
LED:<r>,<g>,<b>,<ms>\n     Flash that colour for <ms>, then revert to the last persistent colour.
```

**Controller → SCADA:**

```
ODOM:<dl_m>:<dr_m>:<heading_rad>\n   Per-50ms signed wheel travel + absolute integrated heading.
TOF:<mm>\n                           VL53L0X range in true millimetres (8190 = no target).
STATUS:TURN_DONE\n                   A TURN primitive finished.
STATUS:FWD_DONE\n                    A FWD primitive finished.
STATUS:REPAIRED\n                    End of firmware repair cycle.
```

Note the ODOM contract: `dl`/`dr` are **per-interval signed deltas in metres**, while the
third field is the **absolute** heading integrated on-device from differential-drive
kinematics. `udp_bridge_node._publish_odom` integrates `dl`/`dr` into `x`/`y` but takes
`heading` as-is.

---

## 5. ArUco marker scheme

All markers are `DICT_4X4_50`, generated by `scripts/aruco_gen.py` (400 px marker + 50 px
white border, printed ~8×8 cm). The ID space is **disjoint by camera**, so the overhead cam
and the bot cam can never confuse one another's markers:

| IDs | Printed on | Seen by | Purpose |
|---|---|---|---|
| **0, 1, 2, 3** | box **tops** | overhead phone cam | Navigation **beacons** (`TOP_ARUCO_IDS`). The bot homes on these |
| **5** | bot top, on a ~5 cm standoff | overhead phone cam | Bot pose `[u, v, θ]` (`BOT_ARUCO_ID`) |
| **10, 11, 12, 13** | box **fronts** (inspection faces) | bot's inspector cam | Fault/OK identity (`BOX_ARUCO_IDS`, keyed into `fault_config`) |

Each physical box has **one top beacon (0–3)** and **one front marker (10–13)**. The overhead
camera never sees the front markers and vice-versa.

**Beacon forward direction convention:** a box-top marker's "top edge" (computed by
`_marker_heading`) points *toward that box's inspection face*. The controller approaches from
that side with its nose pointing 180° opposite — face-to-face docking (`bot_hdg ≈ box_hdg + π`).

**Fault config** (`arena_config.fault_config`) is keyed by the front ArUco ID, not by
position: `{10:"FAULT", 11:"OK", 12:"FAULT", 13:"OK"}`. A box's identity is discovered only
when the bot's camera reads the front marker at the dock — the overhead cam knows a box only
as a beacon ID and a pixel location.

---

## 6. Repository layout

```
warden/
├── README.md                     ← this file
│
├── bot_code/                     ← ESP-IDF firmware (two projects)
│   ├── esp_bot/                  ← controller: motors, encoders, VL53L0X, UDP, mDNS, watchdog, primitives
│   │   └── main/main.c
│   └── esp_cam/                  ← inspector: camera + MJPEG server + UART SCAN
│       └── main/{main.c, camera_core.c, camera_core.h, idf_component.yml}
│
├── scada/                        ← laptop SCADA hub (ROS2 + PyQt5), plain Python files
│   ├── main.py                   ← entry point; launches all nodes + GUI in one process
│   ├── vision_node.py            ← overhead cam → /bot_px, /box_px
│   ├── control_node.py           ← mission FSM + all motion control
│   ├── udp_bridge_node.py        ← ROS2 ↔ UDP ↔ controller
│   ├── bot_cam.py                ← inspector MJPEG → /box_aruco_detected + dashboard image
│   ├── gui_node.py               ← PyQt5 dashboard
│   ├── arena_config.py           ← ALL tunables, IPs, marker IDs, geometry, fault_config
│   └── nav2_config/
│       └── bringup.launch.py     ← launch file (arena-free: starts the 4 live nodes;
│                                    legacy ekf.yaml / nav2_params.yaml retained, unused)
│
├── scripts/                      ← standalone debug/calibration (NO ROS2 import)
│   ├── aruco_gen.py              ← generate the marker PNGs
│   ├── stream_test.py            ← phone MJPEG + FPS + ArUco overlay
│   ├── mock_bot.py               ← fake controller UDP server for SCADA testing
│   ├── udp_echo.py               ← pretty-print every bot UDP packet
│   ├── tof_plot.py               ← live VL53L0X distance plot
│   ├── encoder_plot.py           ← live encoder tick + velocity plot
│   ├── odom_calib.py             ← drive 1 m, read summed ODOM → derive TICKS_PER_M
│   └── bot_cam_test.py           ← inspector stream + ArUco decode test
│
├── sim/                          ← Gazebo simulation
│   ├── warden_description/       ← URDF/xacro, worlds, launch, rviz, maps, materials
│   ├── sim_inspection_node.py    ← mocks the inspector verdict from ground-truth pose
│   └── sim_vision_node.py        ← mocks the overhead cam from a Gazebo camera
│
└── arena/markers/                ← generated ArUco PNGs
```

---

## 7. SCADA software (the laptop)

All `scada/*.py` are plain Python nodes (not an installed ament package). They run directly
under the project's `uv` venv. `sys.path.insert(0, ".../scada")` lets each import its siblings
(`arena_config`).

### 7.1 main.py — process & thread model

`main.py` runs **all** nodes in one process. Each background node (`ControlNode`,
`UdpBridgeNode`, `VisionNode`, `BotCamNode`) gets its **own** `SingleThreadedExecutor` in a
daemon thread — sharing the global executor causes a "generator already executing" crash.
`GuiNode` owns the Qt main thread; `app.exec_()` runs the event loop while its ROS executor
spins in yet another thread.

A subtle but critical detail: **PyQt5's C extensions must be imported before cv2**, because
cv2 bundles its own Qt `.so` files and whoever is `dlopen`'d first wins the linker cache.
`main.py` forces `QT_QPA_PLATFORM_PLUGIN_PATH` to PyQt5's plugin dir as a backstop so the
`xcb` platform plugin resolves.

### 7.2 vision_node.py — the overhead eye

A background thread (`_run_capture`) reads the phone MJPEG stream and keeps only the
**latest** frame (`_latest_frame` under a lock); a 15 Hz timer (`_process_frame`) processes it.
`_mjpeg_frames` is a hand-rolled multipart-JPEG splitter (finds `0xFFD8`..`0xFFD9`) rather than
`cv2.VideoCapture`, for a tighter latest-frame guarantee.

ArUco detection uses `DICT_4X4_50` with **sub-pixel corner refinement** (`CORNER_REFINE_SUBPIX`).
This is essential: the bot heading is derived from the marker's corner geometry, and raw
integer-pixel corners jitter by several degrees frame-to-frame — enough that alignment could
never settle. SUBPIX localises each corner to a fraction of a pixel.

`_process_frame_px` publishes two topics:

- **Bot marker (ID 5):** `_marker_center` gives pixel `(u, v)`; `_marker_heading` gives the
  forward heading from the marker's top edge (TL/TR midpoint minus BL/BR midpoint, with `-dy`
  to correct for image-Y-down). Published on **`/bot_px`** as a `PoseWithCovarianceStamped`
  (position = pixels, orientation = planar quaternion of the image heading, `frame_id="image"`).
- **Box-top beacons (IDs 0–3):** published on **`/box_px`** as a `PoseArray`, one `Pose` per
  visible beacon: `position.x/y` = pixel centre, `position.z` = `float(ArUco id)`,
  orientation = beacon heading.

A throttled (~1 Hz) diagnostic line reports what the camera sees (bot pixel pose + visible box
markers) so the pipeline is observable from the console.

### 7.3 control_node.py — the mission FSM

This is the brain — it owns navigation, alignment, docking, inspection, repair, the operator
protocol, and the watchdog. The motion strategy is detailed in
[§8](#8-the-navigation-algorithm-pixel-space-visual-servoing),
[§9](#9-encoder-motion-primitives-turn--fwd),
[§10](#10-odom-fusion-latency-compensation),
[§11](#11-the-inspection-read-distance-back-off), and [§12](#12-the-repair-cycle).

#### FSM states

| State | Meaning |
|---|---|
| `SETUP` | Waiting for the operator to press START |
| `IDLE` | Pick the next box, or finish |
| `NAVIGATING` | Pixel-servo toward the beacon's approach point |
| `ALIGNING` | Rotate in place (and side-nudge) to square-on, centred, face-to-face |
| `APPROACH_SCAN` | Straight TOF creep to `SCAN_STOP_MM`, then inspect |
| `APPROACH_DOCK` | Straight TOF creep to `DOCK_STOP_MM`, then repair |
| `INSPECTING` | Distance back-off read until the front marker is confirmed (or UNKNOWN) |
| `AWAITING_HUMAN` | FAULT found — operator must choose Repair / Queue / Skip |
| `REVIEWING_QUEUE` | All boxes visited; operator orders the queued repairs |
| `REPAIRING` | Mock repair: nose in, pause, reverse the same distance back |
| `RETURNING_HOME` / `MISSION_COMPLETE` | — |

`_transition` logs every change (`[HH:MM:SS] FSM: OLD → NEW`), re-publishes `/fsm_state`,
dispatches the entry action for the new state, and sets the bot's **status colour** for that
state; momentary verdicts (OK / FAULT / REPAIRED / UNKNOWN) are flashed. See
[§7.8](#78-rgb-status-led-status-colours).

**START → patrol plan:** on the `START` operator decision, `_snapshot_visit_queue` locks the
box list. It applies the **snapshot persistence filter**: a beacon must appear in at least
`SNAPSHOT_MIN_SEEN` (4) of the last `SNAPSHOT_WINDOW` (10) frames (`_beacon_history`, a rolling
`deque` of per-frame seen-ID sets) before it enters the queue. This rejects a single-frame
ghost detection (a front marker glimpsed from above, a reflection) from ever becoming a patrol
target. The remaining beacons are ordered nearest-first from the bot's current pixel position
(greedy nearest-neighbour).

**Watchdog:** `_watchdog_tick` fires every `WATCHDOG_MS × 0.8 = 400 ms`. It sends `STOP` while
in `AWAITING_HUMAN` so the bot holds still (and keeps the firmware watchdog fed) while the
human decides, and it also re-issues a motion primitive if a `STATUS:*_DONE` reply was lost.

**Operator protocol** (`/operator_decision`, handled in `_on_decision`):
`START`, `REPAIR_NOW`, `QUEUE`, `SKIP`, and `REPAIR_ORDER:<ids|NONE>` (from the queue dialog).

### 7.4 bot_cam.py — the inspection decoder

Reads the inspector MJPEG stream and publishes the confirmed front-marker ID.

- **Reconnect with exponential backoff** (2 s → 15 s cap): a flaky inspector otherwise hammers
  the shared hotspot and starves the overhead stream the whole nav loop depends on.
- **New-frame gate:** detection runs only on genuinely new frames (`_frame_seq` vs
  `_seen_seq`), not on every 20 Hz tick.
- **Liveness heartbeat:** every real new frame publishes `ALIVE` on **`/bot_cam_status`**. The
  inspection search in `control_node` refuses to advance while this is stale, so a dead stream
  is never silently counted as "no fault seen → OK".
- **Enhanced detector:** CLAHE contrast equalization + sub-pixel corner refinement + a wide
  adaptive-threshold window range, for close-range / dim front markers.
- **Nearest-marker verdict:** among detected box IDs it picks the one with the **largest quad
  area** — the box the bot is nose-to-nose with fills the most pixels, so a neighbour box
  clipping the frame edge can't win the verdict.
- **Multi-frame confirmation:** a verdict is published on **`/box_aruco_detected`** only once
  the dominant ID appears ≥ `INSPECT_CONFIRM_FRAMES` (2) times in the last
  `INSPECT_CONFIRM_WINDOW` (5) real frames — window is in frames, not ticks, so it holds up at
  a few fps.
- Also republishes the annotated frame on `/camera/camera_sensor/image_raw` for the dashboard.

### 7.5 udp_bridge_node.py — ROS2 ↔ UDP

A relay node, no state beyond accumulated odometry.

- **`/cmd_vel` → PWM** (`vel_to_pwm`): differential-drive mixing `v_left = x − ω·L/2`,
  `v_right = x + ω·L/2`, scaled by `MAX_PWM / MAX_LINEAR_VEL`, clamped to ±`MAX_PWM`. If the
  dominant wheel is below `MIN_MOVE_PWM` (the static-friction floor), **both** wheels are
  boosted proportionally so the differential steering ratio is preserved rather than one wheel
  being floored alone. Zero twist → `STOP\n`.
- **`/bot_command` → raw UDP:** forwards the string verbatim (`SCAN`, `TURN:…`, `FWD:…`,
  `REPAIR_MODE`, `STOP`) with a trailing newline.
- **Odom heartbeat:** `/odom` is republished at 10 Hz even with no new packet, so downstream
  always has input and a TF even before the bot connects. Covariance: 0.01 m² on x/y, 0.05 rad²
  on yaw, 1e9 on the 2-D-mode-ignored axes. `child_frame_id = base_footprint`.
- **`/tof_range`:** `TOF:<mm>` → `sensor_msgs/Range` (INFRARED, 25° cone, 0.03–2.0 m).
- Learns the bot's address from any incoming packet.

### 7.6 gui_node.py — the dashboard

PyQt5 `QMainWindow`. Thread model: Qt main thread for all widgets, a ROS executor thread, plus
worker threads doing the heavy cv2 work off the ROS callbacks (1-slot "latest frame" queues,
non-blocking drops).

Layout:

- **Phone cam** (top-left): read directly via a `CameraThread` on `PHONE_STREAM_URL`.
- **Bot cam** (top-right): fed from the ROS `/camera/camera_sensor/image_raw` topic, so there
  is no second connection fighting `bot_cam_node` over the inspector stream.
- **Arena map** (`ArenaMap` widget): draws box dots + bot dot/heading arrow from `/robot_pose`,
  colour-coded by status.
- **TOF top-down** panel: populated only in `--sim`.
- **Sidebar**: state badge (blinks in `AWAITING_HUMAN` / `REVIEWING_QUEUE`), live TOF, pose
  readout, per-box status grid, and START / STOP / MANAGE-QUEUE buttons.

Dialogs: `RepairDialog` (modal on FAULT → publishes `REPAIR_NOW` / `QUEUE` / `SKIP`) and
`RepairQueueDialog` (operator clicks queued boxes into an execution order → `REPAIR_ORDER:<ids>`).

Box status colours: `UNKNOWN` grey · `OK` green · `FAULT` red · `QUEUED` amber · `REPAIRED`
cyan · `SKIPPED` dashed red.

### 7.7 arena_config.py — single source of truth

Every tunable, IP, marker ID, geometry offset, and `fault_config` lives here. No IPs or magic
numbers are hard-coded in the nodes. See [§15 Key constants](#15-key-constants).

### 7.8 RGB status LED (status colours)

The bot carries an onboard **WS2812** RGB LED (GPIO48) that shows what it is doing. True to the
dumb-actuator rule, **the firmware decides nothing** — `control_node` picks the colour (the
FAULT/OK verdict only exists on the laptop) and sends it as a `LED:` packet
([§4](#4-network-ports--wire-protocols)); the firmware just drives whatever it is handed.

Each **FSM state** sets a *persistent* colour (held until the next change); each momentary
**verdict** is *flashed* for `LED_FLASH_MS` (~1.5 s) and then auto-reverts to the state colour.
The scheme is the single source of truth in `arena_config.LED_COLORS` (state→colour map in
`control_node.STATE_LED`):

| State / event | Colour | Meaning |
|---|---|---|
| `IDLE` | dim blue | ready / between boxes |
| `NAVIGATING`, `RETURNING_HOME` | cyan | homing on a beacon / driving home |
| `ALIGNING` | amber | squaring up to the box face |
| `APPROACH_SCAN`, `INSPECTING` | magenta | creeping to scan / reading the marker |
| `APPROACH_DOCK` | orange | final creep to dock |
| `REPAIRING` | green | executing the mock repair |
| `AWAITING_HUMAN` / **FAULT** | red | fault found — operator must decide |
| **OK** verdict | green **flash** (~1.5 s) | box passed |
| **REPAIRED** | cyan **flash** (~1.5 s) | repair complete |
| `MISSION_COMPLETE` | white | run done |
| inspection **UNKNOWN** | red **flash** (~1.5 s) | inconclusive read |

Boot shows a dim white. Full 0-255 values live in config; the firmware's `RGB_BRIGHTNESS` caps
global brightness. Firmware mechanism: [§13.1](#131-controller-firmware-bot_codeesp_bot).

---

## 8. The navigation algorithm (pixel-space visual servoing)

No map, no metric planner. The controller steers the **bot marker toward a box-top beacon in
raw image pixels**, and lets the TOF handle the final centimetres. The homing loop runs at
20 Hz on a timer inside `control_node` (`_drive_tick_px`).

A hard physical constraint shapes every motion: **`MIN_MOVE_PWM` floors any nonzero wheel
command up to ~100**, so a "gentle" continuous command is really a full-speed lurch, and the
overhead camera only delivers an absolute pose at **~1 Hz** while the bot reacts ~0.5 s later
over the link. A naive 20 Hz P-controller fires ~6 open-loop pulses per physical response and
whips the bot past its target. Two mechanisms handle this:

1. **When odom fusion is live** ([§10](#10-odom-fusion-latency-compensation)) the homing loop
   runs as a smooth continuous controller on the fused 20 Hz pose.
2. **Otherwise** it falls back to **one-pulse-per-camera-frame**: fire one short pulse, hold
   motors off until a *fresh* camera frame arrives (`_px_seq`), then re-measure. Pulse
   *duration* (not speed) sets the angle, since speed is floored.

### NAVIGATING — `_drive_tick_px`

The goal is not the beacon itself but its **approach point** (`_beacon_approach_point`):
`PX_APPROACH_STANDOFF` (200 px, ~12–14 cm) in front of the inspection face, along the beacon's
forward heading. Arriving there puts the bot's nose 180° opposite the beacon — face-to-face.

- Steering error = bearing from bot to approach point minus bot heading (`_approach_bearing_err`).
- Fused/live: `linear.x = PX_DRIVE_SPEED·max(0, cos(err))` (slows to a pivot when far
  off-heading, arcs in once aimed) and `angular.z = PX_TURN_KP·err` clamped to ±`PX_ANG_MAX`.
- Fallback: turn-in-place as one pulse per frame if `|err| > PX_TURN_FIRST_DEG`, else creep
  forward with gentle steer.
- Handoff to `ALIGNING` when the gap ≤ `PX_NEAR_GAP` (25 px), or on a TOF safety guard while
  close.

Once close, alignment, approach, inspection, and repair are all driven by **encoder motion
primitives** rather than pixel pulses — see [§9](#9-encoder-motion-primitives-turn--fwd).

### ALIGNING — `_start_align_prim` (measure → one primitive → re-check)

Alignment ends up **both** square-on and centred, so the line joining the two marker centres
is perpendicular to the box face. The loop is:

```
measure heading (bot stopped, HEADING_SAMPLES fresh frames → circular median)
   |err| ≤ TURN_TOL_DEG ?  ── no ──► one TURN:<err>  ──► re-measure
   |                └── yes ──► measure lateral offset (LAT_SAMPLES → median)
   |                              |lat| ≤ LAT_TOL_PX ? ── yes ──► done (APPROACH)
   |                              └── no ──► one TURN to the sideways heading,
   |                                        then one FWD across the offset,
   |                                        then back to measure heading
   └────────────────────────────────────────────────────────────────────────┘
```

Heading target is `box_hdg + π` (`_box_desired_hdg`). The lateral offset is the bot's distance
from the box's forward axis in overhead pixels (`_lateral_box_err`); it is corrected by turning
to face sideways and driving the offset across, converted px→mm by `LAT_PX_PER_MM` and damped
by `LAT_GAIN` (0.6) so any residual scale error converges instead of oscillating. The loop
caps at `ALIGN_MAX_ROUNDS` (4), after which it proceeds best-effort; if the bot marker is
unreadable for several windows it also proceeds best-effort rather than hang.

### APPROACH_SCAN / APPROACH_DOCK — `_start_approach_prim`

The bot creeps to the target TOF distance in straight `FWD:<DOCK_STEP_MM>` steps, **re-reading
the TOF between each step** so sensor noise / wheel slip can't accumulate. Straight steps keep
the bot square-on and centred — there is no skid to carry the camera off the box face.

- **`APPROACH_SCAN`** stops at `SCAN_STOP_MM` (~17 cm) — the wider stand-off where the whole
  front marker sits in the inspector's field of view with margin for small tilt.
- **`APPROACH_DOCK`** stops at `DOCK_STOP_MM` (~10 cm) before a repair.

The VL53L0X is the sole primary dock-stop (validated against a ruler). It reads reliably down
to ~10 cm; below ~5 cm it saturates (reports 8190), so the bot is deliberately kept inside the
sensor's trustworthy band.

---

## 9. Encoder motion primitives (TURN / FWD)

The overhead ArUco heading is too noisy to servo on directly, and open-loop `LEFT/RIGHT` PWM
can't travel straight (the two wheels drift and the bot skids). So the controller does **not**
stream a continuous command for precise moves. Instead it computes a **goal** from the camera
while the bot is stopped, then fires **one firmware primitive**:

- **`TURN:<deg>`** — an exact in-place rotation, closed-loop on differential encoder ticks.
- **`FWD:<mm>`** — a straight drive (signed; negative = reverse) that actively equalises the
  two wheels' travel so the path stays dead-straight.

The firmware runs the tick math to completion and replies `STATUS:TURN_DONE` / `STATUS:FWD_DONE`.
On the SCADA side `_send_primitive` sets a pending flag (suppressing `cmd_vel`), arms a
`PRIM_TIMEOUT_S` deadline (re-issue on a lost reply), and runs a continuation callback on
completion. Measurement between primitives is done stopped: `_start_sampling` collects
`HEADING_SAMPLES` / `LAT_SAMPLES` fresh camera frames and takes a robust **median** (circular
for angles) so a single flipped/garbage ArUco reading can't drag the goal.

On the firmware the primitive is a P-controller per wheel (`PRIM_TURN_KP` / `PRIM_FWD_KP`) with
an anti-skid steer term (`PRIM_STRAIGHT_KP`) that feeds back the mismatch between the two
wheels' travel. Output is floored to `PRIM_MIN_PWM` (60) only while still out of band and capped
at `PRIM_MAX_PWM` (110) for a gentle top speed; "done" requires `PRIM_DONE_CYCLES` consecutive
in-band cycles within `PRIM_TOL_TICKS` (≈0.8 mm / ≈0.76°). A `PRIM_SAFETY_MS` (4 s) hard
timeout guarantees it always reports done. The firmware watchdog does **not** zero the motors
while a primitive is running (`s_primitive_active`).

---

## 10. Odom fusion (latency compensation)

`ODOM_FUSION = True`. The overhead camera gives an absolute pixel pose at ~1 Hz; `/odom` gives
smooth 20 Hz relative motion that drifts. The fuser dead-reckons the pixel pose on odom between
camera frames and **hard-snaps** to the camera on every fresh frame — pure latency
compensation, no metric map.

The odom→image transform is **unknown** (there is no warp), so it is **fit online** from paired
motion samples:

- On each fresh camera frame (`_fuse_on_camera`), if the bot moved ≥ `FUSE_MIN_TRANS_M` since
  the last anchor, the paired (Δodom, Δcamera) displacement is accumulated into running
  normal-equation sums with a forgetting factor `FUSE_FORGET` (0.95).
- `_refit_odom_to_image` solves the 2×2 map `A = Σ(cam·odomᵀ)·(Σ odom·odomᵀ)⁻¹`, trusted once
  `|det| ≥ FUSE_MIN_DET` and `≥ FUSE_MIN_SAMPLES` (4) samples exist.
- Heading handedness between odom-yaw and image-heading is resolved by the sign of a separate
  running product `Σ(Δθ_img·Δyaw_odom)`.
- `_cur_px` returns the fused pose (anchor camera pose + `A·Δodom`); `_px_is_live` gates the
  continuous homing controller (odom fresh within `ODOM_STALE_S`, fit converged). If fusion is
  not live, homing falls back to the raw camera pose and the one-pulse-per-frame loop, so a
  miscalibrated odom never regresses behaviour.

---

## 11. The inspection read (distance back-off)

`INSPECT_BACKOFF = True`. The inspector reads a front marker cleanly at **~18–22 cm**, where
the whole marker sits in frame with margin for small tilt — but the scan dock is closer than
that. Because the bot arrives square-on and centred, the marker is dead ahead. So rather than
panning (which swings the camera off the face), `_start_inspect_backoff_prim` **reverse-creeps
dead-straight out in ~1 cm `FWD` steps**, pausing motors-off at each step for a sharp frame,
walking the distance into the readable band:

```
SETTLE (motors off, INSPECT_BACKOFF_SETTLE_TICKS)  →  one reverse FWD step
(target += INSPECT_BACKOFF_STEP_MM, re-read TOF)   →  SETTLE  →  …
```

- The reverse is **dead-straight** (the `FWD` primitive locks the wheels together); a heading
  term at creep speed would trace an arc off the face over ~16 cm.
- **Stream-liveness gate:** it never steps while `/bot_cam_status` is stale
  (> `INSPECT_STREAM_STALE_S`); it holds until frames resume, so a dead stream can't "complete"
  the band looking at nothing.
- A confirmed read from `bot_cam` (`_on_aruco_detected`) halts the search instantly and resolves
  the verdict.
- **End-of-band grace:** at `INSPECT_BACKOFF_MAX_MM` with no read, it holds and keeps looking
  for `INSPECT_BACKOFF_HOLD_TICKS` (~3 s) before giving up — a laggy 1–2 fps inspector often
  confirms a few hundred ms after the band is walked.
- A genuine miss → `_inspect_miss` → verdict **`UNKNOWN`** (never a silent OK), surfaced to the
  GUI as grey. `_inspect_safety` is an absolute 30 s wall-clock backstop so INSPECTING can never
  hang the FSM.

On a confirmed read, `_resolve_inspection` looks up `fault_config[id]`: `FAULT` →
`AWAITING_HUMAN` (modal), anything else → next box. The bot reaches INSPECTING only via a
TOF-gated square-on dock, and a 4×4 ArUco decodes to the right ID or not at all, so a confirmed
search read is accepted outright.

An **angular sweep** (`INSPECT_SWEEP`) is available as an alternate inspection strategy in
config, selected when `INSPECT_BACKOFF` is disabled.

---

## 12. The repair cycle

**SCADA-side mock repair** is the active behaviour (`_start_mock_repair_prim`, entered on
`REPAIR_NOW` → `APPROACH_DOCK` → `REPAIRING`):

1. Read the live TOF gap and compute a nose-in distance of `tof − REPAIR_CONTACT_MM` (so the
   bot stops `REPAIR_CONTACT_MM` ≈ 25 mm short of the box face), falling back to a fixed
   `MOCK_REPAIR_MM` nudge if the TOF is unreadable.
2. Nose in that distance in small `DOCK_STEP_MM` chunks (short steps keep the firmware P-term in
   its gentle region so the bot creeps up instead of launching).
3. Pause 0.5 s.
4. Reverse the **exact same** distance in the same step chunks, so the bot ends back at the
   docking stand-off.
5. Send `STOP`, mark the box `REPAIRED` (cyan on the dashboard), return to `IDLE` for the next
   box.

**Firmware repair cycle** also exists on the controller (`repair_task`): on `REPAIR_MODE` it
drives forward until `REPAIR_STOP` (or a safety timeout), records the forward time, reverses for
the same duration, and replies `STATUS:REPAIRED`. `control_node` handles `STATUS:REPAIRED`
identically whichever path produced it.

---

## 13. Firmware

Both projects are **ESP-IDF 5.x**. FreeRTOS tasks, lwIP sockets for UDP, `ledc_*` for PWM,
`i2c_master_*` for I2C. Activate the toolchain with `source ~/esp/esp-idf-5.3/export.sh`,
target `esp32s3`.

### 13.1 Controller firmware (bot_code/esp_bot)

Tasks (`app_main` spawns them with these priorities):

| Task | Prio | Job |
|---|---|---|
| `watchdog` | 7 | Every 100 ms: if no UDP for `WATCHDOG_TIMEOUT_MS` (500 ms) and not repairing / not running a primitive → STOP motors |
| `motor` | 6 | Applies the latest `motor_cmd_t` with a slew-rate ramp every 10 ms |
| `prim` | 6 | Runs a `TURN`/`FWD` primitive to completion on the encoder loop, replies `STATUS:*_DONE` |
| `udp_rx` | 5 | `recvfrom` loop: parses `LEFT/RIGHT`, `TURN`, `FWD`, `STOP`, `REPAIR_MODE`, `REPAIR_STOP`, `SCAN`, `LED`; learns SCADA addr; feeds watchdog |
| `repair` | 5 | Firmware repair cycle (forward → indicator → timed reverse → `STATUS:REPAIRED`) |
| `encoder` | 4 | Every 50 ms: signed tick deltas → metres, integrate heading, send `ODOM:dl:dr:heading` |
| `tof` | 3 | Poll VL53L0X interrupt, read range at reg `0x1E`, send `TOF:<mm>` (started only if the sensor inited) |
| `rgb` | 3 | Drives the onboard WS2812 (GPIO48) from `LED:` commands: holds a base colour, runs a flash for its `ms` then reverts to the base ([§7.8](#78-rgb-status-led-status-colours)) |

**Signed quadrature encoders:** each C1 channel interrupts on its rising edge (`IRAM_ATTR`
ISR); the C2 phase level sampled inside the ISR gives rotation direction, so counts are signed
(reverse decrements). `ENC_L_SIGN = -1`, `ENC_R_SIGN = +1` make a forward command yield
positive ticks on both sides (set empirically for the wiring). `TICKS_PER_M = 2465` (calibrate
with `scripts/odom_calib.py`), `WHEEL_SEP_M = 0.12` (matches `arena_config.WHEEL_SEPARATION`).
Heading is integrated on-device from `(dr − dl)/WHEEL_SEP` and wrapped to ±π.

**Asymmetric motor slew ramp** (`ramp_toward`): because the SCADA side floors commands up to
~100, an instantly-applied step snaps the wheels and blurs the inspector's view. The applied
duty ramps toward the target a few counts per 10 ms tick — **`RAMP_ACCEL = 6`** (slow,
blur-free acceleration) but **`RAMP_DECEL = 14`** (fast stop, 100→0 in ~70 ms) so STOP /
dock-stop still bite quickly and docking gains no extra coast.

**VL53L0X** (I2C0, SDA 6 / SCL 7, 10 kHz with internal pull-ups): the full ST-API register init
sequence runs in `vl53l0x_init`, including the mandatory **two reference calibrations** (VHV
init byte `0x40`, then phase `0x00`) — without both, every read returns the 8190 no-target
sentinel. The device-specific `stop_var` is read and written back during the continuous-ranging
start. Readings at register `0x1E` are **true millimetres natively**; `tof_finalize` applies a
single honest offset (`TOF_OFFSET_MM = -97`) and the 8190 sentinel. Init runs *before* Wi-Fi so
a sensor fault is visible immediately; if the probe fails, the `tof` task is simply not started.

**Networking:** `wifi_init_sta` (station mode, WPA2, up to 10 retries), then `mdns` publishes
`esp-bot.local`, then a UDP socket binds `:4210`. TX power is capped
(`esp_wifi_set_max_tx_power(40)`) before the first probe burst.

**RGB status LED:** the onboard WS2812 (GPIO48) is driven via the RMT-based `espressif/led_strip`
component. `rgb_set` scales each channel by `RGB_BRIGHTNESS/255` (global brightness cap) before
the strip refresh. `idf_component.yml` pulls in `espressif/mdns` and `espressif/led_strip`.

### 13.2 Camera firmware (bot_code/esp_cam)

- `wifi_init_sta` → mDNS `esp-cam.local` → `camera_core_init()` (pins in §3.3) → HTTP server on
  port 81 serving `/stream`.
- `stream_handler`: grab frame → `frame2jpg(fb, 80)` (software JPEG, since the GC2145 outputs
  YUV422) → multipart send → return the framebuffer before the slow HTTP write.
  `max_open_sockets = 2` (one active stream + one queued), `ctrl_port = 32768` to dodge the
  default.
- `uart_scan_task`: UART1 RX on GPIO2 at 115200; on `"SCAN"` from the controller, flash the LED
  (GPIO48) 200 ms. The actual ArUco decode happens on the laptop reading the stream — the camera
  does not decode onboard.
- `idf_component.yml` pulls in `espressif/esp32-camera` and `espressif/mdns`.

---

## 14. ROS2 topic reference

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/bot_px` | `PoseWithCovarianceStamped` | vision_node → control_node (bot marker, pixels) |
| `/box_px` | `PoseArray` | vision_node → control_node (box-top beacons, pixels; id in `position.z`) |
| `/odom` | `Odometry` | udp_bridge → control_node (fusion) |
| `/tof_range` | `Range` | udp_bridge → control_node, gui_node (docking / inspection) |
| `/cmd_vel` | `Twist` | control_node → udp_bridge |
| `/bot_command` | `String` | control_node → udp_bridge (`SCAN` / `TURN:` / `FWD:` / `STOP` / `REPAIR_MODE`) |
| `/bot_status` | `String` | udp_bridge → control_node (`STATUS:TURN_DONE` / `FWD_DONE` / `REPAIRED`) |
| `/box_aruco_detected` | `String` | bot_cam → control_node (confirmed front ID 10–13) |
| `/bot_cam_status` | `String` | bot_cam → control_node (`ALIVE` liveness heartbeat) |
| `/camera/camera_sensor/image_raw` | `Image` | bot_cam → gui_node (annotated inspector frame) |
| `/fsm_state` | `String` | control_node → gui_node |
| `/inspection_result` | `String` | control_node → gui_node (`FAULT:id` / `OK:id` / `UNKNOWN:` / `QUEUED:` / `SKIPPED:` / `REPAIRED:`) |
| `/operator_decision` | `String` | gui_node → control_node |
| `/robot_pose` | `PoseWithCovarianceStamped` | vision_node → gui_node |

---

## 15. Key constants

All in `scada/arena_config.py` unless noted. Firmware copies are kept in sync.

**Geometry / arena**
```
WHEEL_SEPARATION  = 0.12 m      (firmware WHEEL_SEP_M = 0.12f)
CAM_FWD = 0.20049 m, CAM_LAT = 0.0614 m    (S3 camera offset from base_footprint)
AXLE_FWD = 0.05549 m, AXLE_LAT = 0.06137 m (wheel-axle = in-place rotation centre)
```

**Motors**
```
MAX_PWM = 180, MIN_MOVE_PWM = 100 (static-friction floor), MAX_LINEAR_VEL = 0.15 m/s
firmware: LEDC 20 kHz 8-bit; RAMP_ACCEL = 6, RAMP_DECEL = 14 counts / 10 ms tick
```

**Motion primitives**
```
MOVE_PRIMITIVES = True, TURN_TOL_DEG = 0.8, HEADING_SAMPLES = 3, LAT_SAMPLES = 3
SETTLE_S = 0.5, PRIM_TIMEOUT_S = 5.0, DOCK_STEP_MM = 15
LAT_PX_PER_MM = 0.70, LAT_GAIN = 0.6
firmware: PRIM_TURN_KP = 0.6, PRIM_FWD_KP = 0.6, PRIM_STRAIGHT_KP = 2.0
         PRIM_MIN_PWM = 60, PRIM_MAX_PWM = 110, PRIM_TOL_TICKS = 2, PRIM_SAFETY_MS = 4000
```

**Docking / TOF**
```
SCAN_STOP_MM = 170, DOCK_STOP_MM = 105 (~10 cm, env-overridable)
DOCK_NEAR_BAND_MM = 70, DOCK_CREEP_SPEED = 0.015 m/s, DOCK_ANTICIPATE_MM = 18
APPROACH_SPEED = 0.04 m/s, USE_TOF = True
firmware: TOF native mm, TOF_OFFSET_MM = -97, TOF_NO_TARGET = 8190
```

**Alignment (pixel + primitive)**
```
PERP_TOL_DEG = 2.0, LAT_TOL_PX = 12 (~1.8 cm), LAT_TURN_TOL_DEG = 10
ALIGN_SETTLE_TICKS = 14 (~0.7 s), ALIGN_MAX_PULSES = 15, ALIGN_MAX_ROUNDS = 4
```

**Pixel servoing**
```
ARENA_FREE = True
PX_DRIVE_SPEED = 0.08 m/s, PX_TURN_KP = 1.2, PX_TURN_FIRST_DEG = 35, PX_ANG_MAX = 0.6 rad/s
PX_NEAR_GAP = 25 px, PX_APPROACH_STANDOFF = 200 px (~12-14 cm), PX_TURN_SIGN = 1.0
```

**Inspection**
```
INSPECT_BACKOFF = True, STEP = 10 mm, MAX = 250 mm, SETTLE = 8 ticks, HOLD = 60 ticks (~3 s)
INSPECT_CONFIRM_FRAMES = 2 of INSPECT_CONFIRM_WINDOW = 5 real frames
INSPECT_STREAM_STALE_S = 1.0, INSPECT_SAFETY_S = 30.0
```

**Repair**
```
REPAIR_CONTACT_MM = 25 (nose-in clearance), MOCK_REPAIR_CM = 3.5 (fixed-nudge fallback)
firmware: REPAIR_FWD_SPEED = 90, REPAIR_FWD_SAFETY_MS = 5000
```

**Odom fusion**
```
ODOM_FUSION = True, ODOM_STALE_S = 0.5, FUSE_MIN_TRANS_M = 0.02, FUSE_MIN_ROT_RAD = 0.05
FUSE_FORGET = 0.95, FUSE_MIN_DET = 1e-6, FUSE_MIN_SAMPLES = 4
```

**Snapshot filter**
```
SNAPSHOT_WINDOW = 10, SNAPSHOT_MIN_SEEN = 4
```

**Markers & network**
```
BOT_ARUCO_ID = 5, TOP_ARUCO_IDS = [0,1,2,3], BOX_ARUCO_IDS = [10,11,12,13]  (DICT_4X4_50)
WATCHDOG_MS = 500   (firmware WATCHDOG_TIMEOUT_MS = 500)
BOT_IP = esp-bot.local:4210, S3_STREAM_URL = esp-cam.local:81/stream, SCADA_LISTEN_PORT = 4211
fault_config = {10:"FAULT", 11:"OK", 12:"FAULT", 13:"OK"}
firmware: TICKS_PER_M = 2465, ENC_L_SIGN = -1, ENC_R_SIGN = +1
```

**Status LED** (see [§7.8](#78-rgb-status-led-status-colours))
```
LED_FLASH_MS = 1500   (verdict flash hold)
LED_COLORS = { IDLE, NAV, ALIGN, SCAN, DOCK, REPAIR, FAULT, OK, REPAIRED, DONE, ERROR, OFF }
              single source of truth; firmware RGB_BRIGHTNESS caps global brightness
```

---

## 16. How to run

**Full stack (one process, with GUI):**
```bash
source /opt/ros/humble/setup.bash
uv run scada/main.py
```
This launches `control_node`, `udp_bridge_node`, `vision_node`, `bot_cam_node`, and the GUI.
Press **START RUN** in the dashboard to begin the patrol.

**Launch-file variant:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch scada/nav2_config/bringup.launch.py
```
In arena-free mode (`ARENA_FREE = True`) this brings up the same four nodes as `main.py`
(vision, udp_bridge, control, bot_cam) — no Nav2, no EKF — just without the GUI.

**Prerequisites:** phone on hotspot `kafka`, IP Webcam streaming; laptop on `kafka`; both
ESP32-S3 boards powered and on the hotspot (`esp-bot.local` / `esp-cam.local` resolvable).
Python deps via `uv pip install opencv-contrib-python pyqt5 matplotlib numpy` (always `uv`,
never bare `pip`).

**Manual firmware poke (no SCADA):**
```bash
echo -n "LEFT:120 RIGHT:120" | nc -u esp-bot.local 4210   # both wheels forward
echo -n "TURN:90"            | nc -u esp-bot.local 4210   # rotate 90° in place
echo -n "FWD:200"            | nc -u esp-bot.local 4210   # drive 200 mm straight
echo -n "STOP"               | nc -u esp-bot.local 4210
```

---

## 17. Debug & calibration scripts

Plain Python + OpenCV/matplotlib/socket — **no ROS2 import**, run without sourcing ROS.

| Script | Use |
|---|---|
| `aruco_gen.py` | Generate the marker PNGs into `arena/markers/` |
| `stream_test.py` | Phone MJPEG feed + FPS + ArUco overlay (target ≥ 18 fps) |
| `bot_cam_test.py` | Inspector stream + ArUco decode sanity |
| `udp_echo.py` | Pretty-print every UDP packet from the controller (port 4211) |
| `tof_plot.py` | Live VL53L0X distance vs time |
| `encoder_plot.py` | Live left/right ticks + computed velocity |
| `odom_calib.py` | Drive a measured 1.0 m, read summed ODOM, derive `TICKS_PER_M` |
| `mock_bot.py` | Fake controller UDP server — test the SCADA stack with no hardware |

**Encoder calibration:** drive exactly 1.0 m straight, read the summed `dl`/`dr` from
`odom_calib.py` / `udp_echo.py`, and set `TICKS_PER_M` so the sum ≈ 1.0. If a wheel's ticks go
*negative* on forward motion, flip that side's `ENC_*_SIGN`.

**TOF calibration:** validate against a ruler at 100–500 mm (flat target). If a consistent bias
remains, set `TOF_OFFSET_MM = true_mm − reported_mm` at one point. Don't calibrate below ~50 mm
(the sensor saturates / over-reads in the near field).

---

## 18. Simulation

`sim/warden_description/` is a colcon-built ROS2 description package (URDF/xacro with a
diff-drive Gazebo plugin, worlds, maps, materials, and an RViz config). `sim/sim_vision_node.py`
mocks the overhead camera from a Gazebo camera and `sim/sim_inspection_node.py` mocks the
inspector verdict from the bot's ground-truth pose, so the full SCADA stack (FSM, GUI, docking
logic) can be exercised with no hardware. The SCADA nodes run identically in sim and real; only
the hardware interface swaps. Run the GUI with `--sim` to show the extra simulation panels.

---

## 19. Build / flash workflow

```bash
source ~/esp/esp-idf-5.3/export.sh         # activate ESP-IDF 5.x

cd bot_code/esp_bot                         # or bot_code/esp_cam
idf.py set-target esp32s3
idf.py build
idf.py flash
idf.py monitor                              # console on USB Serial/JTAG (USB-CDC)
```

`managed_components/`, `.cache/`, `build/`, and `sdkconfig.old` are gitignored; dependencies are
restored from `idf_component.yml` + `dependencies.lock`.
