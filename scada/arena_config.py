import os

WHEEL_SEPARATION  = 0.12
MAX_PWM           = 180
MIN_MOVE_PWM      = 100   # any nonzero wheel command is bumped to at least this — below it the
                         # motors just buzz without moving (static friction), worse on a low pack
MAX_LINEAR_VEL    = 0.15
WARP_SIZE         = 800
ARENA_SIZE_CM     = 200
PX_PER_CM         = WARP_SIZE / ARENA_SIZE_CM
DOCKING_STOP_MM   = 600   # mm — legacy single-threshold, superseded by SCAN/DOCK below
DOCKING_PHASE_CM  = 50    # cm — legacy Nav2 pre-dock goal, superseded by NAV_STANDOFF_M

# Two-stage approach: read the ArUco at scan distance, creep to dock distance only to repair
SCAN_STOP_MM     = 170    # mm — stop-and-scan distance (~13 cm) for reading the box ArUco
DOCK_STOP_MM     = float(os.environ.get("DOCK_STOP_MM", "105.0"))  # mm — docking stop (~10 cm gap).
                          # The S3 decodes the box ArUco at ~10 cm minimum, and below ~5 cm the VL53L0X
                          # saturates (reads 8190). Stopping at 10 cm keeps the bot inside the TOF's
                          # reliable range, so TOF alone stops it cleanly — no camera-pixel guesswork.
NAV_STANDOFF_M   = 0.45   # m  — Nav2 pre-dock goal south of box centre (clear of inflation;
                          #      the TOF creep covers the rest down to SCAN_STOP_MM)
PERP_TOL_DEG     = 2.0    # deg — final heading band: bot must point within ±2° of 180°-opposite the
                          # box marker (the two arrows cancel). Reachable only on a low-latency link —
                          # over the ~0.5 s Tailscale hop one min motor pulse already overshoots ~11°.
LAT_TOL_PX       = 12.0   # overhead px (~1.8 cm) — bot centre must sit within this of the box marker's
                          # forward axis, so the line joining the two markers is perpendicular to the
                          # box face (bot centred on the marker, not off to one side).
LAT_TURN_TOL_DEG = 10.0   # deg — how closely to face the sideways-drive direction before driving (just
                          # a setup heading for the lateral nudge, so it can be loose)
ALIGN_SETTLE_TICKS = 14   # ticks (×50 ms ≈ 0.7 s) motors-OFF after every pulse before measuring the
                          # next one. The phone cam runs ~15 Hz but the bot lags ~0.5 s over the link,
                          # so without this the loop fires a blur of pulses per physical response and
                          # the heading whips around. One pulse → settle → look → one pulse = fine,
                          # discrete, observable steps instead of a compounding over-correction.
ALIGN_MAX_ROUNDS = 4      # heading+lateral re-centre rounds before proceeding best-effort
ALIGN_MAX_PULSES = 15     # per-stage pulse cap: if a rotation won't settle inside tolerance within this
                          # many pulses (coarse floored motor oscillating on a laggy link), move on
                          # rather than hang the FSM in ALIGNING forever.
LATERAL_TOL_M    = 0.08   # m  — max |camera_x − box_x| for the QR to sit fully in view
APPROACH_SPEED   = 0.04   # m/s — gentle creep speed during docking approach
# Final-approach coast control. At 0.04 m/s the bot crosses DOCK_STOP_MM and then coasts ~5 cm
# into the box before the STOP packet + inertia settle (measured: stops at ~5 cm not 10 cm).
# Fix: once inside the near band, crawl so the coast collapses, and trigger STOP a few mm early.
DOCK_NEAR_BAND_MM   = 70.0   # mm — within (target + this), drop to the crawl speed below
DOCK_CREEP_SPEED    = 0.015  # m/s — crawl in the near band so residual coast is only a few mm
DOCK_ANTICIPATE_MM  = 18.0   # mm — trigger STOP this far before target to absorb the remaining coast
# After the dock stop, reverse-creep until the TOF shows a reliable gap >= DOCK_STOP_MM, so the
# final stand-off is the same (~10-11 cm) every run no matter how the stop was triggered. Backing
# AWAY from the box walks the TOF up through its trustworthy band — fixes the "4-5 cm one run,
# <1 cm the next" inconsistency and guarantees a distance the S3 can read at.
DOCK_SETTLE         = True
# Active inspection sweep: a dead-straight dock is never perfect — ±1-2° or a small lateral offset
# is enough that the S3 sees nothing. Rather than dock-and-hope, pan the bot about its axle so the
# S3 sweeps across the box face until it catches the ArUco, then stop the instant it does.
# PULSE-AND-PAUSE (not a continuous spin): MIN_MOVE_PWM floors any nonzero wheel command up to a
# fast spin, so an open-loop "0.25 rad/s" command actually whips the bot ~60° and the motion blur
# stops the S3 decoding anyway. Instead fire ONE short pulse, then hold motors OFF for a few ticks
# so the bot is physically still (sharp frame for the S3) and the overhead heading can update. The
# arc is bounded by the MEASURED camera heading vs the dock heading — never an integrated command.
INSPECT_SWEEP        = True
INSPECT_SWEEP_SPEED  = 0.30   # rad/s — per-pulse angular command (one 50 ms tick, then pause)
INSPECT_SWEEP_AMPL   = 35.0   # deg — arc half-width each side of the dock heading. Wide so an
                              # edge-clipped marker is covered even if the dock heading is a few
                              # degrees off; read at the ~17 cm scan standoff makes this generous.
INSPECT_SWEEP_PAUSE_TICKS = 6 # ticks (×50 ms) motors-off between pulses — still frame + fresh pose
INSPECT_SWEEP_MAX_PULSES  = 10 # per-leg fallback cap: advance to the next waypoint after this many
                              # pulses if the overhead pose is stale (can't measure the real offset)
INSPECT_SWEEP_CYCLES      = 2  # full +arc→−arc→centre passes with a LIVE stream before a real miss
INSPECT_SWEEP_WP_TOL_DEG  = 6.0  # deg — treat a waypoint as reached within this of the target offset
INSPECT_STREAM_STALE_S    = 1.0  # s — no S3 heartbeat for this long → pause the sweep (stream dead)
INSPECT_SAFETY_S          = 30.0 # s — absolute backstop so INSPECTING can never hang the FSM
# S3 ArUco read confirmation: a verdict is committed only when the same box ID dominates a short
# window of REAL decoded frames (not 20 Hz ticks). Rejects a single spurious decode or a neighbour
# box flickering at the frame edge. Keep CONFIRM small so a slow stream still confirms during a pause.
INSPECT_CONFIRM_FRAMES = 2   # same ID must appear in at least this many frames of the window
INSPECT_CONFIRM_WINDOW = 5   # window length in real decoded frames
# Distance back-off read (supersedes the angular sweep). A square-on, centred dock means the marker
# is dead ahead; the S3 reads it cleanly at ~18-22 cm where the whole marker sits in frame with
# margin for small tilt. So instead of panning, reverse-creep STRAIGHT out in ~1 cm steps — like the
# dock-settle back-off — pausing motors-off at each step for a sharp frame, until bot_cam confirms the
# ID. Deterministic: the marker stays centred the whole time, we just walk the distance into the
# readable band. If the whole band is covered with a live stream and nothing reads → a real miss.
INSPECT_BACKOFF          = True   # True → distance back-off read; False → fall back to INSPECT_SWEEP
INSPECT_BACKOFF_STEP_MM  = 10.0   # mm — reverse this far (TOF) per step
INSPECT_BACKOFF_MAX_MM   = 250.0  # mm — give up (UNKNOWN) once backed off past the readable band
INSPECT_BACKOFF_SETTLE_TICKS = 8  # ticks (×50 ms) motors-off per step so the S3 gets a still frame
INSPECT_BACKOFF_HOLD_TICKS = 60   # ticks (×50 ms ≈ 3 s) — on reaching the far end with no read yet,
                                  # HOLD still and keep looking this long before declaring UNKNOWN.
                                  # A laggy S3 (1-2 fps) can confirm a few hundred ms after the band
                                  # is walked; without this grace the search quit ~250 ms too early
                                  # and threw away a real FAULT read.
MOCK_REPAIR_CM   = 3.5    # cm — legacy fixed nudge; superseded by the TOF-driven nose-in below
REPAIR_CONTACT_MM = 25.0  # mm — mock repair noses forward to this gap from the box face (off the
                          #      live TOF at dock), does the repair, then reverses the SAME distance
                          #      so it ends back at the docking stand-off. Raise if it taps the box.
APPROACH_YAW_KP  = 2.0    # P gain — hold perpendicular heading while creeping forward
ALIGN_YAW_KP     = 1.5    # P gain — rotate to perpendicular / face the centerline

# ── Encoder closed-loop motion primitives ─────────────────────────────────────
# The overhead ArUco heading is too noisy to servo on directly, and open-loop LEFT/RIGHT
# PWM can't travel straight (the two wheels drift → the bot skids). So instead of streaming
# /cmd_vel, control_node computes a GOAL from the camera while the bot is stopped, then fires
# ONE firmware primitive — TURN:<deg> (exact rotation on differential ticks) or FWD:<mm>
# (straight drive that equalises the two wheels' travel) — waits for STATUS:TURN_DONE /
# STATUS:FWD_DONE, re-measures, and repeats. Firmware does the tick math; these govern SCADA.
MOVE_PRIMITIVES  = True   # True → primitive-based align/dock/back/repair; False → legacy cmd_vel servo
TURN_TOL_DEG     = 0.8    # deg — final heading band. ~0.4° is one differential encoder tick at this
                         #       wheel separation, so this is near the hardware floor; tighter can't settle.
HEADING_SAMPLES  = 3      # fresh /bot_px frames collected (bot stopped) → circular median = the goal.
                         #       The phone pose arrives ~1 Hz, so each sample costs ~1 s — 3 still
                         #       rejects a lone outlier while cutting a measure round from ~5 s to ~2 s.
LAT_SAMPLES      = 3      # fresh frames for the lateral-offset median
SETTLE_S         = 0.5    # s — hold still after a STOP before sampling, so the camera pose is current
PRIM_TIMEOUT_S   = 5.0    # s — if STATUS:*_DONE never arrives, re-measure and reissue (lost packet)
LAT_PX_PER_MM    = 0.70   # overhead px per mm of bot travel — CALIBRATED from a run whose centring
                         #       overshot ~2.35x at the old 0.30 (it drove far too far and zig-zagged
                         #       across the centerline). 0.70 makes one round land near the centerline.
LAT_GAIN         = 0.6    # fraction of the measured lateral offset to correct per round. <1 damps the
                         #       loop so any residual scale error CONVERGES instead of oscillating —
                         #       kills the "lots of big 90-deg corrections" zig-zag. Converges for
                         #       scale errors up to ~1/0.6 = 1.67x; smaller moves also feel gentler.
DOCK_STEP_MM     = 15.0   # mm — max straight FWD step per docking iteration (re-reads TOF between steps)
MOCK_REPAIR_MM   = MOCK_REPAIR_CM * 10.0

# ── Arena-free navigation (pixel-space visual servoing) ───────────────────────
# When True, vision_node skips the corner-marker homography entirely and control_node
# steers the bot marker (ID 5) straight at a box's TOP marker in raw image pixels; the
# TOF handles only the final dock. No corner markers, no warp, no metric map, no EKF.
# The overhead top marker is a NAVIGATION BEACON only — the FAULT/OK verdict still comes
# from the S3 reading the box's SIDE marker (fault_config keyed by that side ID).
ARENA_FREE         = True
USE_TOF            = True   # VL53L0X validated against a ruler (±2mm at 4-5cm) — docking stops on
                           # real distance. Set False to fall back to pixel-gap-only if it regresses.
TOP_ARUCO_IDS      = [0, 1, 2, 3]  # markers the OVERHEAD cam sees on box tops (beacons)
SNAPSHOT_WINDOW    = 10  # frames of beacon sighting history kept per START
SNAPSHOT_MIN_SEEN  = 4   # a beacon must appear in at least this many frames to enter the patrol queue
BOT_MARKER_YAW_OFF = 0.0    # deg — add to the bot marker heading if marker-north ≠ bot-forward
PX_TURN_SIGN       = 1.0    # flip to -1.0 if the bot turns the WRONG way (wheel/camera handedness)
PX_TURN_FIRST_DEG  = 35.0   # turn in place if bearing error > this, else drive & steer smoothly
PX_TURN_KP         = 1.2    # rad/s per rad of image-bearing error
PX_DRIVE_SPEED     = 0.08   # m/s forward while homing on the box marker
PX_NEAR_GAP        = 25.0   # px — bot↔approach-point gap under which we switch to in-place ALIGNING
PX_AIM_TOL_DEG     = 20.0   # deg — heading alignment error under which a scan is accepted as square-on
PX_APPROACH_STANDOFF = 200.0 # px — standoff in front of box face for clean in-place alignment (~12-14 cm)
                             # (box top ArUco forward = toward inspection face; bot arrives from that
                             # side with its nose 180° opposite → face-to-face docking)

# ── Odom fusion (arena-free latency compensation) ─────────────────────────────
# The overhead camera gives an absolute pixel pose at only ~1 Hz; /odom gives smooth 20 Hz relative
# motion (drifts). Dead-reckon the pixel pose on odom between camera frames and hard-snap to the
# camera on every fresh frame, so the 20 Hz control loops run on a fresh pose instead of a stale one.
# Fully arena-free: no metric map, just latency compensation. The odom→image scale/rotation is
# unknown (no warp) so it is fit online from paired (Δodom, Δcamera) motion samples.
ODOM_FUSION      = True
ODOM_STALE_S     = 0.5    # s — newest /odom older than this → fall back to raw camera pose
FUSE_MIN_TRANS_M = 0.02   # m — min odom translation between camera frames to log a position sample
FUSE_MIN_ROT_RAD = 0.05   # rad — min odom rotation between frames to log a heading-sign sample
FUSE_FORGET      = 0.95   # forgetting factor for the running online odom→image least-squares fit
FUSE_MIN_DET     = 1e-6   # min |det| of the fit's normal matrix before fused POSITION is trusted
FUSE_MIN_SAMPLES = 4      # position samples required before fused position is trusted
# Continuous control laws (Phase 5) run only when the fused pose is live (odom healthy); otherwise
# the loops fall back to the one-pulse-per-camera-frame gate (no regression if odom is miscalibrated).
PX_ANG_MAX       = 0.6    # rad/s — clamp on continuous angular commands (alignment + arc follower)

# Bot geometry (warden.urdf.xacro). base_footprint (odom origin) sits 6.14 cm off the bot
# centerline; both the S3 camera (camera_jt) and the wheel-axle rotation centre lie ON that
# centerline. So centering the AXLE's ground-x on the box makes the camera land centred and
# survives the in-place perp-turn (the axle is the pivot, so it doesn't move during rotation).
CAM_FWD          = 0.20049  # m  — camera forward of base_footprint (body frame)
CAM_LAT          = 0.0614   # m  — camera left of base_footprint
AXLE_FWD         = 0.05549  # m  — wheel-axle midpoint forward of base_footprint
AXLE_LAT         = 0.06137  # m  — wheel-axle midpoint left of base_footprint
BOT_ARUCO_ID      = 5
CORNER_ARUCO_IDS  = [0, 1, 2, 3]
BOX_ARUCO_IDS     = [10, 11, 12, 13]
WATCHDOG_MS       = 500

# BOX_HSV_LO   = (0, 101, 182)
# BOX_HSV_HI   = (179, 255, 255)
# BOX_MIN_AREA = 5000

# BOX_HSV_LO   = (0, 106, 0)
# BOX_HSV_HI   = (20, 255, 112)
# BOX_MIN_AREA = 0

BOX_HSV_LO   = (0, 48, 60)
BOX_HSV_HI   = (21, 121, 255)
BOX_MIN_AREA = 800

# IPs — update before first run, or override via environment variables
PHONE_STREAM_URL  = os.environ.get("PHONE_STREAM_URL", "http://10.86.116.146:8080/video")
S3_STREAM_URL     = os.environ.get("S3_STREAM_URL", "http://esp-cam.local:81/stream")
BOT_IP            = os.environ.get("BOT_IP", "esp-bot.local")
BOT_PORT          = int(os.environ.get("BOT_PORT", "4210"))
SCADA_LISTEN_PORT = 4211

fault_config = {
    10: "FAULT",
    11: "OK",
    12: "FAULT",
    13: "OK",
}

BOX_POSITIONS = {
    10: (0.40, 1.70),
    11: (0.85, 1.70),
    12: (1.35, 1.70),
    13: (1.60, 1.70),
}
