WHEEL_SEPARATION  = 0.12
MAX_PWM           = 180
MAX_LINEAR_VEL    = 0.15
WARP_SIZE         = 800
ARENA_SIZE_CM     = 200
PX_PER_CM         = WARP_SIZE / ARENA_SIZE_CM
DOCKING_STOP_MM   = 600   # mm — legacy single-threshold, superseded by SCAN/DOCK below
DOCKING_PHASE_CM  = 40    # cm — legacy Nav2 pre-dock goal, superseded by NAV_STANDOFF_M

# Two-stage approach: read the ArUco at scan distance, creep to dock distance only to repair
SCAN_STOP_MM     = 170    # mm — stop-and-scan distance (~13 cm) for reading the box ArUco
DOCK_STOP_MM     = 95     # mm — repair docking distance (~6 cm), only during a repair
NAV_STANDOFF_M   = 0.45   # m  — Nav2 pre-dock goal south of box centre (clear of inflation;
                          #      the TOF creep covers the rest down to SCAN_STOP_MM)
PERP_TOL_DEG     = 1.5    # deg — max |yaw − box-normal| for a valid (full-view) ArUco read
LATERAL_TOL_M    = 0.06   # m  — max |camera_x − box_x| for the QR to sit fully in view
APPROACH_SPEED   = 0.05   # m/s — straight-creep speed during scan/dock approach
APPROACH_YAW_KP  = 2.0    # P gain — hold perpendicular heading while creeping forward
ALIGN_YAW_KP     = 1.5    # P gain — rotate to perpendicular / face the centerline

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

BOX_HSV_LO   = (0, 101, 182)
BOX_HSV_HI   = (179, 255, 255)
BOX_MIN_AREA = 5000

# IPs — update before first run
PHONE_STREAM_URL  = "http://192.168.43.1:8080/video"
S3_STREAM_URL     = "http://192.168.43.__:81/stream"
BOT_IP            = "127.0.0.1"   # localhost for mock_bot; set real C6 IP before hardware run
BOT_PORT          = 4210
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
