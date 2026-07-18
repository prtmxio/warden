WHEEL_SEPARATION  = 0.11
MAX_PWM           = 180
MAX_LINEAR_VEL    = 0.15
WARP_SIZE         = 800
ARENA_SIZE_CM     = 150
PX_PER_CM         = WARP_SIZE / ARENA_SIZE_CM
DOCKING_STOP_MM   = 100
DOCKING_PHASE_CM  = 20
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
BOT_IP            = "192.168.43.__"
BOT_PORT          = 4210
SCADA_LISTEN_PORT = 4211

fault_config = {
    10: "FAULT",
    11: "OK",
    12: "FAULT",
    13: "OK",
}

BOX_POSITIONS = {
    10: (0.30, 0.30),
    11: (1.20, 0.30),
    12: (1.20, 1.20),
    13: (0.30, 1.20),
}
