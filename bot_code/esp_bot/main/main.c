/*
 * Bot firmware — running on ESP32-S3 (replaces Glyph-C6)
 *
 * Tasks: motor, udp_rx, encoder, tof, watchdog, repair
 * Console on USB Serial/JTAG (USB-CDC on S3).
 * VL53L0X on I2C0 (SDA=GPIO6, SCL=GPIO7), continuous mode.
 * SCADA IP learned dynamically from first UDP packet received.
 * Watchdog stops motors if no UDP for 500 ms.
 * REPAIR_MODE triggers forward drive → indicator → reverse → STATUS:REPAIRED.
 *
 * Pin changes from C6 → S3:
 *   STBY:  GPIO3  → GPIO4   (GPIO3 is strapping pin on S3)
 *   BIN1:  GPIO19 → GPIO40  (GPIO19 = USB D− on S3)
 *   BIN2:  GPIO20 → GPIO41  (GPIO20 = USB D+ on S3)
 *   PWMA:  GPIO16 → GPIO38  (GPIO16/17 = PSRAM on N16R8)
 *   PWMB:  GPIO17 → GPIO39
 *   ENC_L: GPIO22 → GPIO12  (GPIO22 not exposed on DevKitC-1)
 *   UART1: GPIO4  → GPIO2   (GPIO4 now used for STBY)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "led_strip.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "mdns.h"

/* ── Hardware / network constants ──────────────────────────────────────── */

#define WIFI_SSID        "kafka"
#define WIFI_PASS        "kafka000"
#define WIFI_MAX_RETRY   10

#define BOT_UDP_PORT     4210
#define SCADA_UDP_PORT   4211

#define RX_BUF_SIZE      256

/* ── TB6612FNG pin assignments ──────────────────────────────────────────── */

#define PIN_STBY   GPIO_NUM_4
#define PIN_AIN1   GPIO_NUM_8
#define PIN_AIN2   GPIO_NUM_9
#define PIN_BIN1   GPIO_NUM_40
#define PIN_BIN2   GPIO_NUM_41
#define PIN_PWMA   GPIO_NUM_38   /* left speed  — LEDC ch0 */
#define PIN_PWMB   GPIO_NUM_39   /* right speed — LEDC ch1 */

#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CH_LEFT   LEDC_CHANNEL_0
#define LEDC_CH_RIGHT  LEDC_CHANNEL_1
#define LEDC_FREQ_HZ   20000
#define LEDC_DUTY_BITS LEDC_TIMER_8_BIT

typedef struct { int left; int right; } motor_cmd_t;
static QueueHandle_t s_motor_q;

/* ── Encoder pin assignments ────────────────────────────────────────────── */

#define PIN_ENC_L_C1  GPIO_NUM_21
#define PIN_ENC_L_C2  GPIO_NUM_12
#define PIN_ENC_R_C1  GPIO_NUM_18
#define PIN_ENC_R_C2  GPIO_NUM_15

/* Odometry calibration. TICKS_PER_M is encoder counts per metre of wheel travel — CALIBRATE by
 * driving a measured 1.0 m straight and reading the summed ODOM deltas (scripts/udp_echo.py), then
 * set this so the sum ≈ 1.0. WHEEL_SEP_M must match arena_config.WHEEL_SEPARATION. The ENC_*_SIGN
 * flip a side to +1 if a FORWARD command yields positive ticks, -1 otherwise (depends on how the
 * C1/C2 phases and motor wiring line up — fix empirically so forward = positive on both sides). */
#define TICKS_PER_M   2465.0f
#define WHEEL_SEP_M   0.12f
#define ENC_L_SIGN    (-1)
#define ENC_R_SIGN    (+1)

static volatile int32_t s_left_ticks  = 0;
static volatile int32_t s_right_ticks = 0;

/* Quadrature decode: C1 fires the interrupt on its rising edge; the C2 phase level at that instant
 * gives the rotation direction, so the counts are signed (reverse decrements) — single-channel
 * counting could only ever go up. gpio_get_level is a plain register read, safe from an IRAM ISR. */
static void IRAM_ATTR enc_left_isr(void *arg)
{
    s_left_ticks  += gpio_get_level(PIN_ENC_L_C2) ? ENC_L_SIGN : -ENC_L_SIGN;
}
static void IRAM_ATTR enc_right_isr(void *arg)
{
    s_right_ticks += gpio_get_level(PIN_ENC_R_C2) ? ENC_R_SIGN : -ENC_R_SIGN;
}

/* ── VL53L0X I2C config ─────────────────────────────────────────────────── */

#define VL53L0X_ADDR    0x29
#define PIN_SDA         GPIO_NUM_6
#define PIN_SCL         GPIO_NUM_7
#define I2C_SPEED_HZ    10000    /* 10 kHz — safe with 45 kΩ internal pull-ups + any wire capacitance */

/* The ST-API init sequence below leaves the sensor reporting TRUE millimetres
 * natively — do NOT rescale it. If, after validating against a ruler at a sane
 * distance (100-500 mm, flat target), a consistent bias remains, measure the true
 * distance at ONE point and set TOF_OFFSET_MM = (true_mm − reported_mm). Leave it 0
 * for the native reading. (Readings below ~50 mm are inherently unreliable — the
 * sensor saturates and over-reads in the near field; don't calibrate there.) */
#define TOF_OFFSET_MM   -97
#define TOF_NO_TARGET   8190   /* sentinel the sensor returns when nothing is in range */
#define TOF_FILTER_N    5      /* median window — smooths per-sample jitter without lag */

/* ── Watchdog + repair constants ────────────────────────────────────────── */

#define WATCHDOG_TIMEOUT_MS     500   /* stop motors if no UDP for this long */
#define REPAIR_FWD_SPEED         90   /* PWM duty for forward drive into box */
#define REPAIR_FWD_SAFETY_MS   5000   /* hard timeout if SCADA never sends REPAIR_STOP */
#define REPAIR_INDICATOR_CYCLES   6   /* LED blink count after contact */

/* ── Closed-loop motion primitives ──────────────────────────────────────── */
/* TURN:<deg> and FWD:<mm> run an on-device encoder loop to completion, then report
 * STATUS:TURN_DONE / STATUS:FWD_DONE. The camera heading from the overhead ArUco is far
 * too noisy to servo on directly; SCADA uses it only to compute a GOAL, then fires one of
 * these — the ticks give repeatable rotation and, crucially, FWD equalises the two wheels'
 * travel so the bot drives (and reverses) dead straight instead of skidding. The outer loop
 * here just re-targets the motor queue every PRIM_TICK_MS; motor_task's ramp smooths it. */
#define PRIM_TICK_MS        10
#define PRIM_TURN_KP        0.6f   /* PWM per tick of remaining rotation error — lower = eases into
                                      target instead of driving near-full-speed then slamming to 0 */
#define PRIM_FWD_KP         0.6f   /* PWM per tick of remaining distance error */
#define PRIM_STRAIGHT_KP    2.0f   /* PWM per tick of left/right travel mismatch (anti-skid) */
#define PRIM_TOL_TICKS      2      /* done band per wheel (~0.8 mm / ~0.76°) */
#define PRIM_MIN_PWM        60     /* floor applied only while still out of band — the closed loop
                                      still reaches target at this duty; lower = gentle small nudges
                                      instead of an 80-PWM snap on every 1° correction */
#define PRIM_MAX_PWM        110    /* gentler top speed on turns/creeps (was 160 = lurchy) */
#define PRIM_SAFETY_MS      4000   /* hard per-primitive timeout — report DONE regardless */
#define PRIM_DONE_CYCLES    3      /* consecutive in-band cycles before declaring done */

typedef struct { int kind; float value; } prim_cmd_t;   /* kind: 0 = TURN (deg), 1 = FWD (mm) */
static QueueHandle_t s_prim_q;

/* ── Indicator LED ──────────────────────────────────────────────────────── */

#define PIN_LED  GPIO_NUM_47

/* ── RGB status LED (onboard WS2812 on GPIO48) ──────────────────────────── */

#define PIN_RGB        GPIO_NUM_48   /* onboard addressable WS2812 */
#define RGB_BRIGHTNESS 255           /* 0-255 global brightness cap; 255 = full */

/* hold_ms == 0 → set persistent base colour; > 0 → flash then revert to base */
typedef struct { uint8_t r, g, b; uint32_t hold_ms; } rgb_cmd_t;
static led_strip_handle_t s_rgb;
static QueueHandle_t      s_rgb_q;
static rgb_cmd_t          s_rgb_base = {0, 0, 0, 0};

/* Shared handles; NULL if sensor failed to init */
static i2c_master_bus_handle_t s_vl53_bus = NULL;
static i2c_master_dev_handle_t s_vl53_dev = NULL;

/* ── Internal ───────────────────────────────────────────────────────────── */

static const char *TAG = "bot_s3";

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry = 0;
static int s_sock  = -1;

/*
 * Outbound SCADA destination — learned dynamically from whoever sends the first
 * UDP command.  Initialized to "not yet known" (sin_addr = 0); tof_task and
 * encoder_task skip sending until a packet has been received.
 */
static struct sockaddr_in s_scada_addr = {
    .sin_family      = AF_INET,
    .sin_addr.s_addr = 0,
    .sin_port        = htons(SCADA_UDP_PORT),
};
static portMUX_TYPE s_scada_mux = portMUX_INITIALIZER_UNLOCKED;

/* Watchdog: ms timestamp of the last received UDP packet (0 = not yet seen) */
static volatile uint32_t s_last_rx_ms = 0;

/* repair_task handle — udp_rx_task notifies it on REPAIR_MODE */
static TaskHandle_t s_repair_task_handle = NULL;

/* Set true during repair cycle so the watchdog doesn't override motor commands */
static volatile bool s_repair_active = false;

/* Set true while a TURN/FWD primitive runs so the watchdog doesn't zero the motors mid-move */
static volatile bool s_primitive_active = false;

/* Set true by udp_rx_task on "REPAIR_STOP"; ends the forward drive in repair_task */
static volatile bool s_repair_stop = false;

/* ── Motor init ─────────────────────────────────────────────────────────── */

static void motors_init(void)
{
    gpio_config_t gc = {
        .pin_bit_mask = (1ULL << PIN_STBY) | (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2)
                       | (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gc);
    gpio_set_level(PIN_STBY, 0);
    gpio_set_level(PIN_AIN1, 0);
    gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0);
    gpio_set_level(PIN_BIN2, 0);

    ledc_timer_config_t lt = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_BITS,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&lt));

    ledc_channel_config_t lc = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_LEFT,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_PWMA,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lc));
    lc.channel  = LEDC_CH_RIGHT;
    lc.gpio_num = PIN_PWMB;
    ESP_ERROR_CHECK(ledc_channel_config(&lc));

    gpio_set_level(PIN_STBY, 1);
    ESP_LOGI(TAG, "motors init done");
}

static void set_motor_side(gpio_num_t in1, gpio_num_t in2,
                            ledc_channel_t ch, int speed)
{
    int duty = abs(speed);
    if (duty > 255) duty = 255;

    if (speed > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (speed < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 1);
    }
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

/* ── Motor task ─────────────────────────────────────────────────────────── */

/* Slew-rate ramp. The SCADA side floors any nonzero wheel command up to MIN_MOVE_PWM (~100) to
 * beat static friction, so even a "gentle" pan or dock-creep arrives as a full ±100 step. Applied
 * instantly that snaps the wheels — blurring the S3's view during the inspection sweep and jerking
 * every traversal. Ramp the applied duty toward the commanded target a few counts per 10 ms tick
 * instead. ASYMMETRIC on purpose: accelerate slowly (small, blur-free, fine-grained pulses) but
 * decelerate fast, so STOP / dock-stop still bite within ~25 ms and docking gains no extra coast. */
#define RAMP_TICK_MS   10
#define RAMP_ACCEL     6    /* duty counts per tick while speeding up (gentler launch) */
#define RAMP_DECEL     14   /* duty counts per tick while slowing/reversing — soft stop instead of a
                               40/tick slam, which was the hard jerk at the end of every primitive.
                               Still fast enough (100→0 in ~70 ms) for the watchdog-STOP to be safe. */

static int ramp_toward(int cur, int tgt)
{
    /* accelerating = same sign (or from rest) AND growing in magnitude; else decelerating */
    bool accel = (cur == 0 || (tgt >= 0) == (cur >= 0)) && abs(tgt) >= abs(cur);
    int step = accel ? RAMP_ACCEL : RAMP_DECEL;
    if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

static void motor_task(void *arg)
{
    motor_cmd_t cmd;
    int tgt_l = 0, tgt_r = 0;   /* latest commanded target */
    int cur_l = 0, cur_r = 0;   /* duty actually applied, ramped toward target */
    while (1) {
        /* Non-blocking: grab the latest target if one was posted, else hold the last one. */
        if (xQueueReceive(s_motor_q, &cmd, 0) == pdTRUE) {
            tgt_l = cmd.left;
            tgt_r = cmd.right;
        }
        cur_l = ramp_toward(cur_l, tgt_l);
        cur_r = ramp_toward(cur_r, tgt_r);
        set_motor_side(PIN_AIN1, PIN_AIN2, LEDC_CH_LEFT,  cur_l);
        set_motor_side(PIN_BIN1, PIN_BIN2, LEDC_CH_RIGHT, cur_r);
        vTaskDelay(pdMS_TO_TICKS(RAMP_TICK_MS));
    }
}

/* ── Encoder init ───────────────────────────────────────────────────────── */

static void encoders_init(void)
{
    /* C1 channels: interrupt on the rising edge — these are the tick source. */
    gpio_config_t gc = {
        .pin_bit_mask = (1ULL << PIN_ENC_L_C1) | (1ULL << PIN_ENC_R_C1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&gc);

    /* C2 channels: plain inputs, sampled inside the C1 ISR for the quadrature direction bit. */
    gpio_config_t gc2 = {
        .pin_bit_mask = (1ULL << PIN_ENC_L_C2) | (1ULL << PIN_ENC_R_C2),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gc2);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_L_C1, enc_left_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_R_C1, enc_right_isr, NULL);

    ESP_LOGI(TAG, "encoders init done (quadrature)");
}

/* ── Encoder task ───────────────────────────────────────────────────────── */

static void encoder_task(void *arg)
{
    char buf[64];
    int32_t prev_l = 0, prev_r = 0;
    float heading = 0.0f;   /* integrated absolute heading (rad) in the odom frame */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_sock < 0) continue;

        struct sockaddr_in scada;
        taskENTER_CRITICAL(&s_scada_mux);
        scada = s_scada_addr;
        taskEXIT_CRITICAL(&s_scada_mux);
        if (scada.sin_addr.s_addr == 0) continue;   /* no SCADA seen yet */

        /* Per-interval SIGNED wheel travel in metres. The SCADA udp_bridge expects exactly
         * ODOM:dl_m:dr_m:heading_rad — dl/dr integrated into x/y, heading taken as ABSOLUTE —
         * so we integrate heading here on-device from the differential-drive kinematics. */
        int32_t l = s_left_ticks;
        int32_t r = s_right_ticks;
        float dl = (l - prev_l) / TICKS_PER_M;
        float dr = (r - prev_r) / TICKS_PER_M;
        prev_l = l;
        prev_r = r;

        heading += (dr - dl) / WHEEL_SEP_M;
        if (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
        if (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

        int n = snprintf(buf, sizeof(buf), "ODOM:%.5f:%.5f:%.5f\n", dl, dr, heading);
        sendto(s_sock, buf, n, 0, (struct sockaddr *)&scada, sizeof(scada));
    }
}

/* ── VL53L0X I2C helpers ────────────────────────────────────────────────── */

static esp_err_t vl53_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_vl53_dev, buf, 2, 50);
}

static esp_err_t vl53_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_vl53_dev, &reg, 1, val, 1, 50);
}

static esp_err_t vl53_read16(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2] = {0, 0};
    esp_err_t ret = i2c_master_transmit_receive(s_vl53_dev, &reg, 1, buf, 2, 50);
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return ret;
}

/* ── VL53L0X reference calibration ─────────────────────────────────────── */
/*
 * Single-shot calibration measurement required twice before continuous mode:
 *   vhv_init_byte = 0x40  →  VHV cal  (SYSRANGE_START = 0x41)
 *   vhv_init_byte = 0x00  →  Phase cal (SYSRANGE_START = 0x01)
 * Without both, comparator thresholds and timing offsets are uninitialised
 * and every measurement returns 8190 (no-target sentinel).
 */
static void vl53_perform_ref_calibration(uint8_t vhv_init_byte)
{
    vl53_write(0x00, 0x01 | vhv_init_byte);

    uint8_t status = 0;
    for (int i = 0; i < 200; i++) {
        vl53_read(0x13, &status);
        if (status & 0x07) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!(status & 0x07)) {
        ESP_LOGW(TAG, "VL53L0X ref cal timeout (init_byte=0x%02X)", vhv_init_byte);
    }

    vl53_write(0x0B, 0x01);   /* clear interrupt */
    vl53_write(0x00, 0x00);   /* stop single shot */
}

/* ── VL53L0X init ───────────────────────────────────────────────────────── */
/*
 * Mandatory register sequence derived from ST VL53L0X API.
 * stop_var is device-specific and must be read then written back during the
 * continuous-ranging start sequence — it calibrates the internal stop condition.
 */
static esp_err_t vl53l0x_init(void)
{
    /* Reset pins before claiming them for I2C to ensure a clean bus state. */
    gpio_reset_pin(PIN_SDA);
    gpio_reset_pin(PIN_SCL);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* --- I2C bus + device --- */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = I2C_NUM_0,
        .scl_io_num             = PIN_SCL,
        .sda_io_num             = PIN_SDA,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_vl53_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VL53L0X_ADDR,
        .scl_speed_hz    = I2C_SPEED_HZ,
    };
    ret = i2c_master_bus_add_device(s_vl53_bus, &dev_cfg, &s_vl53_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* HW_CLR_BUS toggles SCL 9× to unlock any stuck slave and resets the FSM */
    i2c_master_bus_reset(s_vl53_bus);
    vTaskDelay(pdMS_TO_TICKS(10));   /* VL53L0X boot time ≤ 1.2 ms, 10 ms is generous */

    /* Probe first — gives a clean error if the sensor is physically absent */
    ret = i2c_master_probe(s_vl53_bus, VL53L0X_ADDR, 50);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X not found on I2C bus (probe): %s — check SDA/SCL wiring and sensor power",
                 esp_err_to_name(ret));
        s_vl53_dev = NULL;
        return ret;
    }

    /* --- Verify model ID --- */
    uint8_t id = 0;
    ret = vl53_read(0xC0, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L0X ID read failed: %s (check pull-ups on SDA/SCL)", esp_err_to_name(ret));
        s_vl53_dev = NULL;
        return ret;
    }
    if (id != 0xEE) {
        ESP_LOGE(TAG, "VL53L0X not found (ID=0x%02X, expected 0xEE)", id);
        s_vl53_dev = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L0X ID OK (0xEE)");

    /* --- Mandatory tuning settings (from ST API init sequence) --- */
    vl53_write(0x88, 0x00);

    vl53_write(0x80, 0x01);
    vl53_write(0xFF, 0x01);
    vl53_write(0x00, 0x00);
    uint8_t stop_var = 0;
    vl53_read(0x91, &stop_var);
    vl53_write(0x00, 0x01);
    vl53_write(0xFF, 0x00);
    vl53_write(0x80, 0x00);

    uint8_t msrc = 0;
    vl53_read(0x60, &msrc);
    vl53_write(0x60, msrc | 0x12);   /* disable MSRC + PRE_RANGE signal rate checks */
    vl53_write(0xFF, 0x01);
    vl53_write(0x4E, 0x2C);
    vl53_write(0x48, 0x00);
    vl53_write(0x30, 0x20);
    vl53_write(0xFF, 0x00);
    vl53_write(0x30, 0x09);
    vl53_write(0x54, 0x10);
    vl53_write(0x31, 0x04);
    vl53_write(0x32, 0x03);
    vl53_write(0x40, 0x83);
    vl53_write(0x46, 0x25);
    vl53_write(0x60, 0x00);
    vl53_write(0x27, 0x00);
    vl53_write(0x50, 0x06);
    vl53_write(0x51, 0x00);
    vl53_write(0x52, 0x96);
    vl53_write(0x56, 0x08);
    vl53_write(0x57, 0x30);
    vl53_write(0x61, 0x00);
    vl53_write(0x62, 0x00);
    vl53_write(0x64, 0x00);
    vl53_write(0x65, 0x00);
    vl53_write(0x66, 0xA0);

    vl53_write(0xFF, 0x01);
    vl53_write(0x22, 0x32);
    vl53_write(0x47, 0x14);
    vl53_write(0x49, 0xFF);
    vl53_write(0x4A, 0x00);

    vl53_write(0xFF, 0x00);
    vl53_write(0x7A, 0x0A);
    vl53_write(0x7B, 0x00);
    vl53_write(0x78, 0x21);

    vl53_write(0xFF, 0x01);
    vl53_write(0x23, 0x34);
    vl53_write(0x42, 0x00);
    vl53_write(0x44, 0xFF);
    vl53_write(0x45, 0x26);
    vl53_write(0x46, 0x05);
    vl53_write(0x40, 0x40);
    vl53_write(0x0E, 0x06);
    vl53_write(0x20, 0x1A);
    vl53_write(0x43, 0x40);

    vl53_write(0xFF, 0x00);
    vl53_write(0x34, 0x03);
    vl53_write(0x35, 0x44);

    vl53_write(0xFF, 0x01);
    vl53_write(0x31, 0x04);
    vl53_write(0x4B, 0x09);
    vl53_write(0x4C, 0x05);
    vl53_write(0x4D, 0x04);

    vl53_write(0xFF, 0x00);
    vl53_write(0x44, 0x00);
    vl53_write(0x45, 0x20);
    vl53_write(0x47, 0x08);
    vl53_write(0x48, 0x28);
    vl53_write(0x67, 0x00);
    vl53_write(0x70, 0x04);
    vl53_write(0x71, 0x01);
    vl53_write(0x72, 0xFE);
    vl53_write(0x76, 0x00);
    vl53_write(0x77, 0x00);

    vl53_write(0xFF, 0x01);
    vl53_write(0x0D, 0x01);

    vl53_write(0xFF, 0x00);
    vl53_write(0x80, 0x01);
    vl53_write(0x01, 0xF8);

    vl53_write(0xFF, 0x01);
    vl53_write(0x8E, 0x01);
    vl53_write(0x00, 0x01);
    vl53_write(0xFF, 0x00);
    vl53_write(0x80, 0x00);

    /* GPIO: interrupt when new sample is ready */
    vl53_write(0x0A, 0x04);
    vl53_write(0x0B, 0x01);   /* clear any pending interrupt */

    /* VHV calibration — loads comparator thresholds from factory NVM */
    vl53_write(0x01, 0x01);               /* SYSTEM_SEQUENCE_CONFIG: TCC step only */
    vl53_perform_ref_calibration(0x40);

    /* Phase calibration — sets return-signal timing offsets */
    vl53_write(0x01, 0x02);               /* SYSTEM_SEQUENCE_CONFIG: DSS step only */
    vl53_perform_ref_calibration(0x00);

    /* Restore normal sequence steps for continuous ranging */
    vl53_write(0x01, 0xE8);

    /* Start continuous back-to-back ranging */
    vl53_write(0x80, 0x01);
    vl53_write(0xFF, 0x01);
    vl53_write(0x00, 0x00);
    vl53_write(0x91, stop_var);
    vl53_write(0x00, 0x01);
    vl53_write(0xFF, 0x00);
    vl53_write(0x80, 0x00);
    vl53_write(0x00, 0x02);   /* SYSRANGE_START: continuous mode */

    ESP_LOGI(TAG, "VL53L0X init done — continuous ranging started");
    return ESP_OK;
}

/* ── TOF reading finalize ───────────────────────────────────────────────── */
/* The value read from register 0x1E is already true millimetres. Pass it through,
 * applying only the honest single-point offset and the no-target sentinel. */
static uint16_t tof_finalize(uint16_t raw)
{
    if (raw >= 8000) return TOF_NO_TARGET;   /* no target in range */
    int v = (int)raw + TOF_OFFSET_MM;
    if (v < 0) v = 0;
    return (uint16_t)v;
}

/* ── TOF task ───────────────────────────────────────────────────────────── */
/*
 * Polls the sensor interrupt status (~33 ms measurement period), reads the
 * 16-bit range result at register 0x1E (already true mm), then sends TOF:<mm>\n
 * to SCADA. TOF_NO_TARGET (8190) is emitted when the sensor reports no target.
 */
static void tof_task(void *arg)
{
    char buf[24];

    while (1) {
        /* Poll RESULT_INTERRUPT_STATUS (0x13) bits[2:0] — non-zero = ready */
        uint8_t status  = 0;
        uint8_t retries = 0;
        do {
            vl53_read(0x13, &status);
            if (status & 0x07) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        } while (++retries < 50);   /* max ~100 ms wait */

        uint16_t range_mm = TOF_NO_TARGET;
        if (status & 0x07) {
            vl53_read16(0x1E, &range_mm);
            vl53_write(0x0B, 0x01);   /* clear interrupt so next reading is fresh */
            range_mm = tof_finalize(range_mm);
        }

        if (s_sock < 0) continue;

        struct sockaddr_in scada;
        taskENTER_CRITICAL(&s_scada_mux);
        scada = s_scada_addr;
        taskEXIT_CRITICAL(&s_scada_mux);
        if (scada.sin_addr.s_addr == 0) continue;   /* no SCADA seen yet */

        int n = snprintf(buf, sizeof(buf), "TOF:%d\n", (int)range_mm);
        sendto(s_sock, buf, n, 0, (struct sockaddr *)&scada, sizeof(scada));
    }
}

/* ── Wi-Fi event handler ────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_set_max_tx_power(40);   /* limit before first probe burst */
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry++;
            ESP_LOGI(TAG, "retrying Wi-Fi (%d/%d)", s_retry, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi failed after %d retries", WIFI_MAX_RETRY);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* ── Wi-Fi init ─────────────────────────────────────────────────────────── */

static void wifi_init_sta(void)
{
    s_wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,  wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected to SSID: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Wi-Fi connection failed — check SSID/password");
    }
}

/* ── UDP socket init ────────────────────────────────────────────────────── */

static void udp_socket_init(void)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(BOT_UDP_PORT),
    };
    if (bind(s_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(s_sock);
        s_sock = -1;
        return;
    }
    ESP_LOGI(TAG, "UDP socket bound on port %d", BOT_UDP_PORT);
}

/* ── UDP RX task ────────────────────────────────────────────────────────── */

static void udp_rx_task(void *arg)
{
    static char rx_buf[RX_BUF_SIZE];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);
    motor_cmd_t cmd = {0, 0};

    ESP_LOGI(TAG, "udp_rx_task started — listening on port %d", BOT_UDP_PORT);

    while (1) {
        int n = recvfrom(s_sock, rx_buf, sizeof(rx_buf) - 1, 0,
                         (struct sockaddr *)&sender, &sender_len);
        if (n < 0) {
            ESP_LOGE(TAG, "recvfrom error: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        rx_buf[n] = '\0';
        ESP_LOGI(TAG, "RX(%d): %s", n, rx_buf);

        /* Update SCADA destination and reset watchdog timer */
        taskENTER_CRITICAL(&s_scada_mux);
        s_scada_addr.sin_addr = sender.sin_addr;
        taskEXIT_CRITICAL(&s_scada_mux);
        s_last_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (strncmp(rx_buf, "LEFT:", 5) == 0) {
            int l = 0, r = 0;
            char *p = strstr(rx_buf, "RIGHT:");
            l = atoi(rx_buf + 5);
            if (p) r = atoi(p + 6);
            cmd.left  = l;
            cmd.right = r;
            xQueueOverwrite(s_motor_q, &cmd);
        } else if (strncmp(rx_buf, "TURN:", 5) == 0) {
            prim_cmd_t pc = {0, (float)atof(rx_buf + 5)};
            xQueueOverwrite(s_prim_q, &pc);
        } else if (strncmp(rx_buf, "FWD:", 4) == 0) {
            prim_cmd_t pc = {1, (float)atof(rx_buf + 4)};
            xQueueOverwrite(s_prim_q, &pc);
        } else if (strncmp(rx_buf, "STOP", 4) == 0) {
            cmd.left = 0;
            cmd.right = 0;
            xQueueOverwrite(s_motor_q, &cmd);
            ESP_LOGI(TAG, "STOP");
        } else if (strcmp(rx_buf, "REPAIR_MODE") == 0) {
            if (s_repair_task_handle) {
                xTaskNotifyGive(s_repair_task_handle);
            }
        } else if (strcmp(rx_buf, "REPAIR_STOP") == 0) {
            s_repair_stop = true;
            ESP_LOGI(TAG, "REPAIR_STOP received");
        } else if (strcmp(rx_buf, "SCAN") == 0) {
            uart_write_bytes(UART_NUM_1, "SCAN\n", 5);
            ESP_LOGI(TAG, "SCAN forwarded to S3");
        } else if (strncmp(rx_buf, "LED:", 4) == 0) {
            unsigned int r = 0, g = 0, b = 0, ms = 0;
            sscanf(rx_buf + 4, "%u,%u,%u,%u", &r, &g, &b, &ms);
            rgb_cmd_t pc = {(uint8_t)r, (uint8_t)g, (uint8_t)b, ms};
            xQueueSend(s_rgb_q, &pc, 0);
        } else {
            sendto(s_sock, rx_buf, n, 0,
                   (struct sockaddr *)&sender, sender_len);
        }
    }
}

/* ── Watchdog task ──────────────────────────────────────────────────────── */

static void watchdog_task(void *arg)
{
    motor_cmd_t stop = {0, 0};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_last_rx_ms == 0) continue;   /* not yet connected */
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (!s_repair_active && !s_primitive_active && (now - s_last_rx_ms) > WATCHDOG_TIMEOUT_MS) {
            xQueueOverwrite(s_motor_q, &stop);
        }
    }
}

/* ── Motion primitive task ──────────────────────────────────────────────── */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Map a raw P-controller output to a motor duty: cap to ±PRIM_MAX_PWM, and while the wheel
 * is still out of band floor the magnitude up to PRIM_MIN_PWM so static friction is beaten;
 * once in band, command 0 so the wheel doesn't creep past the target. */
static int prim_pwm(float raw, int32_t abserr)
{
    int p = (int)raw;
    if (p >  PRIM_MAX_PWM) p =  PRIM_MAX_PWM;
    if (p < -PRIM_MAX_PWM) p = -PRIM_MAX_PWM;
    if (abserr > PRIM_TOL_TICKS) {
        if (p > 0 && p <  PRIM_MIN_PWM) p =  PRIM_MIN_PWM;
        if (p < 0 && p > -PRIM_MIN_PWM) p = -PRIM_MIN_PWM;
    } else {
        p = 0;
    }
    return p;
}

/* In-place rotation by `deg` (signed; CCW positive, matching the odom heading integration
 * heading += (dr - dl)/WHEEL_SEP). CCW ⇒ left wheel back, right wheel forward. */
static void run_turn(float deg)
{
    int32_t l0 = s_left_ticks, r0 = s_right_ticks;
    float   rad  = deg * (float)M_PI / 180.0f;
    float   sign = (rad >= 0.0f) ? 1.0f : -1.0f;
    int32_t tgt  = (int32_t)(fabsf(rad) * (WHEEL_SEP_M / 2.0f) * TICKS_PER_M);
    int32_t lt   = l0 - (int32_t)(sign * tgt);
    int32_t rt   = r0 + (int32_t)(sign * tgt);

    int done = 0;
    uint32_t t0 = now_ms();
    while (1) {
        int32_t el = lt - s_left_ticks;
        int32_t er = rt - s_right_ticks;
        if (abs(el) <= PRIM_TOL_TICKS && abs(er) <= PRIM_TOL_TICKS) {
            if (++done >= PRIM_DONE_CYCLES) break;
        } else {
            done = 0;
        }
        motor_cmd_t c = {
            prim_pwm(PRIM_TURN_KP * (float)el, abs(el)),
            prim_pwm(PRIM_TURN_KP * (float)er, abs(er)),
        };
        xQueueOverwrite(s_motor_q, &c);
        if (now_ms() - t0 > PRIM_SAFETY_MS) break;
        vTaskDelay(pdMS_TO_TICKS(PRIM_TICK_MS));
    }
    motor_cmd_t stop = {0, 0};
    xQueueOverwrite(s_motor_q, &stop);
}

/* Straight drive by `mm` (signed; negative = reverse). base PWM comes from the mean remaining
 * distance; the steer term feeds back the mismatch between the two wheels' travel so they stay
 * locked together — this is what keeps the line straight and kills the backtrack skid. */
static void run_fwd(float mm)
{
    int32_t l0 = s_left_ticks, r0 = s_right_ticks;
    int32_t tgt = (int32_t)((mm / 1000.0f) * TICKS_PER_M);   /* signed */
    int32_t lt  = l0 + tgt;
    int32_t rt  = r0 + tgt;

    int done = 0;
    uint32_t t0 = now_ms();
    while (1) {
        int32_t el = lt - s_left_ticks;
        int32_t er = rt - s_right_ticks;
        if (abs(el) <= PRIM_TOL_TICKS && abs(er) <= PRIM_TOL_TICKS) {
            if (++done >= PRIM_DONE_CYCLES) break;
        } else {
            done = 0;
        }
        float meanerr = (float)(el + er) / 2.0f;
        float steer   = PRIM_STRAIGHT_KP * (float)((s_right_ticks - r0) - (s_left_ticks - l0));
        int   base    = (int)(PRIM_FWD_KP * meanerr);
        if (meanerr >  PRIM_TOL_TICKS && base <  PRIM_MIN_PWM) base =  PRIM_MIN_PWM;
        if (meanerr < -PRIM_TOL_TICKS && base > -PRIM_MIN_PWM) base = -PRIM_MIN_PWM;

        int pl = base + (int)steer;
        int pr = base - (int)steer;
        if (pl >  PRIM_MAX_PWM) pl =  PRIM_MAX_PWM;
        if (pl < -PRIM_MAX_PWM) pl = -PRIM_MAX_PWM;
        if (pr >  PRIM_MAX_PWM) pr =  PRIM_MAX_PWM;
        if (pr < -PRIM_MAX_PWM) pr = -PRIM_MAX_PWM;

        motor_cmd_t c = {pl, pr};
        xQueueOverwrite(s_motor_q, &c);
        if (now_ms() - t0 > PRIM_SAFETY_MS) break;
        vTaskDelay(pdMS_TO_TICKS(PRIM_TICK_MS));
    }
    motor_cmd_t stop = {0, 0};
    xQueueOverwrite(s_motor_q, &stop);
}

static void primitive_task(void *arg)
{
    prim_cmd_t pc;
    while (1) {
        if (xQueueReceive(s_prim_q, &pc, portMAX_DELAY) != pdTRUE) continue;
        s_primitive_active = true;
        if (pc.kind == 0) run_turn(pc.value);
        else              run_fwd(pc.value);
        s_primitive_active = false;

        struct sockaddr_in scada;
        taskENTER_CRITICAL(&s_scada_mux);
        scada = s_scada_addr;
        taskEXIT_CRITICAL(&s_scada_mux);
        if (scada.sin_addr.s_addr != 0) {
            const char *msg = (pc.kind == 0) ? "STATUS:TURN_DONE\n" : "STATUS:FWD_DONE\n";
            sendto(s_sock, msg, strlen(msg), 0, (struct sockaddr *)&scada, sizeof(scada));
        }
    }
}

/* ── RGB status LED ─────────────────────────────────────────────────────── */

static void rgb_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = PIN_RGB,
        .max_leds       = 1,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,   /* 10 MHz RMT tick */
    };
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_rgb) == ESP_OK) {
        led_strip_clear(s_rgb);
    } else {
        ESP_LOGW(TAG, "RGB LED init failed");
        s_rgb = NULL;
    }
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_rgb) return;
    led_strip_set_pixel(s_rgb, 0,
                        (uint32_t)r * RGB_BRIGHTNESS / 255,
                        (uint32_t)g * RGB_BRIGHTNESS / 255,
                        (uint32_t)b * RGB_BRIGHTNESS / 255);
    led_strip_refresh(s_rgb);
}

/*
 * Drives the WS2812 from LED commands. A command with hold_ms == 0 sets the
 * persistent base colour; hold_ms > 0 flashes the colour for that long. A flash
 * always runs its full duration — base-colour updates that arrive during a
 * flash are stored and only displayed once the flash expires. This lets SCADA
 * fire a verdict flash and immediately transition to the next state's base
 * colour without the flash being clobbered.
 */
static void rgb_task(void *arg)
{
    rgb_cmd_t pc;
    uint32_t flash_until = 0;
    while (1) {
        if (xQueueReceive(s_rgb_q, &pc, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (pc.hold_ms == 0) {
                s_rgb_base = pc;
                if (now_ms() >= flash_until) rgb_set(pc.r, pc.g, pc.b);
            } else {
                flash_until = now_ms() + pc.hold_ms;
                rgb_set(pc.r, pc.g, pc.b);
            }
        } else if (flash_until && now_ms() >= flash_until) {
            flash_until = 0;
            rgb_set(s_rgb_base.r, s_rgb_base.g, s_rgb_base.b);
        }
    }
}

/* ── Repair task ────────────────────────────────────────────────────────── */
/*
 * Waits indefinitely for a task notification (sent by udp_rx_task on REPAIR_MODE).
 * Drives forward until SCADA sends REPAIR_STOP (or REPAIR_FWD_SAFETY_MS elapses),
 * records actual forward time, then reverses for the same duration so the bot
 * returns to its pre-repair position.
 */
static void repair_task(void *arg)
{
    motor_cmd_t cmd;
    motor_cmd_t stop = {0, 0};

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_repair_active = true;
        s_repair_stop   = false;

        /* Drive forward — stop when SCADA sends REPAIR_STOP or safety timeout */
        cmd.left  = REPAIR_FWD_SPEED;
        cmd.right = REPAIR_FWD_SPEED;
        xQueueOverwrite(s_motor_q, &cmd);

        uint32_t t_start = (uint32_t)(esp_timer_get_time() / 1000);
        while (!s_repair_stop) {
            uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t_start;
            if (elapsed >= REPAIR_FWD_SAFETY_MS) break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        uint32_t actual_ms = (uint32_t)(esp_timer_get_time() / 1000) - t_start;
        s_repair_stop = false;
        xQueueOverwrite(s_motor_q, &stop);

        /* LED indicator: 6 blink cycles */
        for (int i = 0; i < REPAIR_INDICATOR_CYCLES; i++) {
            gpio_set_level(PIN_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(PIN_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* Reverse for the same duration the forward drive took */
        cmd.left  = -REPAIR_FWD_SPEED;
        cmd.right = -REPAIR_FWD_SPEED;
        xQueueOverwrite(s_motor_q, &cmd);
        vTaskDelay(pdMS_TO_TICKS(actual_ms));
        xQueueOverwrite(s_motor_q, &stop);

        /* Notify SCADA */
        struct sockaddr_in scada;
        taskENTER_CRITICAL(&s_scada_mux);
        scada = s_scada_addr;
        taskEXIT_CRITICAL(&s_scada_mux);
        if (scada.sin_addr.s_addr != 0) {
            const char *msg = "STATUS:REPAIRED\n";
            sendto(s_sock, msg, strlen(msg), 0,
                   (struct sockaddr *)&scada, sizeof(scada));
        }
        s_repair_active = false;
        ESP_LOGI(TAG, "Repair cycle complete — fwd=%lums STATUS:REPAIRED sent", (unsigned long)actual_ms);
    }
}

/* ── Entry point ────────────────────────────────────────────────────────── */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== Bot S3 (control) booting ===");

    s_motor_q = xQueueCreate(1, sizeof(motor_cmd_t));
    s_prim_q  = xQueueCreate(1, sizeof(prim_cmd_t));
    motors_init();
    encoders_init();

    /* Indicator LED */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);
    gpio_set_level(PIN_LED, 0);

    /* RGB status LED */
    s_rgb_q = xQueueCreate(8, sizeof(rgb_cmd_t));
    rgb_init();
    rgb_set(30, 30, 30);   /* dim white on boot */
    s_rgb_base = (rgb_cmd_t){30, 30, 30, 0};

    /* VL53L0X init runs before Wi-Fi so failures are visible immediately */
    vl53l0x_init();

    wifi_init_sta();

    mdns_init();
    mdns_hostname_set("esp-bot");
    mdns_instance_name_set("Warden Bot");
    ESP_LOGI(TAG, "mDNS started — hostname: esp-bot.local");

    udp_socket_init();

    /* UART1 TX on GPIO2 → camera S3 UART RX: forwards SCAN command */
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_2, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(motor_task,   "motor",   2048, NULL, 6, NULL);
    xTaskCreate(udp_rx_task,  "udp_rx",  4096, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder", 4096, NULL, 4, NULL);

    if (s_vl53_dev != NULL) {
        xTaskCreate(tof_task, "tof", 2048, NULL, 3, NULL);
    } else {
        ESP_LOGW(TAG, "TOF task not started — VL53L0X not detected");
    }

    xTaskCreate(watchdog_task,  "watchdog", 1024, NULL, 7, NULL);
    xTaskCreate(repair_task,    "repair",   2048, NULL, 5, &s_repair_task_handle);
    xTaskCreate(primitive_task, "prim",     4096, NULL, 6, NULL);
    xTaskCreate(rgb_task,       "rgb",      2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "All tasks started — ready");
}
