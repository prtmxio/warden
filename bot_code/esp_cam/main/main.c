/*
 * ESP32-S3 firmware — Day 6
 *
 * Tasks: stream_server (HTTP MJPEG on port 81), uart_scan (UART1 RX GPIO2)
 * Camera: OV5640 via camera_core_init() — QVGA JPEG PSRAM GRAB_LATEST
 * UART: receives SCAN\n from C6 (TX GPIO4) on GPIO2, flashes LED 200ms
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_http_server.h"
#include "esp_camera.h"

#include "camera_core.h"
#include "img_converters.h"

/* ── Network constants ──────────────────────────────────────────────────── */

#define WIFI_SSID        "kafka"
#define WIFI_PASS        "kafka000"
#define WIFI_MAX_RETRY   10

/* ── Hardware constants ─────────────────────────────────────────────────── */

#define PIN_UART_RX   GPIO_NUM_2    /* UART1 RX — receives SCAN from C6 GPIO4 */
#define PIN_LED       GPIO_NUM_48   /* onboard LED — adjust if board differs    */
#define UART_RX_BUF   256

static const char *TAG = "esp32_s3";

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry = 0;

/* ── Wi-Fi ──────────────────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
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

static void wifi_init_sta(void)
{
    s_wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL, NULL));
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
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Wi-Fi failed — check SSID/password");
    }
}

/* ── MJPEG stream handler ───────────────────────────────────────────────── */

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req,
        "multipart/x-mixed-replace;boundary=frame");
    if (res != ESP_OK) return res;

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "frame grab failed");
            res = ESP_FAIL;
            break;
        }

        /* GC2145 outputs YUV422 — encode to JPEG in software before sending */
        uint8_t *jpg_buf = NULL;
        size_t   jpg_len = 0;
        bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        esp_camera_fb_return(fb);   /* return framebuffer before the slow HTTP send */

        if (!ok) {
            ESP_LOGE(TAG, "JPEG encode failed");
            res = ESP_FAIL;
            break;
        }

        char hdr[80];
        int hdr_len = snprintf(hdr, sizeof(hdr),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
            jpg_len);

        res = httpd_resp_send_chunk(req, hdr, hdr_len);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (char *)jpg_buf, jpg_len);
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, "\r\n", 2);

        free(jpg_buf);

        if (res != ESP_OK) break;   /* client disconnected */
    }
    return res;
}

/* ── HTTP server task ───────────────────────────────────────────────────── */

static void stream_server_task(void *arg)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 81;
    cfg.ctrl_port        = 32768;   /* avoid collision with default 32767 */
    cfg.max_open_sockets = 2;       /* one active stream + one queued */

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        vTaskDelete(NULL);
        return;
    }

    httpd_uri_t stream_uri = {
        .uri     = "/stream",
        .method  = HTTP_GET,
        .handler = stream_handler,
    };
    httpd_register_uri_handler(server, &stream_uri);
    ESP_LOGI(TAG, "MJPEG stream ready at http://<ip>:81/stream");

    vTaskDelete(NULL);
}

/* ── UART SCAN receiver task ────────────────────────────────────────────── */

static void uart_scan_task(void *arg)
{
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
        UART_PIN_NO_CHANGE, PIN_UART_RX,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_cfg);
    gpio_set_level(PIN_LED, 0);

    uint8_t buf[UART_RX_BUF];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(100));
        if (len <= 0) continue;
        buf[len] = '\0';
        if (strstr((char *)buf, "SCAN")) {
            ESP_LOGI(TAG, "SCAN received from C6 — LED flash");
            gpio_set_level(PIN_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(PIN_LED, 0);
        }
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

    ESP_LOGI(TAG, "=== ESP32-S3 booting ===");

    wifi_init_sta();

    if (camera_core_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed — stream will not start");
    } else {
        xTaskCreate(stream_server_task, "stream_srv", 4096, NULL, 5, NULL);
    }

    xTaskCreate(uart_scan_task, "uart_scan", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "All tasks started — ready");
}
