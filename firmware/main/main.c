#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "lwip/sockets.h"
#include "esp_timer.h"

// ============================================================
// CONFIGURATION
// ============================================================

// WiFi — connect to the plato-arena AP run by the server
#define WIFI_SSID       "plato-arena"
#define WIFI_PASSWORD   "platopod123"

// Server address — the robot_bridge_node listens on this port
#define SERVER_IP       "192.168.4.1"
#define SERVER_PORT     9999

// Local UDP port for receiving commands from server
#define UDP_PORT        9999

#define LED_PIN         GPIO_NUM_8

// DRV8833 motor pins
#define MOTOR_L_FWD     GPIO_NUM_0   // IN1 - left motor forward
#define MOTOR_L_BWD     GPIO_NUM_1   // IN2 - left motor backward
#define MOTOR_R_FWD     GPIO_NUM_2   // IN3 - right motor forward
#define MOTOR_R_BWD     GPIO_NUM_3   // IN4 - right motor backward

// Robot parameters
#define WHEEL_BASE      0.075f       // distance between wheels (metres)
#define PWM_FREQ        1000         // motor PWM frequency
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT  // 0-255

// Watchdog: stop motors if no command received
#define WATCHDOG_MS     500

// Registration retry interval
#define REG_RETRY_MS    2000

static const char *TAG = "plato";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Robot identity (stored in NVS)
static uint16_t robot_tag_id = 0;
static uint16_t robot_radius_mm = 28;
static int robot_id = -1;  // assigned by server after registration

// ============================================================
// MOTOR CONTROL (PWM)
// ============================================================

static void motor_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channels[] = {
        { .gpio_num = MOTOR_L_FWD, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0 },
        { .gpio_num = MOTOR_L_BWD, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0 },
        { .gpio_num = MOTOR_R_FWD, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0, .duty = 0 },
        { .gpio_num = MOTOR_R_BWD, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_0, .duty = 0 },
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]);
    }
    ESP_LOGI(TAG, "Motors initialized (all stopped)");
}

static void set_motor_pwm(int channel_fwd, int channel_bwd, float speed) {
    int duty = (int)(fabsf(speed) * 255);
    if (duty > 255) duty = 255;

    if (speed >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_fwd, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_fwd);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_bwd, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_bwd);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_fwd, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_fwd);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_bwd, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_bwd);
    }
}

static void drive(float linear, float angular) {
    float left  = linear - angular * WHEEL_BASE / 2.0f;
    float right = linear + angular * WHEEL_BASE / 2.0f;

    float max_abs = fmaxf(fabsf(left), fabsf(right));
    if (max_abs > 1.0f) {
        left /= max_abs;
        right /= max_abs;
    }

    set_motor_pwm(LEDC_CHANNEL_0, LEDC_CHANNEL_1, left);
    set_motor_pwm(LEDC_CHANNEL_2, LEDC_CHANNEL_3, right);

    ESP_LOGI(TAG, "Drive: linear=%.2f angular=%.2f -> L=%.2f R=%.2f", linear, angular, left, right);
}

static void stop_motors(void) {
    for (int ch = 0; ch < 4; ch++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }
}

// ============================================================
// LED (active low on ESP32-C3 SuperMini)
// ============================================================

static void led_init(void) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1); // off
}

// ============================================================
// NVS — Robot identity storage
// ============================================================

static void load_robot_identity(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("robot", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_get_u16(handle, "tag_id", &robot_tag_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot — set defaults
        robot_tag_id = 1;
        nvs_set_u16(handle, "tag_id", robot_tag_id);
        ESP_LOGW(TAG, "No tag_id in NVS, defaulting to %d", robot_tag_id);
    }

    err = nvs_get_u16(handle, "radius_mm", &robot_radius_mm);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        robot_radius_mm = 28;
        nvs_set_u16(handle, "radius_mm", robot_radius_mm);
        ESP_LOGW(TAG, "No radius_mm in NVS, defaulting to %d", robot_radius_mm);
    }

    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Robot identity: tag_id=%d radius_mm=%d", robot_tag_id, robot_radius_mm);
}

// ============================================================
// WIFI
// ============================================================

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "Disconnected reason=%d", e->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ============================================================
// REGISTRATION — send REG to server, wait for OK <robot_id>
// ============================================================

static int register_with_server(int sock) {
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    char msg[64];
    snprintf(msg, sizeof(msg), "REG %d %d", robot_tag_id, robot_radius_mm);

    char rx[128];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    while (1) {
        ESP_LOGI(TAG, "Sending registration: %s", msg);
        sendto(sock, msg, strlen(msg), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));

        // Wait for response with timeout
        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        int n = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&src, &slen);
        if (n > 0) {
            rx[n] = 0;
            ESP_LOGI(TAG, "Registration response: %s", rx);

            if (strncmp(rx, "OK ", 3) == 0) {
                int assigned_id = atoi(rx + 3);
                ESP_LOGI(TAG, "Registered! robot_id=%d", assigned_id);
                return assigned_id;
            } else if (strcmp(rx, "DEFERRED") == 0) {
                ESP_LOGW(TAG, "Tag not visible to camera, retrying in %d ms...", REG_RETRY_MS);
            } else if (strncmp(rx, "ERR", 3) == 0) {
                ESP_LOGE(TAG, "Registration error: %s", rx);
            }
        } else {
            ESP_LOGW(TAG, "No registration response, retrying...");
        }

        vTaskDelay(pdMS_TO_TICKS(REG_RETRY_MS));
    }
}

// ============================================================
// UDP COMMAND SERVER
// ============================================================
// Protocol:
//   "M <linear> <angular>"  - move (e.g. "M 0.10 0.50")
//   "S"                     - stop motors
//   "L1"                    - LED on
//   "L0"                    - LED off
//   "H"                     - heartbeat (returns "OK")
//   "P"                     - ping (returns "PONG")

static volatile int64_t last_cmd_time = 0;

static void udp_server_task(void *arg) {
    char rx[128];
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket failed");
        vTaskDelete(NULL);
        return;
    }
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    ESP_LOGI(TAG, "UDP server on port %d", UDP_PORT);

    // Register with server
    robot_id = register_with_server(sock);

    // Blink 3 times = registered and ready
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Set short timeout for command reception
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&src, &slen);

        if (n > 0) {
            rx[n] = 0;
            last_cmd_time = esp_timer_get_time();

            if (rx[0] == 'M' && rx[1] == ' ') {
                float linear = 0, angular = 0;
                sscanf(rx + 2, "%f %f", &linear, &angular);
                if (linear > 0.3f) linear = 0.3f;
                if (linear < -0.3f) linear = -0.3f;
                if (angular > 2.0f) angular = 2.0f;
                if (angular < -2.0f) angular = -2.0f;
                drive(linear, angular);
                sendto(sock, "OK", 2, 0, (struct sockaddr *)&src, slen);

            } else if (rx[0] == 'S') {
                stop_motors();
                ESP_LOGI(TAG, "STOP");
                sendto(sock, "OK", 2, 0, (struct sockaddr *)&src, slen);

            } else if (rx[0] == 'L' && rx[1] == '1') {
                gpio_set_level(LED_PIN, 0); // active low
                sendto(sock, "OK", 2, 0, (struct sockaddr *)&src, slen);

            } else if (rx[0] == 'L' && rx[1] == '0') {
                gpio_set_level(LED_PIN, 1); // active low
                sendto(sock, "OK", 2, 0, (struct sockaddr *)&src, slen);

            } else if (rx[0] == 'H') {
                sendto(sock, "OK", 2, 0, (struct sockaddr *)&src, slen);

            } else if (rx[0] == 'P') {
                sendto(sock, "PONG", 4, 0, (struct sockaddr *)&src, slen);

            } else {
                sendto(sock, "ERR", 3, 0, (struct sockaddr *)&src, slen);
            }
        }

        // Watchdog
        int64_t now = esp_timer_get_time();
        if (last_cmd_time > 0 && (now - last_cmd_time) > (WATCHDOG_MS * 1000)) {
            stop_motors();
            last_cmd_time = 0;
        }
    }
}

// ============================================================
// MAIN
// ============================================================

void app_main(void) {
    ESP_LOGI(TAG, "Plato Pod Robot starting...");

    // Hardware init — motors FIRST (safety)
    motor_init();
    stop_motors();
    led_init();

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Load robot identity from NVS
    load_robot_identity();

    // WiFi
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = false },
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Connecting to WiFi '%s'...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(60000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "WiFi connection FAILED — fast blink");
        while (1) {
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
