/*
 * RiVitaflow OS - Dialysis Machine Firmware
 * Main application file
 *
 * Hardware: L298N, 28BYJ-48, MAX30102, MAX98357A (I2S audio), LEDs
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
/* Use I2S to drive an external I2S DAC/amplifier such as the MAX98357A.
   This produces much better audio and offloads timing to hardware. */
#include "driver/i2s_std.h"
#include "esp_rom_sys.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include <dirent.h>
#include <sys/stat.h>

#include "rivitaflow.h"

/* ========== Constants ========== */
#define APP_ID                  0
#define DEVICE_NAME             "RiVitaflow"
#define TAG                     "RiVitaflow"

#define TELEMETRY_INTERVAL_MS   1000  // Send telemetry every 1 second

/* ========== 28BYJ-48 Stepper Sequence (Half-Step) ========== */
static const uint8_t stepper_sequence[8][4] = {
    {1, 0, 0, 0},  // Step 0
    {1, 1, 0, 0},  // Step 1
    {0, 1, 0, 0},  // Step 2
    {0, 1, 1, 0},  // Step 3
    {0, 0, 1, 0},  // Step 4
    {0, 0, 1, 1},  // Step 5
    {0, 0, 0, 1},  // Step 6
    {1, 0, 0, 1}   // Step 7
};

/* ========== Global Variables ========== */
machine_state_t g_machine_state = STATE_IDLE;
dialysis_config_t g_dialysis_config = {0};
telemetry_data_t g_telemetry = {0};
bool g_ble_connected = false;
uint16_t g_conn_id = 0;
esp_gatt_if_t g_gatts_if = 0;

static uint16_t g_service_handle = 0;
static uint16_t g_char_command_handle = 0;
static uint16_t g_char_telemetry_handle = 0;
static uint16_t g_char_config_handle = 0;

static uint32_t g_session_start_time = 0;
static TaskHandle_t g_startup_task_handle = NULL;
static TaskHandle_t g_telemetry_task_handle = NULL;
static TaskHandle_t g_spo2_task_handle = NULL;
static TaskHandle_t g_stepper_task_handle = NULL;

/* SPIFFS mount state */
static bool s_spiffs_mounted = false;

static bool g_stepper_running = false;
static uint8_t g_stepper_step_index = 0;

/* ========== BLE Advertisement Configuration ========== */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ========== GATT Service Definition ========== */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint16_t service_uuid = GATTS_SERVICE_UUID;
static const uint16_t char_uuid_command = CHAR_UUID_COMMAND;
static const uint16_t char_uuid_telemetry = CHAR_UUID_TELEMETRY;
static const uint16_t char_uuid_config = CHAR_UUID_CONFIG;

/* Full Database Description */
static const esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE] = {
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
           ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(service_uuid), (uint8_t *)&service_uuid}},

    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_uuid_command,
           ESP_GATT_PERM_WRITE, sizeof(command_packet_t), 0, NULL}},

    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_notify}},

    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_uuid_telemetry,
           ESP_GATT_PERM_READ, sizeof(telemetry_data_t), 0, NULL}},

    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), 0, NULL}},

    [6] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [7] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_uuid_config,
           ESP_GATT_PERM_WRITE, sizeof(dialysis_config_t), 0, NULL}},
};

/* ========== Function Declarations ========== */
static void audio_play_wav_sync(const char *filename);

/* ========== Hardware Initialization Functions ========== */

/* I2S-based audio output for MAX98357A (I2S DAC+amp). We initialize an I2S
   channel using the new I2S driver API (ESP-IDF v6.0+) in standard mode. */

static i2s_chan_handle_t s_i2s_tx_chan = NULL;

// Quick hardware sanity test: toggle I2S pins as GPIO to generate an audible
// click on the speaker/amp. This disables the I2S channel briefly, drives
// the pins manually, then re-enables the channel. Called once from audio_init
// to provide a simple hardware-level check.
// (click test removed) audio diagnostics will use direct I2S writes and
// logging only. For hardware-level checks use an external scope or
// temporarily reintroduce a GPIO pulse routine.

void audio_init(void) {
    // Step 1: Create I2S channel (TX only for playback)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto-clear TX DMA buffer on underflow
    
    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_i2s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_init: i2s_new_channel failed: %s", esp_err_to_name(ret));
        return;
    }

    // Step 2: Configure I2S in standard mode (Philips I2S format for MAX98357A)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_LRCK_PIN,
            .dout = I2S_DATA_OUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(s_i2s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_init: i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret));
        return;
    }

    // Step 3: Enable the I2S channel
    ret = i2s_channel_enable(s_i2s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_init: i2s_channel_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "audio_init: i2s_channel_enable OK (chan=%p)", (void*)s_i2s_tx_chan);
    ESP_LOGI(TAG, "Audio (I2S -> MAX98357A) initialized on BCK=%d, LRCK=%d, DATA=%d",
             I2S_BCK_PIN, I2S_LRCK_PIN, I2S_DATA_OUT_PIN);

    /* Diagnostic: report GPIO levels for configured I2S pins (may help
       determine if the pins are being driven). Values can be read even if
       peripheral controls them. */
    int bck_lvl = gpio_get_level(I2S_BCK_PIN);
    int lrck_lvl = gpio_get_level(I2S_LRCK_PIN);
    int dout_lvl = gpio_get_level(I2S_DATA_OUT_PIN);
    ESP_LOGI(TAG, "audio_init: I2S GPIO levels BCK=%d LRCK=%d DOUT=%d", bck_lvl, lrck_lvl, dout_lvl);

    /* Ensure LED pin is configured so we can blink it during I2S writes */
    gpio_set_direction(LED_STARTUP_RED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_STARTUP_RED, 0);

     /* Click test removed; continuing with standard I2S operation. */
}

// Sine-wave tone generator using I2S: generate 16-bit signed stereo samples
// and write them to the I2S DMA buffer. This produces a much cleaner tone
// than a square wave and uses the I2S peripheral for timing.
void audio_tone(uint16_t frequency, uint32_t duration_ms) {
    if (frequency == 0 || duration_ms == 0) return;
    if (!s_i2s_tx_chan) {
        ESP_LOGW(TAG, "audio_tone: I2S channel not initialized");
        return;
    }

    const uint32_t sample_rate = AUDIO_SAMPLE_RATE;
    const size_t frames_per_buf = 512; // frames (each frame is one sample per channel)

    // total number of frames to generate
    size_t total_frames = (size_t)(((uint64_t)duration_ms * sample_rate) / 1000ULL);

    ESP_LOGI(TAG, "audio_tone: start freq=%u duration_ms=%u sample_rate=%u total_frames=%u",
             (unsigned)frequency, (unsigned)duration_ms, (unsigned)sample_rate, (unsigned)total_frames);

    // allocate buffer for stereo interleaved samples (int16_t)
    // each frame -> 2 samples (L,R)
    int16_t *buf = heap_caps_malloc(sizeof(int16_t) * frames_per_buf * 2, MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "audio_tone: failed to allocate buffer");
        return;
    }

    // amplitude (avoid full-scale clipping; leave headroom)
    const float amplitude = 30000.0f;

    // phase increment per frame
    const float two_pi = 2.0f * M_PI;
    const float phase_inc = two_pi * ((float)frequency / (float)sample_rate);
    float phase = 0.0f;

    while (total_frames > 0) {
        size_t frames = frames_per_buf;
        if (frames > total_frames) frames = total_frames;

        // fill buffer with sine samples (stereo: duplicate to both channels)
        for (size_t i = 0; i < frames; ++i) {
            float s = sinf(phase);
            int16_t samp = (int16_t)(s * amplitude);
            buf[2*i] = samp;     // left
            buf[2*i + 1] = samp; // right
            phase += phase_inc;
            if (phase >= two_pi) phase -= two_pi;
        }

    size_t bytes = frames * 2 * sizeof(int16_t);
    size_t written = 0;
    ESP_LOGI(TAG, "audio_tone: about to write %u bytes to I2S (frames=%u)", (unsigned)bytes, (unsigned)frames);
    /* Blink LED to indicate write start */
    gpio_set_level(LED_STARTUP_RED, 1);
    esp_err_t ret = i2s_channel_write(s_i2s_tx_chan, buf, bytes, &written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "audio_tone: i2s_channel_write failed: %s (expected=%u written=%u)",
                    esp_err_to_name(ret), (unsigned)bytes, (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
            break;
        } else if (written != bytes) {
            ESP_LOGW(TAG, "audio_tone: partial write: expected=%u written=%u",
                    (unsigned)bytes, (unsigned)written);
            /* continue; maybe buffer got filled partially */
            gpio_set_level(LED_STARTUP_RED, 0);
        } else {
            ESP_LOGI(TAG, "audio_tone: wrote %u bytes to I2S", (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
        }

        total_frames -= frames;
    }

    heap_caps_free(buf);
}

void audio_beep(uint32_t duration_ms) {
    audio_tone(BEEP_FREQUENCY, duration_ms);
}

void audio_long_beep(void) {
    audio_beep(3000);
}

// audio_test_task removed — keep startup.wav autorun for diagnostics.

void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_STARTUP_RED) | (1ULL << LED_OPERATION_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(LED_STARTUP_RED, 0);
    gpio_set_level(LED_OPERATION_GREEN, 0);

    ESP_LOGI(TAG, "LEDs initialized");
}

void led_startup_on(void) { gpio_set_level(LED_STARTUP_RED, 1); }
void led_startup_off(void) { gpio_set_level(LED_STARTUP_RED, 0); }
void led_operation_on(void) { gpio_set_level(LED_OPERATION_GREEN, 1); }
void led_operation_off(void) { gpio_set_level(LED_OPERATION_GREEN, 0); }

// LED status task: drive LED_STARTUP_RED (GPIO2) to indicate system/BLE state.
// Patterns:
//  - BLE disconnected / idle: slow blink (1s period)
//  - BLE connected (CONNECTED/READY): solid ON
//  - STARTING: fast blink (200ms)
//  - RUNNING: LED off (operation green indicates running)
//  - STOPPED: immediate 3 quick flashes
static void led_status_task(void *arg)
{
    machine_state_t prev_state = (machine_state_t)0xFF;

    while (1) {
        machine_state_t state = g_machine_state;
        bool ble = g_ble_connected;

        // If we just entered STOPPED, flash 3 times as an attention cue
        if (state == STATE_STOPPED && prev_state != STATE_STOPPED) {
            for (int i = 0; i < 3; ++i) {
                gpio_set_level(LED_STARTUP_RED, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_STARTUP_RED, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        if (!ble) {
            // BLE not connected -> slow blink
            gpio_set_level(LED_STARTUP_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_STARTUP_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // BLE connected - indicate based on machine state
            switch (state) {
                case STATE_CONNECTED:
                case STATE_READY:
                    // Solid on when connected/ready
                    gpio_set_level(LED_STARTUP_RED, 1);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;

                case STATE_STARTING:
                    // Fast blink during startup windup
                    gpio_set_level(LED_STARTUP_RED, 1);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    gpio_set_level(LED_STARTUP_RED, 0);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    break;

                case STATE_RUNNING:
                    // Turn off the startup LED during normal operation
                    gpio_set_level(LED_STARTUP_RED, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;

                case STATE_PAUSED:
                    // Medium blink while paused
                    gpio_set_level(LED_STARTUP_RED, 1);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    gpio_set_level(LED_STARTUP_RED, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    break;

                default:
                    gpio_set_level(LED_STARTUP_RED, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
            }
        }

        prev_state = state;
    }
}

void pumps_init(void) {
    // Configure GPIO pins for L298N control
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP1_IN1_PIN) | (1ULL << PUMP1_IN2_PIN) |
                       (1ULL << PUMP2_IN3_PIN) | (1ULL << PUMP2_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize all direction pins to low
    gpio_set_level(PUMP1_IN1_PIN, 0);
    gpio_set_level(PUMP1_IN2_PIN, 0);
    gpio_set_level(PUMP2_IN3_PIN, 0);
    gpio_set_level(PUMP2_IN4_PIN, 0);

    // Configure PWM timer for pumps
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_PUMPS,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure Pump 1 PWM (ENA)
    ledc_channel_config_t ledc_channel_pump1 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_PUMP1,
        .timer_sel = LEDC_TIMER_PUMPS,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PUMP1_ENA_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_pump1);

    // Configure Pump 2 PWM (ENB)
    ledc_channel_config_t ledc_channel_pump2 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_PUMP2,
        .timer_sel = LEDC_TIMER_PUMPS,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PUMP2_ENB_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_pump2);

    ESP_LOGI(TAG, "L298N pumps initialized");
}

uint8_t flow_rate_to_pwm(uint16_t flow_ml_per_min) {
    // Convert flow rate to PWM duty cycle (0-255)
    // Assume max flow rate is 500 ml/min = 255 PWM
    if (flow_ml_per_min > 500) flow_ml_per_min = 500;
    return (uint8_t)((flow_ml_per_min * 255) / 500);
}

void pump1_start(uint16_t speed) {
    // Set direction (forward)
    gpio_set_level(PUMP1_IN1_PIN, 1);
    gpio_set_level(PUMP1_IN2_PIN, 0);

    // Set PWM speed
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_PUMP1, speed);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_PUMP1);

    ESP_LOGI(TAG, "Pump 1 (Blood) started at PWM duty=%d", speed);
}

void pump1_stop(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_PUMP1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_PUMP1);
    gpio_set_level(PUMP1_IN1_PIN, 0);
    gpio_set_level(PUMP1_IN2_PIN, 0);
    ESP_LOGI(TAG, "Pump 1 (Blood) stopped");
}

void pump2_start(uint16_t speed) {
    // Set direction (forward)
    gpio_set_level(PUMP2_IN3_PIN, 1);
    gpio_set_level(PUMP2_IN4_PIN, 0);

    // Set PWM speed
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_PUMP2, speed);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_PUMP2);

    ESP_LOGI(TAG, "Pump 2 (Waste) started at PWM duty=%d", speed);
}

void pump2_stop(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_PUMP2, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_PUMP2);
    gpio_set_level(PUMP2_IN3_PIN, 0);
    gpio_set_level(PUMP2_IN4_PIN, 0);
    ESP_LOGI(TAG, "Pump 2 (Waste) stopped");
}

void pumps_set_flow_rates(uint16_t blood_flow, uint16_t waste_flow) {
    uint8_t blood_pwm = flow_rate_to_pwm(blood_flow);
    uint8_t waste_pwm = flow_rate_to_pwm(waste_flow);

    pump1_start(blood_pwm);
    pump2_start(waste_pwm);
}

void stepper_init(void) {
    // Configure stepper motor pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEPPER_IN1_PIN) | (1ULL << STEPPER_IN2_PIN) |
                       (1ULL << STEPPER_IN3_PIN) | (1ULL << STEPPER_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize all pins to low
    gpio_set_level(STEPPER_IN1_PIN, 0);
    gpio_set_level(STEPPER_IN2_PIN, 0);
    gpio_set_level(STEPPER_IN3_PIN, 0);
    gpio_set_level(STEPPER_IN4_PIN, 0);

    ESP_LOGI(TAG, "28BYJ-48 stepper initialized");
}

void stepper_set_step(uint8_t step_index) {
    gpio_set_level(STEPPER_IN1_PIN, stepper_sequence[step_index][0]);
    gpio_set_level(STEPPER_IN2_PIN, stepper_sequence[step_index][1]);
    gpio_set_level(STEPPER_IN3_PIN, stepper_sequence[step_index][2]);
    gpio_set_level(STEPPER_IN4_PIN, stepper_sequence[step_index][3]);
}

void stepper_task(void *param) {
    ESP_LOGI(TAG, "Stepper motor task started");
    uint32_t step_counter = 0;

    while (1) {
        if (g_stepper_running) {
            stepper_set_step(g_stepper_step_index);
            g_stepper_step_index = (g_stepper_step_index + 1) % STEPPER_SEQUENCE_LENGTH;
            vTaskDelay(pdMS_TO_TICKS(STEPPER_STEP_DELAY_MS));

            // Periodically yield to ensure other tasks (and the idle task)
            // get CPU time so the Task Watchdog isn't starved. Use a short
            // cooperative delay which is safer than a raw taskYIELD.
            step_counter++;
            if (step_counter >= 10) {
                step_counter = 0;
                // Let the scheduler run other ready tasks and the idle task.
                vTaskDelay(0);
            }
        } else {
            // Motor stopped, turn off all coils
            gpio_set_level(STEPPER_IN1_PIN, 0);
            gpio_set_level(STEPPER_IN2_PIN, 0);
            gpio_set_level(STEPPER_IN3_PIN, 0);
            gpio_set_level(STEPPER_IN4_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void stepper_start(void) {
    g_stepper_running = true;
    ESP_LOGI(TAG, "Stepper motor started");
}

void stepper_stop(void) {
    g_stepper_running = false;
    ESP_LOGI(TAG, "Stepper motor stopped");
}

void spo2_init(void) {
    // Initialize I2C master bus for MAX30102
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "MAX30102 sensor I2C initialized");

    // TODO: Add device configuration when implementing actual sensor reading
    // i2c_device_config_t dev_cfg = {
    //     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    //     .device_address = MAX30102_I2C_ADDR,
    //     .scl_speed_hz = I2C_FREQ_HZ,
    // };
    // i2c_master_dev_handle_t dev_handle;
    // i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
}

void hardware_init(void) {
    audio_init();
    led_init();
    pumps_init();
    stepper_init();
    spo2_init();
    ESP_LOGI(TAG, "Hardware initialization complete");
}

// Initialize SPIFFS filesystem on internal flash partition labeled "storage"
// Mounts at /spiffs and sets s_spiffs_mounted on success. Non-fatal if partition
// not present; logs info and continues.
void audio_fs_init(void)
{
    const char *base_path = "/spiffs";
    const char *partition_label = "storage"; // partition must exist in partition table

    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = partition_label,
        .max_files = 5,
        // During development/provisioning it's convenient to auto-format the
        // SPIFFS partition if it's missing or corrupted so the device can
        // create the filesystem and the host-provided image in `main/CMakeLists.txt`
        // isn't strictly required. Set to `true` for dev; flip to `false` for
        // production if you want to avoid accidental formatting.
        .format_if_mount_failed = true
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err == ESP_OK) {
        s_spiffs_mounted = true;
        ESP_LOGI(TAG, "SPIFFS mounted at %s (partition: %s)", base_path, partition_label);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "SPIFFS mount: partition '%s' not found. Skipping audio file playback from flash.", partition_label);
    } else if (err == ESP_FAIL) {
        ESP_LOGW(TAG, "SPIFFS mount failed: filesystem mount failed");
    } else {
        ESP_LOGW(TAG, "SPIFFS mount error: %s", esp_err_to_name(err));
    }
}

// Minimal WAV parser + player that streams PCM samples to I2S (MAX98357A).
// This is intentionally simple: supports PCM format (1), 8-bit unsigned or 16-bit signed LE,
// and mono/stereo (stereo averaged). Uses hardware I2S for timing and DMA.
static void audio_play_wav_task(void *arg)
{
    const char *filename = (const char *)arg;
    ESP_LOGI(TAG, "audio_play_wav_task: started for %s", filename);
    if (!s_i2s_tx_chan) {
        ESP_LOGW(TAG, "audio_play_wav: I2S channel not initialized, cannot play %s", filename);
        vPortFree((void*)filename);
        vTaskDelete(NULL);
        return;
    }

    // Use the sync helper to play the file using minimal temporary allocations.
    // The helper does the actual streaming in-place.
    audio_play_wav_sync(filename);

    vPortFree((void*)filename);
    vTaskDelete(NULL);
}

// Synchronous WAV player helper: plays the given filename to I2S in the
// caller context. This avoids creating small allocations per-chunk and lets
// callers stream files sequentially with low RAM usage.
static void audio_play_wav_sync(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (!f) {
        ESP_LOGW(TAG, "audio_play_wav_sync: failed to open %s", filename);
        return;
    }

    // Read minimal RIFF header
    uint8_t hdr[44];
    if (fread(hdr, 1, 44, f) != 44) {
        ESP_LOGW(TAG, "audio_play_wav_sync: %s too small or read error", filename);
        fclose(f);
        return;
    }

    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        ESP_LOGW(TAG, "audio_play_wav_sync: %s is not a valid WAV file", filename);
        fclose(f);
        return;
    }

    // parse fmt chunk (assume common layout)
    uint16_t audio_format = *(uint16_t *)(hdr + 20);
    uint16_t num_channels = *(uint16_t *)(hdr + 22);
    uint32_t sample_rate = *(uint32_t *)(hdr + 24);
    uint16_t bits_per_sample = *(uint16_t *)(hdr + 34);

    if (audio_format != 1) {
        ESP_LOGW(TAG, "audio_play_wav_sync: unsupported WAV compression (format=%u)", audio_format);
        fclose(f);
        return;
    }

    ESP_LOGI(TAG, "Playing WAV %s: %u Hz, %u bits, %u channels", filename, sample_rate, bits_per_sample, num_channels);
    if (sample_rate != AUDIO_SAMPLE_RATE) {
        ESP_LOGW(TAG, "audio_play_wav_sync: WAV sample rate (%u) != configured AUDIO_SAMPLE_RATE (%u). Playback speed/pitch may be wrong.", sample_rate, AUDIO_SAMPLE_RATE);
    }

    // Find data chunk offset (simple scan)
    fseek(f, 12, SEEK_SET);
    uint8_t chunk_hdr[8];
    uint32_t data_size = 0;
    long data_start = -1;
    while (fread(chunk_hdr, 1, 8, f) == 8) {
        uint32_t chunk_size = *(uint32_t *)(chunk_hdr + 4);
        if (memcmp(chunk_hdr, "data", 4) == 0) {
            data_size = chunk_size;
            data_start = ftell(f);
            break;
        }
        // skip this chunk
        fseek(f, chunk_size, SEEK_CUR);
    }

    if (data_start < 0) {
        ESP_LOGW(TAG, "audio_play_wav_sync: data chunk not found in %s", filename);
        fclose(f);
        return;
    }

    const int buf_samples = 512; // frames per read
    size_t sample_bytes = bits_per_sample / 8;
    size_t in_buf_size = buf_samples * num_channels * sample_bytes;
    uint8_t *inbuf = malloc(in_buf_size);
    if (!inbuf) {
        ESP_LOGE(TAG, "audio_play_wav_sync: out of memory (inbuf)");
        fclose(f);
        return;
    }

    // outbuf holds stereo 16-bit interleaved samples for buf_samples frames
    int16_t *outbuf = malloc(sizeof(int16_t) * buf_samples * 2);
    if (!outbuf) {
        ESP_LOGE(TAG, "audio_play_wav_sync: out of memory (outbuf)");
        free(inbuf);
        fclose(f);
        return;
    }

    uint32_t samples_remaining = data_size / (sample_bytes * num_channels);
    ESP_LOGI(TAG, "audio_play_wav_sync: data_size=%u sample_bytes=%u num_channels=%u samples_remaining=%u", (unsigned)data_size, (unsigned)sample_bytes, (unsigned)num_channels, (unsigned)samples_remaining);

    while (samples_remaining > 0) {
        uint32_t to_read = buf_samples;
        if (to_read > samples_remaining) to_read = samples_remaining;
        size_t bytes_to_read = to_read * num_channels * sample_bytes;
        size_t actually = fread(inbuf, 1, bytes_to_read, f);
        if (actually == 0) break;

        // convert frames to 16-bit stereo interleaved samples in outbuf
        uint8_t *p = inbuf;
        for (uint32_t s = 0; s < to_read; ++s) {
            int32_t accum = 0;
            for (uint16_t ch = 0; ch < num_channels; ++ch) {
                if (sample_bytes == 1) {
                    uint8_t v = *p++;
                    accum += (int32_t)v - 128; // center 8-bit
                } else if (sample_bytes == 2) {
                    int16_t v = (int16_t)(p[0] | (p[1] << 8));
                    p += 2;
                    accum += (int32_t)v;
                } else {
                    p += sample_bytes;
                }
            }

            int16_t sample_val;
            if (num_channels > 1) sample_val = (int16_t)(accum / num_channels);
            else sample_val = (int16_t)accum;
            if (sample_bytes == 1) sample_val = (int16_t)(sample_val << 8);

            outbuf[2*s] = sample_val;
            outbuf[2*s + 1] = sample_val;
        }

        size_t bytes = to_read * 2 * sizeof(int16_t);
        size_t written = 0;
        ESP_LOGI(TAG, "audio_play_wav_sync: about to write %u bytes to I2S (to_read=%u)", (unsigned)bytes, (unsigned)to_read);
        gpio_set_level(LED_STARTUP_RED, 1);
        esp_err_t wret = i2s_channel_write(s_i2s_tx_chan, outbuf, bytes, &written, portMAX_DELAY);
        if (wret != ESP_OK) {
            ESP_LOGW(TAG, "audio_play_wav_sync: i2s_channel_write error: %s (expected=%u written=%u)", esp_err_to_name(wret), (unsigned)bytes, (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
            break;
        } else if (written != bytes) {
            ESP_LOGW(TAG, "audio_play_wav_sync: partial write: expected=%u written=%u", (unsigned)bytes, (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
        } else {
            ESP_LOGI(TAG, "audio_play_wav_sync: wrote %u bytes to I2S", (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
        }

        samples_remaining -= to_read;
    }

    free(inbuf);
    free(outbuf);
    fclose(f);
}

// Public helper: play a WAV file from mounted SPIFFS asynchronously.
// filename should be an absolute path like "/spiffs/startup.wav".
void audio_play_wav_async(const char *path)
{
    if (!s_spiffs_mounted) {
        ESP_LOGW(TAG, "audio_play_wav_async: SPIFFS not mounted, cannot play %s", path);
        return;
    }

    // copy path to heap for task arg
    char *arg = pvPortMalloc(strlen(path) + 1);
    if (!arg) {
        ESP_LOGE(TAG, "audio_play_wav_async: malloc failed");
        return;
    }
    strcpy(arg, path);

    if (xTaskCreate(audio_play_wav_task, "audio_play_wav", 4096, arg, 6, NULL) != pdPASS) {
        ESP_LOGW(TAG, "audio_play_wav_async: failed to create task");
        vPortFree(arg);
    }
}

// Play raw unsigned 8-bit PCM file (u8) from SPIFFS asynchronously.
// The function spawns a task which converts bytes to 16-bit stereo and writes
// them to the configured I2S peripheral at the given sample_rate (Hz).
typedef struct {
    char *path;
    uint32_t sample_rate;
} audio_u8_arg_t;

static void audio_play_u8_task(void *arg)
{
    audio_u8_arg_t *a = (audio_u8_arg_t *)arg;
    const char *filename = a->path;
    uint32_t sample_rate = a->sample_rate ? a->sample_rate : 8000;
    ESP_LOGI(TAG, "audio_play_u8_task: started for %s @ %u Hz", filename, sample_rate);

    FILE *f = fopen(filename, "rb");
    if (!f) {
        ESP_LOGW(TAG, "audio_play_u8: failed to open %s", filename);
        vPortFree(a->path);
        vPortFree(a);
        vTaskDelete(NULL);
        return;
    }

    if (!s_i2s_tx_chan) {
        ESP_LOGW(TAG, "audio_play_u8: I2S channel not initialized, cannot play %s", filename);
        vPortFree(a->path);
        vPortFree(a);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "audio_play_u8: playing %s @ %" PRIu32 " Hz", filename, sample_rate);

    const size_t buf_size = 512;
    uint8_t *buf = malloc(buf_size);
    if (!buf) {
        ESP_LOGE(TAG, "audio_play_u8: out of memory");
        fclose(f);
        vPortFree(a->path);
        vPortFree(a);
        vTaskDelete(NULL);
        return;
    }

    // Allocate output buffer once to avoid repeated malloc/free
    int16_t *outbuf = malloc(sizeof(int16_t) * buf_size * 2);
    if (!outbuf) {
        ESP_LOGE(TAG, "audio_play_u8: out of memory (outbuf)");
        free(buf);
        fclose(f);
        vPortFree(a->path);
        vPortFree(a);
        vTaskDelete(NULL);
        return;
    }

    // Note: For different sample rates, you may want to reconfigure the I2S channel.
    // For simplicity, we use the configured AUDIO_SAMPLE_RATE here.
    // Dynamic sample rate changes require disabling/reconfiguring/re-enabling the channel.

    while (1) {
        size_t r = fread(buf, 1, buf_size, f);
        if (r == 0) break;

        // convert to 16-bit stereo interleaved in preallocated outbuf
        for (size_t i = 0; i < r; ++i) {
            int16_t v = ((int)buf[i] - 128) << 8;
            outbuf[2*i] = v;
            outbuf[2*i + 1] = v;
        }

        size_t bytes = r * 2 * sizeof(int16_t);
        size_t written = 0;
        ESP_LOGI(TAG, "audio_play_u8: about to write %u bytes to I2S (r=%u)", (unsigned)bytes, (unsigned)r);
        /* Blink LED to indicate write start */
        gpio_set_level(LED_STARTUP_RED, 1);
        esp_err_t wret = i2s_channel_write(s_i2s_tx_chan, outbuf, bytes, &written, portMAX_DELAY);
        if (wret != ESP_OK) {
            ESP_LOGW(TAG, "audio_play_u8: i2s_channel_write error: %s (expected=%u written=%u)", esp_err_to_name(wret), (unsigned)bytes, (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
            break;
        } else if (written != bytes) {
            ESP_LOGW(TAG, "audio_play_u8: partial write: expected=%u written=%u", (unsigned)bytes, (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
        } else {
            ESP_LOGI(TAG, "audio_play_u8: wrote %u bytes to I2S", (unsigned)written);
            gpio_set_level(LED_STARTUP_RED, 0);
        }
    }

    free(outbuf);
    free(buf);
    fclose(f);
    vPortFree(a->path);
    vPortFree(a);
    vTaskDelete(NULL);
}

void audio_play_u8_async(const char *path, uint32_t sample_rate)
{
    if (!s_spiffs_mounted) {
        ESP_LOGW(TAG, "audio_play_u8_async: SPIFFS not mounted, cannot play %s", path);
        return;
    }

    audio_u8_arg_t *a = pvPortMalloc(sizeof(*a));
    if (!a) {
        ESP_LOGE(TAG, "audio_play_u8_async: alloc failed");
        return;
    }
    a->path = pvPortMalloc(strlen(path) + 1);
    if (!a->path) {
        vPortFree(a);
        ESP_LOGE(TAG, "audio_play_u8_async: path alloc failed");
        return;
    }
    strcpy(a->path, path);
    a->sample_rate = sample_rate;

    if (xTaskCreate(audio_play_u8_task, "audio_play_u8", 4096, a, 6, NULL) != pdPASS) {
        ESP_LOGW(TAG, "audio_play_u8_async: failed to create task");
        vPortFree(a->path);
        vPortFree(a);
    }
}

// Play all WAV files found under /spiffs sequentially (blocks until done).
// This uses the low-RAM sync helper so memory usage is bounded.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
static void audio_play_all_spiffs_task(void *arg)
{
    const char *base = "/spiffs";
    DIR *d = opendir(base);
    if (!d) {
        ESP_LOGW(TAG, "audio_play_all_spiffs: opendir failed for %s", base);
        vTaskDelete(NULL);
        return;
    }

    struct dirent *entry;
    char path[512];  // Increased buffer size to avoid truncation issues
    while ((entry = readdir(d)) != NULL) {
        if (entry->d_type != DT_REG && entry->d_type != DT_UNKNOWN) continue;
        const char *name = entry->d_name;
        size_t len = strlen(name);
        if (len > 4) {
            const char *ext = name + len - 4;
            if (strcasecmp(ext, ".wav") == 0) {
                snprintf(path, sizeof(path), "%s/%s", base, name);
                ESP_LOGI(TAG, "audio_play_all_spiffs: playing %s", path);
                audio_play_wav_sync(path);
                // small gap between files
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }

    closedir(d);
    vTaskDelete(NULL);
}
#pragma GCC diagnostic pop

void audio_play_all_spiffs_async(void)
{
    if (xTaskCreate(audio_play_all_spiffs_task, "audio_play_all", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGW(TAG, "audio_play_all_spiffs_async: failed to create task");
    }
}

/* ========== State Machine Functions ========== */
void set_machine_state(machine_state_t new_state) {
    ESP_LOGI(TAG, "State change: %d -> %d", g_machine_state, new_state);
    g_machine_state = new_state;
    g_telemetry.state = new_state;
}

/* ========== Startup Sequence Task ========== */
void startup_sequence_task(void *param) {
    ESP_LOGI(TAG, "=== Starting Startup Sequence ===");

    // Blink LED during startup
    led_startup_on();

    // Step 1: Single beep on boot
    ESP_LOGI(TAG, "Step 1: Boot beep");
    if (s_spiffs_mounted) {
        audio_play_wav_async("/spiffs/startup.wav");
        // let the async task begin; if file missing the task will log and exit
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        audio_beep(200);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 2: Wait for Bluetooth connection
    ESP_LOGI(TAG, "Step 2: Waiting for Bluetooth connection...");
    set_machine_state(STATE_IDLE);

    // Blink LED while waiting for BLE connection
    while (!g_ble_connected) {
        led_startup_on();
        vTaskDelay(pdMS_TO_TICKS(500));
        led_startup_off();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Bluetooth connected!");
    led_startup_on();
    set_machine_state(STATE_CONNECTED);
    audio_beep(100);
    vTaskDelay(pdMS_TO_TICKS(100));
    audio_beep(100);

    // Step 3: Wait for dialysis config
    ESP_LOGI(TAG, "Step 3: Waiting for dialysis configuration...");
    while (g_dialysis_config.session_duration == 0 && g_ble_connected) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!g_ble_connected) {
        ESP_LOGI(TAG, "Bluetooth disconnected during config wait");
        set_machine_state(STATE_IDLE);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Configuration received!");
    set_machine_state(STATE_READY);
    audio_beep(150);
    vTaskDelay(pdMS_TO_TICKS(200));
    audio_beep(150);
    vTaskDelay(pdMS_TO_TICKS(200));
    audio_beep(150);

    // Step 4: Wait for START command
    ESP_LOGI(TAG, "Step 4: Ready to start. Waiting for START command...");
    while (g_machine_state == STATE_READY && g_ble_connected) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (g_machine_state != STATE_STARTING) {
        ESP_LOGI(TAG, "Startup sequence interrupted");
        vTaskDelete(NULL);
        return;
    }

    // Step 5: Initial windup - 3 second beep
    ESP_LOGI(TAG, "Step 5: Initial windup - 3 second beep");
    led_startup_on();
    audio_long_beep();

    // Step 6: Start SPO2 sensor
    ESP_LOGI(TAG, "Step 6: Starting SPO2 sensor readings");
    if (g_spo2_task_handle == NULL) {
        xTaskCreate(spo2_read_task, "spo2_task", 4096, NULL, 5, &g_spo2_task_handle);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 7: Start motors and pumps
    ESP_LOGI(TAG, "Step 7: Starting motors and pumps");

    // Start stepper motor
    stepper_start();
    vTaskDelay(pdMS_TO_TICKS(2000));  // Run for 2 seconds

    // Start blood pump
    uint8_t blood_pwm = flow_rate_to_pwm(g_dialysis_config.blood_flow_rate);
    pump1_start(blood_pwm);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second

    // Start waste pump
    uint8_t waste_pwm = flow_rate_to_pwm(g_dialysis_config.waste_flow_rate);
    pump2_start(waste_pwm);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 8: Operation started
    ESP_LOGI(TAG, "Step 8: Operation started!");
    led_startup_off();
    led_operation_on();
    set_machine_state(STATE_RUNNING);

    audio_beep(100);
    vTaskDelay(pdMS_TO_TICKS(150));
    audio_beep(100);
    vTaskDelay(pdMS_TO_TICKS(150));
    audio_beep(100);

    g_session_start_time = esp_timer_get_time() / 1000000;

    ESP_LOGI(TAG, "=== Startup Sequence Complete ===");

    g_startup_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ========== SPO2 Sensor Reading Task ========== */
void spo2_read_task(void *param) {
    ESP_LOGI(TAG, "SPO2 reading task started");

    while (1) {
        if (g_machine_state == STATE_RUNNING || g_machine_state == STATE_STARTING) {
            // TODO: Read actual MAX30102 sensor data via I2C
            // For now, simulate realistic values
            g_telemetry.spo2 = 95 + (esp_random() % 5);
            g_telemetry.bpm = 70 + (esp_random() % 20);

            ESP_LOGD(TAG, "SPO2: %d%%, BPM: %d", g_telemetry.spo2, g_telemetry.bpm);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ========== Telemetry Functions ========== */
uint16_t get_esp32_temperature(void) {
    // TODO: Read actual ESP32 internal temperature sensor
    return 350 + (esp_random() % 50);
}

void update_telemetry(void) {
    if (g_machine_state == STATE_RUNNING) {
        uint32_t current_time = esp_timer_get_time() / 1000000;
        g_telemetry.elapsed_time = current_time - g_session_start_time;

        uint32_t total_duration = g_dialysis_config.session_duration * 60;
        if (g_telemetry.elapsed_time < total_duration) {
            g_telemetry.remaining_time = total_duration - g_telemetry.elapsed_time;
        } else {
            g_telemetry.remaining_time = 0;
        }
    }

    g_telemetry.blood_flow_rate = g_dialysis_config.blood_flow_rate;
    g_telemetry.dialysate_flow_rate = g_dialysis_config.overall_flow_rate;
    g_telemetry.ufr = g_dialysis_config.target_ufr;

    if (g_machine_state == STATE_RUNNING && g_telemetry.elapsed_time > 0) {
        g_telemetry.uf_volume = (g_dialysis_config.target_ufr * g_telemetry.elapsed_time) / 3600;
    }

    g_telemetry.blood_pressure_sys = 120 + (esp_random() % 20);
    g_telemetry.blood_pressure_dia = 80 + (esp_random() % 10);
    g_telemetry.body_temp = 365 + (esp_random() % 15);

    g_telemetry.venous_pressure = 50 + (esp_random() % 100);
    g_telemetry.arterial_pressure = -50 - (esp_random() % 100);
    g_telemetry.dialysate_pressure = 100 + (esp_random() % 50);

    g_telemetry.dialysate_temp = g_dialysis_config.dialysate_temp;
    g_telemetry.dialysate_conductivity = g_dialysis_config.dialysate_conductivity;
    g_telemetry.esp32_temp = get_esp32_temperature();
    g_telemetry.system_voltage = 5000 + (esp_random() % 200);

    g_telemetry.state = g_machine_state;
    g_telemetry.error_code = 0;
}

void send_telemetry(void) {
    if (!g_ble_connected) {
        return;
    }

    update_telemetry();

    esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id, g_char_telemetry_handle,
                                sizeof(telemetry_data_t), (uint8_t *)&g_telemetry, false);
}

void telemetry_task(void *param) {
    ESP_LOGI(TAG, "Telemetry task started");

    while (1) {
        if (g_ble_connected && g_machine_state >= STATE_READY) {
            send_telemetry();
        }

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS));
    }
}

/* ========== Command Processing ========== */
void process_command(command_packet_t *cmd) {
    ESP_LOGI(TAG, "Processing command: type=0x%02X, value=%d", cmd->cmd_type, cmd->value);

    // Quick LED blink to indicate command received
    led_startup_on();
    vTaskDelay(pdMS_TO_TICKS(50));
    led_startup_off();

    switch (cmd->cmd_type) {
        case CMD_START_MACHINE:
            if (g_machine_state == STATE_READY) {
                ESP_LOGI(TAG, "Starting machine...");
                set_machine_state(STATE_STARTING);

                if (g_startup_task_handle == NULL) {
                    xTaskCreate(startup_sequence_task, "startup_task", 4096, NULL, 5, &g_startup_task_handle);
                }
            } else {
                ESP_LOGW(TAG, "Cannot start: machine not in READY state");
            }
            break;

        case CMD_PAUSE_MACHINE:
            if (g_machine_state == STATE_RUNNING) {
                ESP_LOGI(TAG, "Pausing machine...");
                set_machine_state(STATE_PAUSED);
                stepper_stop();
                pump1_stop();
                pump2_stop();
                audio_beep(500);
            }
            break;

        case CMD_RESTART_MACHINE:
            if (g_machine_state == STATE_PAUSED) {
                ESP_LOGI(TAG, "Restarting machine...");
                set_machine_state(STATE_RUNNING);
                stepper_start();
                uint8_t blood_pwm = flow_rate_to_pwm(g_dialysis_config.blood_flow_rate);
                uint8_t waste_pwm = flow_rate_to_pwm(g_dialysis_config.waste_flow_rate);
                pump1_start(blood_pwm);
                pump2_start(waste_pwm);
                audio_beep(200);
            }
            break;

        case CMD_STOP_MACHINE:
            ESP_LOGI(TAG, "Stopping machine...");
            set_machine_state(STATE_STOPPED);
            stepper_stop();
            pump1_stop();
            pump2_stop();
            led_operation_off();
            
            // Flash LED during shutdown
            for (int i = 0; i < 3; i++) {
                led_startup_on();
                vTaskDelay(pdMS_TO_TICKS(100));
                led_startup_off();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            if (s_spiffs_mounted) {
                audio_play_wav_async("/spiffs/shutdown.wav");
            } else {
                audio_beep(300);
                vTaskDelay(pdMS_TO_TICKS(200));
                audio_beep(300);
            }
            break;

        case CMD_SET_OVERALL_FLOW:
            ESP_LOGI(TAG, "Setting overall flow rate to %d ml/min", cmd->value);
            g_dialysis_config.overall_flow_rate = cmd->value;
            break;

        case CMD_SET_BLOOD_FLOW:
            ESP_LOGI(TAG, "Setting blood flow rate to %d ml/min", cmd->value);
            g_dialysis_config.blood_flow_rate = cmd->value;
            if (g_machine_state == STATE_RUNNING) {
                uint8_t pwm = flow_rate_to_pwm(cmd->value);
                pump1_start(pwm);
            }
            break;

        case CMD_SET_WASTE_FLOW:
            ESP_LOGI(TAG, "Setting waste flow rate to %d ml/min", cmd->value);
            g_dialysis_config.waste_flow_rate = cmd->value;
            if (g_machine_state == STATE_RUNNING) {
                uint8_t pwm = flow_rate_to_pwm(cmd->value);
                pump2_start(pwm);
            }
            break;

        case CMD_SET_TIMER:
            ESP_LOGI(TAG, "Setting session duration to %d minutes", cmd->value);
            g_dialysis_config.session_duration = cmd->value;
            break;

        default:
            ESP_LOGW(TAG, "Unknown command type: 0x%02X", cmd->cmd_type);
            break;
    }
}

/* ========== BLE GAP Event Handler ========== */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising stopped");
            break;

        default:
            break;
    }
}

/* ========== BLE GATTS Event Handler ========== */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, app_id %d", param->reg.app_id);
            g_gatts_if = gatts_if;

            esp_ble_gap_set_device_name(DEVICE_NAME);

            // Advertisement data with service UUID
            uint8_t adv_data[] = {
                // Flags
                0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
                // Complete list of 16-bit Service UUIDs
                0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00,  // Service UUID 0x00FF
                // Complete device name
                0x0B, ESP_BLE_AD_TYPE_NAME_CMPL,
                'R', 'i', 'V', 'i', 't', 'a', 'f', 'l', 'o', 'w'
            };
            esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));

            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE, 0);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Attribute table created, num_handle=%d", param->add_attr_tab.num_handle);

                if (param->add_attr_tab.num_handle == GATTS_NUM_HANDLE) {
                    g_service_handle = param->add_attr_tab.handles[0];
                    g_char_command_handle = param->add_attr_tab.handles[2];
                    g_char_telemetry_handle = param->add_attr_tab.handles[4];
                    g_char_config_handle = param->add_attr_tab.handles[7];

                    esp_ble_gatts_start_service(g_service_handle);
                    ESP_LOGI(TAG, "Service started");
                }
            } else {
                ESP_LOGE(TAG, "Attribute table creation failed, status=0x%x", param->add_attr_tab.status);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
            g_ble_connected = true;
            g_conn_id = param->connect.conn_id;

            // Initialize with default dialysis configuration
            g_dialysis_config.session_duration = 240;        // 4 hours (240 minutes)
            g_dialysis_config.overall_flow_rate = 500;       // 500 ml/min
            g_dialysis_config.blood_flow_rate = 300;         // 300 ml/min
            g_dialysis_config.waste_flow_rate = 200;         // 200 ml/min
            g_dialysis_config.target_ufr = 1000;             // 1000 ml/hr (1L per hour)
            g_dialysis_config.dialysate_temp = 37;           // 37°C (body temperature)
            g_dialysis_config.dialysate_conductivity = 14;   // 14 mS/cm (typical dialysate)
            ESP_LOGI(TAG, "Default configuration loaded: %d min session, %d ml/min blood flow",
                     g_dialysis_config.session_duration, g_dialysis_config.blood_flow_rate);

            esp_ble_gap_stop_advertising();

            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;
            conn_params.max_int = 0x20;
            conn_params.latency = 0;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, reason=0x%02x", param->disconnect.reason);
            g_ble_connected = false;
            g_conn_id = 0;

            if (g_machine_state == STATE_RUNNING || g_machine_state == STATE_PAUSED) {
                stepper_stop();
                pump1_stop();
                pump2_stop();
                led_operation_off();
            }

            set_machine_state(STATE_IDLE);
            memset(&g_dialysis_config, 0, sizeof(dialysis_config_t));

            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == g_char_command_handle) {
                if (param->write.len == sizeof(command_packet_t)) {
                    command_packet_t *cmd = (command_packet_t *)param->write.value;
                    process_command(cmd);
                }
            } else if (param->write.handle == g_char_config_handle) {
                if (param->write.len == sizeof(dialysis_config_t)) {
                    memcpy(&g_dialysis_config, param->write.value, sizeof(dialysis_config_t));
                    ESP_LOGI(TAG, "Dialysis config received: duration=%d min, blood_flow=%d ml/min",
                             g_dialysis_config.session_duration, g_dialysis_config.blood_flow_rate);

                    if (g_machine_state == STATE_CONNECTED) {
                        set_machine_state(STATE_READY);
                    }
                }
            }
            break;

        default:
            break;
    }
}

/* ========== Main Application ========== */
void app_main(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "=== RiVitaflow Dialysis Machine Firmware ===");
    ESP_LOGI(TAG, "Version 1.0.0");
    ESP_LOGI(TAG, "Hardware: L298N + 28BYJ-48 + MAX30102 + MAX98357A (I2S audio)");

     /* Enable debug-level logs for audio diagnostics so verbose logs (ESP_LOGD)
         from the WAV/U8 playback functions are visible without changing menuconfig. */
     esp_log_level_set(TAG, ESP_LOG_DEBUG);
     esp_log_level_set("i2s", ESP_LOG_DEBUG);

    // Initialize NVS (required by some libraries)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Mount SPIFFS for audio files and other resources
    audio_fs_init();

    // Initialize hardware subsystems (audio, LEDs, pumps, stepper, sensors)
    hardware_init();

    // Auto-play a startup WAV once for diagnostic verification if SPIFFS mounted.
    if (s_spiffs_mounted) {
        ESP_LOGI(TAG, "Auto-play: attempting to play /spiffs/startup.wav for diagnostics");
        audio_play_wav_async("/spiffs/startup.wav");
        // Give the player a short moment to start before proceeding with BT init
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        ESP_LOGW(TAG, "Auto-play: SPIFFS not mounted; skipping /spiffs/startup.wav");
    }

    // Initialize and enable BT controller + Bluedroid stack for BLE/GATT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
    } else {
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        } else {
            ret = esp_bluedroid_init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
            } else {
                ret = esp_bluedroid_enable();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
                }
            }
        }
    }

    // Register GATT and GAP callbacks and register application
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(APP_ID);

    // Start background tasks (stepper, telemetry, etc.)
    if (g_stepper_task_handle == NULL) {
        xTaskCreate(stepper_task, "stepper", 4096, NULL, 5, &g_stepper_task_handle);
    }

    if (g_telemetry_task_handle == NULL) {
        xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 5, &g_telemetry_task_handle);
    }

    // Start LED status manager to show BLE / state on LED_STARTUP_RED (GPIO2)
    xTaskCreate(led_status_task, "led_status", 2048, NULL, 5, NULL);

    // audio_test_task removed; rely on startup.wav autorun for diagnostics

    ESP_LOGI(TAG, "System initialization complete; BLE advertising will start when GATT app registers.");
}
