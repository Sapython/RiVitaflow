/*
 * RiVitaflow OS - Dialysis Machine Firmware
 * Hardware definitions, constants, and data structures
 *
 * Hardware Configuration:
 * - L298N Motor Driver (2 DC Pumps: Blood + Waste)
 * - 28BYJ-48 Stepper Motor with ULN2003 Driver
 * - MAX30102 SpO2 and Heart Rate Sensor
 * - MAX98357A I2S DAC + Speakers
 * - LEDs (Red startup, Green operation)
 */

#ifndef RIVITAFLOW_H
#define RIVITAFLOW_H

#include <stdint.h>
#include <stdbool.h>

/* ========== GPIO Pin Definitions ========== */

#define PUMP1_ENA_PIN           GPIO_NUM_18  // PWM speed control
#define PUMP1_IN1_PIN           GPIO_NUM_19  // Direction control (was GPIO26)
#define PUMP1_IN2_PIN           GPIO_NUM_27  // Direction control

// L298N Motor Driver - Pump 2 (Waste Pump)
#define PUMP2_ENB_PIN           GPIO_NUM_14  // PWM speed control
#define PUMP2_IN3_PIN           GPIO_NUM_12  // Direction control
#define PUMP2_IN4_PIN           GPIO_NUM_13  // Direction control

// 28BYJ-48 Stepper Motor (ULN2003 Driver)
#define STEPPER_IN1_PIN         GPIO_NUM_16
#define STEPPER_IN2_PIN         GPIO_NUM_17
#define STEPPER_IN3_PIN         GPIO_NUM_4
#define STEPPER_IN4_PIN         GPIO_NUM_5

// MAX30102 Sensor (I2C)
#define I2C_SDA_PIN             GPIO_NUM_21
#define I2C_SCL_PIN             GPIO_NUM_22
#define I2C_FREQ_HZ             100000
#define I2C_PORT                I2C_NUM_0

// Audio output - using MAX98357A (I2S DAC/amp)
// I2S pins (BCLK / LRCK / DATA_OUT). Choose pins that don't conflict with other
// peripherals. These defaults work for many ESP32 dev boards but verify against
// your hardware wiring.
#define I2S_BCK_PIN             GPIO_NUM_32  // Serial clock (BCK)
#define I2S_LRCK_PIN            GPIO_NUM_25  // Word clock / LRCK
#define I2S_DATA_OUT_PIN        GPIO_NUM_33  // Serial data (DOUT)

// Note: internal ESP32 DAC pins (GPIO25/26) are not used — audio is via I2S.

// LED Indicators
#define LED_STARTUP_RED         GPIO_NUM_2
#define LED_OPERATION_GREEN     GPIO_NUM_15

/* ========== PWM Configuration ========== */
// LEDC channels for PWM control
#define LEDC_TIMER_PUMPS        LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)
#define LEDC_FREQUENCY          1000              // 1kHz for pumps

#define LEDC_CHANNEL_PUMP1      LEDC_CHANNEL_0
#define LEDC_CHANNEL_PUMP2      LEDC_CHANNEL_1

/* ========== Stepper Motor Configuration ========== */
// 28BYJ-48 specifications
#define STEPPER_STEPS_PER_REV   2048  // With gear reduction (64 * 32)
#define STEPPER_STEP_DELAY_MS   2     // Delay between steps (for speed control)

// Step sequence for 28BYJ-48 (half-step mode for smoother operation)
#define STEPPER_SEQUENCE_LENGTH 8

/* ========== Audio Configuration ========== */
#define AUDIO_SAMPLE_RATE       16000  // 16kHz for audio (16-bit stereo)
#define BEEP_FREQUENCY          2000   // 2kHz beep tone

/* ========== Bluetooth GATT Service UUIDs ========== */
// Custom service UUID for dialysis machine
#define GATTS_SERVICE_UUID      0x00FF

// Characteristic UUIDs
#define CHAR_UUID_COMMAND       0xFF01  // Write - Receive commands from app
#define CHAR_UUID_TELEMETRY     0xFF02  // Notify - Send telemetry to app
#define CHAR_UUID_CONFIG        0xFF03  // Write - Receive dialysis config

// GATT handles
#define GATTS_NUM_HANDLE        8

/* ========== System States ========== */
typedef enum {
    STATE_IDLE = 0,          // Waiting for Bluetooth connection
    STATE_CONNECTED,         // BLE connected, waiting for config
    STATE_READY,             // Config received, ready to start
    STATE_STARTING,          // Windup sequence (3 sec beep)
    STATE_RUNNING,           // Normal operation
    STATE_PAUSED,            // Operation paused
    STATE_STOPPED,           // Operation stopped
    STATE_ERROR              // Error state
} machine_state_t;

/* ========== Command Definitions ========== */
typedef enum {
    CMD_START_MACHINE = 0x01,
    CMD_PAUSE_MACHINE = 0x02,
    CMD_RESTART_MACHINE = 0x03,
    CMD_STOP_MACHINE = 0x04,
    CMD_SET_OVERALL_FLOW = 0x10,
    CMD_SET_BLOOD_FLOW = 0x11,
    CMD_SET_WASTE_FLOW = 0x12,
    CMD_SET_TIMER = 0x20
} command_type_t;

/* ========== Data Structures ========== */

// Command packet structure
typedef struct {
    uint8_t cmd_type;        // Command type from command_type_t
    uint16_t value;          // Command value (flow rate, timer, etc.)
} __attribute__((packed)) command_packet_t;

// Dialysis configuration
typedef struct {
    uint16_t session_duration;    // Session duration in minutes
    uint16_t overall_flow_rate;   // Overall flow rate (ml/min)
    uint16_t blood_flow_rate;     // Blood flow rate (ml/min)
    uint16_t waste_flow_rate;     // Waste flow rate (ml/min)
    uint16_t target_ufr;          // Target ultrafiltration rate (ml/hr)
    uint8_t dialysate_temp;       // Dialysate temperature (Celsius)
    uint8_t dialysate_conductivity; // Conductivity (mS/cm)
} __attribute__((packed)) dialysis_config_t;

// Telemetry data structure
typedef struct {
    // Vital signs (from MAX30102)
    uint8_t spo2;                 // SPO2 percentage (0-100)
    uint8_t bpm;                  // Heart rate (beats per minute)
    uint16_t blood_pressure_sys;  // Systolic blood pressure (mmHg)
    uint16_t blood_pressure_dia;  // Diastolic blood pressure (mmHg)
    uint16_t body_temp;           // Body temperature (x10, e.g., 370 = 37.0°C)

    // Flow rates
    uint16_t blood_flow_rate;     // Current blood flow rate (ml/min)
    uint16_t dialysate_flow_rate; // Current dialysate flow rate (ml/min)
    uint16_t ufr;                 // Current ultrafiltration rate (ml/hr)
    uint16_t uf_volume;           // Total ultrafiltration volume (ml)

    // Pressures
    int16_t venous_pressure;      // Venous pressure (mmHg)
    int16_t arterial_pressure;    // Arterial pressure (mmHg)
    int16_t dialysate_pressure;   // Dialysate pressure (mmHg)

    // System parameters
    uint8_t dialysate_temp;       // Dialysate temperature (Celsius)
    uint8_t dialysate_conductivity; // Conductivity (mS/cm)
    uint16_t esp32_temp;          // ESP32 internal temp (x10)
    uint16_t system_voltage;      // System voltage (mV)

    // Time information
    uint32_t elapsed_time;        // Elapsed time (seconds)
    uint32_t remaining_time;      // Remaining time (seconds)

    // Machine state
    uint8_t state;                // Current machine state
    uint8_t error_code;           // Error code (0 = no error)

} __attribute__((packed)) telemetry_data_t;

/* ========== Global Variables ========== */
extern machine_state_t g_machine_state;
extern dialysis_config_t g_dialysis_config;
extern telemetry_data_t g_telemetry;
extern bool g_ble_connected;
extern uint16_t g_conn_id;
extern esp_gatt_if_t g_gatts_if;

/* ========== Function Declarations ========== */

// Hardware initialization
void hardware_init(void);
void audio_init(void);
void led_init(void);
void pumps_init(void);
void stepper_init(void);
void spo2_init(void);

// Audio feedback
void audio_beep(uint32_t duration_ms);
void audio_long_beep(void);  // 3 second beep
void audio_tone(uint16_t frequency, uint32_t duration_ms);

// LED control
void led_startup_on(void);
void led_startup_off(void);
void led_operation_on(void);
void led_operation_off(void);

// L298N Pump control (PWM + direction)
void pump1_start(uint16_t speed);  // speed 0-255 (PWM duty)
void pump1_stop(void);
void pump2_start(uint16_t speed);  // speed 0-255 (PWM duty)
void pump2_stop(void);
void pumps_set_flow_rates(uint16_t blood_flow, uint16_t waste_flow);

// 28BYJ-48 Stepper motor control
void stepper_start(void);
void stepper_stop(void);
void stepper_task(void *param);  // Background task for continuous rotation

// Sensor reading
void spo2_read_task(void *param);

// Command processing
void process_command(command_packet_t *cmd);

// Telemetry
void telemetry_task(void *param);
void send_telemetry(void);

// State machine
void startup_sequence_task(void *param);
void set_machine_state(machine_state_t new_state);

// Utilities
uint16_t get_esp32_temperature(void);
void update_telemetry(void);
uint8_t flow_rate_to_pwm(uint16_t flow_ml_per_min);

#endif // RIVITAFLOW_H
