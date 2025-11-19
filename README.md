# RiVitaflow OS - Dialysis Machine Firmware

Bluetooth-enabled firmware for ESP32-based dialysis machine control system.

## Overview

RiVitaflow OS is a complete firmware solution for controlling a dialysis machine via Bluetooth Low Energy (BLE). The firmware manages the entire operation lifecycle including startup sequences, motor control, sensor monitoring, and real-time telemetry transmission to a companion mobile app.

### Key Features

- **BLE Communication**: Full GATT server implementation for command reception and telemetry transmission
- **Complete Startup Sequence**: Multi-step initialization with audio and visual feedback
- **Real-time Monitoring**: Continuous SPO2, heart rate, and system parameter monitoring
- **Motor Control**: Stepper motor and dual pump control system
- **State Machine**: Robust state management for safe operation
- **Telemetry Streaming**: 1Hz telemetry data transmission including vital signs, flow rates, pressures, and timing
- **Safety Features**: Automatic shutdown on BLE disconnect, state validation

## Supported Hardware

| Supported Targets | ESP32 | ESP32-S3 |
| ----------------- | ----- | -------- |

## Hardware Requirements

### GPIO Pin Assignments

| Component | GPIO Pin | Type | Description |
|-----------|----------|------|-------------|
| **Buzzer** | GPIO 23 | Output | PWM-controlled buzzer (2kHz) |
| **Red LED (Startup)** | GPIO 25 | Output | Startup sequence indicator |
| **Green LED (Operation)** | GPIO 26 | Output | Running operation indicator |
| **Stepper Step** | GPIO 18 | Output | Stepper motor step signal |
| **Stepper Direction** | GPIO 19 | Output | Stepper motor direction |
| **Stepper Enable** | GPIO 21 | Output | Stepper motor enable (active low) |
| **Blood Pump** | GPIO 22 | Output | Blood pump control |
| **Waste Pump** | GPIO 27 | Output | Waste cycle pump control |
| **I2C SDA (SPO2)** | GPIO 32 | I/O | SPO2 sensor data |
| **I2C SCL (SPO2)** | GPIO 33 | Output | SPO2 sensor clock |

### Required Components

1. ESP32 Development Board
2. Active buzzer or speaker
3. 2x LEDs (red and green) with appropriate resistors
4. Stepper motor with driver (A4988, DRV8825, or similar)
5. 2x DC pumps or pump drivers
6. SPO2 sensor module (I2C interface, e.g., MAX30102)

## Software Architecture

### State Machine

```
IDLE → CONNECTED → READY → STARTING → RUNNING ⇄ PAUSED → STOPPED
  ↑        ↓          ↓        ↓           ↓              ↓
  └────────┴──────────┴────────┴───────────┴──────────────┘
           (BLE disconnect triggers return to IDLE)
```

### State Descriptions

- **IDLE**: Waiting for BLE connection
- **CONNECTED**: BLE connected, waiting for configuration
- **READY**: Configuration received, ready to start
- **STARTING**: Executing startup sequence (windup)
- **RUNNING**: Normal operation in progress
- **PAUSED**: Operation paused, can be resumed
- **STOPPED**: Operation stopped, requires full restart

### BLE GATT Service

**Service UUID**: `0x00FF`

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Command | 0xFF01 | Write | Receive commands from app |
| Telemetry | 0xFF02 | Notify | Send telemetry data to app (1Hz) |
| Config | 0xFF03 | Write | Receive dialysis configuration |

## Startup Sequence

The firmware executes the following startup sequence automatically:

1. **Boot Beep**: Single 200ms beep on power-up
2. **BLE Advertising**: Wait for Bluetooth connection
3. **Connection Established**: Double beep (100ms × 2)
4. **Configuration Wait**: Wait for dialysis config from app
5. **Ready State**: Triple beep (150ms × 3)
6. **Start Command**: Wait for START command from app
7. **Windup**: 3-second beep with red LED on
8. **Motor Startup**:
   - Stepper motor runs for 2 seconds
   - Blood pump starts
   - Waste pump starts
9. **Operation**: Red LED off, green LED on, triple beep

## Commands

Commands are sent as binary packets to the Command characteristic (0xFF01).

### Command Packet Structure

```c
struct command_packet_t {
    uint8_t cmd_type;    // Command type code
    uint16_t value;      // Command parameter value
} __attribute__((packed));
```

### Command Types

| Command | Code | Value | Description |
|---------|------|-------|-------------|
| START_MACHINE | 0x01 | N/A | Start operation (only from READY state) |
| PAUSE_MACHINE | 0x02 | N/A | Pause operation (from RUNNING state) |
| RESTART_MACHINE | 0x03 | N/A | Resume operation (from PAUSED state) |
| STOP_MACHINE | 0x04 | N/A | Stop operation completely |
| SET_OVERALL_FLOW | 0x10 | ml/min | Set overall dialysate flow rate |
| SET_BLOOD_FLOW | 0x11 | ml/min | Set blood pump flow rate |
| SET_WASTE_FLOW | 0x12 | ml/min | Set waste pump flow rate |
| SET_TIMER | 0x20 | minutes | Set session duration |

## Configuration

Configuration must be sent to the Config characteristic (0xFF03) after BLE connection and before starting operation.

### Configuration Structure

```c
struct dialysis_config_t {
    uint16_t session_duration;      // Session duration in minutes
    uint16_t overall_flow_rate;     // Overall flow rate (ml/min)
    uint16_t blood_flow_rate;       // Blood flow rate (ml/min)
    uint16_t waste_flow_rate;       // Waste flow rate (ml/min)
    uint16_t target_ufr;            // Target ultrafiltration rate (ml/hr)
    uint8_t dialysate_temp;         // Dialysate temperature (Celsius)
    uint8_t dialysate_conductivity; // Conductivity (mS/cm)
} __attribute__((packed));
```

## Telemetry Data

Telemetry is automatically sent every 1 second via the Telemetry characteristic (0xFF02) when in READY, STARTING, RUNNING, or PAUSED states.

### Telemetry Structure

```c
struct telemetry_data_t {
    // Vital signs
    uint8_t spo2;                   // SPO2 percentage (0-100)
    uint8_t bpm;                    // Heart rate (beats per minute)
    uint16_t blood_pressure_sys;    // Systolic BP (mmHg)
    uint16_t blood_pressure_dia;    // Diastolic BP (mmHg)
    uint16_t body_temp;             // Body temperature (×10, e.g., 370 = 37.0°C)

    // Flow rates
    uint16_t blood_flow_rate;       // Current blood flow (ml/min)
    uint16_t dialysate_flow_rate;   // Current dialysate flow (ml/min)
    uint16_t ufr;                   // Ultrafiltration rate (ml/hr)
    uint16_t uf_volume;             // Total UF volume (ml)

    // Pressures
    int16_t venous_pressure;        // Venous pressure (mmHg)
    int16_t arterial_pressure;      // Arterial pressure (mmHg)
    int16_t dialysate_pressure;     // Dialysate pressure (mmHg)

    // System parameters
    uint8_t dialysate_temp;         // Dialysate temperature (°C)
    uint8_t dialysate_conductivity; // Conductivity (mS/cm)
    uint16_t esp32_temp;            // ESP32 temperature (×10)
    uint16_t system_voltage;        // System voltage (mV)

    // Time information
    uint32_t elapsed_time;          // Elapsed time (seconds)
    uint32_t remaining_time;        // Remaining time (seconds)

    // Machine state
    uint8_t state;                  // Current state
    uint8_t error_code;             // Error code (0 = no error)
} __attribute__((packed));
```

## Building and Flashing

### Prerequisites

1. Install ESP-IDF (version 5.0 or later)
2. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Build Steps

1. **Set Target**:
   ```bash
   idf.py set-target esp32
   ```

2. **Configure (Optional)**:
   ```bash
   idf.py menuconfig
   ```

3. **Build**:
   ```bash
   idf.py build
   ```

4. **Flash and Monitor**:
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

   Replace `/dev/ttyUSB0` with your ESP32's serial port.

5. **Exit Monitor**: Press `Ctrl + ]`

## Usage Example

### App Flow

1. **Scan for BLE devices** and connect to "RiVitaflow"
2. **Discover services** and locate service 0x00FF
3. **Subscribe to telemetry** notifications (characteristic 0xFF02)
4. **Send configuration**:
   ```
   Write to 0xFF03:
   [180, 0, 200, 1, 200, 0, 100, 0, 500, 1, 37, 14]
   // 180 min, 450 ml/min overall, 200 ml/min blood, 100 ml/min waste,
   // 500 ml/hr UFR, 37°C, 14 mS/cm
   ```

5. **Receive confirmation**: Machine beeps 3 times (READY state)

6. **Start operation**:
   ```
   Write to 0xFF01: [0x01, 0x00, 0x00]
   // START_MACHINE command
   ```

7. **Monitor telemetry**: Receive data every 1 second

8. **Control during operation**:
   ```
   Write to 0xFF01: [0x02, 0x00, 0x00]  // Pause
   Write to 0xFF01: [0x03, 0x00, 0x00]  // Restart
   Write to 0xFF01: [0x11, 0x00, 0x01]  // Set blood flow to 256 ml/min
   ```

## Development Notes

### TODO Items

The firmware currently has placeholder implementations for:

1. **Stepper Motor Stepping**: Implement timer-based step generation
2. **Pump PWM Control**: Implement PWM-based flow rate control
3. **Actual SPO2 Reading**: Integrate with MAX30102 or similar sensor via I2C
4. **ESP32 Temperature**: Read internal temperature sensor
5. **Blood Pressure Monitoring**: Integrate actual BP sensor if available

### Adding Features

To add new commands:

1. Add command code to `command_type_t` enum in `rivitaflow.h`
2. Implement handler in `process_command()` in `main.c`
3. Update app to send new command format

## Safety Considerations

- Machine automatically stops all motors on BLE disconnect
- State transitions are validated before execution
- Commands are only accepted in appropriate states
- Configuration must be provided before starting operation

## Troubleshooting

### Build Issues

**Error: `esp_bt.h` not found**
- Ensure Bluetooth is enabled in `sdkconfig`: `CONFIG_BT_ENABLED=y`
- Run `idf.py menuconfig` → Component config → Bluetooth → Enable

**Error: GPIO conflicts**
- Check that GPIO pins are not used by other peripherals
- Modify pin definitions in `rivitaflow.h` if needed

### Runtime Issues

**BLE not advertising**
- Check serial output for initialization errors
- Ensure NVS partition is properly formatted
- Verify Bluetooth controller is enabled

**Motor not running**
- Verify stepper driver enable pin polarity (active low by default)
- Check power supply to motors
- Verify GPIO connections

**No telemetry data**
- Ensure app has subscribed to notifications on characteristic 0xFF02
- Check BLE connection is established (g_ble_connected)
- Verify machine is in READY or higher state

## License

This project is provided as-is for educational and development purposes.

## Version

**Firmware Version**: 1.0.0

## Contributors

RiVitaflow Development Team
# RiVitaflow
