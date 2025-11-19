# RiVitaflow BLE Protocol Reference

Quick reference guide for mobile app developers integrating with RiVitaflow firmware.

## Connection Information

- **Device Name**: `RiVitaflow`
- **Service UUID**: `0x00FF` (or `0000FF00-0000-1000-8000-00805F9B34FB` in 128-bit format)

## GATT Characteristics

| Name | UUID | Type | Max Size | Description |
|------|------|------|----------|-------------|
| Command | 0xFF01 | Write | 3 bytes | Send commands to ESP32 |
| Telemetry | 0xFF02 | Notify | 58 bytes | Receive telemetry data (1Hz) |
| Config | 0xFF03 | Write | 14 bytes | Send dialysis configuration |

## Quick Start Flow

```
1. Scan and connect to "RiVitaflow"
2. Discover services (UUID: 0x00FF)
3. Enable notifications on Telemetry (0xFF02)
4. Send Configuration to 0xFF03
   → ESP32 beeps 3 times (READY)
5. Send START command to 0xFF01
   → ESP32 executes startup sequence
6. Receive telemetry every 1 second
7. Send control commands as needed
```

## Data Structures

### 1. Command Packet (3 bytes)

Send to characteristic **0xFF01**

```
Byte 0: Command Type (uint8)
Byte 1: Value Low Byte (uint8)
Byte 2: Value High Byte (uint8)
```

**Example**: Start machine
```javascript
const cmd = new Uint8Array([0x01, 0x00, 0x00]);
await commandChar.writeValue(cmd);
```

### 2. Configuration Packet (14 bytes)

Send to characteristic **0xFF03**

```
Bytes 0-1:  Session Duration (uint16, minutes)
Bytes 2-3:  Overall Flow Rate (uint16, ml/min)
Bytes 4-5:  Blood Flow Rate (uint16, ml/min)
Bytes 6-7:  Waste Flow Rate (uint16, ml/min)
Bytes 8-9:  Target UFR (uint16, ml/hr)
Byte  10:   Dialysate Temperature (uint8, Celsius)
Byte  11:   Dialysate Conductivity (uint8, mS/cm)
Bytes 12-13: Padding (uint16, reserved)
```

**Example**: 3-hour session, 450/200/100 ml/min, 500 ml/hr UFR, 37°C, 14 mS/cm
```javascript
const config = new Uint8Array([
    180, 0,     // 180 minutes (little-endian)
    194, 1,     // 450 ml/min overall (450 = 0x01C2)
    200, 0,     // 200 ml/min blood
    100, 0,     // 100 ml/min waste
    244, 1,     // 500 ml/hr UFR (500 = 0x01F4)
    37,         // 37°C
    14,         // 14 mS/cm
    0, 0        // Padding
]);
await configChar.writeValue(config);
```

### 3. Telemetry Packet (58 bytes)

Received from characteristic **0xFF02** (notifications)

```
Byte  0:     SPO2 (uint8, %)
Byte  1:     BPM (uint8, beats/min)
Bytes 2-3:   Blood Pressure Systolic (uint16, mmHg)
Bytes 4-5:   Blood Pressure Diastolic (uint16, mmHg)
Bytes 6-7:   Body Temperature (uint16, ×10, e.g., 370 = 37.0°C)
Bytes 8-9:   Blood Flow Rate (uint16, ml/min)
Bytes 10-11: Dialysate Flow Rate (uint16, ml/min)
Bytes 12-13: Ultrafiltration Rate (uint16, ml/hr)
Bytes 14-15: UF Volume (uint16, ml)
Bytes 16-17: Venous Pressure (int16, mmHg)
Bytes 18-19: Arterial Pressure (int16, mmHg)
Bytes 20-21: Dialysate Pressure (int16, mmHg)
Byte  22:    Dialysate Temperature (uint8, °C)
Byte  23:    Dialysate Conductivity (uint8, mS/cm)
Bytes 24-25: ESP32 Temperature (uint16, ×10)
Bytes 26-27: System Voltage (uint16, mV)
Bytes 28-31: Elapsed Time (uint32, seconds)
Bytes 32-35: Remaining Time (uint32, seconds)
Byte  36:    Machine State (uint8)
Byte  37:    Error Code (uint8)
Bytes 38-57: Reserved (padding to 58 bytes)
```

**JavaScript Parsing Example**:
```javascript
function parseTelemetry(dataView) {
    return {
        spo2: dataView.getUint8(0),
        bpm: dataView.getUint8(1),
        bloodPressure: {
            systolic: dataView.getUint16(2, true),
            diastolic: dataView.getUint16(4, true)
        },
        bodyTemp: dataView.getUint16(6, true) / 10,
        bloodFlowRate: dataView.getUint16(8, true),
        dialysateFlowRate: dataView.getUint16(10, true),
        ufr: dataView.getUint16(12, true),
        ufVolume: dataView.getUint16(14, true),
        pressures: {
            venous: dataView.getInt16(16, true),
            arterial: dataView.getInt16(18, true),
            dialysate: dataView.getInt16(20, true)
        },
        dialysateTemp: dataView.getUint8(22),
        dialysateConductivity: dataView.getUint8(23),
        esp32Temp: dataView.getUint16(24, true) / 10,
        systemVoltage: dataView.getUint16(26, true) / 1000,
        elapsedTime: dataView.getUint32(28, true),
        remainingTime: dataView.getUint32(32, true),
        state: dataView.getUint8(36),
        errorCode: dataView.getUint8(37)
    };
}
```

## Command Reference

All commands are 3 bytes: `[cmd_type, value_low, value_high]`

### Machine Control Commands

| Command | Code | Value | Example |
|---------|------|-------|---------|
| Start | 0x01 | 0x0000 | `[0x01, 0x00, 0x00]` |
| Pause | 0x02 | 0x0000 | `[0x02, 0x00, 0x00]` |
| Restart | 0x03 | 0x0000 | `[0x03, 0x00, 0x00]` |
| Stop | 0x04 | 0x0000 | `[0x04, 0x00, 0x00]` |

### Flow Rate Commands

| Command | Code | Value | Example (300 ml/min) |
|---------|------|-------|----------------------|
| Overall Flow | 0x10 | ml/min | `[0x10, 0x2C, 0x01]` (0x012C = 300) |
| Blood Flow | 0x11 | ml/min | `[0x11, 0x2C, 0x01]` |
| Waste Flow | 0x12 | ml/min | `[0x12, 0x2C, 0x01]` |

### Timer Command

| Command | Code | Value | Example (240 min) |
|---------|------|-------|-------------------|
| Set Timer | 0x20 | minutes | `[0x20, 0xF0, 0x00]` (0x00F0 = 240) |

## Machine States

| State Code | Name | Description |
|------------|------|-------------|
| 0 | IDLE | Waiting for BLE connection |
| 1 | CONNECTED | BLE connected, waiting for config |
| 2 | READY | Config received, ready to start |
| 3 | STARTING | Executing startup sequence |
| 4 | RUNNING | Normal operation |
| 5 | PAUSED | Operation paused |
| 6 | STOPPED | Operation stopped |
| 7 | ERROR | Error state |

## Complete JavaScript Example

```javascript
class RiVitaflowDevice {
    constructor() {
        this.device = null;
        this.server = null;
        this.service = null;
        this.commandChar = null;
        this.telemetryChar = null;
        this.configChar = null;
    }

    async connect() {
        // Request device
        this.device = await navigator.bluetooth.requestDevice({
            filters: [{ name: 'RiVitaflow' }],
            optionalServices: [0x00FF]
        });

        // Connect to GATT server
        this.server = await this.device.gatt.connect();

        // Get service
        this.service = await this.server.getPrimaryService(0x00FF);

        // Get characteristics
        this.commandChar = await this.service.getCharacteristic(0xFF01);
        this.telemetryChar = await this.service.getCharacteristic(0xFF02);
        this.configChar = await this.service.getCharacteristic(0xFF03);

        // Subscribe to telemetry
        await this.telemetryChar.startNotifications();
        this.telemetryChar.addEventListener('characteristicvaluechanged',
            this.handleTelemetry.bind(this));
    }

    async sendConfig(duration, overallFlow, bloodFlow, wasteFlow, ufr, temp, conductivity) {
        const config = new Uint8Array(14);
        const view = new DataView(config.buffer);

        view.setUint16(0, duration, true);
        view.setUint16(2, overallFlow, true);
        view.setUint16(4, bloodFlow, true);
        view.setUint16(6, wasteFlow, true);
        view.setUint16(8, ufr, true);
        view.setUint8(10, temp);
        view.setUint8(11, conductivity);

        await this.configChar.writeValue(config);
    }

    async sendCommand(cmdType, value = 0) {
        const cmd = new Uint8Array([
            cmdType,
            value & 0xFF,
            (value >> 8) & 0xFF
        ]);
        await this.commandChar.writeValue(cmd);
    }

    async start() {
        await this.sendCommand(0x01);
    }

    async pause() {
        await this.sendCommand(0x02);
    }

    async restart() {
        await this.sendCommand(0x03);
    }

    async stop() {
        await this.sendCommand(0x04);
    }

    async setBloodFlow(mlPerMin) {
        await this.sendCommand(0x11, mlPerMin);
    }

    handleTelemetry(event) {
        const value = event.target.value;
        const data = {
            spo2: value.getUint8(0),
            bpm: value.getUint8(1),
            bloodPressureSys: value.getUint16(2, true),
            bloodPressureDia: value.getUint16(4, true),
            bodyTemp: value.getUint16(6, true) / 10,
            bloodFlowRate: value.getUint16(8, true),
            dialysateFlowRate: value.getUint16(10, true),
            ufr: value.getUint16(12, true),
            ufVolume: value.getUint16(14, true),
            venousPressure: value.getInt16(16, true),
            arterialPressure: value.getInt16(18, true),
            dialysatePressure: value.getInt16(20, true),
            dialysateTemp: value.getUint8(22),
            dialysateConductivity: value.getUint8(23),
            esp32Temp: value.getUint16(24, true) / 10,
            systemVoltage: value.getUint16(26, true) / 1000,
            elapsedTime: value.getUint32(28, true),
            remainingTime: value.getUint32(32, true),
            state: value.getUint8(36),
            errorCode: value.getUint8(37)
        };

        console.log('Telemetry:', data);
        this.onTelemetry(data);
    }

    onTelemetry(data) {
        // Override this method to handle telemetry data
    }
}

// Usage
async function main() {
    const device = new RiVitaflowDevice();

    // Connect
    await device.connect();

    // Send configuration: 180 min, 450/200/100 ml/min, 500 ml/hr UFR, 37°C, 14 mS/cm
    await device.sendConfig(180, 450, 200, 100, 500, 37, 14);

    // Wait for READY state (listen for 3 beeps)
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Start operation
    await device.start();

    // Handle telemetry
    device.onTelemetry = (data) => {
        console.log(`SPO2: ${data.spo2}%, BPM: ${data.bpm}`);
        console.log(`Blood Flow: ${data.bloodFlowRate} ml/min`);
        console.log(`State: ${data.state}`);
    };
}
```

## React Native Example (Bluetooth LE)

```javascript
import BleManager from 'react-native-ble-manager';

const SERVICE_UUID = '0000FF00-0000-1000-8000-00805F9B34FB';
const CHAR_COMMAND = '0000FF01-0000-1000-8000-00805F9B34FB';
const CHAR_TELEMETRY = '0000FF02-0000-1000-8000-00805F9B34FB';
const CHAR_CONFIG = '0000FF03-0000-1000-8000-00805F9B34FB';

class RiVitaflowManager {
    async connect() {
        // Scan for device
        await BleManager.scan([], 5, true);

        // Find RiVitaflow device
        const devices = await BleManager.getDiscoveredPeripherals();
        const rivitaflow = devices.find(d => d.name === 'RiVitaflow');

        if (rivitaflow) {
            // Connect
            await BleManager.connect(rivitaflow.id);

            // Retrieve services
            await BleManager.retrieveServices(rivitaflow.id);

            // Start notifications
            await BleManager.startNotification(
                rivitaflow.id,
                SERVICE_UUID,
                CHAR_TELEMETRY
            );

            return rivitaflow.id;
        }

        throw new Error('RiVitaflow device not found');
    }

    async sendConfig(deviceId, config) {
        const buffer = new ArrayBuffer(14);
        const view = new DataView(buffer);

        view.setUint16(0, config.duration, true);
        view.setUint16(2, config.overallFlow, true);
        view.setUint16(4, config.bloodFlow, true);
        view.setUint16(6, config.wasteFlow, true);
        view.setUint16(8, config.ufr, true);
        view.setUint8(10, config.temp);
        view.setUint8(11, config.conductivity);

        const data = Array.from(new Uint8Array(buffer));
        await BleManager.write(deviceId, SERVICE_UUID, CHAR_CONFIG, data);
    }

    async start(deviceId) {
        const data = [0x01, 0x00, 0x00];
        await BleManager.write(deviceId, SERVICE_UUID, CHAR_COMMAND, data);
    }
}
```

## Error Codes

| Code | Description |
|------|-------------|
| 0 | No error |
| 1-255 | Reserved for future use |

## Important Notes

1. **All multi-byte integers are little-endian** (LSB first)
2. **Telemetry is sent every 1 second** when state ≥ READY
3. **Configuration must be sent** before starting operation
4. **BLE disconnect** automatically stops all motors and resets to IDLE
5. **Maximum MTU**: 500 bytes (configured in firmware)

## Troubleshooting

**Problem**: Can't find device
- Ensure ESP32 is powered and advertising
- Check device name is exactly "RiVitaflow"
- Try scanning multiple times

**Problem**: Write fails
- Check characteristic UUID is correct
- Ensure data length matches expected size
- Verify device is in correct state for command

**Problem**: No telemetry notifications
- Ensure you've called `startNotifications()` or equivalent
- Check device is in READY or higher state
- Verify BLE connection is active

## Support

For firmware-related questions, refer to the main README.md file.
