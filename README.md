# IAC Valve Tester

## 1997 Jeep Wrangler 2.5L - Idle Air Control Valve Testing System

This project allows you to:
1. **Control the IAC valve directly** - Bypass the PCM and drive the stepper motor in/out
2. **Monitor PCM signals** - See what the PCM is commanding in real-time
3. **Log and analyze** - Record signal timing for diagnosis

---

## Hardware Components

| Component | Qty | Purpose |
|-----------|-----|---------|
| ESP32 DevKitC | 1 | Main controller + WiFi |
| TB6612FNG Motor Driver | 1 | Drive IAC stepper (12V) |
| PC817 8-Channel Optocoupler | 1 | Isolate PCM signals |
| LM2596 Buck Converter | 1 | 12V → 5V for ESP32 |
| 10µF 50V Capacitor | 1 | Filter motor supply |
| Inline Fuse Holder + 5A Fuse | 1 | Protect circuit |
| IAC Connector Pigtail | 1 | Connect to valve |

---

## Wiring Connections

### Power Distribution

```
CAR 12V ──[5A FUSE]──┬── LM2596 IN+ ── OUT+ (5V) ──► ESP32 VIN
                     │
                     └── TB6612 VM (motor power)

CAR GND ─────────────┬── LM2596 IN-
                     ├── TB6612 GND
                     ├── ESP32 GND
                     └── PC817 GND
```

### ESP32 to TB6612FNG (Motor Control)

```
ESP32 Pin    →    TB6612 Pin    Description
─────────────────────────────────────────────
GPIO25       →    AIN1          Coil A control 1
GPIO26       →    AIN2          Coil A control 2
GPIO27       →    BIN1          Coil B control 1
GPIO14       →    BIN2          Coil B control 2
GPIO13       →    STBY          Standby (HIGH = enabled)
3.3V         →    VCC           Logic power
```

### TB6612FNG to IAC Valve

```
TB6612 Pin    →    IAC Pin    Description
─────────────────────────────────────────────
AOUT1         →    A1         Coil A terminal 1
AOUT2         →    A2         Coil A terminal 2
BOUT1         →    B1         Coil B terminal 1
BOUT2         →    B2         Coil B terminal 2
```

### PCM Connector to PC817 Optocoupler

The PCM connector is the original plug that would connect to the IAC valve.
We tap these signals to monitor what the PCM is commanding.

```
PCM Plug Pin    →    PC817 Input    Description
─────────────────────────────────────────────────
A1 wire         →    IN1+           Coil A signal 1
A2 wire         →    IN2+           Coil A signal 2
B1 wire         →    IN3+           Coil B signal 1
B2 wire         →    IN4+           Coil B signal 2
```

### PC817 Optocoupler to ESP32 (Signal Monitoring)

```
PC817 Output    →    ESP32 Pin    Description
─────────────────────────────────────────────────
OUT1            →    GPIO32       PCM A1 state
OUT2            →    GPIO33       PCM A2 state
OUT3            →    GPIO34       PCM B1 state
OUT4            →    GPIO35       PCM B2 state
VCC             →    3.3V         Optocoupler logic power
GND             →    GND          Common ground
```

---

## Physical Setup for Testing

### Bench Testing (IAC Valve Only)
```
                    ┌─────────────────┐
   12V Power ───────┤ Your Tester     │
   Supply           │   Circuit       ├──── IAC Valve (on bench)
   (or battery)     │                 │
                    └─────────────────┘
```

### In-Vehicle Testing
```
                                    ┌─── IAC Valve (installed on Jeep)
                                    │
   ┌────────────┐    Your Tester    │
   │    PCM     ├────────────────┬──┘
   │            │   (monitoring) │
   └────────────┘                │
        │                        │
        └── Original IAC ────────┘
             Connector           (controlling)
             (disconnected
              from valve)
```

**In-vehicle test procedure:**
1. Disconnect the factory IAC connector from the valve
2. Connect your tester to the IAC valve (to control it)
3. Connect the optocoupler inputs to the factory connector (to monitor PCM)
4. Start engine with your tester controlling the IAC

---

## Software Setup

### Prerequisites
- Arduino IDE 2.x (or PlatformIO)
- ESP32 board package installed
- ArduinoJson library installed

### Configuration

Edit the top of `iac_tester.ino`:

```cpp
// Option 1: Access Point Mode (recommended for field use)
const bool AP_MODE = true;
const char* AP_SSID = "IAC_Tester";
const char* AP_PASSWORD = "jeep1997";

// Option 2: Connect to your WiFi
const bool AP_MODE = false;
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### Upload
1. Connect ESP32 via USB
2. Select board: "ESP32 Dev Module"
3. Select correct COM port
4. Upload

### Connect
- **AP Mode:** Connect phone/laptop to "IAC_Tester" WiFi, browse to `192.168.4.1`
- **Station Mode:** Check Serial Monitor for IP address, browse to that IP

---

## Web Interface Features

### Motor Control
- **Extend/Retract buttons** - Hold to move continuously
- **Quick step buttons** - Move fixed number of steps (+10, +50, +100, etc.)
- **Emergency Stop** - Immediately disables motor driver

### Settings
- **Step Delay** - Time between steps (lower = faster movement)
- **Frequency display** - Calculated from step delay

### PCM Signal Monitor
- **LED indicators** - Real-time view of PCM signal states
- **Signal log** - Records all state changes with timestamps
- **IAC Active marker** - Shows when you were pressing motor buttons
- **CSV Export** - Download log for analysis in spreadsheet

---

## Troubleshooting

### Motor doesn't move
1. Check 12V at TB6612 VM pin
2. Verify STBY pin is HIGH (GPIO13)
3. Test IAC coil resistance (should be 10-20Ω per coil)
4. Verify wiring matches coil pairs

### No PCM signals detected
1. Engine must be running for PCM to drive IAC
2. Check optocoupler board power (3.3V on VCC)
3. Verify input polarity on optocoupler
4. Signals are inverted - code compensates for this

### Web interface won't load
1. Confirm WiFi connection to IAC_Tester network
2. Check Serial Monitor for IP address
3. Try 192.168.4.1 in AP mode
4. Disable mobile data on phone when testing

### ESP32 won't boot
1. Verify LM2596 output is 5V (not higher!)
2. Check for shorts
3. Disconnect motor driver, test ESP32 alone

---

## IAC Valve Pinout Reference

### 1997 Jeep Wrangler 2.5L / 4.0L

The IAC connector is a 4-pin Weatherpack connector.

Looking at the connector face (wire side):
```
    ┌─────────┐
    │  1   2  │
    │  3   4  │
    └─────────┘
```

| Pin | Wire Color | Function |
|-----|------------|----------|
| 1 | Gray/Red | Coil A - Terminal 1 |
| 2 | Yellow/Black | Coil A - Terminal 2 |
| 3 | Brown/White | Coil B - Terminal 1 |
| 4 | Violet/Black | Coil B - Terminal 2 |

**Note:** Verify with a multimeter! Measure resistance between pins:
- Pins 1-2: ~10-20Ω (Coil A)
- Pins 3-4: ~10-20Ω (Coil B)
- Any other combination: Open (infinite)

---

## Files in This Project

| File | Description |
|------|-------------|
| `iac_tester.ino` | Main Arduino sketch with embedded web interface |
| `circuit_diagram.svg` | Visual wiring diagram |
| `README.md` | This documentation |

---

## License

This project is for personal/educational use. Use at your own risk. Working with automotive electrical systems can damage your vehicle if done incorrectly.
