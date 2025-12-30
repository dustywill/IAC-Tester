# IAC Valve Tester

A web-based diagnostic tool for testing and monitoring Idle Air Control (IAC) valves on fuel-injected vehicles. Originally designed for the 1997 Jeep Wrangler 2.5L but adaptable to other vehicles with bipolar stepper motor IAC valves.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-green.svg)

## Overview

This project allows you to:
- **Manually control** an IAC valve (extend/retract) independent of the PCM
- **Monitor PCM signals** to see what commands the engine computer is sending
- **Compare positions** between your commands and PCM commands in real-time
- **Log data** for later analysis
- **Diagnose issues** like stuck valves, wiring problems, or PCM faults

## Features

### Web Interface
- **Three-tab layout**: Control, WiFi, Hardware
- **Real-time I/O status** with visual LED indicators
- **Position tracking chart** showing your commands vs PCM response
- **Direction detection** displaying EXTEND/RETRACT/IDLE states
- **High-speed signal logging** with CSV download capability
- **Mobile-friendly** responsive design

### Motor Control
- **Continuous movement** buttons (hold to move)
- **Step buttons** for precise positioning (-100, -50, -10, +10, +50, +100)
- **Safety timeout** (30 seconds max continuous movement)
- **Configurable step timing** (default 20ms per step)

### High-Speed PCM Signal Logging
- **Interrupt-driven capture** - catches every state change (not polling)
- **Microsecond resolution** - accurate timing for signal analysis
- **10,000 entry circular buffer** - holds ~80 seconds of active stepping
- **Auto-overwrite** - oldest data replaced when buffer is full
- **Buffer status display** - shows fill level and overflow status
- **Full log download** - export complete buffer as CSV for analysis

### WiFi Connectivity
- **Access Point mode** - Creates its own network (default)
- **Client mode** - Connects to your existing WiFi
- **Network scanner** - Find and select available networks
- **mDNS support** - Access via http://iac-tester.local
- **Automatic fallback** - Returns to AP mode if connection fails

### Hardware Configuration
- **Web-based pin configuration** - No recompilation needed
- **Auto-configure feature** - Automatically detects input pin mappings
- **NeoPixel RGB LED support** - Color-coded status for ESP32-S3 boards
- **Settings persist** in non-volatile storage (NVS)

### OTA Updates
- **Over-the-Air firmware updates** via Arduino IDE
- **No USB connection required** after initial upload
- **Password protected** (default: `iac1997`)

## Hardware Requirements

### Components

| Component | Description | Quantity |
|-----------|-------------|----------|
| ESP32-S3-WROOM-1 | Main microcontroller | 1 |
| TB6612FNG | Dual H-bridge motor driver | 1 |
| PC817 8-Channel | Optocoupler isolation board | 1 |
| LM2596 | DC-DC buck converter (12V to 5V) | 1 |
| 10uF Capacitor | Electrolytic, 16V+ rated | 1 |
| Jumper wires | For connections | As needed |
| [Idle Air Control Valve Connector](https://www.autozone.com/p/duralast-idle-air-control-valve-connector-217/342417) | To connect to drive the IAC Valve | 1 |
| Cheap IAC Valve | You can't find the other side of the connector | 1 |

### Why These Components?

- **ESP32-S3**: Built-in WiFi, plenty of GPIO, USB-C, OTA capable
- **TB6612FNG**: Handles bipolar stepper motor driving, 3.3V logic compatible
- **PC817 Optocoupler**: Galvanic isolation protects ESP32 from automotive voltages
- **LM2596**: Efficiently converts 12V vehicle power to 5V for ESP32
- **Duralass Connector**: I bought the Duralast connector because i was tired of waiting for th thing to be done and diagnosed. You can find them much cheaper on amazon or other online retailers.
- **Cheap IAC Valve**: You cannot find the mating connector for the IAC side of the Connector to connect the PCM to, so I had to butcher an IAC valve and use that IAC as the connector.

## Wiring

See `circuit_diagram.svg` for the complete wiring diagram.

### The Cheap IAC
I sanded off the back side fof he 90 degree connector and found that there were some pins that were molded ointot the tback of the IAC valve, so I was able to solder directly to those. This allowed me to connect the PCM to the cheap IAC and be abebleo see the cheap IAC move as well as tap that signal from the PCM and record those values in ESP32 tester's memory for download.

### Default Pin Assignments

#### Motor Outputs (ESP32 to TB6612)

| Signal | GPIO | Description |
|--------|------|-------------|
| AIN1 | 7 | Coil A positive |
| AIN2 | 15 | Coil A negative |
| BIN1 | 6 | Coil B positive |
| BIN2 | 5 | Coil B negative |
| STBY | 18 | Standby/Enable |

#### PCM Inputs (PC817 to ESP32)

| Signal | GPIO | Description |
|--------|------|-------------|
| PCM_A1 | 16 | PCM Coil A+ monitor |
| PCM_A2 | 17 | PCM Coil A- monitor |
| PCM_B1 | 8 | PCM Coil B+ monitor |
| PCM_B2 | 3 | PCM Coil B- monitor |

#### Other

| Signal | GPIO | Description |
|--------|------|-------------|
| Status LED | 48 | WiFi status indicator (NeoPixel RGB) |

### Critical Connections

1. **TB6612 PWMA and PWMB** must be connected to **3.3V** to enable outputs
2. **TB6612 VCC** connects to **3.3V** (logic power)
3. **TB6612 VM** connects to **12V** (motor power)
4. **10uF capacitor** between VM and GND (smooths motor noise)
5. **PC817 jumpers** must be installed on channels 1-4

## Software Setup

### Prerequisites

1. [Arduino IDE](https://www.arduino.cc/en/software) (2.0+ recommended)
2. ESP32 board support package
3. Required libraries (installed via Library Manager):
   - ArduinoJson
   - Adafruit NeoPixel (for RGB status LED on most ESP32-S3 boards)

### Installing ESP32 Board Support

1. Open Arduino IDE
2. Go to **File > Preferences**
3. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search for "esp32" and install "ESP32 by Espressif Systems"

### Board Settings

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** |
| USB Mode | Hardware CDC and JTAG |
| Upload Speed | 921600 |
| Flash Size | 4MB (or your board's size) |

### Upload

1. Connect ESP32 via USB
2. Select the correct COM port
3. Click Upload
4. Open Serial Monitor at 115200 baud to verify

## Usage

### First Boot

1. After uploading, the ESP32 creates a WiFi network:
   - **SSID**: `IAC_Tester`
   - **Password**: `jeep1997`
2. Connect your phone/computer to this network
3. Open a browser to `http://192.168.4.1` or `http://iac-tester.local`

### Web Interface Tabs

#### Control Tab
- **Extend/Retract buttons** - Hold to move continuously
- **Step buttons** - Click for precise movement
- **I/O Status** - Real-time GPIO states with LED indicators
- **Status LED indicator** - Shows WiFi status LED state in UI
- **Position Chart** - Visual comparison of your commands vs PCM
- **Direction Display** - Shows current movement direction
- **High-Speed Signal Log** - Interrupt-driven capture of PCM signals
  - Buffer status (entries / capacity)
  - Overflow indicator
  - Download Full Log (CSV) - Complete buffer download
  - Clear Buffer - Reset the circular buffer
  - Live event display with pause option

#### WiFi Tab
- **Current Connection** - Shows active mode, IP, hostname
- **WiFi Mode** - Switch between AP and Client modes
- **Network Scanner** - Find available networks (Client mode)
- **Hostname** - Set mDNS name (default: iac-tester)

#### Hardware Tab
- **Auto-Configure Pins** - Detects input pin mappings automatically
- **Pin Configuration** - Manually set all GPIO assignments
- **Status LED Pin** - Configure LED for your specific board
- **Reset to Defaults** - Restore factory pin settings

### Auto-Configure Feature

With the system wired in loopback mode (TB6612 outputs connected to PC817 inputs):

1. Go to the Hardware tab
2. Click "Auto-Configure Input Pins"
3. The system will:
   - Test each output pin one at a time
   - Detect which input pin responds
   - Update the dropdown selections
4. Click "Save Pins & Restart" to apply

### LED Status Patterns

| Pattern | Color | Meaning |
|---------|-------|---------|
| Fast blink (5Hz) | Orange | Connecting to WiFi |
| Slow blink (0.5Hz) | Blue | AP Mode active |
| Solid | Green | Connected to WiFi |
| Fast blink | Purple | OTA update in progress |

**Note:** Most ESP32-S3 boards have a WS2812 RGB LED on GPIO48. The code auto-detects and uses the NeoPixel library for these LEDs.

## OTA Updates

After the initial USB upload, you can update firmware over WiFi:

1. Ensure ESP32 and computer are on the same network
2. In Arduino IDE, go to **Tools > Port**
3. Select the network port: `iac-tester at 192.168.x.x`
4. Upload as normal
5. Enter OTA password when prompted: `iac1997`

**Note**: OTA updates only work in Client mode (connected to your network), not in AP mode.

## Configuration

### Changing Defaults

Edit these values in `iac_tester.ino` before uploading:

```cpp
// WiFi defaults
#define DEFAULT_AP_SSID "IAC_Tester"
#define DEFAULT_AP_PASSWORD "jeep1997"
#define DEFAULT_HOSTNAME "iac-tester"

// OTA password (in setupWifi function)
ArduinoOTA.setPassword("iac1997");

// Step timing
int stepDelayMs = 20;  // milliseconds per step
```

### Pin Configuration

Pins can be changed via the web interface without recompiling. Settings are stored in non-volatile memory and persist across reboots.

## Troubleshooting

### No Serial Output

1. Check **Tools > USB CDC On Boot** is set to **Enabled**
2. Re-upload the sketch after changing this setting
3. Try the other USB port if your board has two

### Can't Find WiFi Network

1. Wait 10-15 seconds after power-on
2. Check Serial Monitor for startup messages
3. Verify the LED is blinking (slow = AP mode active)

### Motor Not Moving

1. Check TB6612 **STBY** pin is going HIGH (view I/O Status)
2. Verify **PWMA** and **PWMB** are connected to 3.3V
3. Confirm **VM** has 12V power
4. Check motor coil connections (A1/A2, B1/B2)

### PCM Inputs Not Responding

1. Verify PC817 jumpers are installed
2. Check that IN- pins are connected to ground
3. Run Auto-Configure to detect pin mappings
4. View I/O Status to see raw GPIO states

### OTA Upload Fails

1. Ensure both devices are on the same network
2. Verify ESP32 is in Client mode (not AP mode)
3. Check password is correct (`iac1997`)
4. Try disabling firewall temporarily

## Vehicle-Specific Notes

### 1997 Jeep Wrangler 2.5L (TJ)

- IAC valve is a 4-wire bipolar stepper motor wwith 90 degree connector
- Located on the throttle body
- PCM connector pinout: A1, A2, B1, B2
- Normal idle position: ~20-40 steps from closed
- Full range: approximately 125 steps
- Normal warm idle: 650-750 RPM
- PCM step timing: ~8ms per step when actively adjusting

#### Common High Idle Causes (Diagnosed with this tool)

| Symptom | Likely Cause |
|---------|--------------|
| PCM rapidly stepping when RPM drops | PCM closed-loop working correctly |
| PCM slow to respond to RPM changes | Possible sensor issue |
| TPS showing >5% at closed throttle | Throttle stop screw maladjusted |
| High idle with normal sensor readings | Check throttle body idle screw |

#### Diagnostic Approach

1. **Monitor PCM signals** at idle - verify PCM is sending step commands
2. **Manually move IAC** with tester - observe if RPM changes
3. **Watch PCM response** - does it try to compensate?
4. **Check OBD-II data** - ECT, TPS, fuel trims, timing
5. **Download high-speed log** - analyze PCM stepping patterns

### Adapting to Other Vehicles

This project works with any vehicle using a bipolar stepper IAC valve. You may need to:

1. Identify the IAC connector pinout for your vehicle
2. Adjust step timing if motor behaves erratically
3. Modify step count expectations based on your IAC's range

## Analyzing Log Data

The high-speed log captures every PCM signal transition with millisecond timestamps. Here's how to interpret the data:

### CSV Format

```csv
timestamp,pcmA1,pcmA2,pcmB1,pcmB2,iacActive
82674,1,1,1,1,0
82697,1,1,0,0,0
82724,0,1,0,0,0
```

| Column | Description |
|--------|-------------|
| timestamp | Milliseconds since boot |
| pcmA1-B2 | PCM coil states (1=energized, 0=off) |
| iacActive | 1 if YOU were commanding the motor |

### Step Patterns

Valid bipolar stepper patterns:

| Step | A1 | A2 | B1 | B2 | Binary |
|------|----|----|----|----|--------|
| 0 | 1 | 0 | 1 | 0 | 5 |
| 1 | 0 | 1 | 1 | 0 | 6 |
| 2 | 0 | 1 | 0 | 1 | 10 |
| 3 | 1 | 0 | 0 | 1 | 9 |

Pattern `1,1,1,1` (all HIGH) = holding/brake position

### What to Look For

| Observation | Interpretation |
|-------------|----------------|
| Fast stepping (~8ms intervals) | PCM actively adjusting |
| Slow stepping (200-800ms) | PCM making fine corrections |
| No stepping | PCM holding position |
| Invalid patterns | Possible wiring issue |
| Same timestamp entries | Catching transition edges (normal) |

### Calculating Step Rate

```
Steps per second = 1000 / average_interval_ms
Example: 8ms intervals = 125 steps/second
```

## Project Structure

```
iac_tester/
├── iac_tester.ino       # Main Arduino sketch
├── circuit_diagram.svg  # Wiring diagram
├── README.md            # This file
├── LICENSE              # MIT License
└── preview.html         # Standalone web interface preview
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is released under the MIT License. See LICENSE file for details.

```
MIT License

Copyright (c) 2025 Byron

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Acknowledgments

- Inspired by the need to diagnose a stubborn high-idle condition on a 1997 Jeep Wrangler
- Successfully used to identify a maladjusted throttle stop screw causing 14% TPS at idle
- Thanks to the ESP32 and Arduino communities for excellent documentation
- Built with assistance from Claude (Anthropic)

## Disclaimer

This tool is intended for diagnostic purposes. Use at your own risk. The authors are not responsible for any damage to vehicles, components, or persons resulting from the use of this project. Always follow proper safety procedures when working with automotive electrical systems.

---

**Questions or Issues?** Open an issue on GitHub or check the troubleshooting section above.
