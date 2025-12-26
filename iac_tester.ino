/*
 * IAC Valve Tester for 1997 Jeep Wrangler 2.5L
 * 
 * Controls a bipolar stepper motor (IAC valve) via TB6612FNG driver
 * Monitors PCM signals via PC817 optocoupler isolation
 * Provides web interface for control and logging
 * 
 * Hardware:
 *   - ESP32 DevKitC (or similar)
 *   - TB6612FNG Dual Motor Driver
 *   - PC817 8-Channel Optocoupler Board
 *   - LM2596 Buck Converter (12V -> 5V)
 * 
 * Author: Byron
 * Date: 2025
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================
// CONFIGURATION - Update these for your setup
// ============================================

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Set to true to create an Access Point instead of connecting to WiFi
const bool AP_MODE = true;
const char* AP_SSID = "IAC_Tester";
const char* AP_PASSWORD = "jeep1997";

// ============================================
// PIN DEFINITIONS
// ============================================

// TB6612FNG Motor Driver Pins (directly controlling IAC stepper)
const int PIN_AIN1 = 25;  // Coil A control 1
const int PIN_AIN2 = 26;  // Coil A control 2
const int PIN_BIN1 = 27;  // Coil B control 1
const int PIN_BIN2 = 14;  // Coil B control 2
const int PIN_STBY = 13;  // Standby (HIGH = enabled)

// PC817 Optocoupler Inputs (monitoring PCM signals)
// Note: Optocoupler inverts signal - HIGH on PCM = LOW on ESP32
const int PIN_PCM_A1 = 32;  // PCM Coil A pin 1 monitor
const int PIN_PCM_A2 = 33;  // PCM Coil A pin 2 monitor
const int PIN_PCM_B1 = 34;  // PCM Coil B pin 1 monitor
const int PIN_PCM_B2 = 35;  // PCM Coil B pin 2 monitor

// ============================================
// STEPPER MOTOR CONFIGURATION
// ============================================

// Full step sequence for bipolar stepper (4 steps per cycle)
// Each row: {AIN1, AIN2, BIN1, BIN2}
const int STEP_SEQUENCE[4][4] = {
  {HIGH, LOW,  HIGH, LOW },  // Step 0
  {LOW,  HIGH, HIGH, LOW },  // Step 1
  {LOW,  HIGH, LOW,  HIGH},  // Step 2
  {HIGH, LOW,  LOW,  HIGH}   // Step 3
};

// Current step position (0-3)
volatile int currentStep = 0;

// Stepper settings (adjustable via web interface)
volatile int stepDelayMs = 20;        // Delay between steps (ms)
volatile int targetSteps = 0;         // Steps remaining to move
volatile int stepDirection = 1;       // 1 = extend, -1 = retract
volatile bool motorEnabled = false;   // Motor driver enabled state
volatile bool isMoving = false;       // Currently executing movement

// ============================================
// PCM SIGNAL LOGGING
// ============================================

struct SignalLog {
  unsigned long timestamp;
  uint8_t pcmA1 : 1;
  uint8_t pcmA2 : 1;
  uint8_t pcmB1 : 1;
  uint8_t pcmB2 : 1;
  uint8_t iacActive : 1;  // Was IAC button pressed at this time?
};

const int LOG_BUFFER_SIZE = 500;  // Store last 500 signal changes
SignalLog signalLog[LOG_BUFFER_SIZE];
volatile int logIndex = 0;
volatile int logCount = 0;

// Previous PCM state for change detection
volatile uint8_t prevPcmState = 0;

// ============================================
// WEB SERVER
// ============================================

WebServer server(80);

// ============================================
// HTML/JS/CSS - Embedded Web Interface
// ============================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>IAC Valve Tester</title>
  <style>
    :root {
      --bg-primary: #0a0e14;
      --bg-secondary: #141a22;
      --bg-tertiary: #1c242e;
      --accent-orange: #e85d04;
      --accent-orange-dim: #9d3c02;
      --accent-green: #22c55e;
      --accent-red: #ef4444;
      --text-primary: #e6e6e6;
      --text-secondary: #8b949e;
      --text-dim: #484f58;
      --border-color: #2d3640;
    }
    
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Courier New', Courier, monospace;
      background: var(--bg-primary);
      color: var(--text-primary);
      min-height: 100vh;
      padding: 20px;
    }
    
    .container {
      max-width: 900px;
      margin: 0 auto;
    }
    
    header {
      text-align: center;
      margin-bottom: 30px;
      padding-bottom: 20px;
      border-bottom: 2px solid var(--accent-orange);
    }
    
    h1 {
      font-size: 1.8rem;
      color: var(--accent-orange);
      text-transform: uppercase;
      letter-spacing: 3px;
      margin-bottom: 5px;
    }
    
    .subtitle {
      color: var(--text-secondary);
      font-size: 0.9rem;
    }
    
    .panel {
      background: var(--bg-secondary);
      border: 1px solid var(--border-color);
      border-radius: 8px;
      padding: 20px;
      margin-bottom: 20px;
    }
    
    .panel-title {
      font-size: 1rem;
      color: var(--accent-orange);
      text-transform: uppercase;
      letter-spacing: 2px;
      margin-bottom: 15px;
      padding-bottom: 10px;
      border-bottom: 1px solid var(--border-color);
    }
    
    .control-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 15px;
    }
    
    .btn {
      background: var(--bg-tertiary);
      border: 2px solid var(--border-color);
      color: var(--text-primary);
      padding: 15px 25px;
      font-family: inherit;
      font-size: 1rem;
      cursor: pointer;
      border-radius: 6px;
      transition: all 0.15s ease;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    
    .btn:hover {
      border-color: var(--accent-orange);
      background: var(--bg-secondary);
    }
    
    .btn:active, .btn.active {
      background: var(--accent-orange);
      border-color: var(--accent-orange);
      color: var(--bg-primary);
    }
    
    .btn-extend { border-left: 4px solid var(--accent-green); }
    .btn-retract { border-left: 4px solid var(--accent-red); }
    .btn-stop { border-left: 4px solid var(--accent-orange); }
    
    .settings-row {
      display: flex;
      align-items: center;
      margin-bottom: 12px;
    }
    
    .settings-row label {
      flex: 0 0 140px;
      color: var(--text-secondary);
    }
    
    .settings-row input {
      flex: 1;
      background: var(--bg-tertiary);
      border: 1px solid var(--border-color);
      color: var(--text-primary);
      padding: 10px;
      font-family: inherit;
      font-size: 1rem;
      border-radius: 4px;
      max-width: 150px;
    }
    
    .settings-row input:focus {
      outline: none;
      border-color: var(--accent-orange);
    }
    
    .settings-row .unit {
      margin-left: 10px;
      color: var(--text-dim);
    }
    
    .status-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
      text-align: center;
    }
    
    .signal-indicator {
      background: var(--bg-tertiary);
      padding: 15px;
      border-radius: 6px;
      border: 1px solid var(--border-color);
    }
    
    .signal-indicator .label {
      font-size: 0.75rem;
      color: var(--text-secondary);
      margin-bottom: 8px;
    }
    
    .signal-indicator .led {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      margin: 0 auto;
      background: var(--text-dim);
      box-shadow: inset 0 2px 4px rgba(0,0,0,0.3);
      transition: all 0.1s ease;
    }
    
    .signal-indicator .led.on {
      background: var(--accent-green);
      box-shadow: 0 0 15px var(--accent-green), inset 0 2px 4px rgba(255,255,255,0.2);
    }
    
    .log-container {
      background: var(--bg-tertiary);
      border: 1px solid var(--border-color);
      border-radius: 6px;
      height: 250px;
      overflow-y: auto;
      padding: 10px;
      font-size: 0.8rem;
    }
    
    .log-entry {
      display: flex;
      gap: 15px;
      padding: 4px 0;
      border-bottom: 1px solid var(--border-color);
    }
    
    .log-entry .time {
      color: var(--text-dim);
      flex: 0 0 80px;
    }
    
    .log-entry .signals {
      color: var(--text-secondary);
      flex: 0 0 100px;
    }
    
    .log-entry .event {
      color: var(--text-primary);
    }
    
    .log-entry.iac-active {
      background: rgba(232, 93, 4, 0.1);
    }
    
    .log-entry.iac-active .event {
      color: var(--accent-orange);
    }
    
    .log-controls {
      display: flex;
      gap: 10px;
      margin-top: 10px;
    }
    
    .log-controls .btn {
      padding: 10px 20px;
      font-size: 0.85rem;
    }
    
    .motor-status {
      display: flex;
      align-items: center;
      gap: 15px;
      padding: 15px;
      background: var(--bg-tertiary);
      border-radius: 6px;
      margin-top: 15px;
    }
    
    .motor-status .indicator {
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: var(--text-dim);
    }
    
    .motor-status .indicator.enabled {
      background: var(--accent-green);
      box-shadow: 0 0 10px var(--accent-green);
    }
    
    .motor-status .indicator.moving {
      background: var(--accent-orange);
      box-shadow: 0 0 10px var(--accent-orange);
      animation: pulse 0.5s ease-in-out infinite;
    }
    
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    
    .quick-steps {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 15px;
    }
    
    .quick-steps .btn {
      padding: 10px 15px;
      font-size: 0.85rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>IAC Valve Tester</h1>
      <div class="subtitle">1997 Jeep Wrangler 2.5L</div>
    </header>
    
    <!-- Motor Control Panel -->
    <div class="panel">
      <div class="panel-title">Motor Control</div>
      <div class="control-grid">
        <button class="btn btn-extend" id="btnExtend" onmousedown="startMove('extend')" onmouseup="stopMove()" onmouseleave="stopMove()">
          ▲ Extend
        </button>
        <button class="btn btn-retract" id="btnRetract" onmousedown="startMove('retract')" onmouseup="stopMove()" onmouseleave="stopMove()">
          ▼ Retract
        </button>
        <button class="btn btn-stop" onclick="emergencyStop()">
          ■ Stop
        </button>
      </div>
      
      <div class="quick-steps">
        <span style="color: var(--text-secondary); line-height: 38px;">Quick:</span>
        <button class="btn" onclick="moveSteps(10, 1)">+10</button>
        <button class="btn" onclick="moveSteps(50, 1)">+50</button>
        <button class="btn" onclick="moveSteps(100, 1)">+100</button>
        <button class="btn" onclick="moveSteps(10, -1)">-10</button>
        <button class="btn" onclick="moveSteps(50, -1)">-50</button>
        <button class="btn" onclick="moveSteps(100, -1)">-100</button>
      </div>
      
      <div class="motor-status">
        <div class="indicator" id="motorIndicator"></div>
        <span id="motorStatusText">Motor: Idle</span>
      </div>
    </div>
    
    <!-- Settings Panel -->
    <div class="panel">
      <div class="panel-title">Stepper Settings</div>
      <div class="settings-row">
        <label>Step Delay:</label>
        <input type="number" id="stepDelay" value="20" min="1" max="500" onchange="updateSettings()">
        <span class="unit">ms</span>
      </div>
      <div class="settings-row">
        <label>Frequency:</label>
        <span id="frequencyDisplay" style="color: var(--text-primary);">50.0 Hz</span>
      </div>
    </div>
    
    <!-- PCM Signal Monitor -->
    <div class="panel">
      <div class="panel-title">PCM Signal Monitor</div>
      <div class="status-grid">
        <div class="signal-indicator">
          <div class="label">A1</div>
          <div class="led" id="ledA1"></div>
        </div>
        <div class="signal-indicator">
          <div class="label">A2</div>
          <div class="led" id="ledA2"></div>
        </div>
        <div class="signal-indicator">
          <div class="label">B1</div>
          <div class="led" id="ledB1"></div>
        </div>
        <div class="signal-indicator">
          <div class="label">B2</div>
          <div class="led" id="ledB2"></div>
        </div>
      </div>
    </div>
    
    <!-- Signal Log Panel -->
    <div class="panel">
      <div class="panel-title">Signal Log</div>
      <div class="log-container" id="logContainer">
        <div class="log-entry">
          <span class="time">--:--:--</span>
          <span class="signals">----</span>
          <span class="event">Waiting for signals...</span>
        </div>
      </div>
      <div class="log-controls">
        <button class="btn" onclick="clearLog()">Clear Log</button>
        <button class="btn" onclick="downloadLog()">Download CSV</button>
        <button class="btn" id="btnPauseLog" onclick="toggleLogPause()">Pause</button>
      </div>
    </div>
  </div>
  
  <script>
    // State
    let isLogPaused = false;
    let logData = [];
    let pollInterval = null;
    let iacButtonActive = false;
    
    // Initialize
    document.addEventListener('DOMContentLoaded', function() {
      startPolling();
      loadSettings();
    });
    
    // Polling for status updates
    function startPolling() {
      pollInterval = setInterval(fetchStatus, 100);
    }
    
    function fetchStatus() {
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          updateSignalLeds(data.pcm);
          updateMotorStatus(data.motor);
          if (!isLogPaused && data.log && data.log.length > 0) {
            appendLogEntries(data.log);
          }
        })
        .catch(err => console.error('Status fetch error:', err));
    }
    
    function updateSignalLeds(pcm) {
      // Note: Optocoupler inverts, so we show the logical PCM state
      document.getElementById('ledA1').className = 'led' + (pcm.a1 ? ' on' : '');
      document.getElementById('ledA2').className = 'led' + (pcm.a2 ? ' on' : '');
      document.getElementById('ledB1').className = 'led' + (pcm.b1 ? ' on' : '');
      document.getElementById('ledB2').className = 'led' + (pcm.b2 ? ' on' : '');
    }
    
    function updateMotorStatus(motor) {
      const indicator = document.getElementById('motorIndicator');
      const text = document.getElementById('motorStatusText');
      
      if (motor.moving) {
        indicator.className = 'indicator moving';
        text.textContent = 'Motor: Moving (' + (motor.direction > 0 ? 'Extend' : 'Retract') + ')';
      } else if (motor.enabled) {
        indicator.className = 'indicator enabled';
        text.textContent = 'Motor: Ready';
      } else {
        indicator.className = 'indicator';
        text.textContent = 'Motor: Idle';
      }
    }
    
    function appendLogEntries(entries) {
      const container = document.getElementById('logContainer');
      
      entries.forEach(entry => {
        const div = document.createElement('div');
        div.className = 'log-entry' + (entry.iac ? ' iac-active' : '');
        
        const time = new Date(entry.ts).toLocaleTimeString();
        const signals = (entry.a1?'1':'0') + (entry.a2?'1':'0') + (entry.b1?'1':'0') + (entry.b2?'1':'0');
        const event = entry.iac ? 'IAC Active' : 'PCM Change';
        
        div.innerHTML = 
          '<span class="time">' + time + '</span>' +
          '<span class="signals">' + signals + '</span>' +
          '<span class="event">' + event + '</span>';
        
        container.appendChild(div);
        logData.push(entry);
      });
      
      // Auto-scroll to bottom
      container.scrollTop = container.scrollHeight;
      
      // Limit displayed entries
      while (container.children.length > 200) {
        container.removeChild(container.firstChild);
      }
    }
    
    // Motor control functions
    function startMove(direction) {
      iacButtonActive = true;
      const dir = direction === 'extend' ? 1 : -1;
      fetch('/api/move?continuous=1&direction=' + dir, { method: 'POST' });
      document.getElementById(direction === 'extend' ? 'btnExtend' : 'btnRetract').classList.add('active');
    }
    
    function stopMove() {
      iacButtonActive = false;
      fetch('/api/stop', { method: 'POST' });
      document.getElementById('btnExtend').classList.remove('active');
      document.getElementById('btnRetract').classList.remove('active');
    }
    
    function moveSteps(steps, direction) {
      fetch('/api/move?steps=' + steps + '&direction=' + direction, { method: 'POST' });
    }
    
    function emergencyStop() {
      fetch('/api/stop?emergency=1', { method: 'POST' });
    }
    
    // Settings
    function updateSettings() {
      const delay = document.getElementById('stepDelay').value;
      const freq = (1000 / delay).toFixed(1);
      document.getElementById('frequencyDisplay').textContent = freq + ' Hz';
      
      fetch('/api/settings?delay=' + delay, { method: 'POST' });
    }
    
    function loadSettings() {
      fetch('/api/settings')
        .then(r => r.json())
        .then(data => {
          document.getElementById('stepDelay').value = data.delay;
          updateSettings();
        });
    }
    
    // Log controls
    function clearLog() {
      document.getElementById('logContainer').innerHTML = '';
      logData = [];
      fetch('/api/log/clear', { method: 'POST' });
    }
    
    function toggleLogPause() {
      isLogPaused = !isLogPaused;
      document.getElementById('btnPauseLog').textContent = isLogPaused ? 'Resume' : 'Pause';
      document.getElementById('btnPauseLog').classList.toggle('active', isLogPaused);
    }
    
    function downloadLog() {
      let csv = 'Timestamp,A1,A2,B1,B2,IAC_Active\n';
      logData.forEach(entry => {
        csv += entry.ts + ',' + 
               (entry.a1?1:0) + ',' + 
               (entry.a2?1:0) + ',' + 
               (entry.b1?1:0) + ',' + 
               (entry.b2?1:0) + ',' + 
               (entry.iac?1:0) + '\n';
      });
      
      const blob = new Blob([csv], { type: 'text/csv' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'iac_log_' + Date.now() + '.csv';
      a.click();
      URL.revokeObjectURL(url);
    }
  </script>
</body>
</html>
)rawliteral";

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nIAC Valve Tester Starting...");
  
  // Initialize motor driver pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  
  // Start with motor disabled
  digitalWrite(PIN_STBY, LOW);
  setMotorOutputs(LOW, LOW, LOW, LOW);
  
  // Initialize PCM monitor pins (optocoupler outputs)
  pinMode(PIN_PCM_A1, INPUT_PULLUP);
  pinMode(PIN_PCM_A2, INPUT_PULLUP);
  pinMode(PIN_PCM_B1, INPUT_PULLUP);
  pinMode(PIN_PCM_B2, INPUT_PULLUP);
  
  // Initialize WiFi
  if (AP_MODE) {
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("Access Point started. IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print("\nConnected! IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/move", handleMove);
  server.on("/api/stop", handleStop);
  server.on("/api/settings", handleSettings);
  server.on("/api/log/clear", handleLogClear);
  
  server.begin();
  Serial.println("Web server started");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  server.handleClient();
  
  // Monitor PCM signals for changes
  monitorPcmSignals();
  
  // Execute stepper movement if needed
  if (isMoving && targetSteps != 0) {
    executeStep();
    targetSteps += (stepDirection > 0) ? -1 : 1;
    
    if (targetSteps == 0) {
      isMoving = false;
    }
    
    delay(stepDelayMs);
  } else if (isMoving && targetSteps == 0) {
    // Continuous mode - keep moving
    executeStep();
    delay(stepDelayMs);
  }
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// ============================================

void setMotorOutputs(int a1, int a2, int b1, int b2) {
  digitalWrite(PIN_AIN1, a1);
  digitalWrite(PIN_AIN2, a2);
  digitalWrite(PIN_BIN1, b1);
  digitalWrite(PIN_BIN2, b2);
}

void executeStep() {
  // Update step position
  currentStep += stepDirection;
  if (currentStep > 3) currentStep = 0;
  if (currentStep < 0) currentStep = 3;
  
  // Apply step sequence
  setMotorOutputs(
    STEP_SEQUENCE[currentStep][0],
    STEP_SEQUENCE[currentStep][1],
    STEP_SEQUENCE[currentStep][2],
    STEP_SEQUENCE[currentStep][3]
  );
}

void enableMotor() {
  digitalWrite(PIN_STBY, HIGH);
  motorEnabled = true;
}

void disableMotor() {
  digitalWrite(PIN_STBY, LOW);
  setMotorOutputs(LOW, LOW, LOW, LOW);
  motorEnabled = false;
  isMoving = false;
  targetSteps = 0;
}

// ============================================
// PCM SIGNAL MONITORING
// ============================================

void monitorPcmSignals() {
  // Read current state (inverted due to optocoupler)
  uint8_t currentState = 0;
  currentState |= (!digitalRead(PIN_PCM_A1)) << 0;
  currentState |= (!digitalRead(PIN_PCM_A2)) << 1;
  currentState |= (!digitalRead(PIN_PCM_B1)) << 2;
  currentState |= (!digitalRead(PIN_PCM_B2)) << 3;
  
  // Check for change
  if (currentState != prevPcmState) {
    // Log the change
    signalLog[logIndex].timestamp = millis();
    signalLog[logIndex].pcmA1 = (currentState >> 0) & 1;
    signalLog[logIndex].pcmA2 = (currentState >> 1) & 1;
    signalLog[logIndex].pcmB1 = (currentState >> 2) & 1;
    signalLog[logIndex].pcmB2 = (currentState >> 3) & 1;
    signalLog[logIndex].iacActive = isMoving ? 1 : 0;
    
    logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
    if (logCount < LOG_BUFFER_SIZE) logCount++;
    
    prevPcmState = currentState;
  }
}

// ============================================
// WEB SERVER HANDLERS
// ============================================

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  StaticJsonDocument<512> doc;
  
  // PCM signals (inverted from optocoupler)
  JsonObject pcm = doc.createNestedObject("pcm");
  pcm["a1"] = !digitalRead(PIN_PCM_A1);
  pcm["a2"] = !digitalRead(PIN_PCM_A2);
  pcm["b1"] = !digitalRead(PIN_PCM_B1);
  pcm["b2"] = !digitalRead(PIN_PCM_B2);
  
  // Motor status
  JsonObject motor = doc.createNestedObject("motor");
  motor["enabled"] = motorEnabled;
  motor["moving"] = isMoving;
  motor["direction"] = stepDirection;
  motor["step"] = currentStep;
  
  // Recent log entries (send up to 10 new entries)
  JsonArray log = doc.createNestedArray("log");
  static int lastSentIndex = 0;
  int entriesToSend = 0;
  
  while (lastSentIndex != logIndex && entriesToSend < 10) {
    JsonObject entry = log.createNestedObject();
    entry["ts"] = signalLog[lastSentIndex].timestamp;
    entry["a1"] = signalLog[lastSentIndex].pcmA1;
    entry["a2"] = signalLog[lastSentIndex].pcmA2;
    entry["b1"] = signalLog[lastSentIndex].pcmB1;
    entry["b2"] = signalLog[lastSentIndex].pcmB2;
    entry["iac"] = signalLog[lastSentIndex].iacActive;
    
    lastSentIndex = (lastSentIndex + 1) % LOG_BUFFER_SIZE;
    entriesToSend++;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleMove() {
  enableMotor();
  
  if (server.hasArg("continuous")) {
    // Continuous movement mode (hold button)
    stepDirection = server.arg("direction").toInt();
    targetSteps = 0;  // 0 = continuous
    isMoving = true;
  } else if (server.hasArg("steps")) {
    // Fixed step count mode
    targetSteps = server.arg("steps").toInt();
    stepDirection = server.arg("direction").toInt();
    isMoving = true;
  }
  
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  if (server.hasArg("emergency")) {
    disableMotor();
  } else {
    isMoving = false;
    targetSteps = 0;
  }
  
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSettings() {
  if (server.method() == HTTP_POST) {
    if (server.hasArg("delay")) {
      stepDelayMs = server.arg("delay").toInt();
      if (stepDelayMs < 1) stepDelayMs = 1;
      if (stepDelayMs > 500) stepDelayMs = 500;
    }
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    String response = "{\"delay\":" + String(stepDelayMs) + "}";
    server.send(200, "application/json", response);
  }
}

void handleLogClear() {
  logIndex = 0;
  logCount = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}
