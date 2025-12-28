/*
 * IAC Valve Tester for 1997 Jeep Wrangler 2.5L
 * 
 * Controls a bipolar stepper motor (IAC valve) via TB6612FNG driver
 * Monitors PCM signals via PC817 optocoupler isolation
 * Provides web interface for control and logging
 * WiFi settings stored in non-volatile memory
 * 
 * Hardware:
 *   - ESP32-S3-WROOM-1
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
#include <Preferences.h>
#include <ESPmDNS.h>

// ============================================
// FORWARD DECLARATIONS
// ============================================
void setMotorOutputs(int a1, int a2, int b1, int b2);
void executeStep();
void enableMotor();
void disableMotor();
void monitorPcmSignals();
void handleRoot();
void handleStatus();
void handleMove();
void handleStop();
void handleSettings();
void handleLogClear();
void handleReset();
void handleWifiSettings();
void handleWifiScan();
void handleWifiSave();
void loadWifiSettings();
void saveWifiSettings();
void setupWifi();

// ============================================
// DEFAULT WIFI CONFIGURATION
// ============================================
#define DEFAULT_AP_MODE true
#define DEFAULT_AP_SSID "IAC_Tester"
#define DEFAULT_AP_PASSWORD "jeep1997"
#define DEFAULT_STA_SSID ""
#define DEFAULT_STA_PASSWORD ""
#define DEFAULT_HOSTNAME "iac-tester"

// WiFi settings (loaded from NVS)
Preferences preferences;
bool wifiApMode = DEFAULT_AP_MODE;
String wifiApSsid = DEFAULT_AP_SSID;
String wifiApPassword = DEFAULT_AP_PASSWORD;
String wifiStaSsid = DEFAULT_STA_SSID;
String wifiStaPassword = DEFAULT_STA_PASSWORD;
String wifiHostname = DEFAULT_HOSTNAME;

// ============================================
// PIN DEFINITIONS - ESP32-S3-WROOM-1
// ============================================

// TB6612FNG Motor Driver Pins
const int PIN_AIN1 = 5;
const int PIN_AIN2 = 6;
const int PIN_BIN1 = 7;
const int PIN_BIN2 = 15;
const int PIN_STBY = 16;

// PC817 Optocoupler Inputs (monitoring PCM signals)
const int PIN_PCM_A1 = 17;
const int PIN_PCM_A2 = 18;
const int PIN_PCM_B1 = 8;
const int PIN_PCM_B2 = 3;

// ============================================
// STEPPER MOTOR CONFIGURATION
// ============================================

const int STEP_SEQUENCE[4][4] = {
  {HIGH, LOW,  HIGH, LOW },
  {LOW,  HIGH, HIGH, LOW },
  {LOW,  HIGH, LOW,  HIGH},
  {HIGH, LOW,  LOW,  HIGH}
};

volatile int currentStep = 0;
volatile int stepDelayMs = 20;
volatile int targetSteps = 0;
volatile int stepDirection = 1;
volatile bool motorEnabled = false;
volatile bool isMoving = false;

// Motor output state tracking (for UI feedback)
volatile unsigned long lastStepTime = 0;
volatile int totalStepsTaken = 0;

// ============================================
// PCM SIGNAL LOGGING
// ============================================

struct SignalLog {
  unsigned long timestamp;
  uint8_t pcmA1 : 1;
  uint8_t pcmA2 : 1;
  uint8_t pcmB1 : 1;
  uint8_t pcmB2 : 1;
  uint8_t iacActive : 1;
};

const int LOG_BUFFER_SIZE = 500;
SignalLog signalLog[LOG_BUFFER_SIZE];
volatile int logIndex = 0;
volatile int logCount = 0;

volatile uint8_t prevPcmState = 0;

// PCM direction detection
volatile int pcmStepIndex = -1;
volatile int prevPcmStepIndex = -1;
volatile int pcmDirection = 0;
volatile unsigned long lastPcmChange = 0;
const unsigned long PCM_IDLE_TIMEOUT = 500;

// Cumulative step position tracking
volatile long youStepPosition = 0;
volatile long pcmStepPosition = 0;

// Step pattern lookup
const int8_t STEP_LOOKUP[16] = {
// Index: 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
        -1, -1, -1, -1, -1,  0,  1, -1, -1,  3,  2, -1, -1, -1, -1, -1
// Pattern 5=Step0, 6=Step1, 9=Step3, 10=Step2
};

// Web server
WebServer server(80);

// ============================================
// HTML PAGE WITH TABS
// ============================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IAC Valve Tester</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #1a1a2e; color: #eee; padding: 15px; }
    h1 { color: #f39c12; margin-bottom: 10px; font-size: 1.5em; }
    h2 { color: #3498db; margin: 15px 0 10px; font-size: 1.2em; }
    
    /* Tab styles */
    .tab-container { display: flex; margin-bottom: 15px; border-bottom: 2px solid #3498db; }
    .tab { padding: 10px 20px; cursor: pointer; background: #16213e; border: none; color: #aaa; font-size: 1em; }
    .tab:hover { background: #1f3460; }
    .tab.active { background: #1f4068; color: #fff; border-bottom: 2px solid #f39c12; margin-bottom: -2px; }
    .tab-content { display: none; }
    .tab-content.active { display: block; }
    
    .card { background: #16213e; border-radius: 8px; padding: 15px; margin-bottom: 15px; }
    .btn { padding: 12px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 1em; margin: 3px; transition: all 0.2s; }
    .btn-extend { background: #27ae60; color: white; }
    .btn-extend:hover { background: #2ecc71; }
    .btn-extend:active, .btn-extend.active { background: #1e8449; box-shadow: inset 0 2px 5px rgba(0,0,0,0.3); }
    .btn-retract { background: #e74c3c; color: white; }
    .btn-retract:hover { background: #ec7063; }
    .btn-retract:active, .btn-retract.active { background: #b03a2e; box-shadow: inset 0 2px 5px rgba(0,0,0,0.3); }
    .btn-stop { background: #f39c12; color: white; }
    .btn-stop:hover { background: #f5b041; }
    .btn-step { background: #3498db; color: white; min-width: 60px; }
    .btn-step:hover { background: #5dade2; }
    .btn-step:active { background: #2171a7; }
    .btn-secondary { background: #7f8c8d; color: white; }
    .btn-secondary:hover { background: #95a5a6; }
    .btn-save { background: #27ae60; color: white; }
    .btn-save:hover { background: #2ecc71; }
    .btn-scan { background: #9b59b6; color: white; }
    .btn-scan:hover { background: #a569bd; }
    
    .motor-controls { display: flex; flex-wrap: wrap; gap: 10px; justify-content: center; }
    .direction-btns { display: flex; gap: 10px; margin-bottom: 10px; }
    .direction-btns .btn { flex: 1; min-width: 120px; padding: 20px; font-size: 1.2em; }
    .step-btns { display: flex; flex-wrap: wrap; gap: 5px; justify-content: center; }
    
    /* LED Indicator Panel */
    .led-panel { background: #0d1117; border-radius: 8px; padding: 15px; margin: 15px 0; }
    .led-panel h3 { color: #8b949e; font-size: 0.9em; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 1px; }
    .led-row { display: flex; align-items: center; padding: 6px 0; border-bottom: 1px solid #1a1a2e; }
    .led-row:last-child { border-bottom: none; }
    .led { width: 16px; height: 16px; border-radius: 50%; margin-right: 12px; border: 2px solid #333; transition: all 0.15s; }
    .led.off { background: #2a2a2a; box-shadow: inset 0 2px 4px rgba(0,0,0,0.5); }
    .led.on-green { background: #22c55e; box-shadow: 0 0 8px #22c55e, 0 0 16px rgba(34,197,94,0.5); border-color: #16a34a; }
    .led.on-red { background: #ef4444; box-shadow: 0 0 8px #ef4444, 0 0 16px rgba(239,68,68,0.5); border-color: #dc2626; }
    .led.on-yellow { background: #eab308; box-shadow: 0 0 8px #eab308, 0 0 16px rgba(234,179,8,0.5); border-color: #ca8a04; }
    .led.on-blue { background: #3b82f6; box-shadow: 0 0 8px #3b82f6, 0 0 16px rgba(59,130,246,0.5); border-color: #2563eb; }
    .led-label { flex: 1; font-family: monospace; font-size: 0.95em; }
    .led-label .pin { color: #f39c12; }
    .led-label .desc { color: #6b7280; font-size: 0.85em; margin-left: 8px; }
    .led-state { font-family: monospace; font-size: 0.85em; width: 40px; text-align: right; }
    .led-state.high { color: #22c55e; }
    .led-state.low { color: #6b7280; }
    .led-sections { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
    @media (max-width: 700px) { .led-sections { grid-template-columns: 1fr; } }
    
    /* Activity indicator */
    .activity-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; background: #555; margin-left: 10px; transition: background 0.1s; }
    .activity-indicator.active { background: #2ecc71; box-shadow: 0 0 10px #2ecc71; }
    
    .settings-row { display: flex; align-items: center; margin: 10px 0; }
    .settings-row label { width: 120px; }
    .settings-row input, .settings-row select { flex: 1; padding: 8px; border-radius: 4px; border: 1px solid #444; background: #1a1a2e; color: #eee; }
    .slider-container { display: flex; align-items: center; gap: 10px; }
    .slider-container input[type="range"] { flex: 1; }
    
    .direction-box { display: inline-block; padding: 10px 15px; border-radius: 5px; margin: 5px; min-width: 120px; text-align: center; }
    .dir-extend { background: #1e8449; }
    .dir-retract { background: #922b21; }
    .dir-idle { background: #555; }
    .direction-display { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; margin: 10px 0; }
    
    .log-area { background: #0a0a15; border-radius: 5px; padding: 10px; max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 0.85em; }
    .log-entry { padding: 2px 0; border-bottom: 1px solid #222; }
    .chart-container { height: 250px; margin: 10px 0; }
    
    .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(100px, 1fr)); gap: 10px; margin: 10px 0; }
    .status-item { text-align: center; padding: 8px; background: #1a1a2e; border-radius: 5px; }
    .status-item .label { font-size: 0.75em; color: #888; }
    .status-item .value { font-size: 1.1em; font-weight: bold; }
    .status-item .value.positive { color: #2ecc71; }
    .status-item .value.negative { color: #e74c3c; }
    
    /* WiFi settings styles */
    .wifi-mode-toggle { display: flex; gap: 10px; margin: 15px 0; }
    .wifi-mode-toggle .btn { flex: 1; }
    .wifi-mode-toggle .btn.selected { box-shadow: inset 0 0 0 3px #f39c12; }
    .network-list { max-height: 200px; overflow-y: auto; background: #1a1a2e; border-radius: 5px; margin: 10px 0; }
    .network-item { padding: 10px; border-bottom: 1px solid #333; cursor: pointer; display: flex; justify-content: space-between; }
    .network-item:hover { background: #1f4068; }
    .network-item .ssid { font-weight: bold; }
    .network-item .signal { color: #888; }
    .form-group { margin: 15px 0; }
    .form-group label { display: block; margin-bottom: 5px; color: #aaa; }
    .form-group input { width: 100%; padding: 10px; border-radius: 5px; border: 1px solid #444; background: #1a1a2e; color: #eee; font-size: 1em; }
    .current-wifi { padding: 15px; background: #1f4068; border-radius: 5px; margin-bottom: 15px; }
    .current-wifi .label { color: #aaa; font-size: 0.9em; }
    .current-wifi .value { font-size: 1.2em; color: #2ecc71; }
    .note { color: #f39c12; font-size: 0.9em; margin-top: 10px; }
  </style>
</head>
<body>
  <h1>IAC Valve Tester <span class="activity-indicator" id="activityLed"></span></h1>
  
  <!-- Tab Navigation -->
  <div class="tab-container">
    <button class="tab active" onclick="showTab('control')">Control</button>
    <button class="tab" onclick="showTab('settings')">WiFi Settings</button>
  </div>
  
  <!-- Control Tab -->
  <div id="control-tab" class="tab-content active">
    <div class="card">
      <h2>Motor Control</h2>
      <div class="direction-btns">
        <button class="btn btn-extend" id="btnExtend" 
          onmousedown="startMove(1)" onmouseup="stopMove()" 
          ontouchstart="startMove(1); event.preventDefault();" ontouchend="stopMove()">&#9650; EXTEND</button>
        <button class="btn btn-retract" id="btnRetract"
          onmousedown="startMove(-1)" onmouseup="stopMove()"
          ontouchstart="startMove(-1); event.preventDefault();" ontouchend="stopMove()">&#9660; RETRACT</button>
      </div>
      <div class="step-btns">
        <button class="btn btn-step" onclick="moveSteps(-100)">-100</button>
        <button class="btn btn-step" onclick="moveSteps(-50)">-50</button>
        <button class="btn btn-step" onclick="moveSteps(-10)">-10</button>
        <button class="btn btn-stop" onclick="stopMove()">STOP</button>
        <button class="btn btn-step" onclick="moveSteps(10)">+10</button>
        <button class="btn btn-step" onclick="moveSteps(50)">+50</button>
        <button class="btn btn-step" onclick="moveSteps(100)">+100</button>
      </div>
      
      <!-- LED Indicator Panel -->
      <h2>I/O Status</h2>
      <div class="led-panel">
        <div class="led-sections">
          <div>
            <h3>Motor Outputs (GPIO → TB6612)</h3>
            <div class="led-row">
              <div class="led off" id="ledAIN1"></div>
              <div class="led-label"><span class="pin">GPIO5</span> AIN1<span class="desc">Coil A+</span></div>
              <div class="led-state low" id="stateAIN1">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledAIN2"></div>
              <div class="led-label"><span class="pin">GPIO6</span> AIN2<span class="desc">Coil A-</span></div>
              <div class="led-state low" id="stateAIN2">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledBIN1"></div>
              <div class="led-label"><span class="pin">GPIO7</span> BIN1<span class="desc">Coil B+</span></div>
              <div class="led-state low" id="stateBIN1">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledBIN2"></div>
              <div class="led-label"><span class="pin">GPIO15</span> BIN2<span class="desc">Coil B-</span></div>
              <div class="led-state low" id="stateBIN2">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledSTBY"></div>
              <div class="led-label"><span class="pin">GPIO16</span> STBY<span class="desc">Enable</span></div>
              <div class="led-state low" id="stateSTBY">LOW</div>
            </div>
          </div>
          <div>
            <h3>PCM Inputs (PC817 → GPIO)</h3>
            <div class="led-row">
              <div class="led off" id="ledPCMA1"></div>
              <div class="led-label"><span class="pin">GPIO17</span> PCM_A1<span class="desc">Coil A+</span></div>
              <div class="led-state low" id="statePCMA1">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledPCMA2"></div>
              <div class="led-label"><span class="pin">GPIO18</span> PCM_A2<span class="desc">Coil A-</span></div>
              <div class="led-state low" id="statePCMA2">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledPCMB1"></div>
              <div class="led-label"><span class="pin">GPIO8</span> PCM_B1<span class="desc">Coil B+</span></div>
              <div class="led-state low" id="statePCMB1">LOW</div>
            </div>
            <div class="led-row">
              <div class="led off" id="ledPCMB2"></div>
              <div class="led-label"><span class="pin">GPIO3</span> PCM_B2<span class="desc">Coil B-</span></div>
              <div class="led-state low" id="statePCMB2">LOW</div>
            </div>
          </div>
        </div>
      </div>
      <div class="status-grid">
        <div class="status-item"><div class="label">Steps Taken</div><div class="value" id="stepsTaken">0</div></div>
        <div class="status-item"><div class="label">Current Step</div><div class="value" id="currentStep">0</div></div>
        <div class="status-item"><div class="label">Motor</div><div class="value" id="motorState">OFF</div></div>
      </div>
    </div>
    
    <div class="card">
      <h2>Step Timing</h2>
      <div class="slider-container">
        <span>Fast</span>
        <input type="range" id="stepDelay" min="1" max="200" value="20" onchange="updateDelay()">
        <span>Slow</span>
        <span id="delayValue">20ms</span>
      </div>
    </div>
    
    <div class="card">
      <h2>Direction Monitor</h2>
      <div class="direction-display">
        <div>
          <div class="direction-box dir-idle" id="youDir">IDLE</div>
          <div style="text-align:center;margin-top:5px;color:#888">Your Command</div>
        </div>
        <div>
          <div class="direction-box dir-idle" id="pcmDir">IDLE</div>
          <div style="text-align:center;margin-top:5px;color:#888">PCM Response</div>
        </div>
      </div>
      <div class="status-grid">
        <div class="status-item"><div class="label">Your Position</div><div class="value" id="youPos">0</div></div>
        <div class="status-item"><div class="label">PCM Position</div><div class="value" id="pcmPos">0</div></div>
        <div class="status-item"><div class="label">Difference</div><div class="value" id="posDiff">0</div></div>
      </div>
    </div>
    
    <div class="card">
      <h2>Position Chart</h2>
      <div class="chart-container"><canvas id="posChart"></canvas></div>
      <button class="btn btn-secondary" onclick="resetPositions()">Reset Positions</button>
    </div>
    
    <div class="card">
      <h2>Signal Log</h2>
      <div id="logArea" class="log-area"></div>
      <div style="margin-top:10px">
        <button class="btn btn-secondary" onclick="clearLog()">Clear</button>
        <button class="btn btn-secondary" onclick="downloadLog()">Download CSV</button>
        <button class="btn btn-secondary" id="pauseBtn" onclick="togglePause()">Pause</button>
      </div>
    </div>
  </div>
  
  <!-- WiFi Settings Tab -->
  <div id="settings-tab" class="tab-content">
    <div class="card">
      <h2>Current Connection</h2>
      <div class="current-wifi">
        <div class="label">Mode</div>
        <div class="value" id="currentMode">--</div>
        <div class="label" style="margin-top:10px">IP Address</div>
        <div class="value" id="currentIP">--</div>
        <div class="label" style="margin-top:10px">Hostname</div>
        <div class="value" id="currentHostname">--</div>
        <div class="label" style="margin-top:10px">SSID</div>
        <div class="value" id="currentSSID">--</div>
      </div>
    </div>
    
    <div class="card">
      <h2>WiFi Mode</h2>
      
      <div class="form-group">
        <label>Hostname (for http://hostname.local)</label>
        <input type="text" id="hostname" placeholder="iac-tester" maxlength="32">
      </div>
      
      <div class="wifi-mode-toggle">
        <button class="btn btn-secondary" id="modeAP" onclick="setWifiMode(true)">Access Point</button>
        <button class="btn btn-secondary" id="modeSTA" onclick="setWifiMode(false)">Connect to Network</button>
      </div>
      
      <!-- AP Mode Settings -->
      <div id="apSettings">
        <h2>Access Point Settings</h2>
        <div class="form-group">
          <label>Network Name (SSID)</label>
          <input type="text" id="apSsid" placeholder="IAC_Tester" maxlength="32">
        </div>
        <div class="form-group">
          <label>Password (min 8 characters)</label>
          <input type="password" id="apPassword" placeholder="jeep1997" maxlength="63">
        </div>
      </div>
      
      <!-- Station Mode Settings -->
      <div id="staSettings" style="display:none">
        <h2>Connect to WiFi Network</h2>
        <button class="btn btn-scan" onclick="scanNetworks()">Scan for Networks</button>
        <div id="networkList" class="network-list"></div>
        <div class="form-group">
          <label>Network Name (SSID)</label>
          <input type="text" id="staSsid" placeholder="Your WiFi Network" maxlength="32">
        </div>
        <div class="form-group">
          <label>Password</label>
          <input type="password" id="staPassword" placeholder="WiFi Password" maxlength="63">
        </div>
      </div>
      
      <div style="margin-top:20px">
        <button class="btn btn-save" onclick="saveWifiSettings()">Save &amp; Restart</button>
      </div>
      <p class="note">⚠ Device will restart after saving. Reconnect to the new network.</p>
    </div>
  </div>
  
  <script>
    let chart;
    let chartData = { labels: [], you: [], pcm: [] };
    let isPaused = false;
    let logEntries = [];
    let isMoving = false;
    let selectedApMode = true;
    
    // Tab switching
    function showTab(tab) {
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
      document.querySelector(`.tab:nth-child(${tab === 'control' ? 1 : 2})`).classList.add('active');
      document.getElementById(tab + '-tab').classList.add('active');
      if (tab === 'settings') loadWifiSettings();
    }
    
    // Initialize chart
    function initChart() {
      const ctx = document.getElementById('posChart').getContext('2d');
      chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'Your Commands', data: [], borderColor: '#f39c12', backgroundColor: 'rgba(243,156,18,0.1)', tension: 0.1 },
            { label: 'PCM Response', data: [], borderColor: '#3498db', backgroundColor: 'rgba(52,152,219,0.1)', tension: 0.1 }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            x: { display: true, ticks: { color: '#888' } },
            y: { display: true, ticks: { color: '#888' } }
          },
          plugins: { legend: { labels: { color: '#eee' } } },
          animation: { duration: 0 }
        }
      });
    }
    
    // Motor control - improved to prevent sticking
    let moveTimeout = null;
    
    function startMove(dir) {
      if (isMoving) return; // Prevent double-start
      isMoving = true;
      document.getElementById(dir > 0 ? 'btnExtend' : 'btnRetract').classList.add('active');
      fetch('/api/move?dir=' + dir + '&continuous=1');
      
      // Safety timeout - stop after 30 seconds max
      clearTimeout(moveTimeout);
      moveTimeout = setTimeout(stopMove, 30000);
    }
    
    function stopMove() {
      if (!isMoving) return;
      isMoving = false;
      clearTimeout(moveTimeout);
      document.getElementById('btnExtend').classList.remove('active');
      document.getElementById('btnRetract').classList.remove('active');
      fetch('/api/stop');
    }
    
    // Global mouseup to catch missed events
    document.addEventListener('mouseup', function() {
      if (isMoving) stopMove();
    });
    document.addEventListener('touchend', function() {
      if (isMoving) stopMove();
    });
    document.addEventListener('touchcancel', function() {
      if (isMoving) stopMove();
    });
    
    function moveSteps(steps) {
      fetch('/api/move?steps=' + steps);
    }
    
    function updateDelay() {
      const delay = document.getElementById('stepDelay').value;
      document.getElementById('delayValue').textContent = delay + 'ms';
      fetch('/api/settings?delay=' + delay);
    }
    
    function resetPositions() {
      fetch('/api/reset');
      chartData = { labels: [], you: [], pcm: [] };
      chart.data.labels = [];
      chart.data.datasets[0].data = [];
      chart.data.datasets[1].data = [];
      chart.update();
    }
    
    function clearLog() { 
      fetch('/api/log/clear'); 
      logEntries = [];
      document.getElementById('logArea').innerHTML = ''; 
    }
    
    function togglePause() {
      isPaused = !isPaused;
      document.getElementById('pauseBtn').textContent = isPaused ? 'Resume' : 'Pause';
    }
    
    // Update LED indicator panel
    function updateLedIndicators(data) {
      // Motor output LEDs (green when HIGH)
      const outputs = [
        { id: 'AIN1', value: data.motorAIN1 },
        { id: 'AIN2', value: data.motorAIN2 },
        { id: 'BIN1', value: data.motorBIN1 },
        { id: 'BIN2', value: data.motorBIN2 },
        { id: 'STBY', value: data.motorSTBY }
      ];
      
      outputs.forEach(pin => {
        const led = document.getElementById('led' + pin.id);
        const state = document.getElementById('state' + pin.id);
        if (led && state) {
          if (pin.value) {
            led.className = 'led on-green';
            state.textContent = 'HIGH';
            state.className = 'led-state high';
          } else {
            led.className = 'led off';
            state.textContent = 'LOW';
            state.className = 'led-state low';
          }
        }
      });
      
      // PCM input LEDs (blue when HIGH - signal detected)
      const inputs = [
        { id: 'PCMA1', value: data.pcmA1 },
        { id: 'PCMA2', value: data.pcmA2 },
        { id: 'PCMB1', value: data.pcmB1 },
        { id: 'PCMB2', value: data.pcmB2 }
      ];
      
      inputs.forEach(pin => {
        const led = document.getElementById('led' + pin.id);
        const state = document.getElementById('state' + pin.id);
        if (led && state) {
          if (pin.value) {
            led.className = 'led on-blue';
            state.textContent = 'HIGH';
            state.className = 'led-state high';
          } else {
            led.className = 'led off';
            state.textContent = 'LOW';
            state.className = 'led-state low';
          }
        }
      });
      
      // Activity LED
      const actLed = document.getElementById('activityLed');
      if (data.isMoving) {
        actLed.classList.add('active');
      } else {
        actLed.classList.remove('active');
      }
      
      // Status items
      document.getElementById('motorState').textContent = data.motorEnabled ? 'ON' : 'OFF';
      document.getElementById('stepsTaken').textContent = data.totalSteps || 0;
      document.getElementById('currentStep').textContent = data.currentStep || 0;
    }
    
    // Poll status
    function pollStatus() {
      if (isPaused) { setTimeout(pollStatus, 500); return; }
      
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          // Update LED indicators
          updateLedIndicators(data);
          
          // Direction displays
          const youDir = document.getElementById('youDir');
          const pcmDir = document.getElementById('pcmDir');
          
          if (data.youDir > 0) { youDir.textContent = 'EXTEND'; youDir.className = 'direction-box dir-extend'; }
          else if (data.youDir < 0) { youDir.textContent = 'RETRACT'; youDir.className = 'direction-box dir-retract'; }
          else { youDir.textContent = 'IDLE'; youDir.className = 'direction-box dir-idle'; }
          
          if (data.pcmDir > 0) { pcmDir.textContent = 'EXTEND'; pcmDir.className = 'direction-box dir-extend'; }
          else if (data.pcmDir < 0) { pcmDir.textContent = 'RETRACT'; pcmDir.className = 'direction-box dir-retract'; }
          else { pcmDir.textContent = 'IDLE'; pcmDir.className = 'direction-box dir-idle'; }
          
          // Positions
          document.getElementById('youPos').textContent = data.youPos;
          document.getElementById('pcmPos').textContent = data.pcmPos;
          const diff = data.youPos - data.pcmPos;
          const diffElem = document.getElementById('posDiff');
          diffElem.textContent = diff;
          diffElem.className = 'value ' + (diff > 0 ? 'positive' : diff < 0 ? 'negative' : '');
          
          // Update chart
          if (data.youPos !== undefined) {
            const now = new Date().toLocaleTimeString();
            chartData.labels.push(now);
            chartData.you.push(data.youPos);
            chartData.pcm.push(data.pcmPos);
            if (chartData.labels.length > 100) {
              chartData.labels.shift();
              chartData.you.shift();
              chartData.pcm.shift();
            }
            chart.data.labels = chartData.labels;
            chart.data.datasets[0].data = chartData.you;
            chart.data.datasets[1].data = chartData.pcm;
            chart.update('none');
          }
          
          // Log entries
          if (data.log && data.log.length > 0) {
            const logArea = document.getElementById('logArea');
            data.log.forEach(entry => {
              const div = document.createElement('div');
              div.className = 'log-entry';
              div.textContent = entry.t + 'ms: A1=' + entry.a1 + ' A2=' + entry.a2 + ' B1=' + entry.b1 + ' B2=' + entry.b2 + (entry.iac ? ' [IAC]' : '');
              logArea.insertBefore(div, logArea.firstChild);
              logEntries.push(entry);
            });
          }
          
          setTimeout(pollStatus, 200);
        })
        .catch(() => setTimeout(pollStatus, 1000));
    }
    
    // WiFi Settings Functions
    function loadWifiSettings() {
      fetch('/api/wifi/settings')
        .then(r => r.json())
        .then(data => {
          document.getElementById('currentMode').textContent = data.apMode ? 'Access Point' : 'Client';
          document.getElementById('currentIP').textContent = data.ip;
          document.getElementById('currentHostname').textContent = data.hostname + '.local';
          document.getElementById('currentSSID').textContent = data.currentSsid;
          document.getElementById('hostname').value = data.hostname || 'iac-tester';
          document.getElementById('apSsid').value = data.apSsid || 'IAC_Tester';
          document.getElementById('apPassword').value = data.apPassword || '';
          document.getElementById('staSsid').value = data.staSsid || '';
          document.getElementById('staPassword').value = data.staPassword || '';
          setWifiMode(data.apMode);
        });
    }
    
    function setWifiMode(apMode) {
      selectedApMode = apMode;
      document.getElementById('modeAP').classList.toggle('selected', apMode);
      document.getElementById('modeSTA').classList.toggle('selected', !apMode);
      document.getElementById('apSettings').style.display = apMode ? 'block' : 'none';
      document.getElementById('staSettings').style.display = apMode ? 'none' : 'block';
    }
    
    function scanNetworks() {
      document.getElementById('networkList').innerHTML = '<div style="padding:20px;text-align:center">Scanning...</div>';
      fetch('/api/wifi/scan')
        .then(r => r.json())
        .then(data => {
          const list = document.getElementById('networkList');
          if (data.networks.length === 0) {
            list.innerHTML = '<div style="padding:20px;text-align:center;color:#888">No networks found</div>';
          } else {
            list.innerHTML = data.networks.map(n => 
              '<div class="network-item" onclick="selectNetwork(\'' + n.ssid.replace(/'/g, "\\'") + '\')">' +
              '<span class="ssid">' + n.ssid + '</span>' +
              '<span class="signal">' + n.rssi + ' dBm' + (n.secure ? ' 🔒' : '') + '</span></div>'
            ).join('');
          }
        })
        .catch(() => {
          document.getElementById('networkList').innerHTML = '<div style="padding:20px;text-align:center;color:#e74c3c">Scan failed</div>';
        });
    }
    
    function selectNetwork(ssid) {
      document.getElementById('staSsid').value = ssid;
      document.getElementById('staPassword').focus();
    }
    
    function saveWifiSettings() {
      const settings = {
        apMode: selectedApMode,
        hostname: document.getElementById('hostname').value,
        apSsid: document.getElementById('apSsid').value,
        apPassword: document.getElementById('apPassword').value,
        staSsid: document.getElementById('staSsid').value,
        staPassword: document.getElementById('staPassword').value
      };
      
      if (!settings.hostname || settings.hostname.length < 1) {
        alert('Please enter a hostname');
        return;
      }
      
      if (selectedApMode && settings.apPassword.length > 0 && settings.apPassword.length < 8) {
        alert('AP password must be at least 8 characters');
        return;
      }
      
      if (!selectedApMode && !settings.staSsid) {
        alert('Please enter a network name');
        return;
      }
      
      fetch('/api/wifi/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(settings)
      })
      .then(r => r.json())
      .then(data => {
        if (data.success) {
          alert('Settings saved! Device will restart...');
        } else {
          alert('Error: ' + data.error);
        }
      })
      .catch(() => alert('Failed to save settings'));
    }
    
    function downloadLog() {
      let csv = 'timestamp,pcmA1,pcmA2,pcmB1,pcmB2,iacActive\n';
      logEntries.forEach(e => {
        csv += e.t + ',' + e.a1 + ',' + e.a2 + ',' + e.b1 + ',' + e.b2 + ',' + (e.iac ? 1 : 0) + '\n';
      });
      const blob = new Blob([csv], { type: 'text/csv' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'iac_log_' + Date.now() + '.csv';
      a.click();
      URL.revokeObjectURL(url);
    }
    
    // Initialize
    initChart();
    pollStatus();
  </script>
</body>
</html>
)rawliteral";

// ============================================
// WIFI SETTINGS FUNCTIONS
// ============================================

void loadWifiSettings() {
  preferences.begin("wifi", true);  // Read-only
  wifiApMode = preferences.getBool("apMode", DEFAULT_AP_MODE);
  wifiApSsid = preferences.getString("apSsid", DEFAULT_AP_SSID);
  wifiApPassword = preferences.getString("apPass", DEFAULT_AP_PASSWORD);
  wifiStaSsid = preferences.getString("staSsid", DEFAULT_STA_SSID);
  wifiStaPassword = preferences.getString("staPass", DEFAULT_STA_PASSWORD);
  wifiHostname = preferences.getString("hostname", DEFAULT_HOSTNAME);
  preferences.end();
  
  Serial.println("WiFi settings loaded:");
  Serial.println("  AP Mode: " + String(wifiApMode ? "Yes" : "No"));
  Serial.println("  AP SSID: " + wifiApSsid);
  Serial.println("  STA SSID: " + wifiStaSsid);
  Serial.println("  Hostname: " + wifiHostname);
}

void saveWifiSettings() {
  preferences.begin("wifi", false);  // Read-write
  preferences.putBool("apMode", wifiApMode);
  preferences.putString("apSsid", wifiApSsid);
  preferences.putString("apPass", wifiApPassword);
  preferences.putString("staSsid", wifiStaSsid);
  preferences.putString("staPass", wifiStaPassword);
  preferences.putString("hostname", wifiHostname);
  preferences.end();
  Serial.println("WiFi settings saved");
}

void setupWifi() {
  // Set hostname before starting WiFi
  WiFi.setHostname(wifiHostname.c_str());
  
  if (wifiApMode) {
    // Access Point mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiApSsid.c_str(), wifiApPassword.c_str());
    Serial.println("\n=== Access Point Mode ===");
    Serial.println("SSID: " + wifiApSsid);
    Serial.println("Password: " + wifiApPassword);
    Serial.println("IP Address: " + WiFi.softAPIP().toString());
    Serial.println("Hostname: " + wifiHostname);
    Serial.println("========================\n");
  } else {
    // Station mode - connect to network
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiStaSsid.c_str(), wifiStaPassword.c_str());
    Serial.print("Connecting to " + wifiStaSsid);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n=== Connected to WiFi ===");
      Serial.println("SSID: " + wifiStaSsid);
      Serial.println("IP Address: " + WiFi.localIP().toString());
      Serial.println("Hostname: " + wifiHostname);
      Serial.println("=========================\n");
    } else {
      Serial.println("\nFailed to connect! Starting AP mode as fallback...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(wifiApSsid.c_str(), wifiApPassword.c_str());
      Serial.println("Fallback AP - SSID: " + wifiApSsid);
      Serial.println("Password: " + wifiApPassword);
      Serial.println("IP: " + WiFi.softAPIP().toString());
    }
  }
  
  // Start mDNS responder
  if (MDNS.begin(wifiHostname.c_str())) {
    Serial.println("mDNS started: http://" + wifiHostname + ".local");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("mDNS failed to start");
  }
}

// ============================================
// WEB HANDLERS
// ============================================

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleStatus() {
  JsonDocument doc;
  
  // Read actual motor output pin states
  int ain1 = digitalRead(PIN_AIN1);
  int ain2 = digitalRead(PIN_AIN2);
  int bin1 = digitalRead(PIN_BIN1);
  int bin2 = digitalRead(PIN_BIN2);
  int stby = digitalRead(PIN_STBY);
  
  doc["motorAIN1"] = ain1;
  doc["motorAIN2"] = ain2;
  doc["motorBIN1"] = bin1;
  doc["motorBIN2"] = bin2;
  doc["motorSTBY"] = stby;
  doc["isMoving"] = isMoving;
  doc["motorEnabled"] = motorEnabled;
  doc["totalSteps"] = totalStepsTaken;
  doc["currentStep"] = currentStep;
  
  // Read PCM inputs (raw GPIO state, then invert for optocoupler)
  int rawA1 = digitalRead(PIN_PCM_A1);
  int rawA2 = digitalRead(PIN_PCM_A2);
  int rawB1 = digitalRead(PIN_PCM_B1);
  int rawB2 = digitalRead(PIN_PCM_B2);
  
  // Debug: print raw input states periodically
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.printf("DEBUG PCM Raw GPIO: A1=%d A2=%d B1=%d B2=%d (inverted: %d %d %d %d)\n", 
      rawA1, rawA2, rawB1, rawB2, !rawA1, !rawA2, !rawB1, !rawB2);
    Serial.printf("DEBUG Motor Out: AIN1=%d AIN2=%d BIN1=%d BIN2=%d STBY=%d\n",
      ain1, ain2, bin1, bin2, stby);
    lastDebug = millis();
  }
  
  // PCM signals (inverted due to optocoupler)
  doc["pcmA1"] = !rawA1;
  doc["pcmA2"] = !rawA2;
  doc["pcmB1"] = !rawB1;
  doc["pcmB2"] = !rawB2;
  
  // Direction
  doc["youDir"] = isMoving ? stepDirection : 0;
  doc["pcmDir"] = pcmDirection;
  doc["youPos"] = youStepPosition;
  doc["pcmPos"] = pcmStepPosition;
  
  // Recent log entries
  JsonArray logArray = doc["log"].to<JsonArray>();
  static int lastSentLog = 0;
  int entriesToSend = min(10, logCount - lastSentLog);
  for (int i = 0; i < entriesToSend; i++) {
    int idx = (lastSentLog + i) % LOG_BUFFER_SIZE;
    JsonObject entry = logArray.add<JsonObject>();
    entry["t"] = signalLog[idx].timestamp;
    entry["a1"] = signalLog[idx].pcmA1;
    entry["a2"] = signalLog[idx].pcmA2;
    entry["b1"] = signalLog[idx].pcmB1;
    entry["b2"] = signalLog[idx].pcmB2;
    entry["iac"] = signalLog[idx].iacActive;
  }
  lastSentLog = logCount;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleMove() {
  if (server.hasArg("continuous")) {
    int dir = server.arg("dir").toInt();
    stepDirection = (dir > 0) ? 1 : -1;
    targetSteps = 0;  // Continuous mode
    enableMotor();
    isMoving = true;
    Serial.println("Continuous move: " + String(stepDirection > 0 ? "EXTEND" : "RETRACT"));
  } else if (server.hasArg("steps")) {
    int steps = server.arg("steps").toInt();
    stepDirection = (steps > 0) ? 1 : -1;
    targetSteps = abs(steps);
    enableMotor();
    isMoving = true;
    Serial.println("Move " + String(steps) + " steps");
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  isMoving = false;
  targetSteps = 0;
  disableMotor();
  Serial.println("Motor stopped");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSettings() {
  if (server.hasArg("delay")) {
    stepDelayMs = server.arg("delay").toInt();
    if (stepDelayMs < 1) stepDelayMs = 1;
    if (stepDelayMs > 500) stepDelayMs = 500;
    Serial.println("Step delay set to: " + String(stepDelayMs) + "ms");
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLogClear() {
  logIndex = 0;
  logCount = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleReset() {
  youStepPosition = 0;
  pcmStepPosition = 0;
  totalStepsTaken = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleWifiSettings() {
  JsonDocument doc;
  doc["apMode"] = wifiApMode;
  doc["apSsid"] = wifiApSsid;
  doc["apPassword"] = wifiApPassword;
  doc["staSsid"] = wifiStaSsid;
  doc["staPassword"] = "";  // Don't send password back
  doc["hostname"] = wifiHostname;
  doc["ip"] = wifiApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  doc["currentSsid"] = wifiApMode ? wifiApSsid : wifiStaSsid;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWifiScan() {
  JsonDocument doc;
  JsonArray networks = doc["networks"].to<JsonArray>();
  
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    JsonObject net = networks.add<JsonObject>();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }
  WiFi.scanDelete();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleWifiSave() {
  if (server.hasArg("plain")) {
    JsonDocument doc;
    deserializeJson(doc, server.arg("plain"));
    
    wifiApMode = doc["apMode"] | true;
    wifiApSsid = doc["apSsid"] | DEFAULT_AP_SSID;
    wifiApPassword = doc["apPassword"] | DEFAULT_AP_PASSWORD;
    wifiStaSsid = doc["staSsid"] | "";
    wifiStaPassword = doc["staPassword"] | "";
    wifiHostname = doc["hostname"] | DEFAULT_HOSTNAME;
    
    // Sanitize hostname (lowercase, no spaces, alphanumeric and hyphens only)
    wifiHostname.toLowerCase();
    String sanitized = "";
    for (int i = 0; i < wifiHostname.length() && sanitized.length() < 32; i++) {
      char c = wifiHostname.charAt(i);
      if (isalnum(c) || c == '-') {
        sanitized += c;
      }
    }
    if (sanitized.length() == 0) sanitized = DEFAULT_HOSTNAME;
    wifiHostname = sanitized;
    
    saveWifiSettings();
    
    server.send(200, "application/json", "{\"success\":true}");
    
    // Restart after sending response
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}");
  }
}

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n================================");
  Serial.println("   IAC Valve Tester Starting");
  Serial.println("================================\n");
  
  // Initialize motor driver pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  
  // Start with motor disabled
  digitalWrite(PIN_STBY, LOW);
  setMotorOutputs(LOW, LOW, LOW, LOW);
  
  // Initialize PCM monitor pins with pull-ups
  pinMode(PIN_PCM_A1, INPUT_PULLUP);
  pinMode(PIN_PCM_A2, INPUT_PULLUP);
  pinMode(PIN_PCM_B1, INPUT_PULLUP);
  pinMode(PIN_PCM_B2, INPUT_PULLUP);
  
  // Load WiFi settings from NVS
  loadWifiSettings();
  
  // Setup WiFi
  setupWifi();
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/move", handleMove);
  server.on("/api/stop", handleStop);
  server.on("/api/settings", handleSettings);
  server.on("/api/log/clear", handleLogClear);
  server.on("/api/reset", handleReset);
  server.on("/api/wifi/settings", handleWifiSettings);
  server.on("/api/wifi/scan", handleWifiScan);
  server.on("/api/wifi/save", HTTP_POST, handleWifiSave);
  
  server.begin();
  Serial.println("Web server started on port 80");
  Serial.println("Open http://" + (wifiApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + " in browser");
  Serial.println("Or http://" + wifiHostname + ".local\n");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  server.handleClient();
  
  // Monitor PCM signals
  monitorPcmSignals();
  
  // Execute stepper movement
  if (isMoving && targetSteps != 0) {
    executeStep();
    targetSteps--;
    if (targetSteps == 0) {
      isMoving = false;
      disableMotor();
    }
    delay(stepDelayMs);
  } else if (isMoving && targetSteps == 0) {
    // Continuous mode
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
  
  // Debug output
  Serial.printf("Motor: AIN1=%d AIN2=%d BIN1=%d BIN2=%d\n", a1, a2, b1, b2);
}

void executeStep() {
  currentStep += stepDirection;
  if (currentStep > 3) currentStep = 0;
  if (currentStep < 0) currentStep = 3;
  
  youStepPosition += stepDirection;
  totalStepsTaken++;
  lastStepTime = millis();
  
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
  Serial.println("Motor ENABLED (STBY=HIGH)");
}

void disableMotor() {
  digitalWrite(PIN_STBY, LOW);
  setMotorOutputs(LOW, LOW, LOW, LOW);
  motorEnabled = false;
  isMoving = false;
  targetSteps = 0;
  Serial.println("Motor DISABLED (STBY=LOW)");
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
  
  // Idle timeout
  if (millis() - lastPcmChange > PCM_IDLE_TIMEOUT) {
    pcmDirection = 0;
  }
  
  // Check for change
  if (currentState != prevPcmState) {
    lastPcmChange = millis();
    
    int8_t newStepIndex = STEP_LOOKUP[currentState];
    
    // Debug: show PCM state changes
    Serial.printf("PCM Change: pattern=%d stepIndex=%d (prev=%d) ", currentState, newStepIndex, pcmStepIndex);
    
    if (newStepIndex >= 0 && pcmStepIndex >= 0) {
      int8_t expectedFwd = (pcmStepIndex + 1) % 4;
      int8_t expectedRev = (pcmStepIndex + 3) % 4;
      
      if (newStepIndex == expectedFwd) {
        pcmDirection = 1;
        pcmStepPosition++;
        Serial.printf("-> FWD pos=%ld\n", pcmStepPosition);
      } else if (newStepIndex == expectedRev) {
        pcmDirection = -1;
        pcmStepPosition--;
        Serial.printf("-> REV pos=%ld\n", pcmStepPosition);
      } else {
        Serial.printf("-> SKIP (expected %d or %d)\n", expectedFwd, expectedRev);
      }
    } else {
      Serial.println(newStepIndex < 0 ? "-> INVALID" : "-> INIT");
    }
    
    pcmStepIndex = newStepIndex;
    
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
