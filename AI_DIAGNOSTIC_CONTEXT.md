# IAC Tester AI Diagnostic Context

This document provides context for AI assistants to help diagnose idle problems on vehicles equipped with bipolar stepper motor IAC (Idle Air Control) valves. It was developed while diagnosing a 1997 Jeep Wrangler 2.5L TJ but applies to similar systems.

---

## How To Use This Document

1. Provide this entire document to the AI assistant
2. Upload or paste your CSV log file(s) from the IAC tester
3. Include any OBD-II scan data you have (screenshots or values)
4. Describe your symptoms (high idle, rough idle, stalling, etc.)

---

## Vehicle System Overview

### 1997 Jeep Wrangler 2.5L (TJ) - Reference Vehicle

| Component | Description |
|-----------|-------------|
| Engine | 2.5L 4-cylinder (AMC 150) |
| Fuel System | Sequential Multi-Port Fuel Injection |
| PCM | Powertrain Control Module (engine computer) |
| IAC Valve | Bipolar stepper motor, 4 wires |
| Location | Mounted on throttle body |

### Idle Control System

The PCM controls engine idle speed by:
1. Reading sensors (RPM, coolant temp, throttle position, etc.)
2. Calculating a target idle speed based on conditions
3. Commanding the IAC valve to open/close airflow
4. Adjusting timing and fuel as needed

### Normal Operating Parameters (Warm Engine)

| Parameter | Normal Range | Concern If... |
|-----------|--------------|---------------|
| Idle RPM | 650-750 RPM | >900 or <500 |
| Coolant Temp | 195-220°F | <160°F (stuck thermostat) |
| TPS (closed) | 0-5% | >10% at closed throttle |
| TPS Voltage | 0.5-1.0V closed | >1.2V at closed throttle |
| Timing Advance | 8-18° BTDC | >25° suggests cold running |
| Short Term Fuel Trim | ±5% | >±10% indicates mixture problem |
| Long Term Fuel Trim | ±10% | >±15% indicates chronic issue |
| MAP (idle) | 1-2 psi / 15-22 inHg vacuum | Low vacuum = leak or restriction |

---

## IAC Valve Technical Details

### Bipolar Stepper Motor Operation

The IAC valve uses a 4-wire bipolar stepper motor with two coils:
- **Coil A**: Wires A1 (+) and A2 (-)
- **Coil B**: Wires B1 (+) and B2 (-)

The PCM energizes these coils in a specific sequence to rotate the motor, which moves a pintle valve to control airflow bypassing the throttle plate.

### Step Sequence (Full Step Mode)

| Step | A1 | A2 | B1 | B2 | Binary Value |
|------|----|----|----|----|--------------|
| 0 | 1 | 0 | 1 | 0 | 5 |
| 1 | 0 | 1 | 1 | 0 | 6 |
| 2 | 0 | 1 | 0 | 1 | 10 |
| 3 | 1 | 0 | 0 | 1 | 9 |

**Direction Detection:**
- Steps 0→1→2→3→0 = EXTEND (opening, more air, higher idle)
- Steps 0→3→2→1→0 = RETRACT (closing, less air, lower idle)

### Special Patterns

| Pattern (A1,A2,B1,B2) | Binary | Meaning |
|-----------------------|--------|---------|
| 1,1,1,1 | 15 | Brake/Hold - all coils energized |
| 0,0,0,0 | 0 | Coast - all coils off |
| Other invalid | - | Transition state or wiring issue |

### IAC Valve Range

- **Fully closed**: ~0 steps (minimum air bypass)
- **Fully open**: ~125 steps (maximum air bypass)
- **Normal warm idle**: 20-40 steps
- **Cold start**: 50-80 steps (more air for fast idle)

---

## IAC Tester Description

### What It Does

The IAC tester is an ESP32-based device that:
1. **Controls the actual IAC valve** - Can manually extend/retract the valve
2. **Monitors PCM signals** - Captures what the PCM is trying to command
3. **Logs data** - Records every PCM signal change with millisecond timestamps

### Hardware Setup

```
┌─────────────────────────────────────────────────────────┐
│                     IAC TESTER                          │
│                                                         │
│   TB6612 Motor Driver ────► ACTUAL IAC VALVE           │
│   (User controls)           (Physically moves)          │
│                             (Changes engine RPM)        │
│                                                         │
│   PC817 Optocouplers ◄───── PCM IAC CONNECTOR          │
│   (Monitor only)            (PCM's attempted commands)  │
│                             (NOT connected to IAC)      │
└─────────────────────────────────────────────────────────┘
```

**Key Point:** The PCM's output is monitored but NOT connected to the actual IAC valve. The user controls the real IAC. This allows observing how the PCM REACTS to RPM changes it didn't cause.

---

## CSV Log File Format

### File Header

```csv
# IAC Tester High-Speed Log
# Entries: 319 / 10000
# Buffer overflow: NO
# Resolution: Interrupt-driven (microsecond accuracy)
#
timestamp,pcmA1,pcmA2,pcmB1,pcmB2,iacActive
```

### Column Definitions

| Column | Type | Description |
|--------|------|-------------|
| timestamp | Integer | Milliseconds since device boot |
| pcmA1 | 0 or 1 | PCM Coil A+ signal (1 = energized) |
| pcmA2 | 0 or 1 | PCM Coil A- signal (1 = energized) |
| pcmB1 | 0 or 1 | PCM Coil B+ signal (1 = energized) |
| pcmB2 | 0 or 1 | PCM Coil B- signal (1 = energized) |
| iacActive | 0 or 1 | 1 = User was commanding the motor |

### Sample Data

```csv
timestamp,pcmA1,pcmA2,pcmB1,pcmB2,iacActive
82674,1,1,1,1,0
82697,1,1,0,0,0
82724,0,1,0,0,0
82731,0,1,1,1,0
82740,1,1,1,1,0
```

---

## How To Analyze Log Data

### Step 1: Identify Activity Phases

Look for clusters of rapid timestamp changes vs. long gaps:

| Pattern | Timestamp Gaps | Interpretation |
|---------|----------------|----------------|
| Rapid stepping | 4-20ms | PCM actively adjusting |
| Slow stepping | 200-1000ms | PCM making fine corrections |
| Long gaps | >2000ms | PCM holding position |

### Step 2: Check for Valid Step Patterns

Convert each row to a binary value: `(pcmA1) + (pcmA2*2) + (pcmB1*4) + (pcmB2*8)`

| Binary Value | Valid? | Pattern |
|--------------|--------|---------|
| 5 | Yes | Step 0 |
| 6 | Yes | Step 1 |
| 9 | Yes | Step 3 |
| 10 | Yes | Step 2 |
| 15 | Hold | Brake position |
| 0 | Coast | All off |
| Others | Transition | Caught mid-change |

### Step 3: Determine Direction

Track the sequence of valid steps:
- 5 → 6 → 10 → 9 → 5 = EXTEND (opening)
- 5 → 9 → 10 → 6 → 5 = RETRACT (closing)

### Step 4: Correlate with User Actions

When `iacActive = 1`, the user was commanding the motor. Watch what the PCM does AFTER the user stops (`iacActive` returns to 0):
- PCM steps in opposite direction = PCM trying to compensate (GOOD)
- PCM steps in same direction = Unexpected
- PCM does nothing = May not be in closed-loop idle control

### Step 5: Count Steps and Calculate Rate

```
Step interval = timestamp_n - timestamp_(n-1)
Steps per second = 1000 / average_interval_ms
Total steps = count of valid step transitions
```

---

## Common Diagnostic Scenarios

### Scenario 1: High Idle (Engine Runs Fast)

**Symptoms:** Idle >900 RPM when warm

**Analysis Approach:**
1. Check OBD-II TPS reading - should be 0-5% at closed throttle
2. If TPS >10%, throttle plate may not be fully closing
3. Check for vacuum leaks (STFT/LTFT will be positive if lean)
4. Review PCM log - is PCM trying to RETRACT (close) the IAC?

**Log Pattern - PCM Trying to Fix:**
```
PCM stepping with intervals 200-800ms
Direction: Mostly RETRACT attempts
Result: PCM is trying but something is overriding
```

**Log Pattern - PCM Commanding High Idle:**
```
PCM at rest (holding 1,1,1,1) or barely moving
PCM "thinks" high idle is correct
Check sensors: ECT reading cold? TPS not showing closed?
```

### Scenario 2: Low/Rough Idle (Engine Runs Slow or Stumbles)

**Symptoms:** Idle <600 RPM, rough running, may stall

**Analysis Approach:**
1. Check for restricted IAC (carbon buildup)
2. Verify PCM is trying to EXTEND (open) the IAC
3. Check fuel trims - negative LTFT suggests rich condition
4. May be mechanical (timing, compression, vacuum leak)

**Log Pattern:**
```
PCM rapidly stepping EXTEND direction
Intervals: Fast (8-20ms) = PCM in "panic mode"
PCM is trying hard to raise idle but failing
```

### Scenario 3: Surging/Hunting Idle

**Symptoms:** RPM cycles up and down rhythmically

**Analysis Approach:**
1. Look for alternating EXTEND/RETRACT patterns in log
2. Check for vacuum leak (causes lean surge)
3. O2 sensor may be slow or failing
4. IAC valve may be sticking

**Log Pattern:**
```
Alternating bursts: EXTEND for 2-3 sec, RETRACT for 2-3 sec
PCM is "chasing" the correct idle and overshooting
```

### Scenario 4: PCM Not Responding to RPM Changes

**Test:** User manually changes IAC position, watches PCM reaction

**Expected:** When user closes IAC (RPM drops), PCM should try to EXTEND

**Log Pattern - PCM Working:**
```
iacActive=1 (user retracting)
[RPM drops]
iacActive=0 (user stopped)
PCM immediately starts EXTEND sequence (fast stepping)
```

**Log Pattern - PCM NOT Working:**
```
iacActive=1 (user retracting)
[RPM drops]
iacActive=0 (user stopped)
PCM does nothing or continues same direction
PCM may not be in closed-loop idle mode
```

---

## Diagnostic Decision Tree

```
START: High Idle Problem
│
├─► Is TPS showing >10% at closed throttle?
│   ├─► YES: Check throttle cable slack, throttle stop screw,
│   │        TPS adjustment, carbon on throttle plate
│   │        (Physical problem, not IAC)
│   │
│   └─► NO: Continue...
│
├─► Are fuel trims significantly positive (>10%)?
│   ├─► YES: Vacuum leak downstream of MAF/MAP
│   │        Spray test around intake, hoses, gaskets
│   │
│   └─► NO: Continue...
│
├─► Is coolant temp reading correctly (190-220°F when warm)?
│   ├─► NO: ECT sensor faulty, PCM thinks engine is cold
│   │       Commands fast idle, high timing advance
│   │
│   └─► YES: Continue...
│
├─► In log data, is PCM actively trying to RETRACT?
│   ├─► YES: PCM knows idle is high, trying to fix
│   │        Something is overriding IAC airflow
│   │        Check for air leak AFTER throttle plate
│   │
│   └─► NO: PCM is not trying to lower idle
│           PCM may be commanding high idle intentionally
│           Check inputs: TPS, park/neutral switch, A/C request
│
└─► Check throttle body idle air screw adjustment
    Factory setting has tamper cap - if missing, someone adjusted it
```

---

## OBD-II Parameters to Collect

When asking for diagnostic help, provide these values if available:

### Critical Parameters

| Parameter | Your Value | Normal Range |
|-----------|------------|--------------|
| Engine RPM | ___ | 650-750 (warm) |
| Coolant Temp (ECT) | ___°F | 195-220°F |
| Throttle Position | ___% | 0-5% (closed) |
| TPS Voltage | ___V | 0.5-1.0V (closed) |
| Timing Advance | ___° | 8-18° (warm idle) |
| Short Term Fuel Trim | ___% | ±5% |
| Long Term Fuel Trim | ___% | ±10% |

### Helpful If Available

| Parameter | Your Value | Notes |
|-----------|------------|-------|
| MAP Pressure | ___ | Vacuum reading |
| Intake Air Temp | ___°F | Affects fuel calc |
| O2 Sensor Voltage | ___V | 0.1-0.9V cycling |
| Vehicle Speed | ___ | Should be 0 at idle |
| IAC Steps/Position | ___ | If scan tool shows it |
| Closed Throttle Switch | ___ | YES/NO if available |
| Fault Codes | ___ | Even pending/history |

---

## Interpreting Combined Data

### Example: PCM Response Test

**Procedure:**
1. Clear log buffer
2. Let engine idle 10 seconds (baseline)
3. RETRACT IAC 150 steps (drops RPM)
4. Wait 20 seconds, watch PCM response
5. Download log

**Healthy PCM Response:**

| Time | What Happened | Log Shows |
|------|---------------|-----------|
| 0-10s | Baseline idle | PCM slow stepping (fine adjustments) |
| 10-13s | User retracts | iacActive=1, RPM drops |
| 13-15s | User stops | iacActive=0 |
| 15-20s | PCM reacts | BURST of fast EXTEND stepping |
| 20-30s | Recovery | PCM slows down as RPM recovers |

**This pattern indicates:**
- ✅ PCM closed-loop idle control is functioning
- ✅ PCM detects RPM change and compensates
- ✅ IAC valve and wiring are working

**Unhealthy Response:**
- PCM does nothing after user stops
- PCM continues same direction
- PCM makes only 1-2 steps then stops

---

## Quick Reference: Coil Patterns

### Binary Conversion

To convert log data to binary step pattern:
```
Binary = pcmA1 + (pcmA2 × 2) + (pcmB1 × 4) + (pcmB2 × 8)
```

### Pattern Table

| A1 | A2 | B1 | B2 | Binary | Meaning |
|----|----|----|----| -------|---------|
| 1 | 0 | 1 | 0 | 5 | Step 0 |
| 0 | 1 | 1 | 0 | 6 | Step 1 |
| 0 | 1 | 0 | 1 | 10 | Step 2 |
| 1 | 0 | 0 | 1 | 9 | Step 3 |
| 1 | 1 | 1 | 1 | 15 | Hold/Brake |
| 0 | 0 | 0 | 0 | 0 | Coast/Off |

### Direction (reading log top to bottom)

| From Step | To Step | Direction |
|-----------|---------|-----------|
| 0 (5) | 1 (6) | EXTEND |
| 1 (6) | 2 (10) | EXTEND |
| 2 (10) | 3 (9) | EXTEND |
| 3 (9) | 0 (5) | EXTEND |
| 0 (5) | 3 (9) | RETRACT |
| 3 (9) | 2 (10) | RETRACT |
| 2 (10) | 1 (6) | RETRACT |
| 1 (6) | 0 (5) | RETRACT |

---

## Tips for AI Assistants

When helping diagnose:

1. **Always ask for OBD-II data** if not provided - the log alone shows PCM behavior but not WHY
2. **Check TPS first** - a TPS showing >10% at closed throttle is a very common cause of high idle
3. **Look for fast stepping bursts** in the log - these indicate the PCM is actively trying to compensate
4. **Calculate approximate step counts** - knowing how many steps the PCM commanded helps quantify the issue
5. **Correlate iacActive with PCM response** - the key test is whether PCM reacts when the user changes RPM
6. **Consider mechanical causes** - throttle stop screw, vacuum leaks, stuck IAC valve
7. **Check for the obvious** - cable tension, throttle plate closing fully, basic wiring

---

## Document Version

- **Version:** 1.0
- **Date:** December 2024
- **Developed for:** 1997 Jeep Wrangler 2.5L TJ
- **Applicable to:** Any vehicle with bipolar stepper motor IAC valve
- **Tester Hardware:** ESP32-S3 + TB6612FNG + PC817 optocouplers

---

*This context document was created to help AI assistants provide accurate diagnostic guidance for IAC-related idle problems. It is based on real-world troubleshooting that successfully identified a maladjusted throttle stop screw causing high idle.*
