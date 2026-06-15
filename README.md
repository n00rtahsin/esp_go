# ✈️ ESP_GO — ESP32 Wi-Fi Controlled Brushed Micro Drone

<div align="center">

![ESP32](https://img.shields.io/badge/MCU-ESP32-blue?style=for-the-badge&logo=espressif)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Build%20Ready-brightgreen?style=for-the-badge)
![Control](https://img.shields.io/badge/Control-Wi--Fi%20Browser-blueviolet?style=for-the-badge)

**A fully self-leveling micro quadcopter controlled via a browser-based dual-joystick interface over Wi-Fi, powered by an ESP32 microcontroller, MPU6050 IMU, and four 8520 coreless brushed motors.**
</div>

![Uploading image.png…]()

</div>
---

## 📖 Table of Contents

1. [Project Overview](#-project-overview)
2. [Key Features](#-key-features)
3. [System Architecture](#-system-architecture)
4. [Bill of Materials (BOM)](#-bill-of-materials-bom)
5. [Pin Mapping & Wiring Reference](#-pin-mapping--wiring-reference)
6. [Motor Layout & Propeller Configuration](#-motor-layout--propeller-configuration)
7. [Electrical Schematic Overview](#-electrical-schematic-overview)
8. [Software & Firmware](#-software--firmware)
9. [Library Dependencies](#-library-dependencies)
10. [Firmware Configuration (PID & Constants)](#-firmware-configuration-pid--constants)
11. [Build Instructions — Step by Step](#-build-instructions--step-by-step)
12. [Wi-Fi Control Interface](#-wi-fi-control-interface)
13. [Failsafe System](#-failsafe-system)
14. [Pre-Flight Ground Test Procedure](#-pre-flight-ground-test-procedure)
15. [PID Tuning Guide](#-pid-tuning-guide)
16. [First Flight Procedure](#-first-flight-procedure)
17. [Troubleshooting](#-troubleshooting)
18. [Safety Warnings](#%EF%B8%8F-safety-warnings)
19. [Power Budget & Runtime Estimate](#-power-budget--runtime-estimate)
20. [Project File Structure](#-project-file-structure)
21. [Contributing](#-contributing)
22. [License](#-license)

---

## 🚁 Project Overview

**ESP_GO** is an open-source, DIY micro quadcopter project built around the **ESP32 microcontroller**. It is designed to be approachable for hobbyists, students, and embedded systems engineers who want to understand the fundamentals of drone flight control — from IMU sensor fusion and PID loop implementation to PWM motor driving via MOSFETs and real-time wireless control.

The drone is piloted through a **web browser on any Wi-Fi-connected phone or tablet** — no app installation, no dedicated radio transmitter, no proprietary protocol. The ESP32 runs its own **Wi-Fi Access Point (AP)** and serves a dual-joystick HTML/JS page. Joystick inputs are streamed to the ESP32 in real time, which then computes a **PID stabilization loop** using data from the **MPU6050 6-axis IMU** and outputs four independent **PWM signals** to four **SI2302 N-Channel MOSFETs**, each switching a brushed DC motor.

This project covers the **complete stack**: hardware assembly, soldering, firmware flashing, calibration, PID tuning, and first flight.

---

## ✨ Key Features

| Feature | Detail |
|---|---|
| 🌐 **Browser Control** | Dual-joystick HTML5 UI served directly by the ESP32 — no app required |
| 📡 **Onboard Wi-Fi AP** | SSID: `Drone_WiFi` · Password: `drone1234` · IP: `192.168.4.1` |
| 🧠 **PID Self-Leveling** | Full roll + pitch PID stabilization via MPU6050 gyro/accel data |
| 🔌 **3.3 V Logic** | All GPIO signals are 3.3 V; SI2302 MOSFETs are fully enhanced at 3.3 V |
| 🛡️ **Wi-Fi Failsafe** | All motors cut within **500 ms** of Wi-Fi connection loss |
| ⚡ **Direct LiPo Drive** | Motors powered directly from single-cell 3.7 V LiPo for maximum efficiency |
| 🔧 **Runtime PID Config** | KP / KI / KD adjustable in firmware without hardware changes |
| 🛠️ **Hackable Frame** | Compatible with popsicle-stick, CF-rod, or 3D-printed X-frames |
| 📊 **Serial Telemetry** | Live IMU angles + motor PWM values at 115200 baud over USB |
| 🔋 **Onboard Regulation** | AMS1117-3.3 on ESP32 DevKit steps 3.7 V LiPo down to 3.3 V for MCU + IMU |

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         PILOT (Browser)                          │
│              Dual Joystick UI  ─  http://192.168.4.1             │
└─────────────────────────┬────────────────────────────────────────┘
                          │  Wi-Fi (ESP32 Soft-AP)
                          │  WebSocket / HTTP
┌─────────────────────────▼────────────────────────────────────────┐
│                         ESP32 (MCU)                              │
│                                                                  │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────────────┐ │
│  │ Wi-Fi Stack │───▶│  Flight Ctrl │◀───│  MPU6050 (I2C)      │ │
│  │  (Soft-AP)  │    │  PID Loop    │    │  GPIO21=SDA         │ │
│  └─────────────┘    └──────┬───────┘    │  GPIO22=SCL         │ │
│                            │ PWM        └─────────────────────┘ │
│           ┌────────────────┼────────────────────────┐           │
│           │                │                        │           │
│        GPIO13           GPIO25                   GPIO26  GPIO27 │
│        (FL-CW)          (FR-CCW)                 (BL-CCW)(BR-CW)│
└───────────┼────────────────┼────────────────────────┼─────┬─────┘
            │                │                        │     │
     ┌──────▼──────┐  ┌──────▼──────┐  ┌─────────────▼┐  ┌▼───────────┐
     │  SI2302     │  │  SI2302     │  │  SI2302      │  │  SI2302    │
     │  MOSFET FL  │  │  MOSFET FR  │  │  MOSFET BL   │  │  MOSFET BR │
     └──────┬──────┘  └──────┬──────┘  └──────┬───────┘  └─────┬──────┘
            │ Drain→Motor-   │ Drain→Motor-    │ Drain→Motor-   │ Drain→Motor-
     ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼───────┐  ┌─────▼──────┐
     │ 8520 Motor  │  │ 8520 Motor  │  │ 8520 Motor   │  │ 8520 Motor │
     │  CW  (FL)   │  │  CCW (FR)   │  │  CCW  (BL)   │  │  CW  (BR)  │
     └──────▲──────┘  └──────▲──────┘  └──────▲───────┘  └─────▲──────┘
            │                │                 │                 │
            └────────────────┴─────────────────┴─────────────────┘
                              LiPo + Rail → All Motor VCC+

            LiPo GND ──────────────────── GND Rail (all SOURCE pins + ESP32 GND)
            LiPo +V  ──────────────────── ESP32 VIN  (via AMS1117 → 3.3V)
```

---

## 📦 Bill of Materials (BOM)

| # | Component | Qty | Notes | Buy Link |
|---|-----------|:---:|-------|----------|
| 1 | **ESP32 Development Board** | 1 | Any ESP32 DevKit with AMS1117-3.3 regulator and exposed VIN, GND, GPIO13/25/26/27/21/22, 3V3 pins | [OpenCircuit](https://www.opencircuit.shop/product/arduino-nano-esp32?cl=Z7QJBDMZTM) |
| 2 | **LiPo 3.7V 1000mAh Battery** | 1 | Single-cell (1S). JST-PH 2.0 connector preferred. Pair with TP4056 for safe USB recharging | [OpenCircuit](https://www.opencircuit.shop/product/lithium-polymer-battery-3-7v-1000mah?cl=Z7QJBDMZTM) |
| 3 | **MPU6050 IMU Module (GY-521)** | 1 | 6-axis gyroscope + accelerometer, I2C interface, 3.3V compatible | [OpenCircuit](https://www.opencircuit.shop/product/6-dof-gyroscope-accelerometer-module-gy-521?cl=Z7QJBDMZTM) |
| 4 | **SI2302 N-Channel MOSFET** | 4 | SOT-23 package, logic-level gate (~2.5V threshold, fully enhanced at 3.3V), up to 2A continuous | [OpenCircuit](https://www.opencircuit.shop/product/sparkfun-mosfet-power-controller?cl=Z7QJBDMZTM) |
| 5 | **8520 Coreless Brushed Motor — CW** | 2 | 8.5×20mm, clockwise rotation, ~3.7V, peak ~1A. Front-Left + Back-Right positions | [OpenCircuit](https://www.opencircuit.shop/product/sg90-micro-servo?cl=Z7QJBDMZTM) |
| 6 | **8520 Coreless Brushed Motor — CCW** | 2 | 8.5×20mm, counter-clockwise rotation, ~3.7V, peak ~1A. Front-Right + Back-Left positions | [OpenCircuit](https://www.opencircuit.shop/product/sg90-micro-servo?cl=Z7QJBDMZTM) |
| 7 | **Matching CW Propellers** | 2 | Sized for 8520 motor shaft. Press-fit. CW rotation |  |
| 8 | **Matching CCW Propellers** | 2 | Sized for 8520 motor shaft. Press-fit. CCW rotation |  |
| 9 | **Mini Protoboard / PCB** | 1 | For MOSFET mounting and power bus |  |
| 10 | **10kΩ Resistors** | 4 | Optional pull-down resistors between each GATE and SOURCE — highly recommended |  |
| 11 | **Wire (24–26 AWG)** | — | Use 24 AWG for power rails, 26–28 AWG for signal wires |  |
| 12 | **Drone Frame** | 1 | DIY popsicle-stick / CF-rod / 3D-printed X-frame. Keep arm length symmetric |  |
| 13 | **1–2mm Foam Pad** | 1 small | For vibration isolation under MPU6050 module |  |
| 14 | **Hot Glue / Zip Ties** | — | For securing motors and wiring to frame |  |
| 15 | **TP4056 LiPo Charger** | 1 | Recommended for safe USB recharging of LiPo | |
| 16 | **LiPo Voltage Alarm** | 1 | Buzzer alarm to alert when cell drops below 3.2V — strongly recommended |  |

---

## 🔌 Pin Mapping & Wiring Reference

### ESP32 GPIO Assignments

| GPIO | Direction | Connected To | Function |
|------|-----------|-------------|----------|
| `GPIO13` | Output | MOSFET FL — GATE | PWM · Front-Left Motor |
| `GPIO25` | Output | MOSFET FR — GATE | PWM · Front-Right Motor |
| `GPIO26` | Output | MOSFET BL — GATE | PWM · Back-Left Motor |
| `GPIO27` | Output | MOSFET BR — GATE | PWM · Back-Right Motor |
| `GPIO21` | Bidirectional | MPU6050 — SDA | I2C Data |
| `GPIO22` | Output | MPU6050 — SCL | I2C Clock |
| `3V3` | Power Out | MPU6050 — VCC | 3.3 V regulated power |
| `VIN` | Power In | LiPo `+V` | Raw 3.7–4.2 V from battery |
| `GND` | Ground | LiPo `GND` · All MOSFET SOURCE · MPU6050 GND | Common ground rail |

---

### MPU6050 IMU Wiring

| MPU6050 Pin | Connects To | Notes |
|-------------|------------|-------|
| `VCC` | ESP32 `3V3` | **3.3V ONLY — never 5V** |
| `GND` | GND Rail | Common ground |
| `SDA` | ESP32 `GPIO21` | I2C data line |
| `SCL` | ESP32 `GPIO22` | I2C clock line |
| `INT` | *(not connected)* | Interrupt not used in this build |
| `AD0` | *(not connected)* | Leaves I2C address at default 0x68 |

---

### SI2302 MOSFET Wiring (×4)

> **SOT-23 Pinout** (flat face toward you, pins down): `Gate = left pin` · `Source = right pin` · `Drain = middle top pin`
> Verify with your specific datasheet — orientation errors will permanently damage the MOSFET.

| Position | MOSFET Pin | Connects To |
|----------|-----------|------------|
| **Front-Left (FL)** | GATE | ESP32 `GPIO13` |
| | SOURCE | GND Rail |
| | DRAIN | 8520 CW Motor FL — `GND-` (negative terminal) |
| **Front-Right (FR)** | GATE | ESP32 `GPIO25` |
| | SOURCE | GND Rail |
| | DRAIN | 8520 CCW Motor FR — `GND-` |
| **Back-Left (BL)** | GATE | ESP32 `GPIO26` |
| | SOURCE | GND Rail |
| | DRAIN | 8520 CCW Motor BL — `GND-` |
| **Back-Right (BR)** | GATE | ESP32 `GPIO27` |
| | SOURCE | GND Rail |
| | DRAIN | 8520 CW Motor BR — `GND-` |

---

### Motor Power Wiring

| Motor | `VCC+` Connects To | `GND-` Connects To |
|-------|---------------------|---------------------|
| FL (CW) | LiPo `+V` (power bus) | MOSFET FL — DRAIN |
| FR (CCW) | LiPo `+V` (power bus) | MOSFET FR — DRAIN |
| BL (CCW) | LiPo `+V` (power bus) | MOSFET BL — DRAIN |
| BR (CW) | LiPo `+V` (power bus) | MOSFET BR — DRAIN |

> ⚠️ All four motor `VCC+` wires connect to a shared central power bus that links to the LiPo `+V`. Each motor's `GND-` is switched individually by its respective MOSFET.

---

### LiPo Battery Wiring

| LiPo Terminal | Connects To | Purpose |
|---------------|------------|---------|
| `+V` (positive) | ESP32 `VIN` pin + Motor power bus | Powers ESP32 (via AMS1117) and all motors |
| `GND` (negative) | GND Rail | Common ground for entire circuit |

---

## 🔄 Motor Layout & Propeller Configuration

```
              FRONT
         ___         ___
        /FL \       /FR \
       | CW  |     | CCW |     ← Viewed from above
        \___/       \___/
          ↑           ↑
     GPIO13         GPIO25
          
          ↓           ↓
        ___           ___
       /BL \         /BR \
      | CCW |       | CW  |
       \___/         \___/
     GPIO26         GPIO27

              REAR
```

### Motor & Propeller Direction Table

| Position | Label | Rotation | GPIO | Prop Type |
|----------|-------|----------|------|-----------|
| Front-Left | FL | **Clockwise (CW)** | 13 | CW Prop |
| Front-Right | FR | **Counter-Clockwise (CCW)** | 25 | CCW Prop |
| Back-Left | BL | **Counter-Clockwise (CCW)** | 26 | CCW Prop |
| Back-Right | BR | **Clockwise (CW)** | 27 | CW Prop |

> **Why this pattern?** Opposing rotation on diagonal motor pairs cancels **yaw torque**. Without this, the frame would spin uncontrollably. CW and CCW motors must have matching CW and CCW propellers respectively — installing a CW prop on a CCW motor produces **zero thrust** and usually results in the prop flying off.

**Visual identification tip:** Most 8520 motor packs mark CW motors with a **black hub** and CCW motors with a **silver/white hub**. Always verify by briefly powering each motor individually and checking rotation before mounting propellers.

---

## ⚡ Electrical Schematic Overview

```
LiPo 3.7V 1000mAh
  │
  ├─── (+V) ────────────────────────────────────────── ESP32 VIN
  │                                                        │
  │                                                    AMS1117-3.3
  │                                                        │
  │                                                    ESP32 3V3 ──── MPU6050 VCC
  │
  ├─── (+V) ──── Power Bus ──┬──────────┬──────────┬──────────┐
  │                          │          │          │          │
  │                     FL Motor   FR Motor   BL Motor   BR Motor
  │                     VCC+(red)  VCC+(red)  VCC+(red)  VCC+(red)
  │
  │                     FL Motor   FR Motor   BL Motor   BR Motor
  │                     GND-(blk)  GND-(blk)  GND-(blk)  GND-(blk)
  │                          │          │          │          │
  │                       DRAIN      DRAIN      DRAIN      DRAIN
  │                     [MOSFET FL][MOSFET FR][MOSFET BL][MOSFET BR]
  │                       GATE       GATE       GATE       GATE
  │                          │          │          │          │
  │                       GPIO13    GPIO25     GPIO26     GPIO27
  │                          └──────────┴──────────┴──────────┘
  │                                       ESP32
  │                       SOURCE     SOURCE     SOURCE     SOURCE
  │                          │          │          │          │
  └─── (GND) ───── GND Rail ──┴──────────┴──────────┴──────────┘
                        │
                   ESP32 GND ──── MPU6050 GND
                        │
                   MPU6050 SDA ── GPIO21
                   MPU6050 SCL ── GPIO22
```

---

## 💾 Software & Firmware

### How the Firmware Works

The ESP32 firmware runs four concurrent tasks:

#### 1. Wi-Fi Access Point + Web Server
The ESP32 initialises a **Soft Access Point** (`Drone_WiFi` / `drone1234`). It serves a single-page HTML/JS application containing a **dual-joystick touch interface** (Left stick = throttle/yaw, Right stick = pitch/roll). Joystick position data is streamed to the ESP32 in real time, typically via **WebSocket** or HTTP polling, and parsed into throttle/yaw/pitch/roll command values.

#### 2. MPU6050 Sensor Reading
The firmware reads the MPU6050 via I2C at a fixed rate (typically 100–250Hz). The `MPU6050_light` library handles register communication and provides pre-computed **roll** and **pitch** angles (in degrees) using a complementary filter combining accelerometer and gyroscope data.

#### 3. PID Flight Controller
For each loop iteration:
1. Read **setpoint** (desired pitch/roll) from Wi-Fi joystick input
2. Read **current angle** from MPU6050
3. Compute **error** = setpoint − current angle
4. Run PID calculation:
   - **Proportional term**: `P = KP × error`
   - **Integral term**: `I += KI × error × dt` (accumulates over time)
   - **Derivative term**: `D = KD × (error − previous_error) / dt`
   - **PID output** = `P + I + D`
5. Mix PID output with base throttle into four individual motor outputs using standard **quadcopter motor mixing**:
   ```
   motor_FL = throttle + roll_pid + pitch_pid - yaw_pid
   motor_FR = throttle - roll_pid + pitch_pid + yaw_pid
   motor_BL = throttle + roll_pid - pitch_pid + yaw_pid
   motor_BR = throttle - roll_pid - pitch_pid - yaw_pid
   ```
6. Clamp all motor values to [0, 255] and write to PWM channels

#### 4. PWM Motor Output
Four **LEDC PWM channels** on the ESP32 output duty-cycle signals on GPIO13/25/26/27. These drive the GATE of each SI2302 MOSFET, which in turn switches the motor GND path at high frequency (typically 5–20kHz), effectively controlling motor speed.

#### 5. Failsafe Monitor
A background timer tracks the **time since last valid command received from the browser**. If this exceeds **500 milliseconds** (e.g., Wi-Fi disconnected, browser closed, device sleep), all four motor PWM outputs are set to zero immediately.

---

## 📚 Library Dependencies

Install the following libraries via **Arduino IDE Library Manager** or `platformio.ini`:

| Library | Author | Purpose | Install Name |
|---------|--------|---------|-------------|
| `MPU6050_light` | rfetick | Lightweight MPU6050 driver with complementary filter | `MPU6050_light` |
| `WiFi` | Espressif | ESP32 Wi-Fi AP + Station | Built-in (ESP32 Arduino Core) |
| `WebServer` | Espressif | HTTP server to serve joystick UI | Built-in (ESP32 Arduino Core) |
| `WebSocketsServer` | Links2004 | Real-time bidirectional joystick data | `WebSockets` |
| `Wire` | Arduino | I2C communication for MPU6050 | Built-in |

### Arduino IDE Setup

1. Add the ESP32 board package URL in **File → Preferences → Additional Board URLs**:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. Install **ESP32** via **Tools → Board → Boards Manager**
3. Select **Board**: `ESP32 Dev Module`
4. Set **Upload Speed**: `921600`
5. Set **Flash Size**: `4MB (32Mb)`
6. Set **Partition Scheme**: `Default 4MB with spiffs`

### PlatformIO (alternative)

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    rfetick/MPU6050_light @ ^1.0.0
    links2004/WebSockets @ ^2.3.7
monitor_speed = 115200
```

---

## ⚙️ Firmware Configuration (PID & Constants)

All tunable parameters are defined as `#define` constants at the top of the firmware file. **Edit these values and re-flash** to adjust drone behaviour.

```cpp
// ─── Wi-Fi Credentials ────────────────────────────────────────────────────────
#define WIFI_SSID       "Drone_WiFi"
#define WIFI_PASSWORD   "drone1234"
#define AP_IP           "192.168.4.1"

// ─── Motor GPIO Pins ──────────────────────────────────────────────────────────
#define MOTOR_FL_PIN    13    // Front-Left  (CW)
#define MOTOR_FR_PIN    25    // Front-Right (CCW)
#define MOTOR_BL_PIN    26    // Back-Left   (CCW)
#define MOTOR_BR_PIN    27    // Back-Right  (CW)

// ─── IMU I2C Pins ─────────────────────────────────────────────────────────────
#define IMU_SDA_PIN     21
#define IMU_SCL_PIN     22

// ─── PWM Configuration ────────────────────────────────────────────────────────
#define PWM_FREQ        10000   // Hz — 10kHz is a good balance for 8520 motors
#define PWM_RESOLUTION  8       // bits — values 0–255

// ─── Failsafe ─────────────────────────────────────────────────────────────────
#define FAILSAFE_TIMEOUT_MS   500   // ms — motors cut after 500ms with no command

// ─── PID Gains — ROLL Axis ────────────────────────────────────────────────────
#define KP_ROLL   0.8f    // Proportional gain — increase for faster correction
#define KI_ROLL   0.01f   // Integral gain — increase to eliminate steady-state drift
#define KD_ROLL   0.5f    // Derivative gain — increase to dampen oscillations

// ─── PID Gains — PITCH Axis ───────────────────────────────────────────────────
#define KP_PITCH  0.8f
#define KI_PITCH  0.01f
#define KD_PITCH  0.5f

// ─── PID Gains — YAW Axis ─────────────────────────────────────────────────────
#define KP_YAW    1.0f
#define KI_YAW    0.0f
#define KD_YAW    0.0f

// ─── Flight Limits ────────────────────────────────────────────────────────────
#define MAX_ANGLE_DEG       30     // Max tilt angle commanded by joystick (degrees)
#define MIN_THROTTLE         30    // Minimum PWM to keep motors spinning (0–255)
#define MAX_THROTTLE        200    // Maximum safe PWM on 3.7V LiPo (0–255)
#define IMU_CALIBRATION_MS 3000   // ms to hold flat for IMU zero calibration on boot
```

> **Starting point values**: `KP=0.8`, `KI=0.01`, `KD=0.5` are deliberately conservative — safe for a first hover. Tune from here as described in the [PID Tuning Guide](#-pid-tuning-guide).

---

## 🔧 Build Instructions — Step by Step

### Step 1 — Prepare the Frame

1. Select or build your drone frame. An **X-configuration** frame with four symmetric arms is required. Common DIY options:
   - **Popsicle sticks** (≈150mm span, cross-glued) — very light, fragile
   - **Carbon fibre rods** — strong, lightweight, requires cutting tools
   - **3D printed frame** — most rigid and repeatable
2. Mount all four **8520 motors** onto the arm tips using hot glue or zip-ties.
3. Verify motor orientation labeling (CW = black hub / CCW = silver hub in most packs).
4. **Do not attach propellers yet.**

> ⚠️ Keep all four arm lengths equal from centre. Asymmetry causes persistent yaw drift that no PID tuning can fully compensate.

---

### Step 2 — Build the Central Power Bus & Solder Motor Power Wires

1. On a small protoboard (your central power distribution board), create a **positive (+) solder bus** and a **negative (−) / GND solder bus**.
2. Run a single **red wire** from LiPo `+V` to the positive bus pad.
3. From the positive bus, solder the **red (VCC+) wire** of all four motors. Use **24 AWG minimum** on the power rail — this carries up to 4A peak (4 motors × 1A each).
4. **Twist each motor's red/black wire pair** together from the motor to the board — this reduces electromagnetic noise that can corrupt IMU readings.
5. Leave motor **black (GND−) wires** unconnected for now — they connect to MOSFET DRAINs in Step 3.

> ⚠️ **Do NOT connect the LiPo battery yet.** Connect it absolutely last, after all soldering is complete.

---

### Step 3 — Place and Solder the SI2302 MOSFETs

1. Place all four SI2302 MOSFETs on the protoboard, one per motor.
2. Identify the SOT-23 pinout (flat face toward you, pins downward):
   - **Left pin** = GATE
   - **Right pin** = SOURCE
   - **Middle-top pin** = DRAIN
3. For each MOSFET:
   - Solder **SOURCE** → GND rail
   - Solder **DRAIN** → its respective motor's **black (GND−)** wire
   - Leave **GATE** unconnected for now (connected in Step 5)
4. **(Recommended)** Solder a **10kΩ pull-down resistor** between each GATE pin and the GND rail. This prevents noise from floating gate pins causing spurious motor spin-up.

> **Motor → MOSFET DRAIN mapping:**
> - FL Motor (CW) → `mosfet_fl` DRAIN
> - FR Motor (CCW) → `mosfet_fr` DRAIN
> - BL Motor (CCW) → `mosfet_bl` DRAIN
> - BR Motor (CW) → `mosfet_br` DRAIN

> ⚠️ Never connect DRAIN to GND. Drain must only connect to the motor's negative wire.

---

### Step 4 — Mount and Wire the MPU6050 IMU

1. Find the **centre of mass** of your assembled frame (without battery). Mark this point.
2. Place a **1–2mm foam pad** at the centre of mass — this absorbs motor vibration that would corrupt gyro readings.
3. Mount the MPU6050 module (GY-521) on the foam pad, as **level** as possible. Even a 2° permanent tilt is correctable in firmware, but try to minimise it.
4. Solder four short wires (keep all I2C wires **under 10cm**):
   - `MPU6050 VCC` → `ESP32 3V3`
   - `MPU6050 GND` → GND rail
   - `MPU6050 SDA` → `ESP32 GPIO21`
   - `MPU6050 SCL` → `ESP32 GPIO22`
5. Leave the `INT` pin unconnected.

> ⚠️ The MPU6050 operates at **3.3V only**. Connecting VCC to 5V will permanently destroy the chip.

---

### Step 5 — Connect MOSFET Gates to ESP32 PWM Pins

1. Run four **signal wires** from the ESP32 to the MOSFET GATE pins:
   - `ESP32 GPIO13` → `mosfet_fl` GATE (Front-Left)
   - `ESP32 GPIO25` → `mosfet_fr` GATE (Front-Right)
   - `ESP32 GPIO26` → `mosfet_bl` GATE (Back-Left)
   - `ESP32 GPIO27` → `mosfet_br` GATE (Back-Right)
2. Use **26–28 AWG wire** for signal lines — low current.
3. Route signal wires **away from the high-current motor power wires** to minimise EMI interference.
4. Label each wire with coloured tape or marker (FL/FR/BL/BR) before routing.

---

### Step 6 — Power the ESP32

1. Connect a **red wire** from the LiPo `+V` power bus to the ESP32 `VIN` pin.
2. Connect a **black wire** from the GND rail to any `GND` pin on the ESP32.
3. The onboard **AMS1117-3.3** linear voltage regulator on the ESP32 DevKit will step down the 3.7V–4.2V LiPo voltage to 3.3V for the MCU core, Wi-Fi radio, and MPU6050.

> **Note on VIN voltage**: The AMS1117-3.3 has a typical dropout voltage of ~1V, so VIN must be at least ~4.3V for guaranteed regulation. A 3.7V LiPo (nominal) falls right at the edge — it works in practice but if your board behaves unreliably, add an **MT3608 boost converter** between LiPo+ and VIN, set to 5.0V output.

---

### Step 7 — Flash the Firmware

1. Connect the ESP32 to your computer via USB **before connecting the LiPo**.
2. Open the firmware in Arduino IDE or PlatformIO.
3. Edit the configuration constants in the firmware (PID gains, Wi-Fi credentials, GPIO pins) as needed.
4. Select the correct board and port.
5. Click **Upload**.
6. After upload, open **Serial Monitor** at `115200 baud`.
7. Verify you see IMU angle readings and system boot messages.
8. Disconnect USB.

---

### Step 8 — Final Wiring Check (Before First Power-On)

Before connecting the LiPo battery, perform this visual inspection:

- [ ] All four motor `VCC+` (red) wires connect to the power bus (not to MOSFET DRAINs)
- [ ] All four motor `GND-` (black) wires connect to MOSFET DRAINs (not directly to GND rail)
- [ ] All four MOSFET SOURCEs connect to GND rail
- [ ] All four MOSFET GATEs connect to the correct ESP32 GPIO (13/25/26/27)
- [ ] MPU6050 VCC connects to ESP32 `3V3` (NOT `5V` or `VIN`)
- [ ] MPU6050 SDA/SCL connect to GPIO21/22
- [ ] LiPo `+V` connects to ESP32 `VIN` and the motor power bus
- [ ] LiPo `GND` connects to GND rail
- [ ] No bare wire ends touching each other
- [ ] Propellers are **NOT** installed

Only after all boxes are checked: **connect the LiPo battery**.

---

## 🌐 Wi-Fi Control Interface

### Connecting

1. On your phone or tablet, open **Wi-Fi Settings**.
2. Connect to: **`Drone_WiFi`**
3. Password: **`drone1234`**
4. Open a browser and navigate to: **`http://192.168.4.1`**

The ESP32 serves a dual-joystick HTML5 page. No internet connection is required.

### Joystick Layout

```
┌──────────────────────┐  ┌──────────────────────┐
│                      │  │                      │
│    LEFT JOYSTICK     │  │    RIGHT JOYSTICK    │
│                      │  │                      │
│  ↑  Throttle (Up)    │  │  ↑  Pitch (Forward)  │
│  ↓  Throttle (Down)  │  │  ↓  Pitch (Backward) │
│  ←  Yaw (Left)       │  │  ←  Roll (Left)      │
│  →  Yaw (Right)      │  │  →  Roll (Right)     │
│                      │  │                      │
└──────────────────────┘  └──────────────────────┘
```

| Axis | Stick | Direction | Effect |
|------|-------|-----------|--------|
| Throttle | Left | Up/Down | Increases/decreases all motor speeds (altitude) |
| Yaw | Left | Left/Right | Rotates drone left/right around vertical axis |
| Pitch | Right | Up/Down | Tilts drone forward/backward (moves forward/backward) |
| Roll | Right | Left/Right | Tilts drone left/right (moves sideways) |

> **Mobile tip**: The joystick UI uses touch events and works on any modern mobile browser (Chrome, Safari, Firefox). For best experience, rotate your phone to **landscape orientation** before loading the page.

---

## 🛡️ Failsafe System

The drone implements an automatic **motor cutoff failsafe**:

- The ESP32 tracks the timestamp of every received joystick command.
- If **500 milliseconds** elapse without receiving a new command, all four motors are immediately set to **zero PWM**.
- This activates when:
  - Wi-Fi connection drops
  - Browser tab is closed
  - Phone goes to sleep / screen locks
  - Network interference causes packet loss

> The 500ms window is intentionally short to prevent the drone from flying away uncontrolled. If you experience false failsafe triggers from network lag, the `FAILSAFE_TIMEOUT_MS` constant can be increased in firmware, but do not exceed 1000ms.

---

## ✅ Pre-Flight Ground Test Procedure

> ⚠️ **Remove all propellers before this test.**

**Perform this test after first firmware flash and at the start of every session.**

1. Place drone on a flat surface. Do not hold it.
2. Connect the LiPo battery.
3. Wait **3 seconds** for IMU calibration (hold drone perfectly flat and still).
4. On your phone, connect to `Drone_WiFi` and open `http://192.168.4.1`.
5. **Throttle test**: Slowly raise the left joystick upward — all four motors should spin up proportionally. Verify all four motors respond.
6. **IMU test**: Keep throttle at ~30%. Tilt the drone frame by hand (roll axis, then pitch axis). Observe individual motors speeding up and slowing down to compensate for the tilt. If the wrong pair of motors responds, your motor-to-GPIO mapping may be inverted.
7. **Yaw test**: Move the left joystick left/right. The diagonal motor pairs should change speed in opposite directions.
8. **Failsafe test**: Disconnect Wi-Fi on your phone. All motors should stop within 500ms.
9. **Serial monitor check**: Open Arduino Serial Monitor at 115200 baud. Verify IMU angles report correctly (~0° roll, ~0° pitch when flat).

✅ If all six steps pass — you are ready to attach propellers and proceed outdoors.

---

## 📈 PID Tuning Guide

PID tuning is an iterative process. Make **one change at a time** and test between each change. Never change two values simultaneously.

### Understanding the Three Terms

| Term | Constant | Effect | Symptom of Too High | Symptom of Too Low |
|------|----------|--------|--------------------|--------------------|
| **Proportional (P)** | `KP_ROLL` / `KP_PITCH` | Immediate correction proportional to current error | Fast oscillation / shaking | Sluggish response, large drift |
| **Integral (I)** | `KI_ROLL` / `KI_PITCH` | Corrects accumulated steady-state error over time | Slow growing oscillation, wind-up | Persistent drift in one direction |
| **Derivative (D)** | `KD_ROLL` / `KD_PITCH` | Dampens the rate of change, prevents overshoot | High-frequency buzz / jitter | Overshoot, slow settling |

### Recommended Tuning Sequence

**Start with these safe initial values and work outward:**
```
KP = 0.8 | KI = 0.01 | KD = 0.5
```

#### Phase 1 — Tune KP (with KI=0, KD=0 initially)

1. Set `KI=0.0`, `KD=0.0`, `KP=0.5`
2. Hover (tethered). Observe stability.
3. If drone **oscillates rapidly** → reduce KP by 0.1
4. If drone **drifts but is stable** → increase KP by 0.1
5. Find the KP value just below where oscillation begins — that is your base KP.

#### Phase 2 — Tune KD (add derivative dampening)

1. Keep KP from Phase 1. Set `KI=0.0`.
2. Start with `KD=0.2` and increase in steps of 0.1.
3. KD should eliminate oscillation without adding high-frequency buzz.
4. If you hear a high-pitched buzz from the motors, KD is too high — reduce it.

#### Phase 3 — Tune KI (eliminate steady drift)

1. Keep KP and KD from Phases 1 & 2.
2. If the drone drifts slowly in one direction after hovering for 5+ seconds, increase KI by 0.005.
3. **KI should be very small** — typical values are 0.005 to 0.05. Large KI causes slow wind-up oscillation.
4. If adding KI causes the drone to slowly grow more unstable over time, reduce KI — it is winding up.

### Symptom Reference Chart

| Observed Behaviour | Most Likely Cause | Fix |
|--------------------|-------------------|-----|
| Rapid shaking / high-frequency oscillation | `KP` too high | Reduce `KP_ROLL` / `KP_PITCH` |
| Slow back-and-forth rocking | `KD` too low | Increase `KD_ROLL` / `KD_PITCH` |
| Drone drifts consistently in one direction | `KI` too low or IMU offset | Increase `KI`, or re-level IMU |
| Drone overreacts then overshoots | `KD` too low | Increase `KD` |
| Motors make a high-frequency buzz | `KD` too high or PWM freq too low | Reduce `KD`, increase `PWM_FREQ` |
| Slow oscillation that grows over time | `KI` too high (wind-up) | Reduce `KI` |
| One motor always faster/slower than others | Motor mapping error | Check GPIO assignment & MOSFET orientation |
| Drone spins (yaw) uncontrollably | Wrong propeller direction | Verify CW/CCW prop installation |

---

## 🛫 First Flight Procedure

> ⚠️ First flight must be performed **outdoors** or in a **large open space** with no people nearby.

1. **Install propellers** — verify CW props on GPIO13/27 motors and CCW props on GPIO25/26 motors.
2. **Tether the drone** — loosely loop an elastic cord around the frame. This allows the drone to lift but prevents it from flying away if control is lost.
3. Connect LiPo. Wait 3 seconds for IMU calibration.
4. Connect phone to `Drone_WiFi` and open `http://192.168.4.1`.
5. Stand behind the drone (rear facing you).
6. Raise throttle **slowly** from 0 to approximately 30%. Observe if all motors spin.
7. Continue raising throttle until the drone begins to feel light on the surface (hover threshold).
8. At the hover throttle point, observe stability. Note any persistent oscillation or drift.
9. Land immediately and adjust PID values as described above.
10. Repeat with tether until the drone hovers stably for 30 seconds without intervention.
11. Remove tether only after stable untethered hover is confirmed.

> 🔋 **Monitor battery voltage continuously** during tuning. Do not discharge below **3.2V per cell**. Use a LiPo alarm buzzer attached to the battery balance connector.

---

## 🔍 Troubleshooting

### Drone Does Not Connect to `Drone_WiFi`

| Check | Action |
|-------|--------|
| Firmware flashed successfully? | Re-flash and verify Serial output shows AP started |
| SSID visible in Wi-Fi scan? | If not, ESP32 is not booting — check VIN power |
| Browser shows "cannot reach 192.168.4.1"? | Ensure phone is connected to `Drone_WiFi`, not home router |
| IP conflict? | Try clearing browser cache; some phones use alternate DNS |

### Motors Do Not Spin

| Check | Action |
|-------|--------|
| LiPo connected and charged? | Check voltage with multimeter (should be 3.7–4.2V) |
| PWM signal present on GPIO? | Use multimeter or oscilloscope to verify AC signal on GPIO13/25/26/27 when throttle is raised |
| MOSFET orientation correct? | Recheck flat-face orientation: Gate=left, Source=right, Drain=top |
| DRAIN and SOURCE not swapped? | Swapping DRAIN/SOURCE = no switching, MOSFET acts as open circuit |
| Motor wires polarity correct? | Swap VCC+/GND- if motor spins wrong direction |

### Only Some Motors Spin

| Check | Action |
|-------|--------|
| Individual MOSFET failure? | Test each GPIO with a multimeter — verify PWM signal present |
| Bad solder joint? | Re-flow solder on DRAIN/GATE pins of the non-responding MOSFET |
| Motor itself faulty? | Connect motor directly to LiPo briefly (carefully) to test |

### Drone Lifts But Immediately Flips

| Check | Action |
|-------|--------|
| Motor rotation direction correct? | Verify CW/CCW as per the layout table |
| Propeller direction correct? | CW prop on CW motor, CCW prop on CCW motor |
| Motor-to-GPIO mapping correct? | Swap FL/BR or FR/BL connections if drone flips on a consistent axis |
| IMU mounted backwards? | Rotate MPU6050 180° and update `AXIS_INVERT` flag in firmware |

### IMU Reads Wrong Angles / Drone Drifts Badly

| Check | Action |
|-------|--------|
| MPU6050 on foam pad? | Vibrations cause gyro drift — foam isolation is critical |
| IMU mounted level? | Use a spirit level; permanently tilted IMU offsets readings |
| Calibration at boot? | Hold drone perfectly still and flat for 3 seconds after power-on |
| I2C wires too long? | Keep SDA/SCL under 10cm at 3.3V levels |
| Signal noise from motor wires? | Route I2C wires away from motor power wires |

### Serial Monitor Shows No Output

| Check | Action |
|-------|--------|
| Baud rate set to 115200? | Verify baud rate in Serial Monitor dropdown |
| Wrong COM port selected? | Check Device Manager (Windows) or `/dev/tty*` (Linux/Mac) |
| USB cable is data cable (not charge-only)? | Try a different cable |

---

## ⚠️ Safety Warnings

> **Read all warnings before powering on the drone for the first time.**

### LiPo Battery Safety
- ⚡ **Never short-circuit the LiPo.** Even briefly, a short can cause fires, burns, or explosion.
- 🔥 **Never charge unattended.** Use only a purpose-built TP4056 or balance charger.
- 🌡️ **Never charge a hot or swollen LiPo.** A swollen (puffy) cell must be safely disposed of.
- 📉 **Never discharge below 3.2V per cell.** Deep discharge permanently damages LiPo cells.
- 🔌 **Connect the battery LAST**, after all soldering and wiring is verified.
- 🚫 **Never plug LiPo into VIN and USB simultaneously** on the ESP32 DevKit.
- 🗑️ **Dispose of LiPo cells at a certified battery recycling point**, not in household waste.

### Motor & Propeller Safety
- 🚨 **ALWAYS remove propellers during bench testing, wiring, and firmware flashing.**
- 💥 An unintended full-throttle spin-up with propellers attached can cause **serious lacerations**.
- 👐 Never hold the drone while motors are powered with propellers attached.
- 👓 Wear **safety glasses** during any tethered hover test.

### Electrical Safety
- 🔌 Complete all soldering **before** connecting the battery.
- 🔍 Perform a full visual wiring check before every power-on.
- ❌ Never connect MPU6050 VCC to 5V — the chip will be permanently destroyed.
- ❌ Never connect MOSFET DRAIN to GND — this bypasses the switching function entirely.

### Flight Safety
- 🚫 **Never fly over or near people**, especially during PID tuning.
- 🌳 Always fly in an **open outdoor area** on first flights.
- 🧵 Use a tether (elastic cord) on all tuning flights.
- 📵 Keep the control phone away from other Wi-Fi networks that could cause interference.
- 🔋 Land immediately when the LiPo alarm sounds (below 3.2V).

---

## 🔋 Power Budget & Runtime Estimate

| Component | Typical Current Draw |
|-----------|---------------------|
| ESP32 MCU + Wi-Fi | ~150–240 mA |
| MPU6050 IMU | ~3.9 mA |
| 4× 8520 Motors (hover ~50% throttle) | ~800 mA total (~200 mA each) |
| 4× 8520 Motors (full throttle) | ~4000 mA total (~1A each) |
| **Total at hover** | **~1000–1050 mA** |
| **Total at full throttle** | **~4240 mA** |

**Battery**: 1000 mAh × 80% usable capacity (to protect LiPo) = **800 mAh effective**

**Estimated hover runtime**: 800 mAh ÷ 1050 mA ≈ **~46 minutes theoretical**

> **Real-world estimate**: 4–8 minutes. Hover is never at constant 50% throttle; aggressive maneuvers, altitude holds, and wind push motors to 70–90%. Real hover for a micro drone of this class is typically **5–10 minutes** per charge.

---

## 📁 Project File Structure

```
esp_go/
├── README.md                    ← You are here
├── firmware/
│   ├── esp_go/
│   │   ├── esp_go.ino           ← Main Arduino sketch
│   │   ├── config.h             ← All #define constants (PID, pins, Wi-Fi)
│   │   ├── pid.h                ← PID controller class
│   │   ├── pid.cpp
│   │   ├── motor_mix.h          ← Motor mixing functions
│   │   ├── motor_mix.cpp
│   │   ├── imu_handler.h        ← MPU6050 wrapper
│   │   ├── imu_handler.cpp
│   │   ├── wifi_server.h        ← AP setup + WebSocket server
│   │   ├── wifi_server.cpp
│   │   └── data/
│   │       └── index.html       ← Dual joystick control page (served by ESP32)
├── hardware/
│   ├── schematic.pdf            ← Full wiring schematic
│   ├── bom.csv                  ← Bill of materials (machine-readable)
│   └── frame/
│       ├── frame_dimensions.png ← Frame arm length guide
│       └── motor_mount.stl      ← 3D printable motor mount (optional)
├── docs/
│   ├── wiring_diagram.png
│   ├── motor_layout.png
│   └── pid_tuning_guide.md
└── tools/
    └── lipo_monitor/            ← Optional serial LiPo voltage logger
```

---

## 🤝 Contributing

Contributions are welcome! Here's how to help:

1. **Fork** this repository
2. Create a **feature branch**: `git checkout -b feature/yaw-pid-improvement`
3. Make your changes with clear commit messages
4. **Test on hardware** before submitting a PR — untested firmware changes will not be merged
5. Open a **Pull Request** with a description of what changed and why

### Areas Where Help Is Needed

- [ ] Altitude hold using barometer (BMP280)
- [ ] Yaw PID improvement (currently open-loop yaw)
- [ ] OTA (Over-the-Air) firmware updates
- [ ] Battery voltage monitoring via ADC
- [ ] Android/iOS native app alternative to browser joystick
- [ ] Frame 3D print files
- [ ] Video tutorial series

---

## 📄 License

This project is licensed under the **MIT License** — you are free to use, modify, and distribute this project for personal and commercial purposes, with attribution.

```
MIT License

Copyright (c) 2024 Md Nazmun Nur

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
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

## 🙏 Acknowledgements

- [**MPU6050_light library**](https://github.com/rfetick/MPU6050_light) by rfetick — clean, lightweight complementary filter IMU driver
- [**WebSockets library**](https://github.com/Links2004/arduinoWebSockets) by Links2004 — reliable WebSocket implementation for ESP32/ESP8266
- [**Espressif ESP32 Arduino Core**](https://github.com/espressif/arduino-esp32) — the foundation that makes ESP32 development accessible
- The open-source drone and RC communities for decades of PID tuning knowledge

---

<div align="center">

**Built with ❤️ by Md Nazmun Nur**

*If this project helped you build your first drone, consider giving it a ⭐ — it helps others find the project!*

</div>
