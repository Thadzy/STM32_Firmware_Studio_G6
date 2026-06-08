# Pick & Place Robot — Firmware

STM32G474RE firmware for a single-axis pick-and-place robot arm. Implements a cascaded PID motion controller with S-curve trajectory planning, ZVD input shaping, Kalman filtering, Modbus RTU communication, and an optional Bluetooth gamepad interface.

---

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)
2. [Software Prerequisites](#2-software-prerequisites)
3. [Installation & First Build](#3-installation--first-build)
4. [Flashing to the Board](#4-flashing-to-the-board)
5. [Project Structure](#5-project-structure)
6. [Firmware Architecture](#6-firmware-architecture)
7. [Debugging with Live Expressions](#7-debugging-with-live-expressions)
8. [Variable Reference (`g_robot`)](#8-variable-reference-g_robot)
9. [Fault Codes](#9-fault-codes)
10. [Modbus Register Map](#10-modbus-register-map)
11. [Tunable Parameters (`params.h`)](#11-tunable-parameters-paramsh)
12. [PC Backend](#12-pc-backend)
13. [Joystick Control](#13-joystick-control)
14. [Operating Modes](#14-operating-modes)

---

## 1. Hardware Requirements

| Component | Part / Spec |
|-----------|-------------|
| MCU board | NUCLEO-G474RE (STM32G474RET6) |
| Motor driver | H-bridge, 24 V supply |
| Motor | DC motor with encoder |
| Encoder | ATM103 — 2048 CPR, TIM3 quadrature ×4 = 8192 counts/rev |
| Current sensor | WCS1800 (±35 A, 0.040 V/A @ 5 V) on PA0 |
| Gripper actuator | Up/Down + Open/Close solenoid relays |
| Reed switches | Up, Down, Open, Close positions |
| Proximity sensor | Homing reference sensor |
| E-Stop | Normally-closed (NC) pushbutton on PA5 |
| Reset button | Momentary pushbutton |
| Mode switch | Toggle: Base system / Joystick |
| Joystick | ESP32 Bluetooth gamepad → USART3 (115200 8N1) |
| PC link | USB-serial via LPUART1 (230400 8N1) |
| Power supply | 24 V DC, motor; 3.3 V / 5 V logic |

---

## 2. Software Prerequisites

### Firmware (embedded)

| Tool | Version | Notes |
|------|---------|-------|
| STM32CubeIDE | 1.14 or later | Eclipse CDT, manages the build |
| STM32CubeMX | Bundled with CubeIDE | Do **not** re-generate without reading CLAUDE.md first |
| arm-none-eabi-gcc | Bundled with CubeIDE | Compiler flags: `-DUSE_HAL_DRIVER -DSTM32G474xx -DDEBUG -mfpu=fpv4-sp-d16 -mfloat-abi=hard -g3` |
| ST-Link V2 driver | Latest | For OpenOCD flashing via USB |

### PC Backend (optional)

```
Python >= 3.9
pyserial >= 3.5
websockets >= 10.0
pymodbus >= 3.0
```

---

## 3. Installation & First Build

### Clone / open in STM32CubeIDE

1. Open STM32CubeIDE.
2. **File → Open Projects from File System** → select the `Pick_and_Place_Robot` folder.
3. CubeIDE detects the `.project` / `.cproject` files automatically.

### Build

```
Project → Build Project   (Ctrl + B)
```

Output binary: `Debug/Pick_and_Place_Robot.elf`

> **No Makefile / cmake path exists.** Running `make` from a terminal will fail — the IDE manages its own internal build system.

### PC Backend setup & Running the System

First time setup:
```cmd
cd PC_Backend
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

**To run the system (Dashboard + Serial Bridge):**
Open a Command Prompt, navigate to the project root, and execute the batch script:
```cmd
cd c:\Users\Lenovo\Documents\Thadzy\Pick_and_Place_Robot
run_system.bat
```
This script will automatically start the Base System UI and the Serial Bridge together in a split-pane view.

---

## 4. Flashing to the Board

1. Connect the NUCLEO board via USB (ST-Link port).
2. In STM32CubeIDE: **Run → Debug** (F11).
3. OpenOCD programs `Debug/Pick_and_Place_Robot.elf` via ST-Link V2.
4. Click **Resume** (F8) to start execution.

The IWDG (watchdog) is frozen in debug mode (`DBGMCU_APB1FZR1_DBG_IWDG_STOP` is set), so breakpoints will not trigger a reset.

---

## 5. Project Structure

```
Pick_and_Place_Robot/
│
├── Core/
│   ├── Inc/                        # All header files
│   │   ├── params.h                # Every tunable constant — edit here only
│   │   ├── system_state.h          # g_robot struct — the single system view
│   │   ├── app_main.h              # App_Init / App_Run declarations
│   │   ├── motor_controller.h      # Motion API
│   │   ├── hw_io.h                 # GPIO, ADC, actuator API
│   │   ├── uart_dma_manager.h      # TX ring buffer, RX DMA API
│   │   ├── modbus_bridge.h         # Modbus register read/write API
│   │   ├── kalman.h                # Kalman filter API
│   │   ├── joystick.h              # ESP32 gamepad parser API
│   │   └── main.h                  # CubeMX-generated HAL handle declarations
│   │
│   └── Src/                        # All source files
│       ├── app_main.c              # Top-level FSM, homing, gripper sequencer
│       ├── motor_controller.c      # Cascade PID + S-curve + ZVD + Kalman
│       ├── hw_io.c                 # Debounce, ADC, relay/motor/gripper drivers
│       ├── uart_dma_manager.c      # TX ring buffer, RX idle-DMA, Modbus guard
│       ├── modbus_bridge.c         # Modbus RTU parser (slave addr 21)
│       ├── kalman.c                # 4-state Kalman filter (pos/vel/acc/jerk)
│       ├── joystick.c              # ESP32 Bluetooth gamepad UART parser
│       ├── stm32g4xx_it.c          # ISR hooks — TIM6, UART DMA, LPUART1
│       └── main.c                  # CubeMX-generated — thin: App_Init + App_Run only
│
├── Drivers/
│   ├── STM32G4xx_HAL_Driver/       # ST HAL (do not edit)
│   └── CMSIS/                      # ARM CMSIS headers (do not edit)
│
├── PC_Backend/
│   ├── serial_bridge.py            # USB-serial demultiplexer + Modbus forwarder
│   ├── test_robot.py               # Automated motion/gripper test script
│   ├── homing_log.py               # Homing telemetry decoder
│   ├── diag_com10.py               # COM port diagnostic tool
│   └── requirements.txt
│
├── CLAUDE.md                       # AI assistant instructions for this repo
├── CONTEXT.md                      # Full architecture specification
└── README.md                       # This file
```

---

## 6. Firmware Architecture

### Execution model

```
┌─────────────────────────────────────────────────────────────────┐
│  TIM6 ISR  (1 kHz, every 1 ms)                                  │
│                                                                  │
│  Every tick (1 kHz):                                             │
│    MotorCtrl_Tick1kHz()                                          │
│      ├── Read TIM3 encoder → update position counts             │
│      ├── Kalman filter: predict + update (pos/vel/acc/jerk)     │
│      ├── Inner velocity PID → compute PWM                       │
│      ├── Velocity + acceleration feedforward                     │
│      ├── Software safety stack (encoder health, overcurrent,    │
│      │   tracking error)                                        │
│      └── Motor_SetPWM()                                         │
│                                                                  │
│  Every 10th tick (100 Hz, every 10 ms):                          │
│    HwIo_Poll100Hz()                                              │
│      ├── Debounce E-Stop, reed switches, proximity, reset btn   │
│      └── Latch proximity rising edge for homing                 │
│    MotorCtrl_Tick100Hz()                                         │
│      ├── S-curve trajectory integrator                          │
│      ├── ZVD input shaper (3-impulse filter)                    │
│      └── Outer position PID → velocity setpoint for inner loop  │
│                                                                  │
│  Watchdog kick: IWDG->KR = 0xAAAA  (50 ms timeout)             │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  Main Loop  App_Run()  (runs as fast as possible)               │
│                                                                  │
│  1. Poll non-ISR sensors into g_robot                           │
│  2. Detect mode-switch toggle (base / joystick)                 │
│  3. Heartbeat timeout watchdog (2 s, soft-stop if PC silent)    │
│  4. Update g_robot.dbg mirror (visible in Live Expressions)     │
│  5. UartDma_Process()  — drain RX ring → Modbus callbacks       │
│  6. ModbusBridge_Tick() — refresh read registers                │
│  7. gripper_seq_run()  — advance gripper state machine          │
│  8. handle_joystick()  — translate gamepad inputs to commands   │
│  9. fsm_run()          — main application FSM                   │
│ 10. Relay_SetStatus()  — pilot lamp (ON in FAULT)               │
│ 11. Audio feedback on FSM state transitions                     │
│ 12. $ST telemetry every 2 s                                     │
└─────────────────────────────────────────────────────────────────┘
```

### Application FSM (`app_main.c`)

```
         power-on
             │
         STATE_INIT  (single pass)
             │
         STATE_IDLE ◄──────────────────────────────────────────┐
             │                                                  │
     ┌───────┼───────────────────────────────┐                 │
     │       │                               │                 │
  mode=Home mode=Jog / P2P cmd         mode=Auto              │
     │       │                               │                 │
 STATE_HOMING  STATE_RUNNING ──── at target ─┘ (auto loops)   │
     │             │                                           │
  (homing         soft-stop / timeout / all pairs done ────────┘
  complete)
             │
         STATE_FAULT  (latched — clear only with E-stop released + Reset btn)
```

### Homing sequence (`app_main.c` — `homing_run()`)

Two-stage industrial homing: a fast coarse pass finds the sensor, then a slow
precision pass captures both edges at one identical velocity so sensor-latency
bias cancels in the (A + B) / 2 midpoint.

```
HOM_INIT → HOM_FAST_SEARCH → HOM_BACKOFF → HOM_PREC_EDGE_A
        → HOM_PREC_OVERSHOOT → HOM_FIND_EDGE_B → HOM_GO_CENTER → HOM_SETTLE
```

| Stage | What happens |
|-------|-------------|
| INIT | Flush proximity latch; begin FAST creep in +1 direction |
| FAST_SEARCH | Growing triangular sweep at `HOMING_FAST_VEL_RADS` until sensor found (edge discarded — coarse find only) |
| BACKOFF | Reverse at precision vel until prox OFF + `HOMING_BACKOFF_DEG` margin (clean OFF start) |
| PREC_EDGE_A | Slow steady approach at `HOMING_PREC_VEL_RADS`; rising edge = Edge A |
| PREC_OVERSHOOT | Continue past Edge A until sensor physically clears (avoids false Edge B) |
| FIND_EDGE_B | Reverse at precision vel; next rising edge from opposite side = Edge B |
| GO_CENTER | Creep to (A + B) / 2; timeout after 30 s |
| SETTLE | Hold 1 s; call `MotorCtrl_Zero()` to define home; move to park position |

### Control loop (cascade PID)

```
Position target (rad)
        │
   [S-curve trajectory]   — limits velocity, accel, jerk
        │
   [ZVD input shaper]     — 3-impulse filter, removes 12.13 Hz resonance
        │
   velocity setpoint (rad/s)
        │  (outer PID, 100 Hz)
   [Position PID]  Kp=3.0  Ki=0.02  Kd=0.15
        │
   velocity command (rad/s)
        │  (inner PID, 1 kHz)
   [Velocity PID + feedforward]  Kp=1.58  Ki=0.5  Kd=0.0  FF=3.03
        │
   PWM command (−50 … +50)
        │
   [Motor H-bridge]  TIM1, 19.6 kHz, 24 V supply
        │
   [Encoder TIM3]  8192 counts/rev
        │
   [4-state Kalman]  estimates pos / vel / acc / jerk at 1 kHz
        └──────────────────────────────────────────────────────┘
```

### HAL peripherals

| Handle | Used for |
|--------|---------|
| `htim1` | Motor PWM output (19.6 kHz, ARR=50) |
| `htim3` | Encoder quadrature input (TI12 mode) |
| `htim6` | 1 kHz base-rate interrupt — all control loops |
| `hlpuart1` | PC serial link (230400 baud, DMA RX/TX) |
| `huart3` | ESP32 joystick (115200 baud, DMA RX/TX) |
| `hadc1` | Current sensor (WCS1800), DMA continuous |
| `hfdcan1` | Reserved (not used in current firmware) |

---

## 7. Debugging with Live Expressions

Live Expressions is the fastest way to watch the robot in real time without adding `printf` or halting execution.

### Setup (one-time)

1. Start a debug session: **Run → Debug** (F11) then **Resume** (F8).
2. Open the **Live Expressions** view: **Window → Show View → Live Expressions**.
3. Click **Add new expression** and type:

```
g_robot
```

4. Expand the `g_robot` tree. Every sub-struct updates automatically while the program runs.

### What to watch and when

| Scenario | Expression to expand | Key fields |
|----------|---------------------|-----------|
| Motor moving | `g_robot.motion` | `position_counts`, `velocity_rps`, `motor_pwm` |
| Easy-to-read position | `g_robot.dbg` | `pos_deg`, `vel_dps` |
| FSM state | `g_robot` | `fsm` (0=INIT 1=HOMING 2=IDLE 3=RUNNING 4=FAULT) |
| Gripper progress | `g_robot.dbg` | `grip` (see values below) |
| Run mode | `g_robot.dbg` | `run_mode` (0=IDLE 1=JOG 2=AUTO 3=POINT 4=TEST) |
| Joystick | `g_robot.dbg` | `joy_mode`, `joy_btn`, `joy_conn` |
| Safety guards | `g_robot.dbg.safety` | `en_*` (enable flags), `tripped_*` (fault latches) |
| E-Stop / reed states | `g_robot.sensors` | `estop`, `reed_up`, `reed_down`, `reed_open`, `reed_close` |
| Current draw | `g_robot.sensors` | `current_amps`, `raw_adc` |
| Comms health | `g_robot.comms` | `fault_code`, `tx_used`, `telemetry_drops`, `hb_age_ms` |

### Disabling a safety guard during commissioning

In the **Live Expressions** or **Variables** view, navigate to `g_robot.dbg.safety` and set the corresponding `en_*` field to `0` (false):

```
g_robot.dbg.safety.en_encoder_health   = 0   // disable encoder-disconnect guard
g_robot.dbg.safety.en_current_safety   = 0   // disable overcurrent fuse
g_robot.dbg.safety.en_tracking_safety  = 0   // disable jam/tracking guard
```

Re-enable by setting back to `1`. The guards reset automatically on a fault clear.

### Clearing a FAULT in the debugger

In **Live Expressions** set:
```
g_robot.fsm              = 2    // STATE_IDLE
g_robot.comms.fault_code = 0
```

Or press the physical reset button while E-Stop is released.

### Live PID Tuning (Without Recompiling)

You can adjust PID gains, S-curve limits, and Kalman noise parameters on-the-fly:

1. Add `TuningParams` to your Live Expressions.
2. Expand `TuningParams`. It mirrors the defaults from `params.h` on boot.
3. Edit the values directly in the debugger:
   * **Velocity Loop (Inner):** `TuningParams.spd_pid.kp`, `.ki`, `.kd`
   * **Position Loop (Outer):** `TuningParams.pos_pid.kp`, `.ki`, `.kd`
   * **S-Curve Limits:** `TuningParams.scurve.vmax_rads`, `.amax_rads2`, `.jmax_rads3`
4. Once you find the perfect values (e.g., settling time < 0.5s), copy them into `Core/Inc/params.h` so they become the permanent defaults.

---

## 8. Variable Reference (`g_robot`)

`g_robot` is the single global `RobotState_t` instance declared in `system_state.h`. All modules read and write into it.

### `g_robot.fsm`

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `STATE_INIT` | Transient — clears to IDLE after first `App_Run` |
| 1 | `STATE_HOMING` | Homing sequence running |
| 2 | `STATE_IDLE` | Ready — accepting new commands |
| 3 | `STATE_RUNNING` | Motion or auto sequence in progress |
| 4 | `STATE_FAULT` | Latched fault — requires reset |

### `g_robot.motion` (written from TIM6 ISR, 1 kHz)

| Field | Type | Meaning |
|-------|------|---------|
| `position_counts` | `int64_t` | Encoder counts from home (8192 counts = 360°) |
| `velocity_rps` | `float` | Revolutions per second |
| `accel_rps2` | `float` | Rev per second squared |
| `motor_pwm` | `int16_t` | PWM command (−50 = full reverse, +50 = full forward) |

### `g_robot.sensors` (written from ISR at 100 Hz and from main loop)

| Field | Type | Meaning |
|-------|------|---------|
| `estop` | `bool` | True = E-Stop active (latches after 80 ms LOW) |
| `reed_up` | `bool` | Gripper arm is in UP position |
| `reed_down` | `bool` | Gripper arm is in DOWN position |
| `reed_open` | `bool` | Claw is OPEN |
| `reed_close` | `bool` | Claw is CLOSED |
| `proximity` | `bool` | Proximity sensor active (homing reference) |
| `current_amps` | `float` | EMA-filtered motor current in Amperes |
| `raw_adc` | `uint16_t` | Raw ADC count from current sensor (0–4095) |
| `v_zero` | `float` | Auto-calibrated zero voltage (stable after boot) |

### `g_robot.outputs`

| Field | Type | Meaning |
|-------|------|---------|
| `gripper_up` | `bool` | True = arm UP relay energised |
| `claw_closed` | `bool` | True = claw CLOSED relay energised |
| `motor_enabled` | `bool` | True = motor-power relay energised |

### `g_robot.comms`

| Field | Type | Meaning |
|-------|------|---------|
| `heartbeat` | `uint16_t` | Mirror of Modbus reg 0x00 (22881 = "YA") |
| `fault_code` | `uint8_t` | Reason for STATE_FAULT (see §9) |
| `tx_used` | `uint16_t` | TX ring buffer bytes in use (max 1024) |
| `t35_active` | `bool` | True during Modbus inter-frame gap (2 ms) |
| `telemetry_drops` | `uint32_t` | Telemetry strings dropped (TX buffer > 800 bytes) |
| `last_rx_len` | `uint16_t` | Last LPUART1 DMA packet length |

### `g_robot.dbg` (debug mirror — updated every `App_Run`)

| Field | Type | Decoded values |
|-------|------|---------------|
| `fsm` | — | Same as `g_robot.fsm` |
| `run_mode` | `uint8_t` | 0=IDLE 1=JOG 2=AUTO 3=POINT 4=TEST |
| `grip` | `uint8_t` | 0=IDLE 1=PICK_DOWN 2=PICK_CLOSE 3=PICK_UP 4=PLACE_DOWN 5=PLACE_OPEN 6=PLACE_UP |
| `joy_mode` | `uint8_t` | 0=base system 1=joystick |
| `joy_btn` | `char` | ASCII: `'O'`=idle `'L'`=left `'R'`=right `'A'`=pick `'B'`=place `'U'`=up `'D'`=down `'Y'`=home `'X'`=e-stop |
| `joy_conn` | `uint8_t` | 0=gamepad not paired 1=connected |
| `pos_deg` | `float` | Motor position in degrees (easier than counts) |
| `vel_dps` | `float` | Velocity in degrees per second |
| `hb_age_ms` | `uint16_t` | Milliseconds since last heartbeat register change |

### `g_robot.dbg.safety`

| Field | Meaning |
|-------|---------|
| `en_encoder_health` | Enable encoder-disconnect guard |
| `en_current_safety` | Enable overcurrent fuse |
| `en_tracking_safety` | Enable mechanical jam detection |
| `tripped_encoder` | Fault 0x40 — encoder disconnect latched |
| `tripped_boundary` | Fault 0x41 — cable / position limit hit |
| `tripped_current` | Fault 0x42 — overcurrent latched |
| `tripped_tracking` | Fault 0x43 — tracking error / jam latched |

---

## 9. Fault Codes

`g_robot.comms.fault_code` is set when `g_robot.fsm` transitions to `STATE_FAULT`.

| Code | Source | Meaning |
|------|--------|---------|
| `0x01` | E-Stop | Hardware E-Stop activated |
| `0x02` | Homing | Edge B not found within search range (40°) |
| `0x03` | Homing | Overshoot too large — sensor never cleared |
| `0x04` | Homing | Sensor not found within full sweep (180°) |
| `0x10` | Joystick | Joystick emergency button (`X` / `P`) pressed |
| `0x20` | Comms | PC heartbeat timeout (silent > 2 s) — soft-stop |
| `0x40` | Safety | Encoder disconnect (high PWM, zero delta counts) |
| `0x41` | Safety | Boundary / cable limit exceeded |
| `0x42` | Safety | Overcurrent (> 2.0 A for 100 ms) |
| `0x43` | Safety | Tracking error (> 10° for 50 ms) — jam detected |

**Clearing a fault:** Release the E-Stop then press the physical Reset button. The FSM returns to `STATE_IDLE` and all `tripped_*` flags clear.

---

## 10. Modbus Register Map

Slave address: **21** (0x15). Baud rate: **230400**. Protocol: **RTU**.

### Write registers — PC → Robot (function code 0x06 / 0x10)

| Addr | Meaning |
|------|---------|
| 0x00 | Heartbeat: PC writes 18537 ("HI") to confirm; robot writes 22881 ("YA") |
| 0x01 | Mode: `1`=Home `2`=Jog `4`=Auto `8`=SetHome `16`=Test |
| 0x02 | Manual gripper: `0`=Up `1`=Down `2`=Open `4`=Close |
| 0x03 | Gripper sequence: `1`=Pick `2`=Place |
| 0x04 | Auto gripper enable: bit0 = enable gripper in auto mode |
| 0x05 | Jog step (int16, degrees; positive=CCW, negative=CW) |
| 0x06 | Test type: `0`=Precision `1`=Performance |
| 0x07 | Performance test target velocity (int16, rad/s × 10) |
| 0x08 | Performance test target acceleration (int16, rad/s² × 10) |
| 0x09 | Precision test initial position (int16, degrees) |
| 0x10 | Precision test final position (int16, degrees) |
| 0x11 | Precision test repeat count |
| 0x12–0x21 | Pick & place sequence slots (int16; magnitude=index, sign=direction) |
| 0x22 | Number of pick–place pairs (max 8) |
| 0x23 | P2P unit: `0`=degree `1`=index (5° per index slot) |
| 0x24 | P2P target (int16; write to trigger immediate move, auto-cleared) |
| 0x25 | Soft stop: bit0 = stop current motion and return to IDLE |
| 0x32 | Home offset (int16, degrees; applied after homing zeroes position) |

### Read registers — Robot → PC (function code 0x03, block 0x00 + 0x26–0x31)

| Addr | Meaning |
|------|---------|
| 0x00 | Heartbeat (robot writes 22881 / "YA") |
| 0x26 | Reed sensors: bit0=Reed_Up bit1=Reed_Down bit2=Reed_Close(jaw) |
| 0x27 | Task: bit0=Homing bit1=GoPick bit2=GoPlace bit3=GoPoint; `0`=Idle |
| 0x28 | Position (int16 × 10, degrees) |
| 0x29 | Velocity (int16 × 10, degrees/s) |
| 0x30 | Acceleration (int16 × 10, degrees/s²) |
| 0x31 | Emergency: bit0 = E-Stop active |

---

## 11. Tunable Parameters (`params.h`)

All constants live in `Core/Inc/params.h`. Items marked `TODO` need hardware calibration before production use.

### Timing

| Constant | Value | Meaning |
|----------|-------|---------|
| `CONTROL_LOOP_HZ` | 1000 | Inner loop rate (TIM6) |
| `OUTER_LOOP_HZ` | 100 | Outer loop rate (every 10th TIM6 tick) |

### Motor & Encoder

| Constant | Value | Meaning |
|----------|-------|---------|
| `MOTOR_PWM_MAX` | 50 | TIM1 ARR — 100% duty cycle (24 V) |
| `MOTOR_VOLT_LIMIT_PWM` | 12 | Software duty cap (6 V / 24 V × 50) — raise after commissioning |
| `ENCODER_CPR` | 8192 | Counts per revolution (2048 CPR × 4 quadrature) |
| `ENCODER_DIRECTION` | −1 | Flip to +1 if motor runs away on first test |
| `MAX_POSITION_COUNTS` | 8192 | 360° workspace limit |
| `CABLE_MAX_COUNTS` | 12288 | 540° absolute cable limit — never command past this |

### PID Gains (hardware-identified — do not change without re-identification)

| Constant | Value | Units |
|----------|-------|-------|
| `PID_SPEED_KP` | 1.58 | PWM/(rad/s) |
| `PID_SPEED_KI` | 0.5 | PWM/(rad/s) |
| `PID_SPEED_KD` | 0.0 | — |
| `PID_POS_KP` | 3.0 | (rad/s)/rad |
| `PID_POS_KI` | 0.02 | — |
| `PID_POS_KD` | 0.15 | — |
| `FF_VELOCITY` | 3.03 | PWM/(rad/s) |
| `FF_ACCEL` | 0.1 | PWM/(rad/s²) |

### S-Curve Trajectory Limits

| Constant | Value | Units |
|----------|-------|-------|
| `SCURVE_VMAX_RADS` | 7.304 | rad/s (≈ 418°/s) |
| `SCURVE_AMAX_RADS2` | 27.49 | rad/s² |
| `SCURVE_JMAX_RADS3` | 1400 | rad/s³ |

### ZVD Input Shaper (hardware-identified)

| Constant | Value | Meaning |
|----------|-------|---------|
| `ZVD_NATURAL_FREQ_HZ` | 12.13 | Resonance frequency of the arm |
| `ZVD_DAMPING_RATIO` | 0.041 | Damping ratio |
| `ZVD_A1 / A2 / A3` | 0.2832 / 0.4981 / 0.2189 | Impulse amplitudes |
| `ZVD_T2_STEPS` | 4 | Delay of 2nd impulse (at 100 Hz = 40 ms) |
| `ZVD_T3_STEPS` | 8 | Delay of 3rd impulse (at 100 Hz = 80 ms) |

### Software Safety Stack

| Constant | Value | Meaning |
|----------|-------|---------|
| `CURRENT_FAULT_AMPS` | 2.0 A | **TODO** — calibrate to actual motor stall current |
| `SAFETY_CURRENT_MS` | 100 ms | Overcurrent must persist this long before fault |
| `SAFETY_TRACKING_DEG` | 10° | Position error threshold for jam detection |
| `SAFETY_TRACKING_MS` | 50 ms | Jam must persist this long before fault |
| `SAFETY_ENC_STALL_PWM` | 5 | PWM threshold to arm stall guard |
| `SAFETY_ENC_STALL_MS` | 200 ms | Stall must persist this long before fault |

### Heartbeat & Comms

| Constant | Value | Meaning |
|----------|-------|---------|
| `HEARTBEAT_TIMEOUT_MS` | 2000 | PC link silent → soft-stop, fault code 0x20 |
| `UART_TX_BUF_SIZE` | 1024 | TX ring buffer size in bytes |
| `UART_TX_HIGH_WATERMARK` | 800 | Telemetry dropped above this fill level |

---

## 12. PC Backend

`PC_Backend/serial_bridge.py` demultiplexes the single USB serial port shared by telemetry and Modbus.

```
USB serial (LPUART1, 230400 baud)
        │
   serial_bridge.py
        ├── packets starting with '$' and ending '\r\n'
        │       └── forward → WebSocket port 8765  (React dashboard)
        └── all other packets (Modbus RTU frames)
                ├── validate CRC-16
                └── if valid → forward to COM10 (virtual loopback)
                              main.exe listens on COM11
```

**Run the Base System & Bridge (All-in-one):**
You can run both the base system (`main_v1_2.exe`) and the python backend simultaneously in a split-pane terminal using the provided script. You do not need VSCode; simply run this from `cmd` at the project root:
```cmd
run_system.bat
```

Alternatively, to run the bridge manually without the split-pane script:
```cmd
.\.venv\Scripts\activate.bat
cd PC_Backend
python serial_bridge.py
```

**Diagnostic tools:**
```bash
python PC_Backend/diag_com10.py      # monitor raw COM10 traffic
python PC_Backend/homing_log.py      # decode $HOM telemetry from firmware
python PC_Backend/test_robot.py      # automated pick-and-place test sequence
```

---

## 13. Joystick Control

The robot supports an optional **ESP32 Bluetooth gamepad** wired to USART3 (115200 8N1). Switch the physical mode toggle to enable joystick mode.

| Button | Action |
|--------|--------|
| L (left stick) | Jog CCW by 15° |
| R (right stick) | Jog CW by 15° |
| A | Run Pick sequence (down → close → up) |
| B | Run Place sequence (down → open → up) |
| U | Manual gripper arm UP |
| D | Manual gripper arm DOWN |
| Y | Start homing sequence |
| X or P | Emergency stop → STATE_FAULT (fault code 0x10) |

Audio feedback (sent to gamepad buzzer):

| Code | Event |
|------|-------|
| `'h'` | Homing started |
| `'H'` | Homing complete |
| `'E'` | Fault entered |
| `'C'` | Fault cleared |
| `'J'` | Switched to joystick mode |
| `'S'` | Switched to base system mode |

---

## 14. Operating Modes

### Normal startup sequence

1. Power on → `STATE_INIT` → `STATE_IDLE`
2. PC sends heartbeat to reg 0x00 (22881) within 2 s
3. Send mode=1 (Home) → homing runs → `STATE_RUNNING` → `STATE_IDLE`
4. Issue jog, P2P, or auto commands

### Jog mode (reg 0x01 = 2 or write reg 0x05)

Write a non-zero signed degree value to reg 0x05. The robot moves that many degrees and returns to IDLE. Positive = CCW, negative = CW.

### Auto pick-and-place mode (reg 0x01 = 4)

1. Write pick/place index pairs to regs 0x12–0x21
2. Write the pair count to reg 0x22
3. Set gripper enable in reg 0x04 (bit0) if gripper is fitted
4. Write mode=4 to reg 0x01

The robot cycles through all pairs and returns to IDLE.

### Point-to-point (reg 0x23, 0x24)

Write the unit (0=degrees, 1=index) to reg 0x23, then write the target to reg 0x24. The register auto-clears after the move starts. Index 1 = 5°, index 72 = 360°.

### Soft stop (reg 0x25 bit0)

Set bit0 of reg 0x25 at any time to decelerate the motor and return to IDLE.

---

## Golden Rules (for developers)

1. **Never edit outside `/* USER CODE BEGIN … END */` blocks** in `main.c`, `stm32g4xx_it.c`, or `stm32g4xx_hal_msp.c`. CubeMX will silently discard anything outside those guards on next code generation.
2. **No `HAL_Delay()` anywhere in application code.** All timing is managed by TIM6 and `HAL_GetTick()`.
3. **`main.c` must stay thin** — only `App_Init()` in `USER CODE BEGIN 2` and `App_Run()` in the `while(1)`.
4. **Never change PID gains, ZVD constants, or S-curve limits** without hardware re-identification. All values in `params.h` were measured on the physical system.
5. **ISR wiring belongs in `stm32g4xx_it.c`** USER CODE sections only — never call HAL callbacks or module ISR hooks from module `.c` files directly.
