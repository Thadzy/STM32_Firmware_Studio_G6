# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

This is a **STM32CubeIDE** managed-build project (Eclipse CDT, arm-none-eabi-gcc). There is no hand-editable Makefile; the IDE generates one internally.

- **Build**: Project → Build in STM32CubeIDE, or via the IDE's internal make runner. Output lands in `Debug/Pick_and_Place_Robot.elf`.
- **Flash**: Run → Debug (OpenOCD + ST-Link V2) inside STM32CubeIDE.
- **No CLI build path exists** — do not attempt `make` or `cmake` from the terminal.

Compiler flags of note (from `.cproject`): `-DUSE_HAL_DRIVER`, `-DSTM32G474xx`, `-DDEBUG`, FPU `fpv4-sp-d16`, hard-float ABI, `-g3`.

## The Golden Rules

1. **Never edit outside `/* USER CODE BEGIN … END */` blocks** in CubeMX-generated files (`main.c`, `stm32g4xx_it.c`, `stm32g4xx_hal_msp.c`). CubeMX will regenerate these files and silently discard anything outside those guards.
2. **No `HAL_Delay()` or busy-waits anywhere in application logic.** All time-division is handled by TIM6.
3. `main.c` must stay thin: only `App_Init()` in `USER CODE BEGIN 2` and `App_Run()` inside the `while(1)` `USER CODE BEGIN 3`.

## Architecture in One Page

The full spec lives in `CONTEXT.md`. The short version:

```
TIM6 ISR (1 kHz)
  ├── every tick  → motor_controller: read encoder → Kalman → inner speed PID → PWM
  └── every 10th  → hw_io: Poll100Hz (debounce)
                 → motor_controller: S-curve → ZVD shaper → outer pos PID

main loop (App_Run)
  └── uart_dma_manager: drain RX ring → modbus_bridge → app_main FSM
                        drain TX ring → DMA kick
```

**HAL handles** (`hadc1`, `htim1`, `htim3`, `htim6`, `hlpuart1`, `huart3`, `hfdcan1`) live in `main.c`. Custom modules access them with `extern`.

**ISR wiring** lives in `stm32g4xx_it.c` USER CODE sections — never in the module `.c` files themselves.

## Module Responsibilities

| File | Owns |
|------|------|
| `params.h` | All tuneable constants. Items marked `TODO` need hardware calibration. |
| `hw_io.c` | Debounce, ADC DMA averaging, relay/motor/gripper actuators. `HwIo_Poll100Hz()` called from TIM6 at 100 Hz. |
| `uart_dma_manager.c` | 1024-byte TX ring buffer, RX idle-DMA, Modbus T3.5 guard, telemetry drop above 800-byte watermark. |
| `motor_controller.c` | 1 kHz inner speed PID + 100 Hz outer position PID, S-curve trajectory, ZVD input shaper. Owns TIM3 encoder start. |
| `kalman.c` | 4-state Kalman filter called every 1 ms from `motor_controller`. |
| `modbus_bridge.c` | Modbus RTU parser. Slave address **21**. |
| `app_main.c` | FSM: `STATE_INIT → HOMING → IDLE → RUNNING → FAULT`. Motion commands rejected in FAULT/HOMING. |
| `stm32g4xx_it.c` | ISR hooks only — delegates immediately to module functions, no logic. |

## Modbus Register Map (implement in `modbus_bridge.c`)

**WRITE registers (PC → robot):**

| Addr | Meaning |
|------|---------|
| 0x00 | Heartbeat: robot sends 22881 ("YA"); reply 18537 ("HI") |
| 0x01 | Mode: 1=Home, 2=Jog, 4=Auto, 8=SetHome, 16=Test |
| 0x02 | Manual gripper: 0=Up, 1=Down, 2=Open, 4=Close |
| 0x03 | Gripper seq: 1=Pick, 2=Place |
| 0x04 | Gripper in auto: bit0 enable |
| 0x05 | Jog step (int16, degrees; +CCW, −CW) |
| 0x06 | Test type: 0=Precision, 1=Performance |
| 0x07–0x08 | Performance test velocity / acceleration (int16) |
| 0x09, 0x10, 0x11 | Precision test initial pos / final pos / repeat count |
| 0x12–0x21 | Pick & place sequence slots (int16, magnitude=index, sign=direction) |
| 0x22 | Number of pick–place pairs |
| 0x23 | P2P unit: 0=degree, 1=index |
| 0x24 | P2P target (int16) |
| 0x25 | Soft stop: bit0 |

**READ registers (robot → PC, block read 0x00 + 0x26–0x31):**

| Addr | Meaning |
|------|---------|
| 0x00 | Heartbeat (robot writes 22881) |
| 0x26 | Reed sensors: bit0=Reed1, bit1=Reed2, bit2=Reed3(jaw) |
| 0x27 | Task: bit0=Homing, 1=GoPick, 2=GoPlace, 3=GoPoint, 0=Idle |
| 0x28–0x30 | Position / Velocity / Acceleration — stored as `int16 × 10` |
| 0x31 | Emergency: bit0 active |

## PC Backend (not yet built)

`PC_Backend/serial_bridge.py` demultiplexes the shared USB serial port:
- Packets starting with `$` and ending `\r\n` → WebSocket port **8765** (React dashboard).
- All other packets → validate CRC-16; if valid, forward to COM10 (virtual loopback for `main.exe` on COM11).
- Dependencies: `pyserial`, `websockets`, `pymodbus` (see `requirements.txt`).

## Phase 3 — Hardware-Identified Parameters

All values hardware-measured / system-identified. **Never change without asking the user.**

| Parameter | Value | Units |
|-----------|-------|-------|
| Velocity PID Kp / Ki / Kd | 1.58 / 0.5 / 0.0 | PWM/(rad/s) |
| Position PID Kp / Ki / Kd | 3.0 / 0.02 / 0.15 | (rad/s)/rad |
| Velocity feedforward | 3.03 | PWM/(rad/s) |
| Acceleration feedforward | 0.1 | PWM/(rad/s²) |
| Disturbance feedforward | 0.0 | — |
| Vmax | 7.304 | rad/s |
| Amax | 27.49 | rad/s² |
| Jmax | 1400 | rad/s³ |
| ZVD natural frequency | 12.13 | Hz |
| ZVD damping ratio | 0.041 | — |

**Derived ZVD constants** (at 100 Hz outer loop):
- Td = π / (ωn √(1−ζ²)) ≈ 41.2 ms → T2 = 4 steps, T3 = 8 steps
- K = 0.8790; A1 = 0.2832, A2 = 0.4981, A3 = 0.2189

## Implementation Status

- [x] Phase 1 — `params.h`, `hw_io.h/.c`
- [x] Phase 2 — `uart_dma_manager.h/.c`
- [x] Phase 3 — `kalman.h/.c`, `motor_controller.h/.c`
- [x] Phase 4 — `modbus_bridge.h/.c`, `app_main.h/.c`
- [x] Phase 5 — `stm32g4xx_it.c` TIM6 ISR hook (added during Phase 3)
- [ ] Phase 6 — `main.c` USER CODE (App_Init / App_Run calls)
- [ ] Phase 7 — `PC_Backend/serial_bridge.py`
