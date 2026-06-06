# STM32 Firmware Progress Log

## Current Status

Homing sequence is working. E-Stop button is **disabled** in software (must fix — see open issue below).

---

## Open Issue ⚠️

### E-Stop Not Working (Disabled)
The physical emergency button is currently disabled in software because motor EMI noise
couples onto PA5 and continuously false-triggers the E-stop debouncer even with 1 second
debounce. A hardware fix is required (e.g., 100 nF ceramic capacitor across E-Stop pin to GND,
or ferrite bead on motor power line). After hardware fix, re-enable the two commented-out
E-stop guard blocks in `app_main.c` (line ~726) and `motor_controller.c` (line ~418).

---

## Changes Made

### 1. Modbus Heartbeat Robustness
- **File**: `Core/Src/modbus_bridge.c`
- Added `s_regs[0x00] = HB_HI` inside `on_rx()` on every valid Modbus frame.
- Keeps heartbeat alive even when PC app has a timing bug.

### 2. Auto Sequence Done Settlement
- **File**: `Core/Src/app_main.c`
- `auto_run()` now waits for `s_seq_step == s_seq_pairs * 2u + 1u` before declaring DONE.
- Ensures robot fully settles at final position before ending auto sequence.

### 3. FDCAN Interrupt Handler & Integration
- **Files**: `Core/Src/stm32g4xx_it.c`, `Core/Src/app_main.c`
- Added `FDCAN1_IT0_IRQHandler` calling `HAL_FDCAN_IRQHandler()`.
- Added `CanBus_Init()` and `CanBus_Tick()` in `App_Init()` and `App_Run()`.
- Prevents MCU hanging on CAN messages.

### 4. E-Stop & Reed Switch GPIO Pull Configuration
- **File**: `Core/Src/hw_io.c`
- `E_Stop_Pin` changed from `GPIO_PULLDOWN` → `GPIO_PULLUP`.
- Added `GPIO_PULLDOWN` for `Reed_Close_Pin`, `Reed_Down_Pin`, `Reed_Open_Pin` on GPIOC.

### 5. CubeMX .ioc Pin Pull Configuration
- **File**: `STM32_Firmware_Studio_G6.ioc`
- Configured `Reed_Up` (PB0), `Reed_Close` (PC0), `Reed_Down` (PC1), `Reed_Open` (PC3) as `GPIO_PULLDOWN`.
- Survives CubeMX code regeneration.

### 6. Volatile Fields — Mode Switch & Reset Button
- **File**: `Core/Inc/system_state.h`
- Marked `selected_mode` and `reset_btn` fields as `volatile`.
- Fixes STM32CubeIDE Live Expressions not showing live values due to compiler optimization caching.

### 7. E-Stop Active-LOW Polarity & 1-Second Debounce
- **File**: `Core/Src/hw_io.c`, `Core/Inc/params.h`
- `HwIo_Poll100Hz()` checks `GPIO_PIN_RESET` (LOW) as active/pressed.
- `ESTOP_DEBOUNCE_THRESHOLD` raised from `8u` (80 ms) → `100u` (1000 ms).
- Physical button (NO to GND): pressed = LOW = active.
- 1-second debounce filter rejects motor PWM EMI spikes shorter than 1 second.

### 8. Joystick Handshake Echo
- **File**: `Core/Src/joystick.c`
- Added logic inside `joy_rx_cb()` to echo parsed button character back to ESP32.
- Restores gamepad confirmation beeps when buttons are pressed.

### 9. Toggle Gripper via F Key
- **File**: `Core/Src/app_main.c`
- Added `case 'F':` in `handle_joystick()` switch.
- Right Stick forward (F key) toggles claw Open ↔ Close.
- `s_f_pressed` flag prevents repeat-fire while stick is held.

### 10. Reset Button Debounce — 200 ms
- **Files**: `Core/Src/hw_io.c`, `Core/Inc/params.h`
- Added `RESET_BTN_DEBOUNCE_TICKS = 20u` (200 ms).
- Custom debounce block in `HwIo_Poll100Hz()` for reset button.
- Prevents motor EMI from triggering spurious fault clears.

### 11. Persistent Debug last_fault Variable
- **Files**: `Core/Inc/system_state.h`, `Core/Src/app_main.c`
- Added `uint8_t last_fault` to `RobotState.dbg` struct.
- Latches non-zero `fault_code` in `App_Run()` — does not clear when fault resolves.
- View `RobotState.dbg.last_fault` in Live Expressions to see what fault occurred.

### 12. Heartbeat Timeout Disabled (Standalone Mode)
- **File**: `Core/Src/app_main.c`
- Commented out `if (hb_age >= HEARTBEAT_TIMEOUT_MS)` block in `App_Run()`.
- Robot can now run with joystick only, no PC serial connection required.

### 13. E-Stop Guard Disabled *(Temporary — see Open Issue above)*
- **Files**: `Core/Src/app_main.c`, `Core/Src/motor_controller.c`
- Commented out E-stop check in `fsm_run()` and `MotorCtrl_Tick1kHz()`.
- Motor EMI noise continuously drives PA5 LOW, triggering false E-stop faults.
- **Hardware fix needed before re-enabling.**

---

## Fault Code Reference

| Code (hex) | Code (dec) | Meaning |
|---|---|---|
| `0x01` | 1 | E-Stop active |
| `0x02` | 2 | Homing: edge A or B not found (search range exceeded) |
| `0x03` | 3 | Homing: proximity sensor stuck ON / never cleared during backoff |
| `0x04` | 4 | Homing: sensor not found in full wiggle sweep (180°) |
| `0x10` | 16 | Joystick emergency stop (X button or E-press on gamepad) |
| `0x20` | 32 | PC Link Lost (heartbeat timeout) — *currently disabled* |
| `0x40` | 64 | Encoder disconnect — *disabled by default* |
| `0x41` | 65 | Boundary limit exceeded (arm past 540°) |
| `0x42` | 66 | Overcurrent — *disabled by default* |
| `0x43` | 67 | Tracking error (jam) — *disabled by default* |

---

## Joystick Button Map

| Button | Action |
|---|---|
| `Y` | Start homing sequence |
| `L` (hold) | Jog CCW |
| `R` (hold) | Jog CW |
| `A` | Pick sequence (gripper full auto: down → close → up) |
| `B` | Place sequence (gripper full auto: down → open → up) |
| `U` | Manual arm UP |
| `D` | Manual arm DOWN |
| `F` (Right Stick fwd) | Toggle claw Open ↔ Close |
| `X` | Emergency stop (fault `0x10`) |

---

## Rewind Buffer

Original code blocks backed up in:
`Core/buffer.md`
