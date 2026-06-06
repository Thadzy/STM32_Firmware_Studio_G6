# Backup Buffer

Me store original code blocks here so user can rewind if needed.

## 1. modbus_bridge.c - on_rx()

```c
static void on_rx(const uint8_t *data, uint16_t len)
{
    if (len < MB_MIN_FRAME) return;
    if (data[0] != MB_ADDR) return;

    /* Verify CRC */
    uint16_t crc_recv = ((uint16_t)data[len-1] << 8) | data[len-2];
    if (crc16(data, len - 2u) != crc_recv) return;

    switch (data[1]) {
```

## 2. app_main.c - Includes

```c
#include "app_main.h"
#include "modbus_bridge.h"
#include "motor_controller.h"
#include "uart_dma_manager.h"
#include "hw_io.h"
#include "joystick.h"
#include "system_state.h"
#include "params.h"
#include "main.h"
```

## 3. app_main.c - auto_run()

```c
static void auto_run(void)
{
    if (s_seq_pairs == 0 || s_seq_step > s_seq_pairs * 2u || s_seq_step > 16u) {
        snprintf(s_dbg, sizeof(s_dbg), "$AUTO,DONE,%u\r\n", s_seq_step);
        UartDma_SendTelemetry(s_dbg);
        RobotState.fsm         = STATE_IDLE;
        s_run_mode          = RUN_IDLE;
        s_gripper_triggered = false;
        set_task(0x0000);
        ModbusBridge_SetReg(0x22, 0); /* Clear pairs so next tab switch doesn't auto-start */
        return;
    }

    /* Wait: motor must be at target AND gripper must be idle before next action */
    if (s_seq_step > 0u && (!MotorCtrl_IsAtTarget() || s_grip != GRIP_IDLE)) return;
```

## 4. app_main.c - App_Init() and App_Run()

```c
void App_Init(void)
{
    HwIo_Init();
    UartDma_Init();
    Joystick_Init();
    MotorCtrl_Init();
    ModbusBridge_Init();
    Motor_Enable();
    MotorCtrl_SetZvdBypass(false);  /* Rod is attached, ensure ZVD is enabled by default */
    HAL_TIM_Base_Start_IT(&htim6);
    RobotState.fsm         = STATE_INIT;
```

```c
    /* Drain UART RX → Modbus callbacks → register updates */
    UartDma_Process();

    /* Refresh Modbus read registers */
    ModbusBridge_Tick();

    /* Advance gripper sequence (runs independently of FSM) */
    gripper_seq_run();
```

## 5. stm32g4xx_it.c - TIM6_DAC_IRQHandler and USER CODE BEGIN 1

```c
/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    IWDG->KR = 0xAAAAu;  /* kick watchdog — if this ISR hangs, reset fires in 50 ms */
    MotorCtrl_Tick1kHz();

    static uint16_t s_outer_div = 0;
    if (++s_outer_div >= OUTER_LOOP_DIVIDER) {
        s_outer_div = 0;
        HwIo_Poll100Hz();
        MotorCtrl_Tick100Hz();
    }
}
/* USER CODE END 1 */
```

## 6. hw_io.c - HwIo_Init()

```c
void HwIo_Init(void)
{
    /* E-stop: NO switch wired to VCC.  Explicitly set PULLDOWN so pin is always
       defined regardless of CubeMX config.
       Open (normal) → PULLDOWN holds pin LOW → inactive.
       Pressed (closes to VCC) → pin HIGH → active (matches GPIO_PIN_SET check). */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = E_Stop_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLDOWN;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(E_Stop_GPIO_Port, &g);
    }

    /* ADC: self-calibrate, then start continuous conversion.
       Auto-zero: average 64 samples while motor is off to find V_zero.
       This eliminates offset error from sensor VCC tolerance or divider mismatch. */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);

    uint32_t sum = 0u;
    for (int i = 0; i < 64; i++) {
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            sum += HAL_ADC_GetValue(&hadc1);
    }
    float raw_avg = (float)(sum / 64u);
    s_v_zero = (raw_avg * (CS_VREF / CS_ADC_COUNTS)) / CS_DIV_RATIO;

    /* Proximity sensor: open-collector optocoupler output.
       No object  → transistor ON  → sinks pin to GND → LOW  (inactive)
       Object det → transistor OFF → pin floats        → HIGH via PULLUP (active)
       PULLUP lifts the floating state to 3.3 V so the two conditions are
       electrically distinct.  PULLDOWN made both states 0 V — MCU was blind.
       Read logic in Poll100Hz: GPIO_PIN_SET (HIGH) = sensor triggered.        */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = Proximity_Sensor_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Proximity_Sensor_GPIO_Port, &g);
    }

    /* Reed_Up: NO switch to VCC.  PULLDOWN holds pin LOW when open. */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = Reed_Up_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLDOWN;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Reed_Up_GPIO_Port, &g);
    }

    /* PWM: start with 0 % duty so TIM1 CC1 is ready to accept writes */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
}
```

## 7. STM32_Firmware_Studio_G6.ioc - PB0, PC0, PC1, PC3 GPIOParameters

```ini
PB0.GPIOParameters=GPIO_Label
PB0.GPIO_Label=Reed_Up

PC0.GPIOParameters=GPIO_Label
PC0.GPIO_Label=Reed_Close

PC1.GPIOParameters=GPIO_Label
PC1.GPIO_Label=Reed_Down

PC3.GPIOParameters=GPIO_Label
PC3.GPIO_Label=Reed_Open
```

## 8. system_state.h - RobotState.sensors selected_mode and reset_btn fields

```c
                 bool     selected_mode;   /* read on-demand, not from ISR   */
                 bool     reset_btn;       /* read on-demand, not from ISR   */
```

## 9. hw_io.c - E-Stop poll block

```c
    /* E-Stop: active HIGH (PULLDOWN on PA5).
       Normal = contact open = pulled LOW = inactive.
       Pressed = contact closes to VCC = HIGH = active.
       8 consecutive HIGH reads = trigger; one LOW read = immediate clear. */
    if (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin) == GPIO_PIN_SET) {
        if (s_estop_count < ESTOP_DEBOUNCE_THRESHOLD) {
            s_estop_count++;
        }
        if (s_estop_count >= ESTOP_DEBOUNCE_THRESHOLD) {
            s_estop_active = true;
        }
    } else {
        s_estop_count  = 0;
        s_estop_active = false;
    }
```

## 10. joystick.c - joy_rx_cb()

```c
static void joy_rx_cb(const uint8_t *buf, uint16_t size)
{
    /* Frame format: "[base][emergency][status]\r\n"
       Scan forward until we find a valid triplet so partial or multi-frame
       buffers are handled without dropping valid data.                       */
    for (uint16_t i = 0; i + 2 < size; i++) {
        char b = (char)buf[i];
        char e = (char)buf[i + 1];
        char s = (char)buf[i + 2];

        if ((s == 'N' || s == 'C') && (e == 'O' || e == 'P')) {
            /* Cortex-M4 single-word writes are atomic; struct fits in 3 bytes.
               No critical section needed for this read/write pattern.       */
            s_state.base      = b;
            s_state.emergency = e;
            s_state.connected = (s == 'C');
            return; /* take the first valid frame, ignore the rest           */
        }
    }
}
```

## 11. app_main.c - handle_joystick() switches

```c
static void handle_joystick(void)
{
    if (!s_joy_mode) {
```

```c
    JoyState_t joy = Joystick_GetState();
```

```c
    switch (joy.base) {
    case 'A':  gripper_start(true);       break;  /* Pick sequence            */
    case 'B':  gripper_start(false);      break;  /* Place sequence           */
    case 'U':  Gripper_SetVertical(true); break;  /* Manual arm up            */
    case 'D':  Gripper_SetVertical(false);break;  /* Manual arm down          */
    case 'Y':
```

```c
    switch (joy.base) {
    case 'L':
...
    case 'A':  gripper_start(true);        break;
    case 'B':  gripper_start(false);       break;
    case 'U':  Gripper_SetVertical(true);  break;
    case 'D':  Gripper_SetVertical(false); break;
    case 'Y':
```

## 12. params.h - ESTOP_DEBOUNCE_THRESHOLD

```c
/* --- Debounce --------------------------------------------------------------*/
/* E-Stop: 8 × 10 ms = 80 ms consecutive LOW reads required to trigger.
   Clears immediately on a single HIGH read (fail-safe). */
#define ESTOP_DEBOUNCE_THRESHOLD    8u
```

## 13. hw_io.c - Reset button debounce

```c
    /* Reset button: active LOW (PULLUP on PA7) */
    debounce_update(&s_reset_btn,
        HAL_GPIO_ReadPin(Reset_Btn_GPIO_Port, Reset_Btn_Pin) == GPIO_PIN_RESET);
```

## 14. system_state.h - RobotState.dbg sub-struct

```c
        float    vel_dps;
        uint16_t hb_age_ms; /* ms since last heartbeat register change */
```

## 15. app_main.c - App_Init() last_fault init

```c
    RobotState.dbg.safety.en_encoder_health  = false;
    RobotState.dbg.safety.en_current_safety  = false;
    RobotState.dbg.safety.en_tracking_safety = false;
```

## 16. app_main.c - App_Run() last_fault update

```c
    /* Update debug mirror — expand RobotState in Live Expressions to see all */
    {
        JoyState_t _j = Joystick_GetState();
        RobotState.dbg.run_mode = (uint8_t)s_run_mode;
        RobotState.dbg.grip     = (uint8_t)s_grip;
        RobotState.dbg.joy_mode = (uint8_t)s_joy_mode;
        RobotState.dbg.joy_btn  = _j.base;
        RobotState.dbg.joy_conn = (uint8_t)_j.connected;
        RobotState.dbg.pos_deg  = rad_to_deg(MotorCtrl_GetPosition_rad());
        RobotState.dbg.vel_dps  = RobotState.motion.velocity_rps * 360.0f;
    }
```
```

## 17. hw_io.c - E-stop active LOW polling

```c
    /* E-Stop: active LOW (PULLUP on PA5).
       Normal = contact open = pulled HIGH = inactive.
       Pressed = contact closes to GND = LOW = active.
       8 consecutive LOW reads = trigger; one HIGH read = immediate clear. */
    if (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin) == GPIO_PIN_RESET) {
```
```
```

## 18. hw_io.c - E-stop active HIGH polling

```c
    /* E-Stop: active HIGH (PULLUP on PA5).
       Normal = contact closed to GND = LOW = inactive.
       Pressed = contact open (floats) = HIGH = active.
       100 consecutive HIGH reads = trigger; one LOW read = immediate clear. */
    if (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin) == GPIO_PIN_SET) {
```
```
```
```

## 19. app_main.c - Heartbeat timeout check

```c
        /* Do not interrupt homing — it has its own safety limits (FAULT codes 2-4).
           Stopping mid-sweep corrupts edge detection and causes 0.4°/12s creep. */
        if (hb_age >= HEARTBEAT_TIMEOUT_MS &&
            RobotState.fsm != STATE_FAULT    &&
            RobotState.fsm != STATE_HOMING) {
            MotorCtrl_Stop();
            RobotState.fsm              = STATE_IDLE;
            s_run_mode               = RUN_IDLE;
            RobotState.comms.fault_code = 0x20u; /* PC Link Lost */
            set_task(0x0000);
        }
```
```
```

## 20. app_main.c - FSM E-stop check

```c
    /* E-stop: NO switch to VCC, PULLDOWN — safe to enable, no false-triggers */
    if (RobotState.sensors.estop && RobotState.fsm != STATE_FAULT) {
        RobotState.fsm              = STATE_FAULT;
        RobotState.comms.fault_code = 0x01u;
        MotorCtrl_Stop();
        set_task(0x0000);
    }
```

## 21. motor_controller.c - TIM6 E-stop check

```c
    /* E-stop guard */
    if (RobotState.sensors.estop) {
        s_vel_sp = 0.0f;
        s_acc_sp = 0.0f;
        s_spd_integral = 0.0f;
        Motor_SetPWM(0);
        return;
    }
```
```
```
```
```
```
