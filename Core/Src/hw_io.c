#include "hw_io.h"
#include "main.h"
#include "params.h"

/* HAL handles owned by main.c */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

/* ---- Current sensor state -------------------------------------------------*/
static float    s_v_zero    = 0.0f;   /* auto-calibrated zero voltage at boot  */
static float    s_amps_filt = 0.0f;   /* EMA-filtered current output           */
static uint16_t s_raw_last  = 0u;     /* last raw ADC count for diagnostics    */

/* ---- Debounce state -------------------------------------------------------*/
/* Generic 2-tick stable debouncer (20 ms at 100 Hz) */
typedef struct { uint8_t count; bool state; } Debounce_t;

static uint8_t    s_estop_count  = 0;
static bool       s_estop_active = false;

static Debounce_t s_reed[REED_COUNT];
static Debounce_t s_proximity;
static Debounce_t s_reset_btn;

/* Mode switch: long debounce (MODE_SWITCH_DEBOUNCE_TICKS × 10ms) to reject
   relay/button inductive spikes and motor-PWM EMI on adjacent GPIO lines.   */
static uint8_t s_mode_count = 0u;
static bool    s_mode_state = false;

/* Rising-edge latch: set in Poll100Hz (ISR), cleared by HwIo_GetProxRisingEdge */
static bool              s_prox_prev_isr  = false;
static volatile bool     s_prox_latch     = false;

/* ---- Private helpers -----------------------------------------------------*/

static void debounce_update(Debounce_t *d, bool raw)
{
    if (raw == d->state) {
        d->count = 0;
    } else {
        if (++d->count >= 2u) {
            d->state = raw;
            d->count = 0;
        }
    }
}

/* ========================================================================= */

void HwIo_Init(void)
{
    /* E-stop: NC switch wired to GND.
       Explicitly set PULLUP so pin is pulled HIGH when the switch opens.
       Closed (normal) → GND → pin LOW → inactive.
       Pressed (open) → PULLUP pulls pin HIGH → active. */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = E_Stop_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLUP;
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

    /* Reed switches: NO switch to VCC. PULLDOWN holds pin LOW when open. */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = Reed_Up_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLDOWN;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Reed_Up_GPIO_Port, &g);
    }
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = Reed_Close_Pin | Reed_Down_Pin | Reed_Open_Pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLDOWN;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &g);
    }

    /* PWM: start with 0 % duty so TIM1 CC1 is ready to accept writes */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
}

void HwIo_Poll100Hz(void)
{
    /* E-Stop: active HIGH (NC to GND, PULLUP on PA5).
       Normal = contact closed to GND = LOW = inactive.
       Pressed = contact open = pulled HIGH = active.
       Consecutive HIGH reads = trigger; one LOW read = immediate clear. */
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

    /* Reed switches: NO switch to VCC, PULLDOWN — active HIGH.
       2-tick (20 ms) debounce is appropriate for mechanical contacts.   */
    debounce_update(&s_reed[REED_UP],
        HAL_GPIO_ReadPin(Reed_Up_GPIO_Port,    Reed_Up_Pin)    == GPIO_PIN_SET);
    debounce_update(&s_reed[REED_DOWN],
        HAL_GPIO_ReadPin(Reed_Down_GPIO_Port,  Reed_Down_Pin)  == GPIO_PIN_SET);
    debounce_update(&s_reed[REED_OPEN],
        HAL_GPIO_ReadPin(Reed_Open_GPIO_Port,  Reed_Open_Pin)  == GPIO_PIN_SET);
    debounce_update(&s_reed[REED_CLOSE],
        HAL_GPIO_ReadPin(Reed_Close_GPIO_Port, Reed_Close_Pin) == GPIO_PIN_SET);

    /* Proximity sensor: open-collector optocoupler, PULLUP, active HIGH.
       No object  → transistor ON  → pin LOW  → GPIO_PIN_RESET → state = false
       Object det → transistor OFF → pin HIGH (via PULLUP) → GPIO_PIN_SET → state = true
       1-tick (10 ms) debounce: at HOMING_VEL_RADS = 0.4 rad/s the arm
       travels ~4° per tick.  2 ticks would need an 8°-wide detection zone;
       1 tick accepts the first stable HIGH sample and still rejects spikes. */
    {
        bool prox_raw = (HAL_GPIO_ReadPin(Proximity_Sensor_GPIO_Port,
                                          Proximity_Sensor_Pin) == GPIO_PIN_SET);
        if (prox_raw == s_proximity.state) {
            s_proximity.count = 0u;
        } else if (++s_proximity.count >= 1u) {   /* 1-tick threshold */
            s_proximity.state = prox_raw;
            s_proximity.count = 0u;
        }
    }

    /* Rising-edge latch — set here at 100 Hz so App_Run never misses an edge */
    if (s_proximity.state && !s_prox_prev_isr) s_prox_latch = true;
    s_prox_prev_isr = s_proximity.state;

    /* Reset button: NC switch wired to GND (active HIGH when open due to PULLUP on PA7).
       20-tick (200 ms) debounce rejects motor-PWM EMI false resets.             */
    {
        bool raw = (HAL_GPIO_ReadPin(Reset_Btn_GPIO_Port, Reset_Btn_Pin) == GPIO_PIN_SET);
        if (raw == s_reset_btn.state) {
            s_reset_btn.count = 0u;
        } else if (++s_reset_btn.count >= RESET_BTN_DEBOUNCE_TICKS) {
            s_reset_btn.state = raw;
            s_reset_btn.count = 0u;
        }
    }

    /* Mode switch: active LOW (PULLUP on PA6). Maintained switch — a long
       stable-hold window rejects motor-PWM-switching EMI coupled onto the line. */
    {
        bool raw = (HAL_GPIO_ReadPin(Selected_Mode_GPIO_Port, Selected_Mode_Pin) == GPIO_PIN_RESET);
        if (raw == s_mode_state) {
            s_mode_count = 0u;
        } else if (++s_mode_count >= MODE_SWITCH_DEBOUNCE_TICKS) {
            s_mode_state = raw;
            s_mode_count = 0u;
        }
    }
}

/* ---- Digital input getters -----------------------------------------------*/

bool HwIo_GetEStop(void)                 { return s_estop_active; }
void HwIo_ResetEstopDebounce(void)       { s_estop_count = 0u; }
bool HwIo_GetReedSwitch(ReedSwitch_t sw) { return s_reed[sw].state; }
bool HwIo_GetProximity(void)             { return s_proximity.state; }
bool HwIo_GetResetBtn(void)              { return s_reset_btn.state; }

bool HwIo_GetProxRisingEdge(void)
{
    if (s_prox_latch) { s_prox_latch = false; return true; }
    return false;
}

bool HwIo_GetSelectedMode(void)
{
    return s_mode_state;  /* debounced in HwIo_Poll100Hz — 100ms hold */
}

/* ---- Current sense --------------------------------------------------------*/

/* Non-blocking: if the ADC has a fresh conversion ready, consume it, apply
   the voltage-divider + sensitivity formula, and run the EMA filter.
   If called before the next conversion completes, returns the cached value.
   At 100 Hz call rate the ADC (continuous mode) always has a fresh sample. */
float HwIo_GetCurrentAmps(void)
{
    if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK) {
        s_raw_last       = (uint16_t)HAL_ADC_GetValue(&hadc1);
        float v_adc      = (float)s_raw_last * (CS_VREF / CS_ADC_COUNTS);
        float v_sensor   = v_adc / CS_DIV_RATIO;
        float i_raw      = (v_sensor - s_v_zero) / CS_SENSITIVITY;
        s_amps_filt      = CS_EMA_ALPHA * i_raw + (1.0f - CS_EMA_ALPHA) * s_amps_filt;
    }
    return s_amps_filt;
}

uint16_t HwIo_GetRawADC(void)  { return s_raw_last; }
float    HwIo_GetVzero(void)   { return s_v_zero;   }

/* ---- Actuators ------------------------------------------------------------*/

void Gripper_SetVertical(bool go_up)
{
    /* Single-coil relay on PA1 (Gripper_Up_Pin).
       SET = energised = arm UP; RESET = de-energised = arm DOWN (spring return). */
    HAL_GPIO_WritePin(Gripper_Up_GPIO_Port, Gripper_Up_Pin,
                      go_up ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Gripper_SetClaw(bool open)
{
    /* Single-coil relay on PA4 (Gripper_Down_Pin in main.h — controls claw axis).
       RESET = de-energised = claw OPEN (spring return, safe default).
       SET   = energised     = claw CLOSED (gripping). */
    HAL_GPIO_WritePin(Gripper_Down_GPIO_Port, Gripper_Down_Pin,
                      open ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Motor_SetPWM(int16_t pwm)
{
    if (pwm >  (int16_t)MOTOR_PWM_MAX) pwm =  (int16_t)MOTOR_PWM_MAX;
    if (pwm < -(int16_t)MOTOR_PWM_MAX) pwm = -(int16_t)MOTOR_PWM_MAX;

    if (pwm >= 0) {
        HAL_GPIO_WritePin(Motor_Direction_GPIO_Port, Motor_Direction_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Motor_Direction_GPIO_Port, Motor_Direction_Pin, GPIO_PIN_SET);
        pwm = -pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);
}

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(Relay_MotorPower_GPIO_Port, Relay_MotorPower_Pin, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    Motor_SetPWM(0);
    HAL_GPIO_WritePin(Relay_MotorPower_GPIO_Port, Relay_MotorPower_Pin, GPIO_PIN_RESET);
}

void Relay_SetStatus(bool on)
{
    HAL_GPIO_WritePin(Relay_SysStatus_GPIO_Port, Relay_SysStatus_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Relay_SetSysmode(bool joystick)
{
    HAL_GPIO_WritePin(Relay_Sysmode_GPIO_Port, Relay_Sysmode_Pin,
                      joystick ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
