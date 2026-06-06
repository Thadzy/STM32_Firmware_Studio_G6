/* =========================================================================
   test_phase2_safety.c
   Automated Safety Commissioning — Phase 2: Four Fault Guard Verification

   Guard map (from params.h / motor_controller.c):
     0x40  Encoder Health    — |PWM|>5 AND delta==0 for 200 ms  (Guard 1)
     0x41  Boundary          — |s_pos_counts| > 12288 counts     (Guard 2)
     0x42  Current Fuse      — current_amps > 2.0 A for 100 ms   (Guard 3)
     0x43  Tracking / Jam    — |target−pos| > 10° for 50 ms
                               after S-curve done                 (Guard 4)

   ISR hooks required in motor_controller.c — see bottom of this file.
   No hooks needed in hw_io.c (current_amps is not refreshed by App_Run).
   ========================================================================= */

#include "test_phase2_safety.h"
#include "system_state.h"
#include "motor_controller.h"
#include "params.h"
#include "main.h"        /* HAL_GetTick */
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------
   Log macro — routes to SWO/ITM in debug builds if syscalls.c maps printf
   to ITM_SendChar. Replace with UartDma_SendTelemetry() if preferred.

   MISRA-C 2012 deviation: variadic macro (Rule 20.10). Accepted for test
   code only; disable this file in production builds.
   ------------------------------------------------------------------------- */
#define TP2_LOG(fmt, ...)  \
    do { (void)printf("[TP2] " fmt "\r\n", ##__VA_ARGS__); } while (0u)

/* -------------------------------------------------------------------------
   Timing constants — all in milliseconds
   ------------------------------------------------------------------------- */
#define TP2_SETTLE_MS           30u    /* inter-guard motor settle delay                  */
#define TP2_G1_ARM_WAIT_MS     120u    /* wait for velocity PID to saturate PWM > 5       */
#define TP2_G1_FAULT_TIMEOUT   500u    /* > SAFETY_ENC_STALL_MS (200) + large margin      */
#define TP2_G2_FAULT_TIMEOUT    50u    /* boundary check is immediate (first Tick1kHz)     */
#define TP2_G3_FAULT_TIMEOUT   300u    /* > SAFETY_CURRENT_MS (100) + large margin        */
#define TP2_G4_SC_DONE_WAIT    350u    /* ZVD tail flush: 18 × 10 ms = 180 ms + margin    */
#define TP2_G4_FAULT_TIMEOUT   250u    /* > SAFETY_TRACKING_MS (50) + large margin        */
#define TP2_RESET_BTN_HOLD_MS   20u    /* reset_btn pulse width (documentation only —
                                          actual FSM reset is done directly)               */
#define TP2_IDLE_CONFIRM_MS     20u    /* brief hold in WAIT_IDLE to let ISR quiesce       */

/* Drive target for guards that need s_running=true with non-zero PWM.
   1.0 rad (~57°) gives large position error → velocity PID saturates quickly. */
#define TP2_DRIVE_TARGET_RAD   1.0f

/* Guard 4 injection magnitude. Must exceed SAFETY_TRACKING_DEG (10°) in radians.
   0.30 rad ≈ 17.2° provides a comfortable margin without endangering the motor. */
#define TP2_TRACKING_INJECT_RAD 0.30f

/* Current injection level — must exceed CURRENT_FAULT_AMPS (2.0 A). */
#define TP2_INJECT_CURRENT_AMPS 5.0f

/* -------------------------------------------------------------------------
   State enum — one state per atomic action for debuggability.
   Add a Live Expression "s_state" in STM32CubeIDE to watch progress.
   ------------------------------------------------------------------------- */
typedef enum {

    TP2_IDLE = 0,

    /* === Guard 1: Encoder Health (0x40) === */
    TP2_G1_SETUP,        /* enable guard; command move to generate PWM           */
    TP2_G1_WAIT_ARMED,   /* wait for PID to ramp PWM above enc-stall threshold  */
    TP2_G1_INJECT,       /* set inject_enc_stall = true                          */
    TP2_G1_WAIT_FAULT,   /* poll for STATE_FAULT with timeout                    */
    TP2_G1_VERIFY,       /* assert fault_code == 0x40                            */
    TP2_G1_RESET,        /* clear injections, reset FSM, pulse reset_btn         */
    TP2_G1_WAIT_RESET,   /* hold reset_btn for TP2_RESET_BTN_HOLD_MS             */
    TP2_G1_WAIT_IDLE,    /* confirm FSM is back in STATE_IDLE                    */

    /* === Guard 2: Boundary (0x41) === */
    TP2_G2_SETTLE,       /* inter-guard motor settle                             */
    TP2_G2_INJECT,       /* set inject_boundary = true                           */
    TP2_G2_WAIT_FAULT,
    TP2_G2_VERIFY,
    TP2_G2_RESET,
    TP2_G2_WAIT_RESET,
    TP2_G2_WAIT_IDLE,

    /* === Guard 3: Current Fuse (0x42) === */
    TP2_G3_SETTLE,       /* inter-guard motor settle                             */
    TP2_G3_SETUP,        /* enable guard; start move so s_running = true         */
    TP2_G3_INJECT,       /* write current_amps = 5.0 A directly                  */
    TP2_G3_WAIT_FAULT,
    TP2_G3_VERIFY,
    TP2_G3_RESET,
    TP2_G3_WAIT_RESET,
    TP2_G3_WAIT_IDLE,

    /* === Guard 4: Tracking Error / Jam (0x43) === */
    TP2_G4_SETTLE,       /* inter-guard motor settle                             */
    TP2_G4_SETUP,        /* enable guard; set target=0 (s_sc.done true quickly)  */
    TP2_G4_WAIT_SC_DONE, /* wait for S-curve to complete + ZVD settle tail       */
    TP2_G4_INJECT,       /* set inject_tracking_error = true                     */
    TP2_G4_WAIT_FAULT,
    TP2_G4_VERIFY,
    TP2_G4_RESET,
    TP2_G4_WAIT_RESET,
    TP2_G4_WAIT_IDLE,

    TP2_COMPLETE,        /* all four guards tested — normal exit                 */

} TestPhase2State_t;

/* -------------------------------------------------------------------------
   Module-private state
   ------------------------------------------------------------------------- */
static TestPhase2State_t s_state   = TP2_IDLE;
static uint32_t          s_t0      = 0u;        /* HAL_GetTick at state entry  */
static GuardResult_t     s_results[4u];

/* -------------------------------------------------------------------------
   Global injection struct — read from TIM6 ISR hooks in motor_controller.c
   ------------------------------------------------------------------------- */
TestInjection_t g_test_inj = { false, false, false };

/* =========================================================================
   Internal helpers
   ========================================================================= */

static uint32_t elapsed(void)
{
    return HAL_GetTick() - s_t0;
}

static void goto_state(TestPhase2State_t next)
{
    s_state = next;
    s_t0    = HAL_GetTick();
}

static void clear_injections(void)
{
    g_test_inj.inject_enc_stall      = false;
    g_test_inj.inject_boundary       = false;
    g_test_inj.inject_tracking_error = false;
    /* Guard 3 — clear the direct field injection */
    RobotState.sensors.current_amps     = 0.0f;
}

/* Full reset between guards.
   Direct FSM manipulation is intentional: App_Run's FAULT→IDLE handler reads
   HwIo_GetResetBtn() from hardware (not RobotState.sensors.reset_btn), so it
   cannot be triggered by software alone. The test harness has authority to
   reset directly. The reset_btn pulse below is logged for completeness only. */
static void apply_reset(void)
{
    /* 1. Stop all injections first — prevents re-triggering a fault from the ISR */
    clear_injections();

    /* 2. Stop the motor controller — clears PID integrators and PWM */
    MotorCtrl_Stop();

    /* 3. Reset FSM and all fault state */
    RobotState.fsm                          = STATE_IDLE;
    RobotState.comms.fault_code             = 0u;
    RobotState.dbg.safety.tripped_encoder   = false;
    RobotState.dbg.safety.tripped_boundary  = false;
    RobotState.dbg.safety.tripped_current   = false;
    RobotState.dbg.safety.tripped_tracking  = false;

    /* 4. Disable all guards — each test re-enables only the guard under test */
    RobotState.dbg.safety.en_encoder_health  = false;
    RobotState.dbg.safety.en_current_safety  = false;
    RobotState.dbg.safety.en_tracking_safety = false;

    /* 5. Re-zero motor position — gives all subsequent tests a known origin.
          MotorCtrl_Zero re-seeds Kalman, s_pos_counts, and re-enables ZVD.  */
    MotorCtrl_Zero(0.0f);
}

static void record_result(uint8_t idx, uint8_t expected_code, uint32_t inject_elapsed_ms)
{
    bool pass = (RobotState.comms.fault_code == expected_code);

    s_results[idx].ran           = true;
    s_results[idx].passed        = pass;
    s_results[idx].expected_code = expected_code;
    s_results[idx].actual_code   = RobotState.comms.fault_code;
    s_results[idx].elapsed_ms    = inject_elapsed_ms;

    if (pass) {
        TP2_LOG("Guard %u (0x%02X): PASS  elapsed=%lu ms",
                (unsigned)(idx + 1u), (unsigned)expected_code,
                (unsigned long)inject_elapsed_ms);
    } else {
        TP2_LOG("Guard %u (0x%02X): FAIL  actual_code=0x%02X  elapsed=%lu ms",
                (unsigned)(idx + 1u), (unsigned)expected_code,
                (unsigned)RobotState.comms.fault_code,
                (unsigned long)inject_elapsed_ms);
    }
}

/* =========================================================================
   Public API
   ========================================================================= */

void TestPhase2_Init(void)
{
    (void)memset(s_results, 0, sizeof(s_results));
    apply_reset();
    s_state = TP2_G1_SETUP;
    s_t0    = HAL_GetTick();
    TP2_LOG("=== Phase 2 Safety Commissioning: BEGIN ===");
}

bool TestPhase2_IsRunning(void)
{
    return (s_state != TP2_IDLE) && (s_state != TP2_COMPLETE);
}

bool TestPhase2_IsDone(void)
{
    return (s_state == TP2_COMPLETE);
}

bool TestPhase2_AllPassed(void)
{
    uint8_t i;
    if (!TestPhase2_IsDone()) { return false; }
    for (i = 0u; i < 4u; i++) {
        if (!s_results[i].passed) { return false; }
    }
    return true;
}

const GuardResult_t *TestPhase2_GetResult(uint8_t guard_idx)
{
    if (guard_idx >= 4u) { return NULL; }
    return &s_results[guard_idx];
}

void TestPhase2_PrintSummary(void)
{
    static const char * const k_names[4u] = {
        "Guard 1 (0x40) Encoder Health",
        "Guard 2 (0x41) Boundary      ",
        "Guard 3 (0x42) Current Fuse  ",
        "Guard 4 (0x43) Tracking Jam  ",
    };
    uint8_t i;
    TP2_LOG("=== Phase 2 Safety Commissioning: SUMMARY ===");
    for (i = 0u; i < 4u; i++) {
        TP2_LOG("  %s : %s", k_names[i],
                s_results[i].passed ? "PASS" : "FAIL");
    }
    TP2_LOG("OVERALL: %s", TestPhase2_AllPassed() ? "ALL PASS" : "SOME FAILED");
}

/* =========================================================================
   Main non-blocking state machine — call every main-loop iteration
   ========================================================================= */
void TestPhase2_Run(void)
{
    switch (s_state) {

    /* -------------------------------------------------------------------- */
    case TP2_IDLE:
        break;  /* waiting for TestPhase2_Init() */

    /* ==================================================================
       GUARD 1 — Encoder Health (fault 0x40)
       Method: command a 1-rad move (generates large PWM), then force
               encoder delta = 0 via ISR hook. Guard trips after 200 ms.
       ================================================================== */
    case TP2_G1_SETUP:
        TP2_LOG("Guard 1 (0x40): SETUP  driving motor to 1.0 rad, enabling guard");
        RobotState.dbg.safety.en_encoder_health = true;
        /* SetTarget puts s_running=true; velocity PID ramps PWM beyond
           SAFETY_ENC_STALL_PWM (5) within a few ms due to large error.      */
        MotorCtrl_SetTarget(TP2_DRIVE_TARGET_RAD);
        goto_state(TP2_G1_WAIT_ARMED);
        break;

    case TP2_G1_WAIT_ARMED:
        /* Allow PID to ramp: at Kp=3.0, 1-rad error gives ~3 rad/s cmd →
           velocity PID at Kp=1.58 + FF=3.03 → PWM ≫ 5 within ~50 ms.
           TP2_G1_ARM_WAIT_MS (120 ms) is conservative.                      */
        if (elapsed() >= TP2_G1_ARM_WAIT_MS) {
            goto_state(TP2_G1_INJECT);
        }
        break;

    case TP2_G1_INJECT:
        TP2_LOG("Guard 1 (0x40): INJECT  enc_stall=true (encoder delta forced 0)");
        g_test_inj.inject_enc_stall = true;
        goto_state(TP2_G1_WAIT_FAULT);
        break;

    case TP2_G1_WAIT_FAULT:
        if (RobotState.fsm == STATE_FAULT) {
            record_result(0u, 0x40u, elapsed());
            goto_state(TP2_G1_VERIFY);
        } else if (elapsed() > TP2_G1_FAULT_TIMEOUT) {
            TP2_LOG("Guard 1 (0x40): TIMEOUT  no fault in %u ms",
                    (unsigned)TP2_G1_FAULT_TIMEOUT);
            record_result(0u, 0x40u, elapsed());
            goto_state(TP2_G1_RESET);
        }
        break;

    case TP2_G1_VERIFY:
        /* result already recorded in WAIT_FAULT — just log extra detail */
        TP2_LOG("Guard 1 (0x40): tripped_encoder=%u",
                (unsigned)RobotState.dbg.safety.tripped_encoder);
        goto_state(TP2_G1_RESET);
        break;

    case TP2_G1_RESET:
        apply_reset();
        /* Pulse reset_btn for documentation — FSM already at STATE_IDLE.
           App_Run reads HwIo_GetResetBtn() from hardware, not this field.   */
        RobotState.sensors.reset_btn = true;
        goto_state(TP2_G1_WAIT_RESET);
        break;

    case TP2_G1_WAIT_RESET:
        if (elapsed() >= TP2_RESET_BTN_HOLD_MS) {
            RobotState.sensors.reset_btn = false;
            goto_state(TP2_G1_WAIT_IDLE);
        }
        break;

    case TP2_G1_WAIT_IDLE:
        /* apply_reset() already set fsm=STATE_IDLE; confirm + brief quiesce */
        if ((RobotState.fsm == STATE_IDLE) && (elapsed() >= TP2_IDLE_CONFIRM_MS)) {
            goto_state(TP2_G2_SETTLE);
        }
        break;

    /* ==================================================================
       GUARD 2 — Boundary Guard (fault 0x41)
       Method: force s_pos_counts = CABLE_MAX_COUNTS + 1 via ISR hook.
               Boundary check is BEFORE the s_running gate — no motor
               move is required.
       ================================================================== */
    case TP2_G2_SETTLE:
        if (elapsed() >= TP2_SETTLE_MS) {
            TP2_LOG("Guard 2 (0x41): SETUP  boundary guard (always active, no enable)");
            goto_state(TP2_G2_INJECT);
        }
        break;

    case TP2_G2_INJECT:
        TP2_LOG("Guard 2 (0x41): INJECT  boundary=true (s_pos_counts forced to 12289)");
        g_test_inj.inject_boundary = true;
        goto_state(TP2_G2_WAIT_FAULT);
        break;

    case TP2_G2_WAIT_FAULT:
        if (RobotState.fsm == STATE_FAULT) {
            record_result(1u, 0x41u, elapsed());
            goto_state(TP2_G2_VERIFY);
        } else if (elapsed() > TP2_G2_FAULT_TIMEOUT) {
            TP2_LOG("Guard 2 (0x41): TIMEOUT  no fault in %u ms",
                    (unsigned)TP2_G2_FAULT_TIMEOUT);
            record_result(1u, 0x41u, elapsed());
            goto_state(TP2_G2_RESET);
        }
        break;

    case TP2_G2_VERIFY:
        TP2_LOG("Guard 2 (0x41): tripped_boundary=%u",
                (unsigned)RobotState.dbg.safety.tripped_boundary);
        goto_state(TP2_G2_RESET);
        break;

    case TP2_G2_RESET:
        /* clear inject_boundary BEFORE MotorCtrl_Zero to prevent re-trip */
        apply_reset();
        RobotState.sensors.reset_btn = true;
        goto_state(TP2_G2_WAIT_RESET);
        break;

    case TP2_G2_WAIT_RESET:
        if (elapsed() >= TP2_RESET_BTN_HOLD_MS) {
            RobotState.sensors.reset_btn = false;
            goto_state(TP2_G2_WAIT_IDLE);
        }
        break;

    case TP2_G2_WAIT_IDLE:
        if ((RobotState.fsm == STATE_IDLE) && (elapsed() >= TP2_IDLE_CONFIRM_MS)) {
            goto_state(TP2_G3_SETTLE);
        }
        break;

    /* ==================================================================
       GUARD 3 — Current Fuse (fault 0x42)
       Method: write RobotState.sensors.current_amps = 5.0 A directly.
               App_Run() and HwIo_Poll100Hz() do NOT update this field,
               so the value persists until apply_reset() clears it.
               Motor must be running (s_running=true) because the current
               guard is inside the !s_running early-return gate in Tick1kHz.
       ================================================================== */
    case TP2_G3_SETTLE:
        if (elapsed() >= TP2_SETTLE_MS) {
            goto_state(TP2_G3_SETUP);
        }
        break;

    case TP2_G3_SETUP:
        TP2_LOG("Guard 3 (0x42): SETUP  enabling current guard, starting motor");
        RobotState.dbg.safety.en_current_safety = true;
        MotorCtrl_SetTarget(TP2_DRIVE_TARGET_RAD);
        goto_state(TP2_G3_INJECT);
        break;

    case TP2_G3_INJECT:
        /* Inject immediately — guard needs 100 ms of persistence, so fault
           arrives at ~100 ms after injection. The overcurrent EMA filter is
           bypassed because we overwrite the final RobotState field directly.    */
        TP2_LOG("Guard 3 (0x42): INJECT  current_amps=%.1f A (threshold=%.1f A)",
                (double)TP2_INJECT_CURRENT_AMPS, (double)CURRENT_FAULT_AMPS);
        RobotState.sensors.current_amps = TP2_INJECT_CURRENT_AMPS;
        goto_state(TP2_G3_WAIT_FAULT);
        break;

    case TP2_G3_WAIT_FAULT:
        if (RobotState.fsm == STATE_FAULT) {
            record_result(2u, 0x42u, elapsed());
            goto_state(TP2_G3_VERIFY);
        } else if (elapsed() > TP2_G3_FAULT_TIMEOUT) {
            TP2_LOG("Guard 3 (0x42): TIMEOUT  no fault in %u ms",
                    (unsigned)TP2_G3_FAULT_TIMEOUT);
            record_result(2u, 0x42u, elapsed());
            goto_state(TP2_G3_RESET);
        }
        break;

    case TP2_G3_VERIFY:
        TP2_LOG("Guard 3 (0x42): tripped_current=%u",
                (unsigned)RobotState.dbg.safety.tripped_current);
        goto_state(TP2_G3_RESET);
        break;

    case TP2_G3_RESET:
        apply_reset();  /* clears current_amps to 0.0f */
        RobotState.sensors.reset_btn = true;
        goto_state(TP2_G3_WAIT_RESET);
        break;

    case TP2_G3_WAIT_RESET:
        if (elapsed() >= TP2_RESET_BTN_HOLD_MS) {
            RobotState.sensors.reset_btn = false;
            goto_state(TP2_G3_WAIT_IDLE);
        }
        break;

    case TP2_G3_WAIT_IDLE:
        if ((RobotState.fsm == STATE_IDLE) && (elapsed() >= TP2_IDLE_CONFIRM_MS)) {
            goto_state(TP2_G4_SETTLE);
        }
        break;

    /* ==================================================================
       GUARD 4 — Tracking Error / Jam (fault 0x43)
       Method: set target = 0.0 rad (= current position after reset), wait
               for S-curve to complete (s_sc.done=true) + ZVD settle tail,
               then inject s_kalman.x[0] = target + 0.3 rad via ISR hook.
               Guard fires after 50 ms of persistent error.

       Prerequisite satisfied here:
         - s_running=true  → SetTarget(0.0) keeps motor loop active
         - s_sc.done=true  → zero-distance move completes in <10 ms
         - !s_homing_mode  → apply_reset() calls MotorCtrl_Zero
         - en_tracking     → set in SETUP
       ================================================================== */
    case TP2_G4_SETTLE:
        if (elapsed() >= TP2_SETTLE_MS) {
            goto_state(TP2_G4_SETUP);
        }
        break;

    case TP2_G4_SETUP:
        TP2_LOG("Guard 4 (0x43): SETUP  enabling tracking guard, target=0.0 rad");
        RobotState.dbg.safety.en_tracking_safety = true;
        /* Target = current position (0.0 after reset) → s_sc.done becomes
           true on the first Tick100Hz call (~10 ms), s_running stays true. */
        MotorCtrl_SetTarget(0.0f);
        goto_state(TP2_G4_WAIT_SC_DONE);
        break;

    case TP2_G4_WAIT_SC_DONE:
        /* MotorCtrl_IsAtPosition() checks: s_sc.done AND s_settle_ticks >=
           ZVD_T3_STEPS+10 (18 × 10 ms = 180 ms) AND pos within deadband.
           This guarantees s_sc.done=true before we inject tracking error.   */
        if (MotorCtrl_IsAtPosition() || (elapsed() >= TP2_G4_SC_DONE_WAIT)) {
            goto_state(TP2_G4_INJECT);
        }
        break;

    case TP2_G4_INJECT:
        /* ISR hook in motor_controller.c sets s_kalman.x[0] = s_sc.target + 0.3f
           every 1 ms tick, sustaining a 0.3 rad (~17°) error indefinitely.
           Guard threshold: SAFETY_TRACKING_DEG (10°) = 0.1745 rad. Margin: 7°. */
        TP2_LOG("Guard 4 (0x43): INJECT  tracking_error=true (0.30 rad ≈ 17 deg error)");
        g_test_inj.inject_tracking_error = true;
        goto_state(TP2_G4_WAIT_FAULT);
        break;

    case TP2_G4_WAIT_FAULT:
        if (RobotState.fsm == STATE_FAULT) {
            record_result(3u, 0x43u, elapsed());
            goto_state(TP2_G4_VERIFY);
        } else if (elapsed() > TP2_G4_FAULT_TIMEOUT) {
            TP2_LOG("Guard 4 (0x43): TIMEOUT  no fault in %u ms",
                    (unsigned)TP2_G4_FAULT_TIMEOUT);
            record_result(3u, 0x43u, elapsed());
            goto_state(TP2_G4_RESET);
        }
        break;

    case TP2_G4_VERIFY:
        TP2_LOG("Guard 4 (0x43): tripped_tracking=%u",
                (unsigned)RobotState.dbg.safety.tripped_tracking);
        goto_state(TP2_G4_RESET);
        break;

    case TP2_G4_RESET:
        apply_reset();
        RobotState.sensors.reset_btn = true;
        goto_state(TP2_G4_WAIT_RESET);
        break;

    case TP2_G4_WAIT_RESET:
        if (elapsed() >= TP2_RESET_BTN_HOLD_MS) {
            RobotState.sensors.reset_btn = false;
            goto_state(TP2_G4_WAIT_IDLE);
        }
        break;

    case TP2_G4_WAIT_IDLE:
        if ((RobotState.fsm == STATE_IDLE) && (elapsed() >= TP2_IDLE_CONFIRM_MS)) {
            goto_state(TP2_COMPLETE);
        }
        break;

    /* ==================================================================
       Terminal state
       ================================================================== */
    case TP2_COMPLETE:
        /* Only print once — stay in COMPLETE indefinitely */
        if (elapsed() < 10u) {
            TestPhase2_PrintSummary();
        }
        break;

    default:
        /* Unreachable in well-formed code; guard for MISRA Rule 16.4 */
        goto_state(TP2_COMPLETE);
        break;
    }
}

/* =========================================================================
   HOOK INSTALLATION GUIDE — motor_controller.c
   =========================================================================

   Add the extern declaration near the top of motor_controller.c
   (after the existing #include list):

       #include "test_phase2_safety.h"

   Then insert the three hooks shown below at the exact line positions.
   All hooks are inside the USER CODE sections of motor_controller.c
   (which is a hand-written file, not CubeMX-generated).

   --- HOOK A: Guard 1 — encoder stall injection ---
   Location: MotorCtrl_Tick1kHz(), immediately after the delta assignment
             and BEFORE s_pos_counts accumulation.

       uint16_t enc   = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
       int16_t  delta = (int16_t)(enc - s_last_enc);
       s_last_enc     = enc;

       [INSERT HERE]
       if (g_test_inj.inject_enc_stall) { delta = 0; }   // Guard 1 hook

       s_pos_counts  += (int64_t)delta * ENCODER_DIRECTION;

   --- HOOK B: Guard 2 — boundary position injection ---
   Location: after s_pos_counts is accumulated, BEFORE the boundary check.

       s_pos_counts  += (int64_t)delta * ENCODER_DIRECTION;

       [INSERT HERE]
       if (g_test_inj.inject_boundary) {                  // Guard 2 hook
           s_pos_counts = (int64_t)CABLE_MAX_COUNTS + 1;
       }

       if (s_pos_counts >  CABLE_MAX_COUNTS || ...) {     // existing guard

   --- HOOK C: Guard 4 — Kalman position injection ---
   Location: after Kalman_Update(), BEFORE the inner velocity PID block.
             Must be INSIDE the main body (after the !s_running early-return).

       Kalman_Update(&s_kalman, pos_rad);

       RobotState.motion.position_counts = s_pos_counts;
       RobotState.motion.velocity_rps    = s_kalman.x[1] / (2.0f * M_PI);
       RobotState.motion.accel_rps2      = s_kalman.x[2] / (2.0f * M_PI);

       if (!s_running) { Motor_SetPWM(0); return; }

       [INSERT HERE — after the !s_running gate so s_running=true is guaranteed]
       if (g_test_inj.inject_tracking_error) {            // Guard 4 hook
           s_kalman.x[0] = s_sc.target + 0.30f;
       }

       float vel_actual = s_kalman.x[1];
       ...

   Guard 3 needs NO hook: RobotState.sensors.current_amps is written here
   in the test, and App_Run() / HwIo_Poll100Hz() do not overwrite it.
   ========================================================================= */
