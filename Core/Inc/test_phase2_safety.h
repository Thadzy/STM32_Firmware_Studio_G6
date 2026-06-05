/* =========================================================================
   test_phase2_safety.h
   Automated Safety Commissioning — Phase 2: Four Fault Guard Verification

   Architecture:
   - TestPhase2_Run() is a non-blocking state machine; call from App_Run()
     every main-loop iteration while TestPhase2_IsRunning() is true.
   - TestInjection_t fields are read from the TIM6 ISR via hooks added to
     motor_controller.c (Guards 1, 2, 4). Guard 3 is injected directly via
     g_robot.sensors.current_amps (not overwritten by App_Run or hw_io.c).

   Usage in app_main.c App_Run():
       if (TestPhase2_IsRunning()) { TestPhase2_Run(); return; }

   Trigger the test sequence once after system reaches STATE_IDLE:
       TestPhase2_Init();

   ========================================================================= */

#ifndef TEST_PHASE2_SAFETY_H
#define TEST_PHASE2_SAFETY_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
   Fault Injection Control Struct

   All fields are volatile: written from main-loop context (TestPhase2_Run)
   and read from TIM6 ISR context (motor_controller.c hooks).

   Rule: only ONE injection flag is active at any time.
   Rule: all flags must be cleared before calling tp2_apply_reset().
   ------------------------------------------------------------------------- */
typedef struct {
    volatile bool inject_enc_stall;       /* Guard 1 hook: force encoder delta = 0 in Tick1kHz  */
    volatile bool inject_boundary;        /* Guard 2 hook: force s_pos_counts past CABLE_MAX     */
    volatile bool inject_tracking_error;  /* Guard 4 hook: force s_kalman.x[0] = target + 0.3 r */
    /* Guard 3 (current) has no hook flag — write g_robot.sensors.current_amps directly;
       App_Run and HwIo_Poll100Hz do NOT overwrite that field.                                   */
} TestInjection_t;

/* Single global instance — defined in test_phase2_safety.c
   motor_controller.c uses: extern TestInjection_t g_test_inj;              */
extern TestInjection_t g_test_inj;

/* -------------------------------------------------------------------------
   Per-guard result record (index 0 = Guard 1 ... 3 = Guard 4)
   ------------------------------------------------------------------------- */
typedef struct {
    bool     ran;            /* test executed at least once                   */
    bool     passed;         /* fault triggered with correct code             */
    uint8_t  expected_code;  /* fault code that SHOULD have appeared          */
    uint8_t  actual_code;    /* fault code that DID appear (0 on timeout)     */
    uint32_t elapsed_ms;     /* ms from injection activation to fault         */
} GuardResult_t;

/* -------------------------------------------------------------------------
   Public API
   ------------------------------------------------------------------------- */

/* Call once (from App_Run or startup) to begin the sequence.
   Resets motor controller, clears all flags, transitions to TP2_G1_SETUP. */
void TestPhase2_Init(void);

/* Call every main-loop iteration while IsRunning(). */
void TestPhase2_Run(void);

/* True between Init() and the final COMPLETE or FAILED state. */
bool TestPhase2_IsRunning(void);

/* True only after all four guards have been tested (pass or fail). */
bool TestPhase2_IsDone(void);

/* True only if all four guards passed. Call after IsDone(). */
bool TestPhase2_AllPassed(void);

/* Print a summary table via TP2_LOG (see .c for macro definition). */
void TestPhase2_PrintSummary(void);

/* Access individual guard results; guard_idx in [0, 3]. Returns NULL on
   out-of-range index. Pointer is valid for the lifetime of the test module. */
const GuardResult_t *TestPhase2_GetResult(uint8_t guard_idx);

#endif /* TEST_PHASE2_SAFETY_H */
