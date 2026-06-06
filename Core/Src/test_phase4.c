#include "test_config.h"
#if (ACTIVE_TEST == TEST_PHASE4)

#include "test_phase4.h"
#include "app_main.h"

/* =========================================================================
   Phase 4 test — full app_main + modbus_bridge

   This is a passthrough to the real application.
   Connect the PC base system (Docker + main.exe) at 230400 8E1 on the
   STM32 VCP port (LPUART1).

   How to test:
   1. Flash and open the base system web UI at http://localhost:3000
   2. Send Mode=Home (0x01=1) → watch arm wiggle-search the proximity sensor
   3. After homing, use Jog (0x05 = step degrees, 0x01 = 2) to move arm
   4. Check Live Expressions:
        RobotState.fsm            (0=INIT 1=HOMING 2=IDLE 3=RUNNING 4=FAULT)
        RobotState.sensors.proximity
        RobotState.motion.position_counts
        RobotState.comms.heartbeat (should toggle 22881 ↔ 18537)
   ========================================================================= */

void TestPhase4_Init(void) { App_Init(); }
void TestPhase4_Run(void)  { App_Run();  }

#endif /* ACTIVE_TEST == TEST_PHASE4 */
