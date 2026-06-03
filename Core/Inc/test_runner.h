#ifndef __TEST_RUNNER_H
#define __TEST_RUNNER_H

#include "test_config.h"

/* Dispatch to the active phase test, or compile to nothing for production.
   main.c calls only Test_Init() and Test_Run() — never the phase files directly. */

#if (ACTIVE_TEST == TEST_NONE)
    static inline void Test_Init(void) {}
    static inline void Test_Run(void)  {}

#elif (ACTIVE_TEST == TEST_PHASE1)
    #include "test_phase1.h"
    static inline void Test_Init(void) { TestPhase1_Init(); }
    static inline void Test_Run(void)  { TestPhase1_Run();  }

#elif (ACTIVE_TEST == TEST_PHASE2)
    #include "test_phase2.h"
    static inline void Test_Init(void) { TestPhase2_Init(); }
    static inline void Test_Run(void)  { TestPhase2_Run();  }


#elif (ACTIVE_TEST == TEST_PHASE3)
    #include "test_phase3.h"
    static inline void Test_Init(void) { TestPhase3_Init(); }
    static inline void Test_Run(void)  { TestPhase3_Run();  }

#elif (ACTIVE_TEST == TEST_PHASE4)
    #include "test_phase4.h"
    static inline void Test_Init(void) { TestPhase4_Init(); }
    static inline void Test_Run(void)  { TestPhase4_Run();  }

#endif

#endif /* __TEST_RUNNER_H */
