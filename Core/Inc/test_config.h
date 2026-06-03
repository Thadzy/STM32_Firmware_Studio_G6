#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

#define TEST_NONE    0
#define TEST_PHASE1  1
#define TEST_PHASE2  2
#define TEST_PHASE3  3
#define TEST_PHASE4  4

/* -----------------------------------------------------------------------
   Set ACTIVE_TEST to the phase you want to test, or TEST_NONE for the
   real firmware.  This is the only line you ever need to change.
   ----------------------------------------------------------------------- */
#define ACTIVE_TEST  TEST_PHASE3

#endif /* __TEST_CONFIG_H */
