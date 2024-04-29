/*******************************************************************************
 * @file      timers.c
 * @brief     Initialization of LETIMER0
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-09-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures 4 and 5
 *
 ******************************************************************************/


#include "em_letimer.h"
#include "src/timers.h"
#include "app.h"
#include "em_cmu.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#include <stdio.h>

// Conditional compilation
// Use ULFRCO freq for EM3 and LXFO otherwise
#if (LOWEST_ENERGY_MODE == EM3)
#define CLK_FREQ            (1000) // ULFRCO clock freq
#else
#define CLK_FREQ            (32768) // LXFO clock freq
#endif

#define PRESCALER_VALUE     (4)
#define ACTUAL_CLK_FREQ     (CLK_FREQ/PRESCALER_VALUE) // = 8192 for 32768; 250 for 1000
#define LETIMER_PERIOD_MS   (1) // LED time period = 2.25s
#define CLEAR_IRQ_FLAGS     (0xFFFFFFFF) // Clear all interrupts in IF reg

// For ULFRCO, freq = 1kHz so min time period = 1000 us
#define WAIT_LOWER_BOUND    (1000)

// Max value the counter can count to = 3000000 us (3s)
#define WAIT_UPPER_BOUND    (3000000)

#define VALUE_TO_LOAD_COMP0 ((ACTUAL_CLK_FREQ * LETIMER_PERIOD_MS) / 1000)


void initLETIMER0()
{

  uint32_t temp;

  // Data structure for initialization of LETIMER0
  const LETIMER_Init_TypeDef letimerInitData = {.enable = false, // Enable later
                                        .debugRun = true, // timer runs in debug mode
                                        .comp0Top = true, // load CNT from Comp0 register
                                        .bufTop = false, // do not load from Comp1
                                        .out0Pol = 0, // output pin value = 0
                                        .out1Pol = 0, // output pin value = 0
                                        .ufoa0 = letimerUFOANone, // no underflow output action
                                        .ufoa1 = letimerUFOANone, // no underflow output action
                                        .topValue = 0 // calculated below
                                        };

  // Initialize the timer
  LETIMER_Init(LETIMER0, &letimerInitData);

  // Load the values into COMP0 and COMP1 registers
  LETIMER_CompareSet(LETIMER0, 0, VALUE_TO_LOAD_COMP0);

  // Clear all IRQ flags in the LETIMER0 IF status register
  LETIMER_IntClear(LETIMER0, 0xFFFFFFFF);

  // Enable UF and COMP1 interrupt bits
  temp = LETIMER_IEN_UF;
  LETIMER_IntEnable(LETIMER0, temp);

  // Enable LETIMER0 interrupts
  LETIMER_Enable(LETIMER0, true);

}


void timerWaitUs(uint32_t us_wait)
{

  // Range check
  if(us_wait < WAIT_LOWER_BOUND || us_wait > WAIT_UPPER_BOUND)
  {
      LOG_ERROR("timerWaitUs ERROR: Invalid value for wait\n\r");
  }

  // Each tick of timer = 1 ms
  uint32_t ms_wait = (us_wait / 1000);

  uint32_t ms_wait_count = (ms_wait * CMU_ClockFreqGet(cmuClock_LETIMER0))/1000;

  // Check if counter value has move and decrement wait time
  uint32_t current_val = LETIMER_CounterGet(LETIMER0);
  uint32_t prev_val;

  // Wait appropriate number of ms
  while(ms_wait_count)
  {
      prev_val = current_val;

      while(prev_val == current_val)
      {
       current_val = LETIMER_CounterGet(LETIMER0);
      }

      ms_wait_count -= 1;
  }



}
