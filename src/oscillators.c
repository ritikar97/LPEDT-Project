/***************************************************************************//**
 * @file      oscillators.c
 * @brief     Initialization and selection of clocks
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-02-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures 4 and 5
 *
 ******************************************************************************/


#include "em_cmu.h"
#include "oscillators.h"
#include "app.h"

// Selecting ULFRCO for EM3 and LXFO for all other energy modes
#if (LOWEST_ENERGY_MODE == EM3)
#define CMU_CLK_SOURCE (cmuOsc_ULFRCO)
#define CMU_CLK_SELECT (cmuSelect_ULFRCO)
#else
#define CMU_CLK_SOURCE (cmuOsc_LFXO)
#define CMU_CLK_SELECT (cmuSelect_LFXO)
#endif

uint32_t freq; // Variable used for reading the freq

void init_osc()
{
  // Enabling LXFO/ULFRCO clock sources
  CMU_OscillatorEnable(CMU_CLK_SOURCE, true, true);

  // Selecting the low freq clock for the LFA clock tree
  CMU_ClockSelectSet(cmuClock_LFA, CMU_CLK_SELECT);

  // Enabling the LFA clock branch
  CMU_ClockEnable(cmuClock_LFA, true);

  // Selecting the appropriate prescalar
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);

  // Enabling the clock for LETIMER0
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Testing the clock frequency of LETIMER0
  freq = CMU_ClockFreqGet(cmuClock_LETIMER0);
}
