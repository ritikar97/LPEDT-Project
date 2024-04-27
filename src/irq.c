/*******************************************************************************
 * @file      irq.c
 * @brief     Interrupt service routines
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-02-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures
 *
 ******************************************************************************/


#include "src/gpio.h"
#include "em_letimer.h"
#include "src/scheduler.h"
#include "em_i2c.h"
#include "src/irq.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// Sturcture to hold number of milliseconds elapsed
uint32_t count_ms = 0;

// IRQ handler for LETIMER0
void LETIMER0_IRQHandler()
{
  /* Extract flag bits from IF register to determine
     source of the interrupt */
  uint32_t irq_source = LETIMER_IntGetEnabled(LETIMER0);

  // Clear pending interrupts
  LETIMER_IntClear(LETIMER0, irq_source);

  // Call scheduler upon underflow
   if(irq_source & LETIMER_IF_UF)
   {
       schedulerSetUFEvent();
       count_ms += 3000;
   }

   // Call COMP1 scheduler and siable COMP1 IRQ
   if(irq_source & LETIMER_IF_COMP1)
   {
       schedulerSetCOMP1Event();
       LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
   }


}


// IRQ handler for I2C0
void I2C0_IRQHandler()
{
  // Check status of I2C Transfer
  I2C_TransferReturn_TypeDef  transferStatus;
  transferStatus = I2C_Transfer(I2C0);

  // If transfer was successful, call scheduler event
  if(transferStatus == i2cTransferDone)
  {
      schedulerSetI2CTransferDoneEvent();
  }

  if(transferStatus < 0)
  {
      LOG_ERROR("%d", transferStatus);
  }
}


// Function to return number of milliseconds elapsed
uint32_t letimerMilliseconds()
{
  return count_ms;
}

