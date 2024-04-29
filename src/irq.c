/*******************************************************************************
 * @file      irq.c
 * @brief     Interrupt service routines
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-02-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures 4 and 5
 *
 ******************************************************************************/


#include "src/gpio.h"
#include "em_letimer.h"
#include "src/scheduler.h"

volatile uint32_t milliseconds = 0;


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
       milliseconds++;
//       schedulerSetReadTempEvent();
   }
}

// Function to return number of milliseconds elapsed
uint32_t millis()
{
  return milliseconds;
}


void GPIO_EVEN_IRQHandler()
{
  //uint32_t irq_source = GPIO_IntGetEnabled();

  unsigned int pinValue;
  /* Extract flag bits from IF register to determine
       source of the interrupt */
  uint32_t irq_source = GPIO_IntGetEnabled();

  // Clear pending interrupts
  GPIO_IntClear(irq_source);

  pinValue = GPIO_PinInGet(gpioPortA, 0);

  if(pinValue == 0)
  {
      schedulerSetPB0PressEvent();
  }
  else
  {
      schedulerSetPB0ReleaseEvent();
  }

}