/*******************************************************************************
 * @file      timers.h
 * @brief     Interface for initialization of LETIMER0
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-02-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures 4, 5, 6 and 7
 *
 ******************************************************************************/

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_


/*
 * @func        initLETIMER0()
 *
 * @brief       Intialization routine for LETIMER0
 *
 * @parameters  none
 *
 * @returns     void
 */
void initLETIMER0();


/*
 * @func        initLETIMER0()
 *
 * @brief       Intialization routine for LETIMER0
 *
 * @parameters  none
 *
 * @returns     void
 */
void timerWaitUs(uint32_t us_wait);

/*
 * @func        timerWaitUs_irq()
 *
 * @brief       Wait routine for timer (Non-blocking)
 *
 * @parameters  us_wait - number of microseconds to wait
 *
 * @returns     void
 */
void timerWaitUs_irq(uint32_t us_wait);

void initMillis();

#endif /* SRC_TIMERS_H_ */
