/*******************************************************************************
 * @file      irq.h
 * @brief     Interrupt service routines
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-20-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures
 *
 ******************************************************************************/

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include <stdint.h>

/*
 * @func        letimerMilliseconds()
 *
 * @brief       Returns elapsed time n milliseconds
 *
 * @parameters  none
 *
 * @returns     uint32_t - milliseconds elapsed
 */
uint32_t millis();


#endif /* SRC_IRQ_H_ */
