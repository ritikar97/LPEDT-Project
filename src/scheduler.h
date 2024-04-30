/*******************************************************************************
 * @file      scheduler.h
 * @brief     Scheduler routines for setting and getting events
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-09-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures
 *
 ******************************************************************************/

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdint.h>
#include "sl_bt_api.h"

// Enum to keep track of possible events
typedef enum
{
  eventUF = 1,
  eventPB0Pressed = 2,
  eventPB0Released = 4,
  eventCOMP1 = 8
} event_t;

/*
 * @func        schedulerSetUFEvent()
 *
 * @brief       Set underflow flag
 *
 * @parameters  none
 *
 * @returns     void
 */
void schedulerSetUFEvent();


/*
 * @func        schedulerSetI2CTransferDoneEvent()
 *
 * @brief       Invoked upon completion of I2C transfer
 *
 * @parameters  none
 *
 * @returns     void
 */
void schedulerSetI2CTransferDoneEvent();


/*
 * @func        schedulerSetCOMP1Event()
 *
 * @brief       Invoked upon timer interrupt
 *
 * @parameters  none
 *
 * @returns     void
 */
void schedulerSetCOMP1Event();


/*
 * @func        getNextEvent()
 *
 * @brief       Get next event available
 *
 * @parameters  none
 *
 * @returns     event extracted from the data structure
 */
// uint32_t getNextEvent();


/*
 * @func        temperature_state_machine()
 *
 * @brief       State machine for reading temperature
 *
 * @parameters  input event for state advancement
 *
 * @returns     void
 */
void lpedtStateMachine(sl_bt_msg_t *event);

void schedulerSetPB0PressEvent();
void schedulerSetPB0ReleasesEvent();
int32_t getTempInCelsius();
bool getSystemState();
bool getPrintGPS();
void setPrintGPS(bool flag);


#endif /* SRC_SCHEDULER_H_ */
