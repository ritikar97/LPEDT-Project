/*******************************************************************************
 * @file      scheduler.c
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

#include "src/scheduler.h"
#include "em_core.h"
#include <stdint.h>
#include "src/gpio.h"
#include "src/timers.h"
#include "sl_power_manager.h"
#include "src/i2c.h"
#include "src/ble.h"
#include "src/bme280.h"
#include "src/lcd.h"
#include "src/max30101.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// Data structure to store the events
uint32_t myEvents = 0;

// States of temp state machine
typedef enum uint32_t
{
  STATE_IDLE = 0,
  STATE_COMP1_EVENT,
  STATE_I2C_TRANSFER_DONE,
  STATE_TEMP_MEASURE,
  STATE_I2C_READ,
  NUM_STATES
} State_t;


// Scheduler for UF flag
void schedulerSetUFEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  myEvents |= eventUF;

  CORE_EXIT_CRITICAL();


}

void schedulerSetPB0PressEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  myEvents |= eventPB0Pressed;

  CORE_EXIT_CRITICAL();
}


void schedulerSetPB0ReleaseEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  myEvents |= eventPB0Released;

  CORE_EXIT_CRITICAL();
}


// // Scheduler for I2C transfer done
// void schedulerSetI2CTransferDoneEvent()
// {
//   CORE_DECLARE_IRQ_STATE;

//   // Store events within critical section
//   CORE_ENTER_CRITICAL();

//   sl_bt_external_signal(eventI2CTransferDone);

//   CORE_EXIT_CRITICAL();
// }


// Scheduler for COMP1 flag
void schedulerSetCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  myEvents |= eventCOMP1;

  CORE_EXIT_CRITICAL();
}

// uint32_t getNextEvent()
// {
//   uint32_t theEvent;
//   uint32_t index = 0;
//   uint32_t temp = myEvents;

//   CORE_DECLARE_IRQ_STATE;

//   // Extraction of all events, with LSB as highest priority
//   while(temp != 0)
//   {
//      if(temp & 1)
//        break;
//      else
//      {
//          temp = temp >> 1;
//          index++;
//      }
//   }

//   theEvent = (1 << index);

//   // Enter critical structure to clear the event within the shared structure
//   CORE_ENTER_CRITICAL();
//   myEvents &= ~(theEvent);
//   CORE_EXIT_CRITICAL();

//   return theEvent;

// }


// Satte machine for temperature
// IDLE -> COMP1 -> I2C_TRANSFER_DONE ->
// TEMP_MEAUSRE -> I2C_READ -> IDLE
void lpedtStateMachine(sl_bt_msg_t *event)
{
  int32_t temp_in_celsius = 0;
  uint32_t avgBPM = 0;
  uint8_t itr = 0;
  // ble_data_struct_t* ble_data = return_ble_data();

  // Check event header
 if(SL_BT_MSG_ID(event -> header) != sl_bt_evt_system_external_signal_id)
 {
     return;
 }

  // Check if connection is open and indications are enabled
  // if((ble_data -> connection_open_flag && ble_data -> indication_temp_measurement_en))
  // {
    // temp_in_celsius = bme280_meas();
    // displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %0.2f C", (float)temp_in_celsius / 100.0);
//    bt_send_temp(temp_in_celsius);
    return;
  }

  // if((ble_data -> connection_open_flag && ble_data -> indication_bpm_measurement_en))
  // {
  //   avgBPM = retBeatAvg();
  //   bt_send_bpm(avgBPM);
  //   return;
  // }

  

//  uint32_t current_event = event -> data.evt_system_external_signal.extsignals;
//
 // State advancement based on current state
 switch(currentState)
 {

   // Enable Si7021 and wait until power on
   case STATE_IDLE:
     nextState = STATE_IDLE;

     if(current_event == PB0)
     {
         gpioLEDOn();
         LCDEnable();
         temp_in_celsius = bme280_meas();
         displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %0.2f C", (float)temp_in_celsius / 100.0);
         // Take GPS measurement too
         timerWaitUS_irq();
         LOG_INFO("LEAVING STATE_IDLE\r\n");
     }
    break;

   // Send I2C measure temp command
   case STATE_COMP1_EVENT:
     nextState = STATE_COMP1_EVENT;
     if(current_event == eventCOMP1)
     {
          itr++;
          if(itr == 20)
          {
         // Shut everything off here - LCD, GPS, LED
         itr = 0;
         nextState = STATE_IDLE;
          }
     }
     break;

   default:
     break;
 }
}
