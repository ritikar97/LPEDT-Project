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

  sl_bt_external_signal(eventUF);

  CORE_EXIT_CRITICAL();


}


// Scheduler for I2C transfer done
void schedulerSetI2CTransferDoneEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  sl_bt_external_signal(eventI2CTransferDone);

  CORE_EXIT_CRITICAL();
}


// Scheduler for COMP1 flag
void schedulerSetCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  sl_bt_external_signal(eventCOMP1);

  CORE_EXIT_CRITICAL();
}


// Sending the next event to caller
/*uint32_t getNextEvent()
{
  uint32_t theEvent = 0;

  CORE_DECLARE_IRQ_STATE;

  // Enter critical structure to clear the event within the shared structure
  CORE_ENTER_CRITICAL();

  // Getting events in priority order
  if(myEvents & eventUF)
  {
      theEvent = eventUF;
  }
  else if(myEvents & eventCOMP1)
  {
      theEvent = eventCOMP1;
  }
  else if(myEvents & eventI2CTransferDone)
  {
      theEvent = eventI2CTransferDone;
  }

  // Clear event
  myEvents ^= theEvent;
  CORE_EXIT_CRITICAL();

  return theEvent;

}*/


// Satte machine for temperature
// IDLE -> COMP1 -> I2C_TRANSFER_DONE ->
// TEMP_MEAUSRE -> I2C_READ -> IDLE
void temperature_state_machine(sl_bt_msg_t *event)
{
  LOG_INFO("Inside the state machine\r\n");
  State_t currentState;
  static State_t nextState = STATE_IDLE;
  int32_t temp_in_celsius = 0;
  ble_data_struct_t* ble_data = return_ble_data();

  currentState = nextState;

  // Check event header
  if(SL_BT_MSG_ID(event -> header) != sl_bt_evt_system_external_signal_id)
  {
      return;
  }

  // Check if connection is open and indications are enabled
  if(!(ble_data -> connection_open_flag && ble_data -> indication_temp_measurement_en))
  {
      return;
  }

  uint32_t current_event = event -> data.evt_system_external_signal.extsignals;

  // State advancement based on current state
  switch(currentState)
  {

    // Enable Si7021 and wait until power on
    case STATE_IDLE:
      nextState = STATE_IDLE;

      if(current_event == eventUF)
      {
          gpioLEDOn();
          // gpioSensorEnable();
          timerWaitUs_irq(80000);
          nextState = STATE_COMP1_EVENT;
          LOG_INFO("LEAVING STATE_IDLE\r\n");
      }
     break;

    // Send I2C measure temp command
    case STATE_COMP1_EVENT:
      nextState = STATE_COMP1_EVENT;
      if(current_event == eventCOMP1)
      {
          gpioLEDOff();
        //  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        //  i2c_send_cmd();
         if(!(ble_data -> connection_open_flag && ble_data -> indication_temp_measurement_en))
         {
             nextState = STATE_IDLE;
         }
         else
         {
            temp_in_celsius = bme280_meas();
            bt_send_temp(temp_in_celsius);
         }
          nextState = STATE_IDLE;
         LOG_INFO("LEAVING STATE_COMP1_EVENT\r\n");
      }
      break;    

    default:
      break;
  }
}
