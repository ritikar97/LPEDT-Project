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

#define TIMERIRQWAIT (2000000)

 uint8_t itr = 0;
 bool systemState = false;

// Data structure to store the events
// uint32_t myEvents = 0;

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

  // myEvents |= eventUF;
  sl_bt_external_signal(eventUF);

  CORE_EXIT_CRITICAL();


}

void schedulerSetPB0PressEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  // myEvents |= eventPB0Pressed;
  sl_bt_external_signal(eventPB0Pressed);

  CORE_EXIT_CRITICAL();
}


void schedulerSetPB0ReleaseEvent()
{
  CORE_DECLARE_IRQ_STATE;

  // Store events within critical section
  CORE_ENTER_CRITICAL();

  // myEvents |= eventPB0Released;
  sl_bt_external_signal(eventPB0Released);

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




// Satte machine for temperature
// IDLE -> COMP1 -> I2C_TRANSFER_DONE ->
// TEMP_MEAUSRE -> I2C_READ -> IDLE
void lpedtStateMachine(sl_bt_msg_t *event)
{
  State_t currentState;
  static State_t nextState = STATE_IDLE;
  int32_t temp_in_celsius = 0;
  uint32_t avgBPM = 0;
 
  ble_data_struct_t* ble_data = return_ble_data();

  currentState = nextState;


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
  //   return;
  // }

  // if((ble_data -> connection_open_flag && ble_data -> indication_bpm_measurement_en))
  // {
  //   avgBPM = retBeatAvg();
  //   bt_send_bpm(avgBPM);
  //   return;
  // }

  

 uint32_t current_event = event -> data.evt_system_external_signal.extsignals;

  if(current_event == eventPB0Pressed)
  {
    systemState ^= 1; // = !systemState;
  }
  else
  {
    gpioLEDOff();
  }

  // displayPrintf(DISPLAY_ROW_CONNECTION, "System state = %d", systemState);

  if(systemState)
  {
    gpioLEDOn();
    LCDEnable();
    powerGPS(GPS_SEL_HIGH);
    temp_in_celsius = bme280_meas();
    displayPrintf(DISPLAY_ROW_2, "Temp = %0.2f C", (float)temp_in_celsius / 100.0);
    if((ble_data -> connection_open_flag && ble_data -> indication_temp_measurement_en))
    {
      bt_send_temp(temp_in_celsius);
    }
    if((ble_data -> connection_open_flag && ble_data -> indication_bpm_measurement_en))
    {
      avgBPM = retBeatAvg();
      bt_send_bpm(avgBPM);
    }
  }
  else
  {
    // Shut everything off here - LCD, GPS, LED
    LCDDisable();
    gpioLEDOff();
    powerGPS(GPS_SEL_LOW);
  }
//
 // State advancement based on current state
//  switch(currentState)
//  {

  
  
//    // Enable Si7021 and wait until power on
//    case STATE_IDLE:
//      nextState = STATE_IDLE;

//      if(current_event & eventPB0Pressed)
//      {
//          gpioLEDOn();
//          LCDEnable();
//          powerGPS(GPS_SEL_HIGH);
//          temp_in_celsius = bme280_meas();
//          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %0.2f C", (float)temp_in_celsius / 100.0);
//          // Take GPS measurement too
//          timerWaitUs_irq(TIMERIRQWAIT);
//         //  LOG_INFO("LEAVING STATE_IDLE\r\n");
//         // gpioLEDOff();
//      }
//      else if(current_event & eventCOMP1)
//      {
//       // gpioLEDOff();
//       //  LCDDisable();
//       itr++;
//       displayPrintf(DISPLAY_ROW_NAME, "cE = %d, itr = %d", current_event, itr);
//       if(itr == 20)
//       {
//         nextState = STATE_COMP1_EVENT;
//         timerWaitUs_irq(TIMERIRQWAIT);
//       }
//       else
//         {
//           timerWaitUs_irq(TIMERIRQWAIT);
//         }
//      }
//     //  else if(current_event)
//     //  {
//     //   displayPrintf(DISPLAY_ROW_NAME, "cE = %d, itr = %d", current_event, itr);
//     //  }
//     break;

//    // Send I2C measure temp command
//    case STATE_COMP1_EVENT:
//      nextState = STATE_COMP1_EVENT;
     
//      if(current_event & eventCOMP1)
//      {
//          // Shut everything off here - LCD, GPS, LED
//          LCDDisable();
//          gpioLEDOff();
//          powerGPS(GPS_SEL_LOW);
//          itr = 0;
//          nextState = STATE_IDLE;
//      }
//      else
//      {
//       displayPrintf(DISPLAY_ROW_CLIENTADDR, "Bruhhhh");
//      }
//      break;

//    default:
//      break;
//  }
}
