/*******************************************************************************
 * @file      ble.c
 * @brief     Bluetooth routines for communication and responding to events
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-20-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures
 *
 ******************************************************************************/

#include "ble.h"
#include "gatt_db.h"
#include <stdbool.h>

//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "src/max30101.h"

#include "src/lcd.h"
#include "src/gpio.h"

#define ADVERTISING_MIN_MS (400) // 250 ms / 0.625 ms
#define ADVERTISING_MAX_MS (400) // 250 ms / 0.625 ms
#define CONNECTION_MIN_MS  (60) // 75 ms / 1.25 ms
#define CONNECTION_MAX_MS  (60) // 75 ms / 1.25 ms
#define SLAVE_LATENCY      (4) // 300 ms / 75 ms
#define TIMEOUT            (83) // (((1 + 4) * (75 * 2)) + 75) / 10

// BLE private data
ble_data_struct_t ble_data;

// Returning pointer to ble data struct
ble_data_struct_t* return_ble_data()
{
  return &ble_data;
}


/*
 * @func        check_status()
 *
 * @brief       Checking error status returned by GAT routines
 *
 * @param       sc - status returned by GATT routine
 * @param       func_name - String to be printed for logging errors
 *
 * @returns     void
 */
static void check_status(sl_status_t sc, char* func_name)
{
  if(sc != SL_STATUS_OK)
    {
      LOG_ERROR("%s() returned error with status = 0x%04x", func_name, (unsigned int)sc);
    }
}


void bt_handle_event(sl_bt_msg_t *event)
{
  //typedef struct parameters
  sl_status_t sc;

  // Check evetn header to determine the event
  switch(SL_BT_MSG_ID(event -> header))
  {
    // ******************************************************
    // Events common to both Servers and Clients
    // ******************************************************
    // --------------------------------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack API commands before receiving this boot event!
    // Including starting BT stack soft timers!
    // --------------------------------------------------------
    case sl_bt_evt_system_boot_id:
      // handle boot event

      // Set status flags
      ble_data.connection_open_flag = false;
      ble_data.indication_in_flight = false;
      ble_data.indication_temp_measurement_en = false;
      ble_data.indication_bpm_measurement_en = false;

      // Call functions required after boot
      // Get unique BT device address
      sc = sl_bt_system_get_identity_address(&ble_data.addr, &ble_data.addr_type);
      check_status(sc, "sl_bt_system_get_identity_address");

      // Create advertising handle
      sc = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
      check_status(sc, "sl_bt_advertiser_create_set");

      // Set limits for advertising packets
      sc = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle,
                                       ADVERTISING_MIN_MS,
                                       ADVERTISING_MAX_MS,
                                       0,
                                       0);
      check_status(sc, "sl_bt_advertiser_set_timing");

      // Request parameters to the client
      sc = sl_bt_advertiser_start(ble_data.advertisingSetHandle,
                                  sl_bt_advertiser_general_discoverable,
                                  sl_bt_advertiser_connectable_scannable);
      check_status(sc, "sl_bt_advertiser_start");

      // displayInit();

      //const char* bt_addr = {&ble_data.addr.addr[0], ":", &ble_data.addr.addr[0]};
      //for(uint8_t i = 0; i < 6; i++)

      // displayPrintf(DISPLAY_ROW_BTADDR, "%02x:%02x:%02x:%02x:%02x:%02x", ble_data.addr.addr[0],
      //               ble_data.addr.addr[1],
      //               ble_data.addr.addr[2],
      //               ble_data.addr.addr[3],
      //               ble_data.addr.addr[4],
      //               ble_data.addr.addr[5]);

      // displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A6");
      // displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
    break;


    case sl_bt_evt_connection_opened_id:
      // handle open event

      displayPrintf(DISPLAY_ROW_1, "Connected Over Bluetooth");

      // Obtain connection handle and update status flag
      ble_data.connectionHandle = event -> data.evt_connection_opened.connection;
      ble_data.connection_open_flag = true;

      // Stop advertising
      sc = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
      check_status(sc, "sl_bt_advertiser_stop");

      // Set connection parameters
      sc = sl_bt_connection_set_parameters(ble_data.connectionHandle, CONNECTION_MIN_MS, CONNECTION_MAX_MS, SLAVE_LATENCY, TIMEOUT, 0, 0xFFFF);
      check_status(sc, "sl_bt_connection_set_parameters");
      break;


    case sl_bt_evt_connection_closed_id:
      // handle close event
      // Updates status flags
      ble_data.connection_open_flag = false;
      ble_data.indication_in_flight = false;

      // Re-start advertising
      sc = sl_bt_advertiser_start(ble_data.advertisingSetHandle,
                                  sl_bt_advertiser_general_discoverable,
                                  sl_bt_advertiser_connectable_scannable);
      check_status(sc, "sl_bt_advertiser_start");

      // displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
      // displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
    break;


    case sl_bt_evt_connection_parameters_id:
      // Obtain parameter information (determined by the client)
//      LOG_INFO("Value of interval = %d us \r\n", 1250 * (event -> data.evt_connection_parameters.interval));
//      LOG_INFO("Value of latency = %d connection intervals \r\n", (event -> data.evt_connection_parameters.latency));
//      LOG_INFO("Value of interval = %d ms \r\n", 10 * (event -> data.evt_connection_parameters.timeout));
    break;


    case sl_bt_evt_system_external_signal_id:
    break;


    // Get status for temperature measurement characteristic
    case sl_bt_evt_gatt_server_characteristic_status_id:
      // Check whether CCCD was changed
      if(event -> data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
      {
          // Check if change was made to temperature measurement characteristic
          if(event -> data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
          {
              // Check if indications were enabled
              if((event -> data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_indication)
                  || event -> data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_notification_and_indication)
              {
                  ble_data.indication_temp_measurement_en = true;
                  if(getSystemState())
                  {
                    int32_t temp_in_celsius = getTempInCelsius();
                    bt_send_temp(temp_in_celsius);
                  }
              }
              else
              {
                  ble_data.indication_temp_measurement_en = false;
              }
          }
          if(event -> data.evt_gatt_server_characteristic_status.characteristic == gattdb_BPM)
          {
            // Check if indications were enabled
            if((event -> data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_indication)
                || event -> data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_notification_and_indication)
            {
                // gpioLEDOff();
                ble_data.indication_bpm_measurement_en = true;
                if(getSystemState())
                {
                  uint32_t avgBPM = retBeatAvg();
                  bt_send_bpm(avgBPM);
                }
            }
            else
            {
                ble_data.indication_bpm_measurement_en = false;
            }
          }
      }
      // Check if remote GATT received indication successfully
      else if(event -> data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation)
      {
          ble_data.indication_in_flight = false;
      }
    break;

    // Check if indication has timed out
    case sl_bt_evt_gatt_server_indication_timeout_id:
      ble_data.indication_in_flight = false;
    break;


    // Check if timer has started
    case sl_bt_evt_system_soft_timer_id:
      displayUpdate();
    break;

   } // end - switch
} // handle_ble_event()



void bt_send_temp(uint32_t temp_in_celsius)
{
  sl_status_t sc;
  uint8_t temp[4];
  uint8_t* temp_c_ptr = temp;
  ble_data_struct_t* ble_s = return_ble_data();
  uint8_t flags = 0;
  uint8_t htm_temperature_buffer[5];
  uint8_t* p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;

  // Convert decimal value of temperature to a uint8_t*
  UINT32_TO_BITSTREAM(temp_c_ptr, temp_in_celsius);

  // Write temperature attribute
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_temperature_measurement, 0, 4, temp_c_ptr);
  check_status(sc, "sl_bt_gatt_server_write_attribute_value");

  // Convert temperature into IEEE floating point format
  htm_temperature_flt = UINT32_TO_FLOAT(temp_in_celsius*1000, -3);

  UINT8_TO_BITSTREAM(p, flags);
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);

  // If okay to send indication, send to client
  if(ble_s -> connection_open_flag && ble_s -> indication_temp_measurement_en
      && !(ble_s -> indication_in_flight))
  {
      sc = sl_bt_gatt_server_send_indication(
                          ble_s -> connectionHandle,
                          gattdb_temperature_measurement,
                          5,
                          &htm_temperature_buffer[0]);

      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("sl_bt_gatt_server_send_indication() returned error with status = 0x%04x\r\n", (unsigned int)sc);
      }
      else
      {
          ble_s -> indication_in_flight = true;
      }

  }
}


void bt_send_bpm(uint32_t avg_bpm)
{
  sl_status_t sc;
  uint8_t avg_bpm_val = (uint8_t) avg_bpm;
  ble_data_struct_t* ble_s = return_ble_data();
  uint8_t flags = 0;
  uint8_t bpm_buffer[2];
  uint8_t* p = bpm_buffer;


  // Write temperature attribute
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_BPM, 0, sizeof(avg_bpm_val), &avg_bpm_val);
  check_status(sc, "sl_bt_gatt_server_write_attribute_value");

  // Convert temperature into IEEE floating point format
  // htm_bpm_flt = UINT32_TO_FLOAT(avg_bpm*1000, -3);

  UINT8_TO_BITSTREAM(p, flags);
  UINT8_TO_BITSTREAM(p, avg_bpm_val);

  // If okay to send indication, send to client
  if(ble_s -> connection_open_flag && ble_s -> indication_bpm_measurement_en
      && !(ble_s -> indication_in_flight))
  {
      sc = sl_bt_gatt_server_send_indication(
                          ble_s -> connectionHandle,
                          gattdb_BPM,
                          sizeof(bpm_buffer),
                          &bpm_buffer[0]);

      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("sl_bt_gatt_server_send_indication() returned error with status = 0x%04x\r\n", (unsigned int)sc);
      }
      else
      {
          ble_s -> indication_in_flight = true;
      }

  }
}
