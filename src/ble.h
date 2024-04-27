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

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include <stdbool.h>
#include <stdint.h>
#include "sl_bt_api.h"

// Macros to convert integers to bitstream
#define UINT8_TO_BITSTREAM(p, n)  {*(p)++ = (uint8_t)(n);}
#define UINT32_TO_BITSTREAM(p, n) {*(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
                                   *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24);}
#define UINT32_TO_FLOAT(m, e)     (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
 // values that are common to servers and clients
 bd_addr addr;
 uint8_t addr_type;

 // values unique for server
 // The advertising set handle allocated from Bluetooth stack.
 uint8_t advertisingSetHandle;
 uint8_t connectionHandle; // Connection handle

 // Status flags
 bool connection_open_flag;
 bool indication_in_flight;
 bool indication_temp_measurement_en;

 // values unique for client
} ble_data_struct_t;


/*
 * @func        return_ble_data()
 *
 * @brief       Return pointer to BLE data structure
 *
 * @param       none
 *
 * @returns     ble_data_struct_t*
 */
ble_data_struct_t* return_ble_data();


/*
 * @func        bt_handle_event()
 *
 * @brief       Event responder for bluetooth events
 *
 * @param       event - BLE message
 *
 * @returns     void
 */
void bt_handle_event(sl_bt_msg_t *event);


/*
 * @func        bt_send_temp()
 *
 * @brief       Sends temp to client
 *
 * @param       temp in celsius
 *
 * @returns     void
 */
void bt_send_temp(uint32_t temp_in_celsius);


#endif /* SRC_BLE_H_ */
