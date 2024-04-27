/*******************************************************************************
 * @file      i2c.h
 * @brief     I2C Routines for sending commands and reading data
 *
 * @author    Ritika Ramchandani <rira3427@colorado.edu>
 *
 * @date      02-09-2023
 *
 * @ref       EMLIB APIs and Documentation by Silicon Labs
 *            IoT Embedded Firmware - Lectures
 *
 ******************************************************************************/

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "em_i2c.h"

/*
 * @func        i2c_init()
 *
 * @brief       Intialization routine for I2C
 *
 * @parameters  none
 *
 * @returns     void
 */
void i2c_init();


/*
 * @func        i2c_send_cmd()
 *
 * @brief       I2C routine to send read temp command
 *
 * @parameters  transferSeq - The I2C packet that needs to be sent
 *
 * @returns     void
 */
void i2c_send_cmd(I2C_TransferSeq_TypeDef* transferSeq);


/*
 * @func        i2c_read_data()
 *
 * @brief       I2C routine to read temperature data
 *
 * @parameters  none
 *
 * @returns     void
 */
void i2c_read_data();


/*
 * @func        print_temperature()
 *
 * @brief       Printing temperature data
 *
 * @parameters  none
 *
 * @returns     void
 */
void print_temperature();


#endif /* SRC_I2C_H_ */
