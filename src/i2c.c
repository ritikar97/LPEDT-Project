/*******************************************************************************
 * @file      i2c.c
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

#include "src/i2c.h"
#include "sl_i2cspm.h"
#include "em_i2c.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// Defines for Si7021
#define SI7021_CMD_MEASURE_TEMP (0xF3)
#define SI7021_DEVICE_ADDR      (0x40)

// Data structures to hold trasnfer data, read data, command data
I2C_TransferSeq_TypeDef     transferSequence;
uint8_t                     cmd_data[2];
uint16_t                    read_data;
uint8_t                     rx_data[2];


void i2c_init()
{
  // I2C Config
  // I2C0 SCL GPIO pin - PC10
  // I2C0 SDA GPIO pin - PC11
  // Initialise I2C peripheral
  I2CSPM_Init_TypeDef I2C_Config = {
        .port             = I2C0,
        .sclPort          = gpioPortD,
        .sclPin           = 15,
        .sdaPort          = gpioPortD,
        .sdaPin           = 14,
        .portLocationScl  = 22,
        .portLocationSda  = 22,
        .i2cRefFreq       = 0,
        .i2cMaxFreq       = I2C_FREQ_STANDARD_MAX,
        .i2cClhr          = i2cClockHLRStandard
    };

  I2CSPM_Init(&I2C_Config);
}


void i2c_send_cmd(I2C_TransferSeq_TypeDef* transferSeq)
{
  I2C_TransferReturn_TypeDef  transferStatus;

//  i2c_init(); // TODO: Check if I need this all the time

  //NVIC_EnableIRQ(I2C0_IRQn);

  // Sending command
  transferStatus = I2CSPM_Transfer(I2C0, transferSeq);

  // Check if I2C transfer was done successfully
  if(transferStatus != i2cTransferDone)
  {
      LOG_ERROR("I2CSPM_Transfer: I2C bus write of command = 0x%0x failed with %d\n\r", cmd_data, transferStatus);
  }

  if(transferStatus < 0)
  {
      LOG_ERROR("%d", transferStatus);
  }

}


void i2c_read_data()
{
   I2C_TransferReturn_TypeDef  transferStatus;

  // Initialise transfer data structure to read temperature data
    transferSequence.addr         = SI7021_DEVICE_ADDR << 1;
    transferSequence.flags        = I2C_FLAG_READ;
    transferSequence.buf[0].data  = rx_data;
    transferSequence.buf[0].len   = sizeof(read_data);

    NVIC_EnableIRQ(I2C0_IRQn);

    // Transfer I2C sequence
    transferStatus = I2C_TransferInit(I2C0, &transferSequence);

    if(transferStatus < 0)
    {
        LOG_ERROR("%d", transferStatus);
    }
}


void print_temperature()
{
  int32_t                     temp_in_celsius;

  // rx_data[0] stores MSB while rx_data[1] stores LSB
  read_data = (rx_data[0] << 8) | rx_data[1];

  // Conversion of temperature into Celsius
  temp_in_celsius = (int) ((175.72 * read_data / 65536) - 46.85);
  LOG_INFO("Temperature = %d degree Celsius\n\r", temp_in_celsius);
}




