/*
 * max30101.c
 *
 *  Created on: 02 Mar 2024
 *      Author: <rira3427@colorado.edu>
 */

#include "src/i2c.h"
#include "em_core.h"
#include <stdint.h>
#include "src/max30101.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define MAX30101_DEVICE_ID  (0x55)
#define WR_REG              (0xAE)
#define RD_REG              (0xAF)
#define CONFIG_REG          (0x0A)
#define MODE_REG_ADDR       (0x09)
#define FIFO_WR_PTR_ADDR    (0x04)
#define FIFO_RD_PTR_ADDR    (0x06)
#define FIFO_DATA_ADDR      (0x07)
#define PULSE_WIDTH         (0x03)

// MODE - 0x02 (Heart rate sensing)
#define MODE_2 (0x02) // SHDN - 0, RESET - 0, RESV - 000, MODE - 010

uint8_t sensor_reg_data;
//uint8_t mode;

void max30101Config()
{
  uint8_t rdData;
  writeModeReg();
  // Set mode to heartrate
//  max30101WriteReg(MODE_REG_ADDR, 0x02);
  // Set pulse width
//  max30101ReadReg(CONFIG_REG, &rdData);
//  rdData |= PULSE_WIDTH;
//  max30101WriteReg(CONFIG_REG, rdData);

  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);


//
//  // Set samples
//  max30101ReadReg()
//  // Get current register value so that nothing is overwritten.
//    regVal = readRegisterMAX30101(CONFIGURATION_REGISTER);
//    regVal &= SAMP_MASK; // Mask bits to change.
//    regVal |= (bits << 2); // Add bits but shift them first to correct position.
//    writeRegisterMAX30101(CONFIGURATION_REGISTER, regVal); // Write Register

}

void config_sensor()
{
  LOG_INFO("Starting to configure the sensor\r\n");
  uint8_t data[2];
  data[0] = MODE_REG_ADDR;
  data[1] = MODE_2;

  I2C_TransferSeq_TypeDef i2cPacket;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE;
  i2cPacket.buf[0].data = data;
  i2cPacket.buf[0].len = sizeof(data);

  LOG_INFO("Sending to configure the sensor\r\n");
  // Write into MODE register
  i2c_send_cmd(&i2cPacket);

  LOG_INFO("Done configuring the sensor\r\n");
}


void read_fifo()
{

  uint8_t reg;
  reg = FIFO_WR_PTR_ADDR;
  uint8_t wrPtr;

  //  START;
  //  Send device address + write mode
  //  Send address of FIFO_WR_PTR;
  //  REPEATED_START;
  //  Send device address + read mode
  //  Read FIFO_WR_PTR;
  //  STOP;
  I2C_TransferSeq_TypeDef i2cPacket;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE_READ;
  i2cPacket.buf[0].data = &reg;
  i2cPacket.buf[0].len = sizeof(reg);

  i2cPacket.buf[1].data = &wrPtr;
  i2cPacket.buf[1].len = sizeof(wrPtr);

  // Write-read into/from WR_PTR register
  i2c_send_cmd(&i2cPacket);

  reg = FIFO_RD_PTR_ADDR;
  uint8_t rdPtr;

  //  START;
  //  Send device address + write mode
  //  Send address of FIFO_WR_PTR;
  //  REPEATED_START;
  //  Send device address + read mode
  //  Read FIFO_WR_PTR;
  //  STOP;
  i2cPacket.flags = I2C_FLAG_WRITE_READ;
  i2cPacket.buf[0].data = &reg;
  i2cPacket.buf[0].len = sizeof(reg);

  i2cPacket.buf[1].data = &rdPtr;
  i2cPacket.buf[1].len = sizeof(rdPtr);

  // Write-read into/from WR_PTR register
  i2c_send_cmd(&i2cPacket);

//  Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO:
//  START;
//  Send device address + write mode
//  Send address of FIFO_DATA;
//  REPEATED_START;
//  Send device address + read mode
//  for (i = 0; i < NUM_SAMPLES_TO_READ; i++) {
//  Read FIFO_DATA;
//  Save LED1[23:16];
//  Read FIFO_DATA;
//  Save LED1[15:8];
//  Read FIFO_DATA;
//  Save LED1[7:0];
//  Read FIFO_DATA;
//  Save LED2[23:16];
//  Read FIFO_DATA;
//  Save LED2[15:8];
//  Read FIFO_DATA;
//  Save LED2[7:0];
//  Read FIFO_DATA;
//  Save LED3[23:16];
//  Read FIFO_DATA;
//  Save LED3[15:8];
//  Read FIFO_DATA;
//  Save LED3[7:0];
//  Read FIFO_DATA;
//  }
//  STOP;

  uint8_t numSamples = (wrPtr < rdPtr) ? (wrPtr + 32 - rdPtr) : wrPtr - rdPtr;

  LOG_INFO("Number of samples = %d\n\r", numSamples);
  uint8_t fifoData[numSamples];
  reg = FIFO_DATA_ADDR;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE_READ;
  i2cPacket.buf[0].data = &reg;
  i2cPacket.buf[0].len = sizeof(reg);

  i2cPacket.buf[1].data = fifoData;
  i2cPacket.buf[1].len = numSamples;

  // TODO: Not necessary if second transaction was successful
}


void max30101ReadReg(uint8_t reg, uint8_t* rdData)
{
//  uint8_t reg;
//  reg = MODE_REG_ADDR;

  I2C_TransferSeq_TypeDef i2cPacket;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE_READ;
  i2cPacket.buf[0].data = &reg;
  i2cPacket.buf[0].len = sizeof(reg);

  i2cPacket.buf[1].data = &rdData;
  i2cPacket.buf[1].len = 1;

  // Write-read into/from WR_PTR register
  i2c_send_cmd(&i2cPacket);
}

void max30101WriteReg(uint8_t regAddr, uint8_t data)
{
  uint8_t wrData[2];
  wrData[0] = regAddr;
  wrData[1] = data;
  I2C_TransferSeq_TypeDef i2cPacket;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE;
  i2cPacket.buf[0].data = wrData;
  i2cPacket.buf[0].len = 2;

//  i2cPacket.buf[1].data = &data;
//  i2cPacket.buf[1].len = sizeof(data);

  // Write-read into/from WR_PTR register
  i2c_send_cmd(&i2cPacket);
}


void writeModeReg()
{
  uint8_t mode;
  max30101ReadReg(MODE_REG_ADDR, &mode);
  max30101WriteReg(MODE_REG_ADDR, mode | 0x02);
  max30101ReadReg(MODE_REG_ADDR, &mode);
}

//void print_sensor_reg()
//{
//  LOG_INFO("Value of mode is 0x%02x\n", mode);
//}

