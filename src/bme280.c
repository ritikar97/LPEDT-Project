/*
 * max30101.c
 *
 *  Created on: 02 Mar 2024
 *      Author: <rira3427@colorado.edu>
 */

#include "src/i2c.h"
#include "em_core.h"
#include <stdint.h>
#include "src/bme280.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define BME280_DEVICE_ADDR  (0x76)
#define BME280_IDREG_ADDR   (0xD0)
#define CTRL_MEAS_REG       (0xF4)
#define STATUS_REG          (0xF3)
#define MEAS_IN_PROG        (0x09)
#define TEMP_REG_ADDR       (0xFA)
#define DIG_T1_REG          (0x88)
#define DIG_T2_REG          (0x8A)
#define DIG_T3_REG          (0x8C)
#define CONFIG_REG          (0xF5)
#define RESET_REG           (0xE0)

// MODE - 0x02 (Heart rate sensing)
#define ID_VALUE (0x60)

void bme280_read(uint8_t reg, uint8_t* data, uint8_t len)
{

//    LOG_INFO("Reading BME280 sensor\r\n");

    I2C_TransferSeq_TypeDef i2cPacket;
    i2cPacket.addr = BME280_DEVICE_ADDR << 1;
    i2cPacket.flags = I2C_FLAG_WRITE_READ;
    i2cPacket.buf[0].data = &reg;
    i2cPacket.buf[0].len = sizeof(reg);

    i2cPacket.buf[1].data = data;
    i2cPacket.buf[1].len = len;

    // Write-read into/from WR_PTR register
    i2c_send_cmd(&i2cPacket);
//    LOG_INFO("ID value that BME280 returned is 0x%0x\r\n", data);

}

void bme280_read2(uint8_t reg, uint16_t* data, uint8_t len)
{

//    LOG_INFO("Reading BME280 sensor222\r\n");

    I2C_TransferSeq_TypeDef i2cPacket;
    i2cPacket.addr = BME280_DEVICE_ADDR << 1;
    i2cPacket.flags = I2C_FLAG_WRITE_READ;
    i2cPacket.buf[0].data = &reg;
    i2cPacket.buf[0].len = sizeof(reg);

    i2cPacket.buf[1].data = (uint8_t*) data;
    i2cPacket.buf[1].len = len;

    // Write-read into/from WR_PTR register
    i2c_send_cmd(&i2cPacket);
//    LOG_INFO("ID value that BME280 returned is 0x%0x\r\n", data);

}

void bme280_wr(uint8_t reg, uint8_t data)
{
    uint8_t wrData[2];
    wrData[0] = reg;
    wrData[1] = data;
    uint8_t len = 2;
    I2C_TransferSeq_TypeDef i2cPacket;
    i2cPacket.addr = BME280_DEVICE_ADDR << 1;
    i2cPacket.flags = I2C_FLAG_WRITE;
    i2cPacket.buf[0].data = wrData;
    i2cPacket.buf[0].len = len;

    i2c_send_cmd(&i2cPacket);
}

int32_t bme280_temp_read()
{
    int32_t adc_T = 0, var1 = 0, var2 = 0, T = 0, tFine = 0;
    uint16_t calib_t1 = 0;
    int16_t calib_t2 = 0;
    int16_t calib_t3 = 0;
    uint8_t t_xlsb;
    uint16_t t_val;

    bme280_read2(TEMP_REG_ADDR, &t_val, 2);
    bme280_read(TEMP_REG_ADDR + 2, &t_xlsb, 1);
    t_val = (t_val >> 8) | (t_val << 8);
    adc_T = t_val;
    adc_T <<= 8;
    adc_T |= t_xlsb;
    adc_T >>= 4;

    bme280_read2(DIG_T1_REG, (uint16_t*) (&calib_t1), 2);
    bme280_read2(DIG_T2_REG, (uint16_t*) (&calib_t2), 2);
    bme280_read2(DIG_T3_REG, (uint16_t*) (&calib_t3), 2);

    // temp_msb - [19:12], temp_lsb - [11:4], temp_xlsb, [3:0]
    // adc_T |= (i2c_smbus_read_byte_data(bme280_device.bme280_i2c_client, TEMP_REG_ADDR) << 12);
    // adc_T |= (i2c_smbus_read_byte_data(bme280_device.bme280_i2c_client, TEMP_REG_ADDR + 1) << 4);
    // adc_T |= (i2c_smbus_read_byte_data(bme280_device.bme280_i2c_client, TEMP_REG_ADDR + 2) >> 4);

    //   dig_T1_val = (((bme280_device.calib_data[dig_T1 + 1]) << 8) | (bme280_device.calib_data[dig_T1]));
    //   dig_T2_val = (((bme280_device.calib_data[(1 << dig_T2) + 1]) << 8) | (bme280_device.calib_data[(1 << dig_T2)]));
    //   dig_T3_val = (((bme280_device.calib_data[(1 << dig_T3) + 1]) << 8) | (bme280_device.calib_data[(1 << dig_T3)]));

      // Compensation for possible errors in sensor data
      // Reference for logic: BME280 Datasheet
//      var1 = (((adc_T >> 3) - ((long signed int) calib_t1 << 1)) * ((long signed int) calib_t2)) >> 11;
//      var2 = (((((adc_T >> 4) - ((long signed int) calib_t1)) * ((adc_T >> 4) - ((long signed int) calib_t1))) >> 12) *
//          ((long signed int) calib_t3)) >> 14;

    var1 = ((((adc_T>> 3) - ((int32_t) calib_t1 << 1))) * ((int32_t) calib_t2))
             >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) calib_t1))
             * ((adc_T >> 4) - ((int32_t) calib_t1))) >> 12) * ((int32_t) calib_t3))
             >> 14;

    tFine = var1 + var2;
    T = (tFine * 5 + 128) >> 8;
    return T;
}

int32_t bme280_meas()
{
    uint8_t rdData;
    int32_t bme280_temp_val;
    // bme280_wr(RESET_REG, 0x86);
//    bme280_read(BME280_IDREG_ADDR, &rdData, 1);
    bme280_wr(CTRL_MEAS_REG, 0x26);
    bme280_wr(CONFIG_REG, 0x80);

//    LOG_INFO("Reading control measure reg\r\n");
//    bme280_read(CTRL_MEAS_REG, &rdData, 1);
//
//    LOG_INFO("Writing to control measure reg\r\n");
//    bme280_wr(CTRL_MEAS_REG, rdData | 1);

    do
    {
        bme280_read(STATUS_REG, &rdData, 1);
    } while(rdData == MEAS_IN_PROG);

    bme280_temp_val = bme280_temp_read();

    LOG_INFO("Temperature value = %d\r\n", bme280_temp_val);



    return bme280_temp_val;
}



