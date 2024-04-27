/*
 * max30101.h
 *
 *  Created on: 02 Mar 2024
 *      Author: <rira3427@colorado.edu>
 */

#ifndef SRC_BME280_H_
#define SRC_BME280_H_

void bme280_read(uint8_t reg, uint8_t* data, uint8_t len);
void bme280_wr(uint8_t reg, uint8_t data);
int32_t bme280_meas();

#endif /* SRC_BME280_H_ */
