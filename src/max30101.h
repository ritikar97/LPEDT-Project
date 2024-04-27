/*
 * max30101.h
 *
 *  Created on: 02 Mar 2024
 *      Author: <rira3427@colorado.edu>
 */

#ifndef SRC_MAX30131_H_
#define SRC_MAX30131_H_

void max30101Config();
void config_sensor();
void read_fifo();
//void read_mode_reg();
void print_sensor_reg();
void max30101ReadReg(uint8_t reg, uint8_t* rdData);
void writeModeReg();



#endif /* SRC_MAX30131_H_ */
