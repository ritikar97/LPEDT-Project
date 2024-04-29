/*
 * max30101.h
 *
 *  Created on: 02 Mar 2024
 *      Author: <rira3427@colorado.edu>
 */

#ifndef SRC_MAX30131_H_
#define SRC_MAX30131_H_

 #define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
  } sense_struct; //This is our circular buffer of readings from the sensor


void max30101WriteReg(uint8_t regAddr, uint8_t data);
void max30101ReadFifo(uint8_t* rdData, uint8_t len);
void max30101ReadReg(uint8_t reg, uint8_t* rdData);
void max30101Setup();
void rdModifyWrReg(uint8_t reg, uint8_t mask, uint8_t data);
uint32_t getIR(void);
bool safeCheck(uint8_t maxTimeToCheck);
void loop();
uint16_t check(void);


#endif /* SRC_MAX30131_H_ */
