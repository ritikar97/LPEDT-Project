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
#include "src/irq.h"
#include "src/heartrate.h"
#include "src/timers.h"
#include "src/lcd.h"
#include <string.h>

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define MAX30101_DEVICE_ID  (0x57)
#define WR_REG              (0xAE)
#define RD_REG              (0xAF)
#define CONFIG_REG          (0x0A)
#define MODE_REG_ADDR       (0x09)
#define FIFO_WR_PTR_ADDR    (0x04)
#define FIFO_RD_PTR_ADDR    (0x06)
#define FIFO_DATA_ADDR      (0x07)
#define PULSE_WIDTH         (0x03)
#define I2C_BUFFER_LENGTH 32


// FIFO Registers
static const uint8_t MAX30101_FIFOWRITEPTR =  0x04;
static const uint8_t MAX30101_FIFOREADPTR =   0x06;

// Configuration Registers
static const uint8_t MAX30101_FIFOCONFIG =    0x08;
static const uint8_t MAX30101_MODECONFIG =    0x09;
static const uint8_t MAX30101_PARTICLECONFIG =  0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30101_LED1_PULSEAMP =   0x0C;
static const uint8_t MAX30101_LED2_PULSEAMP =   0x0D;
static const uint8_t MAX30101_LED3_PULSEAMP =   0x0E;
static const uint8_t MAX30101_LED_PROX_AMP =  0x10;
static const uint8_t MAX30101_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30101_MULTILEDCONFIG2 = 0x12;

static const uint8_t MAX30101_SAMPLEAVG_MASK =  (uint8_t)~0b11100000;
static const uint8_t MAX30101_SAMPLEAVG_4 =   0x40;


static const uint8_t MAX30101_ROLLOVER_MASK =   0xEF;
static const uint8_t MAX30101_ROLLOVER_ENABLE = 0x10;

static const uint8_t MAX30101_MODE_MASK =     0xF8;
static const uint8_t MAX30101_MODE_MULTILED =   0x07;

static const uint8_t MAX30101_ADCRANGE_MASK =   0x9F;
static const uint8_t MAX30101_ADCRANGE_4096 =   0x20;

static const uint8_t MAX30101_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30101_SAMPLERATE_400 =  0x0C;


static const uint8_t MAX30101_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30101_PULSEWIDTH_411 =  0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30101_SLOT1_MASK =    0xF8;
static const uint8_t MAX30101_SLOT2_MASK =    0x8F;
static const uint8_t MAX30101_SLOT3_MASK =    0xF8;

static const uint8_t SLOT_RED_LED =       0x01;
static const uint8_t SLOT_IR_LED =        0x02;
static const uint8_t SLOT_GREEN_LED =       0x03;

// MODE - 0x02 (Heart rate sensing)
#define MODE_2 (0x02) // SHDN - 0, RESET - 0, RESV - 000, MODE - 010

uint8_t sensor_reg_data;
sense_struct sense = {.head = 0, .tail = STORAGE_SIZE-1};
const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
uint8_t rates[4] = {0, 0, 0, 0}; //Array of heart rates
uint8_t rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute = 0;
int beatAvg = 0;

void max30101Setup()
{
  // Set samples
  rdModifyWrReg(MAX30101_FIFOCONFIG, MAX30101_SAMPLEAVG_MASK, MAX30101_SAMPLEAVG_4);

  // Enable rollover
  rdModifyWrReg(MAX30101_FIFOCONFIG, MAX30101_ROLLOVER_MASK, MAX30101_ROLLOVER_ENABLE);

  // Set mode
  rdModifyWrReg(MAX30101_MODECONFIG, MAX30101_MODE_MASK, MAX30101_MODE_MULTILED);

  // Set ADC range
  rdModifyWrReg(MAX30101_PARTICLECONFIG, MAX30101_ADCRANGE_MASK, MAX30101_ADCRANGE_4096);

  // Set sample rate
  rdModifyWrReg(MAX30101_PARTICLECONFIG, MAX30101_SAMPLERATE_MASK, MAX30101_SAMPLERATE_400);

  // Set pulse width
  rdModifyWrReg(MAX30101_PARTICLECONFIG, MAX30101_PULSEWIDTH_MASK, MAX30101_PULSEWIDTH_411);

  // Set pulse amplitude
  max30101WriteReg(MAX30101_LED1_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30101_LED2_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30101_LED3_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30101_LED_PROX_AMP, 0x1F);

  // Enable slots
  rdModifyWrReg(MAX30101_MULTILEDCONFIG1, MAX30101_SLOT1_MASK, SLOT_RED_LED);
  rdModifyWrReg(MAX30101_MULTILEDCONFIG1, MAX30101_SLOT2_MASK, SLOT_IR_LED << 4);
  rdModifyWrReg(MAX30101_MULTILEDCONFIG2, MAX30101_SLOT3_MASK, SLOT_GREEN_LED);

  max30101WriteReg(MAX30101_LED1_PULSEAMP, 0x0A);
  max30101WriteReg(MAX30101_LED3_PULSEAMP, 0);

}


void max30101ReadFifo(uint8_t* rdData, uint8_t len)
{
    uint8_t reg = FIFO_DATA_ADDR;
    I2C_TransferSeq_TypeDef i2cPacket;
    i2cPacket.addr = MAX30101_DEVICE_ID << 1;
    i2cPacket.flags = I2C_FLAG_WRITE_READ;
    i2cPacket.buf[0].data = &reg;
    i2cPacket.buf[0].len = sizeof(reg);


     i2cPacket.buf[1].data = rdData;
     i2cPacket.buf[1].len = len;

    // Write-read into/from WR_PTR register
    i2c_send_cmd(&i2cPacket);
}

void max30101ReadReg(uint8_t reg, uint8_t* rdData)
{
  I2C_TransferSeq_TypeDef i2cPacket;
  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  i2cPacket.flags = I2C_FLAG_WRITE_READ;
  i2cPacket.buf[0].data = &reg;
  i2cPacket.buf[0].len = sizeof(reg);

  i2cPacket.buf[1].data = rdData;
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

  // Write-read into/from WR_PTR register
  i2c_send_cmd(&i2cPacket);
}


void rdModifyWrReg(uint8_t reg, uint8_t mask, uint8_t data)
{
  uint8_t rdData;
  max30101ReadReg(reg, &rdData);
  rdData = rdData & mask;
  max30101WriteReg(reg, rdData | data);

}


uint32_t getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

bool safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();

  while(1)
  {
  if(millis() - markTime > maxTimeToCheck) return(false);

  if(check() == true) //We found new data!
    return(true);

  timerWaitUs(1000);
  }
}


void loop()
{
  uint32_t irValue = getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (uint8_t x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    if (irValue < 50000)
    {
      displayPrintf(DISPLAY_ROW_10, "No finger!");
    }
    else
      {
        displayPrintf(DISPLAY_ROW_10, "Avg BPM = %d", beatAvg);
      }

//      Serial.print(" No finger?");

//    Serial.println();
}


uint16_t check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t activeLeds = 3;

  uint8_t readPointer, writePointer;
  max30101ReadReg(MAX30101_FIFOREADPTR, &readPointer);
  max30101ReadReg(MAX30101_FIFOWRITEPTR, &writePointer);


  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLeds * 3;

    // Read data from the FIFO register
    uint8_t fifoData[bytesLeftToRead];

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
//        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
//        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
//        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.
//
        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLeds * 3)); //Trim toGet to be a multiple of the samples we need to read
      }
//
      bytesLeftToRead -= toGet;

      max30101ReadFifo(fifoData, toGet);

      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = fifoData[0];
        temp[1] = fifoData[1];
        temp[0] = fifoData[2];

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));

        tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLeds > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = fifoData[3];
          temp[1] = fifoData[4];
          temp[0] = fifoData[5];

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

      sense.IR[sense.head] = tempLong;
        }

        if (activeLeds > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = fifoData[6];
          temp[1] = fifoData[7];
          temp[0] = fifoData[8];

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;
        }

        toGet -= activeLeds * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr



  return (numberOfSamples); //Let the world know how much new data we found
}

