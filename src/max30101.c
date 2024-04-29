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
// Status Registers
static const uint8_t MAX30105_INTSTAT1 =    0x00;
static const uint8_t MAX30105_INTSTAT2 =    0x01;
static const uint8_t MAX30105_INTENABLE1 =    0x02;
static const uint8_t MAX30105_INTENABLE2 =    0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR =  0x04;
static const uint8_t MAX30105_FIFOOVERFLOW =  0x05;
static const uint8_t MAX30105_FIFOREADPTR =   0x06;
static const uint8_t MAX30105_FIFODATA =    0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG =    0x08;
static const uint8_t MAX30105_MODECONFIG =    0x09;
static const uint8_t MAX30105_PARTICLECONFIG =  0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP =   0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP =   0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP =   0x0E;
static const uint8_t MAX30105_LED_PROX_AMP =  0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;



// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH =   0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID =    0xFE;
static const uint8_t MAX30105_PARTID =      0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK =   ~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE =   0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE =  0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK = ~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK = ~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE =  0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK = ~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = ~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =  ~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 =   0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 =   0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 =   0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 =   0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 =  0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 =  0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK =   0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK =   0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK =   0x7F;
static const uint8_t MAX30105_SHUTDOWN =    0x80;
static const uint8_t MAX30105_WAKEUP =      0x00;

static const uint8_t MAX30105_RESET_MASK =    0xBF;
static const uint8_t MAX30105_RESET =       0x40;

static const uint8_t MAX30105_MODE_MASK =     0xF8;
static const uint8_t MAX30105_MODE_REDONLY =  0x02;
static const uint8_t MAX30105_MODE_REDIRONLY =  0x03;
static const uint8_t MAX30105_MODE_MULTILED =   0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK =   0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 =   0x00;
static const uint8_t MAX30105_ADCRANGE_4096 =   0x20;
static const uint8_t MAX30105_ADCRANGE_8192 =   0x40;
static const uint8_t MAX30105_ADCRANGE_16384 =  0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 =   0x00;
static const uint8_t MAX30105_SAMPLERATE_100 =  0x04;
static const uint8_t MAX30105_SAMPLERATE_200 =  0x08;
static const uint8_t MAX30105_SAMPLERATE_400 =  0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 =  0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 =   0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 =  0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 =  0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 =  0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK =    0xF8;
static const uint8_t MAX30105_SLOT2_MASK =    0x8F;
static const uint8_t MAX30105_SLOT3_MASK =    0xF8;
static const uint8_t MAX30105_SLOT4_MASK =    0x8F;

static const uint8_t SLOT_NONE =        0x00;
static const uint8_t SLOT_RED_LED =       0x01;
static const uint8_t SLOT_IR_LED =        0x02;
static const uint8_t SLOT_GREEN_LED =       0x03;
static const uint8_t SLOT_NONE_PILOT =      0x04;
static const uint8_t SLOT_RED_PILOT =     0x05;
static const uint8_t SLOT_IR_PILOT =      0x06;
static const uint8_t SLOT_GREEN_PILOT =     0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

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
//uint8_t mode;

void max30101Setup()
{
  // Set samples
  rdModifyWrReg(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, MAX30105_SAMPLEAVG_4);

  // Enable rollover
  rdModifyWrReg(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);

  // Set mode
  rdModifyWrReg(MAX30105_MODECONFIG, MAX30105_MODE_MASK, MAX30105_MODE_MULTILED);

  // Set ADC range
  rdModifyWrReg(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, MAX30105_ADCRANGE_4096);

  // Set sample rate
  rdModifyWrReg(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, MAX30105_SAMPLERATE_400);

  // Set pulse width
  rdModifyWrReg(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, MAX30105_PULSEWIDTH_411);

  // Set pulse amplitude
  max30101WriteReg(MAX30105_LED1_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30105_LED2_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30105_LED3_PULSEAMP, 0x1F);
  max30101WriteReg(MAX30105_LED_PROX_AMP, 0x1F);

  // Enable slots
  rdModifyWrReg(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, SLOT_RED_LED);
  rdModifyWrReg(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, SLOT_IR_LED << 4);
  rdModifyWrReg(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, SLOT_GREEN_LED);

  max30101WriteReg(MAX30105_LED1_PULSEAMP, 0x0A);
  max30101WriteReg(MAX30105_LED3_PULSEAMP, 0);

  LOG_INFO("turned off GREEN \r\n\r\n");

}

void max30101Config()
{
  uint8_t rdData;
  max30101ReadReg(0xff, &rdData);
  LOG_INFO("Read data = 0x%0x\r\n", rdData);
  writeModeReg();
  // Set mode to heartrate
//  max30101WriteReg(MODE_REG_ADDR, 0x02);
  // Set pulse width
//  max30101ReadReg(CONFIG_REG, &rdData);
//  rdData |= PULSE_WIDTH;
//  max30101WriteReg(CONFIG_REG, rdData);

  rdData = 0x05;

  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  rdData = 0x05;
  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  rdData = 0x05;
  max30101ReadReg(FIFO_DATA_ADDR, &rdData);
  LOG_INFO("Value of RED LED is %d\r\n", rdData);

  rdData = 0x05;
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

void max30101ReadFifo(uint8_t* rdData, uint8_t len)
{
    uint8_t reg = FIFO_DATA_ADDR;
    I2C_TransferSeq_TypeDef i2cPacket;
    i2cPacket.addr = MAX30101_DEVICE_ID << 1;
    i2cPacket.flags = I2C_FLAG_WRITE_READ;
    i2cPacket.buf[0].data = &reg;
    i2cPacket.buf[0].len = sizeof(reg);

  //  i2c_send_cmd(&i2cPacket);
  //
  //  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
  //    i2cPacket.flags = I2C_FLAG_READ;
      i2cPacket.buf[1].data = rdData;
      i2cPacket.buf[1].len = len;

  //  i2cPacket.buf[1].data = rdData;
  //  i2cPacket.buf[1].len = sizeof(rdData);

    // Write-read into/from WR_PTR register
    i2c_send_cmd(&i2cPacket);
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

//  i2c_send_cmd(&i2cPacket);
//
//  i2cPacket.addr = MAX30101_DEVICE_ID << 1;
//    i2cPacket.flags = I2C_FLAG_READ;
    i2cPacket.buf[1].data = rdData;
    i2cPacket.buf[1].len = 1;

//  i2cPacket.buf[1].data = rdData;
//  i2cPacket.buf[1].len = sizeof(rdData);

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


void rdModifyWrReg(uint8_t reg, uint8_t mask, uint8_t data)
{
  uint8_t rdData;
  max30101ReadReg(reg, &rdData);
  rdData = rdData & mask;
  max30101WriteReg(reg, rdData | data);

}

//void print_sensor_reg()
//{
//  LOG_INFO("Value of mode is 0x%02x\n", mode);
//}

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

      LOG_INFO("YOOOOOOO..........\r\n");
      LOG_INFO("Delta = %ld, ", delta);

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

    LOG_INFO("IR = %d, ", irValue);
    if (irValue < 50000)
      {
      LOG_INFO("No finger?");
      displayPrintf(DISPLAY_ROW_10, "No finger!");

      }
    else
      {

        //    Serial.print("IR=");
        //    Serial.print(irValue);
        displayPrintf(DISPLAY_ROW_10, "Avg BPM = %d", beatAvg);
            LOG_INFO("BPM = %0.2f, ", beatsPerMinute);
        //    Serial.print(", BPM=");
        //    Serial.print(beatsPerMinute);
        //    Serial.print(", Avg BPM=");
            LOG_INFO("Avg BPM = %d, ", beatAvg);
        //    Serial.print(beatAvg);

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
  max30101ReadReg(MAX30105_FIFOREADPTR, &readPointer);
  max30101ReadReg(MAX30105_FIFOWRITEPTR, &writePointer);


  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
      LOG_INFO("In here\r\n");
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLeds * 3;

    // Read data from the FIFO register
    uint8_t fifoData[bytesLeftToRead];
//    max30101ReadFifo(fifoData, bytesLeftToRead);
//    max30101ReadReg(fifoData, bytesLeftToRead);

    //Get ready to read a burst of data from the FIFO register
//    _i2cPort->beginTransmission(MAX30105_ADDRESS);
//    _i2cPort->write(MAX30105_FIFODATA);
//    _i2cPort->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
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
      //Request toGet number of bytes from sensor
//      _i2cPort->requestFrom(MAX30105_ADDRESS, toGet);

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

