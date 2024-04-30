/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.
   
   Jan 24, 2023
   Dave Sluiter: Cleaned up gpioInit() to make it less confusing for students regarding
                 drive strength setting. 

 *
 * Student edit: Add your name and email address here:
 * @student    Ritika Ramchandani, Ritika.Ramchandani@Colorado.edu
 *
 
 */



#include "gpio.h"


// LCD EXTCOMIN pin is PA0
#define EXTCOMINPort (gpioPortA)
#define EXTCOMINPin  (5)
#define LCDSELPort   (gpioPortB)
#define LCDSELPin    (11)
#define LEDPin       (1)
#define PBPin        (0)


void gpioInit()
{

	  // Set GPS pins to push-pull mode
	  GPIO_PinModeSet(GPSPort, gpsResetPin, gpioModePushPull, false);
	  GPIO_PinModeSet(GPSPort, gpsSelPin, gpioModePushPull, false);

    // Set EXTCOMIN GPIO pin to push-pull mode
	  GPIO_PinModeSet(EXTCOMINPort, EXTCOMINPin, gpioModePushPull, false);
    GPIO_PinModeSet(LCDSELPort, LCDSELPin, gpioModePushPull, false);

    // Set PB0 and LED pins
    GPIO_PinModeSet(EXTCOMINPort, LEDPin, gpioModePushPull, false);
    GPIO_PinModeSet(EXTCOMINPort, PBPin, gpioModeInputPullFilter, 1);
    GPIO_ExtIntConfig(EXTCOMINPort, PBPin, PBPin, true, true, true);

    // Get GPS out of reset
	  gpsReset(GPS_RESET_HIGH);


} // gpioInit()


void LCDEnable()
{
  // Enable the LCD
  GPIO_PinOutSet(LCDSELPort, LCDSELPin);
}

void LCDDisable()
{
    // Disable the temp-humidity sensor
//    GPIO_PinOutClear(sensorPort, sensorPin);
  // Disable the LCD
  GPIO_PinOutClear(LCDSELPort, LCDSELPin);
}

void gpsReset(uint8_t flag)
{
  if(flag)
  {
    GPIO_PinOutSet(GPSPort, gpsResetPin);
  }
  else
  {
    GPIO_PinOutClear(GPSPort, gpsResetPin);
  }
}

void powerGPS(uint8_t flag)
{
  if(flag)
  {
    GPIO_PinOutSet(GPSPort, gpsSelPin);
  }
  else
  {
    GPIO_PinOutClear(GPSPort, gpsSelPin);
  }
}


void gpioSetDisplayExtcomin(bool setPin)
{
  if(setPin)
  {
      // Enable EXTCOMIN pin
      // GPIO_PinOutSet(LCDSELPort, LCDSELPin);
      GPIO_PinOutSet(EXTCOMINPort, EXTCOMINPin);
  }
  else
  {
      // Disable EXTCOMIN pin
      // GPIO_PinOutClear(LCDSELPort, LCDSELPin);
      GPIO_PinOutClear(EXTCOMINPort, EXTCOMINPin);
  }
}

void gpioLEDOn()
{
  GPIO_PinOutSet(gpioPortA, 1);
}

void gpioLEDOff()
{
  GPIO_PinOutClear(gpioPortA, 1);
}




