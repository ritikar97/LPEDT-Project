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


#include "em_gpio.h"
#include "gpio.h"





void gpioInit()
{

	  // Set sensor enable GPIO pin to push-pull mode
	  GPIO_PinModeSet(MAX30101Port, resetPin, gpioModePushPull, false);
	  GPIO_PinModeSet(MAX30101Port, MFIOPin, gpioModePushPull, false);
	  GPIO_PinModeSet(GPSPort, gpsResetPin, gpioModePushPull, false);
	  GPIO_PinModeSet(GPSPort, gpsSelPin, gpioModePushPull, false);

	  gpsReset(GPS_RESET_HIGH);


} // gpioInit()


void gpioSensorEnable()
{
    // Enable the temp-humidity sensor
    GPIO_PinOutSet(MAX30101Port, MFIOPin);

}

void gpioSensorDisable()
{
    // Disable the temp-humidity sensor
//    GPIO_PinOutClear(sensorPort, sensorPin);
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






