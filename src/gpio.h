/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 *
 * Student edit: Add your name and email address here:
 * @student    Ritika Ramchandani, Ritika.Ramchandani@Colorado.edu
 *
 
 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include "em_gpio.h"

// Reset pin is PD15
#define MAX30101Port (gpioPortD)
#define resetPin  (12)
#define MFIOPin (11)
#define GPSPort  (gpioPortC)
#define gpsSelPin (7)
#define gpsResetPin (9)

#define GPS_RESET_HIGH  (1)
#define GPS_RESET_LOW   (0)
#define GPS_SEL_HIGH  (1)
#define GPS_SEL_LOW   (0)

/*
 * @func        gpioInit()
 *
 * @brief       Initialisation routine for GPIO
 *
 * @parameters  none
 *
 * @returns     void
 */
void gpioInit();


/*
 * @func        gpioSensorEnable()
 *
 * @brief       Enable the temp-humidity sensor pin
 *
 * @parameters  none
 *
 * @returns     void
 */
void LCDEnable();


/*
 * @func        gpioSensorDisable()
 *
 * @brief       Disable the temp-humidity sensor pin
 *
 * @parameters  none
 *
 * @returns     void
 */
void LCDDisable();

void gpsReset(uint8_t flag);

void powerGPS(uint8_t flag);

/*
 * @func        gpioSetDisplayExtcomin()
 *
 * @brief       Enable EXTCOMIN Pin
 */
void gpioSetDisplayExtcomin(bool setPin);
void gpioLEDOn();
void gpioLEDOff();


#endif /* SRC_GPIO_H_ */
