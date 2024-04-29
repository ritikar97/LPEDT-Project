///*
// * gps.c
// *
// *  Created on: 02 Mar 2024
// *      Author: <rira3427@colorado.edu>
// */
//
//#include "src/i2c.h"
//#include "em_core.h"
//#include <stdint.h>
//#include "src/gps.h"
//#include "em_device.h"
//#include "em_cmu.h"
//#include "em_emu.h"
//#include "em_gpio.h"
////#include "em_leuart.h"
//#include "em_chip.h"
//#include "em_usart.h"
//#include "timers.h"
//
//#define INCLUDE_LOG_DEBUG 1
//#include "src/log.h"
//
//#define LEUART_PORT   gpioPortC
//#define LEUART_RX_PIN (10)
//#define LEUART_TX_PIN (11)
//
//void initLEUART(void)
//{
//    // Enable LEUART peripheral clock
//    CMU_ClockEnable(cmuClock_GPIO, true);
//
//    // Configure LEUART pins
//    GPIO_PinModeSet(LEUART_PORT, LEUART_TX_PIN, gpioModePushPull, 1);
//    GPIO_PinModeSet(LEUART_PORT, LEUART_RX_PIN, gpioModeInput, 0);
//
//    // Enable LE (low energy) clocks
//    CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules
//    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); // Set a reference clock
//
//    // Enable clocks for LEUART0
//    CMU_ClockEnable(cmuClock_LEUART0, true);
//    CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); // Don't prescale LEUART clock
//
//    // Initialize LEUART settings
//    LEUART_Init_TypeDef leuartInit = LEUART_INIT_DEFAULT;
//    LEUART_Init(LEUART0, &leuartInit);
//
//
//
////     // Enable RX and TX for LEUART
////     LEUART0->CTRL |= LEUART_CTRL_INV;
////     LEUART0->ROUTELOC0 = /*USART_ROUTELOC0_TXLOC_LOC3 | */USART_ROUTELOC0_RXLOC_LOC30;
////     LEUART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN/* | USART_ROUTEPEN_TXPEN*/;
//
//
//
//     LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
//     NVIC_ClearPendingIRQ(LEUART0_IRQn);
//     NVIC_EnableIRQ(LEUART0_IRQn);
//}
