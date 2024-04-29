#include "em_cmu.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include <stdbool.h>
#include <string.h>
#include "src/lcd.h"
#include "src/gpio.h"

#define LEUART_PORT   gpioPortC
#define LEUART_RX_PIN (10)
#define LEUART_TX_PIN (11)
#define LEUART_BUFSIZE (128) // Adjust buffer size as per your requirements

// Global variables
char leuartBuffer[LEUART_BUFSIZE];
volatile int leuartIndex = 0;
volatile bool gnggaReceived = false;

void initLEUART(void)
{
    // Enable LEUART peripheral clock
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_HFLE, true);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
//    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); // Select HFXO as high-frequency clock source

    CMU_ClockEnable(cmuClock_LEUART0, true);

    // Configure LEUART pins
    GPIO_PinModeSet(LEUART_PORT, LEUART_TX_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(LEUART_PORT, LEUART_RX_PIN, gpioModeInput, 0);

    // Initialize LEUART settings
    LEUART_Init_TypeDef leuartInit = LEUART_INIT_DEFAULT;
    LEUART_Init(LEUART0, &leuartInit);

    // Enable RX and TX for LEUART
    LEUART0->CTRL |= LEUART_CTRL_INV;
    LEUART0->ROUTELOC0 = USART_ROUTELOC0_TXLOC_LOC3 | USART_ROUTELOC0_RXLOC_LOC30;
    LEUART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;

    // Enable RXDATAV interrupt
    LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
    NVIC_ClearPendingIRQ(LEUART0_IRQn);
    NVIC_EnableIRQ(LEUART0_IRQn);

    gpioLEDOn();
}

void LEUART0_IRQHandler(void)
{
    gpioLEDOff();
    // Check if RX data valid interrupt is triggered
    if (LEUART0->IF & LEUART_IF_RXDATAV)
    {
        displayPrintf(DISPLAY_ROW_8, "IRQ!");
        char receivedChar = LEUART_Rx(LEUART0);

        // Check for the start of a GNGGA sentence
        if (receivedChar == '$')
        {
            leuartIndex = 0;
            leuartBuffer[leuartIndex++] = receivedChar;
        }
        else if (leuartIndex > 0 && receivedChar != '\n' && leuartIndex < LEUART_BUFSIZE - 1)
        {
            leuartBuffer[leuartIndex++] = receivedChar;

            // Check if the complete GNGGA sentence is received
            if (strstr(leuartBuffer, "GNGGA") != NULL && receivedChar == '\n')
            {
                leuartBuffer[leuartIndex] = '\0'; // Null-terminate the string
                gnggaReceived = true; // Flag that a GNGGA sentence is received
            }
        }
    }
}

bool getGPSStatus()
{
    return gnggaReceived;
}
