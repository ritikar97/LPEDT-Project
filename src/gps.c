#include "em_cmu.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include <stdbool.h>
#include <string.h>
#include "src/lcd.h"
#include "src/gpio.h"

#define LEUART_PORT   gpioPortC
#define LEUART_RX_PIN (11)
#define LEUART_TX_PIN (10)
#define LEUART_BUFSIZE (256) // Adjust buffer size as per your requirements

// Global variables
char leuartBuffer[LEUART_BUFSIZE];
volatile int leuartIndex = 0;
volatile bool gnggaReceived = false;

void initLEUART(void)
{
    // Enable LEUART peripheral clock
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(LEUART_PORT, LEUART_RX_PIN, gpioModeInput, 0);

    CMU_ClockEnable(cmuClock_HFLE, true);
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
//    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); // Select HFXO as high-frequency clock source

    CMU_ClockEnable(cmuClock_LEUART0, true);
    CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); 
    

    // Configure LEUART pins
    // GPIO_PinModeSet(LEUART_PORT, LEUART_TX_PIN, gpioModePushPull, 1);
    

    // Initialize LEUART settings
    LEUART_Init_TypeDef leuartInit = LEUART_INIT_DEFAULT;
    LEUART_Init(LEUART0, &leuartInit);

    // Enable RX and TX for LEUART
    LEUART0->CTRL |= LEUART_CTRL_INV;
    LEUART0->ROUTELOC0 = /* USART_ROUTELOC0_TXLOC_LOC3 | */ USART_ROUTELOC0_RXLOC_LOC15;
    LEUART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN /*| USART_ROUTEPEN_TXPEN*/;

    // Enable RXDATAV interrupt
    LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
    NVIC_ClearPendingIRQ(LEUART0_IRQn);
    NVIC_EnableIRQ(LEUART0_IRQn);
}

// void LEUART0_IRQHandler(void)
// {
//     leuartIndex = 0;
//     // Check if RX data valid interrupt is triggered
//     if (LEUART0->IF & LEUART_IF_RXDATAV)
//     {
//         char receivedChar = LEUART_Rx(LEUART0);

//         // Check for the start of a GNGGA sentence
//         if (receivedChar == '$')
//         {
//             leuartIndex = 0;
//             leuartBuffer[leuartIndex++] = receivedChar;
//         }
//         else if (leuartIndex > 0 && receivedChar != '\n' && leuartIndex < LEUART_BUFSIZE - 1)
//         {
//             leuartBuffer[leuartIndex++] = receivedChar;

//             // Check if the complete GNGGA sentence is received
//             if (strstr(leuartBuffer, "GNGGA") != NULL && receivedChar == '\n')
//             {
//                 leuartBuffer[leuartIndex] = '\0'; // Null-terminate the string
//                 gnggaReceived = true; // Flag that a GNGGA sentence is received
//             }
//             else
//             {
//                 gnggaReceived = false;
//             }
//         }
//     }
// }

void LEUART0_IRQHandler(void)
{
    // Check if RX data valid interrupt is triggered
    if (LEUART0->IF & LEUART_IF_RXDATAV)
    {
        char receivedChar = LEUART_Rx(LEUART0);

        // Check if it's a new sentence or still parsing the existing one
        if (leuartIndex == 0 && receivedChar != '$') {
            // Ignore characters until a new sentence starts
            return;
        }

        // Store the received character
        leuartBuffer[leuartIndex++] = receivedChar;

        // Check if the buffer is full or a complete GNGGA sentence is received
        if (leuartIndex >= LEUART_BUFSIZE - 1 || (receivedChar == '\n' && leuartIndex > 7)) // Ensure at least $GNGGA,\n is received
        {
            // Null-terminate the string
            leuartBuffer[leuartIndex] = '\0';

            // Check if the buffer contains a GNGGA sentence
            if (strstr(leuartBuffer, "$GNGGA") != NULL)
            {
                // Tokenize and print GNGGA data
                printGNGGAData();
                gnggaReceived = true; // Flag that a GNGGA sentence is received
            }
            else
            {
                // Clear the buffer if it's not a GNGGA sentence
                leuartIndex = 0;
            }
        }
    }
}

bool getGPSStatus()
{
    return gnggaReceived;
}

// Function to tokenize GNGGA data and print latitude, longitude, and altitude
void printGNGGAData()
{
    char *token;
    char *copy = strdup(leuartBuffer); // Make a copy of the sentence to avoid modifying the original
    
    // Split the sentence into fields using comma as delimiter
    token = strtok(copy, ",");
    
    // Loop through the fields to find latitude, longitude, and altitude
    int fieldIndex = 0;
    float latitude, longitude, altitude;
    while (token != NULL)
    {
        if (fieldIndex == 2) // Latitude
        {
            latitude = atof(token);
        }
        else if (fieldIndex == 4) // Longitude
        {
            longitude = atof(token);
        }
        else if (fieldIndex == 9) // Altitude
        {
            altitude = atof(token);
        }
        
        // Move to the next field
        token = strtok(NULL, ",");
        fieldIndex++;
    }
    
    free(copy); // Free the memory allocated for the copy

    // Print latitude, longitude, and altitude on the screen
    displayPrintf(DISPLAY_ROW_7, "Latitude: %.6f", latitude);
    displayPrintf(DISPLAY_ROW_8, "Longitude: %.6f", longitude);
    displayPrintf(DISPLAY_ROW_9, "Altitude: %.2f meters", altitude);
}
