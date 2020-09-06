/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_PORT BOARD_LED_RED_PORT
#define BOARD_LED_PIN  BOARD_LED_RED_PIN

#define EXAMPLE_I2C_MASTER_BASE    (I2C0_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define I2C_MASTER_SLAVE_ADDR_7BIT (0x48U)
#define I2C_BAUDRATE               (100000) /* 100K */
#define I2C_DATA_LENGTH            (3)     /* MAX is 256 */

#define TEMP_SCALE_FACTOR          (10000)
#define CENTER_TEMP				   ((int)(27.875 * TEMP_SCALE_FACTOR)) //center temperature
#define TEMP_GUARDBAND			   ((int)(1.125 * TEMP_SCALE_FACTOR)) //+/- guardband

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;

uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];

i2c_master_handle_t g_m_handle;

volatile bool g_MasterCompletionFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
	i2c_master_transfer_t masterXfer = {0};
	status_t reVal                   = kStatus_Fail;

	/* Enable clock of uart0. */
	CLOCK_EnableClock(kCLOCK_Uart0);
	/* Ser DIV of uart0. */
	CLOCK_SetClkDivider(kCLOCK_DivUsartClk, 1U);
	/* Enable clock of i2c0. */
	CLOCK_EnableClock(kCLOCK_I2c0);

    /* Init output LED GPIO. */
    GPIO_PortInit(GPIO, BOARD_LED_PORT);
    /* Board pin init */
    BOARD_InitPins();
    BOARD_BootClockIRC12M();
    BOARD_InitDebugConsole();

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kI2C_2PinOpenDrain;
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    /* Initialize the I2C master peripheral */
    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    /* Create the I2C handle for the non-blocking transfer */
    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER, &g_m_handle, i2c_master_callback, NULL);

    LED_GREEN_OFF();
    LED_BLUE_OFF();
    LED_RED_OFF();

    while (1)
    {
        uint8_t deviceAddress     = 0x00U;
        /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
          start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
        masterXfer.slaveAddress   = I2C_MASTER_SLAVE_ADDR_7BIT;
        masterXfer.direction      = kI2C_Read;
        masterXfer.subaddress     = (uint32_t)deviceAddress;
        masterXfer.subaddressSize = 1;
        masterXfer.data           = g_master_rxBuff;
        masterXfer.dataSize       = I2C_DATA_LENGTH - 1;
        masterXfer.flags          = kI2C_TransferDefaultFlag;

        reVal = I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);

        /*  Reset master completion flag to false. */
        g_MasterCompletionFlag = false;

        if (reVal != kStatus_Success)
        {
            return -1;
        }

        /*  Wait for transfer completed. */
        while (!g_MasterCompletionFlag)
        {
        }
        g_MasterCompletionFlag = false;

        //byte 0 is MSByte, byte 1 is LSByte shifted by 3 or 4, depending on EM setting
        //Default is EM=0, so we shift by 4 to get the actual raw value.
        int16_t tempRaw = ((((uint16_t)g_master_rxBuff[0] << 8) | g_master_rxBuff[1])) >> 4;
        int32_t temperature = tempRaw*625;
        int32_t tempWhole = temperature/TEMP_SCALE_FACTOR;
        int32_t tempDecimal = temperature-(TEMP_SCALE_FACTOR*tempWhole);
        PRINTF("Temperature (C): %i.%i\r\n", tempWhole, tempDecimal);

        if(temperature > CENTER_TEMP+TEMP_GUARDBAND)
        {
        	//indicate too hot!
        	LED_BLUE_OFF();
        	LED_GREEN_OFF();
        	LED_RED_ON();
        }
        else if(temperature < CENTER_TEMP-TEMP_GUARDBAND)
        {
        	//indicate too cold!
        	LED_BLUE_ON();
        	LED_GREEN_OFF();
        	LED_RED_OFF();
        }
        else
        {
        	//indicate just right!
        	LED_BLUE_OFF();
        	LED_GREEN_ON();
        	LED_RED_OFF();
        }

        SysTick_DelayTicks(250U);
    }
}
