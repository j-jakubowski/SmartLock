/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "audioThread.h"

#include "FreeRTOS.h"
#include "task.h"


void setupBoard(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

    USER_LED_OFF();
}


/*!
 * @brief Main function
 */
int main(void)
{
	setupBoard();

	setupAudioThread();


    if (xTaskCreate(audioThread,
    		        "AudioThread",
					configMINIMAL_STACK_SIZE + 100,
					NULL,
					tskIDLE_PRIORITY + 1,
					NULL)!=  pdPASS)
    {
        PRINTF("Audio Thread creation failed");
        while (1){}
    }

    vTaskStartScheduler();
    while(1)
    {
    }
}
