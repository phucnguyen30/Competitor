/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_inputmux.h"
#include "fsl_iocon.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BOARD_TEST_LED_PORT 3U
#define APP_BOARD_TEST_LED_PIN  13U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
  

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /*Execution Time for Board_InitPins*/
    
    unsigned int start_time, stop_time, cycle_count;
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;
    while(SysTick->VAL!=0);
    start_time = SysTick->VAL;
    BOARD_InitPins();
    stop_time = SysTick->VAL;
    cycle_count = start_time - stop_time;
    
    /*End of execution time*/
    
    BOARD_BootClockPLL180M();
    BOARD_InitDebugConsole();
    
    /*PinMux Enable*/
    /* enable clock for InputMux */
    
    /*Execution Time for InputMux_Init*/
    
    volatile unsigned int start_time, stop_time, end_time[20] = {0};
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;
    while(SysTick->VAL!=0);
    start_time = SysTick->VAL;
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* Set pin mux for single pin */
    INPUTMUX_Init(INPUTMUX);
    stop_time = SysTick->VAL;
    end_time[0] = start_time - stop_time;
    /*End of execution time for InputMux_Init*/
    
    /* Set pin mux for group of pins */
    const iocon_group_t gpio_pins[] = {
    {0, 24, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)},
    {0, 31, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)},
    };
    
    /*Execution Time for InputMux-DeInit*/
    unsigned int start_time, stop_time, cycle_count;
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;
    while(SysTick->VAL!=0);
    start_time = SysTick->VAL;
    INPUTMUX_DeInit(INPUTMUX);
    stop_time = SysTick->VAL;
    end_time[1] = start_time - stop_time;
    /*End of execution time for InputMux-DeInit*/
    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    

    /* Init output LED GPIO. */
    GPIO_PortInit(GPIO, APP_BOARD_TEST_LED_PORT);
    GPIO_PinInit(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, &led_config);
    GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, 1);
    GPIO_PortInit(GPIO, 0);
    
    
    while (1)
    {
        GPIO_PortToggle(GPIO, APP_BOARD_TEST_LED_PORT, 1u << APP_BOARD_TEST_LED_PIN);
        delay();
    }
}
