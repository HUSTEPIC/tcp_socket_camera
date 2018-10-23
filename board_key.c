// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"
#include "utils.h"

// common interface includes 
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if_amo.h"
#endif

#include "pinmux.h"
#include "camera_app.h"

#include "button_if_amo.h"

#include "gpio.h"

#include "timer.h"
#include "timer_if.h"

#if defined(HAL_OLED)  
#include "hal_lcd.h"
#endif

#include "board_key.h"

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

#if defined(HAL_KEY)

static uint8 keysPressed = 0;

static volatile unsigned long g_ulBase;

// Value of keys Pressed
static uint8 keysPressed;

// Pointer to application callback
keysPressedCB_t appKeyChangeHandler = NULL;



void timerStart(unsigned long timer_ms)
{
    //
    // Turn on the timers feeding values in mSec
    //
    Timer_IF_Start(g_ulBase, TIMER_A, timer_ms);
}


void timerStop()
{
    Timer_IF_Stop(g_ulBase, TIMER_A);
}


void TimerBaseIntHandler(void)
{
    uint8 keysPressed2 = 0;
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);
    
    timerStop();

    if (appKeyChangeHandler != NULL)
    {
        if(keysPressed & SW2)
        {
            if(Key_Get_if_SW2_Pressed())
            {
                keysPressed2 |= SW2;
            }
        }
        
        if(keysPressed & SW3)
        {
            if(Key_Get_if_SW3_Pressed())
            {
                keysPressed2 |= SW3;
            }
        }

        if(keysPressed2 != keysPressed)
        {
            keysPressed = 0;
        }

        // Notify the application    
        (*appKeyChangeHandler)(keysPressed2); // 执行app中的按键回调函数        
    }
    
    Button_IF_EnableInterrupt(SW2);
    Button_IF_EnableInterrupt(SW3);
}

void timerInit()
{
    g_ulBase = TIMERA0_BASE;
    //
    // Base address for second timer
    //
//    g_ulRefBase = TIMERA1_BASE;
    //
    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
//    Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
//    Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);

    //
    // Turn on the timers feeding values in mSec
    //
//    Timer_IF_Start(g_ulBase, TIMER_A, 500);
//    Timer_IF_Start(g_ulRefBase, TIMER_A, 1000);
}

void pushButtonInterruptHandler2()
{
    timerStop();
    timerStart(20);

    keysPressed |= SW2;
}

void pushButtonInterruptHandler3()
{
    timerStop();
    timerStart(20);
    
    keysPressed |= SW3;
}

void Board_initKeys(keysPressedCB_t appKeyCB)
{
#if defined(HAL_KEY)
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

    // SW3 
    // GPIO_13   PIN04  ---camera-reset 控制脚
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

    // SW2 
    // GPIO_22   PIN15  ---camera-reset 控制脚
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);    
#endif

    timerInit();

    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3);
    Button_IF_EnableInterrupt(SW2);
    Button_IF_EnableInterrupt(SW3);
    
    // Set the application callback
    appKeyChangeHandler = appKeyCB;  
}

#endif

