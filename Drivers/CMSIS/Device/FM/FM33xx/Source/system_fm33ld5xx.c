 /**************************************************************************//**
  * @file     system_fm33ld5xx.c
  * @brief    CMSIS Starmc1 Device Peripheral Access Layer Source File for
  *           Device FM33LD5XX
  * @version  V1.0.0
  * @date     18. February 2025
  *
  * @note
  *
  ******************************************************************************
  * @attention
  * Copyright 2025 SHANGHAI FUDAN MICROELECTRONICS GROUP CO., LTD.(FUDAN MICRO.)
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met: 
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS"AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *******************************************************************************************************
  */

#include "system_fm33ld5xx.h"

/* Clock Variable definitions ------------------------------------------------*/

uint32_t XTHFClock = XTHF_DEFAULT_VALUE;        /*!< External High-freq Osc Clock Frequency (XTHF) */
uint32_t SystemCoreClock = HCLK_DEFAULT_VALUE;  /*!< System Clock Frequency (Core Clock) */

/* Clock functions -----------------------------------------------------------*/
/**
 *  @brief Update the core clock frequency variable: SystemCoreClock
 *
 */
static uint32_t SystemPLLClockUpdate(void)
{
    uint32_t clock = 0;
    
    /* Acquire PLL multiplier and calculate PLL frequency */
    clock = 1000000 * (((CMU->PLLCR >> 16) & 0x3FF) + 1);
       
    return clock;
}

void SystemCoreClockUpdate(void)
{
    switch ((CMU->SYSCLKCR >> 0) & 0x7)
    {        
        case 1: /* XTHF */
            SystemCoreClock = XTHFClock;
            break;
        
        case 2: /* PLL */
            SystemCoreClock = SystemPLLClockUpdate();
            break;              
        case 3: /* RCLP*/
            SystemCoreClock = 32768;
            break;       
        case 5: /* RCLP */
            SystemCoreClock = 32768;
            break;
        
        default:           
             SystemCoreClock = 16000000;
             break;
    }
                
        //获取AHB时钟分频
    switch ((CMU->SYSCLKCR >> 8) & 0x7)
    { 
            case 0: /* AHB不分频 */
                SystemCoreClock = SystemCoreClock/1;
                break;
            
            case 1: /* AHB不分频 */
                SystemCoreClock = SystemCoreClock/1;
                break;
            
            case 2: /* AHB不分频 */
                SystemCoreClock = SystemCoreClock/1;
                break;        

            case 3: /* AHB不分频 */
                SystemCoreClock = SystemCoreClock/1;
                break;        

            case 4: /* AHB 2分频 */
                SystemCoreClock = SystemCoreClock/2;
                break;                
                
            case 5: /* AHB 4分频 */
                SystemCoreClock = SystemCoreClock/4;
                break;        
                
            case 6: /* AHB 8分频 */
                SystemCoreClock = SystemCoreClock/8;
                break;                    
                
            case 7: /* AHB 16分频 */
                SystemCoreClock = SystemCoreClock/16;
                break;    
                
            default:/* AHB不分频 */
                SystemCoreClock = SystemCoreClock/1;
                break;            
         }
}

/**
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 20U) | (3UL << 22U)); /* set CP10 and CP11 Full Access */
#endif

#if (__USE_ICACHE == 1)
	  __DSB();
    __ISB();
	  SCB->ICIALLU = 0UL;  
	  __DSB();
    __ISB();
    SCB->CCR |= SCB_CCR_IC_Msk;
    __DSB();
    __ISB();
#endif
    
    /* Reset  SYSCLK selection */
    CMU->SYSCLKCR = 0x0A000000U;
    
	  FLASH->ECCCR = 0xAACC;
	
    /* Enable PAD Operation Clock */
    CMU->PCLKCR1 |= (0x1U << 7);

    /* IWDT config */
    CMU->PCLKCR1 |= (0x1U << 5);
    IWDT->SERV = 0x12345A5AU;
    IWDT->CR |= 0x07U;
    IWDT->SERV = 0x12345A5AU;

    /* Disable IWDT & WWDT, enable other peripherals(e.g. timers) under Debug Mode */
    SCU->DBGCFG = 0x03U;
    
    /* Load clock trim value */
    CMU->RCHFTR = RCHF16M_TRIM;
    CMU->RCLPTR = RCLP_TRIM;

    /* Enable SWD port pull up */
    GPIOG->PUDEN |= (0x3U << 8);

    /* Update System Core Clock */
    SystemCoreClockUpdate();
    IWDT->SERV = 0x12345A5AU;
}
