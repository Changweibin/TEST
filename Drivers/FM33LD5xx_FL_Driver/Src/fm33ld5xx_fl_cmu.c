/**
  ****************************************************************************************************
  * @file    fm33ld5xx_fl_cmu.c
  * @author  FMSH Application Team
  * @brief   Src file of CMU FL Module
  *******************************************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"

/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */

/** @addtogroup CMU
  * @{
  */

#ifdef FL_CMU_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CMU_FL_Private_Macros
  * @{
  */

/**
  * @brief  设置系统工作使用时钟。
  * @param  系统工作时钟源及频率。
  * @note   函数中用到了XTHF_VALUE 宏，这个宏应该被定义为外部晶振的输入频率值。
  * @note   其他时钟源可由应用程序实现，如PLL_RCHF_40M, PLL_XTHF_64M
  * @note   PLL倍频频率范围为32M-64M
  * @note   对功耗由要求的应用，注意关闭未使用到的空闲时钟源
  *
  * @retval 系统时钟设置状态。
  *
  */
FL_ErrorStatus FL_CMU_SetSystemClock(FL_SystemClock systemClock)
{
    FL_ErrorStatus errorStatus = FL_PASS;
    /* <= 24M: 0wait; > 24 and <= 48: 1wait;> 48 and <= 72: 2wait; > 72 and <= 96: 3wait;> 96 and <= 120: 4wait;*/
    switch(systemClock)
    {
      
        case FL_SYSTEM_CLOCK_RCHF_16M:
            FL_CMU_RCHF_Enable();
            FL_CMU_RCHF_WriteTrimValue(RCHF16M_TRIM);
            FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_RCHF);
            break;
      
        case FL_SYSTEM_CLOCK_PLL32M_RCHF:
            FL_CMU_RCHF_Enable();
            FL_CMU_RCHF_WriteTrimValue(RCHF16M_TRIM);
            FL_CMU_PLL_SetClockSource(FL_CMU_PLL_CLK_SOURCE_RCHF);
            FL_CMU_PLL_SetPrescaler(FL_CMU_PLL_IPSC_DIV16);
            FL_CMU_PLL_WriteMultiplier(32 - 1);
            FL_CMU_PLL_Enable();
            while(FL_CMU_IsActiveFlag_PLLReady() == 0x0)
            {
            }
						FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_1CYCLE);
            FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_PLL);
            break;
        case FL_SYSTEM_CLOCK_PLL32M_XTHF8M:
            FL_CMU_XTHF_Enable();
            FL_DelayMs(100);
            while(FL_CMU_XTHF_IsReady() == 0x0)
            {
            }
            FL_CMU_PLL_SetClockSource(FL_CMU_PLL_CLK_SOURCE_XTHF);
            FL_CMU_PLL_SetPrescaler(FL_CMU_PLL_IPSC_DIV8);
            FL_CMU_PLL_WriteMultiplier(32 - 1);
            FL_CMU_PLL_Enable();
            while(FL_CMU_IsActiveFlag_PLLReady() == 0x0)
            {
            }
           	FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_1CYCLE);
            FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_PLL);
            break;
        case FL_SYSTEM_CLOCK_RCLP:
            FL_CMU_RCLP_Enable();
            FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_RCLP);
            break;
        case FL_SYSTEM_CLOCK_XTHF:
            FL_CMU_XTHF_Enable();
            FL_DelayMs(100);
            while(FL_CMU_XTHF_IsReady() == 0x0)
            {
            }
            FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_XTHF);
            break;
        default:
            // 不应来到此处
            errorStatus = FL_FAIL;
            break;
    }
    return errorStatus;
}

/**
  * @}
  */

/* Private consts ------------------------------------------------------------*/
/** @addtogroup CMU_FL_Private_Consts
  * @{
  */

/**
  * @brief  获取系统当前工作时钟SYSCLK。
  * @param  None
  * @note   函数中用到了XTHF_VALUE 宏，这个宏应该被定义为外部晶振的输入频率值。
  *
  * @retval 系统时钟频率 (Hz)。
  *
  */
uint32_t FL_CMU_GetSystemClockFreq(void)
{
    uint32_t frequency = 0;
    /* 获取系统时钟源 */
    switch(FL_CMU_GetSystemClockSource())
    {
        /* 系统时钟源为内部RCHF */
        case FL_CMU_SYSTEM_CLK_SOURCE_RCHF:
            /* 内部RCHF默认为8MHz ,可以配置为16或24M */
            frequency = FL_CMU_GetRCHFClockFreq();
            break;
        /* 系统时钟源为XTHF */
        case FL_CMU_SYSTEM_CLK_SOURCE_XTHF:
            frequency = XTHFClock;
            break;
        /* 系统时钟源为PLL */
        case FL_CMU_SYSTEM_CLK_SOURCE_PLL:
            frequency = FL_CMU_GetPLLClockFreq();
            break;
        /* 系统时钟源为RCLP */
        case FL_CMU_SYSTEM_CLK_SOURCE_RCLP:
            frequency = 32768;
            break;				
        default:
            frequency = FL_CMU_GetRCHFClockFreq();
            break;
    }
    return frequency;
}

/**
  * @brief  获取AHB总线时钟频率
  *
  * @retval AHB总线时钟频率(Hz)
  *
  */
uint32_t FL_CMU_GetAHBClockFreq(void)
{
    uint32_t frequency = 0;
    /* 获取AHB分频系数，AHB源自系统主时钟 */
    switch(FL_CMU_GetAHBPrescaler())
    {
        case FL_CMU_AHBCLK_PSC_DIV1:
            frequency = FL_CMU_GetSystemClockFreq();
            break;
        case FL_CMU_AHBCLK_PSC_DIV2:
            frequency = FL_CMU_GetSystemClockFreq() / 2;
            break;
        case FL_CMU_AHBCLK_PSC_DIV4:
            frequency = FL_CMU_GetSystemClockFreq() / 4;
            break;
        case FL_CMU_AHBCLK_PSC_DIV8:
            frequency = FL_CMU_GetSystemClockFreq() / 8;
            break;
        case FL_CMU_AHBCLK_PSC_DIV16:
            frequency = FL_CMU_GetSystemClockFreq() / 16;
            break;
        default:
            frequency = FL_CMU_GetSystemClockFreq();
            break;
    }
    return frequency;
}

/**
  * @brief  获取当前系统的APB1总线时钟
  *
  * @retval APB1总线时钟频率(Hz)
  *
  */
uint32_t FL_CMU_GetAPB1ClockFreq(void)
{
    uint32_t frequency = 0;
    /* 获取APB1分频系数，APB源自AHB */
    switch(FL_CMU_GetAPB1Prescaler())
    {
        case FL_CMU_APB1CLK_PSC_DIV1:
            frequency = FL_CMU_GetAHBClockFreq();
            break;
        case FL_CMU_APB1CLK_PSC_DIV2:
            frequency = FL_CMU_GetAHBClockFreq() / 2;
            break;
        case FL_CMU_APB1CLK_PSC_DIV4:
            frequency = FL_CMU_GetAHBClockFreq() / 4;
            break;
        case FL_CMU_APB1CLK_PSC_DIV8:
            frequency = FL_CMU_GetAHBClockFreq() / 8;
            break;
        case FL_CMU_APB1CLK_PSC_DIV16:
            frequency = FL_CMU_GetAHBClockFreq() / 16;
            break;
        default:
            frequency = FL_CMU_GetAHBClockFreq();
            break;
    }
    return frequency;
}
/**
  * @brief  获取当前系统的APB2总线时钟
  *
  * @retval APB2总线时钟频率(Hz)
  *
  */
uint32_t FL_CMU_GetAPB2ClockFreq(void)
{
    uint32_t frequency = 0;
      
    frequency = FL_CMU_GetAHBClockFreq() / 2;
           
    return frequency;
}

/**
  * @brief  获取RCHF输出时钟频率
  * @param  None
  *
  * @retval 返回RCHF输出时钟频率(Hz)
  *
  */
uint32_t FL_CMU_GetRCHFClockFreq(void)
{
    uint32_t frequency = 0;
   
    frequency = 16000000;
          
    return frequency;
}

/**
  * @brief  获取PLL输出时钟频率
  * @param  None
  *
  * @retval 返回PLL输出时钟频率(Hz)
  *
  */
uint32_t FL_CMU_GetPLLClockFreq(void)
{
    uint32_t frequency = 0;
    uint32_t multiplier = 0;
    /* 获取PLL时钟源 */
    switch(FL_CMU_PLL_GetClockSource())
    {
        case FL_CMU_PLL_CLK_SOURCE_RCHF:
            /* 获取RCHF配置主频 */
            frequency = FL_CMU_GetRCHFClockFreq();
            break;
        case FL_CMU_PLL_CLK_SOURCE_XTHF:
            frequency = XTHFClock;
            break;
        default:
            frequency = FL_CMU_GetRCHFClockFreq();
            break;
    }
    /* 获取PLL时钟分频系数 */
    switch(FL_CMU_PLL_GetPrescaler())
    {
        case FL_CMU_PLL_IPSC_DIV1:
            break;
        case FL_CMU_PLL_IPSC_DIV2:
            frequency /= 2;
            break;
        case FL_CMU_PLL_IPSC_DIV4:
            frequency /= 4;
            break;
        case FL_CMU_PLL_IPSC_DIV8:
            frequency /= 8;
            break;
        case FL_CMU_PLL_IPSC_DIV12:
            frequency /= 12;
            break;
        case FL_CMU_PLL_IPSC_DIV16:
            frequency /= 16;
            break;
        case FL_CMU_PLL_IPSC_DIV24:
            frequency /= 24;
            break;
        case FL_CMU_PLL_IPSC_DIV32:
            frequency /= 32;
            break;
        default:
            break;
    }
    multiplier = FL_CMU_PLL_ReadMultiplier() + 1;
    frequency *= multiplier;
    return frequency;
}


#endif /* FL_CMU_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
