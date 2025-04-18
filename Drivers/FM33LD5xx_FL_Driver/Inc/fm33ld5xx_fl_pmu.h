/**
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_pmu.h
  * @author  FMSH Application Team
  * @brief   Head file of PMU FL Module
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


/* Define to prevent recursive inclusion---------------------------------------------------------------*/
#ifndef __FM33LD5XX_FL_PMU_H
#define __FM33LD5XX_FL_PMU_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes -------------------------------------------------------------------------------------------*/
#include "fm33ld5xx_fl_def.h"
/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */
/* Exported types -------------------------------------------------------------------------------------*/
/** @defgroup PMU_FL_ES_INIT PMU Exported Init structures
  * @{
  */

/**
  * @brief FL PMU Init Sturcture definition
  */
typedef struct
{
    /* 低功耗模式配置 */
    uint32_t powerMode;

    /* 睡眠模式配置 */
    uint32_t deepSleep;

    /* flash唤醒速度控制 */
    uint32_t flashWakeupMode;
           
    /* 额外唤醒延迟 T1A*/
    uint32_t wakeupDelayT1a;
  
    /* CANFD RAM电源模式配置 */
    uint32_t CANFDRAMMode;
  
    /*I-CACHE RAM电源模式配置 */
    uint32_t ICRAMMode;
  
    /* 系统RAM电源模式配置 */
    uint32_t SYSTEMRAMMode;

} FL_PMU_SleepInitTypeDef;
/**
  * @}
  */
/* Exported constants ---------------------------------------------------------------------------------*/
/** @defgroup PMU_FL_Exported_Constants PMU Exported Constants
  * @{
  */

#define    PMU_CR_AVREF_SH_Pos                                    (24U)
#define    PMU_CR_AVREF_SH_Msk                                    (0x3U << PMU_CR_AVREF_SH_Pos)
#define    PMU_CR_AVREF_SH                                        PMU_CR_AVREF_SH_Msk

#define    PMU_CR_WKFSEL_Pos                                      (10U)
#define    PMU_CR_WKFSEL_Msk                                      (0x3U << PMU_CR_WKFSEL_Pos)
#define    PMU_CR_WKFSEL                                          PMU_CR_WKFSEL_Msk

#define    PMU_CR_SLPDP_Pos                                       (9U)
#define    PMU_CR_SLPDP_Msk                                       (0x1U << PMU_CR_SLPDP_Pos)
#define    PMU_CR_SLPDP                                           PMU_CR_SLPDP_Msk

#define    PMU_CR_PMOD_Pos                                        (0U)
#define    PMU_CR_PMOD_Msk                                        (0x3U << PMU_CR_PMOD_Pos)
#define    PMU_CR_PMOD                                            PMU_CR_PMOD_Msk

#define    PMU_WKTR_FWUP_Pos                                      (3U)
#define    PMU_WKTR_FWUP_Msk                                      (0x1U << PMU_WKTR_FWUP_Pos)
#define    PMU_WKTR_FWUP                                          PMU_WKTR_FWUP_Msk

#define    PMU_WKTR_T1A_Pos                                       (0U)
#define    PMU_WKTR_T1A_Msk                                       (0x3U << PMU_WKTR_T1A_Pos)
#define    PMU_WKTR_T1A                                           PMU_WKTR_T1A_Msk

#define    PMU_WKFR0_SVDWKF_Pos                                   (30U)
#define    PMU_WKFR0_SVDWKF_Msk                                   (0x1U << PMU_WKFR0_SVDWKF_Pos)
#define    PMU_WKFR0_SVDWKF                                       PMU_WKFR0_SVDWKF_Msk

#define    PMU_WKFR0_UART3WKF_Pos                                 (29U)
#define    PMU_WKFR0_UART3WKF_Msk                                 (0x1U << PMU_WKFR0_UART3WKF_Pos)
#define    PMU_WKFR0_UART3WKF                                     PMU_WKFR0_UART3WKF_Msk

#define    PMU_WKFR0_UART2WKF_Pos                                 (28U)
#define    PMU_WKFR0_UART2WKF_Msk                                 (0x1U << PMU_WKFR0_UART2WKF_Pos)
#define    PMU_WKFR0_UART2WKF                                     PMU_WKFR0_UART2WKF_Msk

#define    PMU_WKFR0_UART1WKF_Pos                                 (27U)
#define    PMU_WKFR0_UART1WKF_Msk                                 (0x1U << PMU_WKFR0_UART1WKF_Pos)
#define    PMU_WKFR0_UART1WKF                                     PMU_WKFR0_UART1WKF_Msk

#define    PMU_WKFR0_UART0WKF_Pos                                 (26U)
#define    PMU_WKFR0_UART0WKF_Msk                                 (0x1U << PMU_WKFR0_UART0WKF_Pos)
#define    PMU_WKFR0_UART0WKF                                     PMU_WKFR0_UART0WKF_Msk

#define    PMU_WKFR0_COMP4WKF_Pos                                 (24U)
#define    PMU_WKFR0_COMP4WKF_Msk                                 (0x1U << PMU_WKFR0_COMP4WKF_Pos)
#define    PMU_WKFR0_COMP4WKF                                     PMU_WKFR0_COMP4WKF_Msk

#define    PMU_WKFR0_COMP3WKF_Pos                                 (23U)
#define    PMU_WKFR0_COMP3WKF_Msk                                 (0x1U << PMU_WKFR0_COMP3WKF_Pos)
#define    PMU_WKFR0_COMP3WKF                                     PMU_WKFR0_COMP3WKF_Msk

#define    PMU_WKFR0_COMP2WKF_Pos                                 (20U)
#define    PMU_WKFR0_COMP2WKF_Msk                                 (0x1U << PMU_WKFR0_COMP2WKF_Pos)
#define    PMU_WKFR0_COMP2WKF                                     PMU_WKFR0_COMP2WKF_Msk

#define    PMU_WKFR0_COMP1WKF_Pos                                 (17U)
#define    PMU_WKFR0_COMP1WKF_Msk                                 (0x1U << PMU_WKFR0_COMP1WKF_Pos)
#define    PMU_WKFR0_COMP1WKF                                     PMU_WKFR0_COMP1WKF_Msk

#define    PMU_WKFR0_IOWKF_Pos                                    (16U)
#define    PMU_WKFR0_IOWKF_Msk                                    (0x1U << PMU_WKFR0_IOWKF_Pos)
#define    PMU_WKFR0_IOWKF                                        PMU_WKFR0_IOWKF_Msk

#define    PMU_WKFR0_CANFDWKF_Pos                                 (15U)
#define    PMU_WKFR0_CANFDWKF_Msk                                 (0x1U << PMU_WKFR0_CANFDWKF_Pos)
#define    PMU_WKFR0_CANFDWKF                                     PMU_WKFR0_CANFDWKF_Msk

#define    PMU_WKFR0_LPTWKF_Pos                                   (12U)
#define    PMU_WKFR0_LPTWKF_Msk                                   (0x1U << PMU_WKFR0_LPTWKF_Pos)
#define    PMU_WKFR0_LPTWKF                                       PMU_WKFR0_LPTWKF_Msk

#define    PMU_WKFR0_BST16WKF_Pos                                 (10U)
#define    PMU_WKFR0_BST16WKF_Msk                                 (0x1U << PMU_WKFR0_BST16WKF_Pos)
#define    PMU_WKFR0_BST16WKF                                     PMU_WKFR0_BST16WKF_Msk

#define    PMU_WKFR0_DBGWKF_Pos                                   (9U)
#define    PMU_WKFR0_DBGWKF_Msk                                   (0x1U << PMU_WKFR0_DBGWKF_Pos)
#define    PMU_WKFR0_DBGWKF                                       PMU_WKFR0_DBGWKF_Msk

#define    PMU_WKFR1_WKPx_FLAG_Pos                                (0U)
#define    PMU_WKFR1_WKPx_FLAG_Msk                                (0xffU << PMU_WKFR1_WKPx_FLAG_Pos)
#define    PMU_WKFR1_WKPx_FLAG                                    PMU_WKFR1_WKPx_FLAG_Msk

#define    PMU_IER_LPRUN_Pos                                      (0U)
#define    PMU_IER_LPRUN_Msk                                      (0x1U << PMU_IER_LPRUN_Pos)
#define    PMU_IER_LPRUN                                          PMU_IER_LPRUN_Msk

#define    PMU_ISR_LPRUN_Pos                                      (0U)
#define    PMU_ISR_LPRUN_Msk                                      (0x1U << PMU_ISR_LPRUN_Pos)
#define    PMU_ISR_LPRUN                                          PMU_ISR_LPRUN_Msk

#define    PMU_RAMCR_CANFDRAMPG_Pos                               (10U)
#define    PMU_RAMCR_CANFDRAMPG_Msk                               (0x3U << PMU_RAMCR_CANFDRAMPG_Pos)
#define    PMU_RAMCR_CANFDRAMPG                                   PMU_RAMCR_CANFDRAMPG_Msk

#define    PMU_RAMCR_ICRAMPG_Pos                                  (8U)
#define    PMU_RAMCR_ICRAMPG_Msk                                  (0x3U << PMU_RAMCR_ICRAMPG_Pos)
#define    PMU_RAMCR_ICRAMPG                                      PMU_RAMCR_ICRAMPG_Msk

#define    PMU_RAMCR_SYSRAMPG_Pos                                 (8U)
#define    PMU_RAMCR_SYSRAMPG_Msk                                 (0x3U << PMU_RAMCR_SYSRAMPG_Pos)
#define    PMU_RAMCR_SYSRAMPG                                     PMU_RAMCR_SYSRAMPG_Msk

#define    PMU_BUFCR_AVREFBUF_OUTEN_Pos                           (5U)
#define    PMU_BUFCR_AVREFBUF_OUTEN_Msk                           (0x1U << PMU_BUFCR_AVREFBUF_OUTEN_Pos)
#define    PMU_BUFCR_AVREFBUF_OUTEN                               PMU_BUFCR_AVREFBUF_OUTEN_Msk

#define    PMU_BUFCR_AVREFBUF_EN_Pos                              (4U)
#define    PMU_BUFCR_AVREFBUF_EN_Msk                              (0x1U << PMU_BUFCR_AVREFBUF_EN_Pos)
#define    PMU_BUFCR_AVREFBUF_EN                                  PMU_BUFCR_AVREFBUF_EN_Msk

#define    PMU_BUFCR_VPTATBUFFER_OUTEN_Pos                        (3U)
#define    PMU_BUFCR_VPTATBUFFER_OUTEN_Msk                        (0x1U << PMU_BUFCR_VPTATBUFFER_OUTEN_Pos)
#define    PMU_BUFCR_VPTATBUFFER_OUTEN                            PMU_BUFCR_VPTATBUFFER_OUTEN_Msk

#define    PMU_BUFCR_VPTATBUFFER_EN_Pos                           (2U)
#define    PMU_BUFCR_VPTATBUFFER_EN_Msk                           (0x1U << PMU_BUFCR_VPTATBUFFER_EN_Pos)
#define    PMU_BUFCR_VPTATBUFFER_EN                               PMU_BUFCR_VPTATBUFFER_EN_Msk

#define    PMU_PTATCR_EN_Pos                                      (0U)
#define    PMU_PTATCR_EN_Msk                                      (0x1U << PMU_PTATCR_EN_Pos)
#define    PMU_PTATCR_EN                                           PMU_PTATCR_EN_Msk





#define    FL_PMU_WAKEUP0_PIN                                     (0x1U << 0U)
#define    FL_PMU_WAKEUP1_PIN                                     (0x1U << 1U)
#define    FL_PMU_WAKEUP2_PIN                                     (0x1U << 2U)
#define    FL_PMU_WAKEUP3_PIN                                     (0x1U << 3U)
#define    FL_PMU_WAKEUP4_PIN                                     (0x1U << 4U)
#define    FL_PMU_WAKEUP5_PIN                                     (0x1U << 5U)
#define    FL_PMU_WAKEUP6_PIN                                     (0x1U << 6U)
#define    FL_PMU_WAKEUP7_PIN                                     (0x1U << 7U)



#define    FL_PMU_AVERF_SAMPLE_HOLD_2MS                           (0x0U << PMU_CR_AVREF_SH_Pos)
#define    FL_PMU_AVERF_SAMPLE_HOLD_4MS                           (0x1U << PMU_CR_AVREF_SH_Pos)
#define    FL_PMU_AVERF_SAMPLE_HOLD_8MS                           (0x2U << PMU_CR_AVREF_SH_Pos)
#define    FL_PMU_AVERF_SAMPLE_HOLD_16MS                          (0x3U << PMU_CR_AVREF_SH_Pos)


#define    FL_PMU_SLEEP_MODE_DEEP                                 (0x1U << PMU_CR_SLPDP_Pos)
#define    FL_PMU_SLEEP_MODE_NORMAL                               (0x0U << PMU_CR_SLPDP_Pos)


#define    FL_PMU_POWER_MODE_ACTIVE                               (0x0U << PMU_CR_PMOD_Pos)
#define    FL_PMU_POWER_MODE_SLEEP_OR_DEEPSLEEP                   (0x2U << PMU_CR_PMOD_Pos)


#define    FL_PMU_FLASH_WAKEUP_MODE_NORMAL                         (0x0U << PMU_WKTR_FWUP_Pos)
#define    FL_PMU_FLASH_WAKEUP_MODE_FAST                           (0x1U << PMU_WKTR_FWUP_Pos)


#define    FL_PMU_WAKEUP_DELAY_T1A_1                               (0x0U << PMU_WKTR_T1A_Pos)
#define    FL_PMU_WAKEUP_DELAY_T1A_2                               (0x1U << PMU_WKTR_T1A_Pos)
#define    FL_PMU_WAKEUP_DELAY_T1A_3                               (0x2U << PMU_WKTR_T1A_Pos)
#define    FL_PMU_WAKEUP_DELAY_T1A_4                               (0x3U << PMU_WKTR_T1A_Pos)


#define    FL_PMU_CANFDRAM_MODE_NORMAL                            (0x0U << PMU_RAMCR_CANFDRAMPG_Pos)
#define    FL_PMU_CANFDRAM_MODE_RETENTION                         (0x1U << PMU_RAMCR_CANFDRAMPG_Pos)
#define    FL_PMU_CANFDRAM_MODE_POWERDOWN                         (0x2U << PMU_RAMCR_CANFDRAMPG_Pos)



#define    FL_PMU_ICRAM_MODE_NORMAL                               (0x0U << PMU_RAMCR_ICRAMPG_Pos)
#define    FL_PMU_ICRAM_MODE_RETENTION                            (0x1U << PMU_RAMCR_ICRAMPG_Pos)
#define    FL_PMU_ICRAM_MODE_POWERDOWN                            (0x2U << PMU_RAMCR_ICRAMPG_Pos)



#define    FL_PMU_SYSRAM_MODE_NORMAL                              (0x0U << PMU_RAMCR_SYSRAMPG_Pos)
#define    FL_PMU_SYSRAM_MODE_RETENTION                           (0x1U << PMU_RAMCR_SYSRAMPG_Pos)
#define    FL_PMU_SYSRAM_MODE_POWERDOWN                           (0x2U << PMU_RAMCR_SYSRAMPG_Pos)



#define    FL_PMU_AVREF_OUTPUT_DISABLE                            (0x0U << PMU_AUXEN_AUXEN_Pos)
#define    FL_PMU_AVREF_OUTPUT_ENABLE                             (0x2U << PMU_AUXEN_AUXEN_Pos)


/**
  * @}
  */
/* Exported functions ---------------------------------------------------------------------------------*/
/** @defgroup PMU_FL_Exported_Functions PMU Exported Functions
  * @{
  */

/**
  * @brief    Set AVREF Sample-Hold Period
  * @rmtoll   CR    AVREF_SH    FL_PMU_SetSampleHoldPeriod
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_2MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_4MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_8MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_16MS
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetSampleHoldPeriod(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->CR, PMU_CR_AVREF_SH_Msk, mode);
}

/**
  * @brief    Get AVREF Sample-Hold Period
  * @rmtoll   CR    AVREF_SH    FL_PMU_GetSampleHoldPeriod
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_2MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_4MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_8MS
  *           @arg @ref FL_PMU_AVERF_SAMPLE_HOLD_16MS
  */
__STATIC_INLINE uint32_t FL_PMU_GetSampleHoldPeriod(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->CR, PMU_CR_AVREF_SH_Msk));
}

/**
  * @brief    Set RCHF Frequency After Wakeup
  * @rmtoll   CR    WKFSEL    FL_PMU_SetRCHFWakeupFrequency
  * @param    PMUx PMU instance
  * @param    Freq This parameter can be one of the following values:
  *           @arg @ref FL_PMU_RCHF_WAKEUP_FREQ_8MHZ
  *           @arg @ref FL_PMU_RCHF_WAKEUP_FREQ_16MHZ
  *           @arg @ref FL_PMU_RCHF_WAKEUP_FREQ_24MHZ
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetRCHFWakeupFrequency(PMU_Type* PMUx, uint32_t Freq)
{
    MODIFY_REG(PMUx->CR, PMU_CR_WKFSEL_Msk, Freq);
}



/**
  * @brief    Set Sleep Mode
  * @rmtoll   CR    SLPDP    FL_PMU_SetSleepMode
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_SLEEP_MODE_DEEP
  *           @arg @ref FL_PMU_SLEEP_MODE_NORMAL
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetSleepMode(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->CR, PMU_CR_SLPDP_Msk, mode);
}

/**
  * @brief    Get Sleep Mode Setting
  * @rmtoll   CR    SLPDP    FL_PMU_GetSleepMode
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_SLEEP_MODE_DEEP
  *           @arg @ref FL_PMU_SLEEP_MODE_NORMAL
  */
__STATIC_INLINE uint32_t FL_PMU_GetSleepMode(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->CR, PMU_CR_SLPDP_Msk));
}

/**
  * @brief    Set Low Power Mode
  * @rmtoll   CR    PMOD    FL_PMU_SetLowPowerMode
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_POWER_MODE_ACTIVE
  *           @arg @ref FL_PMU_POWER_MODE_SLEEP_OR_DEEPSLEEP
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetLowPowerMode(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->CR, PMU_CR_PMOD_Msk, mode);
}

/**
  * @brief    Get Low Power Mode Setting
  * @rmtoll   CR    PMOD    FL_PMU_GetLowPowerMode
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  */
__STATIC_INLINE uint32_t FL_PMU_GetLowPowerMode(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->CR, PMU_CR_PMOD_Msk));
}



/**
  * @brief    Set FLASH Wakeup Mode 
  * @rmtoll   WKTR    FWUP    FL_PMU_SetFlashWakeupMode
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_FLASH_WAKEUP_MODE_NORMAL
  *           @arg @ref FL_PMU_FLASH_WAKEUP_MODE_FAST
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetFlashWakeupMode(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->WKTR, PMU_WKTR_FWUP_Msk, mode);
}

/**
  * @brief    Get Stop Release Mode After Sleep
  * @rmtoll   WKTR    FWUP    FL_PMU_GetFastWakeupMode
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_FLASH_WAKEUP_MODE_NORMAL
  *           @arg @ref FL_PMU_FLASH_WAKEUP_MODE_FAST
  */
__STATIC_INLINE uint32_t FL_PMU_GetT1AWakeupDelay(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKTR, PMU_WKTR_FWUP_Msk));
}

/**
  * @brief    Set Extra Wakeup Delay Under Sleep/DeepSleep Mode
  * @rmtoll   WKTR    T1A    FL_PMU_SetWakeupDelay
  * @param    PMUx PMU instance
  * @param    time This parameter can be one of the following values:
  *           @arg @ref FL_PMU_WAKEUP_DELAY_1
  *           @arg @ref FL_PMU_WAKEUP_DELAY_2
  *           @arg @ref FL_PMU_WAKEUP_DELAY_3
  *           @arg @ref FL_PMU_WAKEUP_DELAY_4
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetT1AWakeupDelay(PMU_Type* PMUx, uint32_t time)
{
    MODIFY_REG(PMUx->WKTR, PMU_WKTR_T1A_Msk, time);
}

/**
  * @brief    Get Extra Wakeup Delay Under Sleep/DeepSleep Mode Setting
  * @rmtoll   WKTR    T1A    FL_PMU_GetWakeupDelay
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_WAKEUP_DELAY_1
  *           @arg @ref FL_PMU_WAKEUP_DELAY_2
  *           @arg @ref FL_PMU_WAKEUP_DELAY_3
  *           @arg @ref FL_PMU_WAKEUP_DELAY_4
  */
__STATIC_INLINE uint32_t FL_PMU_GetWakeupDelay(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKTR, PMU_WKTR_T1A_Msk));
}

/**
  * @brief    Get SVD interrupt wakeup flag
  * @rmtoll   WKFR0    SVDWKF    FL_PMU_IsActiveFlag_WakeupSVD
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupSVD(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_SVDWKF_Msk) == (PMU_WKFR0_SVDWKF_Msk));
}

/**
  * @brief    Get UART3 interrupt wakeup flag
  * @rmtoll   WKFR0    UART3WKF    FL_PMU_IsActiveFlag_WakeupUART3
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupUART3(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_UART3WKF_Msk) == (PMU_WKFR0_UART3WKF_Msk));
}

/**
  * @brief    Get UART2 interrupt wakeup flag
  * @rmtoll   WKFR0    UART2WKF    FL_PMU_IsActiveFlag_WakeupUART2
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupUART2(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_UART2WKF_Msk) == (PMU_WKFR0_UART2WKF_Msk));
}

/**
  * @brief    Get UART1 interrupt wakeup flag
  * @rmtoll   WKFR0    UART1WKF    FL_PMU_IsActiveFlag_WakeupUART1
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupUART1(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_UART1WKF_Msk) == (PMU_WKFR0_UART1WKF_Msk));
}

/**
  * @brief    Get UART0 interrupt wakeup flag
  * @rmtoll   WKFR0    UART0WKF    FL_PMU_IsActiveFlag_WakeupUART0
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupUART0(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_UART0WKF_Msk) == (PMU_WKFR0_UART0WKF_Msk));
}

/**
  * @brief    Get COMP4 interrrupt wakeup flag
  * @rmtoll   WKFR0    COMP4WKF    FL_PMU_IsActiveFlag_WakeupCOMP4
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupCOMP4(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_COMP4WKF_Msk) == (PMU_WKFR0_COMP4WKF_Msk));
}

/**
  * @brief    Get COMP3 interrrupt wakeup flag
  * @rmtoll   WKFR0    COMP3WKF    FL_PMU_IsActiveFlag_WakeupCOMP3
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupCOMP3(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_COMP3WKF_Msk) == (PMU_WKFR0_COMP3WKF_Msk));
}

/**
  * @brief    Get COMP2 interrrupt wakeup flag
  * @rmtoll   WKFR0    COMP2WKF    FL_PMU_IsActiveFlag_WakeupCOMP2
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupCOMP2(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_COMP2WKF_Msk) == (PMU_WKFR0_COMP2WKF_Msk));
}

/**
  * @brief    Get COMP1 interrrupt wakeup flag
  * @rmtoll   WKFR0    COMP1WKF    FL_PMU_IsActiveFlag_WakeupCOMP1
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupCOMP1(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_COMP1WKF_Msk) == (PMU_WKFR0_COMP1WKF_Msk));
}

/**
  * @brief    Get IO interrupt wakeup flag
  * @rmtoll   WKFR0    IOWKF    FL_PMU_IsActiveFlag_WakeupEXTI
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupEXTI(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_IOWKF_Msk) == (PMU_WKFR0_IOWKF_Msk));
}

/**
  * @brief    Get CANFD interrupt wakeup flag
  * @rmtoll   WKFR0    CANFDWKF    FL_PMU_IsActiveFlag_WakeupCANFD
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupCANFD(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_CANFDWKF_Msk) == (PMU_WKFR0_CANFDWKF_Msk));
}

/**
  * @brief    Get LPTIM16 interrupt wakeup flag
  * @rmtoll   WKFR0    LPTWKF    FL_PMU_IsActiveFlag_WakeupLPTIM16
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupLPTIM16(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_LPTWKF_Msk) == (PMU_WKFR0_LPTWKF_Msk));
}

/**
  * @brief    Get BSTIM16 interrupt wakeup flag
  * @rmtoll   WKFR0    BST16WKF    FL_PMU_IsActiveFlag_WakeupBSTIM16
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupBSTIM16(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_BST16WKF_Msk) == (PMU_WKFR0_BST16WKF_Msk));
}

/**
  * @brief    Get CPU Debugger wakeup flag
  * @rmtoll   WKFR0    DBGWKF    FL_PMU_IsActiveFlag_WakeupDBG
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupDBG(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR0, PMU_WKFR0_DBGWKF_Msk) == (PMU_WKFR0_DBGWKF_Msk));
}

/**
  * @brief    Get pinx wakeup flag
  * @rmtoll   WKFR1    WKPx_FLAG    FL_PMU_IsActiveFlag_WakeupPIN
  * @param    PMUx PMU instance
  * @param    Pin This parameter can be one of the following values:
  *           @arg @ref FL_PMU_WAKEUP0_PIN
  *           @arg @ref FL_PMU_WAKEUP1_PIN
  *           @arg @ref FL_PMU_WAKEUP2_PIN
  *           @arg @ref FL_PMU_WAKEUP3_PIN
  *           @arg @ref FL_PMU_WAKEUP4_PIN
  *           @arg @ref FL_PMU_WAKEUP5_PIN
  *           @arg @ref FL_PMU_WAKEUP6_PIN
  *           @arg @ref FL_PMU_WAKEUP7_PIN
  * @retval   Returned value can be one of the following values:
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_WakeupPIN(PMU_Type* PMUx, uint32_t Pin)
{
    return (uint32_t)(READ_BIT(PMUx->WKFR1, ((Pin & 0xff) << 0x0U)) == ((Pin & 0xff) << 0x0U));
}

/**
  * @brief    Clear pinx wakeup flag
  * @rmtoll   WKFR1    WKPx_FLAG    FL_PMU_ClearFlag_WakeupPIN
  * @param    PMUx PMU instance
  * @param    Pin This parameter can be one of the following values:
  *           @arg @ref FL_PMU_WAKEUP0_PIN
  *           @arg @ref FL_PMU_WAKEUP1_PIN
  *           @arg @ref FL_PMU_WAKEUP2_PIN
  *           @arg @ref FL_PMU_WAKEUP3_PIN
  *           @arg @ref FL_PMU_WAKEUP4_PIN
  *           @arg @ref FL_PMU_WAKEUP5_PIN
  *           @arg @ref FL_PMU_WAKEUP6_PIN
  *           @arg @ref FL_PMU_WAKEUP7_PIN
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_ClearFlag_WakeupPIN(PMU_Type* PMUx, uint32_t Pin)
{
    WRITE_REG(PMUx->WKFR1, ((Pin & 0xff) << 0x0U));
}

/**
  * @brief    LPRUN interrupt enable
  * @rmtoll   IER    LPRUN    FL_PMU_EnableIT_LPRun
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableIT_LPRun(PMU_Type* PMUx)
{
    SET_BIT(PMUx->IER, PMU_IER_LPRUN_Msk);
}

/**
  * @brief    Get LPRUN interrupt enable status
  * @rmtoll   IER    LPRUN    FL_PMU_IsEnabledIT_LPRun
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledIT_LPRun(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->IER, PMU_IER_LPRUN_Msk) == PMU_IER_LPRUN_Msk);
}

/**
  * @brief    LPRUN interrupt disable
  * @rmtoll   IER    LPRUN    FL_PMU_DisableIT_LPRun
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableIT_LPRun(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->IER, PMU_IER_LPRUN_Msk);
}

/**
  * @brief    Get LPRUN interrupt flag
  * @rmtoll   ISR    LPRUN    FL_PMU_IsActiveFlag_LPRun
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsActiveFlag_LPRun(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->ISR, PMU_ISR_LPRUN_Msk) == (PMU_ISR_LPRUN_Msk));
}

/**
  * @brief    Clear LPRUN interrupt flag
  * @rmtoll   ISR    LPRUN    FL_PMU_ClearFlag_LPRun
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_ClearFlag_LPRun(PMU_Type* PMUx)
{
    WRITE_REG(PMUx->ISR, PMU_ISR_LPRUN_Msk);
}

/**
  * @brief    Set CANFD RAM power control
  * @rmtoll   RAMCR    CANFDRAMPG    FL_PMU_SetCANFDRAMPG
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_CANFDRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_CANFDRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_CANFDRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_CANFDRAM_MODE_NORMAL
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetCANFDRAMPG(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->RAMCR, PMU_RAMCR_CANFDRAMPG_Msk, mode);
}

/**
  * @brief    Get CANFD RAM power control setting
  * @rmtoll   RAMCR    CANFDRAMPG    FL_PMU_GetCANFDRAMPG
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_CANFDRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_CANFDRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_CANFDRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_CANFDRAM_MODE_NORMAL
  */
__STATIC_INLINE uint32_t FL_PMU_GetCANFDRAMPG(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->RAMCR, PMU_RAMCR_CANFDRAMPG_Msk));
}

/**
  * @brief    Set I-CACHE RAM power control
  * @rmtoll   RAMCR    ICRAMPG    FL_PMU_SetICRAMPG
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_ICRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_ICRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_ICRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_ICRAM_MODE_NORMAL
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetICRAMPG(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->RAMCR, PMU_RAMCR_ICRAMPG_Msk, mode);
}

/**
  * @brief    Get I-CACHE RAM power control setting
  * @rmtoll   RAMCR    ICRAMPG    FL_PMU_GetICRAMPG
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_ICRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_ICRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_ICRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_ICRAM_MODE_NORMAL
  */
__STATIC_INLINE uint32_t FL_PMU_GetICRAMPG(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->RAMCR, PMU_RAMCR_ICRAMPG_Msk));
}

/**
  * @brief    Set SYSTEM RAM power control
  * @rmtoll   RAMCR    SYSRAMPG    FL_PMU_SetSYSRAMPG
  * @param    PMUx PMU instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_PMU_SYSRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_SYSRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_SYSRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_SYSRAM_MODE_NORMAL
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_SetSYSRAMPG(PMU_Type* PMUx, uint32_t mode)
{
    MODIFY_REG(PMUx->RAMCR, PMU_RAMCR_SYSRAMPG_Msk, mode);
}

/**
  * @brief    Get SYSTEM RAM power control setting
  * @rmtoll   RAMCR    SYSRAMPG    FL_PMU_GetSYSRAMPG
  * @param    PMUx PMU instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_PMU_SYSRAM_MODE_NORMAL
  *           @arg @ref FL_PMU_SYSRAM_MODE_RETENTION
  *           @arg @ref FL_PMU_SYSRAM_MODE_POWERDOWN
  *           @arg @ref FL_PMU_SYSRAM_MODE_NORMAL
  */
__STATIC_INLINE uint32_t FL_PMU_GetSYSRAMPG(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->RAMCR, PMU_RAMCR_SYSRAMPG_Msk));
}

/**
  * @brief    Enable AVREF Buffer Output
  * @rmtoll   BUFCR    AVREFBUF_OUTEN    FL_PMU_EnableAVREFBufferOutput
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableAVREFBufferOutput(PMU_Type* PMUx)
{
    SET_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_OUTEN_Msk);
}

/**
  * @brief    Get AVREF Buffer Output Enable Status
  * @rmtoll   BUFCR    AVREFBUF_OUTEN    FL_PMU_IsEnabledAVREFBufferOutput
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledAVREFBufferOutput(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_OUTEN_Msk) == PMU_BUFCR_AVREFBUF_OUTEN_Msk);
}

/**
  * @brief    Disable AVREF Buffer Output
  * @rmtoll   BUFCR    AVREFBUF_OUTEN    FL_PMU_DisableAVREFBufferOutput
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableAVREFBufferOutput(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_OUTEN_Msk);
}

/**
  * @brief    Enable AVREF Buffer
  * @rmtoll   BUFCR    AVREFBUF_EN    FL_PMU_EnableAVREFBuffer
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableAVREFBuffer(PMU_Type* PMUx)
{
    SET_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_EN_Msk);
}

/**
  * @brief    Get AVREF Buffer Enable Status
  * @rmtoll   BUFCR    AVREFBUF_EN    FL_PMU_IsEnabledAVREFBuffer
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledAVREFBuffer(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_EN_Msk) == PMU_BUFCR_AVREFBUF_EN_Msk);
}

/**
  * @brief    Disable AVREF Buffer 
  * @rmtoll   BUFCR    AVREFBUF_EN    FL_PMU_DisableAVREFBuffer
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableAVREFBuffer(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->BUFCR, PMU_BUFCR_AVREFBUF_EN_Msk);
}

/**
  * @brief    Enable VPTAT Buffer Output
  * @rmtoll   BUFCR    VPTATBUFFER_OUTEN    FL_PMU_EnableVPTATBufferOutput
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableVPTATBufferOutput(PMU_Type* PMUx)
{
    SET_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_OUTEN_Msk);
}

/**
  * @brief    Get VPTAT Buffer Output Enable Status
  * @rmtoll   BUFCR    VPTATBUFFER_OUTEN    FL_PMU_IsEnabledVPTATBufferOutput
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledVPTATBufferOutput(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_OUTEN_Msk) == PMU_BUFCR_VPTATBUFFER_OUTEN_Msk);
}

/**
  * @brief    Disable VPTAT Buffer Output
  * @rmtoll   BUFCR    VPTATBUFFER_OUTEN    FL_PMU_DisableVPTATBufferOutput
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableVPTATBufferOutput(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_OUTEN_Msk);
}

/**
  * @brief    Enable VPTAT Buffer
  * @rmtoll   BUFCR    VPTATBUFFER_EN    FL_PMU_EnableVPTATBuffer
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableVPTATBuffer(PMU_Type* PMUx)
{
    SET_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_EN_Msk);
}

/**
  * @brief    Get VPTAT Buffer Enable Status
  * @rmtoll   BUFCR    VPTATBUFFER_EN    FL_PMU_IsEnabledVPTATBuffer
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledVPTATBuffer(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_EN_Msk) == PMU_BUFCR_VPTATBUFFER_EN_Msk);
}

/**
  * @brief    Disable VPTAT Buffer 
  * @rmtoll   BUFCR    VPTATBUFFER_EN    FL_PMU_DisableVPTATBuffer
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableVPTATBuffer(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->BUFCR, PMU_BUFCR_VPTATBUFFER_EN_Msk);
}

/**
  * @brief    Enable Temperatue Sensor
  * @rmtoll   PTATCR    PTAT_EN    FL_PMU_EnableTemperatureSensor
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_EnableTemperatureSensor(PMU_Type* PMUx)
{
    SET_BIT(PMUx->PTATCR, PMU_PTATCR_EN_Msk);
}

/**
  * @brief    Get Temperatue Sensor Enable Status
  * @rmtoll   PTATCR    PTAT_EN    FL_PMU_IsEnabledTemperatureSensor
  * @param    PMUx PMU instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_PMU_IsEnabledTemperatureSensor(PMU_Type* PMUx)
{
    return (uint32_t)(READ_BIT(PMUx->PTATCR, PMU_PTATCR_EN_Msk) == PMU_PTATCR_EN_Msk);
}

/**
  * @brief    Disable Temperatue Sensor
  * @rmtoll   PTATCR    PTAT_EN    FL_PMU_DisableTemperatureSensor
  * @param    PMUx PMU instance
  * @retval   None
  */
__STATIC_INLINE void FL_PMU_DisableTemperatureSensor(PMU_Type* PMUx)
{
    CLEAR_BIT(PMUx->PTATCR, PMU_PTATCR_EN_Msk);
}
/**
  * @}
  */

/** @defgroup PMU_FL_EF_Init Initialization and de-initialization functions
  * @{
  */
FL_ErrorStatus FL_PMU_Sleep_DeInit(PMU_Type *PMUx);
FL_ErrorStatus FL_PMU_Sleep_Init(PMU_Type *PMUx, FL_PMU_SleepInitTypeDef *LPM_InitStruct);
void FL_PMU_StructInit(FL_PMU_SleepInitTypeDef *LPM_InitStruct);

/**
  * @}
  */


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __FM33LD5XX_FL_PMU_H*/

/*************************Py_Code_Generator Version: 0.1-0.14-0.1 @ 2025-02-21*************************/
/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
