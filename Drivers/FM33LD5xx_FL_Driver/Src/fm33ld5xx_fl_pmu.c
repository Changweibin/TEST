/**
  ****************************************************************************************************
  * @file    fm33ld5xx_fl_pmu.c
  * @author  FMSH Application Team
  * @brief   Src file of PMU FL Module
  ****************************************************************************************************
  * @attention    
  * Copyright 2024 SHANGHAI FUDAN MICROELECTRONICS GROUP CO., LTD.(FUDAN MICRO.)
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
  ****************************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "fm33ld5xx_fl.h"

/** @addtogroup FM33FH0xx_FL_Driver
  * @{
  */

/** @addtogroup PMU
  * @{
  */

#ifdef FL_PMU_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup PMU_FL_Private_Macros
  * @{
  */



#define         IS_FL_PMU_INSTANCE(INSTANCE)                (((INSTANCE) == PMU))

#define         IS_FL_PMU_MODE(__VALUE__)                   (((__VALUE__) == FL_PMU_POWER_MODE_ACTIVE)||\
                                                             ((__VALUE__) == FL_PMU_POWER_MODE_SLEEP_OR_DEEPSLEEP))



#define         IS_FL_PMU_DEEPSLEEP(__VALUE__)              (((__VALUE__) == FL_PMU_SLEEP_MODE_DEEP)||\
                                                             ((__VALUE__) == FL_PMU_SLEEP_MODE_NORMAL))



#define         IS_FL_PMU_WAKEUPDELAYT1A(__VALUE__)          (((__VALUE__) == FL_PMU_WAKEUP_DELAY_T1A_1)||\
                                                             ((__VALUE__) == FL_PMU_WAKEUP_DELAY_T1A_2)||\
                                                             ((__VALUE__) == FL_PMU_WAKEUP_DELAY_T1A_3)||\
                                                             ((__VALUE__) == FL_PMU_WAKEUP_DELAY_T1A_4))
                                                             

#define         IS_FL_PMU_FLASHWAKEUPMODE(__VALUE__)              (((__VALUE__) == FL_PMU_FLASH_WAKEUP_MODE_FAST)||\
                                                             ((__VALUE__) == FL_PMU_FLASH_WAKEUP_MODE_NORMAL))


#define         IS_FL_PMU_CANFDRAMMODE(__VALUE__)              (((__VALUE__) == FL_PMU_CANFDRAM_MODE_NORMAL)||\
                                                             ((__VALUE__) == FL_PMU_CANFDRAM_MODE_RETENTION)||\
                                                             ((__VALUE__) == FL_PMU_CANFDRAM_MODE_POWERDOWN))
                                                             
                                                             
#define         IS_FL_PMU_ICRAMMODE(__VALUE__)              (((__VALUE__) == FL_PMU_ICRAM_MODE_NORMAL)||\
                                                             ((__VALUE__) == FL_PMU_ICRAM_MODE_RETENTION)||\
                                                             ((__VALUE__) == FL_PMU_ICRAM_MODE_POWERDOWN))
                                                             
                                                             
#define         IS_FL_PMU_SYSTEMRAMMODE(__VALUE__)              (((__VALUE__) == FL_PMU_SYSRAM_MODE_NORMAL)||\
                                                             ((__VALUE__) == FL_PMU_SYSRAM_MODE_RETENTION)||\
                                                             ((__VALUE__) == FL_PMU_SYSRAM_MODE_POWERDOWN))
                                                             

 

/**
  *@}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PMU_FL_EF_Init
  * @{
  */

/**
  * @brief  复位pmu外设
  *
  * @param  外设入口地址
  *
  * @retval 返回错误状态，可能值:
  *         -FL_PASS 外设寄存器值回复复位值
  *         -FL_FAIL 位成功执行
  */
FL_ErrorStatus FL_PMU_Sleep_DeInit(PMU_Type *PMUx)
{
    FL_ErrorStatus status = FL_FAIL;
    /* 参数合法性检测 */
    assert_param(IS_FL_PMU_INSTANCE(PMU));
    PMUx->CR   = 0x01000000U;
    PMUx->WKTR = 0x000000D1U;
    PMUx->IER  = 0x00000000U;
    status = FL_PASS;
    return status;
}


/**
  * @brief  根据lpm_initstruct结构体包含的配置信息配置pmu寄存器
  *
  * @note   为更好的睡眠功耗用户可能需要根据实际应用，调用 @ref fm33fh0xx_fl_pmu.h中的其他接口
  *          来完成睡眠前的模式配置，包括睡眠行为和唤醒后的行为(注：此函数会关闭BOR)
  * @param  PMUx  外设入口地址
  * @param  LPM_InitStruct 指向一个 @ref FL_PMU_SleepInitTypeDef 类型的结构体，它包含指定PMU外设的配置信息
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS PMU配置成功
  */
FL_ErrorStatus FL_PMU_Sleep_Init(PMU_Type *PMUx, FL_PMU_SleepInitTypeDef *LPM_InitStruct)
{
    FL_ErrorStatus status = FL_FAIL;
    if(LPM_InitStruct != NULL)
    {
        /* 参数合法性检查 */
        assert_param(IS_FL_PMU_INSTANCE(PMUx));
        assert_param(IS_FL_PMU_DEEPSLEEP(LPM_InitStruct->deepSleep));
        assert_param(IS_FL_PMU_WAKEUPDELAYT1A(LPM_InitStruct->wakeupDelayT1a));
        assert_param(IS_FL_PMU_FLASHWAKEUPMODE(LPM_InitStruct->flashWakeupMode));        
        assert_param(IS_FL_PMU_CANFDRAMMODE(LPM_InitStruct->CANFDRAMMode));  
        assert_param(IS_FL_PMU_ICRAMMODE(LPM_InitStruct->ICRAMMode));  
        assert_param(IS_FL_PMU_SYSTEMRAMMODE(LPM_InitStruct->SYSTEMRAMMode)); 
      
        /* 唤醒速度配置 */
        FL_PMU_SetFlashWakeupMode(PMUx, LPM_InitStruct->flashWakeupMode);
        /* 唤醒时间 */
        FL_PMU_SetT1AWakeupDelay(PMUx, LPM_InitStruct->wakeupDelayT1a);
        /* CANFD RAM配置 */
        FL_PMU_SetCANFDRAMPG(PMUx, LPM_InitStruct->CANFDRAMMode);
        /* I-CACHE RAM配置 */
        FL_PMU_SetICRAMPG(PMUx, LPM_InitStruct->ICRAMMode);
        /* SYSTEM RAM配置 */
        FL_PMU_SetSYSRAMPG(PMUx, LPM_InitStruct->SYSTEMRAMMode);

        /* M0系统控制器，一般配置为0即可*/
        SCB->SCR = 0;
        /* 睡眠模式 */
        FL_PMU_SetSleepMode(PMUx, LPM_InitStruct->deepSleep);
        /* AVREF采样保持周期配置为 2ms*/
        FL_PMU_SetSampleHoldPeriod(PMUx, FL_PMU_AVERF_SAMPLE_HOLD_2MS);
        status = FL_PASS;
    }
    return status;
}

/**
  * @brief  LPM_InitStruct 为默认配置
  * @param  LPM_InitStruct 指向需要将值设置为默认配置的结构体 @ref FL_PMU_SleepInitTypeDef structure
  *         结构体
  * @retval None
  */
void FL_PMU_StructInit(FL_PMU_SleepInitTypeDef *LPM_InitStruct)
{
    if(LPM_InitStruct != NULL)
    {
        LPM_InitStruct->deepSleep           = FL_PMU_SLEEP_MODE_NORMAL;
        LPM_InitStruct->wakeupDelayT1a      = FL_PMU_WAKEUP_DELAY_T1A_4;
        LPM_InitStruct->CANFDRAMMode        = FL_PMU_CANFDRAM_MODE_NORMAL;
        LPM_InitStruct->ICRAMMode           = FL_PMU_ICRAM_MODE_NORMAL;
        LPM_InitStruct->SYSTEMRAMMode       = FL_PMU_SYSRAM_MODE_NORMAL;
    }
}

/**
 * @}
 */

#endif /* FL_PMU_DRIVER_ENABLED */

/**
 * @}
 */

/**
 * @}
 */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/

