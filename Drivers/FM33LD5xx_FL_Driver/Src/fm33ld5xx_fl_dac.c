/**
  ****************************************************************************************************
  * @file    fm33ld5xx_fl_dac.c
  * @author  FMSH Application Team
  * @brief   Source file of DAC FL Module
  ****************************************************************************************************
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
  ****************************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"
/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */

/** @addtogroup DAC
  * @{
  */

#ifdef FL_DAC_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup DAC_FL_Private_Macros
  * @{
  */

#define         IS_FL_DAC_INSTANCE(INSTANCE)                            (((INSTANCE) == DAC0)||\
                                                                        ((INSTANCE) == DAC1))

#define         IS_FL_DAC_TRIGGERMODE(__VALUE__)                        (((__VALUE__) == FL_DISABLE)||\
                                                                        ((__VALUE__) == FL_ENABLE))

#define         IS_FL_DAC_TRIGGE_SOURCE(__VALUE__)                     (((__VALUE__) == FL_DAC_TRGI_SOFTWARE)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_ATIM0)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_GPTIM1)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_GPTIM2)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_BSTIM16)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_LPTIM16)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_GPTIM0)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_ATIM1)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTIA)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTIB)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTIC)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTID)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTIE)||\
                                                                        ((__VALUE__) == FL_DAC_TRGI_EXTIF))

#define         IS_FL_DAC_SWITCHMODE(__VALUE__)                         (((__VALUE__) == FL_DISABLE)||\
                                                                        ((__VALUE__) == FL_ENABLE))

#define         IS_FL_DAC_DMAMODE(__VALUE__)                            (((__VALUE__) == FL_DISABLE)||\
                                                                        ((__VALUE__) == FL_ENABLE))

#define         IS_FL_DAC_BUFFERMODE(__VALUE__)                         (((__VALUE__) == FL_DISABLE)||\
                                                                        ((__VALUE__) == FL_ENABLE))

#define         IS_FL_DAC_SAMPLEMODE(__VALUE__)                         (((__VALUE__) == FL_DAC_SAMPLE_CONTINUE)||\
                                                                        ((__VALUE__) == FL_DAC_SAMPLE_SAMPLEHOLD))
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DAC_FL_EF_Init
  * @{
  */

/**
  * @brief  恢复对应的DAC入口地址寄存器为默认值
  *
  * @param  DACx  外设入口地址
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS DAC配置成功
  */
//FL_ErrorStatus  FL_DAC_DeInit(DAC_Type *DACx)
//{
//    FL_ErrorStatus status = FL_PASS;
//    /* 入口合法性检查 */
//    assert_param(IS_FL_DAC_INSTANCE(DACx));
//    /* 外设复位使能 */
//    FL_RMU_EnablePeripheralReset(RMU);
//    FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC);
//    FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC);
//    FL_RMU_DisablePeripheralReset(RMU);
//    return status;
//}
/**
  * @brief  初始化DACx指定的入口地址的外设寄存器
  * @param  DACx  外设入口地址
  * @param  DAC_InitStruct 向一FL_DAC_InitTypeDef结构体，它包含指定DAC外设的配置信息
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS DAC配置成功
  */
FL_ErrorStatus FL_DAC_Init(DAC_Type *DACx, FL_DAC_InitTypeDef  *DAC_InitStruct)
{
    FL_ErrorStatus status = FL_PASS;
    /* 入口合法性检查 */
    assert_param(IS_FL_DAC_INSTANCE(DACx));
    assert_param(IS_FL_DAC_TRIGGERMODE(DAC_InitStruct->triggerMode));
    assert_param(IS_FL_DAC_TRIGGE_SOURCE(DAC_InitStruct->triggerSource));
    assert_param(IS_FL_DAC_SWITCHMODE(DAC_InitStruct->switchMode));
    assert_param(IS_FL_DAC_BUFFERMODE(DAC_InitStruct->bufferMode));
    assert_param(IS_FL_DAC_SAMPLEMODE(DAC_InitStruct->timeSample));

    FL_RMU_EnablePeripheralReset(RMU);
    if(DACx == DAC0)
    {
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC0);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC0);
        FL_RMU_DisablePeripheralReset(RMU);
        FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_DAC0);
    }
    else if (DACx == DAC1)
    {
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC1);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_DAC1);
        FL_RMU_DisablePeripheralReset(RMU);
        FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_DAC1);
    }
   
    FL_DAC_Disable(DACx);
    FL_DAC_WriteData(DACx, 0);
    if(FL_DAC_IsEnabled(DACx) == 0U)
    {

        /* 触发模式使能配置 */
        if(DAC_InitStruct->triggerMode)
        {
            FL_DAC_SetTriggerSource(DACx, DAC_InitStruct->triggerSource);
            FL_DAC_EnableTriggerMode(DACx);
        }
        else
        {
            FL_DAC_DisableTriggerMode(DACx);
        }

        /* SWITCH使能配置 */
        if(DAC_InitStruct->switchMode)
        {
            FL_DAC_EnableOutputSwitch(DACx);
        }
        else
        {
            FL_DAC_DisableOutputSwitch(DACx);
        }

        /* Buffer使能配置 */
        if(DAC_InitStruct->bufferMode)
        {
            FL_DAC_EnableOutputBuffer(DACx);
        }
        else
        {
            FL_DAC_DisableOutputBuffer(DACx);
        }

        /* 采样保持使能配置 */
        if((DAC_InitStruct->sampleMode) == FL_DAC_SAMPLE_SAMPLEHOLD)
        {
            FL_DAC_EnableSampleHoldMode(DACx);
            FL_DAC_WriteHoldingTime(DACx, DAC_InitStruct->timeHold);
            FL_DAC_WriteSamplingTime(DACx, DAC_InitStruct->timeSample);
        }
        else
        {
            FL_DAC_DisableSampleHoldMode(DACx);
        }
    }
    else
    {
        status = FL_FAIL;
    }
    return status;
}

/**
  * @brief  设置 DAC_InitStruct 为默认配置
  * @param  DAC_InitStruct 指向需要将值设置为默认配置的结构体 @ref FL_DAC_InitTypeDef 结构体
  *
  * @retval None
  */
void FL_DAC_StructInit(FL_DAC_InitTypeDef *DAC_InitStruct)
{
    DAC_InitStruct->switchMode = FL_ENABLE;
    DAC_InitStruct->triggerMode = FL_DISABLE;
    DAC_InitStruct->triggerSource = FL_DAC_TRGI_SOFTWARE;
    DAC_InitStruct->bufferMode = FL_ENABLE;
    DAC_InitStruct->sampleMode = FL_DISABLE;
    DAC_InitStruct->timeHold = 0;
    DAC_InitStruct->timeSample = 0;
}

/**
  * @}
  */

#endif /* FL_DAC_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/********************** (C) COPYRIGHT Fudan Microelectronics **** END OF FILE ***********************/
