/**
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_clm.c
  * @author  FMSH Application Team
  * @brief   Src file of CLM FL Module
  *******************************************************************************************************
  * @attention
  *
  * Copyright (c) [2021] [Fudan Microelectronics]
  * THIS SOFTWARE is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  *          http://license.coscl.org.cn/MulanPSL2
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  * See the Mulan PSL v2 for more details.
  *
  *******************************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"

/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */
#define         IS_FL_CLM_INSTANCE(INSTANCE)                (((INSTANCE) == CLM0) || \
                                                             ((INSTANCE) == CLM1) || \
                                                             ((INSTANCE) == CLM2))

#define         IS_FL_CLM_MODE_INSTANCE(__VALUE__)          (((__VALUE__) == FL_CLM_WORKMODE_INTERRUPT) || \
                                                             ((__VALUE__) == FL_CLM_WORKMODE_RESET))

 /** @defgroup CLM CLM
  * @brief CLM FL driver
  * @{
  */

#ifdef FL_CLM_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CLM_FL_Private_Macros
  * @{
  */
/**
  * @brief  根据CLM_InitStruct初始化对应外设入口地址的寄存器值
  * @param  CLMx 外设入口地址
  * @param  CLM_InitStruct 指向 @ref FL_CLM_InitTypeDef 结构体指针
  * @retval 错误状态，可能值：
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_CLM_Init(CLM_Type *CLMx, FL_CLM_InitTypeDef  *CLM_InitStruct)
{
    FL_ErrorStatus status = FL_PASS;
    /* 入口参数检查 */
    assert_param(IS_FL_CLM_INSTANCE(CLMx));  
    assert_param(IS_FL_CLM_MODE_INSTANCE(CLM_InitStruct->Mode));
    if(CLMx == CLM0)
    {   /* 使能CLM0总线时钟 */
        FL_CMU_EnableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM0);
        /* 配置CLMx监控时钟源 */
        FL_CLM_SetMonitorClock(CLM0,FL_CLM_MONCLK_PLL);
    }
    else if(CLMx == CLM1)
    {   /* 使能CLM1总线时钟 */
        FL_CMU_EnableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM1);
        /* 配置CLMx监控时钟源 */
        FL_CLM_SetMonitorClock(CLM0,FL_CLM_MONCLK_XTHF);
    }
    else
    {   /* 使能CLM2总线时钟 */
        FL_CMU_EnableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM2);
        /* 配置CLMx监控时钟源 */
        FL_CLM_SetMonitorClock(CLM0,FL_CLM_MONCLK_RCHF);
    }
    /* 配置CLMx工作模式 */
    FL_CLM_SetWorkingMode(CLMx,CLM_InitStruct->Mode);
    /* 使能CLMx超时复位输出 */
    if(CLM_InitStruct->TimeoutReset == FL_ENABLE)
    {
        FL_CLM_EnableTimeOutReset(CLMx);
        FL_CLM_WriteTimeoutOverflowLimitValue (CLMx, CLM_InitStruct->TimeoutOverflowLimitValue);
        FL_CLM_WriteTimeoutMonitorClockPrescaler(CLMx, CLM_InitStruct->TimeoutMonitorClockPrescaler);
    }
    else
    {
        FL_CLM_DisableTimeOutReset(CLMx);
    }
    /* 配置CLMx参考时钟周期 */
    FL_CLM_WriteReferenceClockCycle(CLMx,CLM_InitStruct->ReferenceclockCycle);
    /* 配置CLMx时钟监控比较值高阈值 */
    FL_CLM_WriteCompareHighThreshold(CLMx,CLM_InitStruct->CompareHighThreshold);
    /* 配置CLMx时钟监控比较值低阈值 */
    FL_CLM_WriteCompareLowThreshold(CLMx,CLM_InitStruct->CompareLowThreshold);
    return status;
}


/**
  * @brief  CLM 去初始化
  * @param  CLMx 外设入口地址
  * @retval 错误状态，可能值：
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_CLM_DeInit(CLM_Type *CLMx)
{
    FL_ErrorStatus status = FL_PASS;
    /* 参数入口检查 */
    assert_param(IS_FL_CLM_INSTANCE(CLMx));
//    if(CLMx == CLM0)
//    {
//        /* CLM0 复位设置 */
//        FL_RMU_EnablePeripheralReset(RMU);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM0);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM0);
//        FL_RMU_DisablePeripheralReset(RMU);
//        FL_CMU_DisableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM0);
//    }
//    else if(CLMx == CLM1)
//    {
//        /* CLM1 复位设置 */
//        FL_RMU_EnablePeripheralReset(RMU);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM1);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM1);
//        FL_RMU_DisablePeripheralReset(RMU);
//        FL_CMU_DisableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM1);
//    }
//    else
//    {
//        /* CLM2 复位设置 */
//        FL_RMU_EnablePeripheralReset(RMU);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM2);
//        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_CLM2);
//        FL_RMU_DisablePeripheralReset(RMU);
//        FL_CMU_DisableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_CLM2);
//    }
    return status;
}

/**
  * @brief  将 @ref FL_CLM_StructInit 结构体初始化为默认配置
  * @param  initStruct 指向 @ref FL_CLM_StructInit 结构体的指针
  *
  * @retval None
  */
void FL_CLM_StructInit(FL_CLM_InitTypeDef *initStruct)
{
    initStruct->Mode                         = FL_CLM_WORKMODE_INTERRUPT;    /* 配置CLM中断模式 */
    initStruct->ReferenceclockCycle          = 300-1;                        /* 配置参考时钟周期 */
    initStruct->CompareHighThreshold         = 300*488*1.2;                  /* 配置监控时钟高阈值 */
    initStruct->CompareLowThreshold          = 300*488*0.8;                  /* 配置监控时钟低阈值 */
    initStruct->TimeoutReset                 = FL_DISABLE;                   /* 禁能超时复位 */
    initStruct->TimeoutOverflowLimitValue    = 32;                           /* 超时溢出上限 */
    initStruct->TimeoutMonitorClockPrescaler = 2000-1;                       /* 超时检测预分频 */
}


#endif

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
