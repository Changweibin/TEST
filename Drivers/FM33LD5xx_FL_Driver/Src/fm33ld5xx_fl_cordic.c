/**
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_cordic.c
  * @author  FMSH Application Team
  * @brief   Src file of CORDIC FL Module
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

/** @addtogroup CORDIC
  * @{
  */

#ifdef FL_CORDIC_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup DIVAS_FL_Private_Macros
  * @{
  */
#define         IS_CORDIC_ALL_INSTANCE(INTENCE)              ((INTENCE) == CORDIC)



/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CORDIC_FL_EF_Init
  * @{
  */

/**
  * @brief  复位对应HDIV控制寄存器.
  *
  * @param  DIVASx 外设入口地址
  *
  * @retval FL_ErrorStatus枚举值
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_DIVAS_DeInit(CORDIC_Type *CORDICx)
{
    /* 入口参数检查 */
    assert_param(IS_CORDIC_ALL_INSTANCE(CORDICx));
    /* 外设复位使能 */
    FL_RMU_EnablePeripheralReset(RMU);
    /* 恢复寄存器值为默认值 */
    FL_RMU_EnableResetAPBPeripheral(RMU,FL_RMU_RSTAPB_CORDIC);
    FL_RMU_DisableResetAPBPeripheral(RMU,FL_RMU_RSTAPB_CORDIC);
    /* 关闭总线时钟 */
    FL_CMU_DisableGroup4BusClock(FL_CMU_GROUP4_BUSCLK_CORDIC);
    /* 锁定外设复位功能 */
    FL_RMU_DisablePeripheralReset(RMU);
    return FL_PASS;
}

/**
  * @brief  根据 初始化对应外设CORDIC.
  *
  * @param  CORDICx 外设入口地址
  *
  * @retval FL_ErrorStatus枚举值
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_CORDIC_Init(CORDIC_Type *CORDICx,FL_CORDIC_InitTypeDef *CORDIC_InitStruct)
{
    /* 入口参数检查 */
    assert_param(IS_CORDIC_ALL_INSTANCE(CORDICx));
    /* 使能时钟总线 */
    FL_CMU_EnableGroup4BusClock(FL_CMU_GROUP4_BUSCLK_CORDIC);
    /* 设置输出单位 */
    FL_CORDIC_SetOutputUnit(CORDIC,CORDIC_InitStruct->outputunit );
    /* 设置输入单位 */
    FL_CORDIC_SetInputUnit(CORDIC,CORDIC_InitStruct->inputunit );
    /* 使能浮点数据 */ 
    if(CORDIC_InitStruct->floatmode == FL_ENABLE)
    {
        FL_CORDIC_Enable_Float(CORDIC);
    }
    else if(CORDIC_InitStruct->floatmode == FL_DISABLE)
    {
        FL_CORDIC_Disable_Float(CORDIC);
    }
    /* 设置操作码 */
    FL_CORDIC_SetCordicOperationMode(CORDIC,CORDIC_InitStruct->operationmode );
      /* 使能浮点数据 */ 
    if(CORDIC_InitStruct->cordic_en == FL_ENABLE)
    {
        FL_CORDIC_Enable(CORDIC);
    }
    else if(CORDIC_InitStruct->cordic_en == FL_DISABLE)
    {
        FL_CORDIC_Disable(CORDIC);
    }
    return FL_PASS;
}

#endif

/********************** (C) COPYRIGHT Fudan Microelectronics **** END OF FILE ***********************/

