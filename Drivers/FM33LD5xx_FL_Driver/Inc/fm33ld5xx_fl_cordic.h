/**
  *******************************************************************************************************
  * @file    fm33ld0xx_fl_cordic.h
  * @author  FMSH Application Team
  * @brief   Head file of CORDIC FL Module
  *******************************************************************************************************
  * @attention
  *
  * Copyright (c) [2019] [Fudan Microelectronics]
  * THIS SOFTWARE is licensed under the Mulan PSL v1.
  * can use this software according to the terms and conditions of the Mulan PSL v1.
  * You may obtain a copy of Mulan PSL v1 at:
  * http://license.coscl.org.cn/MulanPSL
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR
  * PURPOSE.
  * See the Mulan PSL v1 for more details.
  *
  *******************************************************************************************************
  */


/* Define to prevent recursive inclusion---------------------------------------------------------------*/
#ifndef __FM33LD0XX_FL_CORDIC_H
#define __FM33LD0XX_FL_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes -------------------------------------------------------------------------------------------*/
#include "fm33ld5xx_fl_def.h"
/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */
/* Exported types -------------------------------------------------------------------------------------*/
/** @defgroup CORDIC_FL_ES_INIT CORDIC Exported Init structures
  * @{
  */

typedef struct
{

    /** 输出单位 */
    uint32_t outputunit;
    /** 输入单位 */
    uint32_t inputunit;       
    /** 浮点使能 */
    uint32_t floatmode;
    /** 通道采样时间配置 */
    uint32_t operationmode;  
    /** CORDIC使能 */
    uint32_t cordic_en;

    
} FL_CORDIC_InitTypeDef;

/**
  * @}
  */
/* Exported constants ---------------------------------------------------------------------------------*/
/** @defgroup CORDIC_FL_Exported_Constants CORDIC Exported Constants
  * @{
  */

#define    CORDIC_INA_INA_Pos                                     (0U)
#define    CORDIC_INA_INA_Msk                                     (0xffffffffU << CORDIC_INA_INA_Pos)
#define    CORDIC_INA_INA                                         CORDIC_INA_INA_Msk

#define    CORDIC_INB_INB_Pos                                     (0U)
#define    CORDIC_INB_INB_Msk                                     (0xffffffffU << CORDIC_INB_INB_Pos)
#define    CORDIC_INB_INB                                         CORDIC_INB_INB_Msk

#define    CORDIC_CTRL_OUT_UNIT_Pos                               (14U)
#define    CORDIC_CTRL_OUT_UNIT_Msk                               (0x1U << CORDIC_CTRL_OUT_UNIT_Pos)
#define    CORDIC_CTRL_OUT_UNIT                                   CORDIC_CTRL_OUT_UNIT_Msk

#define    CORDIC_CTRL_IN_UNIT_Pos                                (13U)
#define    CORDIC_CTRL_IN_UNIT_Msk                                (0x1U << CORDIC_CTRL_IN_UNIT_Pos)
#define    CORDIC_CTRL_IN_UNIT                                    CORDIC_CTRL_IN_UNIT_Msk

#define    CORDIC_CTRL_FLOAT_EN_Pos                               (12U)
#define    CORDIC_CTRL_FLOAT_EN_Msk                               (0x1U << CORDIC_CTRL_FLOAT_EN_Pos)
#define    CORDIC_CTRL_FLOAT_EN                                   CORDIC_CTRL_FLOAT_EN_Msk

#define    CORDIC_CTRL_CORDIC_OP_Pos                              (8U)
#define    CORDIC_CTRL_CORDIC_OP_Msk                              (0xfU << CORDIC_CTRL_CORDIC_OP_Pos)
#define    CORDIC_CTRL_CORDIC_OP                                  CORDIC_CTRL_CORDIC_OP_Msk

#define    CORDIC_CTRL_CORDIC_EN_Pos                              (0U)
#define    CORDIC_CTRL_CORDIC_EN_Msk                              (0x1U << CORDIC_CTRL_CORDIC_EN_Pos)
#define    CORDIC_CTRL_CORDIC_EN                                  CORDIC_CTRL_CORDIC_EN_Msk

#define    CORDIC_OUT_CORDIC_OUT_Pos                              (0U)
#define    CORDIC_OUT_CORDIC_OUT_Msk                              (0xffffffffU << CORDIC_OUT_CORDIC_OUT_Pos)
#define    CORDIC_OUT_CORDIC_OUT                                  CORDIC_OUT_CORDIC_OUT_Msk

#define    CORDIC_ISR_CORDIC_IF_Pos                               (0U)
#define    CORDIC_ISR_CORDIC_IF_Msk                               (0x1U << CORDIC_ISR_CORDIC_IF_Pos)
#define    CORDIC_ISR_CORDIC_IF                                   CORDIC_ISR_CORDIC_IF_Msk

#define    CORDIC_IER_CORDIC_IE_Pos                               (0U)
#define    CORDIC_IER_CORDIC_IE_Msk                               (0x1U << CORDIC_IER_CORDIC_IE_Pos)
#define    CORDIC_IER_CORDIC_IE                                   CORDIC_IER_CORDIC_IE_Msk






#define    FL_CORDIC_OUTPUT_RADIAN_TO_PI                   (0x0U << CORDIC_CTRL_OUT_UNIT_Pos)
#define    FL_CORDIC_OUTPUT_RADIAN                         (0x1U << CORDIC_CTRL_OUT_UNIT_Pos)


#define    FL_CORDIC_INPUT_RADIAN_TO_PI                   (0x0U << CORDIC_CTRL_IN_UNIT_Pos)
#define    FL_CORDIC_INPUT_RADIAN                         (0x1U << CORDIC_CTRL_IN_UNIT_Pos)


#define    FL_CORDIC_MODE_SIN                                     (0x0U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_COS                                     (0x1U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_ATAN                                    (0x2U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_MOD                                     (0x3U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_SINH                                    (0x4U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_COSH                                    (0x5U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_EXP                                     (0x6U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_ATANH                                   (0x7U << CORDIC_CTRL_CORDIC_OP_Pos)
#define    FL_CORDIC_MODE_SQRT                                    (0x8U << CORDIC_CTRL_CORDIC_OP_Pos)


/**
  * @}
  */
/* Exported functions ---------------------------------------------------------------------------------*/
/** @defgroup CORDIC_FL_Exported_Functions CORDIC Exported Functions
  * @{
  */

/**
  * @brief    写入CordicA寄存器
  * @rmtoll   INA    INA    FL_CORDIC_WriteCordicValueA
  * @param    CORDICx CORDIC instance
  * @param    number 
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_WriteCordicValueA(CORDIC_Type* CORDICx, int32_t number)
{
    MODIFY_REG(CORDICx->INA, (0xffffffffU << 0U), (number << 0U));
}

/**
  * @brief    读取CordicA寄存器
  * @rmtoll   INA    INA    FL_CORDIC_ReadCordicValueA
  * @param    CORDICx CORDIC instance
  * @retval   
  */
__STATIC_INLINE int32_t FL_CORDIC_ReadCordicValueA(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->INA, (0xffffffffU << 0U)) >> 0U);
}

/**
  * @brief    写入CordicB寄存器
  * @rmtoll   INB    INB    FL_CORDIC_WriteCordicValueB
  * @param    CORDICx CORDIC instance
  * @param    number 
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_WriteCordicValueB(CORDIC_Type* CORDICx, int32_t number)
{
    MODIFY_REG(CORDICx->INB, (0xffffffffU << 0U), (number << 0U));
}

/**
  * @brief    读取CordicB寄存器
  * @rmtoll   INB    INB    FL_CORDIC_ReadCordicValueB
  * @param    CORDICx CORDIC instance
  * @retval   
  */
__STATIC_INLINE int32_t FL_CORDIC_ReadCordicValueB(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->INB, (0xffffffffU << 0U)) >> 0U);
}

/**
  * @brief    设置输出单位
  * @rmtoll   CTRL    OUT_UNIT    FL_CORDIC_SetOutputUnit
  * @param    CORDICx CORDIC instance
  * @param    unit This parameter can be one of the following values:
  *           @arg @ref FL_CORDIC_CORDIC_OUTPUT_RADIAN_TO_PI
  *           @arg @ref FL_CORDIC_CORDIC_OUTPUT_RADIAN
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_SetOutputUnit(CORDIC_Type* CORDICx, uint32_t unit)
{
    MODIFY_REG(CORDICx->CR, CORDIC_CTRL_OUT_UNIT_Msk, unit);
}

/**
  * @brief    读取输出单位
  * @rmtoll   CTRL    OUT_UNIT    FL_CORDIC_GetOutputUnit
  * @param    CORDICx CORDIC instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_CORDIC_CORDIC_OUTPUT_RADIAN_TO_PI
  *           @arg @ref FL_CORDIC_CORDIC_OUTPUT_RADIAN
  */
__STATIC_INLINE uint32_t FL_CORDIC_GetOutputUnit(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->CR, CORDIC_CTRL_OUT_UNIT_Msk));
}

/**
  * @brief    设置输出单位
  * @rmtoll   CTRL    OUT_UNIT    FL_CORDIC_SetInputUnit
  * @param    CORDICx CORDIC instance
  * @param    unit This parameter can be one of the following values:
  *           @arg @ref FL_CORDIC_CORDIC_INPUT_RADIAN_TO_PI
  *           @arg @ref FL_CORDIC_CORDIC_INPUT_RADIAN
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_SetInputUnit(CORDIC_Type* CORDICx, uint32_t unit)
{
    MODIFY_REG(CORDICx->CR, CORDIC_CTRL_IN_UNIT_Msk, unit);
}

/**
  * @brief    读取输出单位
  * @rmtoll   CTRL    OUT_UNIT    FL_CORDIC_GetInputUnit
  * @param    CORDICx CORDIC instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_CORDIC_CORDIC_INPUT_RADIAN_TO_PI
  *           @arg @ref FL_CORDIC_CORDIC_INPUT_RADIAN
  */
__STATIC_INLINE uint32_t FL_CORDIC_GetInputUnit(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->CR, CORDIC_CTRL_IN_UNIT_Msk));
}


/**
  * @brief    使能CORDIC浮点控制
  * @rmtoll   CTRL    FLOAT_EN    FL_CORDIC_Enable_Float
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_Enable_Float(CORDIC_Type* CORDICx)
{
    SET_BIT(CORDICx->CR, CORDIC_CTRL_FLOAT_EN_Msk);
}

/**
  * @brief    失能CORDIC浮点控制
  * @rmtoll   CTRL    FLOAT_EN    FL_CORDIC_Disable_Float
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_Disable_Float(CORDIC_Type* CORDICx)
{
    CLEAR_BIT(CORDICx->CR, CORDIC_CTRL_FLOAT_EN_Msk);
}

/**
  * @brief    获取CORDIC浮点控制使能状态
  * @rmtoll   CTRL    FLOAT_EN    FL_CORDIC_IsEnabled_Float
  * @param    CORDICx CORDIC instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_CORDIC_IsEnabled_Float(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->CR, CORDIC_CTRL_FLOAT_EN_Msk) == CORDIC_CTRL_FLOAT_EN_Msk);
}


/**
  * @brief    Write Cordic Operation Code
  * @rmtoll   CR    CORDIC_OP    FL_CORDIC_SetCordicOperationMode
  * @param    CORDICx CORDIC instance
  * @param    mode This parameter can be one of the following values:
  *           @arg @ref FL_CORDIC_MODE_SIN
  *           @arg @ref FL_CORDIC_MODE_COS
  *           @arg @ref FL_CORDIC_MODE_ATAN
  *           @arg @ref FL_CORDIC_MODE_MOD
  *           @arg @ref FL_CORDIC_MODE_SINH
  *           @arg @ref FL_CORDIC_MODE_COSH
  *           @arg @ref FL_CORDIC_MODE_EXP
  *           @arg @ref FL_CORDIC_MODE_ATANH
  *           @arg @ref FL_CORDIC_MODE_SQRT
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_SetCordicOperationMode(CORDIC_Type* CORDICx, uint32_t mode)
{
    MODIFY_REG(CORDICx->CR, CORDIC_CTRL_CORDIC_OP_Msk, mode);
}

/**
  * @brief    Read Cordic Operation Coder
  * @rmtoll   CR    CORDIC_OP    FL_CORDIC_GetCordicOperationMode
  * @param    CORDICx CORDIC instance
  * @retval   Returned value can be one of the following values:
  *           @arg @ref FL_CORDIC_MODE_SIN
  *           @arg @ref FL_CORDIC_MODE_COS
  *           @arg @ref FL_CORDIC_MODE_ATAN
  *           @arg @ref FL_CORDIC_MODE_MOD
  *           @arg @ref FL_CORDIC_MODE_SINH
  *           @arg @ref FL_CORDIC_MODE_COSH
  *           @arg @ref FL_CORDIC_MODE_EXP
  *           @arg @ref FL_CORDIC_MODE_ATANH
  *           @arg @ref FL_CORDIC_MODE_SQRT
  */
__STATIC_INLINE int32_t FL_CORDIC_GetCordicOperationMode(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->CR, CORDIC_CTRL_CORDIC_OP_Msk));
}

/**
  * @brief    使能CORDIC
  * @rmtoll   CTRL    CORDIC_EN    FL_CORDIC_Enable
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_Enable(CORDIC_Type* CORDICx)
{
    SET_BIT(CORDICx->CR, CORDIC_CTRL_CORDIC_EN_Msk);
}

/**
  * @brief    失能CORDIC
  * @rmtoll   CTRL    CORDIC_EN    FL_CORDIC_Disable
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_Disable(CORDIC_Type* CORDICx)
{
    CLEAR_BIT(CORDICx->CR, CORDIC_CTRL_CORDIC_EN_Msk);
}

/**
  * @brief    获取CORDIC使能状态
  * @rmtoll   CTRL    CORDIC_EN    FL_CORDIC_IsEnabled
  * @param    CORDICx CORDIC instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_CORDIC_IsEnabled(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->CR, CORDIC_CTRL_CORDIC_EN_Msk) == CORDIC_CTRL_CORDIC_EN_Msk);
}

/**
  * @brief    读取Cordic结果
  * @rmtoll   OUT    CORDIC_OUT    FL_CORDIC_ReadCordicResult
  * @param    CORDICx CORDIC instance
  * @retval   
  */
__STATIC_INLINE int32_t FL_CORDIC_ReadCordicResult(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->OUT, (0xffffffffU << 0U)) >> 0U);
}

/**
  * @brief    获取CORDIC计算完成标志

  * @rmtoll   ISR    CORDIC_IF    FL_CORDIC_IsActiveFlag_CalcComplete
  * @param    CORDICx CORDIC instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_CORDIC_IsActiveFlag_CalcComplete(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->ISR, CORDIC_ISR_CORDIC_IF_Msk) == (CORDIC_ISR_CORDIC_IF_Msk));
}

/**
  * @brief    清除CORDIC计算完成标志


  * @rmtoll   ISR    CORDIC_IF    FL_CORDIC_ClearFlag_CalcComplete
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_ClearFlag_CalcComplete(CORDIC_Type* CORDICx)
{
    WRITE_REG(CORDICx->ISR, CORDIC_ISR_CORDIC_IF_Msk);
}

/**
  * @brief    使能CORDIC计算完成中断
  * @rmtoll   IER    CORDIC_IE    FL_CORDIC_EnableIT_CalcComplete
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_EnableIT_CalcComplete(CORDIC_Type* CORDICx)
{
    SET_BIT(CORDICx->IER, CORDIC_IER_CORDIC_IE_Msk);
}

/**
  * @brief    失能CORDIC计算完成中断
  * @rmtoll   IER    CORDIC_IE    FL_CORDIC_DisableIT_CalcComplete
  * @param    CORDICx CORDIC instance
  * @retval   None
  */
__STATIC_INLINE void FL_CORDIC_DisableIT_CalcComplete(CORDIC_Type* CORDICx)
{
    CLEAR_BIT(CORDICx->IER, CORDIC_IER_CORDIC_IE_Msk);
}

/**
  * @brief    获取CORDIC计算完成中断使能状态
  * @rmtoll   IER    CORDIC_IE    FL_CORDIC_IsEnabledIT_CalcComplete
  * @param    CORDICx CORDIC instance
  * @retval   State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t FL_CORDIC_IsEnabledIT_CalcComplete(CORDIC_Type* CORDICx)
{
    return (uint32_t)(READ_BIT(CORDICx->IER, CORDIC_IER_CORDIC_IE_Msk) == CORDIC_IER_CORDIC_IE_Msk);
}

/**
  * @}
  */

/** @defgroup CORDIC_FL_EF_Init Initialization and de-initialization functions
  * @{
  */


/**
  * @}
  */
FL_ErrorStatus FL_CORDIC_Init(CORDIC_Type *CORDICx,FL_CORDIC_InitTypeDef *CORDIC_InitStruct);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __FM33LD0XX_FL_CORDIC_H*/

/*************************Py_Code_Generator Version: 0.1-0.14-0.1 @ 2025-02-28*************************/
/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
