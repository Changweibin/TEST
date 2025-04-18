/**
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_iwdt.c
  * @author  FMSH Application Team
  * @brief   Src file of IWDT FL Module
  *******************************************************************************************************
  * @attention
  *
  * Copyright (c) [2021] [Fudan Microelectronics]
  * THIS SOFTWARE is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
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

/** @addtogroup IWDT
  * @{
  */

#ifdef FL_IWDT_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup IWDT_FL_Private_Macros
  * @{
  */

#define         IS_IWDT_INSTANCE(INTANCE)                    ((INTANCE) == IWDT)

#define         IS_FL_IWDT_WINDOWSVEL(__VALUE__)                ((__VALUE__) < 0xFFF)

#define         IS_FL_IWDT_OVERFLOWPERIOD(__VALUE__)            (((__VALUE__) == FL_IWDT_PERIOD_125MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_250MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_500MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_1000MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_2000MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_4000MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_8000MS)||\
                                                                 ((__VALUE__) == FL_IWDT_PERIOD_16000MS))


/**
  * @}
  */

/* Private consts ------------------------------------------------------------*/
/** @addtogroup IWDT_FL_Private_Consts
  * @{
  */



/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/



/* Exported functions --------------------------------------------------------*/
/** @addtogroup IWDT_FL_EF_Init
  * @{
  */

/**
  * @brief  复位IWDT外设
  *
  * @note   此函数只能用于配制前复位外设，因为IWDT开启后不可以关闭
  *
  * @param  IWDTx 外设入口地址
  *
  * @retval FL_ErrorStatus枚举值
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */

	

	
	
FL_ErrorStatus FL_IWDT_DeInit(IWDT_Type *IWDTx)
{
    assert_param(IS_IWDT_INSTANCE(IWDTx));
    return FL_PASS;
}
/**
  * @brief  根据 IWDT_InitStruct 初始化对应外设的寄存器值.
  *
  * @note   IWTD使能后将无法关闭，直到下一次芯片复位
  *
  * @param  IWDTx  外设入口地址
  * @param  IWDT_InitStruct 是 @ref FL_IWDT_InitTypeDef结构体，它包含指定IWDT外设的配置信息
  *
  * @retval ErrorStatus枚举值
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_IWDT_Init(IWDT_Type *IWDTx, FL_IWDT_InitTypeDef *IWDT_InitStruct)
{
    uint32_t i = 0;
    FL_ErrorStatus status = FL_FAIL;
    uint32_t iwdtCounter = 0;
    if(IWDT_InitStruct != NULL)
    {
        /* 入口参数检查 */
        assert_param(IS_IWDT_INSTANCE(IWDTx));
        assert_param(IS_FL_IWDT_WINDOWSVEL(IWDT_InitStruct->iwdtWindows));
        assert_param(IS_FL_IWDT_OVERFLOWPERIOD(IWDT_InitStruct->overflowPeriod));
        /* 开启总线时钟 */
        FL_CMU_EnableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_IWDT);
        /* 清看门狗 */
       FL_IWDT_ReloadCounter(IWDTx);
        /* 配置独立看门狗溢出周期 */
        FL_IWDT_SetPeriod(IWDTx, IWDT_InitStruct->overflowPeriod);

        iwdtCounter = FL_IWDT_ReadCounter(IWDTx);
        /* 窗口功能延时 */
        if(IWDT_InitStruct->iwdtWindows != 0)
        {
            if(iwdtCounter == 0)
            {
                while((FL_IWDT_ReadCounter(IWDTx) != 1) && (i++ < 500))
                {

                }
            }
            else
            {
                while((FL_IWDT_ReadCounter(IWDTx) != 0) && (i++ < 500))
                {

                }
            }
        }
        /* 配置独立看门狗清狗窗口*/
        FL_IWDT_WriteWindow(IWDTx, IWDT_InitStruct->iwdtWindows);
        status = FL_PASS;
    }
    return status;
}
/**
  * @brief  设置 IWDT_InitStruct 为默认配置
  *
  * @param  IWDT_InitStruct 指向需要将值设置为默认配置的结构体 @ref LL_IWDT_InitTypeDef 结构体
  *
  * @retval None
  */
void FL_IWDT_StructInit(FL_IWDT_InitTypeDef *IWDT_InitStruct)
{
    if(IWDT_InitStruct != NULL)
    {
        /* 默认不使用窗口 */
        IWDT_InitStruct->iwdtWindows    = 0;
        /*最长溢出时间*/
        IWDT_InitStruct->overflowPeriod = FL_IWDT_PERIOD_500MS;
    }
}

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup IWDT_FL_Private_Functions IWDT Private Functions
  * @{
  */



/**
  * @}
  */

#endif /* FL_IWDT_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
