/**
  ****************************************************************************************************
  * @file    fm33ld5xx_fl_svd.c
  * @author  FMSH Application Team
  * @brief   Src file of SVD FL Module
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
  *******************************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"

/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */

/** @addtogroup SVD
  * @{
  */

#ifdef FL_SVD_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup SVD_FL_Private_Macros
  * @{
  */

#define         IS_SVD_INSTANCE(INSTANCE)                           (((INSTANCE) == SVD))

#define         IS_FL_SVD_REFERENCE_VOLTAGE(__VALUE__)              (((__VALUE__) == FL_SVD_REFERENCE_1P0V)||\
                                                                     ((__VALUE__) == FL_SVD_REFERENCE_0P95V)||\
                                                                     ((__VALUE__) == FL_SVD_REFERENCE_0P9V))

#define         IS_FL_SVD_WARNING_THRESHOLD_LEVEL(__VALUE__)        (((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP0)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP1)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP2)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP3)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP4)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP5)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP6)||\
                                                                     ((__VALUE__) == FL_SVD_WARNING_THRESHOLD_GROUP7))

#define         IS_FL_SVD_WORK_MODE(__VALUE__)                      (((__VALUE__) == FL_SVD_WORK_MODE_CONTINUOUS)||\
                                                                     ((__VALUE__) == FL_SVD_WORK_MODE_PERIODIC))

#define         IS_FL_SVD_ENABLE_PERIOD(__VALUE__)                  (((__VALUE__) == FL_SVD_ENABLE_PERIOD_62P5MS)||\
                                                                     ((__VALUE__) == FL_SVD_ENABLE_PERIOD_256MS)||\
                                                                     ((__VALUE__) == FL_SVD_ENABLE_PERIOD_1000MS)||\
                                                                     ((__VALUE__) == FL_SVD_ENABLE_PERIOD_4000MS))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SVD_FL_EF_Init
  * @{
  */

/**
  * @brief  复位SVD外设
  * @param  外设入口地址
  * @retval 返回错误状态，可能值：
  *         -FL_PASS 外设寄存器值恢复复位值
  *         -FL_FAIL 未成功执行
  */
FL_ErrorStatus FL_SVD_DeInit(SVD_Type *SVDx)
{
    /* 参数检查 */
    assert_param(IS_SVD_INSTANCE(SVDx));
    /* 使能复位 */
    FL_RMU_EnablePeripheralReset(RMU);
    /* 复位外设寄存器 */
    FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_SVD);
    FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_SVD);
    /* 关闭外设总线时钟 */
    FL_CMU_DisableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_SVD);
    /* 关闭复位 */
    FL_RMU_DisablePeripheralReset(RMU);
    return FL_PASS;
}

/**
  * @brief  根据需要功能配置SVD寄存器
  *
  * @param  SVDx  外设入口地址
  * @param  init @ref FL_SVD_InitTypeDef类型的结构体
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS SVD配置成功
  */
FL_ErrorStatus FL_SVD_Init(SVD_Type *SVDx, FL_SVD_InitTypeDef *init)
{
    /* 参数检查 */
    assert_param(IS_SVD_INSTANCE(SVDx));
    assert_param(IS_FL_SVD_REFERENCE_VOLTAGE(init->referenceVoltage));
    assert_param(IS_FL_SVD_WARNING_THRESHOLD_LEVEL(init->warningThreshold));
    assert_param(IS_FL_SVD_WORK_MODE(init->workMode));
    assert_param(IS_FL_SVD_ENABLE_PERIOD(init->enablePeriod));
    /* 开启SVD时钟 */
    if(SVDx == SVD)
    {
        FL_CMU_EnableGroup1BusClock(FL_CMU_GROUP1_BUSCLK_SVD);
    }
    else
    {
        return FL_FAIL;
    }
    /* 设置参考基准 */
    FL_SVD_EnableReference(SVDx, init->referenceVoltage);
    /* 设置报警阈值 */
    FL_SVD_SetWarningThreshold(SVDx, init->warningThreshold);
    /* 数字滤波 */
    if(init->digitalFilter == FL_ENABLE)
    {
        FL_SVD_EnableDigitalFilter(SVDx);
    }
    else
    {
        FL_SVD_DisableDigitalFilter(SVDx);
    }
    /* 工作模式 */
    FL_SVD_SetWorkMode(SVDx, init->workMode);
    if(init->workMode == FL_SVD_WORK_MODE_PERIODIC)
    {
        /* 间歇使能间隔 */
        FL_SVD_SetEnablePeriod(SVDx, init->enablePeriod);
    }
    /* SVS通道 */
    if(init->SVSChannel == FL_ENABLE)
    {
        FL_SVD_EnableSVSChannel(SVDx);
    }
    else
    {
        FL_SVD_DisableSVSChannel(SVDx);
    }
    return FL_PASS;
}

/**
  * @brief  设置 SVD_InitStruct 为默认配置
  * @param  init @ref FL_SVD_InitTypeDef类型的结构体
  *
  * @retval None
  */
void FL_SVD_StructInit(FL_SVD_InitTypeDef *init)
{
    init->referenceVoltage = FL_SVD_REFERENCE_1P0V;
    init->SVSChannel = FL_DISABLE;
    init->digitalFilter = FL_DISABLE;
    init->workMode = FL_SVD_WORK_MODE_CONTINUOUS;
    init->enablePeriod = FL_SVD_ENABLE_PERIOD_62P5MS;
    init->warningThreshold = FL_SVD_WARNING_THRESHOLD_GROUP0;
}

/**
  * @}
  */

#endif /* FL_SVD_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/********************** (C) COPYRIGHT Fudan Microelectronics **** END OF FILE ***********************/
