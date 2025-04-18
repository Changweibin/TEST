/**
  ****************************************************************************************************
  * @file    fm33ld5xx_fl_uart.c
  * @author  FMSH Application Team
  * @brief   Src file of UART FL Module
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

/** @addtogroup UART
  * @{
  */

#ifdef FL_UART_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup UART_FL_Private_Macros
  * @{
  */

#define         IS_UART_INSTANCE(INSTANCE)              (((INSTANCE) == UART0)||\
                                                         ((INSTANCE) == UART1)||\
                                                         ((INSTANCE) == UART2)||\
                                                         ((INSTANCE) == UART3)||\
                                                         ((INSTANCE) == UART4)||\
                                                         ((INSTANCE) == UART5))

#define         IS_FL_UART_CLKSRC(__VALUE__)            (((__VALUE__) == FL_CMU_UART0_CLK_SOURCE_APBCLK)||\
                                                         ((__VALUE__) == FL_CMU_UART0_CLK_SOURCE_RCHF)||\
                                                         ((__VALUE__) == FL_CMU_UART0_CLK_SOURCE_XTHF)||\
                                                         ((__VALUE__) == FL_CMU_UART1_CLK_SOURCE_APBCLK)||\
                                                         ((__VALUE__) == FL_CMU_UART1_CLK_SOURCE_RCHF)||\
                                                         ((__VALUE__) == FL_CMU_UART1_CLK_SOURCE_XTHF)||\
                                                         ((__VALUE__) == FL_CMU_UART2_CLK_SOURCE_APBCLK)||\
                                                         ((__VALUE__) == FL_CMU_UART2_CLK_SOURCE_RCHF)||\
                                                         ((__VALUE__) == FL_CMU_UART2_CLK_SOURCE_XTHF)||\
                                                         ((__VALUE__) == FL_CMU_UART3_CLK_SOURCE_APBCLK)||\
                                                         ((__VALUE__) == FL_CMU_UART3_CLK_SOURCE_RCHF)||\
                                                         ((__VALUE__) == FL_CMU_UART3_CLK_SOURCE_XTHF)||\
                                                         ((__VALUE__) == FL_CMU_UART4_CLK_SOURCE_APBCLK)||\
                                                         ((__VALUE__) == FL_CMU_UART4_CLK_SOURCE_RCHF)||\
                                                         ((__VALUE__) == FL_CMU_UART4_CLK_SOURCE_XTHF))

#define         IS_FL_UART_DATAWIDTH(__VALUE__)           (((__VALUE__) == FL_UART_DATA_WIDTH_6B)||\
                                                           ((__VALUE__) == FL_UART_DATA_WIDTH_7B)||\
                                                           ((__VALUE__) == FL_UART_DATA_WIDTH_8B)||\
                                                           ((__VALUE__) == FL_UART_DATA_WIDTH_9B))

#define         IS_FL_UART_STOPBITS(__VALUE__)            (((__VALUE__) == FL_UART_STOP_BIT_WIDTH_1B)||\
                                                           ((__VALUE__) == FL_UART_STOP_BIT_WIDTH_2B))

#define         IS_FL_UART_PARITY(__VALUE__)              (((__VALUE__) == FL_UART_PARITY_NONE)||\
                                                           ((__VALUE__) == FL_UART_PARITY_EVEN)||\
                                                           ((__VALUE__) == FL_UART_PARITY_ODD))

#define         IS_FL_UART_DIRECTION(__VALUE__)           (((__VALUE__) == FL_UART_DIRECTION_NONE)||\
                                                           ((__VALUE__) == FL_UART_DIRECTION_RX)||\
                                                           ((__VALUE__) == FL_UART_DIRECTION_TX)||\
                                                           ((__VALUE__) == FL_UART_DIRECTION_TX_RX))

#define         IS_FL_UART_INFRA_MODULATION(__VALUE__)    (((__VALUE__) == FL_DISABLE)||\
                                                           ((__VALUE__) == FL_ENABLE))

#define         IS_FL_UART_INFRARED_POLARITY(__VALUE__)        (((__VALUE__) == FL_UART_INFRARED_POLARITY_NORMAL)||\
                                                                ((__VALUE__) == FL_UART_INFRARED_POLARITY_INVERT))

#define         IS_FL_UART_INFRARED_MODULATION_DUTY(__VALUE__) (((__VALUE__) <= 100))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_FL_EF_Init
  * @{
  */

/**
  * @brief  复位UART 外设寄存器值为复位值
  * @param  外设入口地址
  * @retval 返回错误状态，可能值:
  *         -FL_PASS 外设寄存器值恢复复位值
  *         -FL_FAIL 复位未成功
  */
FL_ErrorStatus FL_UART_DeInit(UART_Type *UARTx)
{
    FL_ErrorStatus status = FL_PASS;
    /* 参数入口合法性 */
    assert_param(IS_UART_INSTANCE(UARTx));
    /*模式选择--UART*/
    FL_UART_SetMode(UARTx, FL_UART_MODESEL_UART);
    /* 外设复位使能 */
    FL_RMU_EnablePeripheralReset(RMU);
    if(UARTx == UART0)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART0);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART0);
        /* 外设总线时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART0);
        /* 外设工作时钟关闭 */
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART0);
    }
    else if(UARTx == UART1)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART1);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART1);
        /* 外设总线时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART1);
        /* 外设工作时钟关闭 */
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART1);
    }
    else if(UARTx == UART2)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART2);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART2);
        /* 外设总线时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART2);
        /* 外设工作时钟关闭 */
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART2);
    }
    else if(UARTx == UART3)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART3);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART3);
        /* 外设总线时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART3);
        /* 外设工作时钟关闭 */
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART3);
    }
    else if(UARTx == UART4)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART4);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART4);
        /* 总线、工作时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART4);
        /* 外设工作时钟关闭 */
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART4);
    }
    else if(UARTx == UART5)
    {
        /*复位UART*/
        FL_RMU_EnableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART5);
        FL_RMU_DisableResetAPBPeripheral(RMU, FL_RMU_RSTAPB_UART5);
        /* 总线（工作）时钟关闭 */
        FL_CMU_DisableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART5);
    }
    else
    {
        status = FL_FAIL;
    }
    /* 锁定外设复位功能 */
    FL_RMU_DisablePeripheralReset(RMU);
    return (status);
}

/**
  * @brief  根据需要配置UART
  *
  * @param  UARTx  外设入口地址
  * @param  UART_InitStruct指向一个FL_UART_InitTypeDef类型的结构体,它包含外设UART的配置信息
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS UART配置成功
  */
FL_ErrorStatus FL_UART_Init(UART_Type *UARTx, FL_UART_InitTypeDef *initStruct)
{
    FL_ErrorStatus status = FL_FAIL;
    uint32_t Fclk = 0, BaudRate = 0;
    /* 参数合法性检查 */
    assert_param(IS_UART_INSTANCE(UARTx));
    assert_param(IS_FL_UART_CLKSRC(initStruct->clockSrc));
    assert_param(IS_FL_UART_DATAWIDTH(initStruct->dataWidth));
    assert_param(IS_FL_UART_PARITY(initStruct->parity));
    assert_param(IS_FL_UART_STOPBITS(initStruct->stopBits));
    assert_param(IS_FL_UART_DIRECTION(initStruct->transferDirection));
    if(UARTx == UART0)
    {
        /*时钟源选择*/
        FL_CMU_SetUART0ClockSource(initStruct->clockSrc);
        /* 根据不同的时钟源计算baudrate 寄存器值,并配置 */
        switch(initStruct->clockSrc)
        {
            case FL_CMU_UART0_CLK_SOURCE_APBCLK:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
            case FL_CMU_UART0_CLK_SOURCE_RCHF:
                Fclk = FL_CMU_GetRCHFClockFreq();
                break;
            case FL_CMU_UART0_CLK_SOURCE_XTHF:
                Fclk = XTHFClock;
                break;
            default:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
        }
        BaudRate = Fclk / initStruct->baudRate - 1;
    }
    if(UARTx == UART1)
    {
        /*时钟源选择*/
        FL_CMU_SetUART1ClockSource(initStruct->clockSrc);
        /* 根据不同的时钟源计算baudrate 寄存器值,并配置 */
        switch(initStruct->clockSrc)
        {
            case FL_CMU_UART1_CLK_SOURCE_APBCLK:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
            case FL_CMU_UART1_CLK_SOURCE_RCHF:
                Fclk = FL_CMU_GetRCHFClockFreq();
                break;
            case FL_CMU_UART1_CLK_SOURCE_XTHF:
                Fclk = XTHFClock;
                break;
            default:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
        }
        BaudRate = Fclk / initStruct->baudRate - 1;
    }
    if(UARTx == UART2)
    {
        /*时钟源选择*/
        FL_CMU_SetUART2ClockSource(initStruct->clockSrc);
        /* 根据不同的时钟源计算baudrate 寄存器值,并配置 */
        switch(initStruct->clockSrc)
        {
            case FL_CMU_UART2_CLK_SOURCE_APBCLK:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
            case FL_CMU_UART2_CLK_SOURCE_RCHF:
                Fclk = FL_CMU_GetRCHFClockFreq();
                break;
            case FL_CMU_UART2_CLK_SOURCE_XTHF:
                Fclk = XTHFClock;
                break;
            default:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
        }
        BaudRate = Fclk / initStruct->baudRate - 1;
    }
    if(UARTx == UART3)
    {
        /*时钟源选择*/
        FL_CMU_SetUART3ClockSource(initStruct->clockSrc);
        /* 根据不同的时钟源计算baudrate 寄存器值,并配置 */
        switch(initStruct->clockSrc)
        {
            case FL_CMU_UART3_CLK_SOURCE_APBCLK:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
            case FL_CMU_UART3_CLK_SOURCE_RCHF:
                Fclk = FL_CMU_GetRCHFClockFreq();
                break;
            case FL_CMU_UART3_CLK_SOURCE_XTHF:
                Fclk = XTHFClock;
                break;
            default:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
        }
        BaudRate = Fclk / initStruct->baudRate - 1;
    }
    if(UARTx == UART4)
    {
        /*时钟源选择*/
        FL_CMU_SetUART4ClockSource(initStruct->clockSrc);
        /* 根据不同的时钟源计算baudrate 寄存器值,并配置 */
        switch(initStruct->clockSrc)
        {
            case FL_CMU_UART4_CLK_SOURCE_APBCLK:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
            case FL_CMU_UART4_CLK_SOURCE_RCHF:
                Fclk = FL_CMU_GetRCHFClockFreq();
                break;
            case FL_CMU_UART4_CLK_SOURCE_XTHF:
                Fclk = XTHFClock;
                break;
            default:
                Fclk = FL_CMU_GetAPB1ClockFreq();
                break;
        }
        BaudRate = Fclk / initStruct->baudRate - 1;
    }    
    if(UARTx == UART0)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART0);
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART0);
    }
    else if(UARTx == UART1)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART1);
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART1);
    }
    else if(UARTx == UART2)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART2);
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART2);
    }
    else if(UARTx == UART3)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART3);
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART3);
    }
    else if(UARTx == UART4)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART4);
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_UART4);
    }
    else if(UARTx == UART5)
    {
        FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UART5);
        Fclk = FL_CMU_GetAPB1ClockFreq();
        BaudRate = Fclk / initStruct->baudRate - 1;
    }

    /*发送接收控制*/
    if(initStruct->transferDirection & FL_UART_DIRECTION_TX)
    {
        FL_UART_EnableTX(UARTx);
    }
    if(initStruct->transferDirection & FL_UART_DIRECTION_RX)
    {
        FL_UART_EnableRX(UARTx);
    }
    /*配置波特率*/
    FL_UART_WriteBaudRate(UARTx, BaudRate);
    /*配置停止位长度*/
    FL_UART_SetStopBitsWidth(UARTx, initStruct->stopBits);
    /*数据长度*/
    FL_UART_SetDataWidth(UARTx, initStruct->dataWidth);
    /*配置奇偶校验*/
    FL_UART_SetParity(UARTx, initStruct->parity);
    status = FL_PASS;
    return status;
}
/**
  * @brief  根据需要配置红外调制寄存器
  *
  * @param  UARTx  外设入口地址
  *
  * @param  initStruct指向FL_UART_InitTypeDef类型的结构体,包含UART外设信息
  *
  * @retval ErrorStatus枚举值
  *         -FL_FAIL 配置过程出现错误
  *         -FL_PASS UART配置成功
  */
FL_ErrorStatus FL_UART_InfraRed_Init(UART_Type *UARTx, FL_UART_InfraRed_InitTypeDef *initStruct)
{
    FL_ErrorStatus status = FL_FAIL;
    uint32_t  tempTZBRG = 0, tempTH = 0;
    /* 参数合法性检查 */
    assert_param(IS_UART_INSTANCE(UARTx));
    assert_param(IS_FL_UART_INFRARED_POLARITY(initStruct->polarity));
    assert_param(IS_FL_UART_INFRARED_MODULATION_DUTY(initStruct->modulationDuty));
    /*红外发送总线时钟使能*/
    FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_UARTIR);
    /*红外发送使能*/
    FL_UART_EnableIRModulation(UARTx);
    /*红外调制极性*/
    FL_UART_SetIRPolarity(UART, initStruct->polarity);
    /*红外调制频率*/
    tempTZBRG = (uint32_t)((FL_CMU_GetAPB1ClockFreq() * 1.0) / initStruct->modulationFrequency - 1);
    /* 调制占空比 */
    if((tempTZBRG >> 4) != 0)
    {
        tempTH = (uint32_t)(((float)initStruct->modulationDuty / 100.0f) * ((float)(tempTZBRG + 1) / (float)(tempTZBRG >> 4)) + 0.5f);
    }
    else
    {
        tempTH = (uint32_t)(((float)initStruct->modulationDuty / 100.0f) * (float)(tempTZBRG + 1) + 0.5f);
    }
    /* 占空比限位到小于95%，否则结果会有问题 */
    tempTH = ((float)((tempTZBRG >> 4) * tempTH) / (float)(tempTZBRG + 1)) < 0.95f ? tempTH : tempTH - 1;
    /* 占空比和调制频率配置 */
    FL_UART_WriteIRModulationDuty(UART, tempTH);
    FL_UART_WriteIRModulationFrequency(UART, tempTZBRG);
    status = FL_PASS;
    return status;
}

/**
  * @brief  UART_InitStruct 为默认配置
  * @param  UART_InitStruct 指向需要将值设置为默认配置 的结构体@ref FL_UART_InitTypeDef structure 结构体
  *
  * @retval None
  */
void FL_UART_InfraRed_StructInit(FL_UART_InfraRed_InitTypeDef *initStruct)
{
    initStruct->polarity                  = FL_UART_INFRARED_POLARITY_NORMAL;
    initStruct->modulationDuty            = 50;
    initStruct->modulationFrequency       = 38000;
}

/**
  * @brief  UART_InitStruct 为默认配置
  * @param  UART_InitStruct 指向需要将值设置为默认配置 的结构体@ref FL_UART_InitTypeDef structure 结构体
  *         结构体
  * @retval None
  */
void FL_UART_StructInit(FL_UART_InitTypeDef *initStruct)
{
    initStruct->baudRate            = 115200;
    initStruct->dataWidth           = FL_UART_DATA_WIDTH_8B;
    initStruct->stopBits            = FL_UART_STOP_BIT_WIDTH_1B;
    initStruct->parity              = FL_UART_PARITY_EVEN ;
    initStruct->transferDirection   = FL_UART_DIRECTION_TX_RX;
    initStruct->clockSrc            = 0;
}

/**
  * @}
  */

#endif /* FL_UART_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
