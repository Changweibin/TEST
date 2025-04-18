/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mf_config.c
  * @brief          : MCU FUNCTION CONFIG
  ******************************************************************************
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mf_config.h"
#include "User_Param.h"

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
void MF_Clock_Init(void)
{

}

void MF_Config_Init(void)
{

}

void SysTim_Init(void)
{
    FL_BSTIM16_InitTypeDef    TimerBaseInitStruct;
    FL_NVIC_ConfigTypeDef     InterruptConfigStruct;

    TimerBaseInitStruct.prescaler = 168 - 1;
    TimerBaseInitStruct.autoReload = 1000-1;
    TimerBaseInitStruct.autoReloadState = FL_DISABLE;
    TimerBaseInitStruct.clockSource = FL_CMU_BSTIM16_CLK_SOURCE_APBCLK;

    FL_BSTIM16_Init(BSTIM16, &TimerBaseInitStruct);
    
    FL_BSTIM16_ClearFlag_Update(BSTIM16);
    FL_BSTIM16_EnableIT_Update(BSTIM16);
    

    InterruptConfigStruct.preemptPriority = 0x02;
    FL_NVIC_Init(&InterruptConfigStruct, BSTIM16_IRQn); 
    
    FL_BSTIM16_Enable(BSTIM16);
}


void Motor_GPIO_init(void)
{
    FL_GPIO_InitTypeDef         GPIO_InitStruct={0};
    //ATIM IO 配置
    GPIO_InitStruct.pin           = FL_GPIO_PIN_12|FL_GPIO_PIN_13|FL_GPIO_PIN_15;//ATIM 
    GPIO_InitStruct.mode          = FL_GPIO_MODE_DIGITAL;
    GPIO_InitStruct.outputType    = FL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.pull          = FL_GPIO_BOTH_DISABLE;
    GPIO_InitStruct.remapPin      = FL_GPIO_PINREMAP_FUNCTON0;
    GPIO_InitStruct.driveStrength = FL_GPIO_DRIVESTRENGTH_HIGH;
    (void)FL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    GPIO_InitStruct.pin           =  FL_GPIO_PIN_11|FL_GPIO_PIN_12|FL_GPIO_PIN_13;
    GPIO_InitStruct.mode          = FL_GPIO_MODE_DIGITAL;
    GPIO_InitStruct.outputType    = FL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.pull          = FL_GPIO_BOTH_DISABLE;
    GPIO_InitStruct.remapPin      = FL_GPIO_PINREMAP_FUNCTON0;
    GPIO_InitStruct.driveStrength = FL_GPIO_DRIVESTRENGTH_HIGH;
    (void)FL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    
    //ADC通道配置
    GPIO_InitStruct.pin           = FL_GPIO_PIN_0|FL_GPIO_PIN_1|FL_GPIO_PIN_2|FL_GPIO_PIN_9|FL_GPIO_PIN_10;
    GPIO_InitStruct.mode          = FL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.outputType    = FL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.pull          = FL_GPIO_BOTH_DISABLE;
    GPIO_InitStruct.remapPin      = FL_GPIO_PINREMAP_FUNCTON1;
    GPIO_InitStruct.driveStrength = FL_GPIO_DRIVESTRENGTH_HIGH;
    (void)FL_GPIO_Init(GPIOD, &GPIO_InitStruct ); 
}

void ADC_init(void)
{
    FL_ADC_InitTypeDef         defaultInitStruct={0};
    FL_ADC_CommonInitTypeDef    CommonInitStruct = {0};
    FL_NVIC_ConfigTypeDef     InterruptConfigStruct={0};
    
    CommonInitStruct.clockPrescaler                 = FL_ADC_APBCLK_PSC_DIV5;                         
    CommonInitStruct.referenceSource                = FL_ADC_REF_SOURCE_VDDA;                          
    FL_ADC_CommonInit(ADC0,&CommonInitStruct);        
    FL_ADC_CommonInit(ADC3,&CommonInitStruct);
    FL_ADC_CommonInit(ADC2,&CommonInitStruct);
    
    defaultInitStruct.scanacq_continuousMode = FL_DISABLE;                                     		     
    defaultInitStruct.scanacq_autoMode =  FL_ADC_SCANACQ_CONV_MODE_AUTO;                                
    defaultInitStruct.scanacq_Direction = FL_ADC_SCANACQ_SCAN_DIR_FORWARD;							   
    defaultInitStruct.triggerSource = FL_ADC_ACQ_TRGI_ATIM0_TRGO;                                                    
    defaultInitStruct.waitMode = FL_DISABLE;                                                         
    defaultInitStruct.overrunMode = FL_ENABLE;                                                      
    defaultInitStruct.oversamplingMode = FL_DISABLE;                                                
    defaultInitStruct.oversampingMultiplier = FL_ADC_OVERSAMPLING_MUL_16X;                        
    defaultInitStruct.oversamplingShift =   FL_ADC_OVERSAMPLING_SHIFT_4B;                            
    FL_ADC_Init(ADC0, &defaultInitStruct, FL_ADC_SCAN_ACQ);

    FL_ADC_EnableSCANACQ_SequencerChannel(ADC0, FL_ADC_SCANACQ_EXTERNAL_CH1);   //电压
    
//    FL_ADC_SetInjectionMode(ADC0, FL_ADC_INJ_MODE_IMMEDIA);
//    FL_ADC_SetACQ0ChannelSelect(ADC0, FL_ADC_ACQ_SELECT_CHANNEL_EXTERNAL_CH1);  //V相电流
//    FL_ADC_SetACQ0SamplingTime(ADC0, FL_ADC_ACQ_SAMPLING_TIME_8_ADCCLK);
//    FL_ADC_SetACQ0TriggerSource(ADC0, FL_ADC_ACQ_TRGI_ATIM0_TRGO);
//    FL_ADC_EnableACQ(ADC0, FL_ADC_ACQ0);
    
    FL_ADC_SetInjectionMode(ADC3, FL_ADC_INJ_MODE_IMMEDIA);
    FL_ADC_SetACQ0ChannelSelect(ADC3, FL_ADC_ACQ_SELECT_CHANNEL_EXTERNAL_CH1);  //V相电流
    FL_ADC_SetACQ0SamplingTime(ADC3, FL_ADC_ACQ_SAMPLING_TIME_8_ADCCLK);
    FL_ADC_SetACQ0TriggerSource(ADC3, FL_ADC_ACQ_TRGI_ATIM0_TRGO2);
    FL_ADC_EnableACQ(ADC3, FL_ADC_ACQ0);
    
    FL_ADC_SetInjectionMode(ADC2, FL_ADC_INJ_MODE_IMMEDIA);
    FL_ADC_SetACQ0ChannelSelect(ADC2, FL_ADC_ACQ_SELECT_CHANNEL_EXTERNAL_CH0);  //U相电流
    FL_ADC_SetACQ0SamplingTime(ADC2, FL_ADC_ACQ_SAMPLING_TIME_8_ADCCLK);
    FL_ADC_SetACQ0TriggerSource(ADC2, FL_ADC_ACQ_TRGI_ATIM0_TRGO2);
    FL_ADC_EnableACQ(ADC2, FL_ADC_ACQ0);
    


    FL_ADC_ClearFlag_ACQEndOfConversion(ADC2,FL_ADC_ACQ0);
    FL_ADC_EnableIT_ACQEndOfConversion(ADC2, FL_ADC_ACQ0);
    InterruptConfigStruct.preemptPriority = 0x01;
    FL_NVIC_Init(&InterruptConfigStruct, ADC2_IRQn); 
  
    
    FL_ADC_EnableACQ(ADC0, FL_ADC_SCAN_ACQ);
  
    FL_ADC_Enable(ADC0); 
    FL_ADC_Enable(ADC3);
    FL_ADC_Enable(ADC2);
}


void TIM_init(void)
{
     FL_ATIM_InitTypeDef        InitStructer1 = {0};    
     FL_ATIM_OC_InitTypeDef     InitStructer2 = {0};
     FL_ATIM_BDTR_InitTypeDef   InitStructer3 = {0};	
     FL_NVIC_ConfigTypeDef     InterruptConfigStruct;
     
    //TIM1时基单元配置16KHZ
    InitStructer1.prescaler             = 1 - 1;                                       //1分频
    InitStructer1.counterMode           = FL_ATIM_COUNTER_ALIGNED_CENTER_DOWN;           //中心对齐计数
    InitStructer1.autoReload            = PWM_TIM_PULSE-1;                                 //PWM16K
    InitStructer1.clockDivision         = FL_ATIM_CLK_DIVISION_DIV2;                    //刹车和数字滤波
    InitStructer1.repetitionCounter     = REP_RATE;                                    //重复计数
    InitStructer1.autoReloadState       = FL_ENABLE;                                  //预装载使能                        
    FL_ATIM_Init(PWM_TIM,&InitStructer1);
    
    InitStructer2.OCMode       = FL_ATIM_OC_MODE_PWM2;                                 //PWM2模式
    InitStructer2.OCETRFStatus = FL_DISABLE;                                         
    InitStructer2.OCFastMode   = FL_DISABLE;                                         // 快速模式  
    InitStructer2.compareValue = 0;                                                 //比较值，初始值为0，防止初始化时启动电机
    InitStructer2.OCPolarity   = FL_ATIM_OC_POLARITY_NORMAL;                       //OC 高电平
    InitStructer2.OCPreload    = FL_ENABLE;                                       //OC preload 使能
    InitStructer2.OCIdleState  = FL_ATIM_OC_IDLE_STATE_LOW;                      //OC IDLE 高
    InitStructer2.OCNIdleState = FL_ATIM_OCN_IDLE_STATE_LOW;                    //OCN IDLE高
    InitStructer2.OCNPolarity  = FL_ATIM_OCN_POLARITY_NORMAL;                  //OCN 高电平
    InitStructer2.OCNState     = FL_ENABLE;                                   //互补通道使能
    InitStructer2.OCState      = FL_ENABLE;
    FL_ATIM_OC_Init(PWM_TIM,FL_ATIM_CHANNEL_1,&InitStructer2);
    FL_ATIM_OC_Init(PWM_TIM,FL_ATIM_CHANNEL_2,&InitStructer2);
    FL_ATIM_OC_Init(PWM_TIM,FL_ATIM_CHANNEL_3,&InitStructer2);
      

    InitStructer2.OCMode       = FL_ATIM_OC_MODE_PWM1;                                 //PWM2模式
    InitStructer2.OCETRFStatus = FL_DISABLE;                                         
    InitStructer2.OCFastMode   = FL_DISABLE;                                         // 快速模式  
    InitStructer2.compareValue = 5;                                                 //比较值，初始值为0，防止初始化时启动电机
    InitStructer2.OCPolarity   = FL_ATIM_OC_POLARITY_NORMAL;                       //OC 高电平
    InitStructer2.OCPreload    = FL_ENABLE;                                       //OC preload 使能
    InitStructer2.OCIdleState  = FL_ATIM_OC_IDLE_STATE_LOW;                      //OC IDLE 高
    InitStructer2.OCNIdleState = FL_ATIM_OCN_IDLE_STATE_LOW;                    //OCN IDLE高
    InitStructer2.OCNPolarity  = FL_ATIM_OCN_POLARITY_NORMAL;                  //OCN 高电平
    InitStructer2.OCNState     = FL_DISABLE;                                   //互补通道使能
    InitStructer2.OCState      = FL_DISABLE;
    FL_ATIM_OC_Init(PWM_TIM,FL_ATIM_CHANNEL_4,&InitStructer2);
   


    InitStructer2.OCMode       = FL_ATIM_OC_MODE_PWM1;                                 //PWM2模式
    InitStructer2.OCETRFStatus = FL_DISABLE;                                         
    InitStructer2.OCFastMode   = FL_DISABLE;                                         // 快速模式  
    InitStructer2.compareValue =PWM_TIM_PULSE- 5;                                                 //比较值，初始值为0，防止初始化时启动电机
    InitStructer2.OCPolarity   = FL_ATIM_OC_POLARITY_NORMAL;                       //OC 高电平
    InitStructer2.OCPreload    = FL_ENABLE;                                       //OC preload 使能
    InitStructer2.OCIdleState  = FL_ATIM_OC_IDLE_STATE_LOW;                      //OC IDLE 高
    InitStructer2.OCNIdleState = FL_ATIM_OCN_IDLE_STATE_HIGH;                    //OCN IDLE高
    InitStructer2.OCNPolarity  = FL_ATIM_OCN_POLARITY_INVERT;                  //OCN 高电平
    InitStructer2.OCNState     = FL_DISABLE;                                   //互补通道使能
    InitStructer2.OCState      = FL_DISABLE;
    FL_ATIM_OC_Init(PWM_TIM,FL_ATIM_CHANNEL_6,&InitStructer2);
   
    InitStructer3.deadTime            = DEAD_TIME;                                       //死区时间
    InitStructer3.lockLevel           = FL_ATIM_LOCK_LEVEL_OFF;
    InitStructer3.OSSRState           = FL_ATIM_OSSR_ENABLE;                          //OSSR=0
    InitStructer3.OSSIState           = FL_ATIM_OSSI_ENABLE;                         //OSSI=0
    InitStructer3.breakFilter         = FL_ATIM_BREAK_FILTER_DIV2_N6;                  //
    InitStructer3.breakPolarity       = FL_ATIM_BREAK_POLARITY_HIGH;               //刹车极性
    InitStructer3.automaticOutput     = FL_DISABLE;                               //MOE由软件控制
    InitStructer3.breakState          = FL_ENABLE;                               //刹车使能
    InitStructer3.breakSource         = FL_ATIM_BKIN_SOURCE_COMP2;
    FL_ATIM_BDTR_Init(PWM_TIM,FL_ATIM_BREAKCHANNEL_1,&InitStructer3);
    PWM_TIM->AFR|=0x1<<10;
    FL_ATIM_ClearFlag_Break1(PWM_TIM);
    FL_ATIM_EnableIT_Break (PWM_TIM);


    InterruptConfigStruct.preemptPriority = 0x02;
    FL_NVIC_Init(&InterruptConfigStruct, PWM_TIM_IRQn);

    FL_ATIM_SetTriggerOutput(PWM_TIM,FL_ATIM_TRGO_OCREF4_FALLING);
    FL_ATIM_SetTriggerOutput2(PWM_TIM,FL_ATIM_TRGO2_UPDATE);
    
    FL_ATIM_Enable(PWM_TIM); 
}








/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN Assert_Failed */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END Assert_Failed */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT FMSH *****END OF FILE****/
