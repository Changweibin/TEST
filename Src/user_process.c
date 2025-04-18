#include "user_process.h"
#include "User_Param.h"
#include "FOC.h"

/* 变量定义与初始化 */
TaskTime TaskTimePare     = TASKTIME_DEFAULTS;

void BSTIM16_IRQHandler(void)
{
    if((BSTIM16->ISR && 0x01) == 0x01)
    {
        /* 清除中断标志 */
        BSTIM16->ISR = 0x01;
        
        /* 1ms时钟标志置位 */
        TaskTimePare.IntClock_1ms = 1;
        
    }
}

void TIM_Cmd(ATIM_Type *TIMx, uint8_t NewState)
{
 
  if (NewState != FL_DISABLE)
  {
    
    TIMx->CR1 |= 0x1<<0;
  }
  else
  {
    
    TIMx->CR1 &= ~(0x1<<0);
  }
}

void TIM_CtrlPWMOutputs(ATIM_Type *TIMx, uint8_t NewState)
{
    if (NewState != FL_DISABLE) {
        TIMx->BDTR |= 0x1 << 15;
    } else {
        TIMx->BDTR &= ~(0x1 << 15);
    }
}

void TIM_Charge(void)
{
    PWM_TIM->CCR1 = (PWM_TIM_PULSE >> 1);
    PWM_TIM->CCR2 = (PWM_TIM_PULSE >> 1);
    PWM_TIM->CCR3 = (PWM_TIM_PULSE >> 1);

    TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
    TIM_Cmd(PWM_TIM, ENABLE);
}

void ADC_start_regular_conv(void)
{
    if (FL_ADC_IsActiveFlag_EndOfSequence(ADC0) == 0x01) {
        MotorADCResult[2] = (ADC0->ACQ4_DR);       
        FL_ADC_ClearFlag_EndOfSequence(ADC0);
    }
}

void Get_ADC_Curr(void)
{
    ADC_Sample_Para.PhaseU_Curr = (float)(((int16_t)ADC_Sample_Para.offset_PhaseU_Curr - MotorADCResult[0]) \
                                            * SAMPLE_CURR_CON_FACTOR);
    ADC_Sample_Para.PhaseV_Curr = (float)(((int16_t)ADC_Sample_Para.offset_PhaseV_Curr - MotorADCResult[1]) \
                                            * SAMPLE_CURR_CON_FACTOR);

    ADC_Sample_Para.PhaseW_Curr = -ADC_Sample_Para.PhaseU_Curr - ADC_Sample_Para.PhaseV_Curr;
}

void Get_ADC_Vbus(void)
{
    ADC_Sample_Para.VBUS = (float)(MotorADCResult[2] * VBUS_CONVERSION_FACTOR);
}
