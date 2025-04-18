#include "FOC.h"
#include "user_process.h"
#include "User_Param.h"
#include "MotorCtr.h"
#include "Coordinate_Trans.h"
#include "HFI.h"

/* 变量定义与初始化 */
motor_p  Motor                      = MOTOR_P_DEFAULTS;
ADC_Sample ADC_Sample_Para          = ADC_Sample_DEFAULTS;
uint8_t  get_offset_flag            = 0;    /* 电流采样电路零漂采样标志 */
uint16_t get_offset_sample_cnt      = 0;
uint16_t   MotorADCResult[3]    = {0};

void get_offset(uint32_t *a_offset, uint32_t *b_offset)
{
    if(get_offset_sample_cnt < 128)
    {
        *a_offset += MotorADCResult[0];
        *b_offset += MotorADCResult[1];
        get_offset_sample_cnt++;
    }
    else
    {
        get_offset_sample_cnt = 0;

        *a_offset >>= 7;
        *b_offset >>= 7;
        TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
        TIM_Cmd(PWM_TIM, DISABLE);

        get_offset_flag = 2;
    }
}

void motor_Run(void)
{
    /* 获取相电流 */
    Get_ADC_Curr();
    
    /* 获取母线电压 */
    Get_ADC_Vbus();
    
    switch(Motor.M_State)
    {
        case M_STATE_CHARGE:
            if (chargeTimeCnt < chargeTimeLen)
                chargeTimeCnt++;
            else
            {
                chargeTimeCnt = 0;
                
                Motor.M_State = M_STATE_INITTHETA;
            }
        break;
            
        case M_STATE_INITTHETA:
            CLARKE_ICurr.Iu = ADC_Sample_Para.PhaseU_Curr;
            CLARKE_ICurr.Iv = ADC_Sample_Para.PhaseV_Curr;
            CLARKE_Cale((p_CLARK)&CLARKE_ICurr);            /* CLARKE变换 */
        
            /* HFI获取角度 */
            HFI_Angle_Cale((p_HFI)&HFI);
        
            /* 获取当前电角度 PARK变换 */
            PARK_PCurr.Theta = PLL_HFI_Para.PLL_Theta;
            PARK_PCurr.Alpha = CLARKE_ICurr.Alpha;
            PARK_PCurr.Beta  = CLARKE_ICurr.Beta;
            PARK_Cale((p_PARK)&PARK_PCurr);
        
            /* 初始角度与极性辨识 */
            HFI_Init_Theta();
        
            Motor.E_theta = PLL_HFI_Para.PLL_Theta;
        
            /* SVPWM */
            FOC_SVPWM_dq();
            
        break;
        
        case M_STATE_HFI:
            CLARKE_ICurr.Iu = ADC_Sample_Para.PhaseU_Curr;
            CLARKE_ICurr.Iv = ADC_Sample_Para.PhaseV_Curr;
            CLARKE_Cale((p_CLARK)&CLARKE_ICurr);            /* CLARKE变换 */
        
            /* HFI获取角度 */
            HFI_Angle_Cale((p_HFI)&HFI);
        
            /* 获取当前电角度 PARK变换 */
            PARK_PCurr.Theta = PLL_HFI_Para.PLL_Theta;
            PARK_PCurr.Alpha = CLARKE_ICurr.Alpha;
            PARK_PCurr.Beta  = CLARKE_ICurr.Beta;
            PARK_Cale((p_PARK)&PARK_PCurr);
        
            /* HFI启动控制 */
            HFI_Start_Ctr();

            Motor.E_theta = PLL_HFI_Para.PLL_Theta;
        
//            /* 获取观测器角度 */
//            SMO_Theta_Cale(CLARKE_ICurr.Alpha, CLARKE_ICurr.Beta, SVPWM_dq.Ualpha, SVPWM_dq.Ubeta);
            
            /* SVPWM */
            FOC_SVPWM_dq();
        break;
    }
}

void ADC2_IRQHandler(void)
{
    if((ADC2->ACQ_ISR & (0x1U <<0)) == (0x1U << 0)) 
    {   

        ADC2->ACQ_ISR = 0x1U <<0;
        ADC3->ACQ_ISR = 0x1U <<0;
        
        MotorADCResult[0] = ADC2->ACQ0_DR;    /* U相采样值 */
        MotorADCResult[1] = ADC3->ACQ0_DR;    /* V相采样值 */

        if(2 == get_offset_flag)
        {
            motor_Run();
        }
        else
        {
            if(1 == get_offset_flag)
            {
                get_offset(&ADC_Sample_Para.offset_PhaseU_Curr, &ADC_Sample_Para.offset_PhaseV_Curr);
            }
        }
     }
 }
