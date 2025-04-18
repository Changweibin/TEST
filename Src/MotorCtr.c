#include "MotorCtr.h"
#include "Filter.h"
#include "FOC.h"
#include "User_Param.h"
#include "user_process.h"
#include "PI_Cale.h"
#include "HFI.h"
#include "SVPWM.h"
#include "Coordinate_Trans.h"

uint8_t  motor_start_stop = 0;

/* 预充时间与计数值初始化 */
uint32_t chargeTimeCnt        = 0;
uint32_t chargeTimeLen        = 0;

void Motor_Param_Init(void)
{
    Motor.Rs = MotorRs;
    Motor.Ls = MotorLs;
    Motor.Pn = POLEPAIRS;

    Motor.Dir = CW;

    Motor.M_State = M_STATE_IDLE;
    Motor.E_theta = 0;
    Motor.M_theta = 0;
    Motor.speed_M_RPM = 0;
    Motor.speed_M_RPMF = 0;
    Motor.MaxSpeedF = cMaxSpeed;
    Motor.MinSpeedF = cMinSpeed;

    /* 巴特沃斯滤波器参数初始化 */
    IIR_ButterWorth_Coefficient_Init(WE_IIR_LPF_Coeff, &WE_IIR_LPF_Para);
}

void motorCtr(void)
{
    if(motor_start_stop == 1)
    {
        TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
        TIM_Cmd(PWM_TIM, ENABLE);
    }
    else
    {
        PWM_TIM->CCR1 = 0;
        PWM_TIM->CCR2 = 0;
        PWM_TIM->CCR3 = 0;
        
        TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
        TIM_Cmd(PWM_TIM, ENABLE);        /* 为了保证电压故障触发后能够正常采样，定时器保持开启状态 */
        
//        /* 变量清零 */
//        Variables_Clr();
    }
}

void motorCtrStateFlow(void)
{
    switch(Motor.M_State)
    {
        case M_STATE_IDLE:
            if(1 == motor_start_stop)
            {
                Motor.M_State = M_STATE_INIT;

                /* HFI参数初始化 */
                HFI_Init();
                /* 控制参数初始化 */
                PI_Pare_Init();
            }
        break;

        case M_STATE_INIT:
            Charge_Init();
            Motor.M_State = M_STATE_CHARGE;
        break;

        case M_STATE_CHARGE:
        case M_STATE_ALIGN:
        case M_STATE_IFSTART:
        case M_STATE_SPDCLS:

        break;

        case M_STATE_FAULT:

        break;

        default:

        break;
    }
}

void Charge_Init(void)
{
    chargeTimeCnt = 0;
    chargeTimeLen = CHARGE_TIME_CNT;
    TIM_Charge();
}

/* 饱和函数 */
float Sat(float s, float delta)
{
    if(s > delta)
        return 1;
    else if(s < -delta)
        return -1;
    else
        return (float)(s / delta);
}

float my_max(float a, float b)
{
    if(a >= b)
        return a;
        else
    return b;
}

float my_abs(float x)
{
    if(x >= 0)
        return x;
    else
        return -x;
}

s8 Sign(float x)
{
    if(x > 0)
        return 1;
    else if(x < 0)
        return -1;
    else
        return 0;
}

void FOC_SVPWM_dq(void)
{
    SVPWM_dq.Ualpha = IPARK_PVdq.Alpha;
    SVPWM_dq.Ubeta  = IPARK_PVdq.Beta;
    SVPWM_Cale((p_SVPWM)&SVPWM_dq);
}


