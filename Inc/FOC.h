#ifndef __FOC_H__
#define __FOC_H__

#include "fm33ld5xx_fl.h"

typedef struct
{
    float Freq_Max;
    float Freq;
    float Freq_delta;
    float IF_Theta;
    float IF_Curr;
    uint8_t Timer_Count;
    uint16_t Ramp_Timer;
}GRamp, *p_GRamp;
#define GRAMP_DEFAULTS        {0,0,0,0,0,0,0}

typedef struct
{
    uint8_t  M_State;             /* 电机当前运行状态 */
    uint8_t  Ctr_Mode;            /* 电机控制模式 */
    uint8_t  Pn;                  /* 极对数 */
    uint8_t  Dir;                 /* 转向 */
    uint8_t  Init_Over;           /* 初始化完成标志 */
    uint16_t SpdClsCnt;           /* 速度闭环计数 */
    float    M_theta;             /* 转子机械角度 */
    float    E_theta;             /* 转子电角度 */
    float    Theta_Err;           /* 角度误差 */
    float    speed_M_RPM;         /* 电机机械转速 */
    float    speed_M_RPMF;        /* 滤波后电机转速 */
    float    TargetSpd;           /* 电机目标转速 */
    float    MaxSpeedF;           /* 电机最大电频率 */
    float    MinSpeedF;           /* 电机最小电频率 - 速度闭环 */
    float    Bemf_vp;             /* 反电势幅值 */
    float    Rs;                  /* 电机相电阻 */
    float    Ls;                  /* 电机相电感 */
}motor_p;
#define MOTOR_P_DEFAULTS        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
extern motor_p    Motor;

typedef struct 
{
    float VBUS;
    float Temperature;
    float PhaseU_Curr;
    float PhaseV_Curr;
    float PhaseW_Curr;

    uint32_t offset_PhaseU_Curr;
    uint32_t offset_PhaseV_Curr;
}ADC_Sample;
#define ADC_Sample_DEFAULTS    {0,0,0,0,0,0}            /* ADC采样初始化参数 */
extern ADC_Sample    ADC_Sample_Para;

typedef struct
{
    float Kp;                          /* 锁相环PI控制器Kp */
    float Ki;                          /* 锁相环PI控制器Ki */
    float inter;
    float Theta_Err;                   /* 角度误差 */
    float Theta_Err_Last;              /* 上一时刻角度误差 */
    float Omega;                       /* 电角速度 */
    float OmegaF;                      /* 滤波后的电角速度 */
    float PLL_Theta;                   /* 锁相环计算得到的角度 */
    float Angle_Comp;                  /* 经过角度补偿后的电角度 */
    float Omega_Hz;                    /* 电频率 */
    float Theta_Last;                  /* 上一时刻电角度 */
    float Omega2;                      /*  */
    float Omega2F;                     /*  */
}Angle_PLL, *p_Angle_PLL;
#define Angle_PLL_DEFAULTS             {0,0,0,0,0,0,0,0,0,0,0,0}
extern Angle_PLL   PLL_HFI_Para;


extern uint16_t   MotorADCResult[3];
extern uint8_t get_offset_flag;

#endif
