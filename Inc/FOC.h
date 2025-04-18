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
    uint8_t  M_State;             /* �����ǰ����״̬ */
    uint8_t  Ctr_Mode;            /* �������ģʽ */
    uint8_t  Pn;                  /* ������ */
    uint8_t  Dir;                 /* ת�� */
    uint8_t  Init_Over;           /* ��ʼ����ɱ�־ */
    uint16_t SpdClsCnt;           /* �ٶȱջ����� */
    float    M_theta;             /* ת�ӻ�е�Ƕ� */
    float    E_theta;             /* ת�ӵ�Ƕ� */
    float    Theta_Err;           /* �Ƕ���� */
    float    speed_M_RPM;         /* �����еת�� */
    float    speed_M_RPMF;        /* �˲�����ת�� */
    float    TargetSpd;           /* ���Ŀ��ת�� */
    float    MaxSpeedF;           /* �������Ƶ�� */
    float    MinSpeedF;           /* �����С��Ƶ�� - �ٶȱջ� */
    float    Bemf_vp;             /* �����Ʒ�ֵ */
    float    Rs;                  /* �������� */
    float    Ls;                  /* ������� */
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
#define ADC_Sample_DEFAULTS    {0,0,0,0,0,0}            /* ADC������ʼ������ */
extern ADC_Sample    ADC_Sample_Para;

typedef struct
{
    float Kp;                          /* ���໷PI������Kp */
    float Ki;                          /* ���໷PI������Ki */
    float inter;
    float Theta_Err;                   /* �Ƕ���� */
    float Theta_Err_Last;              /* ��һʱ�̽Ƕ���� */
    float Omega;                       /* ����ٶ� */
    float OmegaF;                      /* �˲���ĵ���ٶ� */
    float PLL_Theta;                   /* ���໷����õ��ĽǶ� */
    float Angle_Comp;                  /* �����ǶȲ�����ĵ�Ƕ� */
    float Omega_Hz;                    /* ��Ƶ�� */
    float Theta_Last;                  /* ��һʱ�̵�Ƕ� */
    float Omega2;                      /*  */
    float Omega2F;                     /*  */
}Angle_PLL, *p_Angle_PLL;
#define Angle_PLL_DEFAULTS             {0,0,0,0,0,0,0,0,0,0,0,0}
extern Angle_PLL   PLL_HFI_Para;


extern uint16_t   MotorADCResult[3];
extern uint8_t get_offset_flag;

#endif
