#ifndef __USER_PARAM_H__
#define __USER_PARAM_H__

#include "fm33ld5xx_fl.h"

typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;

typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

/************************* ��ʱ������**************************/
#define PWM_TIM               ATIM0
#define PWM_TIM_IRQn          ATIM0_IRQn
#define PWM_TIM_CLOCK         168000000    /* 168MHz */

#define PWM_TIM_FREQ          10000       /* 10KHz */
#define PWM_TIM_PULSE         (PWM_TIM_CLOCK/(2*PWM_TIM_FREQ))    /* PWM_TIM��ʱ����װ��ֵ */
#define PWM_TIM_PULSE_Half    (PWM_TIM_PULSE / 2)
#define PWM_TIM_PULSE_TPWM    (PWM_TIM_CLOCK/(PWM_TIM_FREQ))

#define FOC_PERIOD            0.0001f
#define FOC_FREQ              PWM_TIM_FREQ

#define DEAD_TIME             ((uint16_t) 1000)
#define PWM_DEAD_TIME         (uint16_t)((unsigned long long)PWM_TIM_CLOCK/2*(unsigned long long)DEAD_TIME/1000000000uL)

#define REP_RATE (1)
#define SAMPLE_FREQ           PWM_TIM_FREQ

#define SPEEDLOOPFREQ         1000       /* �ٶȻ�Ƶ�� ��λ[Hz] */
#define IRP_PERIOD            (PWM_TIM_FREQ/SPEEDLOOPFREQ)

/************************* ���������� **************************/
#define POLEPAIRS             2
#define NOMINALSPEED          3000        /* RPM */
#define MotorRs               0.5f        /* �������� */
#define MotorLs               0.00112f    /* ������� */
#define Motor_Freq_Min        2.0f        /* �����СƵ�� */
#define Motor_Freq_Max        120.0f      /* ������Ƶ�� */

#define Protect_Low_Voltage_V     20.0f      /* Ƿѹ������ֵ */
#define Standard_Work_Voltage_V   24.0f     /* ��׼������ѹ */
#define Protect_High_Voltage_V    26.0f     /* ��ѹ������ֵ */

#define Proctect_High_PhaseCurr_A 3.0f      /* ��������ֵ���� */

#define Proctect_Lost_Min_Speed   50.0f     /* ��ת�ж����ת�� */
#define Proctect_Lost_Max_Speed   4500.0f   /* ��ת�ж����ת�� */

#define cMaxSpeed             3000        /* ������ת�� */
#define cMinSpeed             500

#define CW                    1           /* ��ת */
#define CCW                   2           /* ��ת */

/************************* Ӳ����·���� **************************/
#define ADC_REF_V                   (float)(5.0f)
#define VBUS_UP_RES                 (float)(30.0f) 
#define VBUS_DOWN_RES               (float)(2.0f) 
#define VBUS_CONVERSION_FACTOR      (float)(ADC_REF_V*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/4095.0f)

#define SAMPLE_RES                  (float)(0.02f)
#define AMP_GAIN                    (float)(10.0f)
#define SAMPLE_CURR_CON_FACTOR      (float)(ADC_REF_V/4095.0f/AMP_GAIN/SAMPLE_RES)

/************************* �������״̬ **************************/
#define M_STATE_IDLE                 0
#define M_STATE_INIT                 1
#define M_STATE_CHARGE               2
#define M_STATE_ALIGN                3
#define M_STATE_IFSTART              4
#define M_STATE_INITTHETA            7
#define M_STATE_HFI                  8
#define M_STATE_SPDCLS               5
#define M_STATE_FAULT                6

/*---------------------- PI���������� ------------------------*/
#define BWc_rps             31.4           /* 2*PI*fc/(10 - 20) */
#define TS                  (float)FOC_PERIOD

#define Sqrt_OF_3           0.57735f      /* 1/sqrt(3) */
#define Svpwm_Km            (Standard_Work_Voltage_V * Sqrt_OF_3)
#define Svpwm_Km_Backw      0.1443376f    /* 1/Svpwm_Km */
#define SVGEN_SQRT3_OVER_2  ((float)(0.8660254038f))    /* sqrt(3)/2 */
#define MaxVsMagPu          0.55f         /* Standard_Work_Voltage_V * Sqrt_OF_3 * 0.95 */

#define SPEED_PI_P          0.005f
#define SPEED_PI_I          0.01f
#define PI_MAX_Spd          5.0f          /* �������� */

#define Curr_PI_P           (float)(MotorLs * BWc_rps)
#define Curr_PI_I           (float)(MotorRs / MotorLs * TS)
#define PI_MAX_Ud           (Standard_Work_Voltage_V*MaxVsMagPu)
#define PI_MAX_Uq           (Standard_Work_Voltage_V*MaxVsMagPu)

#define PIX2                6.28318530718f/* 2PI */

/*---------------------- ������ƻ��ڲ������� ------------------------*/
/* Ԥ���ʱ����������� */
#define CHARGE_TIME         (u16)50                 /* ��λ [ms] */
#define CHARGE_TIME_CNT     (u32)(CHARGE_TIME * SAMPLE_FREQ / 1000)

/* ��Ƶע����� */
#define HFI_Id_Offset            (0.5f)    /* �ż����Ա�ʶ ע���Id����ƫ�� */
#define HFI_Uinj_Offset          (2.4f)    /* HFIע���Ƶ��ѹ��ֵ */

#define Speed_Ref_OD           0.1f    /* ����Ŀ���ٶȸı���ݶ�ֵ */
#define Speed_Ref_Timer        30      /* ����Ŀ���ٶȸı���ݶ�ʱ�� */
#define HFI_Speed_Ref_Max      375     /* Ŀ���ٶ�����ֵ����е���ٶ� */

#endif
