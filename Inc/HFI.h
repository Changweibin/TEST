#ifndef _HFI_H_
#define _HFI_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "fm33ld5xx.h"
#include "User_Param.h"


#define HFI_TIME_10MS_CNT        (u16)(10 * PWM_TIM_FREQ / 1000)
#define HFI_TIME_20MS_CNT        (u16)(20 * PWM_TIM_FREQ / 1000)
#define HFI_TIME_30MS_CNT        (u16)(30 * PWM_TIM_FREQ / 1000)

typedef struct
{
    float Theta;                    /* HFI�۲�ĽǶ� */
    float Theta_Err;                /* HFI�۲�ĽǶ���� */
    float Theta_Init;               /* ��ʼ�Ƕ�ֵ */
    float Ialpha_Last;              /* ��һʱ��alpha����� */
    float Ialpha_LLast;             /* k-2ʱ��alpha�����ֵ */
    float Ibeta_Last;               /* ��һʱ��beta����� */
    float Ibeta_LLast;              /* k-2ʱ��beta�����ֵ */
    float Ialpha_H;                 /* alpha���Ƶ�������� */
    float Ibeta_H;                  /* beta���Ƶ�������� */
    float Ialpha_H_Last;            /* ��һʱ��alpha���Ƶ�������� */
    float Ibeta_H_Last;             /* ��һʱ��beta���Ƶ�������� */
    float Ialpha_F;                 /* alpha���Ƶ�������� */
    float Ibeta_F;                  /* beta���Ƶ�������� */
    float Ialpha_H_A;               /* ������������alpha���Ƶ�������� */
    float Ibeta_H_A;                /* ������������beta���Ƶ�������� */
    float Id_F;                     /* d������������� */
    float Iq_F;                     /* q������������� */
    float Id_S;                     /* d���������ֵ */
    float Iq_S;                     /* q���������ֵ */
    float Id_S_Last;                /* ��һʱ��d���������ֵ */
    float Id_S_LLast;               /* k-2ʱ��d���������ֵ */
    float Iq_S_Last;                /* ��һʱ��q���������ֵ */
    float Iq_S_LLast;               /* k-2ʱ��q���������ֵ */
    float Id_H;                     /* d���Ƶ������Ӧ */
    float Uinj;                     /* d��ע���Ƶ��ѹ */
    s8    SignVal;                  /* ���ź�����Ľ�� */
}HFI_Para, *p_HFI;
#define HFI_PARA_DEFAULTS        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    
extern HFI_Para    HFI;
    
extern void HFI_Init(void);
extern void HFI_Angle_Cale(p_HFI pv);
extern void HFI_Init_Theta(void);
extern void HFI_Start_Ctr(void);

#endif    /* _HFI_H_ */