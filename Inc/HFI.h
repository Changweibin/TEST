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
    float Theta;                    /* HFI观测的角度 */
    float Theta_Err;                /* HFI观测的角度误差 */
    float Theta_Init;               /* 初始角度值 */
    float Ialpha_Last;              /* 上一时刻alpha轴电流 */
    float Ialpha_LLast;             /* k-2时刻alpha轴电流值 */
    float Ibeta_Last;               /* 上一时刻beta轴电流 */
    float Ibeta_LLast;              /* k-2时刻beta轴电流值 */
    float Ialpha_H;                 /* alpha轴高频电流分量 */
    float Ibeta_H;                  /* beta轴高频电流分量 */
    float Ialpha_H_Last;            /* 上一时刻alpha轴高频电流分量 */
    float Ibeta_H_Last;             /* 上一时刻beta轴高频电流分量 */
    float Ialpha_F;                 /* alpha轴低频电流分量 */
    float Ibeta_F;                  /* beta轴低频电流分量 */
    float Ialpha_H_A;               /* 经过包络检测后的alpha轴高频电流分量 */
    float Ibeta_H_A;                /* 经过包络检测后的beta轴高频电流分量 */
    float Id_F;                     /* d轴基波电流分量 */
    float Iq_F;                     /* q轴基波电流分量 */
    float Id_S;                     /* d轴采样电流值 */
    float Iq_S;                     /* q轴电流采样值 */
    float Id_S_Last;                /* 上一时刻d轴电流采样值 */
    float Id_S_LLast;               /* k-2时刻d轴电流采样值 */
    float Iq_S_Last;                /* 上一时刻q轴电流采样值 */
    float Iq_S_LLast;               /* k-2时刻q轴电流采样值 */
    float Id_H;                     /* d轴高频电流响应 */
    float Uinj;                     /* d轴注入高频电压 */
    s8    SignVal;                  /* 符号函数后的结果 */
}HFI_Para, *p_HFI;
#define HFI_PARA_DEFAULTS        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    
extern HFI_Para    HFI;
    
extern void HFI_Init(void);
extern void HFI_Angle_Cale(p_HFI pv);
extern void HFI_Init_Theta(void);
extern void HFI_Start_Ctr(void);

#endif    /* _HFI_H_ */