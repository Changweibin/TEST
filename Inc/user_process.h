#ifndef __USER_PROCESS_H__
#define __USER_PROCESS_H__

#include "fm33ld5xx_fl.h"

typedef struct
{
    uint8_t    IntClock_1ms;    /* 1msʱ�ӱ�־ */
    uint8_t    Tim1ms_flag;     /* 1ms��־ */
    uint8_t    Tim1ms_count;    /* 1ms���� */
    uint8_t    Tim10ms_flag;    /* 10ms��־ */
    uint8_t    Tim10ms_count;   /* 10ms���� */
    uint8_t    Tim100ms_flag;   /* 100ms��־ */
    uint8_t    Tim100ms_count;  /* 100ms���� */
    uint8_t    Tim500ms_flag;   /* 500ms��־ */
    uint8_t    Tim500ms_count;  /* 500ms���� */
    uint8_t    Tim1s_flag;      /* 1s��־ */
    uint8_t    Tim1s_count;     /* 1s���� */
}TaskTime;
#define TASKTIME_DEFAULTS            {0,0,0,0,0,0,0,0,0,0,0}
extern TaskTime TaskTimePare;


extern void TIM_Cmd(ATIM_Type *TIMx, uint8_t NewState);
extern void TIM_CtrlPWMOutputs(ATIM_Type *TIMx, uint8_t NewState);
extern void TIM_Charge(void);
extern void ADC_start_regular_conv(void);
extern void Get_ADC_Vbus(void);
extern void Get_ADC_Curr(void);

#endif
