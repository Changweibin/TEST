#ifndef __USER_PROCESS_H__
#define __USER_PROCESS_H__

#include "fm33ld5xx_fl.h"

typedef struct
{
    uint8_t    IntClock_1ms;    /* 1ms时钟标志 */
    uint8_t    Tim1ms_flag;     /* 1ms标志 */
    uint8_t    Tim1ms_count;    /* 1ms计数 */
    uint8_t    Tim10ms_flag;    /* 10ms标志 */
    uint8_t    Tim10ms_count;   /* 10ms计数 */
    uint8_t    Tim100ms_flag;   /* 100ms标志 */
    uint8_t    Tim100ms_count;  /* 100ms计数 */
    uint8_t    Tim500ms_flag;   /* 500ms标志 */
    uint8_t    Tim500ms_count;  /* 500ms计数 */
    uint8_t    Tim1s_flag;      /* 1s标志 */
    uint8_t    Tim1s_count;     /* 1s计数 */
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
