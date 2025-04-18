#ifndef __MOTORCTR_H__
#define __MOTORCTR_H__

#include "fm33ld5xx_fl.h"
#include "User_Param.h"

extern uint8_t  motor_start_stop;

extern uint32_t chargeTimeCnt;
extern uint32_t chargeTimeLen;

extern void Motor_Param_Init(void);
extern void motorCtr(void);
extern void Charge_Init(void);
extern void motorCtrStateFlow(void);

extern float Sat(float s, float delta);
extern float my_max(float a, float b);
extern float my_abs(float x);
extern s8 Sign(float x);
extern void FOC_SVPWM_dq(void);

#endif
