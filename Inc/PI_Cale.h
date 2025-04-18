#ifndef _PI_CALE_H_
#define _PI_CALE_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "fm33ld5xx.h"
#include "User_Param.h"

typedef struct
{
    float Ref;        /* PI�ο�ֵ */
    float Fbk;        /* PI����ֵ */
    float Out;        /* PI���ֵ */
    float OutF;       /* PI�˲������ֵ */
    float Kp;         /*PI����ϵ�� */
    float Ki;         /*PI����ϵ�� */
    float Umax;       /* PI�����������ֵ */
    float Umin;       /* PI���������С��ֵ */
    float err;        /* PI��� */
    float err_1;      /* PI��һ������� */
    float Up;         /* PI���������ֵ */
    float Ui;         /* PI���������ֵ */
}PI_Control, *p_PI_Control;
#define PI_CONTROL_DEFAULTS        {0,0,0,0,0,0,0,0,0,0,0,0}
extern PI_Control    pi_spd;
extern PI_Control    pi_id;
extern PI_Control    pi_iq;

extern float Limit_Sat(float fdata, float fmax, float fmin);
extern void PI_Pare_Init(void);
extern void PI_Controller(p_PI_Control pv);
extern void PISpd_Controller(p_PI_Control pv);

#endif    /* _PI_CALE_H_ */