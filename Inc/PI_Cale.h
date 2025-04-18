#ifndef _PI_CALE_H_
#define _PI_CALE_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "fm33ld5xx.h"
#include "User_Param.h"

typedef struct
{
    float Ref;        /* PI参考值 */
    float Fbk;        /* PI反馈值 */
    float Out;        /* PI输出值 */
    float OutF;       /* PI滤波后输出值 */
    float Kp;         /*PI比例系数 */
    float Ki;         /*PI积分系数 */
    float Umax;       /* PI输出限制最大幅值 */
    float Umin;       /* PI输出限制最小幅值 */
    float err;        /* PI误差 */
    float err_1;      /* PI上一环节误差 */
    float Up;         /* PI比例项计算值 */
    float Ui;         /* PI积分项计算值 */
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