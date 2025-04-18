#ifndef _SVPWM_H_
#define _SVPWM_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "User_Param.h"


#define MODIFY_TX_TY(x, y, z, s)    {z=x+y; if(x+y>s){x=x/z*s;y=y/z*s;}}

typedef struct
{
    float Ualpha;          /* alpha轴电压 */
    float Ubeta;           /* beta轴电压 */
    float Ta;              /* 三相矢量占空比Ta */
    float Tb;              /* 三相矢量占空比Tb */
    float Tc;              /* 三相矢量占空比Tc */
    float u1;              /* 三相静止坐标系的电压temp1 */
    float u2;              /* 三相静止坐标系的电压temp2 */
    float u3;              /* 三相静止坐标系的电压temp3 */
    float T0;              /* 零矢量作用时间 */
    u8    VecSector;       /* 扇区 */
}SVPWM, *p_SVPWM;
#define SVPWM_DEFAULTS        {0,0,0,0,0,0,0,0,0,0}
extern SVPWM        SVPWM_dq;

extern uint16_t  hPWMTime;

extern void SVPWM_Cale(p_SVPWM pv);

#endif    /* _SVPWM_H_ */