#ifndef _SVPWM_H_
#define _SVPWM_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "User_Param.h"


#define MODIFY_TX_TY(x, y, z, s)    {z=x+y; if(x+y>s){x=x/z*s;y=y/z*s;}}

typedef struct
{
    float Ualpha;          /* alpha���ѹ */
    float Ubeta;           /* beta���ѹ */
    float Ta;              /* ����ʸ��ռ�ձ�Ta */
    float Tb;              /* ����ʸ��ռ�ձ�Tb */
    float Tc;              /* ����ʸ��ռ�ձ�Tc */
    float u1;              /* ���ྲֹ����ϵ�ĵ�ѹtemp1 */
    float u2;              /* ���ྲֹ����ϵ�ĵ�ѹtemp2 */
    float u3;              /* ���ྲֹ����ϵ�ĵ�ѹtemp3 */
    float T0;              /* ��ʸ������ʱ�� */
    u8    VecSector;       /* ���� */
}SVPWM, *p_SVPWM;
#define SVPWM_DEFAULTS        {0,0,0,0,0,0,0,0,0,0}
extern SVPWM        SVPWM_dq;

extern uint16_t  hPWMTime;

extern void SVPWM_Cale(p_SVPWM pv);

#endif    /* _SVPWM_H_ */