#ifndef _COORDINATE_TRANS_H_
#define _COORDINATE_TRANS_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "fm33ld5xx.h"
#include "User_Param.h"
#include "arm_math.h"

typedef struct
{
    float Iu;
    float Iv;
    float Iw;
    float Alpha;
    float Beta;
}CLARK, *p_CLARK;

typedef struct
{
    float Alpha;    /* ���ྲֹ����ϵAlpha�� */
    float Beta;     /* ���ྲֹ����ϵBeta�� */
    float Theta;    /* �����Ƕ� */
    float Ds;       /* ��ת����ϵ��d����� */
    float Qs;       /* ��ת����ϵ��q����� */
}PARK, *p_PARK;

typedef struct
{
    float Alpha;    /* ���ྲֹ����ϵAlpha�� */
    float Beta;     /* ���ྲֹ����ϵBeta�� */
    float Theta;    /* �����Ƕ� */
    float VDs;       /* ��ת����ϵ��d���ѹ */
    float VQs;       /* ��ת����ϵ��q���ѹ */
}IPARK, *p_IPARK;

#define CLARK_DEFAULTS        {0,0,0,0,0}
#define PARK_DEFAULTS         {0,0,0,0,0}
#define IPARK_DEFAULTS        {0,0,0,0,0}

extern CLARK    CLARKE_ICurr;
extern PARK     PARK_PCurr;
extern IPARK    IPARK_PVdq;

extern void CLARKE_Cale(p_CLARK pv);
extern void PARK_Cale(p_PARK pv);
extern void IPARK_Cale(p_IPARK pv);

#endif    /* _COORDINATE_TRANS_H_ */