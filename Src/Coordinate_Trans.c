#include "Coordinate_Trans.h"


/* 变量定义与初始化 */
CLARK CLARKE_ICurr        = CLARK_DEFAULTS;
PARK  PARK_PCurr          = PARK_DEFAULTS;
IPARK IPARK_PVdq          = IPARK_DEFAULTS;

void CLARKE_Cale(p_CLARK pv)
{
    pv->Alpha = pv->Iu;
    pv->Beta  = (pv->Iu + 2 * pv->Iv) / 1.73205080757f;
}

void PARK_Cale(p_PARK pv)
{
    pv->Ds = arm_cos_f32(pv->Theta) * pv->Alpha + arm_sin_f32(pv->Theta) * pv->Beta;
    pv->Qs = arm_cos_f32(pv->Theta) * pv->Beta - arm_sin_f32(pv->Theta) * pv->Alpha;
}

void IPARK_Cale(p_IPARK pv)
{
    pv->Alpha = arm_cos_f32(pv->Theta) * pv->VDs - arm_sin_f32(pv->Theta) * pv->VQs;
    pv->Beta  = arm_sin_f32(pv->Theta) * pv->VDs + arm_cos_f32(pv->Theta) * pv->VQs;
}