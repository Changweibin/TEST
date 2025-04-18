#include "PI_Cale.h"


/* 变量定义与初始化 */
PI_Control pi_spd        = PI_CONTROL_DEFAULTS;
PI_Control pi_id         = PI_CONTROL_DEFAULTS;
PI_Control pi_iq         = PI_CONTROL_DEFAULTS;


float Limit_Sat(float fdata, float fmax, float fmin)
{
    fdata = (fdata > fmax) ? fmax : fdata;
    fdata = (fdata < fmin) ? fmin : fdata;

    return fdata;
}

void PI_Pare_Init(void)
{
    pi_spd.Kp    = SPEED_PI_P;
    pi_spd.Ki    = SPEED_PI_I;
    pi_spd.Umax  = PI_MAX_Spd;
    pi_spd.Umin  = -PI_MAX_Spd;

    pi_id.Kp     = Curr_PI_P;
    pi_id.Ki     = Curr_PI_I;;
    pi_id.Umax   = PI_MAX_Ud;
    pi_id.Umin   = -PI_MAX_Ud;

    pi_iq.Kp     = Curr_PI_P;
    pi_iq.Ki     = Curr_PI_I;
    pi_iq.Umax   = PI_MAX_Uq;
    pi_iq.Umin   = -PI_MAX_Uq;
}

void PI_Controller(p_PI_Control pv)
{
    /* proportional term */
    pv->err = pv->Ref - pv->Fbk;
    pv->Up  = pv->Kp * pv->err;
    /* integral term */
    pv->Ui  = Limit_Sat(pv->Ui + (pv->Ki * pv->Up), pv->Umax, pv->Umin);
    pv->Out = Limit_Sat(pv->Up + pv->Ui, pv->Umax, pv->Umin);
}

void PISpd_Controller(p_PI_Control pv)
{
    /* proportional term */
    pv->err = (pv->Ref - pv->Fbk);
    pv->Up  = pv->Kp * pv->err;
    /* integral term */
    pv->Ui  = Limit_Sat(pv->Ui + (pv->Ki * pv->Up), pv->Umax, 0);
    pv->Out = Limit_Sat(pv->Up + pv->Ui, pv->Umax, 0);
}